from ..behaviours.gyro_drive import SpinAround

"""③ ET相撲終了後、ETラリー開始位置の青い丸へ向けて旋回する。"""

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence

from ..runtime import runtime
from ..features.sumo_bearing_motion import SpinToBearing, current_bearing
from ..behaviours.motor_control import StopNow


class TurnTowardRallyStart(Behaviour):
    """緑床回避の有無に応じて旋回角度を切り替える。"""

    def __init__(self, context, config):
        super().__init__("③ 青い丸へ向けて旋回")
        self.context = context
        self.config = config
        self.turn = None

    def initialise(self):
        self.turn = None

    def update(self):
        if self.turn is None:
            avoidance = getattr(
                self.context.sumo,
                "rally_avoidance_performed",
                None,
            )

            angle = 215.0

            # RIGHTは左回り、LEFTは右回り
            if runtime.course == -1:
                signed_angle = -angle
            elif runtime.course == 1:
                signed_angle = angle
            else:
                self.logger.error(
                    "RALLY_TURN: unsupported course=%s" % runtime.course
                )
                return Status.FAILURE

            # 2026-11-17(devSS、ユーザー): 向き終わる方位は同じまま、近い向きに回る(215°ではなく反対回りに145°)。
            # 215°回るのに約2.2秒かかっていた。
            signed_angle = ((signed_angle + 180.0) % 360.0) - 180.0
            start_bearing = current_bearing(self.context)
            target_bearing = (start_bearing + signed_angle) % 360.0

            self.logger.info(
                "RALLY_TURN START course=%s angle=%.1f "
                "start=%.2f target=%.2f"
                % (runtime.course, signed_angle, start_bearing, target_bearing)
            )

            power = 60  # 215°旋回専用
            motion = self.config.sumo

            self.turn = SpinAround(
                "rally start camera direction",
                target=-runtime.course * signed_angle,
                max_power=power,
                min_power=power,
                pid_p=motion.turn_pid_p,
                pid_i=motion.turn_pid_i,
                pid_d=motion.turn_pid_d,
                target_type=HeadingType.RELATIVE,
                tolerance=1.0,
                normalize_error=False,
            )

        self.turn.tick_once()

        if self.turn.status == Status.SUCCESS:
            self.logger.info(
                "RALLY_TURN COMPLETE bearing=%.2f"
                % current_bearing(self.context)
            )
            return Status.SUCCESS

        if self.turn.status == Status.FAILURE:
            return Status.FAILURE

        return Status.RUNNING

    def terminate(self, new_status):
        if self.turn is not None:
            if self.turn.status == Status.RUNNING:
                self.turn.stop(Status.INVALID)

def build_move_sumo_to_rally_start(context, config):
    root = Sequence(
        name="③ 相撲終了位置からETラリー開始位置へ",
        memory=True,
    )

    root.add_children([
        TurnTowardRallyStart(context, config),
        StopNow(name="③ 旋回完了・停止"),
    ])

    return root



# ---- 青い丸への移動 ----

import time

from py_etrobo_util import Color, ColorClassifier
from ..behaviours.gyro_drive import RunByGyro
from ..types import HeadingType


class StableBlueArrival(Behaviour):
    """最初の青検出から77.5mm進み、車軸中点を円中心へ近づける。"""

    BLUE_RADIUS_MM = 39.5
    SENSOR_FORWARD_MM = 38.0
    # 正なら停止目標を手前へ補正。実測するまで0。
    STOP_ADVANCE_MM = 0.0

    def __init__(self):
        super().__init__("青い丸中心への到着判定")
        self.classifier = ColorClassifier()

    def initialise(self):
        self.count = 0
        self.first_blue_distance = None
        self.blue_confirmed = False

    def update(self):
        import math
        runtime.require("color_sensor", "plotter")
        distance = float(runtime.plotter.get_distance())
        if not math.isfinite(distance):
            raise ValueError("invalid blue-centering distance")

        if not self.blue_confirmed:
            h, s, v = runtime.color_sensor.get_raw_color_hsv()
            if self.classifier.classify(h, s, v) == Color.BLUE:
                if self.count == 0:
                    self.first_blue_distance = distance
                self.count += 1
                if self.count >= 3:
                    self.blue_confirmed = True
                    self.logger.info(
                        "RALLY_BLUE CENTER_CONFIRMED first=%.1f now=%.1f "
                        "target_after_first=%.1f"
                        % (
                            self.first_blue_distance, distance,
                            self.BLUE_RADIUS_MM + self.SENSOR_FORWARD_MM
                            - self.STOP_ADVANCE_MM,
                        )
                    )
            else:
                # 単発ノイズは捨て、次の連続検出の先頭を使う。
                self.count = 0
                self.first_blue_distance = None

        if not self.blue_confirmed:
            return Status.RUNNING

        # 確定後はセンサーが青から外れても距離目標を維持する。
        progress = distance - self.first_blue_distance
        target = (
            self.BLUE_RADIUS_MM + self.SENSOR_FORWARD_MM
            - self.STOP_ADVANCE_MM
        )
        if progress >= target:
            self.logger.info(
                "RALLY_BLUE CENTER_STOP progress=%.1f target=%.1f"
                % (progress, target)
            )
            return Status.SUCCESS
        return Status.RUNNING


class WaitForSumoRecordingClose(Behaviour):
    """Check asynchronous recording close after the rally turn."""

    def __init__(self, context):
        super().__init__("③ 相撲録画の終了確認")
        self.context = context

    def initialise(self):
        self.started_at = time.monotonic()

    def update(self):
        recording = runtime.video.get_sumo_alignment_recording_status()
        reason = None
        if recording is None:
            reason = "sumo recording status missing"
        elif recording["error"]:
            reason = "sumo recording failed: " + str(recording["error"])
        elif recording["finished"]:
            return Status.SUCCESS
        elif time.monotonic() - self.started_at > 5.0:
            reason = "sumo recording finalize timeout"
        else:
            return Status.RUNNING

        for motor in (runtime.left_motor, runtime.right_motor):
            if motor is not None:
                motor.set_power(0)
                motor.set_brake(True)
        self.context.sumo.failure_reason = reason
        self.logger.error(reason)
        return Status.FAILURE


class TurnAfterBlue(Behaviour):
    """青い丸到着後、RIGHTは90度、LEFTは270度へ正対する。"""

    def __init__(self, context, config):
        super().__init__("⑤ 青い丸到着後にラリー方向へ正対")
        self.context = context
        self.config = config
        self.turn = None

    def initialise(self):
        self.turn = None
        self.started_at = time.monotonic()

    def update(self):
        if time.monotonic() - self.started_at > 6.0:
            self.logger.error("BLUE_EXIT_TURN timeout")
            return Status.FAILURE

        if self.turn is None:
            if runtime.course not in (-1, 1):
                self.logger.error("BLUE_EXIT_TURN unsupported course")
                return Status.FAILURE

            target = 90.0 if runtime.course == -1 else 270.0
            motion = self.config.sumo
            self.turn = SpinToBearing(
                name="face rally direction after blue arrival",
                context=self.context,
                bearing=target,
                max_power=50,
                min_power=50,
                pid_p=motion.turn_pid_p,
                pid_i=motion.turn_pid_i,
                pid_d=motion.turn_pid_d,
                tolerance=1.0,
            )
            self.logger.info(
                "BLUE_EXIT_TURN START course=%s bearing=%.2f "
                "target=%.2f power=50"
                % (runtime.course, current_bearing(self.context), target)
            )

        self.turn.tick_once()
        if self.turn.status == Status.SUCCESS:
            self.logger.info(
                "BLUE_EXIT_TURN COMPLETE bearing=%.2f target=%.2f"
                % (current_bearing(self.context), self.turn.target_bearing)
            )
        return self.turn.status

    def terminate(self, new_status):
        try:
            if self.turn is not None and self.turn.status == Status.RUNNING:
                self.turn.stop(Status.INVALID)
        finally:
            for motor in (runtime.left_motor, runtime.right_motor):
                if motor is not None:
                    motor.set_power(0)
                    motor.set_brake(True)


class MoveToRallyBlue(Behaviour):
    """画像認識→方向補正→距離制限付き前進→青色検知で停止。"""

    DETECT_TIMEOUT_SEC = 5.0
    TURN_TIMEOUT_SEC = 6.0
    DRIVE_TIMEOUT_SEC = 8.0
    MAX_OBSERVATION_AGE_SEC = 0.5
    MAX_DRIVE_MM = 750.0
    DRIVE_POWER = 70
    TURN_POWER = 20
    CENTER_TOLERANCE_PX = 12

    def __init__(self, context, config):
        super().__init__("④ 青い丸へ移動")
        self.context = context
        self.config = config
        self.session = None
        self.phase = None
        self.started_at = None
        self.phase_started_at = None
        self.turn = None
        self.drive = None

    def initialise(self):
        runtime.require(
            "video", "plotter", "gyro_sensor",
            "color_sensor", "left_motor", "right_motor"
        )
        self.turn = None
        self.drive = None
        self.started_at = time.monotonic()
        self.phase_started_at = self.started_at
        self.phase = "DETECT"
        self.blue_alignment_turns = 0
        self.edge_turns = 0
        self._edge_frames = 0
        self._edge_last_frame = None
        self.session = runtime.video.begin_rally_blue_read()
        self.logger.info(
            "RALLY_BLUE START session=%s" % self.session
        )

    def _stop_motors(self):
        for motor in (runtime.left_motor, runtime.right_motor):
            if motor is not None:
                motor.set_power(0)
                motor.set_brake(True)

    def _fail(self, reason):
        self.logger.error("RALLY_BLUE FAILURE: %s" % reason)
        self._stop_motors()
        return Status.FAILURE

    def _set_phase(self, phase):
        self.phase = phase
        self.phase_started_at = time.monotonic()

    # 2026-11-17(devSS の応急処置): 青い丸が画面の左右の端で切れて候補にならないとき(相撲の終わりの位置によって起きる、
    # ロボットAで左下の端に切れて映り「blue circle not detected」で失敗した)、端の側へ向き直ってから探し直す。
    EDGE_CONFIRM_FRAMES = 3
    EDGE_MIN_TURN_DEG = 12.0
    EDGE_MAX_TURN_DEG = 30.0
    EDGE_MAX_TURNS = 2

    def _edge_turn(self, observation):
        import math
        frame = observation.get("frame_id")
        if frame != getattr(self, "_edge_last_frame", None):
            self._edge_last_frame = frame
            self._edge_frames = getattr(self, "_edge_frames", 0) + 1
        if self._edge_frames < self.EDGE_CONFIRM_FRAMES or getattr(self, "edge_turns", 0) >= self.EDGE_MAX_TURNS:
            return None
        edge = observation["edge"]
        error_px = float(edge["cx"]) - 160.0
        correction = 0.6 * math.degrees(math.atan(error_px * 270.0 / 320.0 / 230.0))
        sign = -1.0 if edge["side"] == "left" else 1.0
        correction = sign * min(self.EDGE_MAX_TURN_DEG, max(self.EDGE_MIN_TURN_DEG, abs(correction)))
        target_bearing = (current_bearing(self.context) + correction) % 360.0
        self.edge_turns = getattr(self, "edge_turns", 0) + 1
        self._edge_frames = 0
        motion = self.config.sumo
        self.turn = SpinToBearing(
            "rally blue edge turn", self.context, target_bearing,
            max_power=50, min_power=50,
            pid_p=motion.turn_pid_p, pid_i=motion.turn_pid_i, pid_d=motion.turn_pid_d,
            tolerance=2.0,
        )
        self.logger.warning(
            "RALLY_BLUE EDGE side=%s cx=%.1f area=%.0f -> turn %.1f deg to %.2f (edge turn %d)"
            % (edge["side"], edge["cx"], edge["area"], correction, target_bearing, self.edge_turns))
        self._set_phase("TURN")
        return Status.RUNNING

    # 2026-11-17(devSS): 青い丸の中心を床の上の位置に直して向き・距離を決める(_ground / _decide_from_ground)。
    # それまでは「画面の左右のずれ×0.6」で向きを直し、色センサーが青を踏んでから77.5mm 進んで止めていた。
    # 丸(直径79mm)の中心からずれて進むと、端を踏んだ所から測るので止まる位置が前後にずれ(ETラリーのスタートが
    # 計画から6〜8cm ずれた)、4°ずれたときは丸の横を通り過ぎて750mm 走って失敗した(ロボットA、2026-11-17)。
    # 保存された画像で、出した距離は色センサーで踏んだ位置と合う(644mm に対し踏んだ位置から 625〜664mm)。
    GROUND_SPIN_MIN_DEG = 6.0     # これより大きくずれていれば、まずその場で向き直る(小さければ走りながら向ける)
    GROUND_MAX_DIST_MM = 1000.0
    GROUND_MIN_DIST_MM = 150.0
    GROUND_DRIVE_POWER = 70   # 11/17: 60 で 547mm に2.5秒。止まりの行き過ぎは4mm だったので前と同じ70に
    _ground_camera = None

    def _ground(self, candidate):
        try:
            import math
            import cv2
            import numpy as np
            from .. import et_rally_start_pose as sp
            cam = MoveToRallyBlue._ground_camera
            if cam is None or cam.calib_path != sp.DEFAULT_CALIB:
                cam = MoveToRallyBlue._ground_camera = sp.GroundCamera(sp.DEFAULT_CALIB, build_map=False)
            # 画像は 640x480 の真ん中の16:9(640x360)を 320x180 に縮めたもの(py_etrobo_util/video.py の BOTTLE と同じ)
            u, v = 2.0 * float(candidate["cx"]), 2.0 * float(candidate["cy"]) + 60.0
            n = cv2.undistortPoints(np.array([[[u, v]]], np.float64), cam.K, cam.dist).reshape(2)
            ray = cam.R0.T @ np.array([n[0], n[1], 1.0])
            if ray[2] >= -1e-6:
                return None
            k = -cam.C0[2] / ray[2]
            fwd, left = float(cam.C0[0] + k * ray[0]), float(cam.C0[1] + k * ray[1])
            dist = math.hypot(fwd, left)
            if not (self.GROUND_MIN_DIST_MM <= dist <= self.GROUND_MAX_DIST_MM) or fwd <= 0:
                self.logger.warning("RALLY_BLUE GROUND out of range fwd=%.0f left=%.0f, using the old way" % (fwd, left))
                return None
            return fwd, left
        except Exception as error:
            self.logger.warning("RALLY_BLUE GROUND unavailable (%r), using the old way" % (error,))
            return None

    def _decide_from_ground(self, ground):
        import math
        fwd, left = ground
        dist = math.hypot(fwd, left)
        angle = math.degrees(math.atan2(left, fwd))      # 左が+
        correction = -angle                               # 方位(bearing)は時計回りが+
        self.logger.info(
            "RALLY_BLUE GROUND center fwd=%.0f left=%+.0f mm -> distance %.0f mm, angle %+.1f deg (calib %s)"
            % (fwd, left, dist, angle, getattr(MoveToRallyBlue._ground_camera, "calib_path", None)))
        if abs(angle) > self.GROUND_SPIN_MIN_DEG and self.blue_alignment_turns < 3:
            self.blue_alignment_turns += 1
            motion = self.config.sumo
            target_bearing = (current_bearing(self.context) + correction) % 360.0
            self.turn = SpinToBearing(
                "rally blue alignment", self.context, target_bearing,
                max_power=50, min_power=50,
                pid_p=motion.turn_pid_p, pid_i=motion.turn_pid_i, pid_d=motion.turn_pid_d,
                tolerance=2.0,
            )
            self.logger.info("RALLY_BLUE TURN correction=%.2f target=%.2f (ground)" % (correction, target_bearing))
            self._set_phase("TURN")
            return Status.RUNNING
        self._start_drive(distance_mm=dist, bearing_offset_deg=correction)
        return Status.RUNNING

    def update(self):
        now = time.monotonic()

        if self.phase == "DETECT":
            if now - self.phase_started_at > self.DETECT_TIMEOUT_SEC:
                return self._fail("blue circle not detected")

            observation = runtime.video.get_rally_blue_observation(
                self.session
            )

            # 1秒ごとに画像認識の状態を記録する。
            if not hasattr(self, "_last_debug_log"):
                self._last_debug_log = 0.0

            if now - self._last_debug_log >= 1.0:
                self._last_debug_log = now

                if observation is None:
                    self.logger.info(
                        "RALLY_BLUE DEBUG observation=None"
                    )
                else:
                    self.logger.info(
                        "RALLY_BLUE DEBUG frame=%s confirmed=%s "
                        "candidate=%s candidate_count=%s age=%.3f"
                        % (
                            observation.get("frame_id"),
                            observation.get("confirmed"),
                            observation.get("candidate"),
                            observation.get("candidate_count"),
                            now - observation.get("t", now),
                        ))

            if (observation and not observation.get("confirmed") and observation.get("edge")
                    and now - observation.get("t", 0.0) <= self.MAX_OBSERVATION_AGE_SEC):
                edge_turn = self._edge_turn(observation)
                if edge_turn is not None:
                    return edge_turn
            elif observation and not observation.get("edge"):
                self._edge_frames = 0

            if not observation or not observation.get("confirmed"):
                return Status.RUNNING

            if now - observation["t"] > self.MAX_OBSERVATION_AGE_SEC:
                return Status.RUNNING

            candidate = observation.get("candidate")
            if not candidate:
                return Status.RUNNING

            cx = float(candidate["cx"])
            error_px = cx - 160.0

            self.logger.info(
                "RALLY_BLUE DETECT cx=%.1f error_px=%.1f area=%.1f"
                % (cx, error_px, candidate["area"])
            )

            # 2026-11-17(devSS): カメラの較正(ETラリーのカメラと同じ)で、丸の中心の床の上の位置(車軸から前・左 mm)を出し、
            # その向きへ向けて、その距離だけ走る(色センサーで丸の端を踏んだ所から決めた距離で止めるのをやめる)。
            ground = self._ground(candidate)
            if ground is not None:
                return self._decide_from_ground(ground)

            # 画像中央に十分近い場合は追加旋回しない。
            if abs(error_px) <= self.CENTER_TOLERANCE_PX:
                self._start_drive()
                return Status.RUNNING

            # 既存の黒ボトル認識と同じ画角モデルによる初期推定。
            # 実機で符号・倍率を検証すること。
            import math
            correction = 0.6 * math.degrees(
                math.atan(error_px * 270.0 / 320.0 / 230.0)
            )

            target_bearing = (
                current_bearing(self.context) + correction
            ) % 360.0

            motion = self.config.sumo
            power = 50  # 青い丸への補正旋回専用

            # 2026-11-17(devSS の応急処置、相撲の改良版が来るまで): 残りの修正が補正旋回の許容幅(2°)以下なら、
            # 旋回しても動かない(spin ... samples=0)ので、合っているとみなして前進する。ロボットA・Cとも、カメラの左右の
            # 向きの個体差で青い丸が真ん中から15画素前後ずれて見え、約2°の修正を3回繰り返して失敗し、ETラリーへ
            # つながらなかった。また回数の上限に達したときも、失敗にせず今の向きで前進する(ETラリーへつなぐ)。
            if abs(correction) <= 2.0:
                self.logger.info(
                    "RALLY_BLUE SMALL correction=%.2f <= spin tolerance 2.0, driving as aligned" % correction
                )
                self._start_drive()
                return Status.RUNNING
            if self.blue_alignment_turns >= 3:
                self.logger.warning(
                    "RALLY_BLUE correction limit reached (correction=%.2f), driving with the current heading" % correction
                )
                self._start_drive()
                return Status.RUNNING
            self.blue_alignment_turns += 1
            self.turn = SpinToBearing(
                "rally blue alignment",
                self.context,
                target_bearing,
                max_power=power,
                min_power=power,
                pid_p=motion.turn_pid_p,
                pid_i=motion.turn_pid_i,
                pid_d=motion.turn_pid_d,
                tolerance=2.0,
            )

            self.logger.info(
                "RALLY_BLUE TURN correction=%.2f target=%.2f"
                % (correction, target_bearing)
            )
            self._set_phase("TURN")
            return Status.RUNNING

        if self.phase == "TURN":
            if now - self.phase_started_at > self.TURN_TIMEOUT_SEC:
                return self._fail("alignment turn timeout")

            self.turn.tick_once()

            if self.turn.status == Status.FAILURE:
                return self._fail("alignment turn failed")

            if self.turn.status == Status.SUCCESS:
                self._stop_motors()
                self.turn = None
                self._set_phase("ALIGN_SETTLE")

            return Status.RUNNING

        if self.phase == "ALIGN_SETTLE":
            self._stop_motors()
            if now - self.phase_started_at < 0.2:
                return Status.RUNNING
            # 旋回前・旋回中の画像を使わず、新しい世代で再確認する。
            runtime.video.end_rally_blue_read(self.session)
            self.session = runtime.video.begin_rally_blue_read()
            self.logger.info(
                "RALLY_BLUE RECHECK session=%s corrections=%d"
                % (self.session, self.blue_alignment_turns)
            )
            self._set_phase("DETECT")
            return Status.RUNNING

        if self.phase == "DRIVE":
            if now - self.phase_started_at > self.DRIVE_TIMEOUT_SEC:
                return self._fail("drive timeout")

            self.drive.tick_once()

            if getattr(self, "drive_by_ground", False) and self.drive_blue_first is None:
                # 確かめのログだけ: 色センサーが最初に青を見た距離(丸の端。センサーは車軸の38mm 前)
                try:
                    h, s_, v = runtime.color_sensor.get_raw_color_hsv()
                    if self.drive_classifier.classify(h, s_, v) == Color.BLUE:
                        self.drive_blue_first = float(runtime.plotter.get_distance()) - self.drive_start_mm
                except Exception:
                    pass

            if self.drive.status == Status.SUCCESS:
                extra = ""
                if getattr(self, "drive_by_ground", False):
                    extra = " driven=%.0f first_blue_at=%s" % (
                        float(runtime.plotter.get_distance()) - self.drive_start_mm,
                        "none" if self.drive_blue_first is None else "%.0f" % self.drive_blue_first)
                self.logger.info(
                    "RALLY_BLUE ARRIVED distance=%.1f%s"
                    % (runtime.plotter.get_distance(), extra)
                )
                return Status.SUCCESS

            if self.drive.status == Status.FAILURE:
                return self._fail(
                    "blue not reached within distance limit"
                )

            return Status.RUNNING

        return self._fail("invalid phase")

    def _start_drive(self, distance_mm=None, bearing_offset_deg=0.0):
        # RunByGyroのABSOLUTEは床面方位ではなく、
        # -runtime.course * gyro角度を使用する。
        # 2026-11-17: distance_mm があれば(カメラで丸の中心の位置が分かったとき)、方位を bearing_offset_deg だけ
        # 向けて(方位は ジャイロ+定数 なので、この向きでは -course 倍)、その距離で止める(色センサーは見ない)。
        target_heading = (
            -runtime.course * (float(runtime.gyro_sensor.get_angle()) + float(bearing_offset_deg))
        )
        motion = self.config.sumo
        by_ground = distance_mm is not None
        power = self.GROUND_DRIVE_POWER if by_ground else self.DRIVE_POWER
        limit = float(distance_mm) if by_ground else self.MAX_DRIVE_MM
        self.drive = RunByGyro(
            name="rally blue gyro approach",
            target=lambda distance: target_heading,
            power=power,
            pid_p=motion.drive_pid_p,
            pid_i=motion.drive_pid_i,
            pid_d=motion.drive_pid_d,
            target_type=HeadingType.ABSOLUTE,
            distance_limit_mm=limit,
            completion_condition=None if by_ground else StableBlueArrival(),
            completion_min_mm=0.0,
        )
        self.drive_by_ground = by_ground
        self.drive_start_mm = float(runtime.plotter.get_distance())
        self.drive_blue_first = None
        self.drive_classifier = ColorClassifier() if by_ground else None

        self.logger.info(
            "RALLY_BLUE DRIVE heading=%.2f power=%d limit=%.1f%s"
            % (
                target_heading,
                power,
                limit,
                " (to the circle centre by camera)" if by_ground else "",
            )
        )
        self._set_phase("DRIVE")

    def terminate(self, new_status):
        if self.turn is not None and self.turn.status == Status.RUNNING:
            self.turn.stop(Status.INVALID)

        if self.drive is not None and self.drive.status == Status.RUNNING:
            self.drive.stop(Status.INVALID)

        self._stop_motors()

        if self.session is not None:
            runtime.video.end_rally_blue_read(self.session)
            self.session = None


# 既存のbuild_move_sumo_to_rally_startを拡張する。
_original_build_move_sumo_to_rally_start = build_move_sumo_to_rally_start


def build_move_sumo_to_rally_start(context, config):
    root = _original_build_move_sumo_to_rally_start(context, config)
    if config.mission_mode in ("sumo-stage-all", "bottle-sumo", "full"):
        root.add_child(WaitForSumoRecordingClose(context))
    root.add_children([
        MoveToRallyBlue(context, config),
        StopNow(name="④ 青い丸到着・停止"),
        TurnAfterBlue(context, config),
        StopNow(name="⑤ ラリー方向正対完了・停止"),
    ])
    return root
