from ..behaviours.gyro_drive import SpinAround

"""③ ET相撲終了後、ETラリー開始位置の青い丸へ向けて旋回する。"""

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence

from ..runtime import runtime
from ..features.sumo_bearing_motion import SpinToBearing, current_bearing
from ..behaviours.motor_control import StopNow


class TurnTowardRallyStart(Behaviour):
    """進入基準の方位へ、相撲終了時の姿勢に応じた角度で旋回する。"""

    def __init__(self, context, config):
        super().__init__("③ 青い丸へ向けて旋回")
        self.context = context
        self.config = config
        self.turn = None

    def initialise(self):
        self.turn = None

    def _target_from_axle_pose(self, reference, start_bearing):
        import math
        pose = reference.get("axle_pose")
        if pose is None:
            self.logger.info(
                "RALLY_TURN GEOMETRY_FALLBACK reason=no-axle-pose"
            )
            return None
        try:
            x, y = float(pose["x"]), float(pose["y"])
            approach = float(reference["approach_bearing"])
            old_bearing = float(pose["bearing"])
            old_left = float(pose["left"])
            old_right = float(pose["right"])
            left = float(runtime.left_motor.get_count())
            right = float(runtime.right_motor.get_count())
            diameter = float(runtime.plotter.tire_diameter)
            values = (
                x, y, approach, old_bearing, old_left, old_right,
                left, right, diameter, start_bearing,
            )
            if not all(math.isfinite(v) for v in values) or diameter <= 0:
                raise ValueError("invalid pose/encoder")

            # 最終計測後の制動中の移動も反映する。
            distance = (
                math.pi * diameter
                * ((left-old_left)+(right-old_right)) / 720.0
            )
            if abs(distance) > 100.0:
                raise ValueError("encoder discontinuity")
            change = (start_bearing-old_bearing+180.0) % 360.0 - 180.0
            angle = math.radians(old_bearing + change/2.0 - approach)
            x += distance * math.cos(angle)
            y += distance * math.sin(angle)

            # 横方向は進行方向の右が正。LEFTは左、RIGHTは右。
            blue_x = 1170.0
            blue_y = -runtime.course * blue_x * math.tan(math.radians(14.0))
            dx, dy = blue_x-x, blue_y-y
            if math.hypot(dx, dy) < 1.0:
                raise ValueError("axle already at blue center")
            target = (
                approach + math.degrees(math.atan2(dy, dx))
            ) % 360.0
            self.logger.info(
                "RALLY_TURN GEOMETRY axle=(%.1f,%.1f) "
                "blue=(%.1f,%.1f) target=%.2f"
                % (x, y, blue_x, blue_y, target)
            )
            return target
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            self.logger.info(
                "RALLY_TURN GEOMETRY_FALLBACK reason=%s" % exc
            )
            return None

    def update(self):
        if self.turn is None:
            import math
            if runtime.course not in (-1, 1):
                self.logger.error("RALLY_TURN: unsupported course")
                return Status.FAILURE
            start_bearing = current_bearing(self.context)
            reference = getattr(self.context.sumo, "rally_turn_reference", None)
            signed_angle = runtime.course * 215.0
            if reference and reference.get("course") == runtime.course:
                approach = float(reference["approach_bearing"])
                if not math.isfinite(approach):
                    return Status.FAILURE
                target = (approach + runtime.course * 215.0) % 360.0
                computed = self._target_from_axle_pose(reference, start_bearing)
                if computed is not None:
                    target = computed
                # 従来と同じ旋回方向を保つ。最短方向には切り替えない。
                signed_angle = (target - start_bearing) % 360.0
                if runtime.course == -1:
                    signed_angle = -((start_bearing - target) % 360.0)
                if not 150.0 <= abs(signed_angle) <= 280.0:
                    self.logger.error("RALLY_TURN: reference outside turn bound")
                    return Status.FAILURE
            target_bearing = (start_bearing + signed_angle) % 360.0

            self.logger.info(
                "RALLY_TURN START course=%s angle=%.1f "
                "start=%.2f target=%.2f"
                % (runtime.course, signed_angle, start_bearing, target_bearing)
            )

            power = 60  # 初期旋回
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
            self.turn.stall_boost = True  # 2026-11-17(devSS): 動かないときは出力を段階的に上げる(gyro_drive.SpinAround)

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
    STOP_ADVANCE_MM = 39.0  # ライトコースの実測による手前補正
    LEFT_STOP_ADVANCE_MM = 29.0  # レフトの右40mm偏りに対する初期補正

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
                            - (self.STOP_ADVANCE_MM if runtime.course == -1
                               else self.LEFT_STOP_ADVANCE_MM),
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
            - (self.STOP_ADVANCE_MM if runtime.course == -1
                               else self.LEFT_STOP_ADVANCE_MM)
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
                max_power=55,
                min_power=55,
                pid_p=motion.turn_pid_p,
                pid_i=motion.turn_pid_i,
                pid_d=motion.turn_pid_d,
                tolerance=1.0,
            )
            self.turn.stall_boost = True  # 2026-11-17(devSS): 動かないときは出力を段階的に上げる(gyro_drive.SpinAround)
            self.logger.info(
                "BLUE_EXIT_TURN START course=%s bearing=%.2f "
                "target=%.2f power=55"
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


# 2026-11-17(devSS): これより小さい青は青い丸として扱わない(上の MoveToRallyBlue の DETECT)
RALLY_BLUE_MIN_AREA = 250.0


class MoveToRallyBlue(Behaviour):
    """画像認識→方向補正→距離制限付き前進→青色検知で停止。"""

    DETECT_TIMEOUT_SEC = 5.0
    TURN_TIMEOUT_SEC = 6.0
    DRIVE_TIMEOUT_SEC = 8.0
    MAX_OBSERVATION_AGE_SEC = 0.5
    MAX_DRIVE_MM = 750.0
    DRIVE_POWER = 70
    TURN_POWER = 20
    CENTER_TOLERANCE_PX = 4
    LEFT_AIM_LEFT_MM = 28.0  # 接近方向に対して左へ狙う初期補正
    FINE_ERROR_PX = 15.0
    FINE_PULSE_SEC = 0.02
    SETTLE_TIMEOUT_SEC = 1.5
    VIEW_BACK_STEP_MM = 30.0
    VIEW_BACK_MAX_STEPS = 3
    VIEW_BACK_POWER = 35
    VIEW_BACK_TIMEOUT_SEC = 2.0
    SEARCH_WAIT_SEC = 2.0
    SEARCH_LIMIT_DEG = 20.0
    ACQUIRE_TIMEOUT_SEC = 45.0

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
        self.view_back_steps = 0
        self.blue_alignment_turns = 0
        self.blue_area_reference = None
        self.blue_area_reject_log_at = None
        self.blue_previous_error = None
        self.blue_pulse_only = False
        self.search_origin = current_bearing(self.context)
        reference = getattr(self.context.sumo, "rally_turn_reference", None)
        bottle_delta = 0.0
        if reference and reference.get("course") == runtime.course:
            bottle_delta = float(reference.get("bottle_delta", 0.0))
        # 左右情報は探索順序だけに使い、未実測の角度換算は行わない。
        direction = 1.0 if bottle_delta >= 0.0 else -1.0
        self.search_offsets = [direction * sign * step
                               for step in (5.0, 10.0, 15.0, 20.0)
                               for sign in (1.0, -1.0)]
        self.search_count = 0
        self._stop_motors()
        self._set_phase("ALIGN_SETTLE")
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
        if phase == "ALIGN_SETTLE":
            self.blue_settle_samples = []
        if phase == "DETECT":
            self.blue_search_frame = -1
            self.blue_empty_frames = 0
            self.blue_candidate_at = None

    def _start_view_back(self):
        import math
        if self.view_back_steps >= self.VIEW_BACK_MAX_STEPS:
            return self._fail("blue view backoff distance limit")

        values = (
            float(runtime.left_motor.get_count()),
            float(runtime.right_motor.get_count()),
            float(runtime.plotter.tire_diameter),
            float(runtime.gyro_sensor.get_angle()),
        )
        if not all(math.isfinite(v) for v in values) or values[2] <= 0:
            return self._fail("invalid blue backoff sensors")

        self.view_back_origin = values[:2]
        self.view_back_mm_per_deg = math.pi * values[2] / 360.0
        self.view_back_heading = values[3]
        self.view_back_steps += 1
        self._stop_motors()
        self._set_phase("VIEW_BACK")
        self.logger.info(
            "RALLY_BLUE VIEW_BACK step=%d target=%.1f"
            % (self.view_back_steps, self.VIEW_BACK_STEP_MM)
        )
        return self._update_view_back()

    def _update_view_back(self):
        import math
        now = time.monotonic()
        values = (
            float(runtime.left_motor.get_count()),
            float(runtime.right_motor.get_count()),
            float(runtime.gyro_sensor.get_angle()),
        )
        if not all(math.isfinite(v) for v in values):
            return self._fail("invalid blue backoff sensors")

        progress = sum(
            abs(v - origin)
            for v, origin in zip(values[:2], self.view_back_origin)
        )
        progress *= self.view_back_mm_per_deg / 2.0
        error = (
            self.view_back_heading - values[2] + 180.0
        ) % 360.0 - 180.0

        if abs(error) > 8.0:
            return self._fail("blue backoff heading drift")

        if progress >= self.VIEW_BACK_STEP_MM:
            self._stop_motors()
            self.search_origin = current_bearing(self.context)
            self.search_count = 0
            self.logger.info(
                "RALLY_BLUE VIEW_BACK_STOP progress=%.1f" % progress
            )
            self._set_phase("ALIGN_SETTLE")
            return Status.RUNNING

        if now - self.phase_started_at > self.VIEW_BACK_TIMEOUT_SEC:
            return self._fail("blue view backoff timeout")

        correction = max(-10.0, min(10.0, 2.0 * error))
        runtime.left_motor.set_power(
            int(round(-self.VIEW_BACK_POWER + correction))
        )
        runtime.right_motor.set_power(
            int(round(-self.VIEW_BACK_POWER - correction))
        )
        return Status.RUNNING

    def _blue_candidate_matches(self, candidate, now):
        import math
        try:
            area = float(candidate["area"])
        except (KeyError, TypeError, ValueError, OverflowError):
            return False
        if not math.isfinite(area) or area <= 0.0:
            return False

        previous = self.blue_area_reference
        if previous is not None and area < previous * 0.45:
            logged = self.blue_area_reject_log_at
            if logged is None or now - logged >= 1.0:
                self.logger.info(
                    "RALLY_BLUE REJECT_AREA area=%.1f previous=%.1f ratio=%.3f"
                    % (area, previous, area / previous)
                )
                self.blue_area_reject_log_at = now
            # 小さな別候補で旋回せず、再検出を待つ。
            return False

        self.blue_area_reference = area
        return True

    def _blue_search_ready(self, observation, now):
        # カメラ未更新と、青候補が見えない状態を区別する。
        if not observation:
            return False

        import math
        try:
            t = float(observation["t"])
            frame = int(observation["frame_id"])
        except (KeyError, TypeError, ValueError, OverflowError):
            return False

        if (
            not math.isfinite(t)
            or not 0 <= now - t <= self.MAX_OBSERVATION_AGE_SEC
        ):
            return False

        visible = (
            observation.get("candidate") is not None
            or observation.get("candidate_count", 0) > 0
        )
        if frame > getattr(self, "blue_search_frame", -1):
            self.blue_search_frame = frame
            if visible:
                self.blue_empty_frames = 0
                self.blue_candidate_at = t
            else:
                self.blue_empty_frames = (
                    getattr(self, "blue_empty_frames", 0) + 1
                )

        if visible:
            return False

        recent = getattr(self, "blue_candidate_at", None)
        return (
            now - self.phase_started_at >= self.SEARCH_WAIT_SEC
            and getattr(self, "blue_empty_frames", 0) >= 3
            and (recent is None or now - recent >= 1.0)
        )

    def _start_search(self):
        if self.search_count >= len(self.search_offsets):
            return self._fail("blue not found within bounded search")
        offset = self.search_offsets[self.search_count]
        self.search_count += 1
        if abs(offset) > self.SEARCH_LIMIT_DEG:
            return self._fail("invalid blue search bound")
        target = (self.search_origin + offset) % 360.0
        motion = self.config.sumo
        self.turn = SpinToBearing(
            "rally blue acquisition search", self.context, target,
            max_power=50, min_power=50,
            pid_p=motion.turn_pid_p, pid_i=motion.turn_pid_i,
            pid_d=motion.turn_pid_d, tolerance=1.0,
        )
        self.turn.stall_boost = True  # 2026-11-17(devSS): 動かないときは出力を段階的に上げる(gyro_drive.SpinAround)
        self.logger.info("RALLY_BLUE SEARCH count=%d offset=%.1f target=%.2f"
                         % (self.search_count, offset, target))
        self._set_phase("TURN")
        return Status.RUNNING

    def update(self):
        now = time.monotonic()
        if self.phase != "DRIVE" and now - self.started_at > self.ACQUIRE_TIMEOUT_SEC:
            return self._fail("blue acquisition total timeout")

        if self.phase == "VIEW_BACK":
            return self._update_view_back()

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

            if (observation and not observation.get("confirmed")
                    and observation.get("candidate_count", 0) == 0):
                import math
                age = now - float(
                    observation.get("t", float("nan"))
                )
                hint = observation.get("bottom_hint")
                if (
                    math.isfinite(age)
                    and 0.0 <= age <= self.MAX_OBSERVATION_AGE_SEC
                    and hint is not None
                ):
                    return self._start_view_back()

            if not observation or not observation.get("confirmed"):
                if self._blue_search_ready(observation, now):
                    return self._start_search()
                return Status.RUNNING

            if now - observation["t"] > self.MAX_OBSERVATION_AGE_SEC:
                return Status.RUNNING

            candidate = observation.get("candidate")
            if not candidate:
                return Status.RUNNING
            # 2026-11-17(devSS): 小さすぎる青(面積 RALLY_BLUE_MIN_AREA 未満)は青い丸として扱わない。
            # 本物の丸は面積約500〜1560、遠くの小さな青は約110〜160で、それを交互に丸と見て左右に向き直りを
            # 繰り返し、上限6回に達して失敗した(ロボットA、16:43)。小さいものは見なかったことにして探し続ける。
            if float(candidate.get("area", 0.0)) < RALLY_BLUE_MIN_AREA:
                if now - getattr(self, "_small_blue_logged", 0.0) >= 1.0:
                    self._small_blue_logged = now
                    self.logger.info("RALLY_BLUE IGNORE small blue area=%.0f cx=%.0f (< %.0f)"
                                     % (float(candidate.get("area", 0.0)), float(candidate.get("cx", 0.0)), RALLY_BLUE_MIN_AREA))
                if now - self.phase_started_at >= self.SEARCH_WAIT_SEC:
                    return self._start_search()
                return Status.RUNNING

            if not self._blue_candidate_matches(candidate, now):
                return Status.RUNNING
            cx = float(candidate["cx"])
            try:
                aim_cx = self._blue_aim_cx(candidate)
            except (KeyError, TypeError, ValueError):
                return self._fail("invalid blue aiming measurement")
            error_px = cx - aim_cx

            self.logger.info(
                "RALLY_BLUE DETECT cx=%.1f error_px=%.1f area=%.1f aim_px=%.1f"
                % (cx, error_px, candidate["area"], aim_cx)
            )

            # 2026-11-18(devSS、ユーザー): カメラの較正(ETラリーと同じ、ロボットごとの較正ファイル)で丸の中心の
            # 床の上の位置(車軸から前・左 mm)を出し、車軸まわりの角度と中心までの距離で向き直り・走る。
            # 較正が読めない・範囲外のときは下の今までのやり方(画面のずれ×0.6、色センサーで止める)。
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

            if self.blue_alignment_turns >= 3:
                self.logger.info(
                    "RALLY_BLUE CORRECTION_LIMIT_CONTINUE corrections=%d "
                    "error_px=%.2f aligned=False"
                    % (self.blue_alignment_turns, error_px)
                )
                self._start_drive()
                return Status.RUNNING
            self.blue_alignment_turns += 1
            if (self.blue_previous_error is not None
                    and self.blue_previous_error * error_px < 0.0):
                self.blue_pulse_only = True
            self.blue_previous_error = error_px
            if abs(error_px) <= self.FINE_ERROR_PX or self.blue_pulse_only:
                self.turn = None
                self.blue_pulse_gyro = float(runtime.gyro_sensor.get_angle())
                if not math.isfinite(self.blue_pulse_gyro):
                    return self._fail("invalid blue fine-turn gyro")
                self._set_phase("FINE_TURN")
                # 画像右は物理的な右旋回。コースによる反転はしない。
                command = power if error_px > 0 else -power
                runtime.left_motor.set_power(command)
                runtime.right_motor.set_power(-command)
                self.logger.info(
                    "RALLY_BLUE FINE_PULSE error_px=%.2f power=%d duration=%.3f"
                    % (error_px, power, self.FINE_PULSE_SEC)
                )
                return Status.RUNNING
            self.turn = SpinToBearing(
                "rally blue alignment",
                self.context,
                target_bearing,
                max_power=power,
                min_power=power,
                pid_p=motion.turn_pid_p,
                pid_i=motion.turn_pid_i,
                pid_d=motion.turn_pid_d,
                tolerance=0.5,
            )
            self.turn.stall_boost = True  # 2026-11-17(devSS): 動かないときは出力を段階的に上げる(gyro_drive.SpinAround)
            self.turn.predictive_brake_horizon_sec = 0.01

            self.logger.info(
                "RALLY_BLUE TURN correction=%.2f target=%.2f"
                % (correction, target_bearing)
            )
            self._set_phase("TURN")
            return Status.RUNNING

        if self.phase == "FINE_TURN":
            import math
            gyro = float(runtime.gyro_sensor.get_angle())
            if not math.isfinite(gyro):
                return self._fail("invalid blue fine-turn gyro")
            travel = abs((gyro - self.blue_pulse_gyro + 180.0) % 360.0 - 180.0)
            if now - self.phase_started_at >= self.FINE_PULSE_SEC or travel >= 1.0:
                self._stop_motors()
                self._set_phase("ALIGN_SETTLE")
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
            import math
            self._stop_motors()
            gyro = float(runtime.gyro_sensor.get_angle())
            if not math.isfinite(gyro):
                return self._fail("invalid blue settle gyro")
            if now - self.phase_started_at > self.SETTLE_TIMEOUT_SEC:
                return self._fail("blue alignment did not settle")
            self.blue_settle_samples.append((now, gyro))
            self.blue_settle_samples = [
                sample for sample in self.blue_settle_samples
                if now - sample[0] <= 0.15
            ]
            origin = self.blue_settle_samples[0][1]
            angles = [
                (angle - origin + 180.0) % 360.0 - 180.0
                for _, angle in self.blue_settle_samples
            ]
            quiet = (
                now - self.blue_settle_samples[0][0] >= 0.10
                and max(angles) - min(angles) <= 0.6
            )
            if now - self.phase_started_at < 0.2 or not quiet:
                return Status.RUNNING
            if self.search_count == 0 and self.blue_alignment_turns == 0:
                self.search_origin = current_bearing(self.context)
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
                    extra = " driven=%.0f first_blue_at=%s (stop command; it coasts about %.0f mm more)" % (
                        float(runtime.plotter.get_distance()) - self.drive_start_mm,
                        "none" if self.drive_blue_first is None else "%.0f" % self.drive_blue_first,
                        self.GROUND_STOP_ADVANCE_MM)
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

    # ---- 2026-11-18(devSS): カメラの較正で丸の中心の位置を出す(11/17 の 99cb177c を今の devAT の流れに入れ直し) ----
    # 11/17 に同じ方法で4回走り、4回とも丸に着いた(ロボットA)。画面のずれ×0.6 の式は、カメラの向きのずれ(Cで左へ約0.7°)
    # や、カメラが車軸より前にあること(回る中心は車軸)を含まない。色センサーで端を踏んだ所から止めると、
    # 丸の中心から横にずれて通ったときに止まる位置が前後にずれる。
    GROUND_SPIN_MIN_DEG = 3.0     # これより大きいずれは、その場で向き直ってから見直す(小さければ走りながら向ける)
    GROUND_MAX_DIST_MM = 1000.0
    GROUND_MIN_DIST_MM = 150.0
    # 止める命令から止まるまでに進む分(出力70)。devAT が色センサーの止め方で実測した手前補正(StableBlueArrival.STOP_ADVANCE_MM)
    # と同じ値。11/17 の4回でも、止めたあと約25〜40mm 進んでいた。
    GROUND_STOP_ADVANCE_MM = 39.0
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
            # 画像は 640x480 の真ん中の16:9(640x360)を 320x180 に縮めたもの(py_etrobo_util/video.py の BOTTLE)
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
        angle = math.degrees(math.atan2(left, fwd))      # 車軸から見た丸の中心の向き、左が+
        correction = -angle                               # 方位(bearing)は時計回りが+
        self.logger.info(
            "RALLY_BLUE GROUND center fwd=%.0f left=%+.0f mm -> distance %.0f mm, angle %+.1f deg (calib %s)"
            % (fwd, left, dist, angle, getattr(MoveToRallyBlue._ground_camera, "calib_path", None)))
        if abs(angle) > self.GROUND_SPIN_MIN_DEG and self.blue_alignment_turns < 3:
            self.blue_alignment_turns += 1
            motion = self.config.sumo
            target_bearing = (current_bearing(self.context) + correction) % 360.0
            # 向き直りは devAT の補正旋回と同じ設定(出力50・許容0.5°・止まったら出力を上げる・先読みブレーキ)
            self.turn = SpinToBearing(
                "rally blue alignment", self.context, target_bearing,
                max_power=50, min_power=50,
                pid_p=motion.turn_pid_p, pid_i=motion.turn_pid_i, pid_d=motion.turn_pid_d,
                tolerance=0.5,
            )
            self.turn.stall_boost = True
            self.turn.predictive_brake_horizon_sec = 0.01
            self.logger.info("RALLY_BLUE TURN correction=%.2f target=%.2f (ground)" % (correction, target_bearing))
            self._set_phase("TURN")
            return Status.RUNNING
        self._start_drive(distance_mm=max(0.0, dist - self.GROUND_STOP_ADVANCE_MM), bearing_offset_deg=correction)
        return Status.RUNNING

    @staticmethod
    def _blue_aim_cx(candidate):
        # 円径を画像内の横方向の尺度にする近似。
        # 左を狙う場合、青い円は画面中央より右に置く。
        if runtime.course != 1:
            return 160.0

        import math
        width = float(candidate["w"])
        if not math.isfinite(width) or not 0.0 < width <= 320.0:
            raise ValueError("invalid blue ellipse width")

        diameter = 2.0 * StableBlueArrival.BLUE_RADIUS_MM
        return (
            160.0
            + width * MoveToRallyBlue.LEFT_AIM_LEFT_MM / diameter
        )

    def _start_drive(self, distance_mm=None, bearing_offset_deg=0.0):
        # RunByGyroのABSOLUTEは床面方位ではなく、
        # -runtime.course * gyro角度を使用する。
        # 2026-11-18(devSS): distance_mm があれば(カメラの較正で丸の中心が分かったとき)、方位を bearing_offset_deg だけ
        # 向けて(方位は ジャイロ+定数 なので、この向きでは -course 倍)、その距離で止める(色センサーは確かめのログだけ)。
        target_heading = (
            -runtime.course * (float(runtime.gyro_sensor.get_angle()) + float(bearing_offset_deg))
        )
        motion = self.config.sumo
        by_ground = distance_mm is not None
        limit = float(distance_mm) if by_ground else self.MAX_DRIVE_MM

        self.drive = RunByGyro(
            name="rally blue gyro approach",
            target=lambda distance: target_heading,
            power=self.DRIVE_POWER,
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
                self.DRIVE_POWER,
                limit,
                " (to the circle centre by camera, offset %+.1f deg)" % bearing_offset_deg if by_ground else "",
            )
        )
        self._set_phase("DRIVE")
        # 2026-11-17(devSS、ユーザー): 位置合わせが終わって丸へ走り出したら、もう本体のカメラは使わない(止まるのは色センサー)。
        self._early_camera_handover()

    def _early_camera_handover(self):
        """2026-11-17(devSS、ユーザー): 本体のカメラを閉じて、ETラリーのカメラ(子プロセス)を開かせる。ETラリーの始めに開くと
        使えるまで約1.9秒かかり、スタートで止まっている時間が延びていた。ETラリーの始め
        (et_rally_course_pose.EtRallyCameraProcessStart)は、開いていればもう渡さない。"""
        try:
            from .. import et_rally_camera_proc
            proc = et_rally_camera_proc.current()
        except Exception:
            proc = None
        if proc is None or not getattr(proc, "defer_open", False) or getattr(proc, "opened", True):
            return
        release = getattr(runtime, "release_camera", None)
        if release is None:
            return
        if self.session is not None and runtime.video is not None:
            try:
                runtime.video.end_rally_blue_read(self.session)
            except Exception:
                pass
        self.session = None
        import threading
        t0 = time.monotonic()
        logger = self.logger

        def work():
            try:
                ok, what = release()
            except Exception as error:
                ok, what = False, "release failed: %r" % (error,)
            proc.open_camera()
            logger.info("RALLY_BLUE camera handover (early): main camera %s (%s), child told to open after %.2fs"
                        % ("closed" if ok else "NOT closed", what, time.monotonic() - t0))

        threading.Thread(target=work, name="rally-blue-camera-handover", daemon=True).start()

    def terminate(self, new_status):
        if self.turn is not None and self.turn.status == Status.RUNNING:
            self.turn.stop(Status.INVALID)

        if self.drive is not None and self.drive.status == Status.RUNNING:
            self.drive.stop(Status.INVALID)

        self._stop_motors()

        if self.session is not None and runtime.video is not None:
            runtime.video.end_rally_blue_read(self.session)
        self.session = None


# 既存のbuild_move_sumo_to_rally_startを拡張する。
_original_build_move_sumo_to_rally_start = build_move_sumo_to_rally_start


def build_move_sumo_to_rally_start(context, config):
    root = _original_build_move_sumo_to_rally_start(context, config)
    # 2026-11-17(devSS): devAT のマージで戻っていた devTK(d7e6598)の "full" を入れ直す
    if config.mission_mode in ("sumo-stage-all", "bottle-sumo", "full"):
        root.add_child(WaitForSumoRecordingClose(context))
    root.add_children([
        MoveToRallyBlue(context, config),
        StopNow(name="④ 青い丸到着・停止"),
        TurnAfterBlue(context, config),
        StopNow(name="⑤ ラリー方向正対完了・停止"),
    ])
    return root
