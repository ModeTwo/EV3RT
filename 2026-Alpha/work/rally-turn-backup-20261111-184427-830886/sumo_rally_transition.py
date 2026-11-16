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

            angle = 245.0

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

            start_bearing = current_bearing(self.context)
            target_bearing = (start_bearing + signed_angle) % 360.0

            self.logger.info(
                "RALLY_TURN START course=%s angle=%.1f "
                "start=%.2f target=%.2f"
                % (runtime.course, signed_angle, start_bearing, target_bearing)
            )

            power = 50  # 245°旋回専用
            motion = self.config.sumo

            self.turn = SpinAround(
                "rally start camera direction",
                target=signed_angle,
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
    """床面カラーセンサーで青を3回連続検出したら成功。"""

    def __init__(self):
        super().__init__("青い丸への到着判定")
        self.classifier = ColorClassifier()
        self.count = 0

    def initialise(self):
        self.count = 0

    def update(self):
        runtime.require("color_sensor")
        h, s, v = runtime.color_sensor.get_raw_color_hsv()
        if self.classifier.classify(h, s, v) == Color.BLUE:
            self.count += 1
        else:
            self.count = 0

        return Status.SUCCESS if self.count >= 3 else Status.RUNNING


class MoveToRallyBlue(Behaviour):
    """画像認識→方向補正→距離制限付き前進→青色検知で停止。"""

    DETECT_TIMEOUT_SEC = 5.0
    TURN_TIMEOUT_SEC = 6.0
    DRIVE_TIMEOUT_SEC = 8.0
    MAX_OBSERVATION_AGE_SEC = 0.5
    MAX_DRIVE_MM = 650.0
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

            # 画像中央に十分近い場合は追加旋回しない。
            if abs(error_px) <= self.CENTER_TOLERANCE_PX:
                self._start_drive()
                return Status.RUNNING

            # 既存の黒ボトル認識と同じ画角モデルによる初期推定。
            # 実機で符号・倍率を検証すること。
            import math
            correction = math.degrees(
                math.atan(error_px * 270.0 / 320.0 / 230.0)
            )

            target_bearing = (
                current_bearing(self.context) + correction
            ) % 360.0

            motion = self.config.sumo
            power = 35  # 青い丸への補正旋回専用

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
                self._start_drive()

            return Status.RUNNING

        if self.phase == "DRIVE":
            if now - self.phase_started_at > self.DRIVE_TIMEOUT_SEC:
                return self._fail("drive timeout")

            self.drive.tick_once()

            if self.drive.status == Status.SUCCESS:
                self.logger.info(
                    "RALLY_BLUE ARRIVED distance=%.1f"
                    % runtime.plotter.get_distance()
                )
                return Status.SUCCESS

            if self.drive.status == Status.FAILURE:
                return self._fail(
                    "blue not reached within distance limit"
                )

            return Status.RUNNING

        return self._fail("invalid phase")

    def _start_drive(self):
        # RunByGyroのABSOLUTEは床面方位ではなく、
        # -runtime.course * gyro角度を使用する。
        target_heading = (
            -runtime.course * float(runtime.gyro_sensor.get_angle())
        )
        motion = self.config.sumo

        self.drive = RunByGyro(
            name="rally blue gyro approach",
            target=lambda distance: target_heading,
            power=self.DRIVE_POWER,
            pid_p=motion.drive_pid_p,
            pid_i=motion.drive_pid_i,
            pid_d=motion.drive_pid_d,
            target_type=HeadingType.ABSOLUTE,
            distance_limit_mm=self.MAX_DRIVE_MM,
            completion_condition=StableBlueArrival(),
            completion_min_mm=0.0,
        )

        self.logger.info(
            "RALLY_BLUE DRIVE heading=%.2f power=%d limit=%.1f"
            % (
                target_heading,
                self.DRIVE_POWER,
                self.MAX_DRIVE_MM,
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
    root.add_children([
        MoveToRallyBlue(context, config),
        StopNow(name="④ 青い丸到着・停止"),
    ])
    return root
