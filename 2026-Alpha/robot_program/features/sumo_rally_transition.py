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

    def update(self):
        if self.turn is None:
            import math
            if runtime.course not in (-1, 1):
                self.logger.error("RALLY_TURN: unsupported course")
                return Status.FAILURE
            start_bearing = current_bearing(self.context)
            reference = getattr(self.context.sumo, "rally_turn_reference", None)
            # 2026-11-17(devSS、実機C・右): 215°だと向いた先(235°)から青い丸が見つかった向きが 208〜228° と
            # 毎回マイナス側へずれ、左右に探して最長約10秒かかった。見つかった向きの真ん中(約222°)に向くよう 228°にする。
            signed_angle = runtime.course * RALLY_TURN_FROM_APPROACH_DEG
            if reference and reference.get("course") == runtime.course:
                approach = float(reference["approach_bearing"])
                if not math.isfinite(approach):
                    return Status.FAILURE
                target = (approach + runtime.course * RALLY_TURN_FROM_APPROACH_DEG) % 360.0
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
                max_power=50,
                min_power=50,
                pid_p=motion.turn_pid_p,
                pid_i=motion.turn_pid_i,
                pid_d=motion.turn_pid_d,
                tolerance=1.0,
            )
            self.turn.stall_boost = True  # 2026-11-17(devSS): 動かないときは出力を段階的に上げる(gyro_drive.SpinAround)
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


# 2026-11-17(devSS): これより小さい青は青い丸として扱わない(上の MoveToRallyBlue の DETECT)
RALLY_BLUE_MIN_AREA = 250.0
# 相撲の向き(approach)から青い丸へ向く角度(元 215)。上の RALLY_TURN を参照
RALLY_TURN_FROM_APPROACH_DEG = 228.0


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
    VIEW_BACK_MAX_STEPS = 4  # 2026-11-17: 最初に何も見えないときの1回を足した
    VIEW_BACK_POWER = 35
    VIEW_BACK_TIMEOUT_SEC = 2.0
    SEARCH_WAIT_SEC = 0.6
    SEARCH_LIMIT_DEG = 30.0
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
        self.blue_previous_error = None
        self.blue_pulse_only = False
        self.search_origin = current_bearing(self.context)
        reference = getattr(self.context.sumo, "rally_turn_reference", None)
        bottle_delta = 0.0
        if reference and reference.get("course") == runtime.course:
            bottle_delta = float(reference.get("bottle_delta", 0.0))
        # 左右情報は探索順序だけに使い、未実測の角度換算は行わない。
        direction = 1.0 if bottle_delta >= 0.0 else -1.0
        # 2026-11-17(devSS): 5°刻みの左右交互は遅かった(見つかるまで7回・約10秒)。丸はロボットに近く画面の下の端に
        # 写るので、少し向きがずれると見えない。8°刻みで、bottle_delta の側を先に2段見てから反対側を見る。
        self.search_offsets = [direction * s for s in (8.0, 16.0)] + [-direction * s for s in (8.0, 16.0)]             + [direction * 24.0, -direction * 24.0]
        self.initial_back_done = False
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
                if now - self.phase_started_at >= self.SEARCH_WAIT_SEC:
                    if not self.initial_back_done and self.search_count == 0:
                        # 2026-11-17(devSS): 最初に何も見えないときは、向きを変える前に一度少し下がる
                        # (丸が近くて画面の下に外れていることが多い。下がると下の端に写り始める)
                        self.initial_back_done = True
                        return self._start_view_back()
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

            if self.blue_alignment_turns >= 6:
                return self._fail("blue alignment correction limit")
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
                tolerance=1.0,
            )
            self.turn.stall_boost = True  # 2026-11-17(devSS): 動かないときは出力を段階的に上げる(gyro_drive.SpinAround)
            self.turn.predictive_brake_horizon_sec = 0.04

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
