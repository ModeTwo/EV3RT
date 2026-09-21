"""Features 16 and 17: find and capture the sumo bottle with the camera."""

from simple_pid import PID
import math

from .bt_imports import Behaviour, BottleColor, HeadingType, Parallel, ParallelPolicy, Selector, Sequence, Status, runtime, time

from ..behaviours.conditions import IsDistanceEarned
from .sumo_bearing_motion import current_bearing, EncoderSpinToBearing as SpinToBearing
from ..behaviours.motor_control import StopNow
from ..timing import CONTROL_INTERVAL_SEC


def _normalize_heading(angle):
    # 方位差を-180度以上180度未満へ正規化し、0/360度境界で逆回転しないようにする。
    return (float(angle) + 180.0) % 360.0 - 180.0


class CaptureSumoBottleWithCamera(Behaviour):
    # 停止状態で黒テープから絶対方位を確定し、その方位をジャイロ基準で維持して前進する。
    # 走行中の遅延した画像角度を追い続けず、高出力のまま大きく回り込む挙動を防ぐ。
    ACQUIRE = 0
    APPROACH = 1
    ALIGN = 2
    SETTLE = 3

    def __init__(self, name, context, settings):
        super().__init__(name)
        self.context = context
        self.settings = settings
        self.phase = self.ACQUIRE
        self.session = None
        self.last_frame_id = -1
        self.confirmed_frames = 0
        self.lost_frames = 0
        self.last_bottom_row = 0
        self.last_theta = None
        self.target_bearing = None
        self.started_at = 0.0
        self.approach_started_at = None
        self.last_drive_log_at = None
        self.pid = None

    def initialise(self):
        self.context.sumo.push_end_position_mm = None
        self.green_turn_needed = False
        self.green_curve_started = False
        self.green_origin_distance = None
        self.alignment_turn = None
        self.alignment_checked = False
        self.settle_until = 0.0
        runtime.require(
            "plotter", "video", "gyro_sensor", "right_motor", "left_motor"
        )
        # 全体ResetDevice直後に登録済みであることを、モーター走行前に確認する。
        current_bearing(self.context)
        self.phase = self.ACQUIRE
        self.session = runtime.video.begin_sumo_bottle_read()
        self.last_frame_id = -1
        self.confirmed_frames = 0
        self.lost_frames = 0
        self.last_bottom_row = 0
        self.last_theta = None
        self.target_bearing = None
        self.started_at = time.monotonic()
        self.approach_started_at = None
        self.last_drive_log_at = None
        self.pid = PID(
            self.settings.camera_steer_gain,
            0.0,
            0.0,
            setpoint=0.0,
            sample_time=CONTROL_INTERVAL_SEC,
            output_limits=(
                -self.settings.camera_max_steer_power,
                self.settings.camera_max_steer_power,
            ),
        )
        # 検出確定時に起点を記録する距離判定。新しい実行では必ず作り直す。
        self.total_distance = IsDistanceEarned(
            name="sumo total capture and push distance",
            delta_dist=self.settings.capture_and_push_distance_mm,
        )
        self.context.sumo.bottle_captured = False
        self.context.sumo.bottle_pushed_out = False
        self.context.sumo.skipped = False
        self.context.sumo.failure_reason = None
        self.context.sumo.camera_capture_bearing_deg = None
        # 黒テープ確定中は必ず静止し、同じフレームを複数回数えない。
        self._stop_motors()
        self.logger.info(
            "%+06d %s.black bottle acquisition started"
            % (runtime.plotter.get_distance(), self.__class__.__name__)
        )

    def _stop_motors(self):
        for motor in (runtime.right_motor, runtime.left_motor):
            motor.set_power(0)
            motor.set_brake(True)

    def _drive_toward_bottle(self, theta):
        # theta自体が画像上の左右方向を持つため、course符号による反転は行わない。
        turn = int(self.pid(theta))
        right_power = max(
            self.settings.camera_min_wheel_power,
            min(
                self.settings.camera_max_wheel_power,
                self.settings.camera_approach_power + turn,
            ),
        )
        left_power = max(
            self.settings.camera_min_wheel_power,
            min(
                self.settings.camera_max_wheel_power,
                self.settings.camera_approach_power - turn,
            ),
        )
        for motor in (runtime.right_motor, runtime.left_motor):
            motor.set_brake(False)
        runtime.right_motor.set_power(right_power)
        runtime.left_motor.set_power(left_power)
        return left_power, right_power

    def _current_bearing(self):
        return current_bearing(self.context)

    def _estimated_bottle_bearing(self, theta):
        # 画像右の角度は時計回りの方位差。カメラの左右はコースで反転しない。
        return (
            self._current_bearing() + float(theta)
        ) % 360.0

    def _drive_toward_locked_bearing(self):
        # 停止中に確定したボトル方位との差を画像角度相当に戻し、既存の高出力範囲で操舵する。
        if self.target_bearing is None:
            self._stop_motors()
            return 0, 0, 0.0
        heading_error = _normalize_heading(
            self.target_bearing - self._current_bearing()
        )
        # 方位目標を既存のジャイロ基準へ変換してから、従来の高出力操舵へ渡す。
        raw = runtime.gyro_sensor.get_angle()
        legacy_target = self.context.sumo.bearing_reference.legacy_target(self.target_bearing, raw, runtime.course)
        legacy_error = _normalize_heading(legacy_target - (-runtime.course * raw))
        equivalent_theta = -runtime.course * legacy_error
        left_power, right_power = self._drive_toward_bottle(equivalent_theta)
        return left_power, right_power, heading_error

    def _fail(self, reason):
        self.context.sumo.skipped = True
        self.context.sumo.failure_reason = reason
        self.logger.warning(
            "%+06d %s.%s"
            % (runtime.plotter.get_distance(), self.__class__.__name__, reason)
        )
        return Status.FAILURE

    def _prepare_green_route(self):
        # 既存の開始位置近似を採用。旋回前の撮影後退は-Xとする。
        self.green_x = -self.settings.camera_retreat_distance_mm
        self.green_y = self.settings.start_straight_distance_mm
        self.green_origin_distance = runtime.plotter.get_distance()
        self.green_last_distance = self.green_origin_distance
        angle = math.radians(-runtime.course * (self.target_bearing - self.settings.entry_bearing_deg))
        forward = max(0.0, math.cos(angle))
        limit = self.settings.green_boundary_y_mm - self.settings.green_clearance_mm
        self.green_turn_needed = self.green_y + self.settings.capture_and_push_distance_mm * forward >= limit
        self.logger.info("Green route risk=%s start_y=%.1f safe_y=%.1f" % (self.green_turn_needed, self.green_y, limit))
        # 捕捉前に境界へ迫る場合は、無理に曲がらず走行開始前に失敗停止する。
        if self.green_turn_needed and (self.green_y + self.settings.green_capture_distance_mm * forward
                                      + self.settings.green_curve_reserve_mm >= limit):
            return False
        return True

    def _update_green_route(self):
        # 前進区間のみを積分。旋回・前進カーブ後の復帰位置計算へ引き渡す。
        distance = runtime.plotter.get_distance()
        delta = abs(distance - self.green_last_distance)
        self.green_last_distance = distance
        angle = math.radians(-runtime.course * (self._current_bearing() - self.settings.entry_bearing_deg))
        self.green_x += delta * math.sin(angle)
        self.green_y += delta * math.cos(angle)
        self.context.sumo.push_end_position_mm = (self.green_x, self.green_y)
        if self.green_y >= self.settings.green_boundary_y_mm - self.settings.green_clearance_mm:
            return False
        if (self.green_turn_needed and not self.green_curve_started
                and abs(distance - self.green_origin_distance) >= self.settings.green_capture_distance_mm):
            # 両輪前進の既存操舵を再利用し、土俵側90度へカーブする。
            self.target_bearing = (self.settings.entry_bearing_deg - runtime.course * 90.0) % 360.0
            self.pid.reset()
            self.green_curve_started = True
            self.logger.info("Capture assumed; curving toward ring bearing=%.1f" % self.target_bearing)
        return True

    def update(self):
        # 前進せず既存のその場旋回Behaviorでボトル方位へ整列する。
        if self.phase == self.ALIGN:
            self.alignment_turn.tick_once()
            if self.alignment_turn.status == Status.FAILURE:
                self._stop_motors()
                return Status.FAILURE
            if self.alignment_turn.status == Status.SUCCESS:
                self._stop_motors()
                self.settle_until = time.monotonic() + self.settings.camera_alignment_settle_sec
                self.phase = self.SETTLE
            return Status.RUNNING
        if self.phase == self.SETTLE:
            self._stop_motors()
            if time.monotonic() >= self.settle_until:
                # 補正は一度だけ。画像再認識へ戻らず、最初に確定した方位で前進する。
                # 旋回中の車輪移動を含めず、この位置を500mm走行の起点にする。
                self.phase = self.APPROACH
                self.total_distance.update()
                self.approach_started_at = time.monotonic()
                self.pid.reset()
                self.context.sumo.camera_capture_bearing_deg = self.target_bearing
                if self.settings.green_avoidance_enabled and not self._prepare_green_route():
                    return self._fail("insufficient_green_clearance_for_capture_and_curve")
                self.logger.info("Single alignment complete; starting distance drive target=%.1f" % self.target_bearing)
                self._drive_toward_locked_bearing()
            return Status.RUNNING
        # 方位確定後は画像更新を待たず、毎制御周期で500mm到達を確認する。
        # キャッチ・押し出しを一つの距離へ含め、死角判定による追加走行は行わない。
        if self.phase == self.APPROACH:
            if self.settings.green_avoidance_enabled and not self._update_green_route():
                return self._fail("green_boundary_clearance_reached")
            if self.total_distance.update() == Status.SUCCESS:
                self._stop_motors()
                self.logger.info("Capture and push distance completed; proceeding to reverse")
                return Status.SUCCESS
            self._drive_toward_locked_bearing()
            return Status.RUNNING

        now = time.monotonic()
        session, frame_id, observation = runtime.video.get_bottle_observation()

        # カメラモード切替前の結果と、制御周期内で再読した同一フレームは無視する。
        if session != self.session or frame_id <= self.last_frame_id:
            if self.phase == self.APPROACH:
                # カメラの新規フレーム待ちでも、20ms周期のジャイロ値で固定方位へ制御し続ける。
                self._drive_toward_locked_bearing()
            return Status.RUNNING
        self.last_frame_id = frame_id

        insight, color, cx, theta, bottom_row, area, in_blind = observation

        if insight and color == BottleColor.BLACK:
            self.logger.info(
                "SUMO BLACK DETECT "
                "frame=%d cx=%s theta=%.1f bottom=%s area=%.1f blind=%s bearing=%.1f"
                % (
                    frame_id,
                    cx,
                    theta,
                    bottom_row,
                    area,
                    in_blind,
                    self._current_bearing(),
                )
            )
        
        

        valid = (
            insight
            and color == BottleColor.BLACK
            and area >= self.settings.camera_min_area_px
        )

        # 実行単位1：静止したまま黒テープを連続した新規フレームで確認する。
        if self.phase == self.ACQUIRE:
            self.confirmed_frames = self.confirmed_frames + 1 if valid else 0
            if self.confirmed_frames < self.settings.camera_confirm_frames:
                return Status.RUNNING
            self.target_bearing = self._estimated_bottle_bearing(theta)
            if not self.alignment_checked or abs(theta) > self.settings.camera_alignment_tolerance_deg:
                self._stop_motors()
                self.alignment_turn = SpinToBearing(
                    name="align to camera bottle bearing", context=self.context,
                    bearing=self.target_bearing, max_power=self.settings.turn_max_power,
                    min_power=self.settings.turn_min_power, pid_p=self.settings.turn_pid_p,
                    pid_i=self.settings.turn_pid_i, pid_d=self.settings.turn_pid_d,
                    tolerance=self.settings.heading_tolerance_deg,
                )
                self.phase = self.ALIGN
                self.logger.info("Bottle alignment requested theta=%.1f target=%.1f" % (theta, self.target_bearing))
                return Status.RUNNING
            self.phase = self.APPROACH
            # 旋回によるエンコーダー距離を含めず、前進開始点から500mmを測る。
            self.total_distance.update()
            self.approach_started_at = now
            self.pid.reset()
            self.context.sumo.camera_capture_bearing_deg = self.target_bearing
            self.logger.info(
                "%+06d %s.black bottle confirmed frame=%d theta=%.1f area=%d target_bearing=%.1f"
                % (
                    runtime.plotter.get_distance(),
                    self.__class__.__name__,
                    frame_id,
                    theta,
                    area,
                    self.target_bearing,
                )
            )

        # 方位が確定したこの周期から前進する。以降は総距離だけで終了する。
        self._drive_toward_locked_bearing()
        return Status.RUNNING

    def terminate(self, new_status):
        if getattr(self, "alignment_turn", None) is not None:
            self.alignment_turn.stop(Status.INVALID)
        # 成功、失敗、中断のどの場合も次のBehaviorへ出力を残さない。
        if runtime.right_motor is not None and runtime.left_motor is not None:
            self._stop_motors()


class MarkCameraCaptureCompleted(Behaviour):
    # 総距離走行を終えたことを、捕捉・押し出し完了の仮定として記録する。
    def __init__(self, name, context):
        super().__init__(name)
        self.context = context

    def update(self):
        self.context.sumo.bottle_captured = True
        self.context.sumo.bottle_pushed_out = True
        return Status.SUCCESS


class CompleteSkippedCameraCapture(Behaviour):
    # 未検出時はET相撲を失敗終了させず、後続の運搬を安全に省略できる状態へする。
    def __init__(self, name, context):
        super().__init__(name)
        self.context = context

    def update(self):
        return Status.SUCCESS if self.context.sumo.skipped else Status.FAILURE


def build_capture_sumo_bottle_camera(context, config):
    # No.16・17：画像で方位確定後、キャッチと押し出しを含む合計500mmを走る。
    # 追加の死角150mm・押し出し距離を足さず、No.18の直線後退へ引き渡す。
    capture = Sequence(name="capture sumo bottle by camera", memory=True)
    capture.add_children([
        CaptureSumoBottleWithCamera(
            name="locate and approach sumo bottle by camera",
            context=context,
            settings=config.sumo,
        ),
        StopNow(name="stop after total sumo capture and push distance"),
        MarkCameraCaptureCompleted(
            name="mark camera sumo capture completed",
            context=context,
        ),
    ])
    return capture
