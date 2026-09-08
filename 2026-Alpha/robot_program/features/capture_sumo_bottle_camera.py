"""Features 16 and 17: find and capture the sumo bottle with the camera."""

import math

from simple_pid import PID

from .bt_imports import Behaviour, BottleColor, HeadingType, Parallel, ParallelPolicy, Selector, Sequence, Status, runtime, time

from ..behaviours.conditions import IsDistanceEarned
from ..behaviours.gyro_drive import RunByGyro
from ..behaviours.motor_control import StopNow
from ..timing import CONTROL_INTERVAL_SEC


def _average_heading(headings):
    # 359度と1度を単純平均して180度にしないよう、単位円上で平均する。
    if not headings:
        return 0.0
    sin_sum = sum(math.sin(math.radians(value)) for value in headings)
    cos_sum = sum(math.cos(math.radians(value)) for value in headings)
    return math.degrees(math.atan2(sin_sum, cos_sum)) % 360.0


class CaptureSumoBottleWithCamera(Behaviour):
    # 停止状態で黒テープを確定後、画像上の角度を0度へ寄せながら前進する。
    # ボトルが近距離の死角へ入ったら、最後に観測した走行方位を後段へ渡す。
    ACQUIRE = 0
    APPROACH = 1

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
        self.heading_history = []
        self.started_at = 0.0
        self.approach_started_at = None
        self.last_drive_log_at = None
        self.pid = None

    def initialise(self):
        runtime.require(
            "plotter", "video", "gyro_sensor", "right_motor", "left_motor"
        )
        self.phase = self.ACQUIRE
        self.session = runtime.video.begin_sumo_bottle_read()
        self.last_frame_id = -1
        self.confirmed_frames = 0
        self.lost_frames = 0
        self.last_bottom_row = 0
        self.heading_history = []
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
        self.context.sumo.skipped = False
        self.context.sumo.failure_reason = None
        self.context.sumo.camera_capture_heading_deg = None
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

    def _current_heading(self):
        return (-runtime.course * runtime.gyro_sensor.get_angle()) % 360.0

    def _fail(self, reason):
        self.context.sumo.skipped = True
        self.context.sumo.failure_reason = reason
        self.logger.warning(
            "%+06d %s.%s"
            % (runtime.plotter.get_distance(), self.__class__.__name__, reason)
        )
        return Status.FAILURE

    def update(self):
        now = time.monotonic()
        session, frame_id, observation = runtime.video.get_bottle_observation()

        # カメラモード切替前の結果と、制御周期内で再読した同一フレームは無視する。
        if session != self.session or frame_id <= self.last_frame_id:
            if (
                self.phase == self.ACQUIRE
                and now - self.started_at >= self.settings.camera_detection_timeout_sec
            ):
                return self._fail("sumo black bottle camera detection timed out")
            if (
                self.phase == self.APPROACH
                and self.approach_started_at is not None
                and now - self.approach_started_at
                >= self.settings.camera_approach_timeout_sec
            ):
                return self._fail("sumo camera frame update timed out")
            return Status.RUNNING
        self.last_frame_id = frame_id

        insight, color, _cx, theta, bottom_row, area, in_blind = observation
        valid = (
            insight
            and color == BottleColor.BLACK
            and area >= self.settings.camera_min_area_px
        )

        # 実行単位1：静止したまま黒テープを連続した新規フレームで確認する。
        if self.phase == self.ACQUIRE:
            self.confirmed_frames = self.confirmed_frames + 1 if valid else 0
            if self.confirmed_frames < self.settings.camera_confirm_frames:
                if now - self.started_at >= self.settings.camera_detection_timeout_sec:
                    return self._fail("sumo black bottle camera detection timed out")
                return Status.RUNNING
            self.phase = self.APPROACH
            self.approach_started_at = now
            self.logger.info(
                "%+06d %s.black bottle confirmed frame=%d theta=%.1f area=%d"
                % (
                    runtime.plotter.get_distance(),
                    self.__class__.__name__,
                    frame_id,
                    theta,
                    area,
                )
            )

        # 実行単位2：回転探索は行わず、前進しながら黒テープを画像中央へ寄せる。
        if valid:
            self.lost_frames = 0
            self.last_bottom_row = bottom_row
            self.heading_history.append(self._current_heading())
            if len(self.heading_history) > 5:
                self.heading_history.pop(0)
            left_power, right_power = self._drive_toward_bottle(theta)
            # 実行中に操舵していることを、画像角度と左右PWMの差で確認できるようにする。
            if (
                self.last_drive_log_at is None
                or now - self.last_drive_log_at
                >= self.settings.camera_drive_log_interval_sec
            ):
                self.logger.info(
                    "%+06d %s.camera steer frame=%d theta=%.1f bottom=%d area=%d pwm_l=%d pwm_r=%d"
                    % (
                        runtime.plotter.get_distance(),
                        self.__class__.__name__,
                        frame_id,
                        theta,
                        bottom_row,
                        area,
                        left_power,
                        right_power,
                    )
                )
                self.last_drive_log_at = now
        else:
            self.lost_frames += 1
            # 一時的な画像欠落では最後の向きから逸れないよう両輪を同出力にする。
            self._drive_toward_bottle(0.0)

        # 実行単位3：テープが画面下端へ達したら、最後の観測方位を死角走行へ渡す。
        near_then_lost = (
            not valid
            and self.last_bottom_row >= self.settings.camera_near_bottom_row
            and self.lost_frames >= self.settings.camera_lost_frame_limit
        )
        if in_blind or near_then_lost:
            headings = self.heading_history or [self._current_heading()]
            self.context.sumo.camera_capture_heading_deg = _average_heading(headings)
            self.logger.info(
                "%+06d %s.black tape entered blind area heading=%.1f"
                % (
                    runtime.plotter.get_distance(),
                    self.__class__.__name__,
                    self.context.sumo.camera_capture_heading_deg,
                )
            )
            return Status.SUCCESS

        # 遠い位置で見失った場合は、ボトル位置を推測して走り続けず安全に省略する。
        if (
            not valid
            and self.lost_frames >= self.settings.camera_lost_frame_limit
            and self.last_bottom_row < self.settings.camera_near_bottom_row
        ):
            return self._fail("sumo black bottle was lost before blind area")

        if (
            self.approach_started_at is not None
            and now - self.approach_started_at
            >= self.settings.camera_approach_timeout_sec
        ):
            return self._fail("sumo camera approach timed out")
        return Status.RUNNING

    def terminate(self, new_status):
        # 成功、失敗、中断のどの場合も次のBehaviorへ出力を残さない。
        if runtime.right_motor is not None and runtime.left_motor is not None:
            self._stop_motors()


class ConfigureBlindCaptureHeading(Behaviour):
    # カメラで最後に安定して見えていた絶対方位を、既存RunByGyroへ設定する。
    def __init__(self, name, context, gyro_drive):
        super().__init__(name)
        self.context = context
        self.gyro_drive = gyro_drive

    def update(self):
        heading = self.context.sumo.camera_capture_heading_deg
        if heading is None:
            return Status.FAILURE
        self.gyro_drive.target = heading
        return Status.SUCCESS


class MarkCameraCaptureCompleted(Behaviour):
    # アーム内へ規定距離押し込んだことを、オープンループの捕捉完了として記録する。
    def __init__(self, name, context):
        super().__init__(name)
        self.context = context

    def update(self):
        self.context.sumo.bottle_captured = True
        return Status.SUCCESS


class CompleteSkippedCameraCapture(Behaviour):
    # 未検出時はET相撲を失敗終了させず、後続の運搬を安全に省略できる状態へする。
    def __init__(self, name, context):
        super().__init__(name)
        self.context = context

    def update(self):
        return Status.SUCCESS if self.context.sumo.skipped else Status.FAILURE


def build_capture_sumo_bottle_camera(context, config):
    # No.16・17：黒テープをカメラで捕捉し、前進操舵と死角直進でアーム内へ収める。
    settings = config.sumo
    blind_drive_command = RunByGyro(
        name="run through camera blind area to capture sumo bottle",
        target=0,
        power=settings.camera_approach_power,
        pid_p=settings.drive_pid_p,
        pid_i=settings.drive_pid_i,
        pid_d=settings.drive_pid_d,
        target_type=HeadingType.ABSOLUTE,
    )
    blind_drive = Parallel(
        name="drive camera blind capture distance",
        policy=ParallelPolicy.SuccessOnOne(),
    )
    blind_drive.add_children(
        [
            blind_drive_command,
            IsDistanceEarned(
                name="sumo camera blind capture distance",
                delta_dist=settings.camera_blind_capture_distance_mm,
            ),
        ]
    )

    capture = Sequence(name="capture sumo bottle by camera", memory=True)
    capture.add_children(
        [
            # 実行順1：停止中に黒テープを確定し、回転せず前進操舵して死角入口まで接近する。
            CaptureSumoBottleWithCamera(
                name="locate and approach sumo bottle by camera",
                context=context,
                settings=settings,
            ),
            # 実行順2：カメラ操舵の終了点で一度制動し、距離計測の起点を固定する。
            StopNow(name="stop at sumo camera blind edge"),
            # 実行順3：最後に観測した方位を、既存のジャイロ直進へ設定する。
            ConfigureBlindCaptureHeading(
                name="configure sumo blind capture heading",
                context=context,
                gyro_drive=blind_drive_command,
            ),
            # 実行順4：黒テープが見えない区間だけ、調整可能な距離を直進する。
            blind_drive,
            # 実行順5：アーム内へ押し込んだ位置で停止する。
            StopNow(name="stop after camera sumo capture"),
            # 実行順6：後続のボトル運搬を有効にする。
            MarkCameraCaptureCompleted(
                name="mark camera sumo capture completed",
                context=context,
            ),
        ]
    )

    root = Selector(name="capture_sumo_bottle_camera", memory=True)
    root.add_children(
        [
            capture,
            CompleteSkippedCameraCapture(
                name="complete skipped sumo camera capture",
                context=context,
            ),
        ]
    )
    return root
