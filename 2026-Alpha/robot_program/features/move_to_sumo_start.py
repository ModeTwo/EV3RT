"""Feature 15: move from the rally exit to the sumo search position."""

from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time

from ..behaviours.conditions import IsDistanceEarned
from ..behaviours.gyro_drive import RunByGyro, SpinAround
from ..behaviours.motor_control import RunAsInstructed, StopNow


class ConfigureCameraRetreatPwm(Behaviour):
    # RunAsInstructedがcourse符号を掛ける前の値を調整し、両コースで実出力を後退にする。
    def __init__(self, name, motor_command, power):
        super().__init__(name)
        self.motor_command = motor_command
        self.power = power

    def update(self):
        runtime.require("right_motor", "left_motor")
        command_power = -self.power if runtime.course >= 0 else self.power
        self.motor_command.pwm_l = command_power
        self.motor_command.pwm_r = command_power
        return Status.SUCCESS


class IsBlackThenBrightSurface(Behaviour):
    # ET相撲開始位置専用。黒線を1回確認した後、明るい路面が継続したことを生V値で判定する。
    # 共通色分類の彩度条件や5サンプル多数決に依存させず、白地がUNKNOWNになる影響を避ける。
    def __init__(self, name, settings):
        super().__init__(name)
        self.settings = settings
        self.black_seen = False
        self.bright_started_at = None
        self.last_log_at = None

    def update(self):
        runtime.require("plotter", "color_sensor")
        h, s, v = runtime.color_sensor.get_raw_color_hsv()
        now = time.monotonic()

        # 実機調整時に閾値の妥当性を判断できるよう、生HSVと判定段階を定期出力する。
        if (
            self.last_log_at is None
            or now - self.last_log_at >= self.settings.line_sensor_log_interval_sec
        ):
            stage = "waiting_black" if not self.black_seen else "waiting_bright"
            self.logger.info(
                "%+06d %s.hsv=(%d,%d,%d) stage=%s"
                % (
                    runtime.plotter.get_distance(),
                    self.__class__.__name__,
                    h,
                    s,
                    v,
                    stage,
                )
            )
            self.last_log_at = now

        # 実行単位1：青円上などの明るい開始地点では成功させず、黒線を先に確認する。
        if not self.black_seen:
            if v <= self.settings.line_black_max_value:
                self.black_seen = True
                self.logger.info(
                    "%+06d %s.black line confirmed v=%d"
                    % (
                        runtime.plotter.get_distance(),
                        self.__class__.__name__,
                        v,
                    )
                )
            return Status.RUNNING

        # 実行単位2：黒線確認後、Vが閾値以上の状態が連続したら白地へ抜けたと判断する。
        if v >= self.settings.line_white_min_value:
            if self.bright_started_at is None:
                self.bright_started_at = now
            if now - self.bright_started_at >= self.settings.line_exit_white_duration_sec:
                self.logger.info(
                    "%+06d %s.bright surface confirmed v=%d for %.3fs"
                    % (
                        runtime.plotter.get_distance(),
                        self.__class__.__name__,
                        v,
                        self.settings.line_exit_white_duration_sec,
                    )
                )
                return Status.SUCCESS
        else:
            self.bright_started_at = None
        return Status.RUNNING


class InitializeSumoState(Behaviour):
    # No.16以降で使用する探索基準方位と実行状態を初期化する。
    def __init__(self, name, context):
        super().__init__(name)
        self.context = context

    def update(self):
        runtime.require("plotter", "gyro_sensor")
        state = self.context.sumo
        state.started_at = time.monotonic()
        state.search_heading_deg = (
            -runtime.course * runtime.gyro_sensor.get_angle()
        ) % 360.0
        state.sonar_samples.clear()
        state.bottle_bearing_deg = None
        state.bottle_distance_mm = None
        state.approach_distance_mm = 0.0
        state.camera_capture_heading_deg = None
        state.skipped = False
        state.bottle_captured = False
        state.bottle_pushed_out = False
        state.bottle_released = False
        state.transport_completed = False
        state.bottle_held_at_exit = False
        state.line_trace_ready = False
        state.failure_reason = None
        self.logger.info(
            "%+06d %s.camera capture reference heading=%.1f"
            % (
                runtime.plotter.get_distance(),
                self.__class__.__name__,
                state.search_heading_deg,
            )
        )
        return Status.SUCCESS


def build_move_to_sumo_start(context, config):
    # No.15：青円上のETラリー終了位置から直進し、黒ラインを抜けてから土俵方向を向く。
    settings = config.sumo

    drive = Parallel(
        name="drive across black line to white area",
        policy=ParallelPolicy.SuccessOnOne(),
    )
    drive.add_children(
        [
            RunByGyro(
                name="run straight from rally exit",
                target=0,
                power=settings.navigation_power,
                pid_p=settings.drive_pid_p,
                pid_i=settings.drive_pid_i,
                pid_d=settings.drive_pid_d,
                target_type=HeadingType.RELATIVE,
            ),
            IsBlackThenBrightSurface(
                name="detect black line exit into white area",
                settings=settings,
            ),
        ]
    )

    clearance_drive = Parallel(
        name="drive clearance distance after leaving black line",
        policy=ParallelPolicy.SuccessOnOne(),
    )
    clearance_drive.add_children(
        [
            RunByGyro(
                name="run straight for sumo turn clearance",
                target=0,
                power=settings.navigation_power,
                pid_p=settings.drive_pid_p,
                pid_i=settings.drive_pid_i,
                pid_d=settings.drive_pid_d,
                target_type=HeadingType.RELATIVE,
            ),
            IsDistanceEarned(
                name="sumo turn clearance distance",
                delta_dist=settings.post_line_clearance_distance_mm,
            ),
        ]
    )

    camera_retreat_command = RunAsInstructed(
        name="reverse after ring turn",
        pwm_l=-settings.camera_retreat_power,
        pwm_r=-settings.camera_retreat_power,
    )
    camera_retreat = Parallel(
        name="reverse to widen sumo camera view",
        policy=ParallelPolicy.SuccessOnOne(),
    )
    camera_retreat.add_children(
        [
            camera_retreat_command,
            IsDistanceEarned(
                name="sumo camera retreat distance",
                delta_dist=settings.camera_retreat_distance_mm,
            ),
        ]
    )

    root = Sequence(name="move_to_sumo_start", memory=True)
    root.add_children(
        [
            # 実行順1：青円上ではライントレースせず、現在方位をジャイロで維持して直進する。
            # 黒を規定時間確認してから白が規定時間続いた時だけ、黒ラインを抜けたと判定する。
            drive,
            # 実行順2：白地の確認位置で一度制動し、追加直進距離の起点を明確にする。
            StopNow(name="stop after leaving black line"),
            # 実行順3：ゲートから旋回半径分離れるため、調整可能な距離を現在方位のまま直進する。
            clearance_drive,
            # 実行順4：追加直進後に制動し、この位置を90度旋回の中心にする。
            StopNow(name="stop after sumo turn clearance"),
            # 実行順5：SpinAroundのcourse反転により、Leftは左、Rightは右へ90度旋回して土俵側を向く。
            SpinAround(
                name="turn 90 degrees toward sumo ring",
                target=settings.ring_turn_deg,
                max_power=settings.turn_max_power,
                min_power=settings.turn_min_power,
                pid_p=settings.turn_pid_p,
                pid_i=settings.turn_pid_i,
                pid_d=settings.turn_pid_d,
                target_type=HeadingType.RELATIVE,
                tolerance=settings.heading_tolerance_deg,
            ),
            # 実行順6：旋回完了位置を確定してから、カメラ視野を広げる後退へ移る。
            StopNow(name="stop at sumo search position"),
            # 実行順7：左右コースにかかわらず、両輪が後退するPWMへ設定する。
            ConfigureCameraRetreatPwm(
                name="configure sumo camera retreat pwm",
                motor_command=camera_retreat_command,
                power=settings.camera_retreat_power,
            ),
            # 実行順8：回転せず50mm後退し、土俵全体と黒テープを画角へ入れやすくする。
            camera_retreat,
            # 実行順9：画像取得前に完全停止し、モーションブラーを抑える。
            StopNow(name="stop at sumo camera capture position"),
            # 実行順10：停止時の正面方位を、カメラ捕捉と死角進入後の基準として保存する。
            InitializeSumoState(name="initialize sumo camera capture", context=context),
        ]
    )
    return root
