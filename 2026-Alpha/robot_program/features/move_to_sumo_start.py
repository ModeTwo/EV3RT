"""Feature 15: move from the rally exit to the sumo search position."""

from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time

from ..behaviours.conditions import IsColorTransitionDetected, IsDistanceEarned
from ..behaviours.gyro_drive import RunByGyro, SpinAround
from ..behaviours.motor_control import StopNow


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
        state.skipped = False
        state.bottle_captured = False
        state.transport_completed = False
        state.failure_reason = None
        self.logger.info(
            "%+06d %s.search center reference heading=%.1f"
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
            IsColorTransitionDetected(
                name="detect black line exit into white area",
                from_color=Color.BLACK,
                to_color=Color.WHITE,
                from_duration_sec=settings.line_entry_black_duration_sec,
                to_duration_sec=settings.line_exit_white_duration_sec,
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
            # 実行順6：探索開始前に制動し、距離センサー測定中の慣性を抑える。
            StopNow(name="stop at sumo search position"),
            # 実行順7：停止時の正面方位を首振り探索の0度として保存する。
            InitializeSumoState(name="initialize sumo sonar search", context=context),
        ]
    )
    return root
