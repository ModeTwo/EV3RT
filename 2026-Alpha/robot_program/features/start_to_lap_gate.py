"""スタート～LAPで「何を走らせるか」を組み立てる。

角度表: start_lap_profile_v1.py / 補間: heading_profile.py
速度・PID: config.py / モーター制御: behaviours/gyro_drive.py
"""

from py_trees.common import ParallelPolicy
from py_trees.composites import Parallel, Sequence
from py_trees.decorators import Timeout

from ..behaviours.conditions import IsColorDetected
from ..behaviours.camera_line_trace import RecoverLineByCamera
from ..behaviours.gyro_drive import RunByGyro
from ..behaviours.line_trace import TraceLine
from ..behaviours.section_motion import DriveDistance
from ..start_lap_calibration import calibrated_profile
from ..start_lap_profile_v1 import POINTS, BLUE_START_MM, LAP_GATE_MM
from ..types import HeadingType
from .bt_imports import Color, TraceSide
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from typing import Callable, Union

from ..runtime import runtime

# 終了位置の調整値（mm）。角度追従の調整でも終了位置は変更しない。
LAP_PASS_MARGIN_MM = 0.0       # LAP単体: ゲートの20mm先で停止

#カーブで速度を落とすために、追加
def speed_profile(distance_mm: float, config) -> int:
    # カーブ区間のリスト（mm）
    curve_sections = [
        (408, 826),
        (1130, 1536),
        (1676, 2187),
        (3058, 4235),
    ]

    # どれかのカーブ区間に入っていたら速度を落とす
    for start, end in curve_sections:
        if start <= distance_mm <= end:
            return 70   # カーブの速度（共通）

    return config.start_lap_power            # 直線の速度 75?80?（config.start_lap_power と同じ）


#カーブでスピードを落とすためにRunByGyroを新しく定義
class RunByGyroSpeedDownatCurve(RunByGyro):
    def __init__(
        self,
        name: str,
        target: Union[float, Callable[[float], float]],
        power: int,
        pid_p: float,
        pid_i: float,
        pid_d: float,
        target_type: HeadingType,
        *,
        distance_limit_mm: float = None,
        completion_condition: Behaviour = None,
        completion_min_mm: float = 0.0,
        feedforward_gain: float = 0.0,
        wheel_tread_mm: float = 110.0,
        cross_track_lookahead_mm: float = 0.0,
        max_heading_correction_deg: float = 8.0,
        profile_log_interval_sec: float = 1.0,
        config=None,   # ★ 追加
    ):
        # ★ super() には RunByGyro と同じ引数だけ渡す
        super().__init__(
            name,
            target,
            power,
            pid_p,
            pid_i,
            pid_d,
            target_type,
            distance_limit_mm=distance_limit_mm,
            completion_condition=completion_condition,
            completion_min_mm=completion_min_mm,
            feedforward_gain=feedforward_gain,
            wheel_tread_mm=wheel_tread_mm,
            cross_track_lookahead_mm=cross_track_lookahead_mm,
            max_heading_correction_deg=max_heading_correction_deg,
            profile_log_interval_sec=profile_log_interval_sec,
        )

        # ★ config を自分で保持する
        self.config = config

    def update(self) -> Status:
        if self.distance_target:
            return self._update_distance_target_speeddown()
        else:
            return super().update()

    def _update_distance_target_speeddown(self) -> Status:
        status = super()._update_distance_target()

        progress = runtime.plotter.get_distance() - self.origin_distance

        # ★ None ではなく self.config が入るようになる
        self.power = speed_profile(progress, self.config)

        return status


def build_start_to_lap_gate(context, config):
    # 旧方式は別ファイルへ保存。通常読む必要はない。
    if config.start_lap_mode == 'legacy':
        from .start_to_lap_gate_legacy import build_legacy_start_to_lap_gate
        return build_legacy_start_to_lap_gate(context, config)
    if config.start_lap_mode != 'profile':
        raise ValueError('start_lap_mode must be profile or legacy')

    # 1. 距離を渡すと目標角を返す関数を用意する。
    profile, blue_start_mm, lap_gate_mm = calibrated_profile(
        POINTS, BLUE_START_MM, LAP_GATE_MM,
        first_straight_mm=config.start_lap_first_straight_mm,
        route_scale=config.start_lap_route_scale,
    )

    # 2. 後続工程があれば、青検知開始位置で方位角走行からライン追従へ渡す。
    follows_bottle = config.enable_bottle_delivery or config.mission_mode in ('hint2', 'hint2-return')
    if follows_bottle:
        line_trace_start_mm = max(
            0.0, blue_start_mm - config.start_lap_camera_before_blue_mm
        )

        run_to_line_trace = RunByGyroSpeedDownatCurve(
            name='start_to_lap_gate gyro section',
            target=profile.heading_at,
            power=config.start_lap_power,
            pid_p=config.start_lap_pid_p,
            pid_i=config.start_lap_pid_i,
            pid_d=config.start_lap_pid_d,
            target_type=HeadingType.RELATIVE,
            distance_limit_mm= 5260.540, #line_trace_start_mm,
            feedforward_gain=config.start_lap_feedforward_gain,
            wheel_tread_mm=config.start_lap_wheel_tread_mm,
            cross_track_lookahead_mm=config.start_lap_cross_track_lookahead_mm,
            max_heading_correction_deg=config.start_lap_max_heading_correction_deg,
            profile_log_interval_sec=config.start_lap_log_interval_sec,

            config=config
        )

        camera_recovery = RecoverLineByCamera(
            name='recover line by camera before lap',
            power=config.start_lap_camera_power,
            pid_p=config.start_lap_camera_pid_p,
            pid_i=config.start_lap_camera_pid_i,
            pid_d=config.start_lap_camera_pid_d,
            max_camera_turn=config.start_lap_camera_max_turn,
            align_power=config.start_lap_camera_align_power,
            handoff_power=config.start_lap_camera_handoff_power,
            handoff_target_v=config.start_lap_line_target_v,
            handoff_pid_p=config.start_lap_camera_handoff_pid_p,
            handoff_turn_cap=config.start_lap_camera_handoff_turn_cap,
            handoff_v_tolerance=config.start_lap_camera_handoff_v_tolerance,
            handoff_stable_samples=config.start_lap_camera_handoff_stable_samples,
            line_v=config.start_lap_camera_rejoin_v,
            line_samples=config.start_lap_camera_rejoin_samples,
            trace_side=TraceSide.NORMAL,
            tilt_ff_gain=config.start_lap_camera_tilt_ff_gain,
            ff_cap=config.start_lap_camera_ff_cap,
            heading_tolerance_deg=config.start_lap_heading_tolerance_deg,
            stable_samples=config.start_lap_camera_stable_samples,
            gyro_heading_deg=0.0,
            gyro_kp=config.start_lap_camera_gyro_kp,
            gyro_turn_cap=config.start_lap_camera_gyro_turn_cap,
        )

        drive_after_gyro = Sequence(
            name='camera recovery then color line trace', memory=True
        )
        drive_after_gyro.add_children([
            camera_recovery,
            TraceLine(
                name='recover and trace line before lap',
                target=config.start_lap_line_target_v,
                power=config.start_lap_line_power,
                pid_p=config.start_lap_line_pid_p,
                pid_i=config.start_lap_line_pid_i,
                pid_d=config.start_lap_line_pid_d,
                trace_side=TraceSide.NORMAL,
            ),
        ])

        # 青はカメラ復帰中から監視し、検知した周期でATへ渡す。
        # 青から293mmの0度走行を始めるため、ここでは方位安定を待たない。
        recover_and_watch_blue = Parallel(
            name='recover line and watch lap blue marker',
            policy=ParallelPolicy.SuccessOnOne(),
        )
        recover_and_watch_blue.add_children([
            drive_after_gyro,
            IsColorDetected('lap blue marker', Color.BLUE),
        ])

        # 旧来の経路（カメラ復帰で黒線に乗り、青検知まで見張る）。
        # 現在は下のroot.add_childrenから外しているだけなので、戻すときは
        # safe_blue_searchをroot.add_childrenの末尾へ差し戻せばよい。
        safe_blue_search = Timeout(
            name='lap blue marker emergency timeout',
            child=recover_and_watch_blue,
            duration=config.start_lap_blue_timeout_sec,
        )

        # 【統合差分】相撲開始位置への後退→旋回→前進→旋回の補正は
        # move_to_sumo_start.py(No.15)側の責務へ移した。LAPゲート終了直後の
        # 実際の停止位置が、相撲側が前提とする初期位置からズレている、という
        # 目的の処理のため、「相撲位置へ移動する」機能として相撲側に置く。
        root = Sequence(name='start_to_lap_gate', memory=True)
        root.add_children([
            run_to_line_trace,
        ])

        return root
    else:
        distance_limit_mm = 5160.540 #lap_gate_mm + LAP_PASS_MARGIN_MM

    # 3. devRE完成版と同じく、LAPまで一つのRunByGyroで走る。
    #    heading_atに括弧を付けず、距離から目標方位を得る関数として渡す。
    return RunByGyro(
        name='start_to_lap_gate',
        target=profile.heading_at,
        power=config.start_lap_power,
        pid_p=config.start_lap_pid_p,
        pid_i=config.start_lap_pid_i,
        pid_d=config.start_lap_pid_d,
        target_type=HeadingType.RELATIVE,
        distance_limit_mm=distance_limit_mm,
        feedforward_gain=config.start_lap_feedforward_gain,
        wheel_tread_mm=config.start_lap_wheel_tread_mm,
        cross_track_lookahead_mm=config.start_lap_cross_track_lookahead_mm,
        max_heading_correction_deg=config.start_lap_max_heading_correction_deg,
        profile_log_interval_sec=config.start_lap_log_interval_sec,
    )