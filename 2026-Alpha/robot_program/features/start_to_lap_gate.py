"""スタート～LAPで「何を走らせるか」を組み立てる。

角度表: start_lap_profile_v1.py / 補間: heading_profile.py
速度・PID: config.py / モーター制御: behaviours/gyro_drive.py
"""

from ..behaviours.conditions import IsColorDetected, IsDistanceEarned
from ..behaviours.gyro_drive import RunByGyro
from ..behaviours.line_trace import TraceLine
from ..start_lap_calibration import calibrated_profile
from ..start_lap_profile_v1 import POINTS, BLUE_START_MM, LAP_GATE_MM
from ..types import HeadingType
from .bt_imports import Color, Parallel, ParallelPolicy, Sequence, TraceSide

# 終了位置の調整値（mm）。角度追従の調整でも終了位置は変更しない。
LAP_PASS_MARGIN_MM = 20.0       # LAP単体: ゲートの20mm先で停止
BLUE_SEARCH_BEFORE_MM = 250.0  # 青予測位置の250mm手前から検知
BLUE_MISS_MARGIN_MM = 100.0    # 青を見逃した場合: ゲートの100mm先で停止


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
        later_turn_distance_scale=config.start_lap_later_turn_distance_scale,
        third_turn_distance_scale=config.start_lap_third_turn_distance_scale,
        second_turn_start_advance_mm=config.start_lap_second_turn_start_advance_mm,
        third_turn_start_delay_mm=config.start_lap_third_turn_start_delay_mm,
    )

    # 2. 黒線中心から作った方位角表で最終直線の途中まで走る。
    gyro_approach = RunByGyro(
        name='start_to_line_trace',
        target=profile.heading_at,
        power=config.start_lap_power,
        pid_p=config.start_lap_pid_p,
        pid_i=config.start_lap_pid_i,
        pid_d=config.start_lap_pid_d,
        target_type=HeadingType.ABSOLUTE,
        distance_limit_mm=config.start_lap_line_trace_from_mm,
        feedforward_gain=config.start_lap_feedforward_gain,
        wheel_tread_mm=config.start_lap_wheel_tread_mm,
        cross_track_lookahead_mm=config.start_lap_cross_track_lookahead_mm,
        max_heading_correction_deg=config.start_lap_max_heading_correction_deg,
        profile_log_interval_sec=config.start_lap_log_interval_sec,
    )

    line_trace = TraceLine(
        name='trace_line_to_lap_gate',
        target=config.start_lap_line_target_v,
        power=config.start_lap_line_power,
        pid_p=config.start_lap_line_pid_p,
        pid_i=config.start_lap_line_pid_i,
        pid_d=config.start_lap_line_pid_d,
        trace_side=TraceSide.NORMAL,
        cutoff_hz=None,
    )

    # 3. 最終直線の途中から黒線へ収束し、青または残距離で終了する。
    follows_bottle = config.enable_bottle_delivery or config.mission_mode in ('hint2', 'hint2-return')
    traced_run = Parallel(name='line_trace_until_lap', policy=ParallelPolicy.SuccessOnOne())
    if follows_bottle:
        traced_run.add_children([
            line_trace,
            IsColorDetected('lap blue marker', Color.BLUE),
        ])
    else:
        traced_run.add_children([
            line_trace,
            IsDistanceEarned(
                'pass lap gate by distance',
                lap_gate_mm + LAP_PASS_MARGIN_MM - config.start_lap_line_trace_from_mm,
            ),
        ])

    root = Sequence(name='start_to_lap_gate', memory=True)
    root.add_children([gyro_approach, traced_run])
    return root
