"""スタート～LAPで「何を走らせるか」を組み立てる。

角度表: start_lap_profile_v1.py / 補間: heading_profile.py
速度・PID: config.py / モーター制御: behaviours/gyro_drive.py
"""

from ..behaviours.conditions import IsColorDetected
from ..behaviours.gyro_drive import RunByGyro
from ..start_lap_calibration import calibrated_profile
from ..start_lap_profile_v1 import POINTS, BLUE_START_MM, LAP_GATE_MM
from ..types import HeadingType
from .bt_imports import Color

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
    )

    # 2. 後続工程があれば青で引渡し、LAP単体なら距離で終了する。
    follows_bottle = config.enable_bottle_delivery or config.mission_mode in ('hint2', 'hint2-return')
    if follows_bottle:
        finish_condition = IsColorDetected('lap blue marker', Color.BLUE)
        finish_check_from_mm = max(0.0, blue_start_mm - BLUE_SEARCH_BEFORE_MM)
        distance_limit_mm = lap_gate_mm + BLUE_MISS_MARGIN_MM
    else:
        finish_condition = None
        finish_check_from_mm = 0.0
        distance_limit_mm = lap_gate_mm + LAP_PASS_MARGIN_MM

    # 3. 走行命令はこの一つ。heading_atに括弧を付けず「関数」を渡す。
    #    RunByGyroが毎周期、heading_at(走行距離mm)を呼んで目標角を更新する。
    return RunByGyro(
        name='start_to_lap_gate',
        target=profile.heading_at,
        power=config.start_lap_power,
        pid_p=config.start_lap_pid_p,
        pid_i=config.start_lap_pid_i,
        pid_d=config.start_lap_pid_d,
        target_type=HeadingType.RELATIVE,
        distance_limit_mm=distance_limit_mm,
        completion_condition=finish_condition,
        completion_min_mm=finish_check_from_mm,
        feedforward_gain=config.start_lap_feedforward_gain,
        wheel_tread_mm=config.start_lap_wheel_tread_mm,
        cross_track_lookahead_mm=config.start_lap_cross_track_lookahead_mm,
        max_heading_correction_deg=config.start_lap_max_heading_correction_deg,
        profile_log_interval_sec=config.start_lap_log_interval_sec,
    )
