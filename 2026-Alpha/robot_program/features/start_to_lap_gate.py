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
from ..start_lap_calibration import calibrated_profile
from ..start_lap_profile_v1 import POINTS, BLUE_START_MM, LAP_GATE_MM
from ..types import HeadingType
from .bt_imports import Color, TraceSide

# 終了位置の調整値（mm）。角度追従の調整でも終了位置は変更しない。
LAP_PASS_MARGIN_MM = 20.0       # LAP単体: ゲートの20mm先で停止


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

        run_to_line_trace = RunByGyro(
            name='start_to_lap_gate gyro section',
            target=profile.heading_at,
            power=config.start_lap_power,
            pid_p=config.start_lap_pid_p,
            pid_i=config.start_lap_pid_i,
            pid_d=config.start_lap_pid_d,
            target_type=HeadingType.RELATIVE,
            distance_limit_mm=line_trace_start_mm,
            feedforward_gain=config.start_lap_feedforward_gain,
            wheel_tread_mm=config.start_lap_wheel_tread_mm,
            cross_track_lookahead_mm=config.start_lap_cross_track_lookahead_mm,
            max_heading_correction_deg=config.start_lap_max_heading_correction_deg,
            profile_log_interval_sec=config.start_lap_log_interval_sec,
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

        root = Sequence(name='start_to_lap_gate', memory=True)
        safe_blue_search = Timeout(
            name='lap blue marker emergency timeout',
            child=recover_and_watch_blue,
            duration=config.start_lap_blue_timeout_sec,
        )
        root.add_children([run_to_line_trace, safe_blue_search])
        return root
    else:
        distance_limit_mm = lap_gate_mm + LAP_PASS_MARGIN_MM

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
