"""Shared ET sumo state and adjustable settings."""

from dataclasses import dataclass, field
from typing import List, Optional


@dataclass(frozen=True)
class SumoSonarSample:
    # 探索開始時の正面を0度とするコース正規化角度。
    # 正角度はLeftでは物理的な左、Rightでは物理的な右（各コースの外側）を表す。
    angle_offset_deg: float
    distance_mm: float


@dataclass
class SumoState:
    # No.15からNo.18までの間だけ共有する、ET相撲固有の実行状態。
    started_at: Optional[float] = None
    search_heading_deg: float = 0.0
    sonar_samples: List[SumoSonarSample] = field(default_factory=list)
    bottle_bearing_deg: Optional[float] = None
    bottle_distance_mm: Optional[float] = None
    approach_distance_mm: float = 0.0
    camera_capture_heading_deg: Optional[float] = None
    skipped: bool = False
    bottle_captured: bool = False
    bottle_pushed_out: bool = False
    bottle_released: bool = False
    transport_completed: bool = False
    bottle_held_at_exit: bool = False
    line_trace_ready: bool = False
    failure_reason: Optional[str] = None


@dataclass(frozen=True)
class SumoSettings:
    # 値はすべて暫定値。レプリカコースでの実験結果に応じてここだけを変更する。
    # 実機で静止摩擦に負けないよう、ET相撲の駆動出力は絶対値50以上にする。
    # 黒は1回の確定判定で通過扱いとし、白地への退出だけを連続検出時間で判定する。
    line_entry_black_duration_sec: float = 0.0
    line_exit_white_duration_sec: float = 0.5
    # ET相撲開始位置では、共通色分類ではなく生の明度で黒線退出を判定する。
    line_black_max_value: int = 45
    line_white_min_value: int = 65
    line_sensor_log_interval_sec: float = 0.25
    # 白地を確認した後、ゲートから旋回半径分離れるために追加直進する。
    post_line_clearance_distance_mm: float = 75.0
    # SpinAroundがcourseを適用するため、同じ正角度でLeftは左、Rightは右へ旋回する。
    # 黒ライン終端から各コースの土俵側を向く基準値。
    ring_turn_deg: float = 90.0
    navigation_power: int = 50
    approach_power: int = 50
    carry_power: int = 50

    # 土俵方向へ90度旋回した直後、カメラ視野を広げるため素早く後退する。
    camera_retreat_distance_mm: float = 80.0
    camera_retreat_power: int = 80

    # 力士ボトルは黒テープだけを対象とし、連続した新規フレームで確定する。
    camera_min_area_px: int = 150
    camera_confirm_frames: int = 3
    camera_detection_timeout_sec: float = 3.0
    camera_approach_timeout_sec: float = 8.0

    # 画像中心へ寄せながら前進する。左右輪とも絶対値50未満にしない。
    camera_approach_power: int = 75
    camera_min_wheel_power: int = 50
    camera_max_wheel_power: int = 100
    camera_steer_gain: float = 2.0
    camera_max_steer_power: int = 25
    camera_drive_log_interval_sec: float = 0.25
    camera_lost_frame_limit: int = 8
    camera_near_bottom_row: int = 130

    # 黒テープがカメラ死角へ入った後、最後の方位を維持してアーム内へ押し込む。
    # 実機のカメラ位置、アーム保持深さに合わせて調整する。
    camera_blind_capture_distance_mm: float = 150.0

    # 距離センサー探索は、正面から左右へこの範囲を段階的に首振りする。
    scan_half_angle_deg: float = 60.0
    scan_step_deg: float = 10.0
    # Falseにすれば従来の10度刻み停止探索へ戻せる。
    continuous_scan_enabled: bool = True
    continuous_sample_interval_sec: float = 0.02
    continuous_scan_timeout_sec: float = 8.0
    continuous_scan_power: int = 50
    # 10度程度の短い旋回は静止摩擦に負けやすいため、通常旋回とは別に高めの出力を使う。
    scan_step_turn_min_power: int = 60
    scan_step_turn_max_power: int = 75
    sonar_samples_per_angle: int = 3
    sonar_max_attempts_per_angle: int = 6
    sonar_settle_time_sec: float = 0.10
    sonar_sample_interval_sec: float = 0.10
    sonar_mm_per_unit: float = 1.0
    sonar_min_distance_mm: float = 50.0
    sonar_max_distance_mm: float = 800.0

    # 未検出のたびに探索中心へ戻り、100、70、50mmと段階的に近づいて再探索する。
    retry_advance_distances_mm: tuple = (100.0, 70.0, 50.0)
    retry_advance_power: int = 50

    # 距離センサーとボトルの間がこの距離になった時、下端アーム内へ捕捉できる想定。
    # センサー取付位置とアーム保持深さに合わせ、実機で必ず調整する。
    bottle_front_target_distance_mm: float = 120.0

    # 捕捉後はボトルを保持したまま押出し側黒ラインまで直進する。
    push_out_drive_power: int = 60
    # 黒ライン検知後も30mm進み、ボトルが境界を越える余裕を確保する。
    push_out_after_line_distance_mm: float = 30.0
    # 直線後退でアームの保持深さ、ボトル直径、安全余裕をまとめて確保する。
    release_reverse_distance_mm: float = 150.0
    release_reverse_power: int = 60

    # 離脱後は緩い後退カーブでガレージ側へ寄せ、復帰用黒ラインを探す。
    # Left用の論理PWMを示し、RightではFeature側で鏡像化する。
    garage_return_reverse_left_pwm: int = 50
    garage_return_reverse_right_pwm: int = 60
    # 黒ラインへ乗った後、短距離ライントレースして姿勢を安定させる。
    line_rejoin_trace_power: int = 50
    line_rejoin_trace_distance_mm: float = 100.0
    line_rejoin_trace_target_v: int = 75

    turn_min_power: int = 60
    turn_max_power: int = 75
    turn_pid_p: float = 0.4
    turn_pid_i: float = 0.001
    turn_pid_d: float = 0.03
    drive_pid_p: float = 1.1
    drive_pid_i: float = 0.1
    drive_pid_d: float = 0.03
    heading_tolerance_deg: float = 3.0
