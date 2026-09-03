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
    skipped: bool = False
    bottle_captured: bool = False
    transport_completed: bool = False
    failure_reason: Optional[str] = None


@dataclass(frozen=True)
class SumoSettings:
    # 値はすべて暫定値。レプリカコースでの実験結果に応じてここだけを変更する。
    # 実機で静止摩擦に負けないよう、ET相撲の駆動出力は原則として絶対値40以上にする。
    # 青円から黒ラインへ入り、白地へ抜けたことを連続検出時間で判定する。
    line_entry_black_duration_sec: float = 0.5
    line_exit_white_duration_sec: float = 0.5
    # 白地を確認した後、ゲートから旋回半径分離れるために追加直進する。
    post_line_clearance_distance_mm: float = 100.0
    # SpinAroundがcourseを適用するため、同じ正角度でLeftは左、Rightは右へ旋回する。
    # 黒ライン終端から各コースの土俵側を向く基準値。
    ring_turn_deg: float = 90.0
    navigation_power: int = 50
    approach_power: int = 40
    carry_power: int = 40

    # 距離センサー探索は、正面から左右へこの範囲を段階的に首振りする。
    scan_half_angle_deg: float = 60.0
    scan_step_deg: float = 10.0
    # 10度程度の短い旋回は静止摩擦に負けやすいため、通常旋回とは別に高めの出力を使う。
    scan_step_turn_min_power: int = 55
    scan_step_turn_max_power: int = 65
    sonar_samples_per_angle: int = 3
    sonar_max_attempts_per_angle: int = 6
    sonar_settle_time_sec: float = 0.10
    sonar_sample_interval_sec: float = 0.10
    sonar_mm_per_unit: float = 1.0
    sonar_min_distance_mm: float = 50.0
    sonar_max_distance_mm: float = 800.0

    # 1回目の全角度探索で未検出の場合、探索中心へ戻って低速前進し、もう一度だけ探索する。
    retry_advance_distance_mm: float = 100.0
    retry_advance_power: int = 40

    # 距離センサーとボトルの間がこの距離になった時、下端アーム内へ捕捉できる想定。
    # センサー取付位置とアーム保持深さに合わせ、実機で必ず調整する。
    bottle_front_target_distance_mm: float = 120.0

    # 捕捉後はその場旋回せず、緩い円弧で出口方向へ向けて黒ラインまで運搬する。
    # Leftの基準値を示し、RightではFeature側が左右PWMを鏡像化する。
    # Leftは左カーブ、Rightはその鏡像となる右カーブで出口側へ運搬する。
    carry_curve_left_pwm: int = 40
    carry_curve_right_pwm: int = 55
    carry_curve_distance_mm: float = 120.0

    turn_min_power: int = 47
    turn_max_power: int = 57
    turn_pid_p: float = 0.4
    turn_pid_i: float = 0.001
    turn_pid_d: float = 0.03
    drive_pid_p: float = 1.1
    drive_pid_i: float = 0.1
    drive_pid_d: float = 0.03
    heading_tolerance_deg: float = 3.0
