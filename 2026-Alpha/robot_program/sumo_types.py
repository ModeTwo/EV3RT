"""Shared ET sumo state and adjustable settings."""

from dataclasses import dataclass, field
from typing import List, Optional

from .features.sumo_bearing import SumoBearingReference


@dataclass(frozen=True)
class SumoSonarSample:
    # 探索開始時の正面を0度とするコース正規化角度。
    # 正角度はLeftでは物理的な左、Rightでは物理的な右（各コースの外側）を表す。
    angle_offset_deg: float
    distance_mm: float


@dataclass
class SumoState:
    # 全体または単体試験のResetDevice直後に登録する方位角とジャイロの対応。
    # 各実行の状態を共有しないよう、必ず個別インスタンスを生成する。
    bearing_reference: SumoBearingReference = field(default_factory=SumoBearingReference)
    search_bearing_deg: float = 0.0
    camera_capture_bearing_deg: Optional[float] = None
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
    garage_line_found: bool = False
    line_trace_ready: bool = False
    failure_reason: Optional[str] = None
    bottle_image_x_ratio: Optional[float] = None
    escape_route: Optional[int] = None


@dataclass(frozen=True)
class SumoSettings:
    # Falseで直前の120度境界・±50度復帰へ戻せる。
    garage_point_return_enabled: bool = True
    # 復帰計算距離を超えて黒ラインを探す追加距離。実機で調整する暫定値。
    garage_search_margin_mm: float = 50.0
    # 開始位置を原点、復帰ライン側X、初期前方Yとしたコース正規化座標。
    garage_line_offset_mm: float = 680.0
    garage_blue_forward_mm: float = 280.0
    garage_rejoin_before_blue_mm: float = 100.0
    # コース図の上0、右90、下180、左270。時計回りを正とする。
    entry_bearing_deg: float = 0.0
    # RightではNo.15が鏡像の90度へ変換する。
    ring_bearing_left_deg: float = 270.0
    garage_bearing_deg: float = 180.0
    # 現在方位±50度から180度と角度差が大きい候補を選ぶ。
    garage_search_offset_deg: float = 50.0
    # 値はすべて暫定値。レプリカコースでの実験結果に応じてここだけを変更する。
    # 実機で静止摩擦に負けないよう、ET相撲の駆動出力は絶対値50以上にする。
    # No.15開始位置からの総直進距離。白検知や追加クリアランスは使用しない。
    start_straight_distance_mm: float = 350.0
    # 以下の黒→白判定設定は旧方式の比較用。現行No.15の終了条件には使用しない。
    line_exit_white_duration_sec: float = 0.5
    # ET相撲開始位置では、共通色分類ではなく生の明度で黒線退出を判定する。
    line_black_max_value: int = 45
    line_white_min_value: int = 65
    line_sensor_log_interval_sec: float = 0.25
    navigation_power: int = 80
    # 旋回後の惰性を待ち、新規画像で正面への整列を確認する。
    camera_alignment_settle_sec: float = 0.2
    camera_alignment_tolerance_deg: float = 5.0
    approach_power: int = 60
    carry_power: int = 60

    # 土俵方向へ90度旋回した直後、カメラ視野を広げるため素早く後退する。
    camera_retreat_distance_mm: float = 60.0
    camera_retreat_power: int = 80

    # 力士ボトルは黒テープだけを対象とし、連続した新規フレームで確定する。
    camera_min_area_px: int = 150
    camera_confirm_frames: int = 3

    # 画像中心へ寄せながら前進する。左右輪とも絶対値50未満にしない。
    camera_approach_power: int = 75
    camera_min_wheel_power: int = 50
    camera_max_wheel_power: int = 100
    camera_steer_gain: float = 2.0
    camera_max_steer_power: int = 25
    # 画像でボトル方位を確定した地点から、キャッチ・押し出し込みで進む総距離。
    capture_and_push_distance_mm: float = 500.0

    # 距離センサー探索は、正面から左右へこの範囲を段階的に首振りする。
    scan_half_angle_deg: float = 60.0
    scan_step_deg: float = 10.0
    # Falseにすれば従来の10度刻み停止探索へ戻せる。
    continuous_scan_enabled: bool = True
    continuous_sample_interval_sec: float = 0.02
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

    # 捕捉完了位置からこの距離を直進し、色判定に依存せずボトルを押し出す。
    push_out_drive_power: int = 60
    push_out_distance_mm: float = 150.0  # 旧方式用。現行No.18では追加直進しない。
    # 直線後退でアームの保持深さ、ボトル直径、安全余裕をまとめて確保する。
    release_reverse_distance_mm: float = 100.0
    release_reverse_power: int = 60
    # ボトル離脱後、黒ライン探索へ入る前に横方向へ退避する。
    garage_avoid_distance_mm: float = 120.0
    garage_avoid_power: int = 50
    # ボトルを押し出した方向から、退避ルート②へ向く角度。
    # Left/Rightの反転はFeature18側でruntime.courseを使って行う。
    escape_route_2_angle_deg: float = 45.0
    # 退避ルート②で斜め方向へ走行する距離。
    escape_route_2_distance_mm: float = 200.0
    # 退避ルート②で走行するときのPWM。
    escape_route_2_power: int = 50 
    # 離脱後はガレージ側へ旋回してから、その絶対方位を維持して黒ラインまで直進する。
    garage_return_drive_power: int = 60
    # 復帰用黒ラインは生の明度で判定し、未検出時は規定距離で安全停止する。
    garage_line_black_max_value: int = 45
    garage_line_confirm_samples: int = 2
    garage_line_sensor_log_interval_sec: float = 0.25
    garage_line_search_max_distance_mm: float = 500.0
    # 黒検知位置から少し乗り込んでから180度へ向く。旋回時のセンサー位置を実機で調整する。
    garage_line_entry_distance_mm: float = 30.0
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
