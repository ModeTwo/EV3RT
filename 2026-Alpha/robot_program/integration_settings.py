"""Non-B AT/TO route settings; distances are connected to feature builders."""

from dataclasses import dataclass
from typing import Optional
import math


@dataclass(frozen=True)
class IntegrationSettings:
    # TOは2026-09-16受領tantou4.py.txtを選択的に反映（旧同名B版とは区別）。
    re_source: str = 'gyro_line_0826.py'
    at_source: str = 'bottle_catch2.py'
    to_source: str = 'tantou4.py.txt (2026-09-16 received)'
    # 旧停止認識方式の互換設定（移動中認識では未使用）。
    at_gate_forward_mm: float = 100.0
    at_recognition_reverse_mm: float = 200.0
    # REの青検知後、色認識を並行して行うカラートレース距離。
    at_to_transfer_trace_mm: float = 400.0
    at_marker_straight_mm: float = 293.0  # 青検知からグレー丸出口まで絶対0度
    # TOが引渡し後に黒線へ接近する区間の距離上限。受領tantou4準拠。
    to_first_black_limit_mm: float = 550.0
    # ヒント1後は青検知を待たず、緑を通り抜けてから固定距離だけ直進する。
    # 緑検知は近距離で起きるはずのため、検知できない場合は320mmで打ち切って
    # 次工程(90度旋回→カメラトレース)へ進む(実機未校正)。
    to_after_hint1_green_pass_mm: float = 320.0
    to_after_hint1_safety_limit_mm: float = 320.0
    to_hint2_trace_mm: float = 1250.0  # 受領値。終了判定は現行の投影距離を維持
    # 左90度旋回後、色センサーtrace_120へ渡す前にカメラでライン中央へ寄せる。
    # LAP前のRecoverLineByCamera(config.start_lap_camera_*)と同じ値を初期値として流用。実機未校正。
    to_after_hint1_camera_power: int = 50
    to_after_hint1_camera_pid_p: float = 2.0
    to_after_hint1_camera_pid_i: float = 0.0
    to_after_hint1_camera_pid_d: float = 0.06
    to_after_hint1_camera_max_turn: int = 30
    to_after_hint1_camera_align_power: int = 35
    to_after_hint1_camera_handoff_power: int = 35
    to_after_hint1_camera_handoff_pid_p: float = 0.3
    to_after_hint1_camera_handoff_turn_cap: float = 10.0
    to_after_hint1_camera_handoff_v_tolerance: int = 10
    to_after_hint1_camera_handoff_stable_samples: int = 5
    to_after_hint1_camera_tilt_ff_gain: float = 8.0
    to_after_hint1_camera_ff_cap: float = 8.0
    to_after_hint1_camera_heading_tolerance_deg: float = 5.0
    to_after_hint1_camera_stable_samples: int = 3
    to_after_hint1_camera_gyro_kp: float = 0.8
    to_after_hint1_camera_gyro_turn_cap: float = 25.0
    to_after_hint1_camera_rejoin_v: int = 65
    to_after_hint1_camera_rejoin_samples: int = 3
    # SEEKがカラーセンサーの黒検知だけに依存すると、カメラが中心と見ている
    # 対象と実際の色センサー位置がずれた場合に1200mm予算を使い切るまで
    # 滞留する(実機で複数回確認)。theta安定、または距離上限でもALIGNへ
    # 進めるフォールバックを追加する。実機未校正の試走初期値。
    to_after_hint1_camera_seek_theta_tolerance_deg: float = 5.0
    to_after_hint1_camera_seek_stable_samples: int = 8
    to_after_hint1_camera_seek_distance_limit_mm: float = 450.0
    to_exit_trace_mm: float = 600.0  # Hint2後の白探索上限（到達は失敗）
    # WHITEフェーズの直進距離を、生の走行距離ではなく方位角90度方向への
    # 投影距離で制約する(to_hint_route.pyのdist_1200_from_75deg_startと
    # 同じ考え方)。実機未校正の試走初期値。
    to_exit_white_straight_projected_limit_mm: float = 210.0
    # Hint2出口のみ。実機未校正の試走初期値。距離は保持ボトルの占有範囲で調整。
    to_exit_power: int = 50
    to_exit_turn_power: int = 25  # 内輪も前進。基準出力未満を維持。
    to_exit_pivot_min_power: int = 60
    to_exit_pivot_max_power: int = 60
    to_exit_pivot_settle_s: float = 0.15  # 前進停止後にその場旋回を開始。
    to_exit_slew_power_per_s: float = 120.0
    to_exit_heading_kp: float = 0.6
    to_exit_line_kp: float = 0.55
    to_exit_white_v: float = 85.0
    to_exit_black_v: float = 65.0
    to_exit_white_min_mm: float = 30.0
    to_exit_detect_cycles: int = 3
    to_exit_lost_cycles: int = 15
    to_exit_offset_mm: float = 0.0  # 白の連続検出後。必要距離を実測して増やす。
    to_exit_target_heading_deg: float = 190.0  # TO基準180度を10度越えて探索。
    to_exit_turn_limit_mm: float = 500.0
    to_exit_search_limit_mm: float = 200.0
    to_exit_follow_mm: float = 80.0
    to_exit_phase_timeout_s: float = 20.0
    to_exit_heading_tolerance_deg: float = 5.0
    # TURN序盤の誤検出を避けるため、目標方位までの残り誤差がこの範囲内に入るまで黒検出を数えない。実機未校正の試走初期値。
    to_exit_turn_black_detect_max_error_deg: float = 60.0
    to_spin_min_power: int = 60
    to_spin_max_power: int = 60
    # Hint2後の移動完了位置から、黄→青→赤のドロップゾーン前を進む。
    delivery_trace_target_v: int = 75
    delivery_trace_power: int = 50
    # 青ラインの手前端から抜ける距離と、中央から抜ける距離を分けて調整する。
    delivery_marker_full_width_mm: float = 120.0
    delivery_marker_half_width_mm: float = 60.0
    delivery_drop_distance_first_mm: float = 100.0
    delivery_drive_first_power: int = 50
    delivery_drop_distance_second_mm: float = 100.0
    delivery_drive_second_power: int = 50
    # ライン進行方向からドロップゾーン側へ向く角度。ラリー内側とは反対側。
    delivery_drop_turn_deg_first: float = -30.0
    delivery_drop_turn_deg_second: float = -90.0
    delivery_inward_turn_deg: float = 90.0

    def __post_init__(self):
        if not math.isfinite(self.at_marker_straight_mm) or not 0 < self.at_marker_straight_mm < self.at_to_transfer_trace_mm:
            raise ValueError("Require 0 < AT marker straight distance < AT total distance")
        if (type(self.to_exit_pivot_min_power) is not int or
                type(self.to_exit_pivot_max_power) is not int or
                not 0 < self.to_exit_pivot_min_power <= self.to_exit_pivot_max_power <= 100):
            raise ValueError('Require 0 < pivot min <= max <= 100')
        if not math.isfinite(self.to_exit_pivot_settle_s) or not 0 <= self.to_exit_pivot_settle_s < self.to_exit_phase_timeout_s:
            raise ValueError('Pivot settle time must be nonnegative and below phase timeout')
        for name in ('to_exit_slew_power_per_s', 'to_exit_heading_kp',
                     'to_exit_line_kp', 'to_exit_turn_limit_mm', 'to_exit_search_limit_mm',
                     'to_exit_follow_mm', 'to_exit_phase_timeout_s', 'to_exit_heading_tolerance_deg',
                     'to_exit_turn_black_detect_max_error_deg', 'to_exit_white_straight_projected_limit_mm'):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(name + ' must be positive and finite')
        if not self.to_exit_heading_tolerance_deg <= self.to_exit_turn_black_detect_max_error_deg:
            raise ValueError('Black-detect gate must not be tighter than the heading tolerance')
        for name in ('to_exit_offset_mm', 'to_exit_white_min_mm'):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError(name + ' must be nonnegative and finite')
        if not math.isfinite(self.to_exit_target_heading_deg) or not 180 <= self.to_exit_target_heading_deg <= 210:
            raise ValueError('Exit TO heading must be in 180..210 degrees')
        if not 0 <= self.to_exit_black_v < self.delivery_trace_target_v < self.to_exit_white_v <= 100:
            raise ValueError('Require black < line target < white in 0..100')
        if not self.to_exit_white_min_mm < self.to_exit_trace_mm:
            raise ValueError('White detection must begin before distance limit')
        for name in ('to_exit_detect_cycles', 'to_exit_lost_cycles',
                     'to_exit_power', 'to_exit_turn_power'):
            if type(getattr(self, name)) is not int or getattr(self, name) <= 0:
                raise ValueError(name + ' must be a positive integer')
        if not self.to_exit_turn_power < self.to_exit_power or self.to_exit_power + self.to_exit_turn_power > 100:
            raise ValueError('Exit outputs must stay forward and within 100')
        for name in ('to_after_hint1_camera_power', 'to_after_hint1_camera_max_turn',
                     'to_after_hint1_camera_align_power', 'to_after_hint1_camera_handoff_power',
                     'to_after_hint1_camera_handoff_v_tolerance', 'to_after_hint1_camera_handoff_stable_samples',
                     'to_after_hint1_camera_stable_samples', 'to_after_hint1_camera_rejoin_v',
                     'to_after_hint1_camera_rejoin_samples', 'to_after_hint1_camera_seek_stable_samples'):
            if type(getattr(self, name)) is not int or getattr(self, name) <= 0:
                raise ValueError(name + ' must be a positive integer')
        for name in ('to_after_hint1_camera_pid_p', 'to_after_hint1_camera_pid_d',
                     'to_after_hint1_camera_handoff_pid_p', 'to_after_hint1_camera_handoff_turn_cap',
                     'to_after_hint1_camera_tilt_ff_gain', 'to_after_hint1_camera_ff_cap',
                     'to_after_hint1_camera_heading_tolerance_deg', 'to_after_hint1_camera_gyro_kp',
                     'to_after_hint1_camera_gyro_turn_cap', 'to_after_hint1_camera_seek_theta_tolerance_deg',
                     'to_after_hint1_camera_seek_distance_limit_mm'):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(name + ' must be positive and finite')
        if not math.isfinite(self.to_after_hint1_camera_pid_i) or self.to_after_hint1_camera_pid_i < 0:
            raise ValueError('to_after_hint1_camera_pid_i must be nonnegative and finite')
        for name in ('at_gate_forward_mm', 'at_recognition_reverse_mm',
                     'at_to_transfer_trace_mm', 'to_first_black_limit_mm',
                     'to_after_hint1_green_pass_mm', 'to_after_hint1_safety_limit_mm',
                     'to_hint2_trace_mm', 'to_exit_trace_mm',
                     'delivery_marker_full_width_mm', 'delivery_marker_half_width_mm',
                     'delivery_drop_distance_first_mm', 'delivery_drop_distance_second_mm'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be a positive finite distance')
        for name in ('delivery_trace_power', 'delivery_drive_first_power', 'delivery_drive_second_power'):
            value = getattr(self, name)
            if not isinstance(value, int) or not 1 <= value <= 100:
                raise ValueError(f'{name} must be an integer from 1 to 100')
        for name in ('delivery_drop_turn_deg_first', 'delivery_drop_turn_deg_second', 'delivery_inward_turn_deg'):
            if not math.isfinite(getattr(self, name)):
                raise ValueError(f'{name} must be finite')


@dataclass
class HandoffState:
    """AT boundary measurements; heading is diagnostic, not a TO origin."""
    distance_mm: Optional[float] = None
    heading_deg: Optional[float] = None

    def absolute_heading(self, local_heading_deg):
        if self.heading_deg is None:
            raise RuntimeError('AT_TO handoff has not been captured')
        # Keep the common course-normalized gyro frame established at startup.
        # The captured AT heading must not rotate subsequent route targets.
        return local_heading_deg
