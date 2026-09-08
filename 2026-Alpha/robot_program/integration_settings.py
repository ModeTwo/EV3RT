"""Non-B AT/TO route settings; distances are connected to feature builders."""

from dataclasses import dataclass
from typing import Optional
import math


@dataclass(frozen=True)
class IntegrationSettings:
    # 非B走行体の採用元。B版のtantou4.pyの値を混ぜない。
    re_source: str = 'gyro_line_0826.py'
    at_source: str = 'bottle_catch.py'
    to_source: str = 'tantou3.py'
    # REが青検知まで担当し、ATは100mm前進から接続する。
    at_gate_forward_mm: float = 100.0
    at_recognition_reverse_mm: float = 200.0
    # AT終了位置（緑円の延長線付近）を調整する第1候補。
    at_to_transfer_trace_mm: float = 460.0
    # TOが引渡し後に黒線へ接近する区間の距離上限。tantou3.py準拠。
    to_first_black_limit_mm: float = 565.0
    to_after_hint1_mm: float = 385.0
    to_hint2_trace_mm: float = 1000.0  # 元コメントは1200だが有効値は1000
    to_exit_trace_mm: float = 600.0
    to_spin_min_power: int = 55
    to_spin_max_power: int = 60
    motion_timeout_sec: float = 30.0
    bottle_timeout_sec: float = 15.0
    qr_timeout_sec: float = 20.0

    def __post_init__(self):
        for name in ('at_gate_forward_mm', 'at_recognition_reverse_mm',
                     'at_to_transfer_trace_mm', 'to_first_black_limit_mm', 'to_after_hint1_mm',
                     'to_hint2_trace_mm', 'to_exit_trace_mm', 'motion_timeout_sec',
                     'bottle_timeout_sec', 'qr_timeout_sec'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be a positive finite distance')


@dataclass
class HandoffState:
    """Position and course-normalized heading at the AT -> TO boundary."""
    distance_mm: Optional[float] = None
    heading_deg: Optional[float] = None

    def absolute_heading(self, local_heading_deg):
        if self.heading_deg is None:
            raise RuntimeError('AT_TO handoff has not been captured')
        return self.heading_deg + local_heading_deg
