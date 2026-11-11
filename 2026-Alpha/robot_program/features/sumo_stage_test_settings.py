"""①〜③単体・通し試験の設定。距離はmm、角度は度、出力はPWM。"""

from dataclasses import dataclass
import math
from typing import Optional


@dataclass(frozen=True)
class SumoStageTestSettings:
    # 実験条件をここに記入。押し出し距離は捕捉後の追加距離。
    push_distance_mm: Optional[float] = 300.0
    approach_distance_mm: float = 950.0
    approach_power: int = 100
    capture_max_mm: float = 500.0
    push_power: int = 80
    turn_power: int = 50
    # ResetDevice直後の物理床面方位と、リセット時からの直進目標差。
    initial_bearing_left_deg: float = 270.0
    initial_bearing_right_deg: float = 90.0
    approach_offset_left_deg: float = 0.0
    approach_offset_right_deg: float = 0.0

    def __post_init__(self):
        for name in ('initial_bearing_left_deg', 'initial_bearing_right_deg',
                     'approach_offset_left_deg', 'approach_offset_right_deg'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(name + ' must be a finite number')
