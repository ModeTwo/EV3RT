"""Explicit, bounded low-power approach test after visual alignment.

相撲の設定と入力検証。距離はmm、角度は度、時間は秒、出力はPWM。既存の設定値と許容範囲を維持する。"""

from dataclasses import dataclass, fields
import math
from typing import Optional


@dataclass(frozen=True)
class SumoCatchSettings:
    # No guessed physical distance: enabling the mission requires an explicit value.
    distance_mm: Optional[float] = None
    power: int = 25
    drive_timeout_sec: float = 8.0
    ready_timeout_sec: float = 2.0
    post_record_sec: float = 1.0
    max_image_age_sec: float = 0.15
    max_error_px: float = 20.0
    max_heading_error_deg: float = 4.0
    stall_timeout_sec: float = 1.0
    stall_progress_mm: float = 2.0

    def __post_init__(self):
        for item in fields(self):
            value = getattr(self, item.name)
            if item.name == "distance_mm" and value is None:
                continue
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value <= 0
            ):
                raise ValueError(item.name + " must be positive finite numeric data")
        if not isinstance(self.power, int) or not 15 <= self.power <= 100:
            raise ValueError("catch test base power must be an integer from 15 to 100")
        if (
            self.max_image_age_sec > 0.4
            or self.max_heading_error_deg > 10
            or self.max_error_px > 40
        ):
            raise ValueError("catch test observation/heading bounds are too loose")
        if not 0.5 <= self.post_record_sec <= 3:
            raise ValueError("post record time must be .5..3 seconds")

    def require_distance(self):
        if self.distance_mm is None:
            raise ValueError("sumo-catch-test requires --sumo-catch-distance-mm")
