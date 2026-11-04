"""Metadata and timing of one static bottle placement measurement.

相撲の設定と入力検証。距離はmm、角度は度、時間は秒、出力はPWM。既存の設定値と許容範囲を維持する。"""

from dataclasses import dataclass
import math
from typing import Optional


@dataclass(frozen=True)
class SumoSensorSettings:
    label: Optional[str] = None
    forward_mm: Optional[float] = None
    lateral_mm: Optional[float] = None
    notes: str = ""
    duration_sec: float = 5.0
    interval_sec: float = 0.1
    mm_per_unit: float = 1.0
    interactive: bool = False

    def __post_init__(self):
        if not isinstance(self.interactive, bool):
            raise ValueError("interactive must be boolean")
        if self.label is not None and (
            not isinstance(self.label, str)
            or not self.label.strip()
            or len(self.label) > 64
            or any(ord(c) < 32 for c in self.label)
        ):
            raise ValueError("sensor label must contain 1..64 printable characters")
        if (
            not isinstance(self.notes, str)
            or len(self.notes) > 240
            or any(ord(c) < 32 for c in self.notes)
        ):
            raise ValueError(
                "sensor notes must be a printable string up to 240 characters"
            )
        for name in (
            "forward_mm",
            "lateral_mm",
            "duration_sec",
            "interval_sec",
            "mm_per_unit",
        ):
            value = getattr(self, name)
            if name in ("forward_mm", "lateral_mm") and value is None:
                continue
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
            ):
                raise ValueError(name + " must be finite numeric data")
        if (
            not 1 <= self.duration_sec <= 60
            or not 0.05 <= self.interval_sec <= 1
            or self.mm_per_unit <= 0
        ):
            raise ValueError(
                "duration 1..60 sec, interval .05..1 sec and positive scale required"
            )

    def require_label(self):
        if self.label is None:
            raise ValueError("sumo-sensor-test requires --sumo-sensor-label")
