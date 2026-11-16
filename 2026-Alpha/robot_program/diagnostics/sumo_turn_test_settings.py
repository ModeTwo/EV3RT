"""Opt-in one-turn diagnostic; positive angles are physically clockwise.

相撲の設定と入力検証。距離はmm、角度は度、時間は秒、出力はPWM。既存の設定値と許容範囲を維持する。"""

from dataclasses import dataclass, fields
import math


@dataclass(frozen=True)
class SumoTurnTestSettings:
    angle_deg: float = 5.0
    power: int = 75
    hold_sec: float = 1.0
    baseline_sec: float = 0.3
    turn_timeout_sec: float = 4.0
    overall_timeout_sec: float = 20.0

    def __post_init__(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
            ):
                raise ValueError(field.name + " must be finite numeric data")
        if not 1 <= abs(self.angle_deg) <= 20:
            raise ValueError("sumo-turn-angle must have magnitude 1..20 degrees")
        if type(self.power) is not int or not 20 <= self.power <= 80:
            raise ValueError("sumo-turn-power must be an integer 20..80")
        if not 0.5 <= self.hold_sec <= 3:
            raise ValueError("sumo-turn-hold-sec must be 0.5..3 seconds")
        if (
            self.baseline_sec < 0.2
            or self.turn_timeout_sec <= 0
            or self.overall_timeout_sec < 5
        ):
            raise ValueError("invalid diagnostic timing")
