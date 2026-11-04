"""相撲の設定と入力検証。距離はmm、角度は度、時間は秒、出力はPWM。既存の設定値と許容範囲を維持する。"""

import math
from dataclasses import dataclass, fields


@dataclass(frozen=True)
class RelativePushSettings:
    push_distance_mm: float = None
    capture_max_mm: float = 500.0
    power: int = 80
    turn_power: int = 50
    green_bearing_deg: float = 0.0
    green_turn_deadband_deg: float = 2.0
    capture_raw_max: float = 80.0
    capture_confirm_sec: float = 0.2
    sensor_interval_sec: float = 0.05
    max_cycle_sec: float = 0.2  # capture read-gap guard, not a calibrated motion bound
    settle_sec: float = 0.2
    phase_timeout_sec: float = 15.0
    overall_timeout_sec: float = 90.0

    def __post_init__(self):
        for f in fields(self):
            v = getattr(self, f.name)
            if f.name == "push_distance_mm" and v is None:
                continue
            if (
                isinstance(v, bool)
                or not isinstance(v, (int, float))
                or not math.isfinite(v)
            ):
                raise ValueError(f.name + " must be finite numeric")
            if f.name != "green_bearing_deg" and v <= 0:
                raise ValueError(f.name + " must be positive")
        if not isinstance(self.power, int) or not 1 <= self.power <= 100:
            raise ValueError("push PWM must be 1..100")
        if not isinstance(self.turn_power, int) or not 1 <= self.turn_power <= 80:
            raise ValueError("turn PWM must be 1..80")
        if self.green_turn_deadband_deg >= 45:
            raise ValueError("invalid angle deadband")
