"""Settings for the opt-in, approach-and-face-only sumo feature.

相撲の設定と入力検証。距離はmm、角度は度、時間は秒、出力はPWM。既存の設定値と許容範囲を維持する。"""

import math
from dataclasses import dataclass, fields


@dataclass(frozen=True)
class SumoAlignSettings:
    distance_mm: float = 950.0
    power: int = 80
    drive_timeout_sec: float = 15.0
    settle_sec: float = 0.3
    observation_timeout_sec: float = 3.0
    max_observation_age_sec: float = 0.4
    stable_frames: int = 2
    target_cx: float = 160.0
    center_tolerance_px: float = 8.0
    max_stationary_heading_error_deg: float = 2.0
    max_turn_step_deg: float = 20.0
    max_turns: int = 3
    turn_timeout_sec: float = 4.0
    overall_timeout_sec: float = 40.0

    visual_px_per_degree: float = 7.5
    visual_fast_power: int = 75
    visual_slow_power: int = 50
    visual_fast_error_deg: float = 12.0
    visual_fine_error_deg: float = 4.0
    visual_stop_horizon_sec: float = 0.10
    visual_stop_margin_deg: float = 0.5
    visual_max_image_age_sec: float = 0.12
    visual_fast_image_age_sec: float = 0.06
    visual_brake_hold_sec: float = 0.12
    visual_post_settle_sec: float = 0.04
    visual_pulse_on_sec: float = 0.02
    visual_pulse_off_sec: float = 0.12
    visual_kick_sec: float = 0.04
    visual_loss_timeout_sec: float = 0.6
    visual_max_reversals: int = 5
    visual_max_pulses: int = 16

    def __post_init__(self):
        for f in fields(self):
            v = getattr(self, f.name)
            if (
                isinstance(v, bool)
                or not isinstance(v, (int, float))
                or not math.isfinite(v)
            ):
                raise ValueError(f"{f.name} must be finite numeric data")
        for f in fields(self):
            if f.name.startswith("visual_") and getattr(self, f.name) <= 0:
                raise ValueError(f"{f.name} must be positive")
        for name in (
            "visual_fast_power",
            "visual_slow_power",
            "visual_max_reversals",
            "visual_max_pulses",
        ):
            if not isinstance(getattr(self, name), int):
                raise ValueError(name + " must be an integer")
        if not 1 <= self.visual_slow_power <= self.visual_fast_power <= 80:
            raise ValueError("invalid visual motor powers")
        if (
            not self.visual_fast_image_age_sec
            <= self.visual_max_image_age_sec
            <= self.max_observation_age_sec
        ):
            raise ValueError("invalid visual image age limits")
        if self.visual_fine_error_deg >= self.visual_fast_error_deg:
            raise ValueError("fine turn threshold must be below fast threshold")
        if (
            self.distance_mm <= 0
            or not isinstance(self.power, int)
            or not 1 <= self.power <= 100
        ):
            raise ValueError(
                "distance_mm must be positive; power must be an integer from 1 to 100"
            )
        for name in (
            "drive_timeout_sec",
            "observation_timeout_sec",
            "max_observation_age_sec",
            "turn_timeout_sec",
            "overall_timeout_sec",
            "center_tolerance_px",
            "max_stationary_heading_error_deg",
            "max_turn_step_deg",
        ):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive")
        if (
            self.settle_sec < 0
            or not 0 <= self.target_cx < 320
            or self.max_turn_step_deg > 45
        ):
            raise ValueError("invalid settling, target center or turn step")
        if not isinstance(self.stable_frames, int) or self.stable_frames < 2:
            raise ValueError("stable_frames must be an integer >= 2")
        if not isinstance(self.max_turns, int) or self.max_turns < 1:
            raise ValueError("max_turns must be a positive integer")
