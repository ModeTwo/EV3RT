"""Settings for the opt-in, approach-and-face-only sumo feature."""
import math
from dataclasses import dataclass, fields


@dataclass(frozen=True)
class SumoAlignSettings:
    distance_mm: float = 1000.0
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
    max_turns: int = 5
    turn_timeout_sec: float = 4.0
    overall_timeout_sec: float = 40.0

    def __post_init__(self):
        for f in fields(self):
            v = getattr(self, f.name)
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                raise ValueError(f'{f.name} must be finite numeric data')
        if self.distance_mm <= 0 or not isinstance(self.power, int) or not 1 <= self.power <= 100:
            raise ValueError('distance_mm must be positive; power must be an integer from 1 to 100')
        for name in ('drive_timeout_sec', 'observation_timeout_sec', 'max_observation_age_sec',
                     'turn_timeout_sec', 'overall_timeout_sec', 'center_tolerance_px',
                     'max_stationary_heading_error_deg', 'max_turn_step_deg'):
            if getattr(self, name) <= 0:
                raise ValueError(f'{name} must be positive')
        if self.settle_sec < 0 or not 0 <= self.target_cx < 320 or self.max_turn_step_deg > 45:
            raise ValueError('invalid settling, target center or turn step')
        if not isinstance(self.stable_frames, int) or self.stable_frames < 2:
            raise ValueError('stable_frames must be an integer >= 2')
        if not isinstance(self.max_turns, int) or self.max_turns < 1:
            raise ValueError('max_turns must be a positive integer')
