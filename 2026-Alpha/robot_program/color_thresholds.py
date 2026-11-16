"""Central HSV thresholds for ET robot color detection.

Sensor HSV and OpenCV HSV use different scales; do not mix them.
Values are existing defaults, not newly calibrated measurements.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class HSVRange:
    h_min: int
    h_max: int
    s_min: int
    s_max: int
    v_min: int
    v_max: int

    def contains(self, h, s, v):
        return (self.h_min <= h <= self.h_max
                and self.s_min <= s <= self.s_max
                and self.v_min <= v <= self.v_max)


# Robot color sensor: raw values (existing sumo thresholds).
SENSOR_BLUE_LINE = HSVRange(190, 230, 58, 255, 0, 255)
SENSOR_BLACK_LINE = HSVRange(0, 255, 0, 255, 0, 25)

# Camera: OpenCV HSV (H: 0-179, S/V: 0-255).
CAMERA_SUMO_BLACK_TAPE = HSVRange(0, 179, 0, 120, 0, 40)

# Other bottle/line ranges should be added here after reviewing their
# existing detector implementations and their actual HSV conventions.
