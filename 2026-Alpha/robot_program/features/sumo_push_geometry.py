"""Conservative continuous swept-disc checks in a measured floor frame.

Floor x=right, y=up, bearing=clockwise from up. No PDF pixel coordinates.
The disc encloses chassis, arm and retained bottle at every orientation.


ロボットIOを持たない計算。床面幾何、角度方針、捕捉証拠を扱う。相対方式は角度方針・捕捉証拠だけを利用する。"""

import math
from dataclasses import dataclass, fields
import json
from pathlib import Path


def delta(a, b):
    return (a - b + 180) % 360 - 180


def advance(p, bearing, distance):
    a = math.radians(bearing)
    return (p[0] + distance * math.sin(a), p[1] + distance * math.cos(a))


def point_segment(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    if dx or dy:
        t = max(
            0.0,
            min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy)),
        )
    else:
        t = 0.0
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def cross(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def segments_intersect(a, b, c, d):
    aa, bb, cc, dd = cross(a, b, c), cross(a, b, d), cross(c, d, a), cross(c, d, b)
    if aa * bb < 0 and cc * dd < 0:
        return True
    return any(
        abs(v) < 1e-8 and point_segment(p, u, w) < 1e-8
        for v, p, u, w in [(aa, c, a, b), (bb, d, a, b), (cc, a, c, d), (dd, b, c, d)]
    )


def inside(p, poly):
    hit = False
    for a, b in zip(poly, poly[1:] + poly[:1]):
        if point_segment(p, a, b) < 1e-8:
            return True
        if (a[1] > p[1]) != (b[1] > p[1]) and p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (
            b[1] - a[1]
        ) + a[0]:
            hit = not hit
    return hit


def clearance(a, b, polygons):
    result = math.inf
    for poly in polygons:
        if inside(a, poly) or inside(b, poly):
            return 0.0
        for c, d in zip(poly, poly[1:] + poly[:1]):
            if segments_intersect(a, b, c, d):
                return 0.0
            result = min(
                result,
                point_segment(a, c, d),
                point_segment(b, c, d),
                point_segment(c, a, b),
                point_segment(d, a, b),
            )
    return result


@dataclass(frozen=True)
class PushCalibration:
    calibration_id: str
    course: str
    calibrated: bool
    start_xy_mm: tuple
    start_bearing_deg: float
    ring_center_mm: tuple
    ring_radius_mm: float
    green_polygons_mm: tuple
    workspace_bounds_mm: tuple  # xmin,xmax,ymin,ymax: allowed white floor region
    body_half_width_mm: float
    body_front_mm: float  # include arm, from axle midpoint
    body_rear_mm: float
    held_bottle_offset_mm: float
    bottle_radius_mm: float
    hold_uncertainty_mm: float
    initial_position_error_mm: float
    heading_error_deg: float
    drift_mm_per_mm: float
    drift_mm_per_turn_deg: float
    braking_distance_mm: float
    max_speed_mm_s: float
    max_wheel_speed_mm_s: float
    max_turn_rate_deg_s: float
    max_cycle_sec: float
    mm_per_encoder_degree: float
    left_forward_sign: int
    right_forward_sign: int
    capture_max_mm: float
    push_max_mm: float
    push_out_margin_mm: float
    turn_power: int  # loaded turning requires separate field validation
    green_bearing_deg: float = 0.0  # measured direction toward green, floor coordinates
    green_turn_deadband_deg: float = 2.0  # provisional classification band
    drive_power: int = 80
    capture_raw_max: float = 70.0
    capture_confirm_sec: float = 0.2
    sensor_interval_sec: float = 0.05
    turn_tolerance_deg: float = 1.0
    turn_settle_sec: float = 0.2
    phase_timeout_sec: float = 15.0
    mission_timeout_sec: float = 90.0
    stall_timeout_sec: float = 1.5

    def __post_init__(self):
        if (
            self.calibrated is not True
            or not isinstance(self.calibration_id, str)
            or not self.calibration_id.strip()
        ):
            raise ValueError("measured calibration_id and calibrated=true required")
        if self.course not in ("left", "right"):
            raise ValueError("calibration course must be left/right")
        non_numeric = {
            "calibration_id",
            "course",
            "calibrated",
            "start_xy_mm",
            "ring_center_mm",
            "green_polygons_mm",
            "workspace_bounds_mm",
        }
        for f in fields(self):
            v = getattr(self, f.name)
            if f.name in non_numeric:
                continue
            if (
                isinstance(v, bool)
                or not isinstance(v, (int, float))
                or not math.isfinite(v)
            ):
                raise ValueError(f.name + " must be finite numeric")
            if (
                f.name
                not in (
                    "start_bearing_deg",
                    "green_bearing_deg",
                    "left_forward_sign",
                    "right_forward_sign",
                )
                and v <= 0
            ):
                raise ValueError(f.name + " must be positive")
        for name in ["start_xy_mm", "ring_center_mm", "workspace_bounds_mm"]:
            v = getattr(self, name)
            if name == "workspace_bounds_mm":
                n = 4
            else:
                n = 2
            if (
                not isinstance(v, (list, tuple))
                or len(v) != n
                or any(
                    isinstance(x, bool)
                    or not isinstance(x, (int, float))
                    or not math.isfinite(x)
                    for x in v
                )
            ):
                raise ValueError(name + " invalid")
        if self.left_forward_sign not in (-1, 1) or self.right_forward_sign not in (
            -1,
            1,
        ):
            raise ValueError("encoder signs must be +/-1")
        if (
            not isinstance(self.drive_power, int)
            or not 1 <= self.drive_power <= 100
            or not isinstance(self.turn_power, int)
            or not 1 <= self.turn_power <= 80
        ):
            raise ValueError("invalid PWM")
        if (
            not 0 < self.heading_error_deg < 45
            or self.turn_tolerance_deg > self.heading_error_deg
        ):
            raise ValueError("invalid heading uncertainty/tolerance")
        if (
            self.sensor_interval_sec > self.max_cycle_sec * 5
            or self.capture_confirm_sec < self.sensor_interval_sec * 2
        ):
            raise ValueError("invalid capture timing")
        if self.green_turn_deadband_deg >= 45:
            raise ValueError("green deadband must be below 45 degrees")
        if self.max_wheel_speed_mm_s < self.max_speed_mm_s:
            raise ValueError("wheel speed bound below center speed")
        x0, x1, y0, y1 = self.workspace_bounds_mm
        if not x0 < x1 or not y0 < y1:
            raise ValueError("invalid workspace bounds")
        if (
            not isinstance(self.green_polygons_mm, (list, tuple))
            or not self.green_polygons_mm
        ):
            raise ValueError("green polygons required")
        for poly in self.green_polygons_mm:
            if not isinstance(poly, (list, tuple)) or len(poly) < 3:
                raise ValueError("invalid green polygon")
            for p in poly:
                if (
                    not isinstance(p, (list, tuple))
                    or len(p) != 2
                    or any(
                        isinstance(v, bool)
                        or not isinstance(v, (int, float))
                        or not math.isfinite(v)
                        for v in p
                    )
                ):
                    raise ValueError("invalid green vertex")
            n = len(poly)
            if (
                abs(
                    sum(
                        poly[i][0] * poly[(i + 1) % n][1]
                        - poly[(i + 1) % n][0] * poly[i][1]
                        for i in range(n)
                    )
                )
                < 1e-6
            ):
                raise ValueError("zero polygon area")
            for i in range(n):
                if point_segment(poly[i], poly[(i + 1) % n], poly[(i + 1) % n]) < 1e-8:
                    raise ValueError("zero polygon edge")
                for j in range(i + 1, n):
                    if (j - i) % n not in (1, n - 1) and segments_intersect(
                        poly[i], poly[(i + 1) % n], poly[j], poly[(j + 1) % n]
                    ):
                        raise ValueError("self intersecting polygon")

    @classmethod
    def load(cls, path):
        return cls(**json.loads(Path(path).read_text(encoding="utf-8")))

    @property
    def radius(self):
        return max(
            math.hypot(self.body_half_width_mm, self.body_front_mm),
            math.hypot(self.body_half_width_mm, self.body_rear_mm),
            self.held_bottle_offset_mm
            + self.bottle_radius_mm
            + self.hold_uncertainty_mm,
        )

    @property
    def stop_allowance(self):
        # Covers delayed capture confirmation as well as reaction and coast.
        return self.braking_distance_mm + self.max_speed_mm_s * (
            self.capture_confirm_sec + self.sensor_interval_sec + self.max_cycle_sec
        )

    def safe_segment(self, a, b, error):
        r = self.radius + error + self.stop_allowance
        x0, x1, y0, y1 = self.workspace_bounds_mm
        for p in (a, b):
            if not x0 + r < p[0] < x1 - r or not y0 + r < p[1] < y1 - r:
                return False
        return clearance(a, b, self.green_polygons_mm) > r


@dataclass
class FloorPose:
    calibration: PushCalibration
    x: float
    y: float
    bearing: float
    gyro_origin: float
    left: float
    right: float
    at: float
    signed_mm: float = 0.0
    travel_mm: float = 0.0
    rotation_deg: float = 0.0

    @property
    def xy(self):
        return (self.x, self.y)

    @property
    def error(self):
        c = self.calibration
        return (
            c.initial_position_error_mm
            + self.travel_mm * c.drift_mm_per_mm
            + self.rotation_deg * c.drift_mm_per_turn_deg
        )

    def update(self, left, right, gyro, now):
        if not all(math.isfinite(v) for v in (left, right, gyro, now)):
            raise ValueError("nonfinite pose")
        c = self.calibration
        dt = now - self.at
        if dt < 0 or dt > c.max_cycle_sec:
            raise ValueError("control cycle outside calibrated bound")
        dl = (left - self.left) * c.left_forward_sign * c.mm_per_encoder_degree
        dr = (right - self.right) * c.right_forward_sign * c.mm_per_encoder_degree
        b = (c.start_bearing_deg + delta(gyro, self.gyro_origin)) % 360
        turn = delta(b, self.bearing)
        if (
            max(abs(dl), abs(dr))
            > c.max_wheel_speed_mm_s * dt + c.mm_per_encoder_degree * 2
        ):
            raise ValueError("encoder jump/speed bound")
        ds = (dl + dr) / 2
        if (
            abs(ds) > c.max_speed_mm_s * dt + c.mm_per_encoder_degree * 2
            or abs(turn) > c.max_turn_rate_deg_s * dt + 1.0
        ):
            raise ValueError("pose rate outside calibration")
        self.x, self.y = advance(self.xy, self.bearing + turn / 2, ds)
        self.bearing = b
        self.left = left
        self.right = right
        self.at = now
        self.signed_mm += ds
        self.travel_mm += abs(ds)
        self.rotation_deg += abs(turn)


def future_error(c, pose, distance, turn=0):
    return (
        pose.error
        + distance * (c.drift_mm_per_mm + math.sin(math.radians(c.heading_error_deg)))
        + abs(turn) * c.drift_mm_per_turn_deg
    )


def plan_push(c, pose, only_bearing=None, check_clearance=True):
    """Direct first, then smallest +/-5 degree change. No unsafe fallback."""
    if not check_clearance and only_bearing is None:
        raise ValueError("unchecked plan requires an explicit angle-policy bearing")
    if only_bearing is not None:
        changes = [delta(only_bearing, pose.bearing)]
    else:
        changes = [0] + [sign * a for a in range(5, 181, 5) for sign in (-1, 1)]
    for change in changes:
        bearing = (pose.bearing + change) % 360
        p = advance(pose.xy, bearing, c.held_bottle_offset_mm)
        radius = (
            c.ring_radius_mm
            + c.bottle_radius_mm
            + c.hold_uncertainty_mm
            + c.push_out_margin_mm
        )
        # Include final pose/heading uncertainty conservatively at maximum length.
        e = future_error(
            c, pose, c.push_max_mm, change
        ) + c.held_bottle_offset_mm * math.sin(math.radians(c.heading_error_deg))
        radius += e
        q = (p[0] - c.ring_center_mm[0], p[1] - c.ring_center_mm[1])
        u = advance((0, 0), bearing, 1)
        projection = q[0] * u[0] + q[1] * u[1]
        disc = projection * projection + radius * radius - q[0] * q[0] - q[1] * q[1]
        if math.hypot(*q) < radius:
            distance = max(0.0, -projection + math.sqrt(max(0.0, disc)))
        else:
            distance = 0.0
        if distance <= 0:  # Already outside before pushing: do not spin unnecessarily.
            if change and only_bearing is None:
                continue
        if distance > c.push_max_mm:
            continue
        end = advance(pose.xy, bearing, distance)
        if not check_clearance or c.safe_segment(pose.xy, end, e):
            return dict(
                kind="DIRECT" if abs(change) <= c.turn_tolerance_deg else "REDIRECT",
                bearing_deg=bearing,
                turn_deg=change,
                distance_mm=distance,
                end_xy_mm=end,
                error_mm=e,
                capture_assumed=True,
                physical_capture_confirmed=False,
            )
    return None


class CaptureEvidence:
    """API observations are candidates, not manufacturer freshness/retention proof."""

    def __init__(self, c):
        self.c = c
        self.since = None
        self.last = None
        self.count = 0

    def read(self, raw, now):
        valid = (
            not isinstance(raw, bool)
            and isinstance(raw, (int, float))
            and math.isfinite(raw)
            and 0 < raw <= self.c.capture_raw_max
        )
        if (
            not valid
            or self.last is not None
            and now - self.last > self.c.sensor_interval_sec + self.c.max_cycle_sec
        ):
            self.since = None
            self.count = 0
        self.last = now
        if valid:
            if self.since is None:
                self.since = now
            self.count += 1
        return (
            valid and self.count >= 3 and now - self.since >= self.c.capture_confirm_sec
        )


def alignment_push_policy(c, approach_bearing, aligned_bearing):
    """Latch final measured alignment, including all corrective turns.

    Green direction is a physical floor direction, never mirrored by course.
    A small positive component is in the configured deadband; geometry still
    must permit it. No geometry-selected alternative can override this policy.
    """
    change = delta(aligned_bearing, approach_bearing)
    component = math.cos(math.radians(delta(aligned_bearing, c.green_bearing_deg)))
    toward = component > math.sin(math.radians(c.green_turn_deadband_deg)) + 1e-12
    target = aligned_bearing % 360
    if toward:
        target = approach_bearing % 360
    return dict(
        approach_bearing_deg=approach_bearing % 360,
        aligned_bearing_deg=aligned_bearing % 360,
        actual_alignment_delta_deg=change,
        green_bearing_deg=c.green_bearing_deg,
        green_component=component,
        deadband_deg=c.green_turn_deadband_deg,
        toward_green=toward,
        decision="RETURN_APPROACH" if toward else "KEEP_ALIGNED",
        push_bearing_deg=target,
    )
