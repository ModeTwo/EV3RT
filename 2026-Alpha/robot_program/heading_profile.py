"""Continuous course-normalized heading (degrees) by travelled distance (mm)."""

from bisect import bisect_right
import math


class HeadingProfile:
    def __init__(self, points):
        self.points = tuple((float(s), float(h)) for s, h in points)
        if len(self.points) < 2 or self.points[0] != (0.0, 0.0):
            raise ValueError('profile must start at (0 mm, 0 deg) and have >=2 points')
        if any(not math.isfinite(v) for p in self.points for v in p):
            raise ValueError('profile values must be finite')
        if any(b[0] <= a[0] for a, b in zip(self.points, self.points[1:])):
            raise ValueError('distances must strictly increase')
        self.distances = tuple(p[0] for p in self.points)
        self.length_mm = self.distances[-1]

    def heading_at(self, distance_mm):
        if not math.isfinite(distance_mm):
            raise ValueError('distance must be finite')
        if distance_mm <= 0:
            return 0.0
        if distance_mm >= self.length_mm:
            return self.points[-1][1]
        i = bisect_right(self.distances, distance_mm)
        s0, h0 = self.points[i - 1]
        s1, h1 = self.points[i]
        return h0 + (h1 - h0) * (distance_mm - s0) / (s1 - s0)
