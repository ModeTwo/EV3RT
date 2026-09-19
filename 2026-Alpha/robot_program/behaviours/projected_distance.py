"""Straight-section progress projected from the existing encoder/gyro odometry."""
import math
import time

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_etrobo_util.plotter import IMU_HEADING_SIGN

from ..runtime import runtime


class IsProjectedDistanceEarned(Behaviour):
    def __init__(self, name, context, delta_dist, local_heading_deg=90.0):
        super().__init__(name)
        if not math.isfinite(delta_dist) or delta_dist <= 0:
            raise ValueError('Projected distance must be positive and finite')
        if not math.isfinite(local_heading_deg):
            raise ValueError('Local heading must be finite')
        self.context = context
        self.delta_dist = delta_dist
        self.local_heading_deg = local_heading_deg

    def initialise(self):
        runtime.require('plotter')
        self.invalid = None
        self.last_log = -math.inf
        try:
            if runtime.course not in (-1, 1):
                raise ValueError('Invalid course')
            heading = self.context.at_to.absolute_heading(self.local_heading_deg)
            # Controllers use -course * gyro_angle; Plotter uses IMU_SIGN * gyro_angle.
            self.raw_heading_deg = -runtime.course * IMU_HEADING_SIGN * heading
            angle = math.radians(self.raw_heading_deg)
            self.axis_x, self.axis_y = math.sin(angle), math.cos(angle)
            self.start_x = runtime.plotter.loc_x
            self.start_y = runtime.plotter.loc_y
            self.start_distance = runtime.plotter.get_distance()
            if not all(math.isfinite(v) for v in
                       (heading, self.start_x, self.start_y, self.start_distance)):
                raise ValueError('Non-finite initial odometry or heading')
        except (ValueError, RuntimeError) as exc:
            self.invalid = str(exc)

    def update(self):
        if self.invalid:
            self.logger.error('PROJECTED_DISTANCE failed: ' + self.invalid)
            return Status.FAILURE
        dx = runtime.plotter.loc_x - self.start_x
        dy = runtime.plotter.loc_y - self.start_y
        travelled = runtime.plotter.get_distance() - self.start_distance
        progress = dx * self.axis_x + dy * self.axis_y
        if not all(math.isfinite(v) for v in (dx, dy, travelled, progress)):
            self.logger.error('PROJECTED_DISTANCE failed: non-finite odometry')
            return Status.FAILURE
        done = progress >= self.delta_dist
        now = time.monotonic()
        if done or now - self.last_log >= 0.5:
            self.logger.info(
                'PROJECTED_DISTANCE %s projected_mm=%.1f travelled_mm=%.1f target_mm=%.1f raw_heading_deg=%.1f done=%s'
                % (self.name, progress, travelled, self.delta_dist, self.raw_heading_deg, done))
            self.last_log = now
        return Status.SUCCESS if done else Status.RUNNING
