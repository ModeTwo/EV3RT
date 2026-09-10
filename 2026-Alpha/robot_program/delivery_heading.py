"""Bottle-final course heading: line=0, drop=-90, inward=90 (mirrored by course)."""
import math
from dataclasses import dataclass
from typing import Optional
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from .runtime import runtime


@dataclass
class DeliveryHeadingReference:
    reset_heading: Optional[float] = None
    reset_gyro: Optional[float] = None
    course: Optional[int] = None

    def register(self, heading, gyro, course):
        if course not in (-1, 1) or not all(math.isfinite(float(v)) for v in (heading, gyro)):
            raise ValueError('Invalid bottle heading reference')
        self.reset_heading, self.reset_gyro, self.course = float(heading), float(gyro), course

    def heading(self, gyro):
        if self.reset_heading is None or self.reset_gyro is None:
            raise RuntimeError('Bottle heading must be registered after device reset')
        if not math.isfinite(float(gyro)):
            raise ValueError('Invalid gyro angle')
        return self.reset_heading - self.course * (float(gyro) - self.reset_gyro)


def initial_delivery_heading(mission):
    # Only bottle-final starts along the delivery line. Combined start faces 180.
    return 0.0 if mission == 'bottle-final' else 180.0


class RegisterDeliveryHeading(Behaviour):
    def __init__(self, name, context, heading):
        super().__init__(name)
        self.context, self.heading = context, heading

    def update(self):
        runtime.require('gyro_sensor')
        raw = runtime.gyro_sensor.get_angle()
        self.context.delivery_heading.register(self.heading, raw, runtime.course)
        self.logger.info('bottle heading reference: heading=%.1f gyro=%.1f course=%d' % (
            self.heading, raw, runtime.course))
        return Status.SUCCESS
