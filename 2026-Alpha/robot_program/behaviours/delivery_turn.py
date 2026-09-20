"""Bottle heading adapter using the original sample.py SpinAround algorithm."""
from py_trees.composites import Sequence
from .gyro_drive import SpinAround
from .motor_control import StopNow
from .conditions import IsTimePassed
from ..runtime import runtime
from ..types import HeadingType


class DeliveryPulseTurn(SpinAround):
    # Keep the public class name for callers; pulse correction has been removed.
    def __init__(self, name, context, target, power, max_power=None):
        self.context, self.delivery_target = context, target
        super().__init__(name=name, target=target, min_power=power,
                         max_power=power if max_power is None else max_power,
                         pid_p=0.2, pid_i=0.00075, pid_d=0.03,
                         target_type=HeadingType.ABSOLUTE)

    def update(self):
        if not self.running:
            raw = runtime.gyro_sensor.get_angle()
            current = self.context.delivery_heading.heading(raw)
            self.target = -runtime.course * raw + self.delivery_target - current
        return super().update()


def delivery_turn(name, context, settings, target):
    root = Sequence(name=name, memory=True)
    root.add_children([
        DeliveryPulseTurn(name+' spin', context, target,
                          settings.to_spin_min_power, settings.to_spin_max_power),
        StopNow(name=name+' brake'),
        # IsTimePassed(name=name+' settle', delta_time=.5),
    ])
    return root
