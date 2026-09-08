"""Motion primitives for AT and TO, using the shared devices and period."""
from py_trees.behaviour import Behaviour
from py_trees.common import Status, ParallelPolicy
from py_trees.composites import Sequence, Parallel
from py_trees.decorators import Timeout
from ..runtime import runtime
from ..types import HeadingType
from .conditions import IsDistanceEarned, IsTimePassed
from .gyro_drive import RunByGyro, SpinAround
from .motor_control import StopNow


class DriveDistance(Behaviour):
    """AT source contract: signed PWM, absolute travelled distance, final brake."""
    def __init__(self, name, distance_mm, power):
        super().__init__(name)
        self.distance_mm = distance_mm
        self.power = power

    def initialise(self):
        runtime.require('plotter', 'left_motor', 'right_motor')
        self.start_distance = runtime.plotter.get_distance()
        self.logger.info('%s distance_mm=%.1f power=%d' % (self.name, self.distance_mm, self.power))

    def update(self):
        if abs(runtime.plotter.get_distance() - self.start_distance) >= self.distance_mm:
            return Status.SUCCESS
        for motor in (runtime.left_motor, runtime.right_motor):
            motor.set_brake(False)
            motor.set_power(self.power)
        return Status.RUNNING

    def terminate(self, new_status):
        for motor in (runtime.left_motor, runtime.right_motor):
            if motor is not None:
                motor.set_power(0)
                motor.set_brake(True)


class LocalSpin(SpinAround):
    """Translate TO local absolute targets without resetting the gyro."""
    def __init__(self, name, context, target, **kwargs):
        self.context, self.local_target = context, target
        super().__init__(name=name, target=target, **kwargs)

    def update(self):
        if not self.running and self.target_type == HeadingType.ABSOLUTE:
            self.target = self.context.at_to.absolute_heading(self.local_target)
        return super().update()


class LocalDrive(RunByGyro):
    def __init__(self, name, context, target, **kwargs):
        self.context, self.local_target = context, target
        super().__init__(name=name, target=target, **kwargs)

    def update(self):
        if not self.running and self.target_type == HeadingType.ABSOLUTE:
            self.target = self.context.at_to.absolute_heading(self.local_target)
        return super().update()


def distance_motion(name, motion, distance, timeout):
    parallel = Parallel(name=name, policy=ParallelPolicy.SuccessOnOne())
    parallel.add_children([motion, IsDistanceEarned(name=name + ' distance', delta_dist=distance)])
    root = Sequence(name=name + ' segment', memory=True)
    root.add_children([Timeout(name=name + ' timeout', child=parallel, duration=timeout),
                       StopNow(name=name + ' brake')])
    return root


def to_turn(name, context, settings, target, relative=False):
    root = Sequence(name=name, memory=True)
    spin = LocalSpin(name=name + ' spin', context=context, target=target,
                     max_power=settings.to_spin_max_power, min_power=settings.to_spin_min_power,
                     pid_p=0.2, pid_i=0.00075, pid_d=0.03,
                     target_type=HeadingType.RELATIVE if relative else HeadingType.ABSOLUTE)
    root.add_children([Timeout(name=name + ' timeout', child=spin, duration=settings.motion_timeout_sec),
                       StopNow(name=name + ' brake'),
                       IsTimePassed(name=name + ' settle', delta_time=0.5)])
    return root


def to_drive(name, context, settings, distance):
    return distance_motion(name,
        LocalDrive(name=name + ' gyro', context=context, target=0, power=60,
                   pid_p=0.0001, pid_i=0.00001, pid_d=0.04, target_type=HeadingType.ABSOLUTE),
        distance, settings.motion_timeout_sec)
