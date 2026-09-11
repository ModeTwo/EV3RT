"""Trial turn controller for the three bottle-delivery turns only."""
import time
from collections import deque

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence

from .gyro_drive import _normalize_heading_error
from .motor_control import StopNow
from .conditions import IsTimePassed
from ..runtime import runtime

# Bottle-final absolute heading only.
PULSE_SEC = 0.040
COARSE_STOP_DEG = 12.0
TOLERANCE_DEG = 2.0
STABLE_WINDOW_SEC = 0.2
STABLE_RANGE_DEG = 1.0
MAX_CORRECTIONS = 12
BRAKE_WAIT_LIMIT_SEC = 2.0


class DeliveryPulseTurn(Behaviour):
    def __init__(self, name, context, target, power):
        super().__init__(name)
        if not isinstance(power, int) or not 1 <= power <= 100:
            raise ValueError('Turn power must be an integer from 1 to 100')
        self.context, self.target, self.power = context, target, power

    def initialise(self):
        runtime.require('gyro_sensor', 'right_motor', 'left_motor')
        heading = self.context.delivery_heading.heading(runtime.gyro_sensor.get_angle())
        self.goal = float(self.target)
        error = _normalize_heading_error(self.goal - heading)
        self.direction = 1 if error >= 0 else -1
        self.state = 'coarse'
        self.corrections = 0
        self.samples = deque()
        self.last_log = float('-inf')
        self._brake()

    def _brake(self):
        for motor in (runtime.right_motor, runtime.left_motor):
            if motor is not None:
                motor.set_power(0)
                motor.set_brake(True)

    def _drive(self, direction):
        runtime.right_motor.set_brake(False)
        runtime.left_motor.set_brake(False)
        runtime.right_motor.set_power(runtime.course * direction * self.power)
        runtime.left_motor.set_power(-runtime.course * direction * self.power)

    def _wait(self, now, heading):
        self._brake()
        self.state = 'brake'
        self.brake_start = now
        self.samples = deque([(now, heading)])

    def _hold(self, reason):
        # Latch RUNNING under braking so parent trees cannot retry or advance.
        self.state = 'hold'
        self._brake()
        self.logger.error('pulse turn HOLD: %s; stop and restart the mission' % reason)

    def update(self):
        now = time.monotonic()
        heading = self.context.delivery_heading.heading(runtime.gyro_sensor.get_angle())
        error = _normalize_heading_error(self.goal - heading)
        before = self.state
        command = 0
        result = Status.RUNNING
        if self.state == 'coarse':
            if abs(error) <= COARSE_STOP_DEG or error * self.direction <= 0:
                self._wait(now, heading)
            else:
                command = self.direction * self.power
                self._drive(self.direction)
        elif self.state == 'pulse':
            # Preserve the user-tested 40ms pulse; completion is dispatch-quantized.
            command = self.pulse_direction * self.power
            if now - self.pulse_started >= PULSE_SEC:
                self._wait(now, heading)
                command = 0
        elif self.state == 'brake':
            self._brake()
            self.samples.append((now, heading))
            while len(self.samples) > 2 and now - self.samples[1][0] >= STABLE_WINDOW_SEC:
                self.samples.popleft()
            offsets = [_normalize_heading_error(h - self.samples[0][1]) for _, h in self.samples]
            stable = (now - self.samples[0][0] >= STABLE_WINDOW_SEC
                      and max(offsets) - min(offsets) <= STABLE_RANGE_DEG)
            if stable:
                # Every reading in the window must be inside the error band.
                if all(abs(_normalize_heading_error(self.goal-h)) <= TOLERANCE_DEG for _, h in self.samples):
                    self.state = 'done'
                    result = Status.SUCCESS
                elif self.corrections >= MAX_CORRECTIONS:
                    self._hold('correction limit reached, error=%.2f' % error)
                else:
                    self.corrections += 1
                    self.state = 'pulse'
                    self.pulse_started = now
                    direction = 1 if error > 0 else -1
                    self.pulse_direction = direction
                    command = direction * self.power
                    self._drive(direction)
            elif now - self.brake_start >= BRAKE_WAIT_LIMIT_SEC:
                self._hold('gyro did not settle under braking')
        else:
            self._brake()
        if self.state != before or now - self.last_log >= .1:
            self.logger.info('pulse turn state=%s target=%.2f heading=%.2f error=%.2f pwmR=%d pwmL=%d corrections=%d' % (
                self.state, self.goal, heading, error, runtime.course*command,
                -runtime.course*command, self.corrections))
            self.last_log = now
        return result

    def terminate(self, new_status):
        self._brake()


def delivery_turn(name, context, settings, target):
    root = Sequence(name=name, memory=True)
    root.add_children([
        DeliveryPulseTurn(name+' spin', context, target, settings.to_spin_min_power),
        StopNow(name=name+' brake'),
        IsTimePassed(name=name+' settle', delta_time=.5),
    ])
    return root
