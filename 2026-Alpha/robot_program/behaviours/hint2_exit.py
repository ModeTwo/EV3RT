"""Hint2 exit: heading hold, white edge, pivot turn and line acquisition."""
import math
import time

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_etrobo_util import TraceSide
from py_etrobo_util.plotter import IMU_HEADING_SIGN
from .line_trace import TraceLine

from ..runtime import runtime
from ..timing import CONTROL_INTERVAL_SEC


def angle_error(target, current):
    return (target - current + 180.0) % 360.0 - 180.0


class ExitTraceLine(TraceLine):
    """Shared PID/filter/edge logic, with the exit's bounded and ramped outputs."""
    def __init__(self, owner):
        self.owner = owner
        super().__init__(owner.name + ' PID trace',
                         target=owner.s.delivery_trace_target_v, power=owner.s.to_exit_power,
                         pid_p=0.65, pid_i=0.000001, pid_d=0.045,
                         trace_side=TraceSide.NORMAL)

    def set_motor_power(self, left_power, right_power):
        turn = (right_power - left_power) / (2.0 * runtime.course)
        limit = self.owner.s.to_exit_turn_power
        self.owner.command(self.owner.s.to_exit_power, max(-limit, min(limit, turn)))


class Hint2Exit(Behaviour):
    """Keep motion and progress limits in one node; once TURN starts, every limit or
    lost-line condition advances the phase instead of failing, so the tree always
    reaches FOLLOW and then SUCCESS. Only an invalid setup or a non-finite sensor
    reading still fails outright."""

    def __init__(self, name, context, settings):
        super().__init__(name)
        self.context, self.s = context, settings
        self.trace = None

    def initialise(self):
        runtime.require('plotter', 'gyro_sensor', 'color_sensor', 'left_motor', 'right_motor')
        self.left = self.right = 0.0
        self.trace = None
        self.last_log = -math.inf
        self.white_count = self.black_count = 0
        self.seen_line = False
        self.invalid = None
        try:
            self.straight_heading = self.context.at_to.absolute_heading(90.0)
            self.turn_heading = self.context.at_to.absolute_heading(self.s.to_exit_target_heading_deg)
            if runtime.course not in (-1, 1):
                raise ValueError('Invalid course')
            # WHITEの直進距離は、生の走行距離ではなく方位角90度方向への投影距離で
            # 制約する(to_hint_route.pyのdist_1200_from_75deg_startと同じ考え方)。
            raw_heading_deg = -runtime.course * IMU_HEADING_SIGN * self.straight_heading
            angle = math.radians(raw_heading_deg)
            self.straight_axis_x, self.straight_axis_y = math.sin(angle), math.cos(angle)
            self.white_origin_x = runtime.plotter.loc_x
            self.white_origin_y = runtime.plotter.loc_y
        except (RuntimeError, ValueError) as exc:
            self.invalid = str(exc)
        self.enter('WHITE')

    def enter(self, phase):
        self.phase = phase
        self.start_distance = runtime.plotter.get_distance()
        self.start_time = time.monotonic()
        self.white_count = self.black_count = 0
        self.logger.info('HINT2_EXIT phase=%s distance=%.1f' % (phase, self.start_distance))
        if phase == 'TURN':
            # Remove the previous forward command before ramping opposite wheel outputs.
            self.left = self.right = 0.0
            for motor in (runtime.left_motor, runtime.right_motor):
                motor.set_power(0)
                motor.set_brake(True)
        if phase == 'FOLLOW':
            self.trace = ExitTraceLine(self)

    def fail(self, reason):
        self.logger.error('HINT2_EXIT failed: ' + reason)
        return Status.FAILURE

    def command(self, base, turn):
        # Positive course-normalized turn agrees with shared SpinAround.
        step = self.s.to_exit_slew_power_per_s * CONTROL_INTERVAL_SEC
        requested = (base - runtime.course * turn, base + runtime.course * turn)
        for attr, target, motor in zip(('left', 'right'), requested,
                                       (runtime.left_motor, runtime.right_motor)):
            current = getattr(self, attr)
            value = current + max(-step, min(step, target - current))
            setattr(self, attr, value)
            motor.set_brake(False)
            motor.set_power(int(round(value)))

    def update(self):
        if self.invalid:
            return self.fail(self.invalid)
        s = self.s
        raw_angle = runtime.gyro_sensor.get_angle()
        _, _, v = runtime.color_sensor.get_raw_color_hsv()
        distance = abs(runtime.plotter.get_distance() - self.start_distance)
        white_projected_exceeded = False
        if self.phase == 'WHITE':
            dx = runtime.plotter.loc_x - self.white_origin_x
            dy = runtime.plotter.loc_y - self.white_origin_y
            white_projected_mm = dx * self.straight_axis_x + dy * self.straight_axis_y
            if not math.isfinite(white_projected_mm):
                return self.fail('non-finite odometry')
            white_projected_exceeded = white_projected_mm >= s.to_exit_white_straight_projected_limit_mm
        if not all(math.isfinite(x) for x in (raw_angle, v, distance)):
            return self.fail('non-finite sensor value')
        limit = {'WHITE': s.to_exit_trace_mm,
                 'OFFSET': s.to_exit_offset_mm + 50.0,
                 'TURN': s.to_exit_turn_limit_mm,
                 'BLACK': s.to_exit_search_limit_mm,
                 'FOLLOW': s.to_exit_follow_mm + 50.0}[self.phase]
        timed_out = time.monotonic() - self.start_time >= s.to_exit_phase_timeout_s
        if timed_out or distance >= limit or white_projected_exceeded:
            # どのフェーズで打ち切りになっても停止させず、常に前進させて最終的にFOLLOWを完了させる。
            if self.phase in ('WHITE', 'OFFSET'):
                self.enter('TURN')
                return Status.RUNNING
            if self.phase in ('TURN', 'BLACK'):
                self.enter('FOLLOW')
                self.trace.tick_once()
                return Status.RUNNING
            self.logger.info('HINT2_EXIT line acquisition complete (forced by %s)'
                             % ('timeout' if timed_out else 'distance limit'))
            return Status.SUCCESS

        current = -runtime.course * raw_angle
        if time.monotonic() - self.last_log >= 0.2:
            self.logger.info('HINT2_EXIT sample phase=%s mm=%.1f v=%.1f heading=%.1f left=%.1f right=%.1f white=%d black=%d' %
                             (self.phase, distance, v, current, self.left, self.right,
                              self.white_count, self.black_count))
            self.last_log = time.monotonic()
        error = angle_error(self.straight_heading, current)
        base = s.to_exit_power
        turn_limit = s.to_exit_turn_power

        if self.phase == 'WHITE':
            self.seen_line |= v <= s.delivery_trace_target_v
            eligible = self.seen_line and distance >= s.to_exit_white_min_mm
            self.white_count = self.white_count + 1 if eligible and v >= s.to_exit_white_v else 0
            if self.white_count >= s.to_exit_detect_cycles:
                self.enter('OFFSET' if s.to_exit_offset_mm > 0 else 'TURN')
                if self.phase == 'TURN':
                    error = angle_error(self.turn_heading, current)
        elif self.phase == 'OFFSET':
            if distance >= s.to_exit_offset_mm:
                self.enter('TURN')
                error = angle_error(self.turn_heading, current)
        elif self.phase in ('TURN', 'BLACK'):
            # Same TO absolute frame as return_heading=90, mirrored by course.
            error = angle_error(self.turn_heading, current)
            # 旋回序盤(目標方位まで遠い間)の偽検出でFOLLOWへ早期離脱しないよう、
            # 目標方位に十分近づくまでは黒検出のカウントを開始しない。
            black_eligible = abs(error) <= s.to_exit_turn_black_detect_max_error_deg
            self.black_count = self.black_count + 1 if (black_eligible and v <= s.to_exit_black_v) else 0
            if self.black_count >= s.to_exit_detect_cycles:
                self.enter('FOLLOW')
                # Tick the real shared PID behaviour immediately; no wait for 190 degrees.
                self.trace.tick_once()
                return Status.RUNNING
            if self.phase == 'TURN' and abs(error) <= s.to_exit_heading_tolerance_deg:
                # 190度まで旋回できれば、黒検出条件を満たしていなくてもそのままライントレースへ渡す。
                self.enter('FOLLOW')
                self.trace.tick_once()
                return Status.RUNNING
        else:
            # Same NORMAL edge polarity as TraceLine; bounded low-power acquisition.
            self.white_count = self.white_count + 1 if v >= s.to_exit_white_v else 0
            if self.white_count >= s.to_exit_lost_cycles:
                # ラインを見失っても停止せず、この時点の姿勢のまま次工程へ引き渡す。
                self.logger.info('HINT2_EXIT line acquisition complete (line lost tolerated)')
                return Status.SUCCESS
            if distance >= s.to_exit_follow_mm:
                self.logger.info('HINT2_EXIT line acquisition complete')
                return Status.SUCCESS
            self.trace.tick_once()
            return Status.RUNNING

        turn = s.to_exit_heading_kp * error
        if self.phase == 'TURN':
            # Brief braking interval; commanded translation remains zero for the whole turn.
            if time.monotonic() - self.start_time < s.to_exit_pivot_settle_s:
                return Status.RUNNING
            magnitude = min(s.to_exit_pivot_max_power,
                            max(s.to_exit_pivot_min_power, abs(turn)))
            self.command(0, math.copysign(magnitude, turn))
        else:
            self.command(base, max(-turn_limit, min(turn_limit, turn)))
        return Status.RUNNING

    def terminate(self, new_status):
        if self.trace is not None:
            self.trace.stop(Status.INVALID)
        for motor in (runtime.left_motor, runtime.right_motor):
            if motor is not None:
                motor.set_power(0)
                motor.set_brake(True)
