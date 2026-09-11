"""Reusable gyro drive behaviors."""

import time
from collections import deque
import math
from typing import Callable, Union

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from simple_pid import PID

from py_etrobo_util import SymmetricClamper

from ..runtime import runtime
from ..types import HeadingType
from ..path_tracking import PathTracking, curvature_turn


from ..timing import CONTROL_INTERVAL_SEC as EXEC_INTERVAL


def _normalize_heading_error(error: float) -> float:
    # 角度差を-180度以上180度未満へ正規化する。
    return (error + 180.0) % 360.0 - 180.0


class SpinAround(Behaviour):
    # Heading origin and target semantics are unchanged; stopping uses 40ms pulses.
    def __init__(self, name, target, max_power, min_power, pid_p, pid_i, pid_d,
                 target_type, tolerance=2.0, settle_time=0.2,
                 slowdown_angle=12.0, fine_power=None):
        super().__init__(name)
        fine_power = min_power if fine_power is None else fine_power
        if not (0 < min_power <= max_power <= 100 and
                0 < tolerance < slowdown_angle and settle_time > 0):
            raise ValueError('Invalid spin power or stopping settings')
        self.target, self.target_type = target, target_type
        self.pid_p, self.pid_i, self.pid_d = pid_p, pid_i, pid_d
        self.min_power, self.max_power = min_power, max_power
        self.power = int(max(min_power, min(fine_power, max_power)))
        self.tolerance, self.settle_time = tolerance, settle_time
        self.slowdown_angle = slowdown_angle
        self.clamper = SymmetricClamper(min_power, max_power)
        self.running = False
        self.target_heading = 0.0
        self.pid = None  # Compatibility attribute; pulse control has no PID history.

    def _start(self):
        runtime.require('gyro_sensor', 'right_motor', 'left_motor')
        heading = -runtime.course * runtime.gyro_sensor.get_angle()
        self.goal = (heading + self.target if self.target_type == HeadingType.RELATIVE
                     else float(self.target))
        self.target_heading = self.goal
        self.running = True
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
        if not self.running:
            self._start()
        now = time.monotonic()
        heading = -runtime.course * runtime.gyro_sensor.get_angle()
        error = _normalize_heading_error(self.goal - heading)
        before = self.state
        command = 0
        result = Status.RUNNING
        if self.state == 'coarse':
            if abs(error) <= self.slowdown_angle or error * self.direction <= 0:
                self._wait(now, heading)
            else:
                command = self.direction * self.power
                self._drive(self.direction)
        elif self.state == 'pulse':
            # Preserve the user-tested 40ms pulse; completion is dispatch-quantized.
            command = self.pulse_direction * self.power
            if now - self.pulse_started >= 0.040:
                self._wait(now, heading)
                command = 0
        elif self.state == 'brake':
            self._brake()
            self.samples.append((now, heading))
            while len(self.samples) > 2 and now - self.samples[1][0] >= self.settle_time:
                self.samples.popleft()
            offsets = [_normalize_heading_error(h - self.samples[0][1]) for _, h in self.samples]
            stable = (now - self.samples[0][0] >= self.settle_time
                      and max(offsets) - min(offsets) <= 1.0)
            if stable:
                # Every reading in the window must be inside the error band.
                if all(abs(_normalize_heading_error(self.goal-h)) <= self.tolerance for _, h in self.samples):
                    self.state = 'done'
                    result = Status.SUCCESS
                elif self.corrections >= 12:
                    self._hold('correction limit reached, error=%.2f' % error)
                else:
                    self.corrections += 1
                    self.state = 'pulse'
                    self.pulse_started = now
                    direction = 1 if error > 0 else -1
                    self.pulse_direction = direction
                    command = direction * self.power
                    self._drive(direction)
            elif now - self.brake_start >= 2.0:
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
        self.running = False


class RunByGyro(Behaviour):
    """ジャイロで目標角を追従する走行命令。

    target=90.0: 従来どおり一定角度へ走る。
    target=heading_at: 毎周期 heading_at(開始からの距離mm) で目標角を得る。
    関数を渡す場合はRELATIVEを使い、開始時を0度とする連続角を返す。
    関数名には括弧を付けない。PIDは開始時に一度だけ生成する。
    """
    def __init__(
        self,
        name: str,
        target: Union[float, Callable[[float], float]],
        power: int,
        pid_p: float,
        pid_i: float,
        pid_d: float,
        target_type: HeadingType,
        *,
        distance_limit_mm: float = None,
        completion_condition: Behaviour = None,
        completion_min_mm: float = 0.0,
        feedforward_gain: float = 0.0,
        wheel_tread_mm: float = 110.0,
        cross_track_lookahead_mm: float = 0.0,
        max_heading_correction_deg: float = 8.0,
        profile_log_interval_sec: float = 1.0,
    ) -> None:
        super().__init__(name)
        self.target = target
        self.target_type = target_type
        self.power = power
        self.pid_p = pid_p
        self.pid_i = pid_i
        self.pid_d = pid_d
        self.last_log_time = None
        self.running = False
        self.target_heading = 0
        self.pid = None
        self.distance_target = callable(target)
        self.distance_limit_mm = distance_limit_mm
        self.completion_condition = completion_condition
        self.completion_min_mm = completion_min_mm
        # 追加補正は既定で無効。スタート～LAPのfeatureが明示的に有効化する。
        self.feedforward_gain = feedforward_gain
        self.wheel_tread_mm = wheel_tread_mm
        self.cross_track_lookahead_mm = cross_track_lookahead_mm
        self.max_heading_correction_deg = max_heading_correction_deg
        self.profile_log_interval_sec = profile_log_interval_sec
        self.path_tracker = None
        values = (feedforward_gain, wheel_tread_mm, cross_track_lookahead_mm,
                  max_heading_correction_deg, profile_log_interval_sec)
        if (not all(map(math.isfinite, values)) or feedforward_gain < 0 or wheel_tread_mm <= 0
                or cross_track_lookahead_mm < 0 or not 0 <= max_heading_correction_deg <= 45
                or profile_log_interval_sec <= 0):
            raise ValueError('Invalid path tracking settings')
        if not self.distance_target and (feedforward_gain != 0 or cross_track_lookahead_mm != 0):
            raise ValueError('path tracking requires a target function')
        if self.distance_target:
            # 距離関数モードだけの契約。固定角度を使う他工程の仕様は維持。
            if target_type != HeadingType.RELATIVE:
                raise ValueError('distance target requires HeadingType.RELATIVE')
            if distance_limit_mm is None or not math.isfinite(distance_limit_mm) or distance_limit_mm <= 0:
                raise ValueError('distance_limit_mm must be positive and finite')
            if not math.isfinite(completion_min_mm) or not 0 <= completion_min_mm < distance_limit_mm:
                raise ValueError('completion window must be within distance_limit_mm')
            if not isinstance(power, int) or not 1 <= power <= 100:
                raise ValueError('power must be an integer from 1 to 100')
            if any(not math.isfinite(gain) for gain in (pid_p, pid_i, pid_d)):
                raise ValueError('PID gains must be finite')
        elif distance_limit_mm is not None or completion_condition is not None or completion_min_mm != 0:
            raise ValueError('distance completion options require a target function')

    def update(self) -> Status:
        # 固定角度モードは従来の制御をそのまま使用する。
        if self.distance_target:
            return self._update_distance_target()
        runtime.require("plotter", "gyro_sensor", "right_motor", "left_motor")
        current_heading = -runtime.course * runtime.gyro_sensor.get_angle()

        # 走行周期への影響を抑えるため、方位ログは1秒に1回だけ出力する。
        if self.last_log_time is None or time.time() - self.last_log_time >= 1.0:
            self.logger.info(
                "%+06d %s.current heading=%d"
                % (runtime.plotter.get_distance(), self.__class__.__name__, current_heading)
            )
            self.last_log_time = time.time()

        if not self.running:
            if self.target_type == HeadingType.RELATIVE:
                self.target_heading = current_heading + self.target
            else:
                self.target_heading = self.target
            self.pid = PID(
                self.pid_p,
                self.pid_i,
                self.pid_d,
                setpoint=self.target_heading,
                sample_time=EXEC_INTERVAL,
                output_limits=(-self.power, self.power),
            )
            self.logger.info(
                "%+06d %s.gyro run started toward heading=%d"
                % (
                    runtime.plotter.get_distance(),
                    self.__class__.__name__,
                    self.target_heading,
                )
            )
            self.running = True

        turn = int(self.pid(current_heading))
        runtime.right_motor.set_brake(False)
        runtime.left_motor.set_brake(False)
        runtime.right_motor.set_power(self.power + runtime.course * turn)
        runtime.left_motor.set_power(self.power - runtime.course * turn)
        return Status.RUNNING


    def _update_distance_target(self) -> Status:
        """距離関数モードの1周期。順序: 計測→終了判定→目標角→出力。"""
        runtime.require('plotter', 'gyro_sensor', 'right_motor', 'left_motor')
        distance = float(runtime.plotter.get_distance())
        angle = float(runtime.gyro_sensor.get_angle())
        if runtime.course not in (-1, 1) or not all(map(math.isfinite, (distance, angle))):
            return self._finish_distance_run(Status.FAILURE)
        heading = -runtime.course * angle

        # 1. 開始時だけ原点とPIDを作る。目標角が変わってもPIDを作り直さない。
        if not self.running:
            self.origin_distance = distance
            self.previous_heading = heading
            self.relative_heading = 0.0
            self.path_tracker = (PathTracking(self.cross_track_lookahead_mm, self.max_heading_correction_deg)
                                 if self.cross_track_lookahead_mm > 0 else None)
            self.last_log_time = None
            turn_limit = min(self.power, 100 - self.power)
            self.pid = PID(self.pid_p, self.pid_i, self.pid_d, setpoint=0,
                           sample_time=EXEC_INTERVAL,
                           output_limits=(-turn_limit, turn_limit))
            self.running = True
        progress = distance - self.origin_distance
        if progress < 0:
            return self._finish_distance_run(Status.FAILURE)
        # 359→0度等の折り返しを解除して開始からの連続角を得る。
        self.relative_heading += _normalize_heading_error(heading - self.previous_heading)
        self.previous_heading = heading

        # 2. 終了条件なし: 距離で成功。あり: 検知で成功、距離上限で失敗。
        if self.completion_condition is not None and progress >= self.completion_min_mm:
            self.completion_condition.tick_once()
            if self.completion_condition.status == Status.SUCCESS:
                return self._finish_distance_run(Status.SUCCESS)
        if progress >= self.distance_limit_mm:
            result = Status.SUCCESS if self.completion_condition is None else Status.FAILURE
            return self._finish_distance_run(result)

        # 3. featureから渡された関数をここで呼ぶ。単位はmm→度。
        nominal_heading = float(self.target(progress))
        if not math.isfinite(nominal_heading):
            return self._finish_distance_run(Status.FAILURE)
        # 推定横ずれがあるときは、計画の向きへ戻すため小さな追加角度を与える。
        correction = (self.path_tracker.correction(progress, self.relative_heading, self.target)
                      if self.path_tracker is not None else 0.0)
        self.target_heading = nominal_heading + correction
        turn_limit = min(self.power, 100-self.power)
        feedforward = curvature_turn(self.target, progress, self.power,
                                     self.wheel_tread_mm, self.feedforward_gain)
        feedforward = max(-turn_limit, min(turn_limit, feedforward))
        # 曲線に必要な出力を先に用意し、PIDは残った誤差を補正する。
        # FF分を差し引いた範囲に制限し、合計PWMの飽和時も積分を制限する。
        self.pid.output_limits = (-turn_limit-feedforward, turn_limit-feedforward)
        self.pid.setpoint = self.target_heading
        feedback = self.pid(self.relative_heading)
        turn = max(-turn_limit, min(turn_limit, feedforward+feedback))

        # 4. 基本旋回＋PID補正の合計を左右コースへ変換して出力する。
        runtime.right_motor.set_brake(False)
        runtime.left_motor.set_brake(False)
        runtime.right_motor.set_power(round(self.power + runtime.course * turn))
        runtime.left_motor.set_power(round(self.power - runtime.course * turn))
        now = time.monotonic()
        if self.last_log_time is None or now-self.last_log_time >= self.profile_log_interval_sec:
            p, i, d = self.pid.components
            xte = self.path_tracker.cross_track_mm if self.path_tracker else 0.0
            self.logger.info(
                'profile s=%.1f nominal=%.2f target=%.2f actual=%.2f error=%.2f '
                'xte_est=%.1f correction=%.2f ff=%.2f p=%.2f i=%.2f d=%.2f '
                'turn=%.2f left=%d right=%d' %
                (progress, nominal_heading, self.target_heading, self.relative_heading,
                 self.target_heading-self.relative_heading, xte, correction, feedforward, p, i, d,
                 turn, round(self.power-runtime.course*turn), round(self.power+runtime.course*turn)))
            self.last_log_time = now
        return Status.RUNNING

    def _finish_distance_run(self, status: Status) -> Status:
        # updateを直接呼ぶ場合も、終了時の出力0を保証する。
        self.terminate(status)
        return status


    def terminate(self, new_status: Status) -> None:
        for motor in (runtime.left_motor, runtime.right_motor):
            if motor is not None:
                motor.set_power(0)
        self.running = False
