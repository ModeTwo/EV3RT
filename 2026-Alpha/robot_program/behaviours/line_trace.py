"""Reusable line trace behaviors."""

import math

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from simple_pid import PID

from py_etrobo_util import LowPassFilter, TraceSide

from ..runtime import runtime


EXEC_INTERVAL = 0.02


class TraceLine(Behaviour):
    # カラーセンサーの明度を使ってライン端を追従する。
    def __init__(
        self,
        name: str,
        target: int,
        power: int,
        pid_p: float,
        pid_i: float,
        pid_d: float,
        trace_side: TraceSide,
        cutoff_hz: float = 12.0,
        median_window: int = 0,
        power_min: int = None,
        err_lo: float = 6.0,
        err_hi: float = 22.0,
        accel_per_s: float = 60.0,
        decel_per_s: float = 180.0,
        metric_hz: float = 2.0,
        gains_slow: tuple = None,
        gains_fast: tuple = None,
        recover_v: int = None,
        recover_after: int = 3,
        recover_turn: int = None,
    ) -> None:
        super().__init__(name)
        self.power_max = power
        self.power_min = power if power_min is None else power_min
        self.power = power
        self.adapt = power_min is not None
        self.target = target
        self.pid = PID(
            pid_p,
            pid_i,
            pid_d,
            setpoint=target,
            sample_time=EXEC_INTERVAL,
            output_limits=(-self.power_max, self.power_max),
        )
        self.trace_side = trace_side
        self.lpf = (
            LowPassFilter(cutoff_hz, EXEC_INTERVAL, median_window)
            if cutoff_hz
            else None
        )
        self.err_lo = err_lo
        self.err_hi = err_hi
        self.metric_lpf = LowPassFilter(metric_hz, EXEC_INTERVAL)
        self.err_metric = 0.0
        self.accel_step = accel_per_s * EXEC_INTERVAL
        self.decel_step = decel_per_s * EXEC_INTERVAL
        self.gains_slow = gains_slow
        self.gains_fast = gains_fast
        self.schedule = (
            gains_slow is not None
            and gains_fast is not None
            and self.power_max > self.power_min
        )
        self.recover_v = recover_v
        self.recover_after = recover_after
        self.recover_turn = recover_turn
        self._lost_count = 0
        self.running = False

    def update(self) -> Status:
        runtime.require("plotter", "color_sensor", "right_motor", "left_motor")
        if not self.running:
            if self.lpf:
                self.lpf.reset()
            self.metric_lpf.reset()
            self.running = True
            self.logger.info(
                "%+06d %s.trace started with TS=%s"
                % (
                    runtime.plotter.get_distance(),
                    self.__class__.__name__,
                    self.trace_side.name,
                )
            )

        _, _, v_raw = runtime.color_sensor.get_raw_color_hsv()
        v = self.lpf(v_raw) if self.lpf else v_raw

        # 追従誤差を平滑化し、直線では加速、曲線では減速する。
        self.err_metric = self.metric_lpf(abs(self.target - v_raw))
        if self.adapt:
            frac = (self.err_metric - self.err_lo) / (self.err_hi - self.err_lo)
            frac = max(0.0, min(1.0, frac))
            target_power = self.power_max - frac * (self.power_max - self.power_min)
            delta_power = target_power - self.power
            if delta_power > self.accel_step:
                delta_power = self.accel_step
            elif delta_power < -self.decel_step:
                delta_power = -self.decel_step
            self.power += delta_power

        if self.schedule:
            factor = (self.power - self.power_min) / (self.power_max - self.power_min)
            factor = max(0.0, min(1.0, factor))
            kp_now = self.gains_slow[0] + factor * (
                self.gains_fast[0] - self.gains_slow[0]
            )
            kd_now = self.gains_slow[1] + factor * (
                self.gains_fast[1] - self.gains_slow[1]
            )
            self.pid.tunings = (kp_now, self.pid.Ki, kd_now)

        if self.trace_side == TraceSide.NORMAL:
            turn = -runtime.course * int(self.pid(v))
        else:
            turn = runtime.course * int(self.pid(v))

        # 明るい路面が連続した場合は、ラインを再取得するまで旋回量を強める。
        if self.recover_v is not None:
            if v_raw >= self.recover_v:
                self._lost_count += 1
            else:
                self._lost_count = 0
            if self._lost_count >= self.recover_after and turn != 0:
                magnitude = (
                    self.power_max if self.recover_turn is None else self.recover_turn
                )
                turn = int(math.copysign(magnitude, turn))

        base_power = int(round(self.power))
        left_power = max(-100, min(100, base_power + turn))
        right_power = max(-100, min(100, base_power - turn))
        runtime.right_motor.set_power(right_power)
        runtime.left_motor.set_power(left_power)
        return Status.RUNNING

