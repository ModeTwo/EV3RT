"""Reusable gyro drive behaviors."""

import time

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from simple_pid import PID

from py_etrobo_util import SymmetricClamper

from ..runtime import runtime
from ..types import HeadingType


from ..timing import CONTROL_INTERVAL_SEC as EXEC_INTERVAL


def _normalize_heading_error(error: float) -> float:
    # 角度差を-180度以上180度未満へ正規化する。
    return (error + 180.0) % 360.0 - 180.0


class SpinAround(Behaviour):
    # 指定した絶対角度または現在角度からの相対角度まで、その場で旋回する。
    def __init__(
        self,
        name: str,
        target: int,
        max_power: int,
        min_power: int,
        pid_p: float,
        pid_i: float,
        pid_d: float,
        target_type: HeadingType,
        tolerance: float = 2.0,
    ) -> None:
        super().__init__(name)
        self.target = target
        self.target_type = target_type
        self.pid_p = pid_p
        self.pid_i = pid_i
        self.pid_d = pid_d
        self.tolerance = tolerance
        self.clamper = SymmetricClamper(min_power, max_power)
        self.running = False
        self.target_heading = 0.0
        self.pid = None

    def update(self) -> Status:
        runtime.require("plotter", "gyro_sensor", "right_motor", "left_motor")
        current_heading = -runtime.course * runtime.gyro_sensor.get_angle()
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
            )
            self.running = True
            self.logger.info(
                "%+06d %s.spin started at heading=%d for %d"
                % (
                    runtime.plotter.get_distance(),
                    self.__class__.__name__,
                    current_heading,
                    self.target_heading,
                )
            )

        error = _normalize_heading_error(self.target_heading - current_heading)
        if abs(error) < self.tolerance:
            self.logger.info(
                "%+06d %s.spin ended at heading=%d"
                % (
                    runtime.plotter.get_distance(),
                    self.__class__.__name__,
                    current_heading,
                )
            )
            return Status.SUCCESS

        # PIDの出力方向は維持しつつ、角度境界をまたぐ場合は正規化した誤差方向を採用する。
        raw_power = float(self.pid(current_heading))
        if raw_power == 0.0:
            raw_power = error
        elif raw_power * error < 0.0:
            raw_power = -raw_power
        power = int(self.clamper.clamp(raw_power))
        runtime.right_motor.set_brake(False)
        runtime.left_motor.set_brake(False)
        runtime.right_motor.set_power(runtime.course * power)
        runtime.left_motor.set_power(-runtime.course * power)
        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        # 旋回完了または中断時にモーター出力を残さない。
        if runtime.right_motor is not None:
            runtime.right_motor.set_power(0)
        if runtime.left_motor is not None:
            runtime.left_motor.set_power(0)
        self.running = False


class RunByGyro(Behaviour):
    # 指定した絶対角度または相対角度を維持しながら直進する。
    def __init__(
        self,
        name: str,
        target: int,
        power: int,
        pid_p: float,
        pid_i: float,
        pid_d: float,
        target_type: HeadingType,
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

    def update(self) -> Status:
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


    def terminate(self, new_status: Status) -> None:
        for motor in (runtime.left_motor, runtime.right_motor):
            if motor is not None:
                motor.set_power(0)
        self.running = False
