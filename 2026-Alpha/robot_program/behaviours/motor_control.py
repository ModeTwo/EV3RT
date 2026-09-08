"""Reusable direct motor control behaviors."""

from py_trees.behaviour import Behaviour
from py_trees.common import Status

from ..runtime import runtime


class StopNow(Behaviour):
    # 左右の走行モーターを停止し、ブレーキを有効にする。
    def update(self) -> Status:
        runtime.require("plotter", "right_motor", "left_motor")
        runtime.right_motor.set_power(0)
        runtime.right_motor.set_brake(True)
        runtime.left_motor.set_power(0)
        runtime.left_motor.set_brake(True)
        self.logger.info(
            "%+06d %s.motors stopped"
            % (runtime.plotter.get_distance(), self.__class__.__name__)
        )
        return Status.SUCCESS


class RunAsInstructed(Behaviour):
    # 指定された左右PWMをそのまま出力し、他の終了条件が成立するまで走り続ける。
    # 2026baseから継続して使われている名称を維持する。
    def __init__(self, name: str, pwm_l: int, pwm_r: int) -> None:
        super().__init__(name)
        self.pwm_l = pwm_l
        self.pwm_r = pwm_r
        self.running = False

    def update(self) -> Status:
        runtime.require("plotter", "right_motor", "left_motor")
        # ツリー構築時にはcourseが未設定なので、実行時に左右コース符号を適用する。
        power_l = runtime.course * self.pwm_l
        power_r = runtime.course * self.pwm_r
        if not self.running:
            self.running = True
            self.logger.info(
                "%+06d %s.started with pwm=(%s, %s)"
                % (
                    runtime.plotter.get_distance(),
                    self.__class__.__name__,
                    power_l,
                    power_r,
                )
            )
        runtime.right_motor.set_power(power_r)
        runtime.left_motor.set_power(power_l)
        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        # Parallelの終了条件などで中断された場合に、直前のPWMを残さない。
        if runtime.right_motor is not None:
            runtime.right_motor.set_power(0)
        if runtime.left_motor is not None:
            runtime.left_motor.set_power(0)
        self.running = False

