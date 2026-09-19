"""Reusable device initialization behaviors."""

from enum import IntEnum

from py_trees.behaviour import Behaviour
from py_trees.common import Status

from py_etrobo_util import TargetInterested

from ..runtime import runtime


class ArmDirection(IntEnum):
    UP = -1
    DOWN = 1


class ArmUpDownFull(Behaviour):
    # アームをどちらか一方の機械的な限界まで動かしてブレーキで固定する。
    # 元はキャリブレーション専用でalpha.pyに配置していたが、et_rally工程の
    # 開始時にも使うため、runtime経由の共通Behaviorとしてこちらへ移した。
    ARM_SHIFT_PWM = 35
    STALL_TOLERANCE = 5
    STALL_TICKS = 20

    def __init__(self, name: str, direction: ArmDirection) -> None:
        super().__init__(name)
        self.direction = direction
        self.running = False

    def update(self) -> Status:
        runtime.require("arm_motor", "plotter")
        if not self.running:
            self.running = True
            self.prev_degree = runtime.arm_motor.get_count()
            self.logger.info(
                "%+06d %s.start position is %d"
                % (runtime.plotter.get_distance(), self.__class__.__name__, self.prev_degree)
            )
            self.count = 0
            runtime.arm_motor.set_power(self.ARM_SHIFT_PWM * self.direction)
        else:
            cur_degree = runtime.arm_motor.get_count()
            if abs(cur_degree - self.prev_degree) < self.STALL_TOLERANCE:
                if self.count > self.STALL_TICKS:
                    runtime.arm_motor.set_power(0)
                    runtime.arm_motor.set_brake(True)
                    self.logger.info(
                        "%+06d %s.position set to %d"
                        % (runtime.plotter.get_distance(), self.__class__.__name__, cur_degree)
                    )
                    return Status.SUCCESS
                else:
                    self.count += 1
            self.prev_degree = cur_degree
        return Status.RUNNING


class ResetDevice(Behaviour):
    # 競技開始前にモーター・ジャイロ・カメラ設定を初期化する。
    # 走行途中に実行するとPlotterの累積値と不整合になるため、キャリブレーション専用とする。
    def __init__(
        self,
        name: str,
        gs_min: int = 0,
        gs_max: int = 55,
        stationary_samples: int = 4,
    ) -> None:
        super().__init__(name)
        self.gs_min = gs_min
        self.gs_max = gs_max
        self.stationary_samples = stationary_samples
        self.stationary_count = 0
        self.reset_started = False

    def update(self) -> Status:
        # 実行単位1：リセットに必要な共有デバイス参照が揃っていることを確認する。
        runtime.require(
            "hub",
            "arm_motor",
            "right_motor",
            "left_motor",
            "gyro_sensor",
            "plotter",
        )

        # 実行単位2：初回tickだけ実機デバイスとカメラ設定を初期化する。
        if not self.reset_started:
            runtime.arm_motor.reset_count()
            runtime.right_motor.reset_count()
            runtime.left_motor.reset_count()
            runtime.gyro_sensor.reset()
            if runtime.video is not None:
                runtime.video.set_thresholds(self.gs_min, self.gs_max)
                runtime.video.set_target_interested(TargetInterested.LINE)
            self.reset_started = True
            self.logger.info(
                "%+06d %s.resetting..."
                % (runtime.plotter.get_distance(), self.__class__.__name__)
            )
            self.logger.info(
                "%+06d %s.waiting for IMU to be stationary..."
                % (runtime.plotter.get_distance(), self.__class__.__name__)
            )

        # 実行単位3：IMUの静止を規定回数連続で確認する。
        if runtime.hub.hub_imu_is_stationary():
            self.stationary_count += 1
        else:
            self.stationary_count = 0

        # 実行単位4：静止確認完了後に次のキャリブレーション工程へ進む。
        if self.stationary_count >= self.stationary_samples:
            self.logger.info(
                "%+06d %s.complete"
                % (runtime.plotter.get_distance(), self.__class__.__name__)
            )
            return Status.SUCCESS
        return Status.RUNNING
