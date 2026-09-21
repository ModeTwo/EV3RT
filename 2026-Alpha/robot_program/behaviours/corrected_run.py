"""ラップゲート通過後(ET相撲〜ETラリー)の直進用に、RunByGyro互換の引数で使えるET用の直進。

ETラリーで実績のあるEtRallyRunByGyro(behaviours/et_rally_drive.py)を、既存の直進クラス
(RunByGyro/LocalDrive/RunAtBearing)の呼び出し形に合わせて包んだもの。共通のgyro_drive.pyは変更しない。

向きのPIDは、呼び出し側の値ではなくETラリーで較正した値(ET_MOVE_PID)を使う。
USE_ET_STRAIGHT_PID=Falseにすると、呼び出し側のPID値をそのまま使う(倍率補正は有効なまま)。
"""

from ..runtime import runtime
from .et_rally_drive import EtRallyRunByGyro

# ETラリーの直進で較正した向きのPID(案B)。移動出力70〜80で調整した値。
ET_MOVE_PID = (4.0, 0.6, 0.06)
# 直進のPIDをETの値に統一するか。Falseなら、各工程が指定するPIDのまま(元の値)。
USE_ET_STRAIGHT_PID = True


class EtRun(EtRallyRunByGyro):
    """RunByGyroと同じ引数(name/target/power/pid_p/pid_i/pid_d/target_type)で使えるET用の直進。"""

    def __init__(self, name, target, power, pid_p, pid_i, pid_d, target_type, **kwargs):
        pid = ET_MOVE_PID if USE_ET_STRAIGHT_PID else (pid_p, pid_i, pid_d)
        super().__init__(name=name, target=target, power=power,
                         pid_p=pid[0], pid_i=pid[1], pid_d=pid[2],
                         target_type=target_type, **kwargs)


class LocalEtRun(EtRun):
    """TO区間用。TOの目標角(共通ジャイロ座標)をそのまま使う(section_motion.LocalDriveと同じ意味)。"""

    def __init__(self, name, context, target, **kwargs):
        self.context, self.local_target = context, target
        super().__init__(name=name, target=target, **kwargs)

    def update(self):
        from ..types import HeadingType
        if not self.running and self.target_type == HeadingType.ABSOLUTE:
            self.target = self.context.at_to.absolute_heading(self.local_target)
        return super().update()


class BrakeReleasingEtRun(EtRun):
    """単体開始時や直前工程の停止ブレーキを解除して走行する(catch_bottle.MarkerZeroDriveと同じ)。"""

    def update(self):
        runtime.require('left_motor', 'right_motor')
        runtime.left_motor.set_brake(False)
        runtime.right_motor.set_brake(False)
        return super().update()
