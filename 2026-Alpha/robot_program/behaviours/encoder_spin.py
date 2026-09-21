"""ラップゲート通過後(ET相撲〜ETラリー)の旋回用に、SpinAround互換の引数で使えるエンコーダ旋回。

ETラリーで実績のあるEtRallySpinAroundByEncoder(behaviours/et_rally_drive.py)を、既存の旋回クラスの
呼び出し形に合わせて包んだもの。共通のgyro_drive.pyのSpinAroundは変更せず、この工程だけが差し替えて使う。

fine_trim(ジャイロでの仕上げ)を切り替えられる:
- fine_trim=True : フェーズ1(エンコーダで回して停止)のあと、残った誤差を低出力で細かく仕上げる(ETラリーと同じ)。
- fine_trim=False: フェーズ1だけで終える。ボトルを先端で運んでいる区間(ATのキャッチ〜配置)で、
                   小刻みな動きでボトルが離れないようにする。
"""

from py_trees.composites import Sequence

from ..gyro_scale import ensure_scaled_gyro
from ..runtime import runtime
from ..types import HeadingType
from .et_rally_drive import EtRallySpinAroundByEncoder
from .motor_control import StopNow

# ジャイロ仕上げの出力(ETラリーと同じ値)。フェーズ1の出力(main_power)より大きくならないよう丸める。
FINE_MAX_POWER = 60
FINE_MIN_POWER = 50


class EncoderSpin(EtRallySpinAroundByEncoder):
    """SpinAroundと同じ引数(max_power/min_power/pid/target_type/tolerance)で使えるエンコーダ旋回。
    max_powerがフェーズ1の出力、toleranceが仕上げの許容(fine_trim=Falseのときは使われない)。"""

    def __init__(self, name, target, max_power, min_power, pid_p, pid_i, pid_d, target_type,
                 tolerance=0.5, fine_trim=True):
        fine_max = min(FINE_MAX_POWER, max_power)
        fine_min = min(FINE_MIN_POWER, fine_max)
        super().__init__(
            name=name, target=target, main_power=max_power,
            fine_max_power=fine_max, fine_min_power=fine_min,
            pid_p=pid_p, pid_i=pid_i, pid_d=pid_d, target_type=target_type,
            fine_tolerance_deg=tolerance, fine_trim=fine_trim)


class LocalEncoderSpin(EncoderSpin):
    """TO区間用。TOの目標角(共通ジャイロ座標)をそのまま使う(section_motion.LocalSpinと同じ意味)。
    TOはボトルを運びながら走るため、既定でfine_trim=False。"""

    def __init__(self, name, context, target, fine_trim=False, **kwargs):
        self.context, self.local_target = context, target
        super().__init__(name=name, target=target, fine_trim=fine_trim, **kwargs)

    def update(self):
        if not self.running and self.target_type == HeadingType.ABSOLUTE:
            self.target = self.context.at_to.absolute_heading(self.local_target)
        return super().update()


class DeliveryEncoderTurn(EncoderSpin):
    """ボトル配置・ラリー準備用。配置ライン基準(delivery_heading)の目標角へ旋回する
    (delivery_turn.DeliveryPulseTurnと同じ目標角の求め方)。"""

    def __init__(self, name, context, target, power, max_power=None, fine_trim=True):
        self.context, self.delivery_target = context, target
        super().__init__(name=name, target=target, min_power=power,
                         max_power=power if max_power is None else max_power,
                         pid_p=0.2, pid_i=0.00075, pid_d=0.03,
                         target_type=HeadingType.ABSOLUTE, fine_trim=fine_trim)

    def update(self):
        if not self.running:
            # 補正済みのジャイロで目標角を求めるため、先に補正を有効にする。
            ensure_scaled_gyro()
            raw = runtime.gyro_sensor.get_angle()
            current = self.context.delivery_heading.heading(raw)
            self.target = -runtime.course * raw + self.delivery_target - current
        return super().update()


def delivery_encoder_turn(name, context, settings, target, fine_trim=True):
    """delivery_turn.delivery_turnのエンコーダ版。fine_trim=Falseはボトルを運んでいる区間用。"""
    root = Sequence(name=name, memory=True)
    root.add_children([
        DeliveryEncoderTurn(name + ' spin', context, target,
                            settings.to_spin_min_power, settings.to_spin_max_power,
                            fine_trim=fine_trim),
        StopNow(name=name + ' brake'),
    ])
    return root
