"""ET rally phase composition."""

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence

from py_etrobo_util.plotter import ET_RALLY_TIRE_DIAMETER, TIRE_DIAMETER

from ..behaviours.device_control import ArmDirection, ArmUpDownFull
from ..features.execute_strategy import build_execute_strategy
from ..behaviours.motor_control import StopNow
from ..features.receive_strategy import build_receive_strategy
from ..gyro_scale import EnableGyroScale
from ..runtime import runtime


class SetTireDiameter(Behaviour):
    # ETラリー専用のタイヤ径較正を、この工程の間だけ走行距離計算へ適用する。
    def __init__(self, name, diameter):
        super().__init__(name)
        self.diameter = diameter

    def update(self):
        runtime.require("plotter")
        runtime.plotter.tire_diameter = self.diameter
        self.logger.info("Plotter tire diameter set to %.2f mm" % self.diameter)
        return Status.SUCCESS


def build_et_rally_phase(context, config):
    # PCまたは固定planが全周回分を一つのSEQとして返すため、SEQは一度だけ実行する。
    # ボトル配送までアームを下げたまま運んでいるため、ETラリー走行の前に上げ直す。
    root = Sequence(name="et_rally", memory=True)
    children = [
        # 通しの実行ではラップゲート後に有効済み(何もしない)。ETラリーだけ単独で走らせる場合の保険。
        EnableGyroScale("et_rally gyro scale"),
        SetTireDiameter("et_rally tire diameter", ET_RALLY_TIRE_DIAMETER),
        ArmUpDownFull(name="et_rally arm up", direction=ArmDirection.UP),
    ]
    if config.et_rally_strategy_source == "received":
        children.append(build_receive_strategy(context, config))
    elif config.et_rally_strategy_source != "file":
        raise ValueError(
            "Unknown ET rally strategy source: " + str(config.et_rally_strategy_source)
        )
    children.append(build_execute_strategy(context, config))
    # ET用の直進は、区間が連続するゲート通過中に止まらないよう終了時のブレーキを持たない。
    # 最後の区間の後は、ここで明示的に停止する(単体テストでは、この後は何も動かさないため必須)。
    children.append(StopNow(name="et_rally final stop"))
    children.append(SetTireDiameter("et_rally tire diameter restore", TIRE_DIAMETER))
    root.add_children(children)
    return root
