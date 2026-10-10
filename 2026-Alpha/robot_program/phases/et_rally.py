"""ET rally phase composition."""

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence

from py_etrobo_util.plotter import ET_RALLY_TIRE_DIAMETER, TIRE_DIAMETER, UNCALIBRATED_TIRE_DIAMETER

from ..behaviours.device_control import ArmDirection, ArmUpDownFull
from ..features.execute_strategy import build_execute_strategy
from ..behaviours.motor_control import StopNow
from ..features.receive_strategy import build_receive_strategy
from .. import gyro_scale
from ..gyro_scale import EnableGyroScale
from ..runtime import runtime


class SetGyroScaleDisabled(Behaviour):
    # DD-19較正用: EtRallyRunByGyro等が内部で無条件に呼ぶensure_scaled_gyro()を、
    # この工程の間だけ無効化する(--rally-gyro-tire-calibration off)。
    def __init__(self, name, disabled):
        super().__init__(name)
        self.disabled = disabled

    def update(self):
        gyro_scale.set_disabled(self.disabled)
        self.logger.info("gyro scale disabled=%s" % self.disabled)
        return Status.SUCCESS


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
    # DD-19ベースライン計測用: Falseならジャイロ倍率補正・タイヤ径較正(DD-17)を適用しない。
    gyro_tire_calibration = getattr(config, "et_rally_gyro_tire_calibration_enabled", True)
    children = []
    if gyro_tire_calibration:
        children.extend([
            # 通しの実行ではラップゲート後に有効済み(何もしない)。ETラリーだけ単独で走らせる場合の保険。
            EnableGyroScale("et_rally gyro scale"),
            SetTireDiameter("et_rally tire diameter", ET_RALLY_TIRE_DIAMETER),
        ])
    else:
        # TIRE_DIAMETER(57.33)自体が既に2段階較正済みの値のため、「補正なし」の
        # ベースラインでは、較正前の最初の想定値(55.0)を明示的に使う。
        children.append(SetTireDiameter("et_rally tire diameter uncalibrated", UNCALIBRATED_TIRE_DIAMETER))
        # EtRallyRunByGyro等はフラグに関わらず内部でensure_scaled_gyro()を呼ぶため、
        # 窓口自体を無効化してジャイロ倍率補正を確実に無しにする。
        children.append(SetGyroScaleDisabled("et_rally gyro scale disabled", True))
    children.append(ArmUpDownFull(name="et_rally arm up", direction=ArmDirection.UP))
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
    # calibration有効時はET_RALLY_TIRE_DIAMETERから、無効時はUNCALIBRATED_TIRE_DIAMETERから、
    # いずれの場合も共通デフォルト(TIRE_DIAMETER)へ戻す。
    children.append(SetTireDiameter("et_rally tire diameter restore", TIRE_DIAMETER))
    if not gyro_tire_calibration:
        # 後続の工程(同一プロセス内の別ミッション等)に影響させないよう元に戻す。
        children.append(SetGyroScaleDisabled("et_rally gyro scale re-enabled", False))
    root.add_children(children)
    return root
