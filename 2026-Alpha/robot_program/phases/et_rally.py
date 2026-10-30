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
from ..et_rally_heading import EtRallyArmUpWithDriftMeasure, InstallEtRallyHeading, RestoreEtRallyHeading
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
    # 2026-10-29: 四捨五入前の向き・止まっている間のずれの差し引き(robot_program/et_rally_heading.py)。
    float_heading = bool(getattr(config, "et_rally_gyro_float_heading", None))
    drift_compensation = bool(getattr(config, "et_rally_gyro_drift_compensation", None))
    # 2026-10-30: 左回り(headingが減る向き)だけの倍率。
    scale_neg = getattr(config, "et_rally_gyro_scale_neg", None)
    install_heading = gyro_tire_calibration and (float_heading or drift_compensation or scale_neg is not None)
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
    if install_heading:
        children.append(InstallEtRallyHeading("et_rally heading", float_heading, drift_compensation, scale_neg))
    arm_up = ArmUpDownFull(name="et_rally arm up", direction=ArmDirection.UP)
    # 2026-10-29: ハブのプログラムを書き換えた版(ブレーキ中もハブがジャイロのくせを測り直す)用に、
    # アームを上げ切って振動が収まってから、この秒数だけブレーキをかけて止まる。
    hub_recal_sec = getattr(config, "et_rally_gyro_hub_recal_sec", None)
    if hub_recal_sec and not (install_heading and drift_compensation):
        settle_sec = getattr(config, "et_rally_gyro_drift_settle_sec", None)
        arm_up = EtRallyArmUpWithDriftMeasure(
            "et_rally arm up", arm_up,
            settle_sec=1.0 if settle_sec is None else settle_sec,
            measure_sec=hub_recal_sec)
    if install_heading and drift_compensation:
        # 2026-10-29: アームを上げ切って振動が収まってから、止まってジャイロのずれの速さを測る
        # (et_rally_heading.py)。既定は振動待ち1秒+測定2秒。
        settle_sec = getattr(config, "et_rally_gyro_drift_settle_sec", None)
        measure_sec = getattr(config, "et_rally_gyro_drift_extra_sec", None)
        arm_up = EtRallyArmUpWithDriftMeasure(
            "et_rally arm up", arm_up,
            settle_sec=1.0 if settle_sec is None else settle_sec,
            measure_sec=2.0 if measure_sec is None else measure_sec)
    children.append(arm_up)
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
    if install_heading:
        children.append(RestoreEtRallyHeading("et_rally heading restore"))
    if not gyro_tire_calibration:
        # 後続の工程(同一プロセス内の別ミッション等)に影響させないよう元に戻す。
        children.append(SetGyroScaleDisabled("et_rally gyro scale re-enabled", False))
    root.add_children(children)
    return root
