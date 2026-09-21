"""ラップゲート通過後のジャイロ倍率補正(GYRO_SCALE_FACTOR)を、読み取りの窓口で1か所に適用する。

ジャイロの生値は実際の回転量を約0.6%過少に報告する(py_etrobo_util/plotter.pyのGYRO_SCALE_FACTOR)。
向きを読む全クラスは runtime.gyro_sensor.get_angle() を使うため、その窓口を包むだけで、
既存のクラスを変更せずに、旋回・直進・カメラ・相撲の方位管理などが同じ補正済みの値を読む。

補正は「有効にした瞬間の生値(origin)」を起点にする:
    補正後 = origin + (生値 - origin) * GYRO_SCALE_FACTOR
有効にした瞬間の値は変わらないので、それ以前に登録した基準(delivery_heading、相撲のbearing、
AT_TOのhandoff)は、有効化の前後で連続したまま使える。ラップゲートまでの区間は補正しない。
"""

from py_trees.behaviour import Behaviour
from py_trees.common import Status

from py_etrobo_util.plotter import GYRO_SCALE_FACTOR

from .runtime import runtime


class ScaledGyro:
    """GyroSensorを包み、get_angle()だけ倍率補正して返す。他の属性・メソッドはそのまま委譲する。"""

    def __init__(self, sensor, scale, origin):
        self._sensor = sensor
        self._scale = scale
        self._origin = origin

    def get_angle(self):
        raw = self._sensor.get_angle()
        return self._origin + (raw - self._origin) * self._scale

    def reset(self):
        # ResetDeviceでジャイロが0に戻るため、起点も0に戻す(以降は0からの積算に倍率を掛ける)。
        self._sensor.reset()
        self._origin = 0.0

    def __getattr__(self, name):
        # get_angle/reset以外(ジャイロの他の取得メソッド等)は、実物のセンサーへ委譲する。
        return getattr(self._sensor, name)


def ensure_scaled_gyro():
    """runtime.gyro_sensorを補正済みの窓口にする。すでに補正済みなら何もしない(何度呼んでもよい)。
    戻り値: 今回新しく有効にしたらTrue。"""
    runtime.require("gyro_sensor")
    if isinstance(runtime.gyro_sensor, ScaledGyro):
        return False
    origin = float(runtime.gyro_sensor.get_angle())
    runtime.gyro_sensor = ScaledGyro(runtime.gyro_sensor, GYRO_SCALE_FACTOR, origin)
    return True


class EnableGyroScale(Behaviour):
    """このノード以降、ジャイロの読み取りに倍率補正を適用する(ラップゲート通過後の先頭に置く)。"""

    def __init__(self, name="enable gyro scale"):
        super().__init__(name)

    def update(self):
        enabled = ensure_scaled_gyro()
        self.logger.info(
            "gyro scale %s (factor=%.5f, origin=%.2f)" % (
                "enabled" if enabled else "already enabled", GYRO_SCALE_FACTOR,
                runtime.gyro_sensor._origin))
        return Status.SUCCESS
