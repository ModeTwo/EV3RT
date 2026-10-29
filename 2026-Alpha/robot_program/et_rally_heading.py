"""ETラリー工程の間だけ使う、ジャイロの向きの読み取り窓口(2026-10-29)。

ScaledGyro(gyro_scale.py)を差し替えずに継承し、ETラリー工程の開始時にruntime.gyro_sensorへ入れ、
終了時に元のScaledGyroへ戻す(タイヤ径の切り替えと同じやり方)。ETラリーの各クラスは
runtime.gyro_sensor.get_angle()を読むだけなので、クラス側は変えずに次の2つが効く。

1. 四捨五入前の向きを使う(float_heading)
   etrobo_pythonのGyroSensor.get_angle()は、ハブの向き(hub_imu_get_heading()、小数)を
   round()で整数にして返す。ここではハブの値を直接読み、1度刻みにしない。

2. 止まっている間のずれを差し引く(drift_compensation)
   ハブ(Pybricks由来のpbio)は、全モーターが脱力しているときだけジャイロのくせを測り直す
   (imu.cのpbio_imu_is_stationary)。走行中はモーターにブレーキや出力が入っているので測り直されず、
   静止テストでは0.01〜0.05度/秒のずれが続いた。脱力をやめると、くせは10秒ほどで変わっていく
   (2026-10-29の静止テスト、全モーター脱力→全モーターブレーキ)ので、脱力中に測った値は使えない。
   そこで、走行中と同じ状態(ブレーキあり)で車輪とアームが止まっている間に、ハブの回転の速さ
   (z軸、くせを引いた後の値)を平均し、それをずれの速さとして経過時間ぶん差し引く。
   ハブの向きは時計回りが正で、回転の速さのz軸とは符号が逆(静止テストで確認)なので、
   ずれの速さ = -(z軸の平均)。
"""

import time

from py_trees.behaviour import Behaviour
from py_trees.common import Status

from .gyro_scale import ScaledGyro, ensure_scaled_gyro
from .runtime import runtime

# 止まっているとみなす条件: 左右の車輪のエンコーダが変わらず、z軸の回転の速さがこれ以下。
# アームは見ない(ETラリー開始時のアームを上げている約2秒も測れるようにするため。アームは上下に
# 動くだけで、旋回の軸(z軸)の平均はほぼ0になる想定。ログのsamplesと静止テストで確かめる)。
STILL_MAX_ANGV_DEG_S = 1.0
# 車輪が止まってから、この回数(読み取り)続いたら平均に入れ始める(止まった直後の揺れを外すため)。
STILL_SETTLE_SAMPLES = 5
# 読み取りの最小間隔(秒)。get_angle()は1tickに何度も呼ばれるので、平均に入れるのはこの間隔ごと。
SAMPLE_INTERVAL_SEC = 0.015
# ずれの速さの平均の重み。最初はただの平均、件数がこれを超えたら直近ほど重く(約この件数ぶんの平均)。
# 0.02秒ごとなら500件で約10秒ぶん。走行中に温度でずれの速さが変わっても追いかけられるようにする。
RATE_WINDOW_SAMPLES = 500


class EtRallyHeading(ScaledGyro):
    """ScaledGyroと同じ倍率補正に、四捨五入前の向きと、ずれの差し引きを加える。"""

    def __init__(self, base, float_heading, drift_compensation):
        super().__init__(base._sensor, base._scale, base._origin)
        self.base = base
        self.float_heading = float_heading
        self.drift_compensation = drift_compensation
        # ずれの速さ(ハブの向きの単位、度/秒)と、これまでに差し引いた量(度)。
        self.drift_rate = 0.0
        self.drift_total = 0.0
        self.rate_samples = 0
        self.last_time = time.monotonic()
        self.last_sample_time = 0.0
        self.prev_counts = None
        self.still_count = 0

    def _raw_heading(self):
        if self.float_heading and runtime.hub is not None:
            try:
                return float(runtime.hub.hub_imu_get_heading())
            except Exception:
                pass
        return float(self._sensor.get_angle())

    def _update_drift(self, now):
        # これまでのずれの速さで、前回からの経過時間ぶんの差し引きを足す。
        self.drift_total += self.drift_rate * (now - self.last_time)
        self.last_time = now
        if now - self.last_sample_time < SAMPLE_INTERVAL_SEC:
            return
        self.last_sample_time = now
        try:
            counts = (runtime.right_motor.get_count(), runtime.left_motor.get_count())
            angv_z = float(runtime.hub.hub_imu_get_angular_velocity()[2])
        except Exception:
            return
        if counts != self.prev_counts or abs(angv_z) > STILL_MAX_ANGV_DEG_S:
            self.prev_counts = counts
            self.still_count = 0
            return
        self.still_count += 1
        if self.still_count < STILL_SETTLE_SAMPLES:
            return
        self.rate_samples += 1
        weight = max(1.0 / self.rate_samples, 1.0 / RATE_WINDOW_SAMPLES)
        self.drift_rate += weight * (-angv_z - self.drift_rate)

    def get_angle(self):
        raw = self._raw_heading()
        if self.drift_compensation:
            self._update_drift(time.monotonic())
            raw -= self.drift_total
        return self._origin + (raw - self._origin) * self._scale

    def reset(self):
        super().reset()
        self.drift_total = 0.0

    def summary(self):
        return "float_heading=%s drift_compensation=%s drift_rate=%+.4f deg/s drift_total=%+.3f deg samples=%d" % (
            self.float_heading, self.drift_compensation, self.drift_rate, self.drift_total, self.rate_samples)


class InstallEtRallyHeading(Behaviour):
    """ETラリー工程の開始時に、runtime.gyro_sensorをEtRallyHeadingに差し替える。"""

    def __init__(self, name, float_heading, drift_compensation):
        super().__init__(name)
        self.float_heading = float_heading
        self.drift_compensation = drift_compensation

    def update(self):
        ensure_scaled_gyro()
        current = runtime.gyro_sensor
        if not isinstance(current, ScaledGyro):
            # 倍率補正が無効(DD-19較正用)のときは差し替えない。
            self.logger.info("et_rally heading not installed (gyro scale disabled)")
            return Status.SUCCESS
        if isinstance(current, EtRallyHeading):
            return Status.SUCCESS
        before = current.get_angle()
        runtime.gyro_sensor = EtRallyHeading(current, self.float_heading, self.drift_compensation)
        self.logger.info("et_rally heading installed: heading %.3f -> %.3f %s" % (
            before, runtime.gyro_sensor.get_angle(), runtime.gyro_sensor.summary()))
        return Status.SUCCESS


class LogEtRallyHeading(Behaviour):
    """ずれの速さと、差し引いた量をログに残す(周回の区切りなどに置く)。"""

    def __init__(self, name, tag):
        super().__init__(name)
        self.tag = tag

    def update(self):
        if isinstance(runtime.gyro_sensor, EtRallyHeading):
            self.logger.info("et_rally heading %s: %s" % (self.tag, runtime.gyro_sensor.summary()))
        return Status.SUCCESS


class RestoreEtRallyHeading(Behaviour):
    """ETラリー工程の終了時に、元のScaledGyroへ戻す(後続の工程に影響させない)。"""

    def update(self):
        current = runtime.gyro_sensor
        if isinstance(current, EtRallyHeading):
            self.logger.info("et_rally heading restored: %s" % current.summary())
            current.base._origin = current._origin
            runtime.gyro_sensor = current.base
        return Status.SUCCESS
