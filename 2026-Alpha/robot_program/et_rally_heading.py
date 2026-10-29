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
   そこで、ETラリー開始時にアームを上げている間(約2秒、車輪にはブレーキ)に、ハブの回転の速さ
   (z軸、くせを引いた後の値)を平均し、それをずれの速さとしてETラリーの間ずっと差し引く
   (EtRallyArmUpWithDriftMeasure)。ハブの向きは時計回りが正で、回転の速さのz軸とは符号が逆
   (静止テストで確認)なので、ずれの速さ = -(z軸の平均)。
   2026-10-29: 走行中の短い停止(旋回の前後)でも測っていたが、エンコーダが止まっていても歯車の
   あそびで車体がまだ回っており、ずれの速さを逆の符号で読んでいた(1周目の開始時に-0.26度/秒、
   静止テストでは毎回プラス)。走行中に測るのはやめ、開始時に測った値で固定する。
"""

import time

from py_trees.behaviour import Behaviour
from py_trees.common import Status

from .gyro_scale import ScaledGyro, ensure_scaled_gyro
from .runtime import runtime

# 平均に入れるz軸の回転の速さの上限(度/秒)。アームの動きによる大きな揺れの読み取りを外す。
# アームは上下に動くだけで、旋回の軸(z軸)への影響は平均すれば小さい想定(ログのstd・rejectedで確かめる)。
MEASURE_MAX_ANGV_DEG_S = 1.0
# この件数に満たなければ、ずれの速さは0のまま(差し引かない)にする。
MEASURE_MIN_SAMPLES = 30


class EtRallyHeading(ScaledGyro):
    """ScaledGyroと同じ倍率補正に、四捨五入前の向きと、ずれの差し引きを加える。"""

    def __init__(self, base, float_heading, drift_compensation):
        super().__init__(base._sensor, base._scale, base._origin)
        self.base = base
        self.float_heading = float_heading
        self.drift_compensation = drift_compensation
        # ずれの速さ(ハブの向きの単位、度/秒)と、これまでに差し引いた量(度)。
        # ずれの速さはset_drift_rate()で決まるまで0。決まったら、差し替えた時点から差し引く。
        self.drift_rate = 0.0
        self.drift_total = 0.0
        self.rate_samples = 0
        self.installed_time = time.monotonic()
        self.last_time = self.installed_time

    def _raw_heading(self):
        if self.float_heading and runtime.hub is not None:
            try:
                return float(runtime.hub.hub_imu_get_heading())
            except Exception:
                pass
        return float(self._sensor.get_angle())

    def _update_drift(self, now):
        # ずれの速さで、前回からの経過時間ぶんの差し引きを足す。
        self.drift_total += self.drift_rate * (now - self.last_time)
        self.last_time = now

    def set_drift_rate(self, rate, samples):
        """ずれの速さを決める。差し替えた時点から今までの分も、まとめて差し引く。"""
        now = time.monotonic()
        self.drift_rate = rate
        self.rate_samples = samples
        self.drift_total = rate * (now - self.installed_time)
        self.last_time = now

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


class EtRallyArmUpWithDriftMeasure(Behaviour):
    """ETラリー開始時にアームを上げながら、その間のz軸の回転の速さを平均して、ずれの速さを決める。
    車輪には走行中と同じくブレーキをかける(モーターの状態でジャイロのくせが変わるため)。
    extra_secを指定すると、アームを上げ終えたあと、その秒数だけ止まったまま測り続ける。"""

    def __init__(self, name, arm_up, extra_sec=0.0):
        super().__init__(name)
        self.arm_up = arm_up
        self.extra_sec = extra_sec
        self.started = False

    def update(self):
        now = time.monotonic()
        if not self.started:
            self.started = True
            for motor in (runtime.right_motor, runtime.left_motor):
                motor.set_power(0)
                motor.set_brake(True)
            self.start_time = now
            self.arm_done_time = None
            self.values = []
            self.rejected = 0
        self._sample()
        if self.arm_done_time is None:
            if self.arm_up.update() != Status.SUCCESS:
                return Status.RUNNING
            self.arm_done_time = now
        if now - self.arm_done_time < self.extra_sec:
            return Status.RUNNING
        self._finish(now)
        return Status.SUCCESS

    def _sample(self):
        if runtime.hub is None:
            return
        try:
            angv_z = float(runtime.hub.hub_imu_get_angular_velocity()[2])
        except Exception:
            return
        if abs(angv_z) > MEASURE_MAX_ANGV_DEG_S:
            self.rejected += 1
            return
        self.values.append(angv_z)

    def _finish(self, now):
        n = len(self.values)
        mean = sum(self.values) / n if n else 0.0
        std = (sum((v - mean) ** 2 for v in self.values) / n) ** 0.5 if n else 0.0
        heading = runtime.gyro_sensor
        used = isinstance(heading, EtRallyHeading) and heading.drift_compensation and n >= MEASURE_MIN_SAMPLES
        if used:
            heading.set_drift_rate(-mean, n)
        self.logger.info(
            "drift measure: %.2fs (arm %.2fs) samples=%d rejected=%d angv_z_mean=%+.4f std=%.3f "
            "-> drift_rate=%+.4f deg/s (%s)" % (
                now - self.start_time, self.arm_done_time - self.start_time, n, self.rejected, mean, std,
                -mean if used else 0.0, "used" if used else "not used"))

    def terminate(self, new_status):
        self.started = False
