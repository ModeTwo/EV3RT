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
   そこで、ETラリー開始時にアームを上げ終えたあと、振動が収まるのを待ってから止まったまま
   (車輪にはブレーキ)ハブの回転の速さ(z軸、くせを引いた後の値)を平均し、それをずれの速さとして
   ETラリーの間ずっと差し引く(EtRallyArmUpWithDriftMeasure)。
   2026-10-29: アームを上げている間に測ると、アームの動きと上の端に当たった衝撃でz軸が乱れ、
   ずれの速さを逆の符号で読んだ(-0.079度/秒、実際は+0.019度/秒)。アームが動いている間と、
   上げ切ったあとの振動の間(settle_sec)は測らない。ハブの向きは時計回りが正で、回転の速さのz軸とは符号が逆
   (静止テストで確認)なので、ずれの速さ = -(z軸の平均)。
   2026-10-29: 走行中の短い停止(旋回の前後)でも測っていたが、エンコーダが止まっていても歯車の
   あそびで車体がまだ回っており、ずれの速さを逆の符号で読んでいた(1周目の開始時に-0.26度/秒、
   静止テストでは毎回プラス)。走行中に測るのはやめ、開始時に測った値で固定する。
"""

import math
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
        # 2026-10-30: ETラリー用の位置(plotterと同じ座標・同じ式で、向きだけ四捨五入前の値を使う)。
        # plotterはジャイロの整数値で向きを計算するため、0.5度丸められると757mmの直進で約7mm横にずれる。
        self.pose_x = float(runtime.plotter.loc_x) if runtime.plotter is not None else 0.0
        self.pose_y = float(runtime.plotter.loc_y) if runtime.plotter is not None else 0.0
        self.pose_counts = None
        self.pose_azimuth = None

    def _update_pose(self, heading_deg):
        """左右のエンコーダの変化と、前回と今回の向きの中間で、位置を進める(plotter.pyと同じ式)。"""
        try:
            counts = (runtime.right_motor.get_count(), runtime.left_motor.get_count())
            diameter = runtime.plotter.tire_diameter
        except Exception:
            return
        azimuth = math.radians(heading_deg)
        if self.pose_counts is None:
            self.pose_counts = counts
            self.pose_azimuth = azimuth
            return
        if counts == self.pose_counts:
            self.pose_azimuth = azimuth
            return
        dist = math.pi * diameter * ((counts[0] - self.pose_counts[0]) + (counts[1] - self.pose_counts[1])) / 2.0 / 360.0
        mid = (self.pose_azimuth + azimuth) / 2.0
        self.pose_x += dist * math.sin(mid)
        self.pose_y += dist * math.cos(mid)
        self.pose_counts = counts
        self.pose_azimuth = azimuth

    def pose_text(self):
        return " et_loc=(%.1f,%.1f)" % (self.pose_x, self.pose_y)

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
        heading = self._origin + (raw - self._origin) * self._scale
        if self.float_heading:
            self._update_pose(heading)
        return heading

    def reset(self):
        super().reset()
        self.drift_total = 0.0

    def summary(self):
        return "float_heading=%s drift_compensation=%s drift_rate=%+.4f deg/s drift_total=%+.3f deg samples=%d" % (
            self.float_heading, self.drift_compensation, self.drift_rate, self.drift_total, self.rate_samples)


def et_pose_text():
    """ETラリー用の位置(四捨五入前の向きで計算)をログ用の文字列で返す。使っていなければ空文字。"""
    pose_text = getattr(runtime.gyro_sensor, "pose_text", None)
    return pose_text() if pose_text is not None else ""


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
    """ETラリー開始時にアームを上げ、上げ切ったあとsettle_sec秒待って振動を収めてから、measure_sec秒
    止まったままz軸の回転の速さを平均して、ずれの速さを決める。車輪には走行中と同じくブレーキをかける
    (モーターの状態でジャイロのくせが変わるため)。待っている間の揺れ幅もログに残す。"""

    def __init__(self, name, arm_up, settle_sec=1.0, measure_sec=2.0):
        super().__init__(name)
        self.arm_up = arm_up
        self.settle_sec = settle_sec
        self.measure_sec = measure_sec
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
            self.settle_min = None
            self.settle_max = None
            self.stationary_true = 0
            self.stationary_n = 0
        if self.arm_done_time is None:
            if self.arm_up.update() != Status.SUCCESS:
                return Status.RUNNING
            self.arm_done_time = now
            return Status.RUNNING
        elapsed = now - self.arm_done_time
        angv_z = self._read_angv_z()
        if elapsed < self.settle_sec:
            if angv_z is not None:
                self.settle_min = angv_z if self.settle_min is None else min(self.settle_min, angv_z)
                self.settle_max = angv_z if self.settle_max is None else max(self.settle_max, angv_z)
            return Status.RUNNING
        # ハブが「止まっている」と判定しているか(ハブのプログラムを書き換えた版では、ブレーキ中も
        # 判定が出てハブ自身がくせを測り直す。元の版では全モーター脱力のときだけ)。
        if runtime.hub is not None:
            try:
                self.stationary_n += 1
                if runtime.hub.hub_imu_is_stationary():
                    self.stationary_true += 1
            except Exception:
                pass
        if angv_z is not None:
            if abs(angv_z) > MEASURE_MAX_ANGV_DEG_S:
                self.rejected += 1
            else:
                self.values.append(angv_z)
        if elapsed < self.settle_sec + self.measure_sec:
            return Status.RUNNING
        self._finish(now)
        return Status.SUCCESS

    def _read_angv_z(self):
        if runtime.hub is None:
            return None
        try:
            return float(runtime.hub.hub_imu_get_angular_velocity()[2])
        except Exception:
            return None

    def _finish(self, now):
        n = len(self.values)
        mean = sum(self.values) / n if n else 0.0
        std = (sum((v - mean) ** 2 for v in self.values) / n) ** 0.5 if n else 0.0
        heading = runtime.gyro_sensor
        used = isinstance(heading, EtRallyHeading) and heading.drift_compensation and n >= MEASURE_MIN_SAMPLES
        if used:
            heading.set_drift_rate(-mean, n)
        settle_range = (self.settle_max - self.settle_min) if self.settle_min is not None else -1.0
        self.logger.info(
            "drift measure: %.2fs (arm %.2fs, settle %.1fs range=%.2f, measure %.1fs) samples=%d rejected=%d "
            "angv_z_mean=%+.4f std=%.3f imu_stationary=%d/%d -> drift_rate=%+.4f deg/s (%s)" % (
                now - self.start_time, self.arm_done_time - self.start_time, self.settle_sec, settle_range,
                self.measure_sec, n, self.rejected, mean, std, self.stationary_true, self.stationary_n,
                -mean if used else 0.0, "used" if used else "not used"))

    def terminate(self, new_status):
        self.started = False
