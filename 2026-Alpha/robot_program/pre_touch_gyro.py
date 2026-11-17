"""2026-11-17(ユーザー): タッチ(競技の開始)を待つ間に、止まっているジャイロのずれを測る。

タッチを待つ間ずっと読み続け、押された直前の WINDOW_SEC 秒(押す瞬間の揺れの EXCLUDE_SEC 秒は除く)から、
z軸の回転の速さの平均と、ハブの向き(小数)の変化の速さの2通りでずれを出してログに書く。待ち時間は増えない。
結果は RESULT に残す。

2026-11-17(ユーザー): 測ったずれをタッチの時点から差し引く(ENABLED、--pre-touch-gyro-drift off で切る)。
ジャイロの実物(GyroSensor)の get_angle を差し替えるので、alpha.py の g_gyro_sensor・plotter・runtime.gyro_sensor
(ScaledGyro の中身)が同じ値を読む。reset() で差し引きの起点も今に戻す。ETラリーの四捨五入前の向き
(EtRallyHeading がハブから直接読む)は drift_total() を引く。ETラリーの始めのずれの差し引き
(--rally-gyro-drift-comp on)を使うときは、その時点で止める(stop_at)。
"""

import collections
import time

from py_trees.behaviour import Behaviour
from py_trees.common import Status

from .runtime import runtime

WINDOW_SEC = 3.0
EXCLUDE_SEC = 0.3
MAX_ANGV_DEG_S = 1.0   # これより大きい読みは揺れとして外す(et_rally_heading.MEASURE_MAX_ANGV_DEG_S と同じ)
MIN_SAMPLES = 30

MAX_RATE_DEG_S = 0.1   # これより大きいずれは、動いた・触ったなどとみなして差し引かない(過去の値は ±0.03 以内)

# 最後の測定結果: dict(rate_angv, rate_heading, samples, window, stationary, std) または None
RESULT = None
ENABLED = True
# 差し引きの状態: 速さ(度/秒、ハブの向きの単位。差し引く量 = RATE × 経過秒)、起点の時刻、止めた時刻
RATE = 0.0
_T0 = None
_STOP = None


def drift_total(now=None):
    """タッチ(または最後の reset)から今までに差し引く量(度)。"""
    if _T0 is None or RATE == 0.0:
        return 0.0
    now = time.monotonic() if now is None else now
    if _STOP is not None:
        now = min(now, _STOP)
    return RATE * max(0.0, now - _T0)


def stop_at(t):
    """この時刻より後は増やさない(ETラリーの始めに測った速さに切り替えるとき)。"""
    global _STOP
    if _T0 is not None and _STOP is None:
        _STOP = t


def _install(sensor, rate, logger):
    global RATE, _T0, _STOP
    if getattr(sensor, "_pre_touch_patched", False):
        RATE, _T0, _STOP = rate, time.monotonic(), None
        return
    orig_get = sensor.get_angle
    orig_reset = sensor.reset

    def get_angle():
        return orig_get() - drift_total()

    def reset():
        global _T0, _STOP
        orig_reset()
        if _T0 is not None:
            _T0, _STOP = time.monotonic(), None

    sensor.get_angle = get_angle
    sensor.reset = reset
    sensor._pre_touch_patched = True
    RATE, _T0, _STOP = rate, time.monotonic(), None


class PreTouchGyroDrift(Behaviour):
    """タッチ待ちと並べて置く。ずっと RUNNING を返し、タッチで止められたときに直前の窓で計算する。"""

    def __init__(self, name="pre-touch gyro drift"):
        super().__init__(name)
        self.buf = collections.deque()
        self.started = False

    def initialise(self):
        self.buf.clear()
        self.started = True

    def update(self):
        hub = runtime.hub
        if hub is None:
            return Status.RUNNING
        now = time.monotonic()
        try:
            angv = float(hub.hub_imu_get_angular_velocity()[2])
            heading = float(hub.hub_imu_get_heading())
        except Exception:
            return Status.RUNNING
        try:
            stationary = bool(hub.hub_imu_is_stationary())
        except Exception:
            stationary = None
        self.buf.append((now, angv, heading, stationary))
        keep = WINDOW_SEC + EXCLUDE_SEC + 0.5
        while self.buf and now - self.buf[0][0] > keep:
            self.buf.popleft()
        return Status.RUNNING

    def terminate(self, new_status):
        if not self.started:
            return
        self.started = False
        global RESULT
        if not self.buf:
            self.logger.info("pre-touch gyro drift: no samples")
            return
        end = time.monotonic() - EXCLUDE_SEC
        rows = [r for r in self.buf if end - WINDOW_SEC <= r[0] <= end]
        if len(rows) < MIN_SAMPLES:
            self.logger.info("pre-touch gyro drift: only %d samples (touched too early?)" % len(rows))
            return
        values = [r[1] for r in rows if abs(r[1]) <= MAX_ANGV_DEG_S]
        n = len(values)
        mean = sum(values) / n if n else 0.0
        std = (sum((v - mean) ** 2 for v in values) / n) ** 0.5 if n else 0.0
        span = rows[-1][0] - rows[0][0]
        rate_heading = (rows[-1][2] - rows[0][2]) / span if span > 0 else 0.0
        st = [r[3] for r in rows if r[3] is not None]
        RESULT = dict(rate_angv=-mean, rate_heading=rate_heading, samples=n, window=span,
                      stationary=(sum(st), len(st)), std=std)
        if not ENABLED:
            how = "measured only (--pre-touch-gyro-drift off)"
        elif abs(mean) > MAX_RATE_DEG_S:
            how = "too large, not subtracted"
        elif runtime.gyro_sensor is None:
            how = "no gyro, not subtracted"
        else:
            sensor = runtime.gyro_sensor
            while hasattr(sensor, "_sensor") and hasattr(sensor, "_scale"):   # ScaledGyro などの包みを外して実物へ
                sensor = sensor._sensor
            try:
                _install(sensor, -mean, self.logger)
                how = "subtracted from now on"
            except Exception as error:
                how = "could not install (%s), not subtracted" % error
        # 符号: ずれの速さ = -(z軸の速さの平均)(et_rally_heading と同じ)。向きの変化の速さも同じ向きのはず
        self.logger.info(
            "pre-touch gyro drift: window %.2fs samples=%d rejected=%d angv_z_mean=%+.4f std=%.3f -> drift %+.4f deg/s, "
            "heading change %+.4f deg/s (%+.3f deg) imu_stationary=%d/%d (%s)" % (
                span, n, len(rows) - n, mean, std, -mean, rate_heading, rows[-1][2] - rows[0][2],
                sum(st), len(st), how))
