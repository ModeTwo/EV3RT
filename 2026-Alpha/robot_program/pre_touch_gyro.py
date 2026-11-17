"""2026-11-17(ユーザー): タッチ(競技の開始)を待つ間に、止まっているジャイロのずれを測る。

タッチを待つ間ずっと読み続け、押された直前の WINDOW_SEC 秒(押す瞬間の揺れの EXCLUDE_SEC 秒は除く)から、
z軸の回転の速さの平均と、ハブの向き(小数)の変化の速さの2通りでずれを出してログに書く。待ち時間は増えない。
結果は RESULT に残す(今は測ってログに出すだけで、向きの差し引きには使っていない)。
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

# 最後の測定結果: dict(rate_angv, rate_heading, samples, window, stationary, std) または None
RESULT = None


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
        # 符号: ずれの速さ = -(z軸の速さの平均)(et_rally_heading と同じ)。向きの変化の速さも同じ向きのはず
        self.logger.info(
            "pre-touch gyro drift: window %.2fs samples=%d rejected=%d angv_z_mean=%+.4f std=%.3f -> drift %+.4f deg/s, "
            "heading change %+.4f deg/s (%+.3f deg) imu_stationary=%d/%d (measured only, not subtracted)" % (
                span, n, len(rows) - n, mean, std, -mean, rate_heading, rows[-1][2] - rows[0][2],
                sum(st), len(st)))
