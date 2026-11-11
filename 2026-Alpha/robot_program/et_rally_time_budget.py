"""ETラリー専用: 経過時間で周回数を決める(2026-11-11、--rally-time-budget を付けたときだけ)。

本番は競技の制限時間120秒のうち、ETラリーを始めるまでに約50秒かかる。持ち時間(120 − 始めるまでの時間、
テストでは70秒)の中で、何周してガレージに止まるかを、走りながら決める。

- 経過時間の数え始め: ETラリーの始め(EtRallyTimeBudgetStart)。テストでスタート位置まで進むとき(--rally-pre-move-mm)は、
  スタート位置に着いた時刻(EtRallyArrivedAtStart から start() を呼ぶ)で数え直す。
- 判断(EtRallyLapDecision): N周目の黄ゲートを抜けたところ(計画の印 lapN-end)で、
    経過時間 + この先の見込み(次の1周 + そこからガレージまで、最後の周の前では最後の周 + ガレージまで) + 余裕
  が持ち時間を超えるなら、残りの周を飛ばして、計画の exit(N周目の黄の抜けからガレージまでのつなぎ)を走る。
  見込みは、計画の距離から出した時間(estimate_steps)に、直前の1周の実際の時間 ÷ その周の見込み を掛けて、
  その日の速さ(電圧で数%変わる)に合わせる。1周目は、スタートからの準備・つなぎも含めて数える(長めに出る=安全側)。
- 点数: 3周まで1周5点、4周目から1周3点、時間内にガレージで止まると5点。1周(3点)のためにガレージ(5点)を落とさない。
"""

import math
import time

from py_trees.behaviour import Behaviour
from py_trees.common import Status

_t0 = None
_marks = {}      # 周 → その周の終わりの時刻

# 見込みの時間の目安(出力100の実測、11/11)。日によって数%変わる分は、実際の周の時間との比で合わせる。
V_LINE = 460.0            # mm/s
ARC_K = 0.745
HALF_TREAD = 58.5
REV = 0.9
T_STOP = 0.4
T_ARC_FROM_REST = 0.45
T_TURN0, T_TURN_DEG = 0.87, 0.0047
T_STEP = 0.012
T_LINE_START = 0.08


def start():
    global _t0
    _t0 = time.monotonic()
    _marks.clear()


def elapsed():
    return None if _t0 is None else time.monotonic() - _t0


def estimate_steps(steps):
    """計画のステップ(checked のあと)の見込みの時間(秒)。"""
    t = 0.0
    prev_heading = None
    stopped = True
    for st in steps:
        typ = st.get("type")
        t += T_STEP
        if typ == "turn":
            d = abs(((st["target_heading_deg"] - (prev_heading if prev_heading is not None else st["target_heading_deg"]))
                     + 180.0) % 360.0 - 180.0)
            t += T_TURN0 + T_TURN_DEG * d
            prev_heading = st["target_heading_deg"]
            stopped = True
            continue
        rev = st.get("reverse", False)
        L = float(st.get("distance_mm") or 0.0)
        if typ == "arc" and not L:
            L = float(st.get("radius_mm", 0.0)) * math.radians(abs(float(st.get("sweep_deg", 0.0))))
        if typ == "arc":
            r = max(float(st.get("radius_mm", 80.0)), 1.0)
            v = ARC_K * V_LINE / (1.0 + HALF_TREAD / r)
            t += L / (v * (REV if rev else 1.0))
            if stopped:
                t += T_ARC_FROM_REST
        else:
            t += L / (V_LINE * (REV if rev else 1.0))
            if L >= 150.0:
                t += T_LINE_START
        if st.get("stop", True):
            t += T_STOP
            stopped = True
        else:
            stopped = False
        prev_heading = st.get("target_heading_deg", prev_heading)
    return t


class EtRallyTimeBudgetStart(Behaviour):
    """ETラリーの始めに置く。経過時間を数え始める。"""

    def update(self):
        start()
        return Status.SUCCESS


class EtRallyLapDecision(Behaviour):
    """N周目の終わりで、残りを走るか(FAILURE)、ガレージへ切り上げるか(SUCCESS)を決める。
    セレクタの左に [これ, 切り上げのつなぎ] を、右に [残りの周] を置く。"""

    def __init__(self, name, lap, laps_total, budget_sec, margin_sec, est_this_lap, est_continue, est_exit):
        super().__init__(name)
        self.lap = lap
        self.laps_total = laps_total
        self.budget = float(budget_sec)
        self.margin = float(margin_sec)
        self.est_this_lap = max(float(est_this_lap), 0.1)
        self.est_continue = float(est_continue)   # 次の周(+最後の周ならガレージまで、そうでなければ切り上げのつなぎ)
        self.est_exit = float(est_exit)

    def update(self):
        now = time.monotonic()
        if _t0 is None:
            start()
        prev = _marks.get(self.lap - 1, _t0)
        _marks[self.lap] = now
        lap_time = now - prev
        ratio = lap_time / self.est_this_lap
        el = now - _t0
        need = self.est_continue * ratio
        exit_now = self.est_exit * ratio
        go_on = el + need + self.margin <= self.budget
        self.logger.info(
            "TIME BUDGET lap%d done: elapsed %.1fs (lap %.1fs, x%.2f of plan), continue needs %.1fs, exit now %.1fs, "
            "budget %.1fs margin %.1fs -> %s" % (
                self.lap, el, lap_time, ratio, need, exit_now, self.budget, self.margin,
                "CONTINUE to lap%d" % (self.lap + 1) if go_on else "GO TO GARAGE after %d laps" % self.lap))
        return Status.FAILURE if go_on else Status.SUCCESS
