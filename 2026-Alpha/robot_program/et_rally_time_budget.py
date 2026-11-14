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
- 持ち時間(2026-11-14): --rally-time-total 120 なら、ETラリーの始めに 120 − (始めの時刻 − 競技の開始のタッチの時刻) を自動で出す。
  付けなければ --rally-time-budget の秒をそのまま使う(テストは70)。
- 点数: 3周まで1周5点、4周目から1周3点、時間内にガレージで止まると5点。1周(3点)のためにガレージ(5点)を落とさない。
"""

import math
import time

from py_trees.behaviour import Behaviour
from py_trees.common import Status

_t0 = None
_marks = {}      # 周 → その周の終わりの時刻
_drive_t0 = None  # 2026-11-12: 計画の最初の区間を走り出した時刻(1周目の比に使う)
# 2026-11-14: 持ち時間を競技の開始から自動で出す(--rally-time-total)。競技の開始 = 最初のタッチ(alpha.py の IsTouchOn)。
# 本番(--mission full)はタッチ1回で全工程を通すので、ETラリーの始めの時刻 − タッチの時刻 が「ETラリーを始めるまでの秒」。
_race_t0 = None
_total = None     # 競技の制限時間(秒、本番120)。None なら --rally-time-budget の値をそのまま使う
_budget = None    # start() で出した持ち時間(秒)
BUDGET_MIN_SEC = 10.0

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


def mark_race_start():
    """競技の開始(最初のタッチ)。2回目以降は無視する。"""
    global _race_t0
    if _race_t0 is None:
        _race_t0 = time.monotonic()


def set_total(total_sec):
    global _total
    _total = None if total_sec is None else float(total_sec)


def start():
    global _t0, _drive_t0, _budget
    _t0 = time.monotonic()
    _drive_t0 = None
    _marks.clear()
    if _total is not None and _race_t0 is not None:
        _budget = max(BUDGET_MIN_SEC, _total - (_t0 - _race_t0))
    else:
        _budget = None


def before_rally_sec():
    """競技の開始からETラリーの始めまでの秒(分からなければ None)。"""
    return None if (_t0 is None or _race_t0 is None) else _t0 - _race_t0


def current_budget(fallback):
    """持ち時間(秒)。--rally-time-total のときは 制限時間 − ETラリーを始めるまでの秒、
    競技の開始の時刻が分からないとき・使わないときは fallback(--rally-time-budget の値)。"""
    return _budget if _budget is not None else fallback


def mark_drive_start():
    global _drive_t0
    _drive_t0 = time.monotonic()


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
    """ETラリーの始めに置く。経過時間を数え始める。total_sec(--rally-time-total)があれば、持ち時間を
    total_sec − 競技の開始からの秒 にする。"""

    def __init__(self, name, total_sec=None):
        super().__init__(name)
        self.total_sec = total_sec

    def update(self):
        if self.total_sec is not None:
            set_total(self.total_sec)
        start()
        if self.total_sec is not None:
            if _budget is None:
                self.logger.warning("TIME BUDGET: race start (touch) unknown, using --rally-time-budget instead of total %.0fs"
                                    % self.total_sec)
            else:
                self.logger.info("TIME BUDGET: %.1fs before the ET rally, budget %.1fs of total %.0fs" % (
                    before_rally_sec(), _budget, self.total_sec))
        return Status.SUCCESS


class EtRallyTimeBudgetDriveStart(Behaviour):
    """2026-11-12: 計画の最初の区間の直前に置く。1周目の「実際/見込み」の比を、走り出してからの時間で出すため
    (スタート位置に着いてからのジャイロの測り直し・スタートの取り込みなど約3〜4秒が1周目に入り、比が1.33〜1.35倍に
    なっていた。持ち時間が短いと1周目の終わりで早めに切り上げてしまう)。経過時間(持ち時間との比べ)は今までどおり。"""

    def update(self):
        mark_drive_start()
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
        prev = _marks.get(self.lap - 1, _drive_t0 if _drive_t0 is not None else _t0)
        _marks[self.lap] = now
        lap_time = now - prev
        ratio = lap_time / self.est_this_lap
        el = now - _t0
        need = self.est_continue * ratio
        exit_now = self.est_exit * ratio
        self.budget = float(current_budget(self.budget))
        go_on = el + need + self.margin <= self.budget
        self.logger.info(
            "TIME BUDGET lap%d done: elapsed %.1fs (lap %.1fs, x%.2f of plan), continue needs %.1fs, exit now %.1fs, "
            "budget %.1fs margin %.1fs -> %s" % (
                self.lap, el, lap_time, ratio, need, exit_now, self.budget, self.margin,
                "CONTINUE to lap%d" % (self.lap + 1) if go_on else "GO TO GARAGE after %d laps" % self.lap))
        return Status.FAILURE if go_on else Status.SUCCESS
