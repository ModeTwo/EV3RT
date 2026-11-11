"""走行中のモーター出力・バッテリー状態を、分析用ログとして一定間隔で記録する。

DD-19以降(補正なしベースラインからのやり直し)で、誤差の原因候補として
バッテリー出力に応じた調整の有無(5-5要素技術③)を検証するために追加。
hub.get_battery_voltage()/get_battery_current()は、これまでvideo.pyの
プレビュー表示でのみ使われており、ログへは出していなかった。

ログ形式はwireless_device/et_rally_lqi/parse_log.pyと同じ
"TAG key=value key=value ..." の行指向フォーマットに合わせている
(将来、同じ要領でパーサを追加しやすくするため)。
"""

from .runtime import runtime

TAG = "RALLY_PWR"

# 毎tick出すとログ量が多すぎるため、間引く(既定: 5tick=0.1秒に1回)。
_DEFAULT_INTERVAL_TICKS = 5
_tick_counts: dict = {}


def log_power_state(source: str, right_power: float, left_power: float,
                     interval_ticks: int = _DEFAULT_INTERVAL_TICKS) -> None:
    """直前にset_power()した値をそのまま渡してもらい、バッテリー状態と合わせて記録する。

    source: 呼び出し元のクラス名など(ログを見たときの区別用)。
    right_power/left_power: このtickで実際にモーターへ指示した出力値
        (モーター側にget_power()が無いため、呼び出し側が指示値をそのまま渡す)。
    """
    count = _tick_counts.get(source, 0) + 1
    _tick_counts[source] = count
    if count % interval_ticks != 0:
        return
    if runtime.hub is None:
        return
    try:
        battery_mv = runtime.hub.get_battery_voltage()
        battery_ma = runtime.hub.get_battery_current()
    except Exception:
        battery_mv = battery_ma = -1
    if battery_mv > 0:
        _battery_stats_add(battery_mv, battery_ma)
    dist = runtime.plotter.get_distance() if runtime.plotter is not None else -1
    print("%s source=%s dist=%d right_power=%+d left_power=%+d battery_mv=%d battery_ma=%d" % (
        TAG, source, dist, int(right_power), int(left_power), battery_mv, battery_ma))


# 2026-11-11: ETラリーの電池の記録(ハブから取れるのは電圧と電流だけ。充電の状態・電池の温度は取れない)。
# 同じ日の6周で、走り始めの電圧 8.37V→8.11V に合わせて巡航の速さが 434→404mm/s(約7%)落ちていた。
BATTERY_TAG = "RALLY_BATT"
# 走る前(止まっている間)の電圧がこれより低ければ、速さが落ちている(充電を勧める)と警告する。
BATTERY_LOW_MV = 8150
_battery_stats = {"n": 0, "min_mv": None, "sum_mv": 0, "max_ma": None, "sum_ma": 0}


def _battery_stats_add(mv, ma):
    st = _battery_stats
    st["n"] += 1
    st["sum_mv"] += mv
    st["sum_ma"] += ma
    st["min_mv"] = mv if st["min_mv"] is None else min(st["min_mv"], mv)
    st["max_ma"] = ma if st["max_ma"] is None else max(st["max_ma"], ma)


def reset_battery_stats() -> None:
    _battery_stats.update(n=0, min_mv=None, sum_mv=0, max_ma=None, sum_ma=0)


def read_battery():
    """(mV, mA)。読めなければ (-1, -1)。"""
    if runtime.hub is None:
        return -1, -1
    try:
        return runtime.hub.get_battery_voltage(), runtime.hub.get_battery_current()
    except Exception:
        return -1, -1


def battery_snapshot(label: str, warn_low: bool = False) -> int:
    """今の電圧・電流を1行出す。warn_low なら、BATTERY_LOW_MV 未満で警告も出す。戻り値は mV。"""
    mv, ma = read_battery()
    print("%s %s battery_mv=%d battery_ma=%d" % (BATTERY_TAG, label, mv, ma))
    if warn_low and 0 < mv < BATTERY_LOW_MV:
        print("%s WARNING battery low (%d mV < %d): motors are slower than when fully charged "
              "(11/11: 8.37V -> 8.11V was about 7%% slower). Charge the hub." % (BATTERY_TAG, mv, BATTERY_LOW_MV))
    return mv


def battery_summary(label: str) -> None:
    """reset_battery_stats() 以降に RALLY_PWR で読んだ電圧・電流のまとめを1行出す。"""
    st = _battery_stats
    if not st["n"]:
        print("%s %s no samples" % (BATTERY_TAG, label))
        return
    print("%s %s samples=%d min_mv=%d mean_mv=%.0f max_ma=%d mean_ma=%.0f" % (
        BATTERY_TAG, label, st["n"], st["min_mv"], st["sum_mv"] / st["n"], st["max_ma"], st["sum_ma"] / st["n"]))


def reset_power_log_counters() -> None:
    """ミッション間で間引きカウンタを引き継がないよう、必要なら呼び出す。"""
    _tick_counts.clear()


class OscillationTracker:
    """旋回の仕上げ(連続フィードバック)中に、出力の符号が何回反転したかを数える。

    5-3(解消しきれなかった誤差の分析)で見つかった「揺れの多い旋回ほど位置ズレが
    大きい」という相関を、次のラウンドでも定量的に確認できるようにするためのもの。
    バックラッシュ(不感帯の往復)が起きるたびに出力の符号が反転しやすいので、
    反転回数を「揺れの大きさ」の目安として使う。
    """

    def __init__(self) -> None:
        self._prev_sign = 0
        self.reversal_count = 0
        self.sample_count = 0

    def update(self, power: float) -> None:
        self.sample_count += 1
        sign = 1 if power > 0 else (-1 if power < 0 else 0)
        if sign != 0:
            if self._prev_sign != 0 and sign != self._prev_sign:
                self.reversal_count += 1
            self._prev_sign = sign
