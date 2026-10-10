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
    dist = runtime.plotter.get_distance() if runtime.plotter is not None else -1
    print("%s source=%s dist=%d right_power=%+d left_power=%+d battery_mv=%d battery_ma=%d" % (
        TAG, source, dist, int(right_power), int(left_power), battery_mv, battery_ma))


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
