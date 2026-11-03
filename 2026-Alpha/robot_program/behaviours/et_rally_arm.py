"""ETラリー専用: アームを機械的な限界まで動かして止める(ArmUpDownFullの止め方を変えた版)。

2026-11-03: 共通の ArmUpDownFull は、出力35で押し続け、「1tickに5度未満しか動かなかったtick」が
通算21回たまると止まる(連続でなくてよい)。実機では、限界に着いたあと歯車が飛んで(カチカチ)
エンコーダが動くため回数がたまりにくく、数回空回りしてから止まっていた。歯車が飛ぶたびにモーターの角度と
アームの位置の対応がずれ、止まった位置の値が毎回ばらばらだった(上げ -438/-610/-1145、下げたあと 16/13/-666)。

この版は、次のどれかで止める。
  - 動きが遅いtickが stop_ticks 回「連続」した(限界に着いた直後、歯車が飛ぶ前に止める)。
    遅いとは、1tickの動きが max(MIN_SLOW_DEG, SLOW_RATIO × それまでの一番速いtick) 未満のこと。
    走り出しの GRACE_TICKS は加速中なので数えない。
  - TIMEOUT_SEC を過ぎた(念のため)。
止めたら、出力0にしてブレーキをかける(従来どおり)。共通の ArmUpDownFull は変えず、
--rally-arm-stop-ticks を指定したときだけ、ETラリーの各所(起動時の上げ下げ・走行前の上げ)でこちらを使う。
"""

import time

from py_trees.behaviour import Behaviour
from py_trees.common import Status

from ..runtime import runtime
from .device_control import ArmUpDownFull


class EtRallyArmUpDown(Behaviour):
    GRACE_TICKS = 5
    MIN_SLOW_DEG = 2.0
    SLOW_RATIO = 0.3
    TIMEOUT_SEC = 4.0
    TRAIL_TICKS = 12   # ログに出す、止める直前の1tickごとの動き

    def __init__(self, name: str, direction, stop_ticks: int = 3, power: float = 35.0, slow_deg=None) -> None:
        super().__init__(name)
        self.direction = direction
        self.stop_ticks = max(1, int(stop_ticks))
        self.power = abs(float(power))
        # 11/03実機: 出力20ではふつうの動きが1〜3度/tickで、遅いの下限2度だと動いている途中で止まった(上げで45度だけ)。
        self.min_slow_deg = self.MIN_SLOW_DEG if slow_deg is None else float(slow_deg)
        self.running = False

    def update(self) -> Status:
        runtime.require("arm_motor", "plotter")
        now = time.monotonic()
        if not self.running:
            self.running = True
            self.start_time = now
            self.start_degree = self.prev_degree = runtime.arm_motor.get_count()
            self.ticks = 0
            self.peak = 0.0
            self.slow = 0
            self.trail = []
            self.logger.info("%+06d %s.start position is %d power=%d stop_ticks=%d" % (
                runtime.plotter.get_distance(), self.__class__.__name__, self.start_degree,
                self.power, self.stop_ticks))
            runtime.arm_motor.set_power(int(round(self.power)) * int(self.direction))
            return Status.RUNNING
        cur = runtime.arm_motor.get_count()
        delta = abs(cur - self.prev_degree)
        self.prev_degree = cur
        self.ticks += 1
        self.trail = (self.trail + [delta])[-self.TRAIL_TICKS:]
        if self.ticks > self.GRACE_TICKS:
            if delta < max(self.min_slow_deg, self.SLOW_RATIO * self.peak):
                self.slow += 1
            else:
                self.slow = 0
        if self.ticks > 1:
            # 11/03実機: 起動直後の1tick目に、ハブのモーター初期化と重なって74度の飛びが出て、
            # それを一番速い動きとしたため、ふつうに動いている(5度/tick)のに遅いと判定して止まった。
            self.peak = max(self.peak, delta)
        timed_out = now - self.start_time > self.TIMEOUT_SEC
        if self.slow >= self.stop_ticks or timed_out:
            runtime.arm_motor.set_power(0)
            runtime.arm_motor.set_brake(True)
            self.logger.info(
                "%+06d %s.position set to %d (moved %d in %.2fs, ticks=%d, peak=%d deg/tick%s) last=%s" % (
                    runtime.plotter.get_distance(), self.__class__.__name__, cur, cur - self.start_degree,
                    now - self.start_time, self.ticks, self.peak, ", TIMEOUT" if timed_out else "",
                    ",".join("%d" % d for d in self.trail)))
            return Status.SUCCESS
        return Status.RUNNING


class EtRallyArmAlreadyUp(Behaviour):
    """--rally-arm-up-before-touch のとき、ETラリー開始時のアーム上げの代わりに置く(タッチの前に上げ済み)。
    2026-11-03: 本番ではETラリーの走行位置へ向かう間にアームを上げ、止まってからはジャイロの測り直しだけをする想定。
    その時間を測るため、タッチの後はアームを動かさず、すぐ振動待ち・測定に入る。"""

    def update(self) -> Status:
        self.logger.info("arm already up before touch (position %d)" % runtime.arm_motor.get_count())
        return Status.SUCCESS


def arm_up_before_touch(config):
    return bool(getattr(config, "et_rally_arm_up_before_touch", None)) if config is not None else False


def make_arm_up_down(name, direction, config):
    """--rally-arm-stop-ticks があればこの版、なければ従来の ArmUpDownFull を返す。"""
    stop_ticks = getattr(config, "et_rally_arm_stop_ticks", None) if config is not None else None
    if not stop_ticks:
        return ArmUpDownFull(name=name, direction=direction)
    power = getattr(config, "et_rally_arm_power", None)
    return EtRallyArmUpDown(name, direction, stop_ticks=stop_ticks,
                            power=ArmUpDownFull.ARM_SHIFT_PWM if power is None else power,
                            slow_deg=getattr(config, "et_rally_arm_slow_deg", None))
