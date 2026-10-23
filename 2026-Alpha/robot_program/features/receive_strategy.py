"""Feature 11 subtree factory."""

from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time
from ..behaviours.motor_control import StopNow
from ..et_rally_fallback import fallback_strategy


class WaitForStrategy(Behaviour):
    def __init__(self, context, fallback_enabled=False):
        super().__init__(name="wait for PC strategy")
        self.context = context
        self.fallback_enabled = fallback_enabled

    def _fail_or_fallback(self, reason):
        # ETラリー開始時にエラーが出たら、ミッションを止めず、固定ルートでゴールへ向かう。
        if not self.fallback_enabled:
            return Status.FAILURE
        self.context.strategy = fallback_strategy()
        self.context.selected_rally_laps = 0
        self.context.strategy_status = "ready"
        self.context.strategy_fallback_used = True
        self.logger.error("ETラリー開始時にエラー(%s)。固定ルートでゴールへ向かいます" % reason)
        return Status.SUCCESS

    def update(self):
        # socket.recvは呼ばず、通信スレッドが反映した状態だけを参照する。
        if self.context.strategy_status == "ready":
            return Status.SUCCESS
        if self.context.strategy_status == "failed":
            self.logger.error("Strategy unavailable: %s" % self.context.strategy_error)
            return self._fail_or_fallback(self.context.strategy_error)
        if not self.context.hint1 or not (
                self.context.hint2 or self.context.hint2_gate_info):
            self.logger.error("Both hints are required before starting ET rally")
            return self._fail_or_fallback("Hintが揃っていない")
        return Status.RUNNING


def build_receive_strategy(context, config):
    # No.11 PCからのSEQ受信と未受信時の縮退処理を担当する。
    root = Sequence(name="receive_strategy", memory=True)
    root.add_children([
        StopNow(name="stop before strategy wait"),
        WaitForStrategy(context, fallback_enabled=getattr(config, "et_rally_fallback_enabled", False)),
    ])
    return root
