"""Behavior-tree lifecycle adapter for the background exchange."""

from py_trees.decorators import Decorator

from ..runtime import runtime
from .strategy_exchange import StrategyExchange


class WithStrategyExchange(Decorator):
    def __init__(self, child, context, config):
        super().__init__(name="mission with strategy exchange", child=child)
        self.context, self.config = context, config
        self.exchange = StrategyExchange(config.strategy_host, config.strategy_port,
                                         config.strategy_timeout_s)

    def initialise(self):
        # 固定plan互換モードではPC通信を起動しない。
        if self.config.et_rally_strategy_source == "received":
            self.exchange.start()

    def update(self):
        # 子のFeatureがcontext.hint1/hint2へ格納した周期に送信キューへ渡す。
        # 走行中でも受信キューを取り込み、No.11へ到着する前にSEQを保存できる。
        if self.config.et_rally_strategy_source == "received":
            self.exchange.poll(self.context, "right" if runtime.course == -1 else "left",
                               self.config.et_rally_laps)
        return self.decorated.status

    def terminate(self, new_status):
        self.exchange.close()
