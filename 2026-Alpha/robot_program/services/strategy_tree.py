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

    def _local_poll(self):
        """2026-11-17(ユーザー): "local" のとき、ヒント1・2が揃ったら、裏のスレッドでヒント2を復号し(PBKDF2・AES は重い)、
        前もって作った計画(~/et_rally_plans の plan_<鍵>.json か plans.zip)を選ぶ。結果はこのスレッド(BT)で context に入れる。"""
        import queue
        import threading
        ctx = self.context
        if getattr(self, "_local_queue", None) is None:
            self._local_queue = queue.Queue()
            self._local_started = False
        try:
            kind, value = self._local_queue.get_nowait()
        except queue.Empty:
            kind = None
        if kind == "ok":
            gate_info, path, how = value
            ctx.hint2_gate_info = gate_info
            ctx.strategy_plan_path = str(path)
            ctx.strategy_plan_source = "robot %s" % how
            ctx.strategy_status = "ready"
            print(" -- ET rally local plan: %s (hint1=%s hint2=%s)" % (how, ctx.hint1, gate_info), flush=True)
        elif kind == "error":
            ctx.strategy_status = "failed"
            ctx.strategy_error = value
            print(" -- ET rally local plan failed: %s" % value, flush=True)
        if self._local_started or ctx.strategy_status in ("ready", "failed"):
            return
        if not ctx.hint1 or not (ctx.hint2 or ctx.hint2_gate_info):
            return
        self._local_started = True
        ctx.strategy_status = "pending"
        hint1, raw, decoded, key = ctx.hint1, ctx.hint2, ctx.hint2_gate_info, getattr(ctx, "decryption_key", None)
        q = self._local_queue

        def work():
            try:
                gate_info = decoded
                if not gate_info:
                    if not key:
                        raise ValueError("decryption key is not set")
                    from shared_communication.hint_decoder import decode_hint2
                    gate_info = decode_hint2(raw, key)
                from types import SimpleNamespace
                from ..et_rally_plan_select import select_on_robot
                path, how = select_on_robot(SimpleNamespace(hint1=hint1, hint2_gate_info=gate_info))
                q.put(("ok", (gate_info, path, how)))
            except Exception as error:
                q.put(("error", "local plan selection failed: %r" % (error,)))

        threading.Thread(target=work, name="et-rally-local-plan", daemon=True).start()

    def update(self):
        if self.config.et_rally_strategy_source == "local" and self.context.strategy_requests_enabled:
            self._local_poll()
        # 子のFeatureがcontext.hint1/hint2へ格納した周期に送信キューへ渡す。
        # 走行中でも受信キューを取り込み、No.11へ到着する前にSEQを保存できる。
        if (self.config.et_rally_strategy_source == "received"
                and self.context.strategy_requests_enabled):
            self.exchange.poll(self.context, "right" if runtime.course == -1 else "left",
                               self.config.et_rally_laps)
        return self.decorated.status

    def terminate(self, new_status):
        self.exchange.close()
