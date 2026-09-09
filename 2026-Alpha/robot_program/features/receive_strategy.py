"""Feature 11 subtree factory."""

from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time
from ..behaviours.motor_control import StopNow


class WaitForStrategy(Behaviour):
    def __init__(self, context):
        super().__init__(name="wait for PC strategy")
        self.context = context

    def update(self):
        # socket.recvは呼ばず、通信スレッドが反映した状態だけを参照する。
        if self.context.strategy_status == "ready":
            return Status.SUCCESS
        if self.context.strategy_status == "failed":
            self.logger.error("Strategy unavailable: %s" % self.context.strategy_error)
            return Status.FAILURE
        if not self.context.hint1 or not self.context.hint2:
            self.logger.error("Both hints are required before starting ET rally")
            return Status.FAILURE
        return Status.RUNNING


def build_receive_strategy(context, config):
    # No.11 PCからのSEQ受信と未受信時の縮退処理を担当する。
    root = Sequence(name="receive_strategy", memory=True)
    root.add_children([StopNow(name="stop before strategy wait"), WaitForStrategy(context)])
    return root
