"""Feature 11 subtree factory."""

from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time
from ..placeholder import PendingFeature


def build_receive_strategy(context, config):
    # No.11 PCからのSEQ受信と未受信時の縮退処理を担当する。
    root = Sequence(name="receive_strategy", memory=True)
    root.add_children([PendingFeature(name="receive_strategy_pending")])
    return root
