"""Feature 20 subtree factory."""

from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time
from ..placeholder import PendingFeature


def build_stop_in_garage(context, config):
    # No.20 ガレージ内停止と停止保持を担当する。
    root = Sequence(name="stop_in_garage", memory=True)
    root.add_children([PendingFeature(name="stop_in_garage_pending")])
    return root
