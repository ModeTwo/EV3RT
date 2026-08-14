"""Feature 04 subtree factory."""

from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time
from ..placeholder import PendingFeature


def build_move_to_hint1(context, config):
    # No.4 ヒントカード1の読取位置までの移動を担当する。
    root = Sequence(name="move_to_hint1", memory=True)
    root.add_children([PendingFeature(name="move_to_hint1_pending")])
    return root
