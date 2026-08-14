"""Feature 05 subtree factory."""

from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time
from ..placeholder import PendingFeature


def build_move_to_hint2(context, config):
    # No.5 ヒントカード2の読取位置からドロップゾーン方面への移動を担当する。
    root = Sequence(name="move_to_hint2", memory=True)
    root.add_children([PendingFeature(name="move_to_hint2_pending")])
    return root
