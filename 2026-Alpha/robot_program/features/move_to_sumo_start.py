"""Feature 15 subtree factory."""

from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time
from ..placeholder import PendingFeature


def build_move_to_sumo_start(context, config):
    # No.15 ET相撲開始位置への移動を担当する。
    root = Sequence(name="move_to_sumo_start", memory=True)
    root.add_children([PendingFeature(name="move_to_sumo_start_pending")])
    return root
