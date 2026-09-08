"""Feature 09 subtree factory."""

from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time
from ..placeholder import PendingFeature


def build_move_to_rally_ready(context, config):
    # No.9 ETラリー準備終了位置への移動を担当する。
    root = Sequence(name="move_to_rally_ready", memory=True)
    root.add_children([PendingFeature(name="move_to_rally_ready_pending")])
    return root
