"""Feature 03 subtree factory."""

from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time
from ..placeholder import PendingFeature


def build_catch_bottle(context, config):
    # No.3 LAPゲート通過、ボトル色認識、ボトル取得を担当する。
    root = Sequence(name="catch_bottle", memory=True)
    root.add_children([PendingFeature(name="catch_bottle_pending")])
    return root
