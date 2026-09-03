"""Feature 17 subtree factory."""

from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time
from ..placeholder import PendingFeature


def build_push_sumo_bottle(context, config):
    # No.17 ボトル押し出しと完了判定を担当する。
    root = Sequence(name="push_sumo_bottle", memory=True)
    root.add_children([PendingFeature(name="push_sumo_bottle_pending")])
    return root
