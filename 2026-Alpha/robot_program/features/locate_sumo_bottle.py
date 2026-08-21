"""Feature 16 subtree factory."""

from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time
from ..placeholder import PendingFeature


def build_locate_sumo_bottle(context, config):
    # No.16 ET相撲対象ボトルの位置把握を担当する。
    root = Sequence(name="locate_sumo_bottle", memory=True)
    root.add_children([PendingFeature(name="locate_sumo_bottle_pending")])
    return root
