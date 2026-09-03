"""Feature 19 subtree factory."""

from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time
from ..placeholder import PendingFeature


def build_drive_to_garage(context, config):
    # No.19 ガレージまでのライントレース走行を担当する。
    root = Sequence(name="drive_to_garage", memory=True)
    root.add_children([PendingFeature(name="drive_to_garage_pending")])
    return root
