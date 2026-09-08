"""Feature 07 subtree factory."""

from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time
from ..placeholder import PendingFeature


def build_select_drop_zone(context, config):
    # No.7 ボトル色に対応する青ライン上の停止位置選択を担当する。
    root = Sequence(name="select_drop_zone", memory=True)
    root.add_children([PendingFeature(name="select_drop_zone_pending")])
    return root
