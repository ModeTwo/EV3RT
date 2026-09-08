"""Template for a new robot-side feature subtree."""

from .bt_imports import (
    Behaviour,
    BottleColor,
    Color,
    Failure,
    HeadingType,
    Parallel,
    ParallelPolicy,
    Running,
    Selector,
    Sequence,
    Status,
    Success,
    TargetInterested,
    TraceSide,
    runtime,
    time,
)
from ..placeholder import PendingFeature


def build_feature_name(context, config):
    # 新しいFeatureはこのファイルを複製し、関数名・ノード名・処理内容を変更する。
    root = Sequence(name="feature_name", memory=True)
    root.add_children([PendingFeature(name="feature_name_pending")])
    return root
