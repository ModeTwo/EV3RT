"""Feature 08 subtree factory."""

from py_trees.composites import Sequence
from ..placeholder import PendingFeature


def build_drop_bottle(context, config):
    # No.8 旋回、前進、後退、復帰旋回によるボトル配置を担当する。
    root = Sequence(name="drop_bottle", memory=True)
    root.add_children([PendingFeature(name="drop_bottle_pending")])
    return root

