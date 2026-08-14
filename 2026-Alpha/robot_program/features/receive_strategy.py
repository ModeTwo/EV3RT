"""Feature 11 subtree factory."""

from py_trees.composites import Sequence
from ..placeholder import PendingFeature


def build_receive_strategy(context, config):
    # No.11 PCからのSEQ受信と未受信時の縮退処理を担当する。
    root = Sequence(name="receive_strategy", memory=True)
    root.add_children([PendingFeature(name="receive_strategy_pending")])
    return root

