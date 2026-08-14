"""Feature 18 subtree factory."""

from py_trees.composites import Sequence
from ..placeholder import PendingFeature


def build_move_to_sumo_exit(context, config):
    # No.18 ET相撲終了位置への移動を担当する。
    root = Sequence(name="move_to_sumo_exit", memory=True)
    root.add_children([PendingFeature(name="move_to_sumo_exit_pending")])
    return root

