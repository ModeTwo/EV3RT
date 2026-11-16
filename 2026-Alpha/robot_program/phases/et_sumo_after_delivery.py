"""ボトルデリバリー後に実行するET相撲の工程構成。"""

from py_trees.composites import Sequence

from ..features.sumo_delivery_transition import build_move_delivery_to_sumo_straight
from ..features.sumo_push_tree import build_sumo_push_tree
from ..features.sumo_rally_transition import build_move_sumo_to_rally_start


def build_et_sumo_after_delivery_phase(context, config):
    """①移動 → ②直線相撲 → ③移動。各担当の走行内容はfeaturesに置く。"""
    # ②は単体試験用の初回0°ではなく、全体スタート基準を使う。
    if config.mission_mode == "sumo-push-route-test":
        raise ValueError("after-delivery phase requires an integrated mission mode")
    if config.sumo_push_calibration is not None:
        raise ValueError("after-delivery phase uses the no-calibration sumo route")
    if not config.sumo_push_execute or config.sumo_relative_push.push_distance_mm is None:
        raise ValueError("set sumo_push_execute=True and an explicit sumo push distance")

    root = Sequence(name="et_sumo_after_delivery", memory=True)
    root.add_children([
        build_sumo_push_tree(context, config),
        build_move_sumo_to_rally_start(context, config),
    ])
    return root
