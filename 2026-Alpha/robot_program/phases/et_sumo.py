"""ET sumo phase composition."""

from py_trees.composites import Sequence
from dataclasses import replace
from ..features.sumo_push_tree import build_sumo_push_tree

from ..features.capture_sumo_bottle_camera import build_capture_sumo_bottle_camera
from ..features.move_to_sumo_exit import build_move_to_sumo_exit
from ..features.move_to_sumo_start import build_move_to_sumo_start


def build_et_sumo_phase(context, config):
    return build_sumo_push_tree(context, config)


def build_sumo_after_delivery(context, config):
    """カーブから直線相撲へ接続。前進距離は相撲制御器の仕様に従う。"""
    push = replace(config.sumo_relative_push, push_distance_mm=1000.0)
    config = replace(config, sumo_push_calibration=None, sumo_push_execute=True,
                     sumo_relative_push=push)
    config = replace(config, sumo_align=replace(
        config.sumo_align, distance_mm=1100.0
    ))
    tree = build_sumo_push_tree(context, config)
    tree.engine.continuous_entry = True
    tree.engine.delivery_green_on_right = (
        config.mission_mode == "bottle-sumo"
    )
    return tree
