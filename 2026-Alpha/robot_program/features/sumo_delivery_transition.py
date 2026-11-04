"""① ボトルデリバリー終了位置から相撲の直進開始位置へ。"""

from py_trees.behaviours import Failure
from py_trees.composites import Sequence

from ..behaviours.motor_control import StopNow


def build_move_delivery_to_sumo_straight(context, config):
    """担当者はここに走行ツリーを実装し、所定の位置・方位で終了する。"""
    # TODO ①: 開始位置、到達位置、左右コース、ジャイロ基準を実測して決める。
    # 未実装のまま後続へ進ませない。完成時は下記の2ノードを実走行に置換。
    root = Sequence(name="① デリバリー終了位置から相撲直進開始位置へ", memory=True)
    root.add_children([
        StopNow(name="① 未実装のため停止"),
        Failure(name="① 経路未実装"),
    ])
    return root
