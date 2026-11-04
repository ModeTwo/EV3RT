"""③ 相撲終了位置からETラリーの開始位置へ。"""

from py_trees.behaviours import Failure
from py_trees.composites import Sequence

from ..behaviours.motor_control import StopNow


def build_move_sumo_to_rally_start(context, config):
    """担当者はここに走行ツリーを実装し、ラリーの開始位置・方位で終了する。"""
    # TODO ③: ②終了時の位置・方位のばらつきと、到達位置を実測して決める。
    # 未実装のまま後続へ進ませない。完成時は下記の2ノードを実走行に置換。
    root = Sequence(name="③ 相撲終了位置からETラリー開始位置へ", memory=True)
    root.add_children([
        StopNow(name="③ 未実装のため停止"),
        Failure(name="③ 経路未実装"),
    ])
    return root
