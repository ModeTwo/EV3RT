"""Bottle delivery and rally preparation phase composition."""

from py_trees.composites import Sequence

from ..features.catch_bottle import build_catch_bottle
from ..features.to_hint_route import build_tantou_tree
from ..features.drop_bottle import build_drop_bottle
from ..features.move_to_rally_ready import build_move_to_rally_ready
from ..features.select_drop_zone import build_select_drop_zone


def build_bottle_and_rally_preparation_phase(context, config):
    # このファイルは処理順だけを管理し、個別の制御ロジックは持たない。
    root = Sequence(name="bottle_and_rally_preparation", memory=True)
    children = []
    if config.enable_bottle_delivery:
        children.append(build_catch_bottle(context, config))
    if config.enable_et_rally or config.enable_bottle_delivery:
        # TOの元の全ツリーを実行し、出口移動後にボトル配置へ進む。
        children.append(build_tantou_tree(context, config))
    if config.enable_bottle_delivery:
        children.extend(
            [
                build_select_drop_zone(context, config),
                build_drop_bottle(context, config),
            ]
        )
    children.append(build_move_to_rally_ready(context, config))
    root.add_children(children)
    return root


def build_bottle_delivery_final_phase(context, config):
    # Hint2後移動の終了位置から、色別配置とラリー開始位置への復帰だけを実行する。
    root = Sequence(name="bottle_delivery_final", memory=True)
    root.add_children(
        [
            build_select_drop_zone(context, config),
            build_drop_bottle(context, config),
            build_move_to_rally_ready(context, config),
        ]
    )
    return root
