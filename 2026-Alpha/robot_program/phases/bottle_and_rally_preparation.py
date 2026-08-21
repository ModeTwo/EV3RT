"""Bottle delivery and rally preparation phase composition."""

from py_trees.composites import Sequence

from ..features.catch_bottle import build_catch_bottle
from ..features.drop_bottle import build_drop_bottle
from ..features.move_to_hint1 import build_move_to_hint1
from ..features.move_to_hint2 import build_move_to_hint2
from ..features.move_to_rally_ready import build_move_to_rally_ready
from ..features.read_hint import build_read_hint
from ..features.select_drop_zone import build_select_drop_zone


def build_bottle_and_rally_preparation_phase(context, config):
    # このファイルは処理順だけを管理し、個別の制御ロジックは持たない。
    root = Sequence(name="bottle_and_rally_preparation", memory=True)
    children = []
    if config.enable_bottle_delivery:
        children.append(build_catch_bottle(context, config))
    if config.enable_et_rally:
        children.extend(
            [
                build_move_to_hint1(context, config),
                build_read_hint(context, config, hint_number=1),
                build_move_to_hint2(context, config),
                build_read_hint(context, config, hint_number=2),
            ]
        )
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
