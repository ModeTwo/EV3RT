"""ET sumo phase composition."""

from py_trees.composites import Sequence

from ..features.locate_sumo_bottle import build_locate_sumo_bottle
from ..features.move_to_sumo_exit import build_move_to_sumo_exit
from ..features.move_to_sumo_start import build_move_to_sumo_start
from ..features.push_sumo_bottle import build_push_sumo_bottle


def build_et_sumo_phase(context, config):
    # このファイルはET相撲の処理順だけを管理する。
    root = Sequence(name="et_sumo", memory=True)
    root.add_children(
        [
            build_move_to_sumo_start(context, config),
            build_locate_sumo_bottle(context, config),
            build_push_sumo_bottle(context, config),
            build_move_to_sumo_exit(context, config),
        ]
    )
    return root

