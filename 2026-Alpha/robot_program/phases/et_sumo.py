"""ET sumo phase composition."""

from py_trees.composites import Sequence

from ..features.capture_sumo_bottle_camera import build_capture_sumo_bottle_camera
from ..features.move_to_sumo_exit import build_move_to_sumo_exit
from ..features.move_to_sumo_start import build_move_to_sumo_start


def build_et_sumo_phase(context, config):
    # このファイルはET相撲の処理順だけを管理する。
    root = Sequence(name="et_sumo", memory=True)
    root.add_children(
        [
            build_move_to_sumo_start(context, config),
            build_capture_sumo_bottle_camera(context, config),
            build_move_to_sumo_exit(context, config),
        ]
    )
    return root
