"""Finish phase composition."""

from py_trees.composites import Sequence

from ..features.drive_to_garage import build_drive_to_garage
from ..features.stop_in_garage import build_stop_in_garage


def build_finish_phase(context, config):
    # このファイルはFINISH工程の処理順だけを管理する。
    root = Sequence(name="finish", memory=True)
    root.add_children(
        [
            build_drive_to_garage(context, config),
            build_stop_in_garage(context, config),
        ]
    )
    return root

