"""ET rally phase composition."""

from py_trees.composites import Sequence

from ..features.correct_rally_heading import build_correct_rally_heading
from ..features.execute_strategy import build_execute_strategy
from ..features.receive_strategy import build_receive_strategy


def build_et_rally_phase(context, config):
    # 周回数の反映と周回ループは、各走行機能ではなくこの工程合成層で行う。
    root = Sequence(name="et_rally", memory=True)
    children = [build_receive_strategy(context, config)]
    for lap_number in range(1, config.et_rally_laps + 1):
        children.extend(
            [
                build_correct_rally_heading(context, config, lap_number),
                build_execute_strategy(context, config, lap_number),
            ]
        )
    root.add_children(children)
    return root
