"""ET rally phase composition."""

from py_trees.composites import Sequence

from ..behaviours.device_control import ArmDirection, ArmUpDownFull
from ..features.execute_strategy import build_execute_strategy
from ..features.receive_strategy import build_receive_strategy


def build_et_rally_phase(context, config):
    # PCまたは固定planが全周回分を一つのSEQとして返すため、SEQは一度だけ実行する。
    # ボトル配送までアームを下げたまま運んでいるため、ETラリー走行の前に上げ直す。
    root = Sequence(name="et_rally", memory=True)
    children = [ArmUpDownFull(name="et_rally arm up", direction=ArmDirection.UP)]
    if config.et_rally_strategy_source == "received":
        children.append(build_receive_strategy(context, config))
    elif config.et_rally_strategy_source != "file":
        raise ValueError(
            "Unknown ET rally strategy source: " + str(config.et_rally_strategy_source)
        )
    children.append(build_execute_strategy(context, config))
    root.add_children(children)
    return root
