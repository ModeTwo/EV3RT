"""TO non-B route up to the stationary Hint1 reading position."""
from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time
from ..behaviours.section_motion import to_turn, to_drive


def build_move_to_hint1(context, config):
    settings = config.integration
    root = Sequence(name='TO move to hint1', memory=True)
    # 元の黒線判定は無効。tantou3.pyの有効な距離終了条件を維持する。
    root.add_children([
        to_turn('TO first turn local90', context, settings, 90),
        to_drive('TO first approach', context, settings, settings.to_first_black_limit_mm),
        to_turn('TO face hint1 local0', context, settings, 0),
    ])
    return root
