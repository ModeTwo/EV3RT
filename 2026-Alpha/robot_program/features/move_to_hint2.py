"""TO non-B route from Hint1 to Hint2, following active tantou3.py values."""
from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time
from ..behaviours.section_motion import to_turn, to_drive, distance_motion
from ..behaviours.line_trace import TraceLine


def build_move_to_hint2(context, config):
    settings = config.integration
    root = Sequence(name='TO move to hint2', memory=True)
    root.add_children([
        to_drive('TO after hint1 straight', context, settings, settings.to_after_hint1_mm),
        to_turn('TO second turn local90', context, settings, 90),
        distance_motion('TO hint2 approach trace',
            TraceLine(name='TO hint2 trace motor', target=65, power=60,
                      pid_p=0.055, pid_i=0.005, pid_d=0.5,
                      trace_side=TraceSide.NORMAL, cutoff_hz=None),
            settings.to_hint2_trace_mm, settings.motion_timeout_sec),
        to_turn('TO face hint2 relative25', context, settings, 25, relative=True),
    ])
    return root
