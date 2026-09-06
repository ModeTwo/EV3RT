"""Optional original TO exit after Hint2: -25 degrees then 600mm trace."""
from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time
from ..behaviours.section_motion import to_turn, distance_motion
from ..behaviours.line_trace import TraceLine


def build_move_after_hint2(context, config):
    settings = config.integration
    root = Sequence(name='TO original exit after hint2', memory=True)
    root.add_children([
        to_turn('TO return relative-25', context, settings, -25, relative=True),
        distance_motion('TO exit trace',
            TraceLine(name='TO exit trace motor', target=65, power=50,
                      pid_p=0.55, pid_i=0.0000009, pid_d=0.015,
                      trace_side=TraceSide.NORMAL, cutoff_hz=None),
            settings.to_exit_trace_mm, settings.motion_timeout_sec),
    ])
    return root
