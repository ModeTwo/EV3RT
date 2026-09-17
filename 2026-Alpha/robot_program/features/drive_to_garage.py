"""Feature 19: trace the garage-side line until the blue marker."""
from .bt_imports import Color, Parallel, ParallelPolicy, TraceSide
from py_trees.decorators import Timeout
from ..behaviours.line_trace import TraceLine
from ..behaviours.conditions import IsColorDetected


def build_drive_to_garage(context, config):
    root = Parallel(name="drive_to_garage", policy=ParallelPolicy.SuccessOnOne())
    root.add_children([
        TraceLine(name="garage line to blue", target=config.garage_line_target_v,
                  power=50, pid_p=0.65, pid_i=0.000001, pid_d=0.045,
                  trace_side=TraceSide.NORMAL),
        IsColorDetected(name="garage blue marker", color=Color.BLUE),
    ])
    return Timeout(name="garage blue search timeout", child=root,
                   duration=config.garage_blue_timeout_sec)
