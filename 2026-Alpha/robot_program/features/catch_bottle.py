"""AT: gate crossing, bottle recognition and transport to the TO handoff."""
from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time
from py_trees.decorators import Timeout
from ..behaviours.section_motion import DriveDistance, distance_motion
from ..behaviours.line_trace import TraceLine
from ..behaviours.motor_control import StopNow
from ..behaviours.detect_bottle_color import DetectBottleColor
from ..behaviours.handoff import CaptureAtToHandoff


def build_catch_bottle(context, config):
    settings = config.integration
    root = Sequence(name='AT bottle capture and transfer', memory=True)
    # 青検知はREが完了済み。タッチ待ちと青検知を重複させない。
    root.add_children([
        Timeout(name='AT gate forward timeout', duration=settings.motion_timeout_sec,
                child=DriveDistance('AT gate forward', settings.at_gate_forward_mm, 60)),
        Timeout(name='AT recognition reverse timeout', duration=settings.motion_timeout_sec,
                child=DriveDistance('AT recognition reverse', settings.at_recognition_reverse_mm, -60)),
        StopNow(name='AT stop before detection'),
        DetectBottleColor('AT detect bottle', context, settings),
        distance_motion('AT_TO transfer trace',
            TraceLine(name='AT_TO trace motor', target=75, power=60,
                      pid_p=0.65, pid_i=0.000001, pid_d=0.045, trace_side=TraceSide.NORMAL),
            settings.at_to_transfer_trace_mm, settings.motion_timeout_sec),
        CaptureAtToHandoff('AT_TO capture boundary', context),
    ])
    return root
