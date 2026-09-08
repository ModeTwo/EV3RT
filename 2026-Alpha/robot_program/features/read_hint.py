"""TO reads each hint while stationary; raw text is saved for PC processing."""
from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time
from ..behaviours.hint_reader import ReadHintCard
from ..behaviours.motor_control import StopNow


def build_read_hint(context, config, hint_number):
    root = Sequence(name=f'TO read hint{hint_number}', memory=True)
    root.add_children([
        StopNow(name=f'TO hint{hint_number} stationary'),
        ReadHintCard(name=f'TO capture hint{hint_number}', hint_number=hint_number,
                     context=context, timeout_sec=config.integration.qr_timeout_sec),
        StopNow(name=f'TO hint{hint_number} captured brake'),
    ])
    return root
