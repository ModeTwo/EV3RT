"""Feature 12 subtree factory."""

from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time
from ..placeholder import PendingFeature


def build_execute_strategy(context, config, lap_number):
    # No.12 受信した走行指令に従う走行を担当する。
    root = Sequence(name=f"execute_strategy_lap{lap_number}", memory=True)
    root.add_children([PendingFeature(name=f"execute_strategy_lap{lap_number}_pending")])
    return root
