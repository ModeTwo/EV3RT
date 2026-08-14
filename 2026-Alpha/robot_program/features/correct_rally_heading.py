"""Feature 13 subtree factory."""

from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time
from ..placeholder import PendingFeature


def build_correct_rally_heading(context, config, lap_number):
    # No.13 各周回開始前の赤ゲート認識と走行角度補正を担当する。
    root = Sequence(name=f"correct_rally_heading_lap{lap_number}", memory=True)
    root.add_children([PendingFeature(name=f"correct_rally_heading_lap{lap_number}_pending")])
    return root
