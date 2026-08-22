"""Feature 20 subtree factory."""

from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time
from ..placeholder import PendingFeature


def build_stop_in_garage(context, config):
    # No.20 ガレージ内停止と停止保持を担当する。
    root = Sequence(name="stop_in_garage", memory=True)

    #ゴール01（ライントレース）
    goal_01 = Parallel(name="goal 01", policy=ParallelPolicy.SuccessOnOne())

    #ゴール02（ジャイロ走行）
    goal_02 = Parallel(name="goal 02", policy=ParallelPolicy.SuccessOnOne())

    
    # goal_01:ライントレースで進む→青いマーカーまで進む
    goal_01.add_children(
       [
            TraceLine(name="sensor trace normal edge", target=TRACELINE_TARGET_V, power=50,
                pid_p=0.55, pid_i=0.0000009, pid_d=0.015, trace_side=TraceSide.NORMAL),
            IsColorDetected(name="check color", color=Color.BLUE),
        ]
    )

    # goal_02：ゴールまで角度0°で直進 → 距離650mmで成功
    goal_02.add_children(
        [
            RunByGyro(
                name="run straight",
                target=0,
                power=60,
                pid_p=1.1,
                pid_i=0.1,
                pid_d=0.03,
                target_type=HeadingType.ABSOLUTE
            ),
            IsDistanceEarned(name="check distance", delta_dist=650),
        ]
    )

    root.add_children([goal_01,goal_02])
    return root
