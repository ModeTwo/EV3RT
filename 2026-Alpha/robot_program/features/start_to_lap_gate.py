"""Feature 02 subtree factory."""

from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time

from py_etrobo_util import Color, TraceSide

from ..behaviours.conditions import IsColorDetected, IsDistanceEarned
from ..behaviours.gyro_drive import RunByGyro
from ..behaviours.line_trace import TraceLine
from ..types import HeadingType


def build_start_to_lap_gate(context, config):
    # No.2 スタートからLAPゲート通過までを、このファイル内で実装する。
    root = Sequence(name="start_to_lap_gate", memory=True)

    # 青ラインを検出するまで、通常側のライン端を追従する。
    lap2 = Parallel(name="lap2", policy=ParallelPolicy.SuccessOnOne())
    lap2.add_children(
        [
            TraceLine(
                name="sensor trace normal edge",
                target=75,
                power=70,
                power_min=33,
                pid_p=0.65,
                pid_i=0.000001,
                pid_d=0.045,
                err_lo=6,
                err_hi=16,
                decel_per_s=350,
                gains_slow=(0.65, 0.045),
                gains_fast=(0.55, 0.065),
                recover_v=97,
                recover_after=3,
                recover_turn=35,
                trace_side=TraceSide.NORMAL,
            ),
            IsColorDetected(name="check blue line", color=Color.BLUE),
        ]
    )

    # 青ライン検知後、絶対角度3度を維持して370mm直進する。
    lap3 = Parallel(name="lap3", policy=ParallelPolicy.SuccessOnOne())
    lap3.add_children(
        [
            RunByGyro(
                name="run straight to catch the bottle",
                target=3,
                power=33,
                pid_p=1.1,
                pid_i=0.1,
                pid_d=0.03,
                target_type=HeadingType.ABSOLUTE,
            ),
            IsDistanceEarned(name="check travel distance", delta_dist=370),
        ]
    )

    root.add_children([lap2, lap3])
    return root
