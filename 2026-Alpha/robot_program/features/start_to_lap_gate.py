"""RE start section from gyro_line_0826.py; hardware validation pending."""

from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time
from ..behaviours.conditions import IsColorDetected, IsDistanceEarned
from ..behaviours.gyro_drive import RunByGyro
from ..behaviours.line_trace import TraceLine

TRACELINE_TARGET_V = 65


def build_start_to_lap_gate(context, config):
    # RE担当範囲だけを採用。青検知後の100mm前進からATへ渡す。
    # 全工程共通のtiming.CONTROL_INTERVAL_SECを使用する。
    root = Sequence(name="start_to_lap_gate", memory=True)
    edge_01 = Parallel(name='edge_01', policy=ParallelPolicy.SuccessOnOne())
    edge_02 = Parallel(name='edge_02', policy=ParallelPolicy.SuccessOnOne())
    edge_03 = Parallel(name='edge_03', policy=ParallelPolicy.SuccessOnOne())
    edge_04 = Parallel(name='edge_04', policy=ParallelPolicy.SuccessOnOne())
    edge_05 = Parallel(name='edge_05', policy=ParallelPolicy.SuccessOnOne())
    square = Sequence(name='square', memory=True)
    lap2_1 = Parallel(name='lap2_1', policy=ParallelPolicy.SuccessOnOne())
    lap2_2 = Parallel(name='lap2_2', policy=ParallelPolicy.SuccessOnOne())
    lap2_3 = Parallel(name='lap2_3', policy=ParallelPolicy.SuccessOnOne())
    edge_01.add_children([RunByGyro(name='run straight', target=0, power=70, pid_p=1.1, pid_i=0.1, pid_d=0.03, target_type=HeadingType.ABSOLUTE), IsDistanceEarned(name='check distance', delta_dist=500)])
    edge_02.add_children([RunByGyro(name='run straight', target=-45, power=70, pid_p=1.1, pid_i=0.1, pid_d=0.03, target_type=HeadingType.ABSOLUTE), IsDistanceEarned(name='check distance', delta_dist=200)])
    edge_03.add_children([RunByGyro(name='run straight', target=-90, power=70, pid_p=1.1, pid_i=0.1, pid_d=0.03, target_type=HeadingType.ABSOLUTE), IsDistanceEarned(name='check distance', delta_dist=550)])
    edge_04.add_children([RunByGyro(name='run straight', target=-135, power=70, pid_p=1.1, pid_i=0.1, pid_d=0.03, target_type=HeadingType.ABSOLUTE), IsDistanceEarned(name='check distance', delta_dist=230)])
    edge_05.add_children([RunByGyro(name='run straight', target=-180, power=60, pid_p=1.1, pid_i=0.1, pid_d=0.03, target_type=HeadingType.ABSOLUTE), IsDistanceEarned(name='check distance', delta_dist=300)])
    square.add_children([edge_01, edge_02, edge_03, edge_04, edge_05])
    lap2_1.add_children([TraceLine(name='sensor trace normal edge', target=TRACELINE_TARGET_V, power=33, pid_p=0.55, pid_i=9e-07, pid_d=0.015, trace_side=TraceSide.NORMAL, cutoff_hz=None), IsDistanceEarned(name='check distance', delta_dist=100)])
    lap2_2.add_children([TraceLine(name='sensor trace normal edge', target=TRACELINE_TARGET_V, power=60, pid_p=0.55, pid_i=9e-07, pid_d=0.08, trace_side=TraceSide.NORMAL, cutoff_hz=None), IsDistanceEarned(name='check distance', delta_dist=2460)])
    lap2_3.add_children([TraceLine(name='sensor trace normal edge', target=TRACELINE_TARGET_V, power=60, pid_p=0.1, pid_i=9e-07, pid_d=0.08, trace_side=TraceSide.NORMAL, cutoff_hz=None), IsColorDetected(name='check color', color=Color.BLUE)])
    root.add_children([square, lap2_1, lap2_2, lap2_3])
    return root
