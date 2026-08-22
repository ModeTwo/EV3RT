"""Feature 02 subtree factory."""

from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time

from py_etrobo_util import Color, TraceSide

from ..behaviours.conditions import IsColorDetected, IsDistanceEarned
from ..behaviours.gyro_drive import RunByGyro
from ..behaviours.line_trace import TraceLine
from ..types import HeadingType

TRACELINE_TARGET_V = 65     # ライントレース時の目標とする輝度値（センサーの真下の明るさ目標）

def build_start_to_lap_gate(context, config):
    # No.2 スタートからLAPゲート通過までを、このファイル内で実装する。
    root = Sequence(name="start_to_lap_gate", memory=True)
    
    # 青ラインを検出するまで、通常側のライン端を追従する。
    lap2 = Parallel(name="lap2", policy=ParallelPolicy.SuccessOnOne())
    lap3 = Parallel(name="lap3", policy=ParallelPolicy.SuccessOnOne())
    edge_01 = Parallel(name="lap2", policy=ParallelPolicy.SuccessOnOne())#parallelでmemory=trueは持っていないが、sequenceに組み込むことでparallelの結果も覚えられる。
    edge_02 = Parallel(name="lap2", policy=ParallelPolicy.SuccessOnOne())
    edge_03 = Parallel(name="lap2", policy=ParallelPolicy.SuccessOnOne())
    edge_04 = Parallel(name="lap2", policy=ParallelPolicy.SuccessOnOne())
    edge_05 = Parallel(name="lap2", policy=ParallelPolicy.SuccessOnOne())
    # ジャイロ走行全体
    square = Sequence(name="square", memory=True)
    lap2_4 = Parallel(name="lap2", policy=ParallelPolicy.SuccessOnOne())#カーブ箇所のライントレース\


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
     # lap2_1：角度0°で直進 → 距離500で成功
    edge_01.add_children(
        [
            RunByGyro(
                name="run straight",
                target=0,
                power=70,
                pid_p=1.1,
                pid_i=0.1,
                pid_d=0.03,
                target_type=HeadingType.ABSOLUTE
            ),
            IsDistanceEarned(name="check distance", delta_dist=500),
        ]
    )

     # lap2_2：角度45°で直進 → 距離200で成功
    edge_02.add_children(
        [
            RunByGyro(
                name="run straight",
                target=-45,
                power=70,
                pid_p=1.1,
                pid_i=0.1,
                pid_d=0.03,
                target_type=HeadingType.ABSOLUTE
            ),
            IsDistanceEarned(name="check distance", delta_dist=200),
        ]
    )

     # lap2_3：角度90°で直進 → 距離550で成功
    edge_03.add_children(
        [
            RunByGyro(
                name="run straight",
                target=-90,
                power=70,
                pid_p=1.1,
                pid_i=0.1,
                pid_d=0.03,
                target_type=HeadingType.ABSOLUTE
            ),
            IsDistanceEarned(name="check distance", delta_dist=550),
        ]
    )

    
    # lap2_3：角度135°で直進 → 距離200で成功
    edge_04.add_children(
        [
            RunByGyro(
                name="run straight",
                target=-135,
                power=70,
                pid_p=1.1,
                pid_i=0.1,
                pid_d=0.03,
                target_type=HeadingType.ABSOLUTE
            ),
            IsDistanceEarned(name="check distance", delta_dist=200),
        ]
    )

     
    # lap2_3：角度180°で直進 → 距離200で成功
    edge_05.add_children(
         [
             RunByGyro(
                 name="run straight",
                 target=-180,
                 power=70, #ここだけ60
                 pid_p=1.1,
                 pid_i=0.1,
                 pid_d=0.03,
                 target_type=HeadingType.ABSOLUTE
                ),
                IsDistanceEarned(name="check distance", delta_dist=200),
        ]
    )
    # ジャイロ走行区間の構成（直進→回転→直進→回転…）
    square.add_children(
        [
            edge_01,
            edge_02,
            edge_03,
            edge_04,
            edge_05
           
        ]
    )

    #lap2_4（床の線を見ながら通常エッジをトレースし、地面に青い線が見えるまで突っ走る）
    lap2_4.add_children(
        [
            TraceLine(
                name="sensor trace normal edge",
                target=TRACELINE_TARGET_V,
                power=50,
                pid_p=0.55,
                pid_i=0.0000009,
                pid_d=0.015,
                trace_side=TraceSide.NORMAL
                ),
            IsColorDetected(name="check color", color=Color.BLUE),
        ]
    )


    root.add_children(
        [
            square,           #ジャイロ走行
            lap2_4,           #カーブ箇所のライントレース
            lap3
        ]
    )
    return root
