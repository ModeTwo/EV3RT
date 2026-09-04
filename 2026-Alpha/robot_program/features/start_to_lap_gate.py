"""Feature 02 subtree factory."""

from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time

from py_etrobo_util import Color, TraceSide

from ..behaviours.conditions import IsColorDetected, IsDistanceEarned
from ..behaviours.gyro_drive import RunByGyro
from ..behaviours.line_trace import TraceLine
from ..types import HeadingType

TRACELINE_TARGET_V = 65 


def build_start_to_lap_gate(context, config):
    # No.2 スタートからLAPゲート通過までを、このファイル内で実装する。
    root = Sequence(name="start_to_lap_gate", memory=True)
    start = Parallel(name="start", policy=ParallelPolicy.SuccessOnOne()) # 並行処理ノード（どれか1つがSUCCESSになればクリア）
    edge_01 = Parallel(name="lap2", policy=ParallelPolicy.SuccessOnOne())#parallelでmemory=trueは持っていないが、sequenceに組み込むことでparallelの結果も覚えられる。
    edge_02 = Parallel(name="lap2", policy=ParallelPolicy.SuccessOnOne())
    edge_03 = Parallel(name="lap2", policy=ParallelPolicy.SuccessOnOne())
    edge_04 = Parallel(name="lap2", policy=ParallelPolicy.SuccessOnOne())
    edge_05 = Parallel(name="lap2", policy=ParallelPolicy.SuccessOnOne())
    # ジャイロ走行全体
    square = Sequence(name="square", memory=True)
    
    lap2_1 = Parallel(name="lap2", policy=ParallelPolicy.SuccessOnOne())#カーブ箇所のライントレース
    lap2_2 = Parallel(name="lap2", policy=ParallelPolicy.SuccessOnOne())#カーブ箇所のライントレース

    # lap2_1：角度0°で直進 → 距離500で成功
    edge_01.add_children(
        [
            RunByGyro(
                name="run straight",
                target=0,
                power=80,
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
                power=80,
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
                power=80,
                pid_p=1.1,
                pid_i=0.1,
                pid_d=0.03,
                target_type=HeadingType.ABSOLUTE
            ),
            IsDistanceEarned(name="check distance", delta_dist=550),
        ]
    )

    
    # lap2_3：角度135°で直進 → 距離190で成功(1cm短くした)
    edge_04.add_children(
        [
            RunByGyro(
                name="run straight",
                target=-135,
                power=80,
                pid_p=1.1,
                pid_i=0.1,
                pid_d=0.03,
                target_type=HeadingType.ABSOLUTE
            ),
            IsDistanceEarned(name="check distance", delta_dist=190),
        ]
    )

     
    # lap2_3：角度180°で直進 → 距離200で成功
    edge_05.add_children(
         [
             RunByGyro(
                 name="run straight",
                 target=-180,
                 power=60, #ここだけ60
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

    #lap2_4（床の線を見ながら通常エッジをトレース。50mmまではpower=33でゆっくり進む）
    lap2_1.add_children(
        [
            TraceLine(name="fukki", target=TRACELINE_TARGET_V, power=33,
                pid_p=0.55, pid_i=0.0000009, pid_d=0.015, trace_side=TraceSide.NORMAL),
            IsDistanceEarned(name="check distance", delta_dist=80)
        ]
    )


     #lap2_4（床の線を見ながら通常エッジをトレースし、地面に青い線が見えるまで突っ走る）
    lap2_2.add_children(
        [
            TraceLine(name="sensor trace normal edge", target=TRACELINE_TARGET_V, power=50,
                pid_p=0.55, pid_i=0.0000009, pid_d=0.015, trace_side=TraceSide.NORMAL),
            IsColorDetected(name="check color", color=Color.BLUE),
        ]
    )

    """
    power:前進のパワー
    pid_p:ズレに対する反応の強さ（大きい P → せっかちで反応が速い。小さい P → 落ち着いていて慎重）
    pid_i:誤差が長時間続いたときに、少しずつ補正を積み上げていく仕組み（大きい I → 我慢強くズレを直そうとする（でも過剰補正しやすい）。
          小さい I → ほぼ無視する（でも安定する）
    pid_d:誤差の変化の速さ（変化量）に反応して、ロボットの暴れを抑える役割(大きい D → 落ち着いている（でも反応が鈍くなる）
          小さい D → 反応が速い（でも暴れやすい）)
    target:角度
    """

    root.add_children(
        [
            square,
            lap2_1,
            lap2_2
        ]
    )
    return root
