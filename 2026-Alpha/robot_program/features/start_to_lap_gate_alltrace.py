"""スーパージャイロでできなかたときの予備用。谷口さんベースコードで全部ライントレース。通常はstart_to_lap_gate.pyを編集する。"""

from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time
from ..behaviours.conditions import IsColorDetected, IsDistanceEarned
from ..behaviours.gyro_drive import RunByGyro
from ..behaviours.line_trace import TraceLine

TRACELINE_TARGET_V = 65


def build_alltrace_start_to_lap_gate(context, config):
    # RE担当範囲だけを採用。青検知後の100mm前進からATへ渡す。
    # 全工程共通のtiming.CONTROL_INTERVAL_SECを使用する。
    root = Sequence(name="start_to_lap_gate", memory=True)

    trace_to_blue = Parallel(name='trace_to_blue', policy=ParallelPolicy.SuccessOnOne())
    gyro_after_blue = Parallel(name='gyro_after_blue', policy=ParallelPolicy.SuccessOnOne())

    trace_to_blue.add_children([TraceLine(name='sensor trace normal edge', 
                                          target=TRACELINE_TARGET_V, 
                                          power=60, pid_p=0.65, pid_i=0.000001, pid_d=0.045, 
                                          trace_side=TraceSide.NORMAL, cutoff_hz=None), 
                                IsColorDetected(name='check color', color=Color.BLUE)])

    gyro_after_blue.add_children([RunByGyro(name='run straight', 
                                            target=0, power=70, pid_p=1.1, pid_i=0.1, pid_d=0.03, 
                                            target_type=HeadingType.ABSOLUTE), 
                                  IsDistanceEarned(name='check distance', delta_dist=120)])

    root.add_children([trace_to_blue, gyro_after_blue])
    return root

