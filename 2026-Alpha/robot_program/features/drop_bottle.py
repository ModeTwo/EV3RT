"""Feature 08 subtree factory."""

from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time
from ..behaviours.bottle import MarkBottleDelivered
from ..behaviours.line_trace import TraceLine
from ..behaviours.section_motion import DriveDistance, distance_motion, to_turn


def build_drop_bottle(context, config):
    # No.8 旋回、前進、後退、復帰旋回によるボトル配置を担当する。
    settings = config.integration
    root = Sequence(name="drop_bottle", memory=True)
    root.add_children(
        [
            # 青ラインの手前端から中央まで進み、配置と復帰の基準位置を揃える。
            distance_motion(
                "center selected blue line",
                TraceLine(
                    name="center selected blue line motor",
                    target=settings.delivery_trace_target_v,
                    power=settings.delivery_trace_power,
                    pid_p=0.65,
                    pid_i=0.000001,
                    pid_d=0.045,
                    trace_side=TraceSide.NORMAL,
                ),
                settings.delivery_marker_half_width_mm,
            ),
            # 相対角度はcourseで鏡像化され、Left/Rightともドロップゾーン側へ向く。
            to_turn(
                "turn toward bottle drop zone",
                context,
                settings,
                settings.delivery_drop_turn_deg,
                relative=True,
            ),
            # アームに保持されたボトルをゾーンへ運び、同じ距離を戻って離す。
            DriveDistance(
                "enter bottle drop zone",
                settings.delivery_drop_distance_mm,
                settings.delivery_drive_power,
            ),
            DriveDistance(
                "leave bottle drop zone",
                settings.delivery_drop_distance_mm,
                -settings.delivery_drive_power,
            ),
            # 次の青ラインへ進めるよう、元のライン進行方向へ戻す。
            to_turn(
                "turn back to drop zone line",
                context,
                settings,
                -settings.delivery_drop_turn_deg,
                relative=True,
            ),
            MarkBottleDelivered("mark bottle delivered", context),
        ]
    )
    return root
