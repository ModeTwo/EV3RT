"""Feature 08 subtree factory."""

from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time
from ..behaviours.bottle import MarkBottleDelivered
from ..behaviours.line_trace import TraceLine
from ..behaviours.section_motion import DriveDistance, distance_motion, to_turn


def build_drop_bottle(context, config):
    # No.8 旋回、前進、後退、復帰旋回によるボトル配置を担当する。
    settings = config.integration
    root = Sequence(name="drop_bottle", memory=True)

    # 1. 検知した青ラインの手前端から中央まで進む。
    move_to_blue_line_center = distance_motion(
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
    )

    # 2. ライン進行方向からドロップゾーン側へ90度旋回する。
    turn_toward_drop_zone = to_turn(
        "turn toward bottle drop zone",
        context,
        settings,
        settings.delivery_drop_turn_deg,
        relative=True,
    )

    # 3. ボトルを置くため、ドロップゾーンへ規定距離だけ前進する。
    drive_into_drop_zone = DriveDistance(
        "enter bottle drop zone",
        settings.delivery_drop_distance_mm,
        settings.delivery_drive_power,
    )

    # 4. 前進した距離と同じ距離を後退し、青ライン中央へ戻る。
    reverse_to_blue_line = DriveDistance(
        "leave bottle drop zone",
        settings.delivery_drop_distance_mm,
        -settings.delivery_drive_power,
    )

    # 5. 次の青ラインへ進めるよう、元のライン進行方向へ向きを戻す。
    turn_back_to_line = to_turn(
        "turn back to drop zone line",
        context,
        settings,
        -settings.delivery_drop_turn_deg,
        relative=True,
    )

    # 6. ここまでの制御が完了したことを後続工程へ記録する。
    mark_delivery_complete = MarkBottleDelivered(
        "mark bottle delivered",
        context,
    )

    root.add_children(
        [
            move_to_blue_line_center,
            turn_toward_drop_zone,
            drive_into_drop_zone,
            reverse_to_blue_line,
            turn_back_to_line,
            mark_delivery_complete,
        ]
    )
    return root
