"""Feature 08 subtree factory."""

from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time
from ..behaviours.bottle import MarkBottleDelivered
from ..behaviours.line_trace import TraceLine
from ..behaviours.corrected_run import DeliveryEtRun
from ..behaviours.section_motion import distance_motion
from ..behaviours.encoder_spin import delivery_encoder_turn as to_turn


def build_drop_bottle(context, config):
    # No.8 旋回、前進、後退、復帰旋回によるボトル配置を担当する。
    # 前進・後退は、ETラリーで実績のある補正込みのDeliveryEtRun(RunByGyro相当、
    # power<0で後退)で方位を保持しながら走る。DriveDistance(無補正)は使わない。
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

    # 2. ライン進行方向からドロップゾーン側へ旋回する(1本目)。
    turn_toward_drop_zone_first = to_turn(
        "turn toward bottle drop zone",
        context,
        settings,
        settings.delivery_drop_turn_deg_first,
    )

    # 3. ボトルを置くため、ドロップゾーンへ規定距離だけ前進する(1本目、方位保持)。
    drive_into_drop_zone_first = distance_motion(
        "enter bottle drop zone (leg 1)",
        DeliveryEtRun(
            "enter bottle drop zone motor (leg 1)", context,
            target=settings.delivery_drop_turn_deg_first,
            power=settings.delivery_drive_first_power,
            pid_p=settings.delivery_drive_pid_p,
            pid_i=settings.delivery_drive_pid_i,
            pid_d=settings.delivery_drive_pid_d,
        ),
        settings.delivery_drop_distance_first_mm,
    )

    # 4. さらにドロップゾーン側へ旋回する(2本目)。
    turn_toward_drop_zone_second = to_turn(
        "turn toward bottle drop zone",
        context,
        settings,
        settings.delivery_drop_turn_deg_second,
    )

    # 5. ボトルを置くため、ドロップゾーンへ規定距離だけ前進する(2本目、方位保持)。
    drive_into_drop_zone_second = distance_motion(
        "enter bottle drop zone (leg 2)",
        DeliveryEtRun(
            "enter bottle drop zone motor (leg 2)", context,
            target=settings.delivery_drop_turn_deg_second,
            power=settings.delivery_drive_second_power,
            pid_p=settings.delivery_drive_pid_p,
            pid_i=settings.delivery_drive_pid_i,
            pid_d=settings.delivery_drive_pid_d,
        ),
        settings.delivery_drop_distance_second_mm,
    )

    # 6. 直前(2本目)の向きを保ったまま、2本目を打ち消す距離だけ後退する。
    #    (旋回前に後退する: 進んだ経路をそのまま逆再生する)
    reverse_leg_second = distance_motion(
        "leave bottle drop zone (undo leg 2)",
        DeliveryEtRun(
            "leave bottle drop zone motor (undo leg 2)", context,
            target=settings.delivery_drop_turn_deg_second,
            power=-settings.delivery_drive_second_power,
            pid_p=settings.delivery_drive_pid_p,
            pid_i=settings.delivery_drive_pid_i,
            pid_d=settings.delivery_drive_pid_d,
        ),
        settings.delivery_drop_distance_second_mm,
    )

    # 7. 1本目の向きへ旋回して戻す。
    turn_back_to_line_first = to_turn(
        "turn back to drop zone line",
        context,
        settings,
        settings.delivery_drop_turn_deg_first,  # 絶対方位: 1本目の進行方向へ戻る
    )

    # 8. 1本目の向きを保ったまま、1本目を打ち消す距離だけ後退し、青ライン中央へ戻る。
    reverse_leg_first = distance_motion(
        "leave bottle drop zone (undo leg 1)",
        DeliveryEtRun(
            "leave bottle drop zone motor (undo leg 1)", context,
            target=settings.delivery_drop_turn_deg_first,
            power=-settings.delivery_drive_first_power,
            pid_p=settings.delivery_drive_pid_p,
            pid_i=settings.delivery_drive_pid_i,
            pid_d=settings.delivery_drive_pid_d,
        ),
        settings.delivery_drop_distance_first_mm,
    )

    # 9. 次の青ラインへ進めるよう、元のライン進行方向(0度)へ向きを戻す。
    turn_back_to_line_second = to_turn(
        "turn back to drop zone line",
        context,
        settings,
        0.0,  # 絶対方位: ライン進行方向へ戻る
    )

    # 10. ここまでの制御が完了したことを後続工程へ記録する。
    mark_delivery_complete = MarkBottleDelivered(
        "mark bottle delivered",
        context,
    )

    root.add_children(
        [
            move_to_blue_line_center,
            turn_toward_drop_zone_first,
            drive_into_drop_zone_first,
            turn_toward_drop_zone_second,
            drive_into_drop_zone_second,
            reverse_leg_second,
            turn_back_to_line_first,
            reverse_leg_first,
            turn_back_to_line_second,
            mark_delivery_complete,
        ]
    )
    return root
