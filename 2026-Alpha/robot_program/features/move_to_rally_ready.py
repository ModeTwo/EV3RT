"""Feature 09 subtree factory."""

from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time
from ..behaviours.bottle import IsDropZoneUnset, IsSelectedDropZone, MarkRallyReady
from ..behaviours.conditions import IsColorDetected
from ..behaviours.line_trace import TraceLine
from ..behaviours.motor_control import StopNow
from ..behaviours.section_motion import distance_motion, to_turn


def _trace_motor(name, settings):
    return TraceLine(
        name=name,
        target=settings.delivery_trace_target_v,
        power=settings.delivery_trace_power,
        pid_p=0.65,
        pid_i=0.000001,
        pid_d=0.045,
        trace_side=TraceSide.NORMAL,
    )


def _trace_until_blue(name, settings):
    search = Parallel(name=name, policy=ParallelPolicy.SuccessOnOne())
    search.add_children(
        [
            _trace_motor(name + " motor", settings),
            IsColorDetected(name=name + " detector", color=Color.BLUE),
        ]
    )
    root = Sequence(name=name + " segment", memory=True)
    root.add_children([search, StopNow(name=name + " brake")])
    return root


def _leave_current_center_and_find_next(name, settings):
    # 配置後は青ライン中央にいるため、残り半分だけ進んでから次の線を探す。
    root = Sequence(name=name, memory=True)
    root.add_children(
        [
            distance_motion(
                name + " leave current marker",
                _trace_motor(name + " leave motor", settings),
                settings.delivery_marker_half_width_mm,
            ),
            _trace_until_blue(name + " find next blue", settings),
        ]
    )
    return root


def _pass_intermediate_and_find_next(name, settings):
    # 次の青ライン手前端から全幅を抜け、さらに上段の青ラインを探す。
    root = Sequence(name=name, memory=True)
    root.add_children(
        [
            distance_motion(
                name + " pass intermediate marker",
                _trace_motor(name + " pass motor", settings),
                settings.delivery_marker_full_width_mm,
            ),
            _trace_until_blue(name + " find next blue", settings),
        ]
    )
    return root


def build_move_to_rally_ready(context, config):
    # No.9 最上段の青ライン中央へ移動し、ETラリーエリア内側へ向ける。
    settings = config.integration
    root = Sequence(name="move_to_rally_ready", memory=True)
    start_zone_route = Selector(name="route from delivered zone to rally start", memory=True)

    # 赤へ配置した場合は、すでに最上段の青ライン中央へ戻っている。
    red_route = Sequence(name="red zone is rally start", memory=True)
    red_route.add_children(
        [IsSelectedDropZone("started from red zone", BottleColor.RED, context)]
    )

    # 青からは一段、黄からは二段上の赤ゾーン前まで進む。
    blue_route = Sequence(name="blue zone to rally start", memory=True)
    blue_route.add_children(
        [
            IsSelectedDropZone("started from blue zone", BottleColor.BLUE, context),
            _leave_current_center_and_find_next("blue to red rally line", settings),
            distance_motion(
                "center red rally line from blue",
                _trace_motor("center red rally line motor from blue", settings),
                settings.delivery_marker_half_width_mm,
            ),
        ]
    )

    yellow_route = Sequence(name="yellow zone to rally start", memory=True)
    yellow_route.add_children(
        [
            IsSelectedDropZone("started from yellow zone", BottleColor.YELLOW, context),
            _leave_current_center_and_find_next("yellow to blue rally line", settings),
            _pass_intermediate_and_find_next("blue to red rally line from yellow", settings),
            distance_motion(
                "center red rally line from yellow",
                _trace_motor("center red rally line motor from yellow", settings),
                settings.delivery_marker_half_width_mm,
            ),
        ]
    )

    # ラリー単体モードではボトル配置を経ないため、黄ゾーン前の手前端から赤まで進む。
    no_bottle_route = Sequence(name="first marker to rally start without bottle", memory=True)
    no_bottle_route.add_children(
        [
            IsDropZoneUnset("drop zone was not selected", context),
            _trace_until_blue("find yellow zone without bottle", settings),
            _pass_intermediate_and_find_next("yellow to blue without bottle", settings),
            _pass_intermediate_and_find_next("blue to red without bottle", settings),
            distance_motion(
                "center red rally line without bottle",
                _trace_motor("center red rally line motor without bottle", settings),
                settings.delivery_marker_half_width_mm,
            ),
        ]
    )
    # 配置した色ごとに、最上段の赤ゾーン前までの移動量を切り替える。
    start_zone_route.add_children(
        [red_route, blue_route, yellow_route, no_bottle_route]
    )

    # ライン進行方向からcourse正規化した+90度へ旋回すると内向きになる。
    turn_toward_rally = to_turn(
        "turn inward at rally start",
        context,
        settings,
        settings.delivery_inward_turn_deg,
        relative=True,
    )
    stop_at_rally_start = StopNow(name="stop at rally start")
    mark_rally_ready = MarkRallyReady("mark rally ready", context)

    root.add_children(
        [
            start_zone_route,
            turn_toward_rally,
            stop_at_rally_start,
            mark_rally_ready,
        ]
    )
    return root
