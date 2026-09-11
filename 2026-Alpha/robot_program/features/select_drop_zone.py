"""Feature 07 subtree factory."""

from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time
from ..behaviours.bottle import IsSelectedDropZone, SelectBottleDropZone
from ..behaviours.conditions import IsColorDetected
from ..behaviours.line_trace import TraceLine
from ..behaviours.motor_control import StopNow
from ..behaviours.section_motion import distance_motion


def _trace_motor(name, settings):
    # ドロップゾーン列は通常側のライン端を追従する。
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
    # 青ラインを見つけた周期でライントレースを止め、次の動作へ引き渡す。
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


def _pass_marker_and_find_next(name, settings):
    # 現在の青ラインを完全に抜けてから検知を再開し、同じ線の再検知を防ぐ。
    root = Sequence(name=name, memory=True)
    root.add_children(
        [
            distance_motion(
                name + " pass marker",
                _trace_motor(name + " pass motor", settings),
                settings.delivery_marker_full_width_mm,
            ),
            _trace_until_blue(name + " find next blue", settings),
        ]
    )
    return root


def build_select_drop_zone(context, config):
    # No.7 ボトル色に対応する青ライン上の停止位置選択を担当する。
    settings = config.integration
    root = Sequence(name="select_drop_zone", memory=True)
    color_route = Selector(name="route to selected drop zone", memory=True)

    # Hint2後の移動完了位置から最初の青ライン（黄ゾーン前）まで追従する。
    yellow_route = Sequence(name="select yellow zone", memory=True)
    yellow_route.add_children(
        [IsSelectedDropZone("is yellow zone", BottleColor.YELLOW, context)]
    )

    # 青ボトルでは黄ゾーン前を一つ通過する。
    blue_route = Sequence(name="select blue zone", memory=True)
    blue_route.add_children(
        [
            IsSelectedDropZone("is blue zone", BottleColor.BLUE, context),
            _pass_marker_and_find_next("yellow to blue", settings),
        ]
    )

    # 赤ボトルおよび認識失敗時は、黄・青ゾーン前を通過して最上段へ進む。
    red_route = Sequence(name="select red zone", memory=True)
    red_route.add_children(
        [
            IsSelectedDropZone("is red zone", BottleColor.RED, context),
            _pass_marker_and_find_next("yellow to blue for red", settings),
            _pass_marker_and_find_next("blue to red", settings),
        ]
    )
    # Selectorは上から順に色を確認し、一致した経路だけを実行する。
    color_route.add_children([yellow_route, blue_route, red_route])

    decide_drop_zone = SelectBottleDropZone(
        "decide bottle drop zone",
        context,
    )
    move_to_first_blue_line = _trace_until_blue(
        "find yellow zone blue line",
        settings,
    )

    root.add_children(
        [
            decide_drop_zone,
            move_to_first_blue_line,
            color_route,
        ]
    )
    return root
