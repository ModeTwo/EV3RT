"""配置後、方位0度の色別距離を進み、ラリー準備完了を記録する機能。

処理順は、コース絶対方位+0度へ旋回、選択色に応じた前進、
ラリー準備完了の記録。
"""

from ..behaviours.bottle import IsSelectedDropZone, MarkRallyReady
from ..behaviours.corrected_run import DeliveryEtRun
from ..behaviours.encoder_spin import delivery_encoder_turn as to_turn
from ..behaviours.section_motion import distance_motion
from .bt_imports import BottleColor, Selector, Sequence

# 方位+0度を向いた後に進む距離[mm]。センチ指定をmmへ換算している。
_RALLY_CLEARANCE = {
    BottleColor.YELLOW: 800.0,  # 黄: 80 cm
    BottleColor.BLUE: 550.0,   # 青: 55 cm
    BottleColor.RED: 350.0,     # 赤: 5 cm
}


def _drive_to_rally(name, context, settings, distance_mm):
    """コース絶対方位+0度を保って、指定距離だけ前進する動作を作る。"""
    return distance_motion(
        name,
        DeliveryEtRun(
            name + " motor", context,
            target=0.0,
            power=settings.delivery_drive_first_power,
            pid_p=settings.delivery_drive_pid_p,
            pid_i=settings.delivery_drive_pid_i,
            pid_d=settings.delivery_drive_pid_d,
        ),
        distance_mm,
    )


def build_move_to_rally_ready(context, config):
    """色別の短い前進と最終方位合わせを行うBTシーケンスを作る。"""
    settings = config.integration
    root = Sequence(name="move_to_rally_ready", memory=True)

    # まず現在方位に関係なく、コース絶対方位+0度へ向きを合わせる。
    root.add_child(to_turn("face course absolute 0", context, settings, 0.0))

    # 確定済みドロップゾーンを判定し、黄45 cm・青25 cm・赤5 cmを選択する。
    # Selector内では、色判定とその色に対応する距離の前進を一つのSequenceにする。
    color_route = Selector(name="move from drop zone by bottle color", memory=True)
    for color, distance_mm in _RALLY_CLEARANCE.items():
        route = Sequence(name=f"{color.name.lower()} zone rally approach", memory=True)
        route.add_children([
            IsSelectedDropZone(f"is {color.name.lower()} zone", color, context),
            _drive_to_rally(f"advance from {color.name.lower()} zone", context,
                            settings, distance_mm),
        ])
        color_route.add_child(route)
    root.add_child(color_route)

    # 色別前進のあと、rally_readyを設定して後続のラリー工程へ準備完了を伝える。
    root.add_child(MarkRallyReady("mark rally ready", context))
    return root
