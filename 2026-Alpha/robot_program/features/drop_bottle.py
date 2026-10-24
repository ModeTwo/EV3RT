"""ボトル色に対応するゾーンへ進入し、ボトルを置く機能。

色ごとの進入方位・距離・後退距離でゾーンへ進み、配置完了を
共有contextへ記録する。
"""

from .bt_imports import BottleColor, Selector, Sequence
from ..behaviours.bottle import IsSelectedDropZone, MarkBottleDelivered
from ..behaviours.corrected_run import DeliveryEtRun
from ..behaviours.encoder_spin import delivery_encoder_turn as to_turn
from ..behaviours.section_motion import distance_motion

# 色ごとの (ドロップゾーン側への進入方位, ゾーンまでの距離[mm])。
# 方位はドロップコース基準。既存の方位座標系ではゾーン側が負方向なので、
# 30/20/10度をそれぞれ -30/-20/-10度として指定する。
_DROP_ROUTE = {
    BottleColor.YELLOW: (-30.0, 400.0),  # 黄: -30度で40 cm
    BottleColor.BLUE: (-20.0, 700.0),    # 青: -20度で70 cm
    BottleColor.RED: (-15.0, 900.0),     # 赤: -15度で90 cm
}

# 色ごとの後退距離[mm]。黄は従来どおり10 cm。
_DROP_BACK_DISTANCE_MM = {
    BottleColor.YELLOW: 100.0,
    BottleColor.BLUE: 200.0,
    BottleColor.RED: 200.0,
}


def _drive(name, context, settings, heading, distance_mm, power):
    """指定方位をジャイロ補正で保ちながら、指定距離だけ直進する動作を作る。

    distance_motionが走行距離を監視し、DeliveryEtRunが走行中の方位ずれを
    補正する。powerを負にすると、headingを保ったまま後退する。
    """
    return distance_motion(
        name,
        DeliveryEtRun(
            name + " motor", context,
            target=heading,
            power=power,
            pid_p=settings.delivery_drive_pid_p,
            pid_i=settings.delivery_drive_pid_i,
            pid_d=settings.delivery_drive_pid_d,
        ),
        distance_mm,
    )


def build_drop_bottle(context, config):
    """選択されたボトル色に応じた配置シーケンスを作る。

    Selectorは黄・青・赤の分岐を順に確認し、IsSelectedDropZoneが成功した
    色のシーケンスだけを実行する。分岐内の順序は、色の確認、ゾーン側へ旋回、
    色別距離の前進、色別距離の後退、配置完了の記録。
    """
    settings = config.integration
    root = Sequence(name="drop_bottle", memory=True)
    color_route = Selector(name="approach color drop zone", memory=True)

    for color, (heading, distance_mm) in _DROP_ROUTE.items():
        back_distance_mm = _DROP_BACK_DISTANCE_MM[color]
        # この色が確定配置先の場合だけ、その色に対応する動作を実行する。
        route = Sequence(name=f"{color.name.lower()} bottle drop", memory=True)
        route.add_children([
            IsSelectedDropZone(f"is {color.name.lower()} zone", color, context),
            # コース基準の色別方位でドロップゾーン側を向く。
            to_turn(f"turn toward {color.name.lower()} drop zone", context, settings, heading),
            # 色ごとの所定距離を前進し、ボトル色のゾーンまで進む。
            _drive(f"drive to {color.name.lower()} drop zone", context, settings,
                   heading, distance_mm, settings.delivery_drive_first_power),
                 # 同じ方位を維持して色別距離だけ後退し、ボトルを放す位置を作る。
            _drive(f"back away from {color.name.lower()} bottle", context, settings,
                     heading, back_distance_mm, -settings.delivery_drive_first_power),
            # この分岐の移動がすべて成功したあと、配置済み状態を後続へ通知する。
            MarkBottleDelivered(f"mark {color.name.lower()} bottle delivered", context),
        ])
        color_route.add_child(route)

    root.add_child(color_route)
    return root
