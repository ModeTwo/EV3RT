"""ボトルの色から、使用するドロップゾーンを確定する機能。

この機能は走行・旋回を行わず、後続のdrop_bottleと
move_to_rally_readyが参照するcontext.selected_drop_zoneを設定する。
"""

from ..behaviours.bottle import SelectBottleDropZone


def build_select_drop_zone(context, config):
    """取得済みボトル色をドロップゾーンとしてcontextに保存するBT動作を作る。

    SelectBottleDropZoneがcontext.bottle_colorを読み、赤・青・黄のいずれかを
    context.selected_drop_zoneへ設定する。色が未設定または不正な場合の扱いは、
    SelectBottleDropZone側の既定動作（赤ゾーン）に任せる。

    config引数は他のfeature factoryと呼び出し形式をそろえるために受け取る。
    この選択処理自体には走行設定を使わない。
    """
    return SelectBottleDropZone(
        name="select drop zone from bottle color",
        context=context,
    )
