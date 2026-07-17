"""走行体と無線通信デバイスのJSONプロトコルとヒント形式を定義する。"""

from __future__ import annotations

import re
from dataclasses import dataclass


POSITION_RE = re.compile(r"^[1-5]{2}$")


@dataclass(frozen=True)
class GridPosition:
    """競技規約のゲートポジションGx-yを表す。

    属性:
        x: 左から右へ数えた列番号。競技コース上では1～5を使用する。
        y: 上から下へ数えた行番号。競技コース上では1～5を使用する。
    """

    x: int
    y: int

    @classmethod
    def parse(cls, text: str) -> "GridPosition":
        """2桁のゲート位置文字列を列番号と行番号へ分解する。

        引数:
            text: ``"11"`` や ``"35"`` のように、xとyを1桁ずつ並べた文字列。
                前後の空白は取り除いてから検証する。

        戻り値:
            解析した列番号と行番号を保持するゲート位置。

        例外:
            ValueError: 文字列が2桁でない、またはいずれかの桁が1～5でない場合。
        """
        value = text.strip()
        if not POSITION_RE.fullmatch(value):
            raise ValueError(f"ゲートポジションは11～55の2桁で指定してください: {text!r}")
        return cls(x=int(value[0]), y=int(value[1]))

    def label(self) -> str:
        """ゲート位置を競技規約と同じ ``Gx-y`` 表記で返す。

        戻り値:
            例として ``G1-1`` や ``G3-5`` の形式にした文字列。
        """
        return f"G{self.x}-{self.y}"


@dataclass(frozen=True)
class GateHint:
    """1色のゲートを構成する、隣接した2つのゲートポジションを保持する。

    属性:
        color: ゲートの色を表す識別子。経路計算では ``red``、``blue``、
            ``yellow`` のいずれかを使用する。
        first: ヒント文字列で先に指定されたゲート端点。
        second: ヒント文字列で後に指定された、``first`` に隣接する端点。
    """

    color: str
    first: GridPosition
    second: GridPosition

    @classmethod
    def parse(cls, color: str, text: str) -> "GateHint":
        """色と2端点の文字列から、検証済みのゲート情報を作る。

        引数:
            color: ゲートの色を表す識別子。
            text: ``"11,21"`` のように2つのゲート位置をカンマで区切った文字列。

        戻り値:
            色と隣接する2端点を保持するゲート情報。

        例外:
            ValueError: 端点が2つでない、端点の形式が不正、または2端点が
                上下左右に隣接していない場合。
        """
        parts = [part.strip() for part in text.split(",")]
        if len(parts) != 2:
            raise ValueError(f"{color}ゲートはXY,XY形式で指定してください: {text!r}")
        first, second = map(GridPosition.parse, parts)
        if abs(first.x - second.x) + abs(first.y - second.y) != 1:
            raise ValueError(
                f"{color}ゲートの両端は隣接する必要があります: "
                f"{first.label()}, {second.label()}"
            )
        return cls(color=color, first=first, second=second)

    def encoded(self) -> str:
        """ゲートの2端点を無線送信用の ``XY,XY`` 形式へ戻す。

        戻り値:
            例として ``"11,21"`` の形式にしたゲート位置文字列。
        """
        return f"{self.first.x}{self.first.y},{self.second.x}{self.second.y}"


@dataclass(frozen=True)
class HintSet:
    """赤・青・黄ゲートの位置を保持する復号済みヒント一式。

    属性:
        red: ヒントカード1から取得した赤ゲートの位置。
        blue: 復号済みヒントカード2の前半から取得した青ゲートの位置。
        yellow: 復号済みヒントカード2の後半から取得した黄ゲートの位置。
    """

    red: GateHint
    blue: GateHint
    yellow: GateHint

    def ordered_gates(self) -> tuple[GateHint, GateHint, GateHint]:
        """競技規約の通過順序である赤→青→黄に並べたゲートを返す。

        戻り値:
            赤、青、黄の順に並んだ3つのゲート情報。
        """
        return self.red, self.blue, self.yellow


def parse_hint_set(hint1: str, hint2: str) -> HintSet:
    """ヒント1と復号済みヒント2を検証し、3色のゲート情報へ変換する。

    引数:
        hint1: 赤ゲートを示す ``XY,XY`` 形式のヒントカード1の値。
        hint2: 青ゲートと黄ゲートをこの順に並べた
            ``XY,XY/XY,XY`` 形式の復号済みヒントカード2の値。

    戻り値:
        競技規約に従って配置方向まで検証した赤・青・黄ゲート一式。

    例外:
        ValueError: 入力形式が不正、端点が隣接していない、赤・黄が横向きでない、
            青が縦向きでない、または複数色が同じゲート位置を指定した場合。
    """
    hint2_parts = [part.strip() for part in hint2.split("/")]
    if len(hint2_parts) != 2:
        raise ValueError(f"ヒント2はXY,XY/XY,XY形式で指定してください: {hint2!r}")
    hints = HintSet(
        red=GateHint.parse("red", hint1),
        blue=GateHint.parse("blue", hint2_parts[0]),
        yellow=GateHint.parse("yellow", hint2_parts[1]),
    )
    # 競技規約5.17.2: 赤・黄は横向き、青は縦向きに設置される。
    if hints.red.first.y != hints.red.second.y:
        raise ValueError("赤ゲートは横向きになる2点を指定してください")
    if hints.blue.first.x != hints.blue.second.x:
        raise ValueError("青ゲートは縦向きになる2点を指定してください")
    if hints.yellow.first.y != hints.yellow.second.y:
        raise ValueError("黄ゲートは横向きになる2点を指定してください")
    # 異なる色が同じ2端点を逆順で指定した場合も、同じ物理ゲートとして重複を検出する。
    physical_segments = [
        tuple(sorted(((gate.first.x, gate.first.y), (gate.second.x, gate.second.y))))
        for gate in hints.ordered_gates()
    ]
    if len(set(physical_segments)) != len(physical_segments):
        raise ValueError("赤・青・黄ゲートに同じ位置を重複指定できません")
    return hints
