# 【日本語解説】 Raspberry PiからSPIKEとWebカメラを連携させ、ETロボコン2026の走行・画像認識を制御する。
# 【日本語解説】 行動木の各update()は短時間で1周期だけ処理し、完了まではRUNNINGを返す。
from enum import Enum
from collections import deque
import math

# 【日本語解説】 制御量の符号を保ったまま、絶対値を最小値から最大値の範囲へ制限するクラス。
class SymmetricClamper:
    # 【日本語解説】 SymmetricClamperの設定値と実行中に保持する状態を初期化する。
    def __init__(self, min_val: float, max_val: float):
        # 【引数】 min_val: 符号付き制限で許可する絶対値の下限。
        # 【引数】 max_val: 符号付き制限で許可する絶対値の上限。
        assert 0 <= min_val <= max_val, "Require 0 <= min_val <= max_val"
        self.min_val = min_val
        self.max_val = max_val

    # 【日本語解説】 値の符号を維持し、絶対値を設定範囲へ丸めて返す。
    def clamp(self, value: float) -> float:
        # 【引数】 value: 上下限へ丸める入力値。
        if value > 0:
            return max(self.min_val, min(value, self.max_val))
        elif value < 0:
            return min(-self.min_val, max(value, -self.max_val))
        else:
            return 0.0

# 【日本語解説】 カラーセンサーで識別する色と判定不能状態を表す列挙型。
class Color(Enum):
    BLACK = "black"
    BLUE = "blue"
    RED = "red"
    YELLOW = "yellow"
    GREEN = "green"
    WHITE = "white"
    UNKNOWN = "unknown"

# 【日本語解説】 HSV値を単発判定し、直近の多数決でノイズを抑えて色を確定するクラス。
class ColorClassifier:
    _WINDOW_SIZE = 5

    # 【日本語解説】 ColorClassifierの設定値と実行中に保持する状態を初期化する。
    def __init__(self):
        self.window: deque[int] = deque(maxlen=self._WINDOW_SIZE)

    # --- 収集データから導いた閾値 ---
    # 現在の制御状態に必要な値を更新し、次の処理へ進む。

    # 【日本語解説】 1組のHSV値をしきい値と比較し、対応する色を返す。
    def classify_single(self,h: int, s: int, v: int) -> Color:
        # 【引数】 h: 色相値（度）。
        # 【引数】 s: 彩度値。
        # 【引数】 v: 明度値。
        """1サンプルのHSVから色を判定する。"""
        # 純白：収集データのS最大値は19。S<20 かつ高輝度のみ白と認める
        if s < 20 and v > 75:
            return Color.WHITE
        # 有彩色判定（S が高い領域）
        # 青は境界でSが59〜71まで低下するため閾値を緩める
        if 205 <= h <= 220 and s > 50:
            return Color.BLUE
        if 140 <= h <= 155 and s > 60:
            return Color.GREEN
        if 35 <= h <= 65 and v > 88:
            return Color.YELLOW
        if (h > 345 or h < 10) and s > 75:
            return Color.RED
        # 上記いずれにも該当しない低彩度(S<35)はすべて黒扱い
        # 外乱光でVが上昇しても、彩度が低ければ有色ラインではない
        if s < 35:
            return Color.BLACK
        return Color.UNKNOWN


    # 【日本語解説】 直近の色判定を多数決し、一時的な照明変化や走行振動の影響を抑える。
    def classify_robust(self, window: deque) -> Color:
        # 【引数】 window: 直近の色判定結果を保持する両端キュー。
        """
        直近 WINDOW_SIZE サンプルの多数決で色を決定する。
        走行体の左右揺れや外乱光による一時的なノイズを抑制する。
        黒判定は除外し、有色・白の多数決を優先する。
        """
        if not window:
            return Color.UNKNOWN

        votes: dict[Color, int] = {}
        for color in window:
            votes[color] = votes.get(color, 0) + 1

        # 黒以外の票を集計し、過半数なら採用
        non_black = {k: v for k, v in votes.items() if k != Color.BLACK}
        if non_black:
            best = max(non_black, key=lambda k: non_black[k])
            if non_black[best] >= self._WINDOW_SIZE // 2 + 1:
                return best

        # 黒が最多なら黒
        return max(votes, key=lambda k: votes[k])


    # 【日本語解説】 単発の色判定を履歴へ追加し、平滑化済みの最終判定を返す。
    def classify(self, h: int, s: int, v: int) -> Color:
        # 【引数】 h: 色相値（度）。
        # 【引数】 s: 彩度値。
        # 【引数】 v: 明度値。
        single = self.classify_single(h, s, v)
        self.window.append(single)
        return self.classify_robust(self.window)

# 【日本語解説】 センサー値の突発ノイズを抑える一次IIRローパスフィルター。
class LowPassFilter:
    """一次IIR（指数移動平均）ローパスフィルター。
    
        遮断周波数とサンプリング周期から係数を計算する。遮断周波数を上げると平滑化と位相遅れが小さくなり、
        下げると平滑化と位相遅れが大きくなる。必要に応じて前段の中央値フィルターで単発ノイズも除去する。
    """
 
    # 【日本語解説】 LowPassFilterの設定値と実行中に保持する状態を初期化する。
    def __init__(self, cutoff_hz: float, sample_time: float,
                 median_window: int = 0) -> None:
        # 【引数】 cutoff_hz: ローパスフィルターの遮断周波数（Hz）。
        # 【引数】 sample_time: 入力値を更新するサンプリング周期（秒）。
        # 【引数】 median_window: 単発ノイズ除去に使う中央値フィルターのサンプル数。0で無効。
        w = 2.0 * math.pi * cutoff_hz * sample_time
        self.alpha = w / (w + 1.0)
        self.y = None                      # 初回入力値で初期化し、ゼロ開始による立ち上がり遅延を避ける
        # 必要に応じて短い中央値フィルターを前段に置き、単発ノイズを除去する
        # 単発ノイズ除去に使う中央値フィルターの窓幅。0で無効。
        self._mwin = median_window
        self._buf = []
 
    # 【日本語解説】 保持しているフィルター出力と中央値判定用バッファを初期状態へ戻す。
    def reset(self) -> None:
        self.y = None
        self._buf = []
 
    # 【日本語解説】 LowPassFilterを関数のように呼び出し、入力値に対する処理結果を返す。
    def __call__(self, x: float) -> float:
        # 平滑化の前に、必要に応じて単発ノイズを除去する
        # 【引数】 x: フィルターへ入力する現在のサンプル値。
        if self._mwin > 1:
            self._buf.append(x)
            if len(self._buf) > self._mwin:
                self._buf.pop(0)
            x = sorted(self._buf)[len(self._buf) // 2]
        # 指数移動平均によるローパス処理
        if self.y is None:
            self.y = x  # 初回入力値でフィルターを初期化し、ゼロ始動による遅れを防ぐ。
        else:
            self.y += self.alpha * (x - self.y)
        return self.y