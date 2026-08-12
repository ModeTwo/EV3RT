from enum import Enum
from collections import deque
import math

class SymmetricClamper:
    def __init__(self, min_val: float, max_val: float):
        assert 0 <= min_val <= max_val, "Require 0 <= min_val <= max_val"
        self.min_val = min_val
        self.max_val = max_val

    def clamp(self, value: float) -> float:
        if value > 0:
            return max(self.min_val, min(value, self.max_val))
        elif value < 0:
            return min(-self.min_val, max(value, -self.max_val))
        else:
            return 0.0

class Color(Enum):
    BLACK = "black"
    BLUE = "blue"
    RED = "red"
    YELLOW = "yellow"
    GREEN = "green"
    WHITE = "white"
    UNKNOWN = "unknown"

class ColorClassifier:
    _WINDOW_SIZE = 5

    def __init__(self):
        self.window: deque[int] = deque(maxlen=self._WINDOW_SIZE)

    # --- 収集データから導いた閾値 ---
    # black : V < 30, S < 25  (V avg≈26)
    # white : V > 55, S < 22  (V avg≈98, S avg≈5)
    # blue  : H in [205,220], S > 80  (H avg≈213, S avg≈93)
    # green : H in [140,155], S > 60  (H avg≈148, S avg≈73)
    # yellow: H in [35,65], V > 88    (H avg≈52, V avg≈99)
    # red   : H > 345 or H < 10, S > 75  (H avg≈354, S avg≈87)

    def classify_single(self,h: int, s: int, v: int) -> Color:
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


    def classify_robust(self, window: deque) -> Color:
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


    def classify(self, h: int, s: int, v: int) -> Color:
        single = self.classify_single(h, s, v)
        self.window.append(single)
        return self.classify_robust(self.window)

class LowPassFilter:
    """First-order IIR (exponential) low-pass filter.
 
    The cutoff is given in Hz so it carries physical meaning independent of the
    loop rate, then converted once to an EMA coefficient `alpha`:
 
        w     = 2*pi * cutoff_hz * sample_time
        alpha = w / (w + 1)
        y[n]  = y[n-1] + alpha * (x[n] - y[n-1])
 
    Higher cutoff -> alpha -> 1 -> less smoothing, less phase lag.
    Lower  cutoff -> alpha -> 0 -> more smoothing, more phase lag.
 
    Phase lag added at a frequency f is approximately atan(f / cutoff_hz);
    keep cutoff_hz well above the loop's working bandwidth (~2 Hz here) so the
    filter removes sensor spikes without eating the phase margin the PID needs.
    """
 
    def __init__(self, cutoff_hz: float, sample_time: float,
                 median_window: int = 0) -> None:
        w = 2.0 * math.pi * cutoff_hz * sample_time
        self.alpha = w / (w + 1.0)
        self.y = None                      # lazy init -> no start-up ramp from 0
        # optional tiny median pre-stage to reject single-sample spikes
        # (line crossings, glare). 0 disables it; 3 is a good value if enabled.
        self._mwin = median_window
        self._buf = []
 
    def reset(self) -> None:
        self.y = None
        self._buf = []
 
    def __call__(self, x: float) -> float:
        # optional spike rejection before smoothing
        if self._mwin > 1:
            self._buf.append(x)
            if len(self._buf) > self._mwin:
                self._buf.pop(0)
            x = sorted(self._buf)[len(self._buf) // 2]
        # exponential low-pass
        if self.y is None:
            self.y = x                     # seed with the first real sample
        else:
            self.y += self.alpha * (x - self.y)
        return self.y