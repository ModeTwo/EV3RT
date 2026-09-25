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
    # 無彩色判定
    _ACHROMATIC_MAX_S = 45
    #白黒境界
    _WHITE_MIN_V = 83
    _BLACK_MAX_V = 82

     # 青判定
    _BLUE_MIN_H = 205
    _BLUE_MAX_H = 215
    _BLUE_MIN_S = 60
    _BLUE_MIN_V = 70
    _BLUE_MAX_V = 100

    def __init__(self):
        self.window: deque[color] = deque(maxlen=self._WINDOW_SIZE)

    def classify_single(self,h: int, s: int, v: int) -> Color:
         """
        1サンプルのHSV値から色を判定する。

        判定順序:
            1. 青などの有彩色
            2. 白
            3. 黒
            4. UNKNOWN

        青ラインはS（彩度）が白・黒より大幅に高いため、
        白黒より先に判定する。
        """

        # --------------------------------------------------------
        # BLUE
        #
        # 実測値:
        # H=207～212
        # S=67～96
        # V=77～95
        #
        # 白・黒はS≒26～32なので、
        # S>=60を条件にすることで誤検知を抑える。
        # --------------------------------------------------------
         if (
            self._BLUE_MIN_H <= h <= self._BLUE_MAX_H
            and s >= self._BLUE_MIN_S
            and self._BLUE_MIN_V <= v <= self._BLUE_MAX_V
        ):
            return Color.BLUE
         # GREEN
         if 140 <= h <= 155 and s > 60:
            return Color.GREEN

        # YELLOW
         if 35 <= h <= 65 and v > 88:
            return Color.YELLOW

        # RED
         if (h > 345 or h < 10) and s > 75:
            return Color.RED

        # --------------------------------------------------------
        # WHITE
        #
        # 白・黒はHが安定しないため、
        # Hは使用せずS・Vを使用する。
        #
        # V >= 83 を暫定的にWHITEとする。
        # --------------------------------------------------------
         if (
            s <= self._ACHROMATIC_MAX_S
            and v >= self._WHITE_MIN_V
        ):
            return Color.WHITE

        # --------------------------------------------------------
        # BLACK
        #
        # 今回の黒ラインは以前想定していたV<=45ではなく、
        # V=60～80台が多数確認された。
        #
        # そのためBLACK_MAX_Vを82まで拡張する。
        # --------------------------------------------------------
         if (
            s <= self._ACHROMATIC_MAX_S
            and v <= self._BLACK_MAX_V
        ):
            return Color.BLACK

         return Color.UNKNOWN


    def classify_robust(self, window: deque) -> Color:
        """
        直近5サンプルの多数決で最終的な色を決定する。

        一瞬だけ青ラインを横切った場合や、
        外乱光による1サンプルだけの誤検知を抑制する。
        """
        if not window:
            return Color.UNKNOWN

        votes: dict[Color, int] = {}
        for color in window:
            votes[color] = votes.get(color, 0) + 1

        # 最多票の色を取得
        best = max(votes, key=lambda color: votes[color])

        # 5サンプル中3サンプル以上一致した場合のみ確定
        if votes[best] >= 3:
            return best

        return Color.UNKNOWN

    def classify(self, h: int, s: int, v: int) -> Color:
        """
        HSV値を入力し、多数決を考慮した最終的な色を返す。
        """
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
