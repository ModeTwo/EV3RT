"""距離・方位から経路の横ずれを推定する。カラーセンサーの実測ではない。"""

import math


class PathTracking:
    def __init__(self, lookahead_mm, max_correction_deg):
        self.lookahead_mm = lookahead_mm
        self.max_correction_deg = max_correction_deg
        self.previous_distance = 0.0
        self.previous_heading = 0.0
        self.x = self.y = 0.0
        self.reference_x = self.reference_y = 0.0
        self.cross_track_mm = 0.0

    def correction(self, distance_mm, actual_heading_deg, heading_at):
        """開始位置は既知・開始方位0度と仮定。返すのは目標への追加角度。"""
        ds = distance_mm - self.previous_distance
        # 車軸中心の位置を、左右平均の移動距離と区間中央の方位で積算。
        middle_heading = math.radians((self.previous_heading + actual_heading_deg) / 2)
        self.x += ds * math.sin(middle_heading)
        self.y += ds * math.cos(middle_heading)

        # 計画側も同じ距離だけ積算。tick間隔が広い場合は5mm以下へ分割する。
        count = max(1, math.ceil(ds / 5.0))
        step = ds / count
        for i in range(count):
            s = self.previous_distance + (i + .5) * step
            h = math.radians(heading_at(s))
            self.reference_x += step * math.sin(h)
            self.reference_y += step * math.cos(h)
        self.previous_distance = distance_mm
        self.previous_heading = actual_heading_deg

        # 進行方向の右側を正とする横ずれ。右へ外れたら負の角度で戻す。
        h = math.radians(heading_at(distance_mm))
        self.cross_track_mm = ((self.x-self.reference_x)*math.cos(h)
                               - (self.y-self.reference_y)*math.sin(h))
        correction = -math.degrees(math.atan2(self.cross_track_mm, self.lookahead_mm))
        return max(-self.max_correction_deg, min(self.max_correction_deg, correction))


def curvature_turn(heading_at, distance_mm, power, wheel_tread_mm, gain, window_mm=30.0):
    """左右差動の理想式 turn = power × 車輪間隔/2 × 曲率(rad/mm)。

    PWMと車輪速度の比例は近似なので、実機差はgainで調整する。
    PDFの短いつなぎ目で出力が急増しないよう後方30mmで平均する。
    最初の500mm直線より前に旋回を始めない。
    """
    before = max(0.0, distance_mm-window_mm)
    if distance_mm <= before or gain == 0:
        return 0.0
    curvature = math.radians(heading_at(distance_mm)-heading_at(before)) / (distance_mm-before)
    return gain * power * wheel_tread_mm * curvature / 2.0
