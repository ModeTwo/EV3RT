"""開始位置と後続経路の縮尺を分けて、PDFの距離表を補正する。"""
import math

from .heading_profile import HeadingProfile

# 保存済みv1表の最初のカーブ位置。車体寸法ではなく元データの基準。
SOURCE_FIRST_STRAIGHT_MM = 500.0


def calibrated_profile(points, blue_start_mm, lap_gate_mm, *,
                       first_straight_mm=600.0, route_scale=1.0):
    """補正後の角度表・青位置・LAP位置を同じ距離基準で返す。"""
    if any(not math.isfinite(v) or v <= 0
           for v in (first_straight_mm, route_scale)):
        raise ValueError('first_straight_mm and route_scale must be positive and finite')

    def adjusted_distance(source_mm):
        if source_mm <= SOURCE_FIRST_STRAIGHT_MM:
            return source_mm * first_straight_mm / SOURCE_FIRST_STRAIGHT_MM
        return first_straight_mm + (source_mm - SOURCE_FIRST_STRAIGHT_MM) * route_scale

    profile = HeadingProfile((adjusted_distance(s), h) for s, h in points)
    return profile, adjusted_distance(blue_start_mm), adjusted_distance(lap_gate_mm)
