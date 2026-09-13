"""開始位置と後続経路の縮尺を分けて、PDFの距離表を補正する。"""
import math

from .heading_profile import HeadingProfile

# 保存済みv1表の最初のカーブ位置。車体寸法ではなく元データの基準。
SOURCE_FIRST_STRAIGHT_MM = 500.0

# POINTS上で約90度曲がる区間。最初のカーブ(500～840mm)は含めない。
# 後続カーブが長い場合は、config.pyの倍率だけを変更する。
LATER_TURN_RANGES_MM = (
    (1310.0, 1660.0),
    (1675.0, 2385.0),
    (2395.0, 4475.0),
)


def calibrated_profile(points, blue_start_mm, lap_gate_mm, *,
                       first_straight_mm=600.0, route_scale=1.0,
                       later_turn_distance_scale=1.0,
                       third_turn_distance_scale=1.0,
                       second_turn_start_advance_mm=0.0,
                       third_turn_start_delay_mm=0.0):
    """補正後の角度表・青位置・LAP位置を同じ距離基準で返す。"""
    if any(not math.isfinite(v) or v <= 0
           for v in (first_straight_mm, route_scale, later_turn_distance_scale,
                     third_turn_distance_scale)):
        raise ValueError('distance scales must be positive and finite')
    if any(not math.isfinite(v) or v < 0
           for v in (second_turn_start_advance_mm, third_turn_start_delay_mm)):
        raise ValueError('turn start adjustments must be non-negative and finite')

    def adjusted_distance(source_mm):
        if source_mm <= SOURCE_FIRST_STRAIGHT_MM:
            return source_mm * first_straight_mm / SOURCE_FIRST_STRAIGHT_MM
        return first_straight_mm + (source_mm - SOURCE_FIRST_STRAIGHT_MM) * route_scale

    def turn_adjusted_distance(source_mm):
        """後続カーブ内だけ距離を縮め、後ろの全区間を同じ量だけ前へ詰める。"""
        distance_mm = adjusted_distance(source_mm)
        turn_scales = (
            later_turn_distance_scale,
            third_turn_distance_scale,
            later_turn_distance_scale,
        )
        for (turn_start_mm, turn_end_mm), turn_scale in zip(
                LATER_TURN_RANGES_MM, turn_scales):
            overlap_mm = max(0.0, min(source_mm, turn_end_mm) - turn_start_mm)
            distance_mm -= overlap_mm * route_scale * (1.0 - turn_scale)

        # 2番目は到着点を保ち、前の直線から旋回開始だけを滑らかに前倒しする。
        previous_turn_end_mm = 840.0
        second_start_mm, second_end_mm = LATER_TURN_RANGES_MM[0]
        if previous_turn_end_mm < source_mm < second_start_mm:
            ratio = (source_mm - previous_turn_end_mm) / (second_start_mm - previous_turn_end_mm)
            distance_mm -= second_turn_start_advance_mm * ratio
        elif second_start_mm <= source_mm < second_end_mm:
            remaining = (second_end_mm - source_mm) / (second_end_mm - second_start_mm)
            distance_mm -= second_turn_start_advance_mm * remaining

        # 3番目以降を同じ量だけ後ろへ移し、3番目の旋回開始を遅らせる。
        third_start_mm = LATER_TURN_RANGES_MM[1][0]
        if source_mm >= third_start_mm:
            distance_mm += third_turn_start_delay_mm
        return distance_mm

    profile = HeadingProfile((turn_adjusted_distance(s), h) for s, h in points)
    return (
        profile,
        turn_adjusted_distance(blue_start_mm),
        turn_adjusted_distance(lap_gate_mm),
    )
