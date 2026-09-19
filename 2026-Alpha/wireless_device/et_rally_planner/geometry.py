"""2D幾何計算ユーティリティ。座標は (x, y) のタプルで表す。"""

import math

Point = tuple  # (float, float)


# --- 基本のベクトル演算 ---
# 座標・ベクトルはどちらも同じ (x, y) タプルとして扱う(型としては区別しない)。


def sub(a, b):
    """ベクトル a - b (bからaへの向きのベクトル)。"""
    return (a[0] - b[0], a[1] - b[1])


def add(a, b):
    """ベクトル a + b。"""
    return (a[0] + b[0], a[1] + b[1])


def scale(a, s):
    """ベクトル a を s倍する(sが負なら向きが反転する)。"""
    return (a[0] * s, a[1] * s)


def dot(a, b):
    """内積。垂直なら0、同じ向きなら正、逆向きなら負になる。"""
    return a[0] * b[0] + a[1] * b[1]


def norm(a):
    """ベクトル a の長さ(原点からの距離)。"""
    return math.hypot(a[0], a[1])


def distance(a, b):
    """2点a, b間の距離。"""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def point_segment_distance(p, a, b):
    """点pと線分abの最短距離。"""
    return distance(p, closest_point_on_segment(p, a, b))


def closest_point_on_segment(p, a, b):
    """点pに最も近い、線分ab上の点。"""
    ab = sub(b, a)
    ab_len2 = dot(ab, ab)
    if ab_len2 == 0:
        return a
    t = dot(sub(p, a), ab) / ab_len2
    t = max(0.0, min(1.0, t))
    return add(a, scale(ab, t))


def segment_segment_distance(a1, a2, b1, b2):
    """線分a1-a2と線分b1-b2の最短距離。

    2本の線分の最短距離は、交差している(0)か、そうでなければ必ず
    どちらかの端点から相手の線分への最短距離のいずれかで実現される
    (計算幾何の標準的な性質)。そのため、4通りの点-線分距離の最小値を
    とればよい(交差判定を別途行わなくても、交差していれば少なくとも
    1つの端点-線分距離が0に非常に近い値になるため、実用上問題ない)。
    """
    return min(
        point_segment_distance(a1, b1, b2),
        point_segment_distance(a2, b1, b2),
        point_segment_distance(b1, a1, a2),
        point_segment_distance(b2, a1, a2),
    )


def segment_blocked_by_circle(a, b, center, radius, eps=1e-6):
    """線分abが円(center, radius)の内部を横切るならTrue。"""
    return point_segment_distance(center, a, b) < radius - eps


def collinear_param(a, b, p, tol=1e-6):
    """点pが、直線ab(aからbへ向かう向き)と同一直線上にあるかを判定する。
    乗っていれば a + t*(b-a) = p となるt(範囲は問わない、a/bの外側もOK)を、
    乗っていなければNoneを返す。"""
    d = sub(b, a)
    len2 = dot(d, d)
    if len2 < tol:
        return None
    t = dot(sub(p, a), d) / len2
    proj = add(a, scale(d, t))
    if distance(p, proj) > tol:
        return None
    return t


def circle_boundary_points(center, radius, n):
    """円周上にn個の等間隔な点を生成する(障害物回避のグラフノード用)。"""
    points = []
    for i in range(n):
        theta = 2 * math.pi * i / n
        points.append((center[0] + radius * math.cos(theta), center[1] + radius * math.sin(theta)))
    return points


def normalize_deg(angle_deg):
    """角度を(-180, 180]の範囲に正規化する。"""
    return (angle_deg + 180.0) % 360.0 - 180.0


def angle_in_arc(angle_deg, start_deg, end_deg):
    """angle_degが、start_degからend_degへの短い方の弧の範囲内にあるか。"""
    span = normalize_deg(end_deg - start_deg)
    rel = normalize_deg(angle_deg - start_deg)
    if span >= 0:
        return 0 <= rel <= span
    return span <= rel <= 0


def rotate(v, angle_deg):
    """ベクトルvを反時計回りにangle_deg度回転させる。"""
    rad = math.radians(angle_deg)
    c, s = math.cos(rad), math.sin(rad)
    return (v[0] * c - v[1] * s, v[0] * s + v[1] * c)
