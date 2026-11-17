"""2026-11-18(devSS): 押し出し後の旋回で土俵の上側の緑に車体がかかるかを見積もる(計算だけ・実機の部品に触れない)。

座標は相撲の押し出しの座標(sumo_relative_push の fixed_pose): x は進入方位の向き、y は進行方向の右が正。
向き a は「方位 - 進入方位」(度、時計回りが正)。
緑の側の符号 g は runtime.course と同じ(右コース -1 → 緑は y の負の側)。
"""

import math

# 車体(hybrid_route の params_day.json): 前 7.5cm + アーム 5cm、後ろ 17.9cm、半幅 7.0cm
BODY_FRONT_MM = 125.0
BODY_REAR_MM = 179.0
BODY_HALF_WIDTH_MM = 70.0
# その場旋回の中心は車軸の 9.5mm 後ろ(10/31 の実測)
SPIN_PIVOT_BACK_MM = 9.5
RING_RADIUS_MM = 185.0


def wrap(angle):
    return (angle + 180.0) % 360.0 - 180.0


def _corners():
    return (
        (BODY_FRONT_MM, BODY_HALF_WIDTH_MM), (BODY_FRONT_MM, -BODY_HALF_WIDTH_MM),
        (-BODY_REAR_MM, BODY_HALF_WIDTH_MM), (-BODY_REAR_MM, -BODY_HALF_WIDTH_MM),
    )


def _body_extreme(x, y, a_deg, g):
    """車軸 (x, y)・向き a のとき、車体の角のうち緑の側へ最も出ている g*y。"""
    a = math.radians(a_deg)
    c, s = math.cos(a), math.sin(a)
    return max(g * (y + cx * s + cy * c) for cx, cy in _corners())


def spin_extreme(x, y, a0_deg, turn_deg, g, steps=None):
    """その場旋回(中心は車軸の後ろ)で角が緑の側へ最も出る g*y と、旋回後の車軸。"""
    a0 = math.radians(a0_deg)
    px = x - SPIN_PIVOT_BACK_MM * math.cos(a0)
    py = y - SPIN_PIVOT_BACK_MM * math.sin(a0)
    steps = steps or max(8, int(abs(turn_deg)))
    worst = -float("inf")
    for i in range(steps + 1):
        a = a0_deg + turn_deg * i / steps
        ar = math.radians(a)
        ax = px + SPIN_PIVOT_BACK_MM * math.cos(ar)
        ay = py + SPIN_PIVOT_BACK_MM * math.sin(ar)
        worst = max(worst, _body_extreme(ax, ay, a, g))
    return worst


def reverse_arc(x, y, a0_deg, arc_deg, radius, g, steps=None):
    """後退しながら arc_deg(符号つき)だけ向きを変えたあとの (x, y, a) と、その間の角の g*y の最大。"""
    steps = steps or max(4, int(abs(arc_deg)))
    worst = _body_extreme(x, y, a0_deg, g)
    a = a0_deg
    if steps == 0 or arc_deg == 0:
        return x, y, a, worst
    step = arc_deg / steps
    ds = -radius * math.radians(abs(step))
    for _ in range(steps):
        mid = math.radians(a + step / 2.0)
        x += ds * math.cos(mid)
        y += ds * math.sin(mid)
        a += step
        worst = max(worst, _body_extreme(x, y, a, g))
    return x, y, a, worst


def turn_to_blue(x, y, a_deg, direction, blue):
    """青い丸へ向く旋回量(direction の向きに固定、最短方向にはしない)。"""
    target = math.degrees(math.atan2(blue[1] - y, blue[0] - x))
    if direction > 0:
        return (target - a_deg) % 360.0
    return -((a_deg - target) % 360.0)


def green_line(g, ring_center_y, gap_mm):
    """緑の縁の g*y。"""
    return g * ring_center_y + RING_RADIUS_MM + gap_mm


def plan(x, y, a0_deg, direction, g, blue, ring_center_y, gap_mm, need_margin,
         radius, max_arc, step_deg=5.0):
    """旋回だけのときの余裕と、余裕が need_margin 以上になる最小の後退円弧を返す。

    direction はその場旋回の向き(+1 時計回り)。円弧も同じ向きに回るので、後退すると緑から離れる。
    """
    line = green_line(g, ring_center_y, gap_mm)
    start = line - _body_extreme(x, y, a0_deg, g)
    plain_turn = turn_to_blue(x, y, a0_deg, direction, blue)
    plain = line - spin_extreme(x, y, a0_deg, plain_turn, g)
    result = dict(line=line, start_margin=start, plain_margin=plain, plain_turn=plain_turn,
                  arc=0.0, arc_margin=plain, end=(x, y, a0_deg))
    # 止まった時点ですでに緑に近いときは、それより悪くしないことを目標にする。
    goal = min(need_margin, start)
    if plain >= goal:
        return result
    best = None
    arc = step_deg
    while arc <= max_arc + 1e-6:
        ex, ey, ea, worst_arc = reverse_arc(x, y, a0_deg, direction * arc, radius, g)
        rest = turn_to_blue(ex, ey, ea, direction, blue)
        margin = line - max(worst_arc, spin_extreme(ex, ey, ea, rest, g))
        if best is None or margin > best["arc_margin"] + 1.0:
            best = dict(arc=arc, arc_margin=margin, end=(ex, ey, ea), rest_turn=rest)
        if margin >= goal:
            best = dict(arc=arc, arc_margin=margin, end=(ex, ey, ea), rest_turn=rest)
            break
        arc += step_deg
    if best is not None and best["arc_margin"] > plain:
        result.update(best)
    return result
