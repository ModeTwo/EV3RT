"""hybrid_route.py の重い計算を numba で速くする部品(2026-11-09)。計算式は hybrid_route.py の numpy 版と同じ。
numba が入っていない環境では AVAILABLE = False になり、hybrid_route.py は numpy 版のまま動く。
HR_NO_NUMBA=1 でも numpy 版を使う(結果を比べるため)。
"""
import math
import os

import numpy as np

AVAILABLE = False
if not os.environ.get('HR_NO_NUMBA'):
    try:
        from numba import njit
        AVAILABLE = True
    except ImportError:
        pass
if not AVAILABLE:
    def njit(*a, **k):
        return lambda f: f


@njit(cache=True)
def rect_front_rear(px, py, c, s, lc, lu, lv, lhu, lhv, corners, keep, front, rear, hw):
    """_rect_clear_front_rear と同じ。corners は (脚の数, 4, 2)。"""
    n = len(px)
    cf = np.empty(n); cr = np.empty(n)
    cxs = (0.0, 0.0, front, front, -rear, -rear)
    cys = (-hw, hw, -hw, hw, -hw, hw)
    for i in range(n):
        x, y, ci, si = px[i], py[i], c[i], s[i]
        d1f = np.inf; d1r = np.inf
        dd = [np.inf] * 6
        for k in keep:
            for j in range(4):
                dx = corners[k, j, 0] - x; dy = corners[k, j, 1] - y
                lx = dx * ci + dy * si; ly = -dx * si + dy * ci
                ey = max(abs(ly) - hw, 0.0)
                ef = max(max(-lx, lx - front), 0.0)
                er = max(max(-rear - lx, lx), 0.0)
                d1f = min(d1f, math.hypot(ef, ey)); d1r = min(d1r, math.hypot(er, ey))
            for j in range(6):
                wx = x + cxs[j] * ci - cys[j] * si; wy = y + cxs[j] * si + cys[j] * ci
                rx = wx - lc[k, 0]; ry = wy - lc[k, 1]
                du = abs(rx * lu[k, 0] + ry * lu[k, 1]) - lhu[k]
                dv = abs(rx * lv[k, 0] + ry * lv[k, 1]) - lhv[k]
                dd[j] = min(dd[j], math.hypot(max(du, 0.0), max(dv, 0.0)))
        cf[i] = min(d1f, min(dd[0], dd[1], dd[2], dd[3]))
        cr[i] = min(d1r, min(dd[4], dd[5], dd[0], dd[1]))
    return cf, cr


@njit(cache=True)
def rect_clear(px, py, c, s, x0, x1, hw, lc, lu, lv, lhu, lhv, corners):
    """_rect_clear(keep=None、x0・x1 はスカラー)と同じ。"""
    n = len(px)
    out = np.empty(n)
    cxs = (x0, x0, x1, x1)
    cys = (-hw, hw, -hw, hw)
    nl = len(lhu)
    for i in range(n):
        x, y, ci, si = px[i], py[i], c[i], s[i]
        d = np.inf
        for k in range(nl):
            for j in range(4):
                dx = corners[k, j, 0] - x; dy = corners[k, j, 1] - y
                lx = dx * ci + dy * si; ly = -dx * si + dy * ci
                d = min(d, math.hypot(max(max(x0 - lx, lx - x1), 0.0), max(abs(ly) - hw, 0.0)))
                wx = x + cxs[j] * ci - cys[j] * si; wy = y + cxs[j] * si + cys[j] * ci
                rx = wx - lc[k, 0]; ry = wy - lc[k, 1]
                du = abs(rx * lu[k, 0] + ry * lu[k, 1]) - lhu[k]
                dv = abs(rx * lv[k, 0] + ry * lv[k, 1]) - lhv[k]
                d = min(d, math.hypot(max(du, 0.0), max(dv, 0.0)))
        out[i] = d
    return out


@njit(cache=True)
def seg_cross(xs, ys, pax, pay, pbx, pby):
    """折れ線 xs,ys が線分 pa-pb を横切るか。"""
    for i in range(len(xs) - 1):
        ax, ay, bx, by = xs[i], ys[i], xs[i + 1], ys[i + 1]
        o1 = (bx - ax) * (pay - ay) - (by - ay) * (pax - ax)
        o2 = (bx - ax) * (pby - ay) - (by - ay) * (pbx - ax)
        o3 = (pbx - pax) * (ay - pay) - (pby - pay) * (ax - pax)
        o4 = (pbx - pax) * (by - pay) - (pby - pay) * (bx - pax)
        if o1 * o2 < 0 and o3 * o4 < 0:
            return True
    return False


@njit(cache=True)
def _ccw(x):
    TAU = 2 * math.pi
    x = x % TAU
    return 0.0 if x > TAU - 1e-7 else x


@njit(cache=True)
def _clip1(v):
    return max(-1.0, min(1.0, v))


@njit(cache=True)
def dubins_vec(x0, y0, t0, x1, y1, t1, r, INF):
    """hybrid_route.dubins_vec と同じ。8つの形の順番は WORDS と同じ。"""
    n = len(x0)
    A = np.empty((8, n)); L = np.empty((8, n))
    words = ((0, 1.0, 1.0), (0, 1.0, -1.0), (0, -1.0, 1.0), (0, -1.0, -1.0),
             (1, 1.0, 1.0), (1, 1.0, -1.0), (1, -1.0, 1.0), (1, -1.0, -1.0))
    S0 = np.sin(t0); C0 = np.cos(t0); S1 = np.sin(t1); C1 = np.cos(t1)
    for w in range(8):
        kind, s1, s2 = words[w]
        for i in range(n):
            st0, ct0, st1, ct1 = S0[i], C0[i], S1[i], C1[i]
            if kind == 0:
                c1x = x0[i] - s1 * r * st0; c1y = y0[i] + s1 * r * ct0
                c2x = x1[i] - s2 * r * st1; c2y = y1[i] + s2 * r * ct1
                dx, dy = c2x - c1x, c2y - c1y
                D = math.hypot(dx, dy); k = (s1 - s2) * r
                if D > abs(k) + 1e-9:
                    phi = math.atan2(dy, dx) + math.asin(_clip1(k / D))
                    A[w, i] = _ccw(s1 * (phi - t0[i])) + _ccw(s2 * (t1[i] - phi))
                    L[w, i] = math.sqrt(max(D * D - k * k, 0.0))
                else:
                    A[w, i] = INF; L[w, i] = 0.0
            else:
                s, sg = s1, s2
                c1x = x0[i] - s * r * st0; c1y = y0[i] + s * r * ct0
                c3x = x1[i] - s * r * st1; c3y = y1[i] + s * r * ct1
                dx, dy = c3x - c1x, c3y - c1y
                D = math.hypot(dx, dy)
                if 1e-9 < D < 4 * r:
                    al = math.atan2(dy, dx); be = math.acos(_clip1(D / (4 * r)))
                    c2x = c1x + 2 * r * math.cos(al + sg * be); c2y = c1y + 2 * r * math.sin(al + sg * be)
                    th1 = math.atan2(-(c1x - c2x) / (2 * s * r), (c1y - c2y) / (2 * s * r))
                    th2 = math.atan2(-(c3x - c2x) / (2 * s * r), (c3y - c2y) / (2 * s * r))
                    A[w, i] = _ccw(s * (th1 - t0[i])) + _ccw(-s * (th2 - th1)) + _ccw(s * (t1[i] - th2))
                    L[w, i] = 0.0
                else:
                    A[w, i] = INF; L[w, i] = 0.0
    return A, L


@njit(cache=True)
def min_over_e(cur, C):
    """(cur[:, :, None] + C).min(1) と同じ。cur は (K, nE)、C は (nE, nS)。"""
    K, nE = cur.shape
    nS = C.shape[1]
    out = np.full((K, nS), np.inf)
    for k in range(K):
        for e in range(nE):
            v = cur[k, e]
            if v == np.inf:
                continue
            for s in range(nS):
                t = v + C[e, s]
                if t < out[k, s]:
                    out[k, s] = t
    return out


@njit(cache=True)
def visit_min(bs, s_arr, e_arr, c_arr, nE):
    """s_r ごとに、抜けの姿勢ごとの一番短い時間。bs は (K, nS)。"""
    K = bs.shape[0]
    out = np.full((K, nE), np.inf)
    for k in range(K):
        for v in range(len(s_arr)):
            t = bs[k, s_arr[v]] + c_arr[v]
            e = e_arr[v]
            if t < out[k, e]:
                out[k, e] = t
    return out


@njit(cache=True)
def poses_near(px, py, lc, reach):
    """どれかの姿勢から reach 以内の脚の番号と、どれかの脚から reach 以内の姿勢の番号。"""
    n = len(px); m = len(lc)
    leg = np.zeros(m, np.bool_); pose = np.zeros(n, np.bool_)
    r2 = reach * reach
    for i in range(n):
        for j in range(m):
            dx = px[i] - lc[j, 0]; dy = py[i] - lc[j, 1]
            if dx * dx + dy * dy <= r2:
                leg[j] = True; pose[i] = True
    return np.flatnonzero(leg), np.flatnonzero(pose)


@njit(cache=True)
def keepout_hit(px, py, ch, sh, xs, ys, x0, x1, y0, y1, ko_corners, front, rear, hw):
    """車体の輪郭の点が長方形の中にあるか、長方形の角が車体の中にあるか(姿勢ごと)。"""
    n = len(px)
    hit = np.zeros(n, np.bool_)
    for i in range(n):
        c, s = ch[i], sh[i]
        for j in range(len(xs)):
            bx = px[i] + xs[j] * c - ys[j] * s; by = py[i] + xs[j] * s + ys[j] * c
            if bx > x0 and bx < x1 and by > y0 and by < y1:
                hit[i] = True
                break
        if not hit[i]:
            for j in range(len(ko_corners)):
                dx = ko_corners[j, 0] - px[i]; dy = ko_corners[j, 1] - py[i]
                lx = dx * c + dy * s; ly = -dx * s + dy * c
                if lx > -rear and lx < front and abs(ly) < hw:
                    hit[i] = True
                    break
    return hit


@njit(cache=True)
def dubins_params(ax, ay, tha, bx, by, thb, r, w):
    """hybrid_route.dubins_one の数値の部分。戻り値: (ok, 円弧1の角度, 真ん中の円弧の角度, 最後の円弧の角度, th1, th2,
    p1x, p1y, p2x, p2y, c1x, c1y, c2x, c2y)。CSC は p1→p2 が直進、th1=th2=phi。CCC の c2 は真ん中の円弧の中心、最後の円弧の中心は c3。"""
    TAU = 2 * math.pi
    kinds = (0, 0, 0, 0, 1, 1, 1, 1)
    s1s = (1.0, 1.0, -1.0, -1.0, 1.0, 1.0, -1.0, -1.0)
    s2s = (1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0)
    kind, s1, s2 = kinds[w], s1s[w], s2s[w]
    bad = (False, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    if kind == 0:
        c1x = ax - s1 * r * math.sin(tha); c1y = ay + s1 * r * math.cos(tha)
        c2x = bx - s2 * r * math.sin(thb); c2y = by + s2 * r * math.cos(thb)
        dx, dy = c2x - c1x, c2y - c1y
        D = math.hypot(dx, dy); k = (s1 - s2) * r
        if D <= abs(k) + 1e-9:
            return bad
        phi = math.atan2(dy, dx) + math.asin(k / D)
        p1x = c1x + s1 * r * math.sin(phi); p1y = c1y - s1 * r * math.cos(phi)
        p2x = c2x + s2 * r * math.sin(phi); p2y = c2y - s2 * r * math.cos(phi)
        a1 = _ccw(s1 * (phi - tha)); a3 = _ccw(s2 * (thb - phi))
        th_end = phi + s2 * a3
        ex = c2x + s2 * r * math.sin(th_end); ey = c2y - s2 * r * math.cos(th_end)
        ok = math.hypot(ex - bx, ey - by) <= 1e-3 and abs(((th_end - thb) * 180.0 / math.pi + 180.0) % 360.0 - 180.0) <= 1e-3
        return (ok, a1, 0.0, a3, phi, phi, p1x, p1y, p2x, p2y, c1x, c1y, c2x, c2y, c2x, c2y)
    s, sg = s1, s2
    c1x = ax - s * r * math.sin(tha); c1y = ay + s * r * math.cos(tha)
    c3x = bx - s * r * math.sin(thb); c3y = by + s * r * math.cos(thb)
    dx, dy = c3x - c1x, c3y - c1y
    D = math.hypot(dx, dy)
    if not (1e-9 < D < 4 * r):
        return bad
    al = math.atan2(dy, dx); be = math.acos(D / (4 * r))
    c2x = c1x + 2 * r * math.cos(al + sg * be); c2y = c1y + 2 * r * math.sin(al + sg * be)
    p1x = (c1x + c2x) / 2; p1y = (c1y + c2y) / 2; p2x = (c2x + c3x) / 2; p2y = (c2y + c3y) / 2
    th1 = math.atan2(-(c1x - p1x) / (s * r), (c1y - p1y) / (s * r))
    th2 = math.atan2(-(c3x - p2x) / (s * r), (c3y - p2y) / (s * r))
    a1 = _ccw(s * (th1 - tha)); a2 = _ccw(-s * (th2 - th1)); a3 = _ccw(s * (thb - th2))
    th_end = th2 + s * a3
    ex = c3x + s * r * math.sin(th_end); ey = c3y - s * r * math.cos(th_end)
    ok = math.hypot(ex - bx, ey - by) <= 1e-3 and abs(((th_end - thb) * 180.0 / math.pi + 180.0) % 360.0 - 180.0) <= 1e-3
    return (ok, a1, a2, a3, th1, th2, p1x, p1y, p2x, p2y, c1x, c1y, c2x, c2y, c3x, c3y)


@njit(cache=True)
def _box_gap(x, y, c, s, x0, x1, hw, lc, lu, lv, lhu, lhv, corners, reach):
    """1つの姿勢の車体の箱 x∈[x0,x1] と、近くの脚の長方形の最小のすき間。"""
    d = np.inf
    cxs = (x0, x0, x1, x1); cys = (-hw, hw, -hw, hw)
    for k in range(len(lhu)):
        if (x - lc[k, 0]) ** 2 + (y - lc[k, 1]) ** 2 > reach * reach:
            continue
        for j in range(4):
            dx = corners[k, j, 0] - x; dy = corners[k, j, 1] - y
            lx = dx * c + dy * s; ly = -dx * s + dy * c
            d = min(d, math.hypot(max(max(x0 - lx, lx - x1), 0.0), max(abs(ly) - hw, 0.0)))
            wx = x + cxs[j] * c - cys[j] * s; wy = y + cxs[j] * s + cys[j] * c
            rx = wx - lc[k, 0]; ry = wy - lc[k, 1]
            du = abs(rx * lu[k, 0] + ry * lu[k, 1]) - lhu[k]
            dv = abs(rx * lv[k, 0] + ry * lv[k, 1]) - lhv[k]
            d = min(d, math.hypot(max(du, 0.0), max(dv, 0.0)))
    return d


@njit(cache=True)
def _arc_hits(cx, cy, r, th, s, sweep, m, front, rear, hw, mf, mr, lc, lu, lv, lhu, lhv, corners, reach):
    """円弧の途中で、続けて2点すき間が足りなくなるなら True(確実に当たる)。"""
    if sweep < 1e-6:
        return False
    n = max(2, int(r * sweep) + 1, int(math.degrees(sweep)) + 1)
    prev = False
    add = 0.0 if m > 0 else math.pi
    for i in range(n + 1):
        t = th + s * sweep * i / n
        x = cx + s * r * math.sin(t); y = cy - s * r * math.cos(t)
        c = math.cos(t + add); si = math.sin(t + add)
        bad = (_box_gap(x, y, c, si, 0.0, front, hw, lc, lu, lv, lhu, lhv, corners, reach) < mf
               or _box_gap(x, y, c, si, -rear, 0.0, hw, lc, lu, lv, lhu, lhv, corners, reach) < mr)
        if bad and prev:
            return True
        prev = bad
    return False


@njit(cache=True)
def _line_hits(ax, ay, bx, by, m, front, rear, hw, mf, mr, lc, lu, lv, lhu, lhv, corners, reach):
    L = math.hypot(bx - ax, by - ay)
    if L < 1e-6:
        return False
    h = math.atan2(by - ay, bx - ax) + (0.0 if m > 0 else math.pi)
    c = math.cos(h); si = math.sin(h)
    if m > 0:
        f0, f1, r0, r1 = 0.0, L + front, -rear, L
    else:
        f0, f1, r0, r1 = -L, front, -L - rear, 0.0
    far = reach + L
    return (_box_gap(ax, ay, c, si, f0, f1, hw, lc, lu, lv, lhu, lhv, corners, far) < mf
            or _box_gap(ax, ay, c, si, r0, r1, hw, lc, lu, lv, lhu, lhv, corners, far) < mr)


@njit(cache=True)
def _pivot_hits(px, py, h_from, h_to, back, front, rear, hw, mf, mr, lc, lu, lv, lhu, lhv, corners, reach):
    """その場旋回(度)。1°おきの点が続けて2つ当たるなら True。"""
    dd = (h_to - h_from + 180.0) % 360.0 - 180.0
    if abs(dd) < 2.0:
        return False
    qx = px - back * math.cos(math.radians(h_from)); qy = py - back * math.sin(math.radians(h_from))
    n = int(abs(dd)) + 1
    prev = False
    for i in range(n + 1):
        a = math.radians(h_from + dd * i / n)
        c = math.cos(a); si = math.sin(a)
        x = qx + back * c; y = qy + back * si
        bad = (_box_gap(x, y, c, si, 0.0, front, hw, lc, lu, lv, lhu, lhv, corners, reach) < mf
               or _box_gap(x, y, c, si, -rear, 0.0, hw, lc, lu, lv, lhu, lhv, corners, reach) < mr)
        if bad and prev:
            return True
        prev = bad
    return False


@njit(cache=True)
def _piv_opt_hits(f, m, r, s1, ex, ey, he, sx, sy, hs, ds, tail, back, front, rear, hw, mf, mr, lc, lu, lv, lhu, lhv, corners, reach, de=1, lead=0.0):
    """その場旋回を含む候補(1=PLP, 2=PLA, 3=ALP)が確実に当たるなら True。形が作れないときも True(hybrid_route は None になる)。"""
    add = 0.0 if m > 0 else 180.0
    stop_s = ds == 0 or ds != m
    if f == 1:
        hl = math.degrees(math.atan2(sy - ey, sx - ex)) + add
        return (_line_hits(ex, ey, sx, sy, m, front, rear, hw, mf, mr, lc, lu, lv, lhu, lhv, corners, reach)
                or _pivot_hits(ex, ey, he, hl, back, front, rear, hw, mf, mr, lc, lu, lv, lhu, lhv, corners, reach)
                or _pivot_hits(sx, sy, hl, hs, back, front, rear, hw, mf, mr, lc, lu, lv, lhu, lhv, corners, reach))
    if f == 2:
        thb = math.radians(hs + add)
        bx = sx - tail * math.cos(thb) if stop_s else sx
        by = sy - tail * math.sin(thb) if stop_s else sy
        c2x = bx - s1 * r * math.sin(thb); c2y = by + s1 * r * math.cos(thb)
        D = math.hypot(c2x - ex, c2y - ey)
        if D <= r + 1e-9:
            return True
        phi = math.atan2(c2y - ey, c2x - ex) - math.asin(s1 * r / D)
        p2x = c2x + s1 * r * math.sin(phi); p2y = c2y - s1 * r * math.cos(phi)
        return (_line_hits(ex, ey, p2x, p2y, m, front, rear, hw, mf, mr, lc, lu, lv, lhu, lhv, corners, reach)
                or _pivot_hits(ex, ey, he, math.degrees(phi) + add, back, front, rear, hw, mf, mr, lc, lu, lv, lhu, lhv, corners, reach)
                or _arc_hits(c2x, c2y, r, phi, s1, _ccw(s1 * (thb - phi)), m, front, rear, hw, mf, mr, lc, lu, lv, lhu, lhv, corners, reach))
    tha = math.radians(he + add)
    if lead > 0.0 and (de == 0 or de != m):
        # 2026-11-11: 止まった状態からは lead の直進のあとに円弧(hybrid_route の lead_cm)
        ex = ex + lead * math.cos(tha); ey = ey + lead * math.sin(tha)
    c1x = ex - s1 * r * math.sin(tha); c1y = ey + s1 * r * math.cos(tha)
    D = math.hypot(sx - c1x, sy - c1y)
    if D <= r + 1e-9:
        return True
    phi = math.atan2(sy - c1y, sx - c1x) + math.asin(s1 * r / D)
    p1x = c1x + s1 * r * math.sin(phi); p1y = c1y - s1 * r * math.cos(phi)
    return (_arc_hits(c1x, c1y, r, tha, s1, _ccw(s1 * (phi - tha)), m, front, rear, hw, mf, mr, lc, lu, lv, lhu, lhv, corners, reach)
            or _line_hits(p1x, p1y, sx, sy, m, front, rear, hw, mf, mr, lc, lu, lv, lhu, lhv, corners, reach)
            or _pivot_hits(sx, sy, math.degrees(phi) + add, hs, back, front, rear, hw, mf, mr, lc, lu, lv, lhu, lhv, corners, reach))


@njit(cache=True)
def next_option(ex, ey, he, sx, sy, hs, ds, order, ptr, approx, inf2, fam, ms, rs, ws, tail, back, front, rear, hw, mf, mr, lc, lu, lv, lhu, lhv, corners, reach, de=1, lead=0.0):
    """今の順番 ptr から、「円弧-直進-円弧」の形で柱に確実に当たる候補を飛ばし、次に Python で調べる候補の番号を返す。
    円弧-直進-円弧 以外の形(その場旋回を含む)は、ここでは判断せずに返す。"""
    n = len(order)
    while ptr < n and approx[ptr] < inf2:
        o = order[ptr]
        if fam[o] != 0:
            if not _piv_opt_hits(fam[o], ms[o], rs[o], float(ws[o]), ex, ey, he, sx, sy, hs, ds, tail, back, front, rear, hw, mf, mr, lc, lu, lv, lhu, lhv, corners, reach, de, lead):
                return ptr
            ptr += 1
            continue
        m = ms[o]; r = rs[o]; w = ws[o]
        add = 0.0 if m > 0 else 180.0
        tha = math.radians(he + add); thb = math.radians(hs + add)
        stop_s = ds == 0 or ds != m
        bx = sx - tail * math.cos(thb) if stop_s else sx
        by = sy - tail * math.sin(thb) if stop_s else sy
        fx, fy = ex, ey
        if lead > 0.0 and (de == 0 or de != m):
            fx = ex + lead * math.cos(tha); fy = ey + lead * math.sin(tha)
        ok, a1, a2, a3, th1, th2, p1x, p1y, p2x, p2y, c1x, c1y, c2x, c2y, c3x, c3y = dubins_params(fx, fy, tha, bx, by, thb, r, w)
        if ok:
            s1 = 1.0 if w in (0, 1, 4, 5) else -1.0
            s2 = (1.0 if w in (0, 2) else -1.0) if w < 4 else s1
            hit = _arc_hits(c1x, c1y, r, tha, s1, a1, m, front, rear, hw, mf, mr, lc, lu, lv, lhu, lhv, corners, reach)
            if not hit:
                if w < 4:
                    L = math.hypot(p2x - p1x, p2y - p1y)
                    if L > 1e-6:
                        h = th1 if m > 0 else th1 + math.pi   # 車体の向き(動く向きの逆が後退)
                        c = math.cos(h); si = math.sin(h)
                        if m > 0:
                            f0, f1, r0, r1 = 0.0, L + front, -rear, L
                        else:
                            f0, f1, r0, r1 = -L, front, -L - rear, 0.0
                        far = reach + L
                        hit = (_box_gap(p1x, p1y, c, si, f0, f1, hw, lc, lu, lv, lhu, lhv, corners, far) < mf
                               or _box_gap(p1x, p1y, c, si, r0, r1, hw, lc, lu, lv, lhu, lhv, corners, far) < mr)
                    if not hit:
                        hit = _arc_hits(c2x, c2y, r, th2, s2, a3, m, front, rear, hw, mf, mr, lc, lu, lv, lhu, lhv, corners, reach)
                else:
                    hit = (_arc_hits(c2x, c2y, r, th1, -s1, a2, m, front, rear, hw, mf, mr, lc, lu, lv, lhu, lhv, corners, reach)
                           or _arc_hits(c3x, c3y, r, th2, s1, a3, m, front, rear, hw, mf, mr, lc, lu, lv, lhu, lhv, corners, reach))
            if not hit:
                return ptr
        ptr += 1
    return ptr


@njit(cache=True)
def layer(prev, C, s_arr, e_arr, c_arr, n_e):
    """solve_same_laps の layer と同じ(1周目と s_r 固定の復元)。戻り値 (cur, be, arg)。"""
    nE, nS = C.shape
    bs = np.full(nS, np.inf); be = np.zeros(nS, np.int64)
    for e in range(nE):
        v = prev[e]
        for s in range(nS):
            t = v + C[e, s]
            if t < bs[s]:
                bs[s] = t; be[s] = e
    cur = np.full(n_e, np.inf); arg = np.full(n_e, -1, dtype=np.int64)
    for v in range(len(s_arr)):
        t = bs[s_arr[v]] + c_arr[v]
        e = e_arr[v]
        if t < cur[e]:
            cur[e] = t; arg[e] = v
    return cur, be, arg
