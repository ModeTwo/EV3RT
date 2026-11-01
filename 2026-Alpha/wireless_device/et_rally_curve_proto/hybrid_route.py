"""試作: 前進の円弧・後退・その場の旋回を、つなぎ区間ごとに混ぜて選ぶ経路の探索(4周+ゴール)。

ゲートの通り方は F(前進)/B(後退)/FB(前進→後退の往復)/BF、どちら側から、手前と奥の余り(ext)、
ゲートの中での横の位置(offsets)の組み合わせ。つなぎ区間(前のゲートの終わりの姿勢→次のゲートの始めの姿勢)は、
  1. 1本の走り(前進または後退の Dubins 経路: 円弧-直進-円弧 / 円弧-円弧-円弧)
  2. その場の旋回を使うもの(旋回-直進-旋回、旋回-直進-円弧、円弧-直進-旋回)
  3. 中継を1つ経由するもの(1と2で当たらないときだけ: 旋回-直進-旋回-直進-旋回、走り-走り)
から、当たらずに最短のものを選ぶ。止まる(前後の切り替え・旋回の前・ゴール)直前は直進で終える(tail_cm)。
順番どおりのゲート以外の線は横切らない(同じ色の重複はよいが、色の順番が変わる走行は規則違反)。

探索は「当たり判定を後回しにする」動的計画法: まず当たりを無視した時間で最短経路を出し、使ったつなぎ区間だけ
厳密に判定する。当たっていたら、その区間の次の候補に替えて解き直す(使わない区間は判定しない)。

時間(秒)と精度の重みは設定で変えられる(HR_PARAMS=設定JSONのパス、または HR_<名前>=値)。
usage: hybrid_route.py gates.json out_curved.json    (出力は curve_route.py と同じ形式。to_plan.py に渡せる)
"""
import json, math, sys, os, time
import numpy as np

# ---------------- 設定 ----------------
PARAMS = dict(
    laps=4,
    # 車体と当たり判定(cm)
    front=7.5, rear=17.5, half_width=6.85, margin_front=1.5, margin_rear=0.4,
    # 経路の形
    radii=[20.0, 15.0, 12.0, 10.0], tail_cm=4.0,
    ext=[0.0, 4.0, 8.0, 14.0, 22.0, 35.0], offsets=[-2.5, 0.0, 2.5],
    allow_reverse=True, allow_pivot=True,
    # 時間: 直進 cm/s、円弧は arc_k*v_line/(1+half_tread/r)、後退は rev_speed_scale 倍の速さ
    v_line=36.0, arc_k=0.77, half_tread=5.85, rev_speed_scale=1.0,
    t_stop=0.67, t_turn0=0.7, t_turn_deg=0.012,
    # 精度の重み(秒に換算して時間に足す。0なら時間だけで選ぶ)
    pen_stop=0.0, pen_pivot=0.0, pen_rev_m=0.0, pen_arc_100deg=0.0,
    # 中継
    wp_step=8.0, wp_headings=12, wp_tries=60, via_max_tries=150,
)
if os.environ.get('HR_PARAMS'):
    PARAMS.update(json.load(open(os.environ['HR_PARAMS'], encoding='utf-8')))
for k in list(PARAMS):
    v = os.environ.get('HR_' + k.upper())
    if v is not None:
        d = PARAMS[k]
        PARAMS[k] = ([float(x) for x in v.split(',')] if isinstance(d, list) else
                     (v == '1' if isinstance(d, bool) else type(d)(v)))
globals().update({k.upper(): v for k, v in PARAMS.items()})

P = 24.64
POST_R, ARM = 1.0, 4.0
PM, CLR = 2.0, 0.5
INF = 1e9
TAU = 2 * math.pi
START = np.array([2 * 2 * P - (4 * P + 17.8), 3.5 * P]); START_H = 0.0
GOAL = np.array([2 * 2 * P - (0 * P - 14.9), 3.5 * P]); GOAL_H = 90.0
_mx = lambda x: 4 * P - x
KEEPOUT = [(_mx(INF), _mx(-17.0), -INF, -37.0),
           (_mx(INF), _mx(4 * P + 17.8 + 14.0), -INF, INF),
           (_mx(-28.0), INF, 0.0, INF)]
BOUNDS = (-31.0, 126.0, -36.0, 4 * P + 40)


def nd(a):
    return (a + 180.0) % 360.0 - 180.0


def v_arc(r):
    return ARC_K * V_LINE / (1 + HALF_TREAD / r)


def spd(m):
    return 1.0 if m > 0 else 1.0 / REV_SPEED_SCALE


def turn_cost(deg):
    return 0.0 if deg < 0.3 else T_TURN0 + T_TURN_DEG * deg


# ---------------- ゲート ----------------
class Gate:
    def __init__(self, color, a, b):
        self.color = color
        self.pa = np.array([a[0] * P, a[1] * P], float); self.pb = np.array([b[0] * P, b[1] * P], float)
        self.c = (self.pa + self.pb) / 2
        self.t = (self.pb - self.pa) / np.linalg.norm(self.pb - self.pa)
        self.n = np.array([-self.t[1], self.t[0]])


GJ = json.load(open(sys.argv[1], encoding='utf-8'))['gates']
GATES = {c: Gate(c, GJ[c]['a'], GJ[c]['b']) for c in ('red', 'blue', 'yellow')}
CAP = np.array([p + g.n * ARM * (2 * t - 1) for g in GATES.values() for p in (g.pa, g.pb) for t in np.linspace(0, 1, 9)])
GSEG = [(g.pa, g.pb, g.color) for g in GATES.values()]


# ---------------- 当たり判定 ----------------
def box_clear(px, py, h_rad, x0, x1):
    """姿勢(配列)の車体の箱 x∈[x0,x1], |y|<=HW と支柱の最小すき間(配列)。x0,x1 はスカラーか配列。"""
    c = np.cos(h_rad)[:, None]; s = np.sin(h_rad)[:, None]
    dx = CAP[None, :, 0] - px[:, None]; dy = CAP[None, :, 1] - py[:, None]
    lx = dx * c + dy * s; ly = -dx * s + dy * c
    x0 = np.asarray(x0, float).reshape(-1, 1) if np.ndim(x0) else x0
    x1 = np.asarray(x1, float).reshape(-1, 1) if np.ndim(x1) else x1
    ex = np.maximum(np.maximum(x0 - lx, lx - x1), 0.0)
    ey = np.maximum(np.abs(ly) - HALF_WIDTH, 0.0)
    return np.min(np.hypot(ex, ey), axis=1) - POST_R


def poses_clear(px, py, h_deg):
    """姿勢ごとに、前の部分と後ろの部分のすき間が足りているか(bool配列)。"""
    h = np.radians(h_deg)
    c = np.cos(h)[:, None]; s = np.sin(h)[:, None]
    dx = CAP[None, :, 0] - px[:, None]; dy = CAP[None, :, 1] - py[:, None]
    lx = dx * c + dy * s; ly = -dx * s + dy * c
    ey = np.maximum(np.abs(ly) - HALF_WIDTH, 0.0)
    ef = np.maximum(np.maximum(-lx, lx - FRONT), 0.0)
    er = np.maximum(np.maximum(-REAR - lx, lx), 0.0)
    cf = np.min(np.hypot(ef, ey), axis=1) - POST_R
    cr = np.min(np.hypot(er, ey), axis=1) - POST_R
    return (cf >= MARGIN_FRONT) & (cr >= MARGIN_REAR)


def poses_ok(px, py, h_deg):
    return bool(np.all(poses_clear(np.asarray(px, float), np.asarray(py, float), np.asarray(h_deg, float))))


_piv_int = {}


def pivot_interval(p, h):
    """点pで向きhから、その場で回れる最大の角度(左回り, 右回り)。"""
    key = (round(float(p[0]), 3), round(float(p[1]), 3), round(float(h), 3))
    if key not in _piv_int:
        ks = np.arange(1, 182)
        hs = np.concatenate([h + ks, h - ks])
        ok = poses_clear(np.full(len(hs), p[0]), np.full(len(hs), p[1]), hs)
        res = []
        for part in (ok[:181], ok[181:]):
            bad = np.flatnonzero(~part)
            res.append(181 if len(bad) == 0 else int(bad[0]))
        _piv_int[key] = tuple(res)
    return _piv_int[key]


def in_keepout(px, py):
    bad = np.zeros(len(px), bool)
    for x0, x1, y0, y1 in KEEPOUT:
        bad |= (px > x0) & (px < x1) & (py > y0) & (py < y1)
    return bad


def _orient(ax, ay, bx, by, cx, cy):
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)


def crosses_gate(xs, ys, allow=None):
    """折れ線(xs,ys)がゲートの線を横切るか(allowの色は除く)。"""
    ax, ay, bx, by = xs[:-1], ys[:-1], xs[1:], ys[1:]
    for pa, pb, col in GSEG:
        if col == allow:
            continue
        o1 = _orient(ax, ay, bx, by, pa[0], pa[1]); o2 = _orient(ax, ay, bx, by, pb[0], pb[1])
        o3 = _orient(pa[0], pa[1], pb[0], pb[1], ax, ay); o4 = _orient(pa[0], pa[1], pb[0], pb[1], bx, by)
        if np.any((o1 * o2 < 0) & (o3 * o4 < 0)):
            return True
    return False


def line_ok(a, b, m, allow=None):
    v = b - a; L = float(np.hypot(*v))
    if L < 1e-6:
        return True
    h = math.atan2(v[1], v[0]) + (0 if m > 0 else math.pi)
    if m > 0:
        f0, f1, r0, r1 = 0.0, L + FRONT, -REAR, L
    else:
        f0, f1, r0, r1 = -L, FRONT, -L - REAR, 0.0
    px, py, hh = np.array([a[0]]), np.array([a[1]]), np.array([h])
    if box_clear(px, py, hh, f0, f1)[0] < MARGIN_FRONT or box_clear(px, py, hh, r0, r1)[0] < MARGIN_REAR:
        return False
    n = max(2, int(L / 2) + 1)
    xs = np.linspace(a[0], b[0], n); ys = np.linspace(a[1], b[1], n)
    if np.any(in_keepout(xs, ys)):
        return False
    return not crosses_gate(xs, ys, allow)


def nL(phi):
    return np.array([-math.sin(phi), math.cos(phi)])


def tdir(phi):
    return np.array([math.cos(phi), math.sin(phi)])


def ccw(x):
    x = x % TAU
    return 0.0 if x > TAU - 1e-7 else x


def arc_prim(p, theta, s, sweep, r, m):
    """点pで進む向き(動く向き)theta、s=+1左/-1右に sweep(rad) の円弧。m=+1前進/-1後退。"""
    return ('arc', p + s * r * nL(theta), r, theta, s, sweep, m, p)


def arc_end(pr):
    _, c, r, theta, s, sweep, m, p0 = pr
    th = theta + s * sweep
    return c - s * r * nL(th), th


def arc_samples(pr, step=1.0):
    _, c, r, theta, s, sweep, m, p0 = pr
    n = max(2, int(r * sweep / step) + 1)
    th = theta + s * sweep * np.arange(n + 1) / n
    xs = c[0] + s * r * np.sin(th); ys = c[1] - s * r * np.cos(th)
    hb = np.degrees(th) + (0 if m > 0 else 180)
    return xs, ys, hb


def arc_ok(pr):
    if pr[5] < 1e-6:
        return True
    xs, ys, hb = arc_samples(pr)
    if not poses_ok(xs, ys, hb):
        return False
    if np.any(in_keepout(xs, ys)):
        return False
    return not crosses_gate(xs, ys)


_pivot_cache = {}


def pivot_ok(p, h_from, h_to):
    dd = nd(h_to - h_from)
    if abs(dd) < 0.3:
        return True
    key = (round(float(p[0]), 2), round(float(p[1]), 2), round(h_from, 2), round(h_to, 2))
    if key not in _pivot_cache:
        hs = h_from + dd * np.linspace(0, 1, int(abs(dd)) + 2)
        _pivot_cache[key] = poses_ok(np.full(len(hs), p[0]), np.full(len(hs), p[1]), hs)
    return _pivot_cache[key]


def prim_ok(pr):
    if pr[0] == 'line':
        return line_ok(pr[1], pr[2], pr[3])
    if pr[0] == 'arc':
        return arc_ok(pr)
    return pivot_ok(pr[1], pr[2], pr[3])


def prim_len(pr):
    if pr[0] == 'line':
        return float(np.hypot(*(pr[2] - pr[1])))
    if pr[0] == 'arc':
        return pr[2] * pr[5]
    return 0.0


def prim_m(pr):
    return pr[3] if pr[0] == 'line' else (pr[6] if pr[0] == 'arc' else None)


def seq_cost(dE, prims, dS):
    """前のゲートの最後の走りの向きdE(0=止まっている)、つなぎのプリミティブ、次のゲートの最初の走りの向きdS(0=ゴール)。
    時間+重み。止まる直前が直進(tail_cm以上)でなければINF。"""
    t = 0.0; stops = 0; pivots = 0; rev = 0.0; arcdeg = 0.0
    cur = dE            # 今の動きの向き(0=止まっている)
    last = 'visit'      # 止まる直前の区間('visit'は通過の直進)
    last_len = INF
    pending_pivot = False

    def stop_here():
        nonlocal stops, cur
        if cur != 0:
            if not (last == 'visit' or (last == 'line' and last_len >= TAIL_CM - 1e-6)):
                return False
            stops += 1
        cur = 0
        return True

    for pr in prims:
        if pr[0] == 'pivot':
            deg = abs(nd(pr[3] - pr[2]))
            if deg < 0.3:
                continue
            if not stop_here():
                return INF
            t += turn_cost(deg); pivots += 1
            continue
        L = prim_len(pr)
        if L < 1e-6:
            continue
        m = prim_m(pr)
        if cur != 0 and cur != m:
            if not stop_here():
                return INF
        t += (L / V_LINE if pr[0] == 'line' else L / v_arc(pr[2])) * spd(m)
        if m < 0:
            rev += L
        if pr[0] == 'arc':
            arcdeg += math.degrees(pr[5])
        cur = m; last = pr[0]; last_len = L
    if dS == 0 or (cur != 0 and cur != dS):
        if not stop_here():
            return INF
    return (t + T_STOP * stops + PEN_STOP * stops + PEN_PIVOT * pivots + PEN_REV_M * rev / 100.0
            + PEN_ARC_100DEG * arcdeg / 100.0)


# ---------------- Dubins(配列) ----------------
WORDS = [('CSC', 1, 1), ('CSC', 1, -1), ('CSC', -1, 1), ('CSC', -1, -1),
         ('CCC', 1, 1), ('CCC', 1, -1), ('CCC', -1, 1), ('CCC', -1, -1)]


def _ccw_v(x):
    x = np.mod(x, TAU)
    return np.where(x > TAU - 1e-7, 0.0, x)


def dubins_vec(x0, y0, t0, x1, y1, t1, r):
    """8つの形の (円弧の角度の合計rad, 直進の長さcm)。だめなものは角度INF。shape (8, N)。"""
    A, L = [], []
    for kind, s1, s2 in WORDS:
        if kind == 'CSC':
            c1x = x0 - s1 * r * np.sin(t0); c1y = y0 + s1 * r * np.cos(t0)
            c2x = x1 - s2 * r * np.sin(t1); c2y = y1 + s2 * r * np.cos(t1)
            dx, dy = c2x - c1x, c2y - c1y
            D = np.hypot(dx, dy); k = (s1 - s2) * r
            ok = D > abs(k) + 1e-9
            Ds = np.where(ok, D, 1.0)
            phi = np.arctan2(dy, dx) + np.arcsin(np.clip(k / Ds, -1, 1))
            l = np.sqrt(np.maximum(Ds ** 2 - k ** 2, 0))
            a = _ccw_v(s1 * (phi - t0)) + _ccw_v(s2 * (t1 - phi))
        else:
            s, sg = s1, s2
            c1x = x0 - s * r * np.sin(t0); c1y = y0 + s * r * np.cos(t0)
            c3x = x1 - s * r * np.sin(t1); c3y = y1 + s * r * np.cos(t1)
            dx, dy = c3x - c1x, c3y - c1y
            D = np.hypot(dx, dy)
            ok = (D > 1e-9) & (D < 4 * r)
            al = np.arctan2(dy, dx); be = np.arccos(np.clip(D / (4 * r), -1, 1))
            c2x = c1x + 2 * r * np.cos(al + sg * be); c2y = c1y + 2 * r * np.sin(al + sg * be)
            th1 = np.arctan2(-(c1x - c2x) / (2 * s * r), (c1y - c2y) / (2 * s * r))
            th2 = np.arctan2(-(c3x - c2x) / (2 * s * r), (c3y - c2y) / (2 * s * r))
            a = _ccw_v(s * (th1 - t0)) + _ccw_v(-s * (th2 - th1)) + _ccw_v(s * (t1 - th2))
            l = np.zeros_like(a)
        A.append(np.where(ok, a, INF)); L.append(np.where(ok, l, 0.0))
    return np.array(A), np.array(L)


def dubins_one(A, thA, B, thB, r, w, m):
    """形wの Dubins 経路(プリミティブの列)。だめならNone。thA/thB は動く向き。"""
    kind, s1, s2 = WORDS[w]
    if kind == 'CSC':
        c1 = A + s1 * r * nL(thA); c2 = B + s2 * r * nL(thB)
        D = float(np.hypot(*(c2 - c1))); k = (s1 - s2) * r
        if D <= abs(k) + 1e-9:
            return None
        phi = math.atan2(*(c2 - c1)[::-1]) + math.asin(k / D)
        p1 = c1 - s1 * r * nL(phi); p2 = c2 - s2 * r * nL(phi)
        prims = [arc_prim(A, thA, s1, ccw(s1 * (phi - thA)), r, m), ('line', p1, p2, m),
                 arc_prim(p2, phi, s2, ccw(s2 * (thB - phi)), r, m)]
    else:
        s, sg = s1, s2
        c1 = A + s * r * nL(thA); c3 = B + s * r * nL(thB)
        D = float(np.hypot(*(c3 - c1)))
        if not (1e-9 < D < 4 * r):
            return None
        al = math.atan2(*(c3 - c1)[::-1]); be = math.acos(D / (4 * r))
        c2 = c1 + 2 * r * tdir(al + sg * be)
        p1 = (c1 + c2) / 2; p2 = (c2 + c3) / 2
        v1 = (c1 - p1) / (s * r); th1 = math.atan2(-v1[0], v1[1])
        v2 = (c3 - p2) / (s * r); th2 = math.atan2(-v2[0], v2[1])
        prims = [arc_prim(A, thA, s, ccw(s * (th1 - thA)), r, m),
                 arc_prim(p1, th1, -s, ccw(-s * (th2 - th1)), r, m),
                 arc_prim(p2, th2, s, ccw(s * (thB - th2)), r, m)]
    p, th = arc_end(prims[-1])
    if np.hypot(*(p - B)) > 1e-3 or abs(nd(math.degrees(th - thB))) > 1e-3:
        return None
    return [q for q in prims if prim_len(q) > 1e-6]


# ---------------- 姿勢とゲートの通り方 ----------------
DIRS = (1, -1) if ALLOW_REVERSE else (1,)


def visit_candidates(g):
    out = []
    modes = ['F', 'B', 'FB', 'BF'] if ALLOW_REVERSE else ['F']
    for sg in (1, -1):
        u = g.n * sg
        hu = math.degrees(math.atan2(u[1], u[0])); hm = nd(hu + 180)
        for off in OFFSETS:
            C = g.c + g.t * off
            for mode in modes:
                if mode in ('F', 'FB'):
                    base_a, h, d1, tip = FRONT + CLR, hu, 1, REAR + PM
                else:
                    base_a, h, d1, tip = REAR + CLR, hm, -1, FRONT + PM
                base_b = FRONT + CLR if mode == 'FB' else REAR + CLR
                for ea in EXT:
                    S = C - u * (base_a + ea)
                    for ee in EXT:
                        if mode in ('F', 'B'):
                            E = C + u * (tip + ee); legs = [(S, E, d1)]; dend = d1
                        else:
                            T = C + u * tip; E = C - u * (base_b + ee); legs = [(S, T, d1), (T, E, -d1)]; dend = -d1
                        if not all(line_ok(a, b, d, allow=g.color) for a, b, d in legs):
                            continue
                        t = sum(float(np.hypot(*(b - a))) / V_LINE * spd(d) for a, b, d in legs)
                        rv = sum(float(np.hypot(*(b - a))) for a, b, d in legs if d < 0)
                        if len(legs) == 2:
                            t += T_STOP + PEN_STOP
                        t += PEN_REV_M * rv / 100.0
                        out.append(dict(color=g.color, mode=mode, sign=sg, off=off, ea=ea, ee=ee, legs=legs, cost=t,
                                        start=(S, h, d1), end=(E, h, dend)))
    return out


def pose_key(p):
    return (round(float(p[0][0]), 3), round(float(p[0][1]), 3), round(float(p[1]), 3), p[2])


# ---------------- つなぎ区間の候補(当たりを無視した時間を配列で) ----------------
# 候補 = (族, m, r, 形 or 回る向き)。族: 'run' 1本の走り / 'PLP' 旋回-直進-旋回 / 'PLA' 旋回-直進-円弧 / 'ALP' 円弧-直進-旋回
OPTS = []
for m in DIRS:
    for r in RADII:
        for w in range(8):
            OPTS.append(('run', m, r, w))
if ALLOW_PIVOT:
    for m in DIRS:
        OPTS.append(('PLP', m, 0.0, 0))
        for r in RADII:
            for s1 in (1, -1):
                OPTS.append(('PLA', m, r, s1))
                OPTS.append(('ALP', m, r, s1))


def _tc(deg):
    return np.where(deg < 0.3, 0.0, T_TURN0 + T_TURN_DEG * deg)


def _piv_bad(dd, lmax, rmax):
    """向きの変化dd(符号つき、deg)が、その場で回れる範囲を超えるか(配列)。"""
    a = np.abs(dd)
    ok = (a < 0.3) | ((dd > 0) & (lmax >= dd - 0.5)) | ((dd < 0) & (rmax >= -dd - 0.5)) | \
         ((np.abs(a - 180) < 0.5) & ((lmax >= 180) | (rmax >= 180)))
    return ~ok


def approx_costs(E, S):
    """E, S: 姿勢の配列 (x, y, 車体の向きdeg, 走りの向き 0/±1)。戻り値 (N, len(OPTS))。"""
    ex, ey, eh, ed, el, er = E; sx, sy, sh, sd, sl, sr = S
    N = len(ex)
    out = np.full((N, len(OPTS)), INF)
    dcache = {}
    for j, (fam, m, r, w) in enumerate(OPTS):
        add = 0.0 if m > 0 else 180.0
        stopE_run = (ed != 0) & (ed != m)
        stopS_run = (sd == 0) | (sd != m)
        if fam == 'run':
            if (m, r) not in dcache:
                thE = np.radians(eh + add); thS = np.radians(sh + add)
                tx = sx - TAIL_CM * stopS_run * np.cos(thS); ty = sy - TAIL_CM * stopS_run * np.sin(thS)
                dcache[(m, r)] = dubins_vec(ex, ey, thE, tx, ty, thS, r)
            A, L = dcache[(m, r)]
            a, l = A[w], L[w]
            t = (r * a / v_arc(r) + (l + TAIL_CM * stopS_run) / V_LINE) * spd(m)
            st = stopE_run.astype(float) + stopS_run
            c = t + (T_STOP + PEN_STOP) * st + PEN_ARC_100DEG * np.degrees(a) / 100.0 \
                + (PEN_REV_M * (r * a + l + TAIL_CM * stopS_run) / 100.0 if m < 0 else 0.0)
            out[:, j] = np.where(a >= INF / 2, INF, c)
        elif fam == 'PLP':
            vx, vy = sx - ex, sy - ey
            L = np.hypot(vx, vy)
            hl = np.degrees(np.arctan2(vy, vx)) + add
            pbad = _piv_bad(nd(hl - eh), el, er) | _piv_bad(-nd(sh - hl), sl, sr)
            pe = np.abs(nd(hl - eh)); ps = np.abs(nd(sh - hl))
            stE = (ed != 0) & ~((pe < 0.3) & (ed == m))
            stS = ~((ps < 0.3) & (sd == m))
            c = _tc(pe) + _tc(ps) + L / V_LINE * spd(m) + (T_STOP + PEN_STOP) * (stE.astype(float) + stS) \
                + PEN_PIVOT * ((pe >= 0.3).astype(float) + (ps >= 0.3)) + (PEN_REV_M * L / 100.0 if m < 0 else 0.0)
            out[:, j] = np.where((L < TAIL_CM) | pbad, INF, c)
        elif fam == 'PLA':
            s1 = w
            thB = np.radians(sh + add)
            bx = sx - TAIL_CM * stopS_run * np.cos(thB); by = sy - TAIL_CM * stopS_run * np.sin(thB)
            c2x = bx - s1 * r * np.sin(thB); c2y = by + s1 * r * np.cos(thB)
            D = np.hypot(c2x - ex, c2y - ey)
            ok = D > r + 1e-9
            phi = np.arctan2(c2y - ey, c2x - ex) - np.arcsin(np.clip(s1 * r / np.where(ok, D, 1.0), -1, 1))
            p2x = c2x + s1 * r * np.sin(phi); p2y = c2y - s1 * r * np.cos(phi)
            a2 = _ccw_v(s1 * (thB - phi))
            L = np.hypot(p2x - ex, p2y - ey)
            pbad = _piv_bad(nd(np.degrees(phi) + add - eh), el, er)
            pe = np.abs(nd(np.degrees(phi) + add - eh))
            stE = (ed != 0) & ~((pe < 0.3) & (ed == m))
            c = _tc(pe) + (L / V_LINE + r * a2 / v_arc(r) + TAIL_CM * stopS_run / V_LINE) * spd(m) \
                + (T_STOP + PEN_STOP) * (stE.astype(float) + stopS_run) + PEN_PIVOT * (pe >= 0.3) \
                + PEN_ARC_100DEG * np.degrees(a2) / 100.0 + (PEN_REV_M * (L + r * a2) / 100.0 if m < 0 else 0.0)
            out[:, j] = np.where(ok & ~pbad, c, INF)
        else:  # ALP
            s1 = w
            thA = np.radians(eh + add)
            c1x = ex - s1 * r * np.sin(thA); c1y = ey + s1 * r * np.cos(thA)
            D = np.hypot(sx - c1x, sy - c1y)
            ok = D > r + 1e-9
            phi = np.arctan2(sy - c1y, sx - c1x) + np.arcsin(np.clip(s1 * r / np.where(ok, D, 1.0), -1, 1))
            p1x = c1x + s1 * r * np.sin(phi); p1y = c1y - s1 * r * np.cos(phi)
            a1 = _ccw_v(s1 * (phi - thA))
            L = np.hypot(sx - p1x, sy - p1y)
            pbad = _piv_bad(-nd(sh - (np.degrees(phi) + add)), sl, sr)
            ps = np.abs(nd(sh - (np.degrees(phi) + add)))
            stS = ~((ps < 0.3) & (sd == m))
            c = _tc(ps) + (L / V_LINE + r * a1 / v_arc(r)) * spd(m) \
                + (T_STOP + PEN_STOP) * (stopE_run.astype(float) + stS) + PEN_PIVOT * (ps >= 0.3) \
                + PEN_ARC_100DEG * np.degrees(a1) / 100.0 + (PEN_REV_M * (L + r * a1) / 100.0 if m < 0 else 0.0)
            out[:, j] = np.where(ok & (L >= TAIL_CM) & ~pbad, c, INF)
    return out


def build_opt(opt, ep, sp):
    """候補をプリミティブの列にする。ep/sp = (点, 車体の向きdeg, 走りの向き)。"""
    fam, m, r, w = opt
    E, hE, dE = ep; S, hS, dS = sp
    add = 0.0 if m > 0 else 180.0
    stopS = dS == 0 or dS != m
    if fam == 'run':
        thE = math.radians(hE + add); thS = math.radians(hS + add)
        B = S - TAIL_CM * tdir(thS) if stopS else S
        prims = dubins_one(E, thE, B, thS, r, w, m)
        if prims is None:
            return None
        return prims + ([('line', B, S, m)] if stopS else [])
    if fam == 'PLP':
        hl = math.degrees(math.atan2(*(S - E)[::-1])) + add
        return [('pivot', E, hE, nd(hl)), ('line', E, S, m), ('pivot', S, nd(hl), hS)]
    if fam == 'PLA':
        s1 = w; thB = math.radians(hS + add)
        B = S - TAIL_CM * tdir(thB) if stopS else S
        c2 = B + s1 * r * nL(thB); D = float(np.hypot(*(c2 - E)))
        if D <= r + 1e-9:
            return None
        phi = math.atan2(*(c2 - E)[::-1]) - math.asin(s1 * r / D)
        p2 = c2 - s1 * r * nL(phi)
        pr = [('pivot', E, hE, nd(math.degrees(phi) + add)), ('line', E, p2, m),
              arc_prim(p2, phi, s1, ccw(s1 * (thB - phi)), r, m)]
        return pr + ([('line', B, S, m)] if stopS else [])
    s1 = w; thA = math.radians(hE + add)
    c1 = E + s1 * r * nL(thA); D = float(np.hypot(*(S - c1)))
    if D <= r + 1e-9:
        return None
    phi = math.atan2(*(S - c1)[::-1]) + math.asin(s1 * r / D)
    p1 = c1 - s1 * r * nL(phi)
    return [arc_prim(E, thA, s1, ccw(s1 * (phi - thA)), r, m), ('line', p1, S, m),
            ('pivot', S, nd(math.degrees(phi) + add), hS)]


# ---------------- 中継(1と2が全部当たるときだけ) ----------------
_WPT = None
_WPP = None
_via_from, _via_to = {}, {}


def _wp():
    global _WPT, _WPP
    if _WPT is None:
        pts, poses = [], []
        for x in np.arange(BOUNDS[0], BOUNDS[1] + 0.1, WP_STEP):
            for y in np.arange(BOUNDS[2], BOUNDS[3] + 0.1, WP_STEP):
                if in_keepout(np.array([x]), np.array([y]))[0]:
                    continue
                hs = np.arange(WP_HEADINGS) * 360.0 / WP_HEADINGS
                okh = [h for h in hs if poses_ok(np.array([x]), np.array([y]), np.array([h]))]
                if len(okh) == len(hs):
                    pts.append((x, y))       # どの向きにも回れる点(旋回の中継)
                poses += [(x, y, h) for h in okh]
        _WPT = np.array(pts); _WPP = np.array(poses)
    return _WPT, _WPP


def via_options(ep, sp):
    """中継の候補を、当たりを無視した時間の順に返す: [(cost, ('via', ...))]"""
    E, hE, dE = ep; S, hS, dS = sp
    pts, poses = _wp()
    out = []
    if ALLOW_PIVOT and len(pts):
        X, Y = pts[:, 0], pts[:, 1]
        for m1 in DIRS:
            for m2 in DIRS:
                h1 = np.degrees(np.arctan2(Y - E[1], X - E[0])) + (0 if m1 > 0 else 180)
                h2 = np.degrees(np.arctan2(S[1] - Y, S[0] - X)) + (0 if m2 > 0 else 180)
                L1 = np.hypot(X - E[0], Y - E[1]); L2 = np.hypot(S[0] - X, S[1] - Y)
                pe = np.abs(nd(h1 - hE)); pw = np.abs(nd(h2 - h1)); ps = np.abs(nd(hS - h2))
                c = _tc(pe) + _tc(pw) + _tc(ps) + (L1 * spd(m1) + L2 * spd(m2)) / V_LINE + (T_STOP + PEN_STOP) * 3 \
                    + PEN_PIVOT * 3
                c = np.where((L1 < TAIL_CM) | (L2 < TAIL_CM), INF, c)
                for i in np.argsort(c)[:WP_TRIES]:
                    out.append((float(c[i]), ('viaP', m1, m2, (float(X[i]), float(Y[i])))))
    if len(poses):
        X, Y, H = poses[:, 0], poses[:, 1], poses[:, 2]
        for m1 in DIRS:
            for m2 in DIRS:
                cusp = m1 != m2
                add1 = 0 if m1 > 0 else 180; add2 = 0 if m2 > 0 else 180
                stopE = dE != 0 and dE != m1
                stopS = dS == 0 or dS != m2
                k1 = (round(float(E[0]), 3), round(float(E[1]), 3), round(hE, 3), m1, cusp)
                if k1 not in _via_from:
                    best1 = np.full(len(X), INF)
                    t1 = np.radians(H + add1)
                    tx = X - TAIL_CM * cusp * np.cos(t1); ty = Y - TAIL_CM * cusp * np.sin(t1)
                    for r in RADII:
                        A, L = dubins_vec(np.full(len(X), E[0]), np.full(len(X), E[1]),
                                          np.full(len(X), math.radians(hE + add1)), tx, ty, t1, r)
                        best1 = np.minimum(best1, (r * A / v_arc(r) + L / V_LINE).min(0) * spd(m1))
                    _via_from[k1] = best1
                k2 = (round(float(S[0]), 3), round(float(S[1]), 3), round(hS, 3), m2, stopS)
                if k2 not in _via_to:
                    best2 = np.full(len(X), INF)
                    ts = math.radians(hS + add2)
                    bx = S[0] - TAIL_CM * stopS * math.cos(ts); by = S[1] - TAIL_CM * stopS * math.sin(ts)
                    for r in RADII:
                        A, L = dubins_vec(X, Y, np.radians(H + add2), np.full(len(X), bx), np.full(len(X), by),
                                          np.full(len(X), ts), r)
                        best2 = np.minimum(best2, (r * A / v_arc(r) + L / V_LINE).min(0) * spd(m2))
                    _via_to[k2] = best2
                best1, best2 = _via_from[k1], _via_to[k2]
                c = best1 + best2 + (TAIL_CM * cusp * spd(m1) + TAIL_CM * stopS * spd(m2)) / V_LINE \
                    + (T_STOP + PEN_STOP) * (float(stopE) + cusp + stopS)
                for i in np.argsort(c)[:WP_TRIES]:
                    if c[i] < INF / 2:
                        out.append((float(c[i]), ('viaR', m1, m2, (float(X[i]), float(Y[i]), float(H[i])))))
    out.sort(key=lambda x: x[0])
    return out


_run_cache = {}


def _best_run(A, hA, B, hB, m, tail):
    key = (round(float(A[0]), 3), round(float(A[1]), 3), round(hA, 3), round(float(B[0]), 3), round(float(B[1]), 3),
           round(hB, 3), m, tail)
    if key not in _run_cache:
        _run_cache[key] = _best_run0(A, hA, B, hB, m, tail)
    return _run_cache[key]


def _best_run0(A, hA, B, hB, m, tail):
    """A→B の1本の走り(当たらない最短)。tail>0なら最後を直進tailで終える。"""
    add = 0.0 if m > 0 else 180.0
    thA, thB = math.radians(hA + add), math.radians(hB + add)
    Bm = B - tail * tdir(thB) if tail > 0 else B
    cands = []
    for r in RADII:
        for w in range(8):
            pr = dubins_one(A, thA, Bm, thB, r, w, m)
            if pr is None:
                continue
            if tail > 0:
                pr = pr + [('line', Bm, B, m)]
            cands.append((sum((prim_len(q) / (V_LINE if q[0] == 'line' else v_arc(q[2]))) for q in pr), pr))
    cands.sort(key=lambda c: c[0])
    for _, pr in cands[:12]:
        if all(prim_ok(q) for q in pr):
            return pr
    return None


def build_via(opt, ep, sp):
    E, hE, dE = ep; S, hS, dS = sp
    if opt[0] == 'viaP':
        _, m1, m2, (x, y) = opt
        Wp = np.array([x, y])
        h1 = nd(math.degrees(math.atan2(y - E[1], x - E[0])) + (0 if m1 > 0 else 180))
        h2 = nd(math.degrees(math.atan2(S[1] - y, S[0] - x)) + (0 if m2 > 0 else 180))
        return [('pivot', E, hE, h1), ('line', E, Wp, m1), ('pivot', Wp, h1, h2), ('line', Wp, S, m2), ('pivot', S, h2, hS)]
    _, m1, m2, (x, y, h) = opt
    Wp = np.array([x, y])
    stopS = dS == 0 or dS != m2
    r1 = _best_run(E, hE, Wp, h, m1, TAIL_CM if m1 != m2 else 0.0)
    if r1 is None:
        return None
    r2 = _best_run(Wp, h, S, hS, m2, TAIL_CM if stopS else 0.0)
    if r2 is None:
        return None
    return r1 + r2


# ---------------- 表(つなぎ区間ごとの候補の状態) ----------------
class Table:
    def __init__(self, ends, starts):
        self.ends, self.starts = ends, starts
        nE, nS = len(ends), len(starts)
        Ea = [np.repeat(np.array(v, float), nS) for v in zip(*[(p[0][0], p[0][1], p[1], p[2]) + pivot_interval(p[0], p[1]) for p in ends])]
        Sa = [np.tile(np.array(v, float), nE) for v in zip(*[(p[0][0], p[0][1], p[1], p[2]) + pivot_interval(p[0], p[1]) for p in starts])]
        ac = approx_costs(Ea, Sa).reshape(nE, nS, len(OPTS))
        self.order = np.argsort(ac, axis=2)
        self.approx = np.take_along_axis(ac, self.order, axis=2)
        self.ptr = np.zeros((nE, nS), int)
        self.C = self.approx[:, :, 0].copy()
        self.done = np.zeros((nE, nS), bool)
        self.prims = {}
        self.via = {}
        self.checks = 0
        self.allow_via = False
        self.pending = set()   # 1と2が全部当たり、中継はまだ試していない区間

    def resolve(self, e, s):
        """(e,s) の今の候補を厳密に判定し、当たらない最初の候補に確定する。"""
        if self.done[e, s]:
            return False
        ep, sp = self.ends[e], self.starts[s]
        old = self.C[e, s]
        while self.ptr[e, s] < len(OPTS) and self.approx[e, s, self.ptr[e, s]] < INF / 2:
            opt = OPTS[self.order[e, s, self.ptr[e, s]]]
            pr = build_opt(opt, ep, sp)
            self.checks += 1
            if pr is not None and all(prim_ok(q) for q in pr):
                c = seq_cost(ep[2], pr, sp[2])
                if c < INF / 2:
                    self.C[e, s] = c; self.done[e, s] = True; self.prims[(e, s)] = pr
                    return abs(c - old) > 1e-9
            self.ptr[e, s] += 1
        # 中継(1段目は使わない。経路が見つからないときだけ2段目で使う)
        if not self.allow_via:
            self.C[e, s] = INF; self.done[e, s] = True; self.pending.add((e, s))
            return True
        if (e, s) not in self.via:
            self.via[(e, s)] = via_options(ep, sp)
        for c0, opt in self.via[(e, s)][:VIA_MAX_TRIES]:
            pr = build_via(opt, ep, sp)
            self.checks += 1
            if pr is not None and all(prim_ok(q) for q in pr):
                c = seq_cost(ep[2], pr, sp[2])
                if c < INF / 2:
                    self.C[e, s] = c; self.done[e, s] = True; self.prims[(e, s)] = pr
                    return True
        self.C[e, s] = INF; self.done[e, s] = True
        return True


def main():
    t0 = time.time()
    cands = {c: visit_candidates(g) for c, g in GATES.items()}
    print('candidates', {c: len(v) for c, v in cands.items()}, f'{time.time() - t0:.1f}s', flush=True)
    for c in cands:
        if not cands[c]:
            print(f'NO ROUTE: no way through {c}'); return
    ends, starts, eidx, sidx = {}, {}, {}, {}
    for c in cands:
        ends[c], starts[c], eidx[c], sidx[c] = [], [], {}, {}
        for v in cands[c]:
            for pose, lst, idx, key in ((v['start'], starts[c], sidx[c], 's'), (v['end'], ends[c], eidx[c], 'e')):
                k = pose_key(pose)
                if k not in idx:
                    idx[k] = len(lst); lst.append(pose)
                v[key] = idx[k]
    start_pose = [(START, START_H, 0)]
    goal_pose = [(GOAL, GOAL_H, 0)]
    T = {('START', 'red'): Table(start_pose, starts['red']),
         ('red', 'blue'): Table(ends['red'], starts['blue']),
         ('blue', 'yellow'): Table(ends['blue'], starts['yellow']),
         ('yellow', 'red'): Table(ends['yellow'], starts['red']),
         ('yellow', 'GOAL'): Table(ends['yellow'], goal_pose)}
    print('tables', {k: v.C.shape for k, v in T.items()}, f'{time.time() - t0:.1f}s', flush=True)
    seq = ['red', 'blue', 'yellow'] * LAPS
    vs = {c: (np.array([v['s'] for v in cands[c]]), np.array([v['e'] for v in cands[c]]),
              np.array([v['cost'] for v in cands[c]])) for c in cands}
    it = 0
    phase = 1
    while True:
        it += 1
        prev = np.zeros(1); prev_c = 'START'; back = []
        for color in seq:
            tab = T[(prev_c, color)]
            M = prev[:, None] + tab.C
            bs = M.min(0); be = M.argmin(0)
            s_arr, e_arr, c_arr = vs[color]
            tot = bs[s_arr] + c_arr
            cur = np.full(len(ends[color]), INF); arg = np.full(len(ends[color]), -1)
            for i in np.argsort(tot)[::-1]:
                cur[e_arr[i]] = tot[i]; arg[e_arr[i]] = i
            back.append((prev_c, color, be, arg))
            prev, prev_c = cur, color
        tab = T[(prev_c, 'GOAL')]
        M = prev + tab.C[:, 0]
        e_last = int(np.argmin(M)); total = float(M[e_last])
        if total >= INF / 2:
            bad = next((f'{a}->{b}' for a, b, _, arg in back if np.all(arg < 0)), 'goal')
            if phase == 1 and any(t.pending for t in T.values()):
                # 2段目: 中継を使えば通れるかもしれない区間を、当たりを無視した時間に戻して解き直す
                phase = 2
                for t in T.values():
                    t.allow_via = True
                    for (pe, ps) in t.pending:
                        t.C[pe, ps] = t.approx[pe, ps, 0]; t.done[pe, ps] = False
                    t.pending = set()
                print(f'phase 2 (detours)  {time.time() - t0:.1f}s', flush=True)
                continue
            print(f'NO ROUTE (stopped at {bad})  checks={sum(t.checks for t in T.values())} {time.time() - t0:.1f}s')
            return
        # 経路を復元して、使ったつなぎ区間を厳密に判定する
        used = [(('yellow', 'GOAL'), e_last, 0)]
        visits = []
        e = e_last
        for prev_c, color, be, arg in reversed(back):
            vi = arg[e]; v = cands[color][vi]
            visits.append(v)
            s = v['s']; ep = int(be[s])
            used.append(((prev_c, color), ep, s))
            e = ep
        used.reverse(); visits.reverse()
        changed = False
        for key, e, s in used:
            changed |= T[key].resolve(e, s)
        if not changed:
            break
    print(f'TOTAL hybrid laps={LAPS}: {total:.1f}s (cost)  iterations={it} checks={sum(t.checks for t in T.values())} '
          f'compute {time.time() - t0:.1f}s', flush=True)

    def ser(p):
        if p[0] == 'line':
            return dict(t='line', a=p[1].tolist(), b=p[2].tolist(), m=p[3])
        if p[0] == 'arc':
            xs, ys, _ = arc_samples(p, 2.0)
            return dict(t='arc', pts=[[float(a), float(b)] for a, b in zip(xs, ys)], m=p[6], r=p[2],
                        deg=math.degrees(p[5]), th0=math.degrees(p[3]), s=p[4])
        return dict(t='pivot', p=[float(p[1][0]), float(p[1][1])], deg=abs(nd(p[3] - p[2])), h_from=p[2], h_to=p[3])

    blocks = []
    for i, (key, e, s) in enumerate(used):
        blocks.append(dict(kind='conn', lap=None, color=None, prims=[ser(p) for p in T[key].prims[(e, s)]]))
        if i < len(visits):
            v = visits[i]
            blocks.append(dict(kind='visit', lap=i // 3 + 1, color=v['color'], mode=v['mode'], sign=v['sign'], off=v['off'],
                               prims=[ser(('line', a, b, d)) for a, b, d in v['legs']]))
    for i, v in enumerate(visits[:3]):
        print(f'  lap1 {v["color"]:6s} {v["mode"]:2s} sign={v["sign"]:+d} off={v["off"]:+.1f} ea={v["ea"]} ee={v["ee"]}')
    json.dump(dict(total=total, params=PARAMS, blocks=blocks), open(sys.argv[2], 'w'))


if __name__ == '__main__':
    main()
