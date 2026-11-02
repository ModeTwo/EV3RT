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
    front=7.5, rear=17.5, half_width=6.85, margin_front=1.5, margin_rear=1.5,   # 11/02: 前後とも1.5cm(以前は後ろ0.4cm)
    # 経路の形
    radii=[20.0, 15.0, 12.0, 10.0], tail_cm=4.0,
    ext=[0.0, 4.0, 8.0, 14.0, 22.0, 35.0], offsets=[0.0], angles=[0.0],   # 11/02: 横のずらしは使わない(反対側の支柱に近づくため)
    # 斜めの通過: 'split' = 真ん中で向きを変える(斜めに入って垂直に抜ける/垂直に入って斜めに抜ける)、
    # 'straight' = 入りから抜けまで同じ斜め、'both' = 両方。split の向きを変える円弧の半径(cm)は split_radii。
    oblique_mode='split', split_radii=[20.0, 15.0, 10.0],
    # 斜めの使い方: 'safest' = そのゲートを(その側から・その通り方で)真ん中を垂直に通れないときだけ、
    # 通過の間の一番近い柱までの距離が最大になる角度と円弧の半径を1つ選ぶ(angle_scan の範囲)。'all' = angles をどこでも使う。
    oblique_policy='safest', angle_scan=[-30.0, 30.0, 1.0],
    # True: 入りも抜けも斜めの通過も候補にする(入りの側にも抜けの側にもとなりのT字があるゲートのため。11/02 seed758743)
    oblique_both=True,
    # スタート: 'default' = 今のスタート位置(向き0)、'goal' = 今のゴール地点(向き180、フィールドの方を向く)
    start_at='default',
    # 進入禁止: True = 車体の箱のどこも入らない(スタートで置いた時点でかかっている長方形は、スタートの車軸から start_exempt_cm 以内だけ外す)
    keepout_body=True, start_exempt_cm=20.0,
    # 進入禁止の長方形(右コースのplanner座標cm、[xmin, xmax, ymin, ymax]、±1e9で無限)。None なら plannerの設定(左コース)を左右反転した今の3つ。
    # 暫定(ユーザー11/02: 測り直す予定、今より狭くなる)。11/02からは、測り直すまで判定しない([])。今の3つを使うときは 'planner'。
    keepout=[],
    allow_reverse=True, allow_pivot=True,
    # 時間: 直進 cm/s、円弧は arc_k*v_line/(1+half_tread/r)、後退は rev_speed_scale 倍の速さ
    v_line=36.0, arc_k=0.77, half_tread=5.85, rev_speed_scale=1.0,
    t_stop=0.67, t_turn0=0.7, t_turn_deg=0.012,
    # 精度の重み(秒に換算して時間に足す。0なら時間だけで選ぶ)
    pen_stop=0.0, pen_pivot=0.0, pen_rev_m=0.0, pen_arc_100deg=0.0,
    # 中継
    wp_step=8.0, wp_headings=12, wp_tries=60, via_max_tries=40,
    # True: 遠回りなしで経路が見つかっても、遠回りで速くなる区間がないか探す(最速の取りこぼしを防ぐ)
    via_always=False, via_budget_sec=8.0,
    # その場の旋回の中心が車軸からどれだけ後ろか(cm)。10/31の左右同期の旋回で約0.5cm(測り直し予定)
    pivot_back_cm=0.5,   # 2段目(遠回りで速くなるか)の時間の上限。超えたら1段目の経路を使う
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
if START_AT == 'goal':
    START = GOAL.copy(); START_H = 180.0
_mx = lambda x: 4 * P - x
KEEPOUT = [(_mx(INF), _mx(-17.0), -INF, -37.0),
           (_mx(INF), _mx(4 * P + 17.8 + 14.0), -INF, INF),
           (_mx(-28.0), INF, 0.0, INF)]
if PARAMS['keepout'] != 'planner':
    KEEPOUT = [tuple(float(v) for v in r) for r in PARAMS['keepout']]
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


_BODY_REACH = math.hypot(max(FRONT, REAR), HALF_WIDTH) + POST_R + max(MARGIN_FRONT, MARGIN_REAR) + 0.5


def poses_clear(px, py, h_deg):
    """姿勢ごとに、前の部分と後ろの部分のすき間が足りているか(bool配列)。"""
    # 車体の近く(車軸から車体の一番遠い角+支柱+すき間)にある支柱の点だけで判定する
    rr = _BODY_REACH
    sel = CAP[(CAP[:, 0] > px.min() - rr) & (CAP[:, 0] < px.max() + rr) & (CAP[:, 1] > py.min() - rr) & (CAP[:, 1] < py.max() + rr)]
    if len(sel) == 0:
        return np.ones(len(px), bool)
    h = np.radians(h_deg)
    c = np.cos(h)[:, None]; s = np.sin(h)[:, None]
    dx = sel[None, :, 0] - px[:, None]; dy = sel[None, :, 1] - py[:, None]
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


def _pivot_axle(p, h_from, hs):
    """その場の旋回で、回る中心が車軸の PIVOT_BACK_CM 後ろにあるときの、向き hs での車軸の位置。"""
    hs = np.asarray(hs, float)
    if PIVOT_BACK_CM == 0:
        return np.full(len(hs), p[0]), np.full(len(hs), p[1])
    qx = p[0] - PIVOT_BACK_CM * math.cos(math.radians(h_from)); qy = p[1] - PIVOT_BACK_CM * math.sin(math.radians(h_from))
    return qx + PIVOT_BACK_CM * np.cos(np.radians(hs)), qy + PIVOT_BACK_CM * np.sin(np.radians(hs))


def pivot_interval(p, h):
    """点pで向きhから、その場で回れる最大の角度(左回り, 右回り)。"""
    key = (round(float(p[0]), 3), round(float(p[1]), 3), round(float(h), 3))
    if key not in _piv_int:
        ks = np.arange(1, 182)
        hs = np.concatenate([h + ks, h - ks])
        ax, ay = _pivot_axle(p, h, hs)
        ok = poses_clear(ax, ay, hs)
        res = []
        for part in (ok[:181], ok[181:]):
            bad = np.flatnonzero(~part)
            res.append(181 if len(bad) == 0 else int(bad[0]))
        _piv_int[key] = tuple(res)
    return _piv_int[key]


_KO_CORNERS = None
_START_EXEMPT = None
_EXEMPT_ON = False   # スタートから最初のゲートへのつなぎ区間を判定している間だけ True


def body_keepout(px, py, hdeg):
    """姿勢(配列)のどれかで、車体の箱が進入禁止の長方形にかかるか。"""
    global _KO_CORNERS, _START_EXEMPT
    if not KEEPOUT:
        return False
    if not KEEPOUT_BODY:
        return bool(np.any(in_keepout(px, py)))
    px = np.asarray(px, float); py = np.asarray(py, float); h = np.radians(np.asarray(hdeg, float))
    if _KO_CORNERS is None:
        _KO_CORNERS = [[(x, y) for x in (x0, x1) for y in (y0, y1) if abs(x) < INF / 2 and abs(y) < INF / 2]
                       for x0, x1, y0, y1 in KEEPOUT]
    xs = np.concatenate([np.linspace(-REAR, FRONT, 14), np.full(8, FRONT), np.linspace(FRONT, -REAR, 14), np.full(8, -REAR)])
    ys = np.concatenate([np.full(14, HALF_WIDTH), np.linspace(HALF_WIDTH, -HALF_WIDTH, 8), np.full(14, -HALF_WIDTH),
                         np.linspace(-HALF_WIDTH, HALF_WIDTH, 8)])
    c = np.cos(h)[:, None]; sn = np.sin(h)[:, None]
    bx = px[:, None] + xs * c - ys * sn; by = py[:, None] + xs * sn + ys * c
    if _START_EXEMPT is None:
        sb = [(START[0] + a * math.cos(math.radians(START_H)) - b * math.sin(math.radians(START_H)),
               START[1] + a * math.sin(math.radians(START_H)) + b * math.cos(math.radians(START_H)))
              for a, b in zip(xs, ys)]
        _START_EXEMPT = {k for k, (x0, x1, y0, y1) in enumerate(KEEPOUT) if any(x0 < x < x1 and y0 < y < y1 for x, y in sb)}
    near_start = np.hypot(px - START[0], py - START[1]) <= START_EXEMPT_CM
    for k, (x0, x1, y0, y1) in enumerate(KEEPOUT):
        hit = np.any((bx > x0) & (bx < x1) & (by > y0) & (by < y1), axis=1)
        for cx, cy in _KO_CORNERS[k]:   # 長方形の角が車体の中に入る場合
            lx = (cx - px) * np.cos(h) + (cy - py) * np.sin(h); ly = -(cx - px) * np.sin(h) + (cy - py) * np.cos(h)
            hit |= (lx > -REAR) & (lx < FRONT) & (np.abs(ly) < HALF_WIDTH)
        if k in _START_EXEMPT and _EXEMPT_ON:
            hit &= ~near_start
        if np.any(hit):
            return True
    return False


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
    nb = max(2, int(L) + 1)
    if body_keepout(np.linspace(a[0], b[0], nb), np.linspace(a[1], b[1], nb), np.full(nb, math.degrees(h))):
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
    n = max(2, int(r * sweep / step) + 1, int(math.degrees(sweep)) + 1)   # 向き1°ごとにも(後ろの角は車軸より大きく振れる)
    th = theta + s * sweep * np.arange(n + 1) / n
    xs = c[0] + s * r * np.sin(th); ys = c[1] - s * r * np.cos(th)
    hb = np.degrees(th) + (0 if m > 0 else 180)
    return xs, ys, hb


_arc_memo = {}   # (始点, 向き, 半径, 回る向き, 前後, allow) -> [当たらないと確かめた角度, 当たる最初の角度]


def arc_ok(pr, allow=None):
    """円弧が当たらないか。同じ始点・半径・回る向きの円弧は、どこまで当たらないかを覚えて使い回す。"""
    if pr[5] < 1e-6:
        return True
    _, c, r, theta, sgn, sweep, m, p0 = pr
    key = (round(float(p0[0]), 3), round(float(p0[1]), 3), round(float(theta), 6), r, sgn, m, allow)
    memo = _arc_memo.get(key)
    if memo is not None:
        if memo[1] is not None and sweep >= memo[1] - 1e-9:
            return False
        if sweep <= memo[0] + 1e-9:
            return True
    ok, fail_at = _arc_check(pr, allow)
    if memo is None:
        memo = _arc_memo[key] = [0.0, None]
    if ok:
        memo[0] = max(memo[0], sweep)
    else:
        memo[1] = fail_at if memo[1] is None else min(memo[1], fail_at)
    return ok


def _arc_check(pr, allow=None):
    """(当たらないか, 当たる最初の角度rad)"""
    _, c, r, theta, sgn, sweep, m, p0 = pr
    xs, ys, hb = arc_samples(pr)
    n = len(xs) - 1
    okp = poses_clear(xs, ys, hb)
    if KEEPOUT:
        okp &= ~np.array([body_keepout(xs[i:i + 1], ys[i:i + 1], hb[i:i + 1]) for i in range(len(xs))])             if not np.all(okp) or body_keepout(xs, ys, hb) else okp
    bad = np.flatnonzero(~okp)
    if len(bad):
        return False, sweep * max(bad[0] - 1, 0) / n
    if KEEPOUT and np.any(in_keepout(xs, ys)):
        return False, 0.0
    if crosses_gate(xs, ys, allow):
        return False, 0.0
    return True, None


def _arc_ok_old(pr, allow=None):
    if pr[5] < 1e-6:
        return True
    xs, ys, hb = arc_samples(pr)
    if not poses_ok(xs, ys, hb):
        return False
    if np.any(in_keepout(xs, ys)) or body_keepout(xs, ys, hb):
        return False
    return not crosses_gate(xs, ys, allow)


_pivot_cache = {}


def pivot_ok(p, h_from, h_to):
    dd = nd(h_to - h_from)
    if abs(dd) < 0.3:
        return True
    key = (round(float(p[0]), 2), round(float(p[1]), 2), round(h_from, 2), round(h_to, 2), _EXEMPT_ON)
    if key not in _pivot_cache:
        hs = h_from + dd * np.linspace(0, 1, int(abs(dd)) + 2)
        ax, ay = _pivot_axle(p, h_from, hs)
        _pivot_cache[key] = poses_ok(ax, ay, hs) and not body_keepout(ax, ay, hs)
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


def _rot(v, deg):
    c, sn = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([v[0] * c - v[1] * sn, v[0] * sn + v[1] * c])


def _pass_prims(g, C, base_u, a_in, a_out, lead, trail, m, ea, ee, r):
    """ゲートの通過のプリミティブ。入りは向き a_in の直進で車軸が C(ゲートの線上)に着くまで、
    a_in != a_out なら C から半径 r の円弧で a_out に向きを変え、a_out の直進で抜ける。
    入りの条件: 先の端の角が線の CLR 手前、抜けの条件: 後の端の角が線を PM 越える(線に垂直な距離)。"""
    u_in, u_out = _rot(base_u, a_in), _rot(base_u, a_out)
    ci, si = math.cos(math.radians(a_in)), abs(math.sin(math.radians(a_in)))
    co, so = math.cos(math.radians(a_out)), abs(math.sin(math.radians(a_out)))
    S = C - u_in * ((lead * ci + HALF_WIDTH * si + CLR) / ci + ea)
    need = trail * co + HALF_WIDTH * so + PM
    prims = [('line', S, C, m)]
    P2 = C
    if abs(a_out - a_in) > 1e-9:
        th_in = math.atan2(u_in[1], u_in[0]); th_out = math.atan2(u_out[1], u_out[0])
        dd = nd(math.degrees(th_out - th_in))
        sgn = 1 if dd > 0 else -1
        arc = arc_prim(C, th_in, sgn, math.radians(abs(dd)), r, m)
        P2, _ = arc_end(arc)
        prims.append(arc)
    d2 = float(np.dot(P2 - C, base_u))
    L = (need - d2) / co
    if L < 0:
        return None
    E = P2 + u_out * (L + ee)
    prims.append(('line', P2, E, m))
    return prims


def _visit_candidates(g, pairs_override=None, only=None):
    """ゲートの通り方の候補。斜め(angles が0以外)は、車軸の中心がゲートの真ん中を通るときだけ(横のずれは0)。"""
    out = []
    modes = ['F', 'B', 'FB', 'BF'] if ALLOW_REVERSE else ['F']
    pairs = [(0.0, 0.0)]
    for a in ([] if pairs_override is not None else ANGLES):
        if abs(a) < 1e-9:
            continue
        if OBLIQUE_MODE in ('split', 'both'):
            pairs += [(a, 0.0), (0.0, a)]
        if OBLIQUE_MODE in ('straight', 'both'):
            pairs.append((a, a))
    for sg in (1, -1):
        base_u = g.n * sg
        for a_in, a_out, *rr in (pairs_override if pairs_override is not None else pairs):
            oblique = abs(a_in) > 1e-9 or abs(a_out) > 1e-9
            radii = (rr if rr else SPLIT_RADII) if abs(a_in - a_out) > 1e-9 else [0.0]
            for off in (OFFSETS if not oblique else [0.0]):
                C = g.c + g.t * off
                for mode in modes:
                    if only is not None and (sg, mode) not in only:
                        continue
                    if mode in ('F', 'FB'):
                        lead, trail, m = FRONT, REAR, 1
                    else:
                        lead, trail, m = REAR, FRONT, -1
                    for r in radii:
                        for ea in EXT:
                            for ee in EXT:
                                if mode in ('F', 'B'):
                                    prims = _pass_prims(g, C, base_u, a_in, a_out, lead, trail, m, ea, ee, r)
                                    if prims is None:
                                        continue
                                elif oblique:
                                    # 斜めの往復: 斜めの通過(抜けの余りなし)で抜けきったら止まり、同じ道を後退で戻って、
                                    # 入りの点からさらに ee だけ戻る(戻りも行きと同じ道なので、柱とのすき間も同じ)
                                    fwd = _pass_prims(g, C, base_u, a_in, a_out, lead, trail, m, ea, 0.0, r)
                                    if fwd is None:
                                        continue
                                    back = [_retrace(q) for q in reversed(fwd)]
                                    S0 = fwd[0][1]; u_in = _rot(base_u, a_in)
                                    prims = fwd + back + ([('line', S0, S0 - u_in * ee, -m)] if ee > 0 else [])
                                else:
                                    # 往復(垂直だけ): 抜けきったら戻って、元の側へ抜けきる
                                    u = base_u
                                    S = C - u * (lead + CLR + ea); T = C + u * (trail + PM)
                                    E = C - u * ((FRONT if mode == 'FB' else REAR) + CLR + ee)
                                    prims = [('line', S, T, m), ('line', T, E, -m)]
                                ok = True
                                for q in prims:
                                    if q[0] == 'line' and not line_ok(q[1], q[2], q[3], allow=g.color):
                                        ok = False; break
                                    if q[0] == 'arc' and not arc_ok(q, allow=g.color):
                                        ok = False; break
                                if not ok:
                                    continue
                                t = sum((prim_len(q) / (V_LINE if q[0] == 'line' else v_arc(q[2]))) * spd(prim_m(q)) for q in prims)
                                rv = sum(prim_len(q) for q in prims if prim_m(q) < 0)
                                if mode in ('FB', 'BF'):
                                    t += T_STOP + PEN_STOP
                                t += PEN_REV_M * rv / 100.0
                                S = prims[0][1]
                                last = prims[-1]
                                E = last[2]
                                hS = nd(math.degrees(math.atan2(*(prims[0][2] - prims[0][1])[::-1])) + (0 if prims[0][3] > 0 else 180))
                                hE = nd(math.degrees(math.atan2(*(last[2] - last[1])[::-1])) + (0 if last[3] > 0 else 180))
                                out.append(dict(color=g.color, mode=mode, sign=sg, off=off, angle=a_in, angle_out=a_out, ea=ea, ee=ee,
                                                prims=prims, legs=[(q[1], q[2], q[3]) for q in prims if q[0] == 'line'], cost=t,
                                                start=(S, hS, prims[0][3]), end=(E, hE, last[3])))
    return out


def pass_clearance(prims):
    """通過の間(プリミティブ列)の、車体と支柱の一番小さいすき間(前と後ろの小さい方、cm)。"""
    xs, ys, hs = [], [], []
    for q in prims:
        if q[0] == 'line':
            L = prim_len(q)
            k = np.linspace(0, 1, max(2, int(L / 0.5) + 1))
            h = math.degrees(math.atan2(*(q[2] - q[1])[::-1])) + (0 if q[3] > 0 else 180)
            xs.append(q[1][0] + (q[2][0] - q[1][0]) * k); ys.append(q[1][1] + (q[2][1] - q[1][1]) * k)
            hs.append(np.full(len(k), h))
        elif q[0] == 'arc':
            ax, ay, ah = arc_samples(q, 0.5)
            xs.append(ax); ys.append(ay); hs.append(ah)
    px, py, hh = np.concatenate(xs), np.concatenate(ys), np.radians(np.concatenate(hs))
    c = np.cos(hh)[:, None]; sn = np.sin(hh)[:, None]
    dx = CAP[None, :, 0] - px[:, None]; dy = CAP[None, :, 1] - py[:, None]
    lx = dx * c + dy * sn; ly = -dx * sn + dy * c
    ey = np.maximum(np.abs(ly) - HALF_WIDTH, 0.0)
    cf = np.hypot(np.maximum(np.maximum(-lx, lx - FRONT), 0.0), ey) - POST_R
    cr = np.hypot(np.maximum(np.maximum(-REAR - lx, lx), 0.0), ey) - POST_R
    return float(min(cf.min() - MARGIN_FRONT, cr.min() - MARGIN_REAR))


OBLIQUE_CHOICE = []   # 記録: (色, 側, 通り方, 入り, 抜け, 半径, すき間)


def _retrace(q):
    """プリミティブを逆向きにたどる(後退と前進を入れ替える)。"""
    if q[0] == 'line':
        return ('line', q[2], q[1], -q[3])
    _, c, r, theta, sgn, sweep, m, p0 = q
    p_end, th_end = arc_end(q)
    return arc_prim(p_end, th_end + math.pi, -sgn, sweep, r, -m)


def visit_candidates(g):
    if OBLIQUE_POLICY != 'safest':
        return _visit_candidates(g)
    perp = _visit_candidates(g, pairs_override=[(0.0, 0.0)])
    have = {(v['sign'], v['mode']) for v in perp}
    modes = ['F', 'B'] if ALLOW_REVERSE else ['F']
    modes = ['F', 'B', 'FB', 'BF'] if ALLOW_REVERSE else ['F']
    missing = [(sg, mo) for sg in (1, -1) for mo in modes if (sg, mo) not in have]
    out = list(perp)
    lo, hi, step = ANGLE_SCAN
    angles = [a for a in np.arange(lo, hi + 1e-9, step) if abs(a) > 1e-9]
    for sg, mo in missing:
        base_u = g.n * sg
        lead, trail, m = (FRONT, REAR, 1) if mo in ('F', 'FB') else (REAR, FRONT, -1)
        # 入りを斜め / 抜けを斜め の両方から、通過の間の一番近い柱までの距離が最大のものを1つだけ選ぶ
        best = None
        # 候補: 入りだけ斜め / 抜けだけ斜め / (OBLIQUE_BOTH なら)入りも抜けも斜め(同じ角度の1本の線と、4°刻みの違う角度)
        trials = [(a, 0.0, r) for a in angles for r in SPLIT_RADII] + [(0.0, a, r) for a in angles for r in SPLIT_RADII]
        if OBLIQUE_BOTH:
            trials += [(a, a, 0.0) for a in angles]
            coarse = [a for a in angles if abs(round(a) % 4) < 1e-9]
            trials += [(a1, a2, 20.0) for a1 in coarse for a2 in coarse if abs(a1 - a2) > 1e-9]
        for a_in, a_out, r in trials:
            prims = _pass_prims(g, g.c, base_u, a_in, a_out, lead, trail, m, 0.0, 0.0, r)
            if prims is None:
                continue
            cl = pass_clearance(prims)
            # すき間が1mm以内で同じなら、斜めが片側だけの方、次に角度が小さい方を選ぶ
            pref = (int(abs(a_in) > 1e-9) + int(abs(a_out) > 1e-9), max(abs(a_in), abs(a_out)))
            if best is None or cl > best[0] + 0.1 or (cl > best[0] - 0.1 and pref < best[4]):
                best = (cl, a_in, a_out, r, pref)
        if best is None or best[0] < 0:
            continue
        OBLIQUE_CHOICE.append((g.color, sg, mo, float(best[1]), float(best[2]), best[3], round(best[0] + MARGIN_FRONT, 2)))
        out += _visit_candidates(g, pairs_override=[(best[1], best[2], best[3])], only={(sg, mo)})
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
                if okh:
                    pts.append((x, y))       # 旋回の中継の候補(必要な角度だけ回れるかは、組み立てたあとに判定する)
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
           round(hB, 3), m, tail, _EXEMPT_ON)
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
    def __init__(self, ends, starts, is_start=False, via_first=False):
        self.ends, self.starts = ends, starts
        self.is_start = is_start
        self.via_first = via_first   # スタート・ゴールの区間: 組み合わせが少ないので、最初から遠回りも比べる
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
        self.allow_via = via_first
        self.pending = set()   # 1と2が全部当たり、中継はまだ試していない区間

    def _best_via(self, e, s, ep, sp, limit):
        """遠回りの候補のうち、limit より速く当たらない最初のもの。(cost, prims) か (INF, None)。"""
        if (e, s) not in self.via:
            self.via[(e, s)] = via_options(ep, sp)
        for c0, opt in self.via[(e, s)][:VIA_MAX_TRIES]:
            if c0 >= limit:
                break
            pr = build_via(opt, ep, sp)
            self.checks += 1
            if pr is not None and all(prim_ok(q) for q in pr):
                c = seq_cost(ep[2], pr, sp[2])
                if c < limit:
                    return c, pr
        return INF, None

    def open_via(self):
        """2段目: 直接つなげなかった区間に、遠回りの見込みの時間(当たりを無視)を入れて、解き直せるようにする。"""
        global _EXEMPT_ON
        self.allow_via = True
        for (pe, ps) in self.pending:
            _EXEMPT_ON = self.is_start
            try:
                if (pe, ps) not in self.via:
                    self.via[(pe, ps)] = via_options(self.ends[pe], self.starts[ps])
            finally:
                _EXEMPT_ON = False
            v = self.via[(pe, ps)]
            self.C[pe, ps] = v[0][0] if v else INF
            self.done[pe, ps] = not v
        self.pending = set()

    def resolve(self, e, s):
        """(e,s) の今の候補を厳密に判定し、当たらない最初の候補に確定する。"""
        global _EXEMPT_ON
        if self.done[e, s]:
            return False
        _EXEMPT_ON = self.is_start
        try:
            return self._resolve(e, s)
        finally:
            _EXEMPT_ON = False

    def _resolve(self, e, s):
        ep, sp = self.ends[e], self.starts[s]
        old = self.C[e, s]
        while self.ptr[e, s] < len(OPTS) and self.approx[e, s, self.ptr[e, s]] < INF / 2:
            opt = OPTS[self.order[e, s, self.ptr[e, s]]]
            pr = build_opt(opt, ep, sp)
            self.checks += 1
            if pr is not None and all(prim_ok(q) for q in pr):
                c = seq_cost(ep[2], pr, sp[2])
                if c < INF / 2:
                    if self.via_first:
                        cv, pv = self._best_via(e, s, ep, sp, c)
                        if pv is not None:
                            c, pr = cv, pv
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


def route_audit(conn_prims, visits, used):
    """できた経路全体を細かく(直進0.5cm・円弧と旋回1°)調べ、柱までの一番小さいすき間と、進入禁止へのはみ出しを出す。"""
    allp = []
    for i, (key, e, s) in enumerate(used):
        allp.append(('conn', key))
        if i < len(visits):
            allp.append(('visit', visits[i]))
    xs, ys, hs = [], [], []
    def add_prims(prims):
        for q in prims:
            if q[0] == 'line':
                L = prim_len(q); k = np.linspace(0, 1, max(2, int(L / 0.5) + 1))
                h = math.degrees(math.atan2(*(q[2] - q[1])[::-1])) + (0 if q[3] > 0 else 180)
                xs.append(q[1][0] + (q[2][0] - q[1][0]) * k); ys.append(q[1][1] + (q[2][1] - q[1][1]) * k); hs.append(np.full(len(k), h))
            elif q[0] == 'arc':
                ax, ay, ah = arc_samples(q, 0.5); xs.append(ax); ys.append(ay); hs.append(ah)
            else:
                dd = nd(q[3] - q[2])
                if abs(dd) >= 0.3:
                    hh = q[2] + dd * np.linspace(0, 1, int(abs(dd)) + 2)
                    ax, ay = _pivot_axle(q[1], q[2], hh); xs.append(ax); ys.append(ay); hs.append(hh)
    for i, (key, e, s) in enumerate(used):
        add_prims(_USED_PRIMS[(key, e, s)])
        if i < len(visits):
            add_prims(visits[i]['prims'])
    px, py, hh = np.concatenate(xs), np.concatenate(ys), np.concatenate(hs)
    h = np.radians(hh); c = np.cos(h)[:, None]; sn = np.sin(h)[:, None]
    dx = CAP[None, :, 0] - px[:, None]; dy = CAP[None, :, 1] - py[:, None]
    lx = dx * c + dy * sn; ly = -dx * sn + dy * c
    ey = np.maximum(np.abs(ly) - HALF_WIDTH, 0.0)
    cf = (np.hypot(np.maximum(np.maximum(-lx, lx - FRONT), 0.0), ey) - POST_R).min(1)
    cr = (np.hypot(np.maximum(np.maximum(-REAR - lx, lx), 0.0), ey) - POST_R).min(1)
    ko = sum(1 for i in range(0, len(px), 4) if body_keepout(px[i:i + 1], py[i:i + 1], hh[i:i + 1]))
    i = int(np.argmin(np.minimum(cf, cr)))
    return dict(min_front=float(cf.min()), min_rear=float(cr.min()), keepout_hits=ko,
                worst_pose=[float(px[i]), float(py[i]), float(hh[i])])


_USED_PRIMS = {}


def main():
    t0 = time.time()
    cands = {c: visit_candidates(g) for c, g in GATES.items()}
    print('candidates', {c: len(v) for c, v in cands.items()}, f'{time.time() - t0:.1f}s', flush=True)
    for oc in OBLIQUE_CHOICE:
        print('  oblique', oc, flush=True)
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
    T = {('START', 'red'): Table(start_pose, starts['red'], is_start=True, via_first=True),
         ('red', 'blue'): Table(ends['red'], starts['blue']),
         ('blue', 'yellow'): Table(ends['blue'], starts['yellow']),
         ('yellow', 'red'): Table(ends['yellow'], starts['red']),
         ('yellow', 'GOAL'): Table(ends['yellow'], goal_pose, via_first=True)}
    print('tables', {k: v.C.shape for k, v in T.items()}, f'{time.time() - t0:.1f}s', flush=True)
    seq = ['red', 'blue', 'yellow'] * LAPS
    vs = {c: (np.array([v['s'] for v in cands[c]]), np.array([v['e'] for v in cands[c]]),
              np.array([v['cost'] for v in cands[c]])) for c in cands}
    it = 0
    phase = 1
    saved = None; t_phase2 = None
    while True:
        it += 1
        if saved is not None and time.time() - t_phase2 > VIA_BUDGET_SEC:
            total, used, visits, prims = saved
            for k in T:
                T[k].prims = prims[k]
            print(f'phase 2 over budget ({VIA_BUDGET_SEC:.0f}s): keep phase 1 route', flush=True)
            break
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
                    t.open_via()
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
            if phase == 1 and VIA_ALWAYS and any(t.pending for t in T.values()):
                # 遠回りなしで経路が見つかっても、直接つなげなかった区間を遠回りで試し、速くなるなら採る
                saved = (total, list(used), list(visits), {k: dict(T[k].prims) for k in T})
                t_phase2 = time.time()
                phase = 2
                for t in T.values():
                    t.open_via()
                print(f'phase 1 {total:.1f}s -> phase 2 (detours)  {time.time() - t0:.1f}s', flush=True)
                continue
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
            blocks.append(dict(kind='visit', lap=i // 3 + 1, color=v['color'], mode=v['mode'], sign=v['sign'], off=v['off'], angle=v['angle'], angle_out=v['angle_out'],
                               prims=[ser(p) for p in v['prims']]))
    for i, v in enumerate(visits[:3]):
        print(f'  lap1 {v["color"]:6s} {v["mode"]:2s} sign={v["sign"]:+d} off={v["off"]:+.1f} angle={v["angle"]:+.0f}/{v["angle_out"]:+.0f} ea={v["ea"]} ee={v["ee"]}')
    for key, e, s in used:
        _USED_PRIMS[(key, e, s)] = T[key].prims[(e, s)]
    audit = route_audit([p for key, e, s in used for p in T[key].prims[(e, s)]], visits, used)
    print('audit: min clearance front %.2fcm rear %.2fcm (needed %.1f / %.1f), keepout hits %d' % (
        audit['min_front'], audit['min_rear'], MARGIN_FRONT, MARGIN_REAR, audit['keepout_hits']), flush=True)
    json.dump(dict(total=total, params=PARAMS, audit=audit, blocks=blocks), open(sys.argv[2], 'w'))


if __name__ == '__main__':
    main()
