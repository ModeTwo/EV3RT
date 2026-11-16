"""試作: ゲートの往復通過・後退を入れた経路の探索(検討用、devSSのソースは変えない)。

座標は右コースのplanner座標(plan JSONのgatesそのまま、スタート/ゴール/進入禁止はet_rally_runnerの反転後)。
時間の評価は 10/31 のログからの近似:
  旋回 0.7 + 0.012*deg 秒(直前の停止込み)、直進 0.5 + 0.0025*mm 秒、前後の切り替え +0.2 秒。
"""
import json, math, sys, time, os
import numpy as np

P = 24.64
FRONT, REAR, HW = 7.5, 17.5, 6.85      # 車軸から前端/後端、半幅(cm)
POST_R, ARM = 1.0, 4.0                 # 支柱半径、T字パーツの片側長
MARGIN = float(sys.argv[2]) if len(sys.argv) > 2 else 1.5   # 車体と支柱(T字込み)の最小すき間
LAPS = int(sys.argv[3]) if len(sys.argv) > 3 else 4
MARGIN_REAR = float(sys.argv[5]) if len(sys.argv) > 5 else 0.4   # 車軸より後ろの部分のすき間(今のplannerは後ろを見ていない)
PM = 2.0          # 通りきったと見なす、後ろ側の端が線を越える量
CLR = 0.5         # 通過前/後に、端が線から離れている量の最小
T_TURN0, T_TURN_DEG, T_MOVE0, T_MOVE_CM, T_REV = 0.7, 0.012, 0.5, 0.025, 0.2

INF = 1e6
START = (2 * 2 * P - (4 * P + 17.8), 3.5 * P); START_H = 0.0
GOAL = (2 * 2 * P - (0 * P - 14.9), 3.5 * P); GOAL_H = 90.0
mx = lambda x: 4 * P - x
KEEPOUT = [(mx(INF), mx(-17.0), -INF, -37.0),
           (mx(INF), mx(4 * P + 17.8 + 14.0), -INF, INF),
           (mx(-28.0), INF, 0.0, INF)]
BOUNDS = (-31.0, 126.0, -36.0, 4 * P + 40)

plan = json.load(open(sys.argv[1], encoding='utf-8'))


class Gate:
    def __init__(self, color, a, b):
        self.color = color
        self.pa = np.array([a[0] * P, a[1] * P]); self.pb = np.array([b[0] * P, b[1] * P])
        self.c = (self.pa + self.pb) / 2
        d = (self.pb - self.pa) / np.linalg.norm(self.pb - self.pa)
        self.n = np.array([-d[1], d[0]])


GATES = {c: Gate(c, plan['gates'][c]['a'], plan['gates'][c]['b']) for c in ('red', 'blue', 'yellow')}
CAPS = []   # (p0, p1) 半径POST_R
for g in GATES.values():
    for p in (g.pa, g.pb):
        CAPS.append((p - g.n * ARM, p + g.n * ARM))
CAP_SAMPLES = np.array([c0 + (c1 - c0) * t for c0, c1 in CAPS for t in np.linspace(0, 1, 9)])  # (54,2)
OPENINGS = [(g.pa, g.pb) for g in GATES.values()]


def ang(v):
    return math.degrees(math.atan2(v[1], v[0]))


def nd(a):
    return (a + 180.0) % 360.0 - 180.0


def body_ok(origin, h_deg, xmin, xmax):
    """車軸より前の部分(x>=0側の前端まで)と後ろの部分で、別のすき間を求める。
    xmin,xmax は車軸を原点にした、動いた範囲込みの車体の前後の範囲。"""
    # 前の部分: 車軸の軌跡 [xmin+REAR, xmax-FRONT] に前の箱 [0,FRONT] を足した範囲
    a0, a1 = xmin + REAR, xmax - FRONT
    if box_clear(origin, h_deg, a0, a1 + FRONT) < MARGIN:
        return False
    if box_clear(origin, h_deg, a0 - REAR, a1) < MARGIN_REAR:
        return False
    return True


def box_clear(origin, h_deg, xmin, xmax):
    """車体の箱(向きh、x∈[xmin,xmax]、y∈[-HW,HW])と支柱カプセルの最小すき間。"""
    c, s = math.cos(math.radians(h_deg)), math.sin(math.radians(h_deg))
    d = CAP_SAMPLES - origin
    lx = d[:, 0] * c + d[:, 1] * s
    ly = -d[:, 0] * s + d[:, 1] * c
    dx = np.maximum(np.maximum(xmin - lx, lx - xmax), 0)
    dy = np.maximum(np.abs(ly) - HW, 0)
    return float(np.min(np.hypot(dx, dy))) - POST_R


def seg_cross(a, b, c, d):
    def orient(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    o1, o2, o3, o4 = orient(a, b, c), orient(a, b, d), orient(c, d, a), orient(c, d, b)
    return (o1 * o2 < 0) and (o3 * o4 < 0)


def keepout_ok(a, b):
    for (x0, x1, y0, y1) in KEEPOUT:
        # 線分が長方形の内部(縁を除く)に入るか
        t0, t1 = 0.0, 1.0
        dx, dy = b[0] - a[0], b[1] - a[1]
        ok = False
        for p, q in ((-dx, a[0] - (x0 + 1e-6)), (dx, (x1 - 1e-6) - a[0]), (-dy, a[1] - (y0 + 1e-6)), (dy, (y1 - 1e-6) - a[1])):
            if abs(p) < 1e-12:
                if q < 0:
                    ok = True; break
            else:
                t = q / p
                if p < 0:
                    t0 = max(t0, t)
                else:
                    t1 = min(t1, t)
                if t0 > t1:
                    ok = True; break
        if not ok:
            return False
    return True


_leg_cache = {}


def leg_ok(a, b, d, allow_gate=None):
    key = (round(float(a[0]), 3), round(float(a[1]), 3), round(float(b[0]), 3), round(float(b[1]), 3), d,
           None if allow_gate is None else allow_gate.color)
    r = _leg_cache.get(key, 0)
    if r != 0:
        return r
    r = _leg_ok(a, b, d, allow_gate)
    _leg_cache[key] = r
    return r


def _leg_ok(a, b, d, allow_gate=None):
    """a→bの直進(d=+1前進/-1後退)が安全か。安全なら車体の向きを返す、だめならNone。"""
    v = b - a
    L = float(np.hypot(*v))
    if L < 1e-9:
        return None
    h = ang(v) if d > 0 else nd(ang(v) + 180)
    if d > 0:
        xmin, xmax = -REAR, L + FRONT
    else:
        xmin, xmax = -L - REAR, FRONT
    if not body_ok(a, h, xmin, xmax):
        return None
    if not keepout_ok(a, b):
        return None
    for g in GATES.values():
        if g is allow_gate:
            continue
        if seg_cross(a, b, g.pa, g.pb):
            return None
    return h


_pivot_cache = {}


def pivot_interval(p, h):
    """点pで向きhから、安全に回れる角度の範囲(左回りの最大, 右回りの最大)。"""
    key = (round(p[0], 2), round(p[1], 2), round(h, 2))
    if key in _pivot_cache:
        return _pivot_cache[key]
    res = []
    for sgn in (1, -1):
        m = 0
        for k in range(1, 182):
            if not body_ok(p, h + sgn * k, -REAR, FRONT):
                break
            m = k
        res.append(m)
    _pivot_cache[key] = tuple(res)
    return _pivot_cache[key]


def turn_ok(p, h_from, h_to):
    dd = nd(h_to - h_from)
    if abs(dd) < 0.3:
        return True, 0.0
    l, r = pivot_interval(p, h_from)
    if abs(abs(dd) - 180) < 0.5:
        return (l >= 180 or r >= 180), 180.0
    if dd > 0:
        return l >= dd - 0.5, abs(dd)
    return r >= -dd - 0.5, abs(dd)


def turn_cost(deg):
    return 0.0 if deg < 0.3 else T_TURN0 + T_TURN_DEG * deg


def move_cost(L):
    return T_MOVE0 + T_MOVE_CM * L


# --- ゲートの通り方の候補 ---
A_EXT = [float(x) for x in os.environ.get('SP_EXT', '0,4,8,14,22,35').split(',')]
E_EXT = list(A_EXT)
W_STEP = float(os.environ.get('SP_W', '5'))
FAST = os.environ.get('SP_FAST', '1') == '1'   # 1=まとめて計算する版(既定)、0=1つずつ判定する元の版(遅い)


def visit_candidates(g):
    """戻り値: [(mode, sign, start_pose, end_pose, legs, internal_cost)]
    pose = (point, heading, dir_of_adjacent_leg)。legs = [(a, b, d)]"""
    out = []
    for s in (1, -1):
        u = g.n * s
        hu = ang(u); hm = nd(hu + 180)
        C = g.c
        for mode in ('F', 'B', 'FB', 'BF'):
            if mode == 'F':
                base_a, h, d1 = FRONT + CLR, hu, 1
                tip = REAR + PM
            elif mode == 'B':
                base_a, h, d1 = REAR + CLR, hm, -1
                tip = FRONT + PM
            elif mode == 'FB':
                base_a, h, d1 = FRONT + CLR, hu, 1
                tip, base_b = REAR + PM, FRONT + CLR
            else:
                base_a, h, d1 = REAR + CLR, hm, -1
                tip, base_b = FRONT + PM, REAR + CLR
            for ea in A_EXT:
                S = C - u * (base_a + ea)
                for ee in E_EXT:
                    if mode in ('F', 'B'):
                        E = C + u * (tip + ee)
                        legs = [(S, E, d1)]
                        end_pose = (E, h, d1)
                    else:
                        T = C + u * tip
                        E = C - u * (base_b + ee)
                        legs = [(S, T, d1), (T, E, -d1)]
                        end_pose = (E, h, -d1)
                    ok = all(leg_ok(a, b, d, allow_gate=g) is not None for a, b, d in legs)
                    if not ok:
                        continue
                    cost = sum(move_cost(np.hypot(*(b - a))) for a, b, d in legs) + T_REV * (len(legs) - 1)
                    out.append(dict(mode=mode, sign=s, start=(S, h, d1), end=end_pose, legs=legs, cost=cost,
                                    ea=ea, ee=ee, color=g.color))
    return out


# --- 中継点(どの向きに回っても支柱に当たらない点) ---
SWEEP = math.hypot(REAR, HW)
W = []
for x in np.arange(BOUNDS[0], BOUNDS[1] + 0.1, W_STEP):
    for y in np.arange(BOUNDS[2], BOUNDS[3] + 0.1, W_STEP):
        p = np.array([x, y])
        if np.min(np.hypot(*(CAP_SAMPLES - p).T)) - POST_R < SWEEP + MARGIN:
            continue
        if any(x0 < x < x1 and y0 < y < y1 for x0, x1, y0, y1 in KEEPOUT):
            continue
        W.append(p)
W = np.array(W)


def connect_direct(ep, sp):
    return connect(ep, sp, direct_only=True)


def connect(ep, sp, direct_only=False):
    """終点姿勢ep(prev)から始点姿勢sp(next)への最短の接続。戻り値 (cost, legs)。
    sp[2] is None ならゴール(到着後に向きを合わせる、つなぎの直進なし)。"""
    E, hE, dE = ep
    S, hS, dS = sp
    best = (INF, None)

    def seq_cost(legs_h):
        # legs_h: [(a,b,d,h)]。E での旋回/合流、途中の旋回、S での旋回/合流を含む。
        cost = 0.0
        prev_h, prev_d, prev_p = hE, dE, E
        for i, (a, b, d, h) in enumerate(legs_h):
            dd = abs(nd(h - prev_h))
            if dd < 0.3 and prev_d == d:
                cost += T_MOVE_CM * np.hypot(*(b - a))   # 止まらずに続ける
            else:
                okk, deg = turn_ok(prev_p, prev_h, h)
                if not okk:
                    return INF
                cost += turn_cost(deg) + move_cost(np.hypot(*(b - a)))
                if dd < 0.3 and prev_d is not None and prev_d != d:
                    cost += T_REV
            prev_h, prev_d, prev_p = h, d, b
        dd = abs(nd(hS - prev_h))
        if dS is not None and dd < 0.3 and prev_d == dS:
            cost -= T_MOVE0          # 次の通過の直進と合流
        else:
            okk, deg = turn_ok(S, prev_h, hS)
            if not okk:
                return INF
            cost += turn_cost(deg)
            if dS is not None and dd < 0.3 and prev_d != dS:
                cost += T_REV
        return cost

    # 0中継
    if np.hypot(*(S - E)) < 1e-6:
        c = seq_cost([])
        if c < best[0]:
            best = (c, [])
    else:
        for d in (1, -1):
            h = leg_ok(E, S, d)
            if h is None:
                continue
            c = seq_cost([(E, S, d, h)])
            if c < best[0]:
                best = (c, [(E, S, d)])
    if best[0] < INF and best[1] is not None and len(best[1]) <= 1:
        direct = best
    if direct_only:
        return best
    # 1中継(中継点では自由に回れる)。距離の下限で枝刈りする。
    dist_lb = np.hypot(*(W - E).T) + np.hypot(*(W - S).T)
    order = np.argsort(dist_lb)
    for idx in order:
        w = W[idx]
        lb = 2 * T_MOVE0 + T_MOVE_CM * dist_lb[idx] - T_MOVE0
        if lb >= best[0]:
            break
        for d1 in (1, -1):
            h1 = leg_ok(E, w, d1)
            if h1 is None:
                continue
            for d2 in (1, -1):
                h2 = leg_ok(w, S, d2)
                if h2 is None:
                    continue
                c = seq_cost([(E, w, d1, h1), (w, S, d2, h2)])
                if c < best[0]:
                    best = (c, [(E, w, d1), (w, S, d2)])
    return best


# ---- まとめて計算する版(方法2) ----
def _legs_vec(origins, targets, d):
    """origins/targets: (N,2)。各直進の(安全か, 車体の向きdeg, 長さcm)を配列で返す。"""
    v = targets - origins
    L = np.hypot(v[:, 0], v[:, 1])
    th = np.arctan2(v[:, 1], v[:, 0])
    if d < 0:
        th = th + np.pi
    c, s_ = np.cos(th)[:, None], np.sin(th)[:, None]
    dd = CAP_SAMPLES[None, :, :] - origins[:, None, :]
    lx = dd[:, :, 0] * c + dd[:, :, 1] * s_
    ly = -dd[:, :, 0] * s_ + dd[:, :, 1] * c
    Lc = L[:, None]
    if d > 0:
        f0, f1, r0, r1 = 0.0 * Lc, Lc + FRONT, -REAR + 0.0 * Lc, Lc
    else:
        f0, f1, r0, r1 = -Lc, FRONT + 0.0 * Lc, -Lc - REAR, 0.0 * Lc
    dy = np.maximum(np.abs(ly) - HW, 0)
    cf = np.min(np.hypot(np.maximum(np.maximum(f0 - lx, lx - f1), 0), dy), axis=1) - POST_R
    cr = np.min(np.hypot(np.maximum(np.maximum(r0 - lx, lx - r1), 0), dy), axis=1) - POST_R
    ok = (cf >= MARGIN) & (cr >= MARGIN_REAR) & (L > 1e-6)
    # 進入禁止(線分が長方形の内部に入るか)
    ax, ay = origins[:, 0], origins[:, 1]
    vx, vy = v[:, 0], v[:, 1]
    for (x0, x1, y0, y1) in KEEPOUT:
        t0 = np.zeros(len(L)); t1 = np.ones(len(L)); miss = np.zeros(len(L), bool)
        for p, q in ((-vx, ax - (x0 + 1e-6)), (vx, (x1 - 1e-6) - ax), (-vy, ay - (y0 + 1e-6)), (vy, (y1 - 1e-6) - ay)):
            zero = np.abs(p) < 1e-12
            miss |= zero & (q < 0)
            with np.errstate(divide='ignore', invalid='ignore'):
                t = np.where(zero, 0.0, q / np.where(zero, 1.0, p))
            t0 = np.where(~zero & (p < 0), np.maximum(t0, t), t0)
            t1 = np.where(~zero & (p > 0), np.minimum(t1, t), t1)
        miss |= t0 > t1
        ok &= miss
    for g in GATES.values():
        def orient(px, py, qx, qy, rx, ry):
            return (qx - px) * (ry - py) - (qy - py) * (rx - px)
        bx, by = targets[:, 0], targets[:, 1]
        o1 = orient(ax, ay, bx, by, g.pa[0], g.pa[1]); o2 = orient(ax, ay, bx, by, g.pb[0], g.pb[1])
        o3 = orient(g.pa[0], g.pa[1], g.pb[0], g.pb[1], ax, ay); o4 = orient(g.pa[0], g.pa[1], g.pb[0], g.pb[1], bx, by)
        ok &= ~((o1 * o2 < 0) & (o3 * o4 < 0))
    return ok, np.degrees(th), L


def _nd_vec(a):
    return (a + 180.0) % 360.0 - 180.0


def _turn_vec(dd, l, r):
    """dd: 回る量(符号付きdeg)の配列、l/r: 左回り/右回りに安全な最大角。"""
    a = np.abs(dd)
    ok = (a < 0.3) | ((dd > 0) & (l >= dd - 0.5)) | ((dd < 0) & (r >= -dd - 0.5))
    ok |= (np.abs(a - 180) < 0.5) & ((l >= 180) | (r >= 180))
    cost = np.where(a < 0.3, 0.0, T_TURN0 + T_TURN_DEG * a)
    return ok, cost


_vec_from = {}
_vec_to = {}


def connect_fast(ep, sp):
    E, hE, dE = ep
    S, hS, dS = sp
    best = connect_direct(ep, sp)
    N = len(W)
    kE = (round(float(E[0]), 3), round(float(E[1]), 3))
    if kE not in _vec_from:
        _vec_from[kE] = {d: _legs_vec(np.repeat(E[None, :], N, 0), W, d) for d in (1, -1)}
    kS = (round(float(S[0]), 3), round(float(S[1]), 3))
    if kS not in _vec_to:
        _vec_to[kS] = {d: _legs_vec(W, np.repeat(S[None, :], N, 0), d) for d in (1, -1)}
    lE, rE = pivot_interval(E, hE)
    lS, rS = pivot_interval(S, hS)
    for d1 in (1, -1):
        ok1, h1, L1 = _vec_from[kE][d1]
        dd = _nd_vec(h1 - hE)
        merge = (np.abs(dd) < 0.3) & (dE == d1)
        tok, tc = _turn_vec(dd, lE, rE)
        c1 = np.where(merge, T_MOVE_CM * L1, tc + T_MOVE0 + T_MOVE_CM * L1
                      + np.where((np.abs(dd) < 0.3) & (dE is not None) & (dE != d1), T_REV, 0.0))
        ok1 = ok1 & (merge | tok)
        for d2 in (1, -1):
            ok2, h2, L2 = _vec_to[kS][d2]
            dw = _nd_vec(h2 - h1)
            mw = (np.abs(dw) < 0.3) & (d1 == d2)
            cw = np.where(mw, T_MOVE_CM * L2, np.where(np.abs(dw) < 0.3, 0.0, T_TURN0 + T_TURN_DEG * np.abs(dw))
                          + T_MOVE0 + T_MOVE_CM * L2 + np.where((np.abs(dw) < 0.3) & (d1 != d2), T_REV, 0.0))
            ds = _nd_vec(hS - h2)
            if dS is not None:
                mS = (np.abs(ds) < 0.3) & (d2 == dS)
            else:
                mS = np.zeros(N, bool)
            # h2→hS の旋回: hS のまわりの安全な範囲で判定する(向きの範囲は同じ)
            sok, sc = _turn_vec(-ds, lS, rS)
            cs = np.where(mS, -T_MOVE0, sc + np.where((dS is not None) & (np.abs(ds) < 0.3) & (d2 != dS), T_REV, 0.0))
            total = np.where(ok1 & ok2 & (mS | sok), c1 + cw + cs, INF)
            i = int(np.argmin(total))
            if total[i] < best[0]:
                best = (float(total[i]), [(E, W[i], d1), (W[i], S, d2)])
    return best


def main():
    t0 = time.time()
    cands = {c: visit_candidates(g) for c, g in GATES.items()}
    print('candidates', {c: len(v) for c, v in cands.items()}, 'W', len(W), f'{time.time()-t0:.1f}s', flush=True)

    def uniq(poses):
        out, keys = [], {}
        for p in poses:
            k = (round(p[0][0], 3), round(p[0][1], 3), round(p[1], 3), p[2])
            if k not in keys:
                keys[k] = len(out); out.append(p)
        return out, keys

    def pkey(p):
        return (round(p[0][0], 3), round(p[0][1], 3), round(p[1], 3), p[2])

    starts, ends = {}, {}
    for c in cands:
        starts[c], _ = uniq([v['start'] for v in cands[c]])
        ends[c], _ = uniq([v['end'] for v in cands[c]])

    conn = {}
    pairs = [('START', 'red'), ('red', 'blue'), ('blue', 'yellow'), ('yellow', 'red'), ('yellow', 'GOAL')]
    for a, b in pairs:
        eps = [(np.array(START), START_H, None)] if a == 'START' else ends[a]
        sps = [(np.array(GOAL), GOAL_H, None)] if b == 'GOAL' else starts[b]
        tab = {}
        for ep in eps:
            for sp in sps:
                tab[(pkey(ep), pkey(sp))] = connect_fast(ep, sp) if FAST else connect(ep, sp)
        conn[(a, b)] = tab
        print('conn', a, b, len(tab), sum(1 for v in tab.values() if v[0] < INF), f'{time.time()-t0:.1f}s', flush=True)

    # DP: ステージごとに「通過後の終点姿勢」までの最小時間
    seq = ['red', 'blue', 'yellow'] * LAPS
    prev = {pkey((np.array(START), START_H, None)): (0.0, None, (np.array(START), START_H, None))}
    prev_color = 'START'
    hist = []
    for color in seq:
        cur = {}
        tab = conn[(prev_color, color)]
        for v in cands[color]:
            ks = pkey(v['start'])
            ke = pkey(v['end'])
            for kp, (cp, _, ep) in prev.items():
                cc, legs = tab[(kp, ks)]
                if cc >= INF:
                    continue
                tot = cp + cc + v['cost']
                if ke not in cur or tot < cur[ke][0]:
                    cur[ke] = (tot, (kp, v, legs), v['end'])
        hist.append(cur)
        prev = cur
        prev_color = color
    tab = conn[(prev_color, 'GOAL')]
    gk = pkey((np.array(GOAL), GOAL_H, None))
    if not prev:
        print('NO ROUTE at stage end'); return
    best = min(((cp + tab[(kp, gk)][0], kp) for kp, (cp, _, _) in prev.items()), key=lambda x: x[0])
    total, kp = best
    goal_legs = tab[(kp, gk)][1]
    # 復元
    steps = []
    for i in range(len(seq) - 1, -1, -1):
        tot, (kprev, v, legs), _ = hist[i][kp]
        steps.append((v, legs))
        kp = kprev
    steps.reverse()
    print(f'TOTAL laps={LAPS} margin={MARGIN} predicted {total:.1f}s  ({time.time()-t0:.1f}s compute)')
    out = []
    for i, (v, legs) in enumerate(steps):
        lap = i // 3 + 1
        out.append(dict(kind='conn', legs=[(a.tolist(), b.tolist(), d) for a, b, d in legs]))
        out.append(dict(kind='visit', lap=lap, color=v['color'], mode=v['mode'], sign=v['sign'],
                        ea=v['ea'], ee=v['ee'], legs=[(a.tolist(), b.tolist(), d) for a, b, d in v['legs']]))
        print(f'  lap{lap} {v["color"]:6s} {v["mode"]:2s} sign={v["sign"]:+d} ea={v["ea"]} ee={v["ee"]} conn_legs={len(legs)}')
    out.append(dict(kind='conn', legs=[(a.tolist(), b.tolist(), d) for a, b, d in goal_legs]))
    json.dump(dict(total=total, laps=LAPS, margin=MARGIN, items=out), open(sys.argv[4] if len(sys.argv) > 4 else 'route.json', 'w'))


if __name__ == '__main__':
    main()
