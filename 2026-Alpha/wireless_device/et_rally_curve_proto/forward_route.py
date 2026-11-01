"""試作: その場旋回も後退もなしで、前進の直進と円弧だけでつないだ経路(4周+ゴール)を一から探す。
ゲートは前進でまっすぐ通る(shuttle_plan.py の 'F' の候補)。つなぎは Dubins 経路(円弧-直進-円弧 / 円弧-円弧-円弧、
半径 RADII)で、当たらないものの中から最短時間のものを選ぶ。止まるのはゴールだけ。
時間は 11/01 の4周(065319)に合わせた見積もり: 直進 360mm/s、円弧 0.77*360/(1+5.85/r) mm/s(半径10cmで約175mm/s)、
走り出して止まるまでの余分 0.6秒/回。
usage: [FR_LAPS=1] forward_route.py gates.json out_curved.json   (出力は curve_route.py と同じ形式。to_plan.py にそのまま渡せる)
"""
import json, math, sys, os, time
import numpy as np

GATES_JSON, OUT = sys.argv[1], sys.argv[2]
sys.argv = [sys.argv[0], GATES_JSON, '1.5', os.environ.get('FR_LAPS', '4'), 'unused.json', '0.4']
src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'shuttle_plan.py'), encoding='utf-8').read()
exec(src.replace("if __name__ == '__main__':", 'if False:'))

RADII = [float(x) for x in os.environ.get('FR_RADII', '20,15,12,10').split(',')]
EXT = [float(x) for x in os.environ.get('FR_EXT', '0,4,8,14,22').split(',')]
TAIL_CM = float(os.environ.get('FR_TAIL_CM', '4'))
V_LINE = 36.0                       # cm/s
ARC_K = 0.77
HALF_TREAD = 5.85
T_RUN = 0.6                         # 走り出して止まるまでの余分(1回)
TAU = 2 * math.pi


def v_arc(r):
    return ARC_K * V_LINE / (1 + HALF_TREAD / r)


def nL(phi):
    return np.array([-math.sin(phi), math.cos(phi)])


def tdir(phi):
    return np.array([math.cos(phi), math.sin(phi)])


def ccw(x):
    x = x % TAU
    return 0.0 if x > TAU - 1e-7 else x


def arc_prim(p, theta, s, sweep, r):
    return ('arc', p + s * r * nL(theta), r, theta, s, sweep, 1, p)


def arc_end(pr):
    _, c, r, theta, s, sweep, m, p0 = pr
    th = theta + s * sweep
    return c - s * r * nL(th), th


def arc_points(pr, step=1.0):
    _, c, r, theta, s, sweep, m, p0 = pr
    n = max(2, int(r * sweep / step) + 1)
    out = []
    for k in range(n + 1):
        th = theta + s * sweep * k / n
        out.append((c - s * r * nL(th), math.degrees(th)))
    return out


def prim_time(pr):
    if pr[0] == 'line':
        return float(np.hypot(*(pr[2] - pr[1]))) / V_LINE
    return pr[2] * pr[5] / v_arc(pr[2])


def prim_len(pr):
    return float(np.hypot(*(pr[2] - pr[1]))) if pr[0] == 'line' else pr[2] * pr[5]


_arc_ok_cache = {}


def prim_ok(pr):
    if pr[0] == 'line':
        if prim_len(pr) < 1e-6:
            return True
        return leg_ok(pr[1], pr[2], 1) is not None
    if pr[5] < 1e-6:
        return True
    key = (round(float(pr[1][0]), 3), round(float(pr[1][1]), 3), pr[2], pr[4], round(pr[3], 5), round(pr[5], 5))
    if key in _arc_ok_cache:
        return _arc_ok_cache[key]
    ok = True
    pts = arc_points(pr)
    for (pt, h) in pts:
        if not body_ok(pt, h, -REAR, FRONT) or any(x0 < pt[0] < x1 and y0 < pt[1] < y1 for x0, x1, y0, y1 in KEEPOUT):
            ok = False; break
    if ok:
        for (a, _), (b, _) in zip(pts, pts[1:]):
            if any(seg_cross(a, b, g.pa, g.pb) for g in GATES.values()):
                ok = False; break
    _arc_ok_cache[key] = ok
    return ok


def dubins(A, thA, B, thB, r):
    """前進だけの Dubins 経路の候補(プリミティブの列)。終わりの姿勢を数値で確かめたものだけ返す。"""
    out = []
    for s1 in (1, -1):
        for s2 in (1, -1):
            c1 = A + s1 * r * nL(thA); c2 = B + s2 * r * nL(thB)
            D = float(np.hypot(*(c2 - c1)))
            k = (s1 - s2) * r
            if D > abs(k) + 1e-9:
                phi = math.atan2(*(c2 - c1)[::-1]) + math.asin(k / D)
                p1 = c1 - s1 * r * nL(phi); p2 = c2 - s2 * r * nL(phi)
                out.append([arc_prim(A, thA, s1, ccw(s1 * (phi - thA)), r), ('line', p1, p2, 1),
                            arc_prim(p2, phi, s2, ccw(s2 * (thB - phi)), r)])
        # 円弧-円弧-円弧(s1 == s3、真ん中は逆回り)
    for s in (1, -1):
        c1 = A + s * r * nL(thA); c3 = B + s * r * nL(thB)
        D = float(np.hypot(*(c3 - c1)))
        if 1e-9 < D < 4 * r:
            al = math.atan2(*(c3 - c1)[::-1]); be = math.acos(D / (4 * r))
            for sg in (1, -1):
                c2 = c1 + 2 * r * tdir(al + sg * be)
                p1 = (c1 + c2) / 2; p2 = (c2 + c3) / 2
                v1 = (c1 - p1) / (s * r); th1 = math.atan2(-v1[0], v1[1])
                v2 = (c3 - p2) / (s * r); th2 = math.atan2(-v2[0], v2[1])
                out.append([arc_prim(A, thA, s, ccw(s * (th1 - thA)), r),
                            arc_prim(p1, th1, -s, ccw(-s * (th2 - th1)), r),
                            arc_prim(p2, th2, s, ccw(s * (thB - th2)), r)])
    good = []
    for prims in out:
        p, th = arc_end(prims[-1])
        if np.hypot(*(p - B)) < 1e-3 and abs(nd(math.degrees(th - thB))) < 1e-3:
            good.append([q for q in prims if prim_len(q) > 1e-6])
    return good


def best_conn(A, hA, B, hB, tail=0.0):
    """A(向きhA)→B(向きhB)の当たらない最短時間の前進経路。tail>0 なら最後を直進tail cmで終える(ゴール用)。"""
    thA, thB = math.radians(hA), math.radians(hB)
    Bm = B - tail * tdir(thB) if tail > 0 else B
    cands = []
    for r in RADII:
        for prims in dubins(A, thA, Bm, thB, r):
            if tail > 0:
                prims = prims + [('line', Bm, B, 1)]
            cands.append((sum(prim_time(p) for p in prims), prims))
    cands.sort(key=lambda x: x[0])
    for t, prims in cands:
        if all(prim_ok(p) for p in prims):
            return t, prims
    return INF, None


def main():
    t0 = time.time()
    global A_EXT, E_EXT
    A_EXT[:] = EXT; E_EXT[:] = EXT
    cands = {c: [v for v in visit_candidates(g) if v['mode'] == 'F'] for c, g in GATES.items()}
    print('candidates', {c: len(v) for c, v in cands.items()}, flush=True)

    def pose_s(v):
        return (tuple(np.round(v['start'][0], 3)), round(v['start'][1], 3))

    def pose_e(v):
        return (tuple(np.round(v['end'][0], 3)), round(v['end'][1], 3))

    starts = {c: {pose_s(v): v['start'] for v in cands[c]} for c in cands}
    ends = {c: {pose_e(v): v['end'] for v in cands[c]} for c in cands}
    sp0 = ((tuple(np.round(START, 3)), START_H), (np.array(START), START_H, 1))
    conn = {}
    for a, b in [('START', 'red'), ('red', 'blue'), ('blue', 'yellow'), ('yellow', 'red'), ('yellow', 'GOAL')]:
        eps = dict([sp0]) if a == 'START' else ends[a]
        tab = {}
        for ke, ep in eps.items():
            if b == 'GOAL':
                tab[(ke, 'GOAL')] = best_conn(ep[0], ep[1], np.array(GOAL), GOAL_H, TAIL_CM)
            else:
                for ks, sp in starts[b].items():
                    tab[(ke, ks)] = best_conn(ep[0], ep[1], sp[0], sp[1])
        conn[(a, b)] = tab
        print('conn', a, b, len(tab), sum(1 for v in tab.values() if v[0] < INF), f'{time.time() - t0:.1f}s', flush=True)

    seq = ['red', 'blue', 'yellow'] * LAPS
    prev = {sp0[0]: (0.0, None)}
    prev_color = 'START'
    hist = []
    for color in seq:
        cur = {}
        tab = conn[(prev_color, color)]
        for v in cands[color]:
            ks, ke = pose_s(v), pose_e(v)
            vt = sum(float(np.hypot(*(b - a))) for a, b, d in v['legs']) / V_LINE
            for kp, (cp, _) in prev.items():
                cc, prims = tab[(kp, ks)]
                if cc >= INF:
                    continue
                tot = cp + cc + vt
                if ke not in cur or tot < cur[ke][0]:
                    cur[ke] = (tot, (kp, v, prims))
        hist.append(cur)
        prev, prev_color = cur, color
    tab = conn[(prev_color, 'GOAL')]
    total, kp = min(((cp + tab[(kp, 'GOAL')][0], kp) for kp, (cp, _) in prev.items()), key=lambda x: x[0])
    goal_prims = tab[(kp, 'GOAL')][1]
    steps = []
    for i in range(len(seq) - 1, -1, -1):
        _, (kprev, v, prims) = hist[i][kp]
        steps.append((v, prims)); kp = kprev
    steps.reverse()
    total += T_RUN
    print(f'TOTAL forward-only laps={LAPS}: {total:.1f}s running  (compute {time.time() - t0:.1f}s)')

    def ser(p):
        if p[0] == 'line':
            return dict(t='line', a=p[1].tolist(), b=p[2].tolist(), m=1)
        return dict(t='arc', pts=[q.tolist() for q, _ in arc_points(p, 2.0)], m=1, r=p[2], deg=math.degrees(p[5]),
                    th0=math.degrees(p[3]), s=p[4])

    blocks = []
    L_line = L_arc = 0.0; t_lap = []; t_acc = 0.0
    for i, (v, prims) in enumerate(steps):
        vp = [('line', np.array(a), np.array(b), 1) for a, b, d in v['legs']]
        blocks.append(dict(kind='conn', lap=None, color=None, prims=[ser(p) for p in prims]))
        blocks.append(dict(kind='visit', lap=i // 3 + 1, color=v['color'], prims=[ser(p) for p in vp]))
        for p in prims + vp:
            t_acc += prim_time(p)
            if p[0] == 'line':
                L_line += prim_len(p)
            else:
                L_arc += prim_len(p)
        t_lap.append((i // 3 + 1, v['color'], round(t_acc, 1)))
        print(f'  lap{i // 3 + 1} {v["color"]:6s} sign={v["sign"]:+d} ea={v["ea"]} ee={v["ee"]} conn=' +
              ' '.join(f'{p[0]}{"" if p[0] == "line" else "(r%g,%+.0f°)" % (p[2], p[4] * math.degrees(p[5]))}:{prim_len(p):.0f}' for p in prims))
    blocks.append(dict(kind='conn', lap=None, color=None, prims=[ser(p) for p in goal_prims]))
    for p in goal_prims:
        if p[0] == 'line':
            L_line += prim_len(p)
        else:
            L_arc += prim_len(p)
    radii = {}
    for p in [q for _, pr in steps for q in pr] + goal_prims:
        if p[0] == 'arc':
            radii[p[2]] = radii.get(p[2], 0) + prim_len(p)
    print('times at gates', t_lap)
    print(f'line {L_line * 10:.0f}mm arc {L_arc * 10:.0f}mm', {k: round(v * 10) for k, v in radii.items()})
    json.dump(dict(total=total, laps=t_lap, blocks=blocks), open(OUT, 'w'))


main()
