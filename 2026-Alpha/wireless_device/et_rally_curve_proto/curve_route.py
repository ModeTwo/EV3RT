"""route.json(shuttle_plan.py)のつなぎ区間を、円弧+直進+円弧(など)に置き換える(後処理)。
ゲートを通る直進はそのまま。時間: 止まらずに続く区間は 0.5秒 + 0.0025秒/mm(円弧も直進と同じ速さと仮定)、
その場旋回 0.7 + 0.012*deg、前後の切り替え +0.2。
usage: curve_route.py plan.json route.json out.json
"""
import json, math, sys
import numpy as np

sys.argv = [sys.argv[0], sys.argv[1], '1.5', '4', 'unused.json', '0.4'] + sys.argv[2:]
src = open(__file__.replace('curve_route.py', 'shuttle_plan.py'), encoding='utf-8').read()
exec(src.replace("if __name__ == '__main__':", 'if False:'))
ROUTE_IN, OUT = sys.argv[6], sys.argv[7]
RADII = [20.0, 15.0, 12.0, 10.0]
HALF_TREAD = 5.85   # plotter.WHEEL_TREAD 117mm の半分。外側の車輪が直進の巡航を超えない速さで円弧を走る
TAU = 2 * math.pi


def nL(phi):
    return np.array([-math.sin(phi), math.cos(phi)])


def tdir(phi):
    return np.array([math.cos(phi), math.sin(phi)])


def ccw(x):
    x = x % TAU
    return 0.0 if x > TAU - 1e-7 else x


def arc_prim(p, theta, s, sweep, r, m):
    """点pで進む向きtheta、s=+1左/-1右、sweep(rad>=0)の円弧。"""
    c = p + s * r * nL(theta)
    return ('arc', c, r, theta, s, sweep, m, p)


def arc_points(prim, step=1.0):
    _, c, r, theta, s, sweep, m, p0 = prim
    n = max(2, int(r * sweep / step) + 1)
    pts = []
    for k in range(n + 1):
        th = theta + s * sweep * k / n
        pt = c - s * r * nL(th)
        h = math.degrees(th) if m > 0 else math.degrees(th) + 180
        pts.append((pt, nd(h)))
    return pts


def prim_len(pr):
    if pr[0] == 'line':
        return float(np.hypot(*(pr[2] - pr[1])))
    if pr[0] == 'arc':
        return pr[2] * pr[5]
    return 0.0


def prim_ok(pr):
    if pr[0] == 'line':
        if prim_len(pr) < 1e-6:
            return True
        return leg_ok(pr[1], pr[2], pr[3]) is not None
    if pr[0] == 'arc':
        if pr[5] < 1e-6:
            return True
        pts = arc_points(pr)
        for (pt, h) in pts:
            if not body_ok(pt, h, -REAR, FRONT):
                return False
            if any(x0 < pt[0] < x1 and y0 < pt[1] < y1 for x0, x1, y0, y1 in KEEPOUT):
                return False
        for (a, _), (b, _) in zip(pts, pts[1:]):
            for g in GATES.values():
                if seg_cross(a, b, g.pa, g.pb):
                    return False
        return True
    if pr[0] == 'pivot':
        okk, _ = turn_ok(pr[1], pr[2], pr[3])
        return okk
    return True


def end_pose(pr):
    """プリミティブの終わりの(点, 車体の向き)"""
    if pr[0] == 'line':
        _, a, b, m = pr
        h = ang(b - a) if m > 0 else nd(ang(b - a) + 180)
        return b, h
    if pr[0] == 'arc':
        _, c, r, theta, s, sweep, m, p0 = pr
        th = theta + s * sweep
        return c - s * r * nL(th), nd(math.degrees(th) + (0 if m > 0 else 180))
    return pr[1], pr[3]


def time_of(prims):
    t, in_run, cur_m, last_m = 0.0, False, None, None
    for pr in prims:
        if pr[0] == 'pivot':
            deg = abs(nd(pr[3] - pr[2]))
            if deg >= 0.3:
                t += turn_cost(deg)
                in_run = False
            continue
        m = pr[3] if pr[0] == 'line' else pr[6]
        L = prim_len(pr)
        if L < 1e-6:
            continue
        if not in_run:
            t += T_MOVE0
            if last_m is not None and last_m != m and not just_pivoted(prims, pr):
                t += T_REV
        elif cur_m != m:
            t += T_REV + T_MOVE0
        t += T_MOVE_CM * L * ((1 + HALF_TREAD / pr[2]) if pr[0] == 'arc' else 1.0)
        in_run, cur_m, last_m = True, m, m
    return t


def just_pivoted(prims, pr):
    i = next(k for k, q in enumerate(prims) if q is pr)
    return i > 0 and prims[i - 1][0] == 'pivot' and abs(nd(prims[i - 1][3] - prims[i - 1][2])) >= 0.3


def options(A, hA, B, hB, is_goal):
    """A(車体の向きhA)からB(向きhB)へのつなぎの候補(プリミティブの列)。"""
    out = []
    for m in (1, -1):
        thA = math.radians(hA if m > 0 else hA + 180)
        thB = math.radians(hB if m > 0 else hB + 180)
        for r in RADII:
            for s1 in (1, -1):
                for s2 in (1, -1):
                    # CSC
                    c1 = A + s1 * r * nL(thA); c2 = B + s2 * r * nL(thB)
                    D = float(np.hypot(*(c2 - c1)))
                    k = (s1 - s2) * r
                    if D > abs(k) + 1e-9:
                        alpha = math.atan2(*(c2 - c1)[::-1])
                        phi = alpha + math.asin(k / D)
                        p1 = c1 - s1 * r * nL(phi); p2 = c2 - s2 * r * nL(phi)
                        a1 = ccw(s1 * (phi - thA)); a2 = ccw(s2 * (thB - phi))
                        out.append([arc_prim(A, thA, s1, a1, r, m), ('line', p1, p2, m), arc_prim(p2, phi, s2, a2, r, m)])
                # その場旋回 → 直進 → 円弧
                c2 = B + s1 * r * nL(thB)
                D = float(np.hypot(*(c2 - A)))
                if D > r + 1e-9:
                    alpha = math.atan2(*(c2 - A)[::-1])
                    phi = alpha - math.asin(s1 * r / D)
                    p2 = c2 - s1 * r * nL(phi)
                    a2 = ccw(s1 * (thB - phi))
                    hphi = nd(math.degrees(phi) + (0 if m > 0 else 180))
                    out.append([('pivot', A, hA, hphi), ('line', A, p2, m), arc_prim(p2, phi, s1, a2, r, m)])
                # 円弧 → 直進 → その場旋回
                c1 = A + s1 * r * nL(thA)
                D = float(np.hypot(*(B - c1)))
                if D > r + 1e-9:
                    alpha = math.atan2(*(B - c1)[::-1])
                    phi = alpha + math.asin(s1 * r / D)
                    p1 = c1 - s1 * r * nL(phi)
                    a1 = ccw(s1 * (phi - thA))
                    hphi = nd(math.degrees(phi) + (0 if m > 0 else 180))
                    out.append([arc_prim(A, thA, s1, a1, r, m), ('line', p1, B, m), ('pivot', B, hphi, hB)])
    return out


TAIL_CM = float(__import__('os').environ.get('CR_TAIL_CM', '4'))


def _opt_motion(opt):
    for p in reversed(opt):
        if p[0] == 'line':
            return p[3]
        if p[0] == 'arc':
            return p[6]
    return None


def stop_safe_options(A, hA, B, hB, is_goal, d_next):
    """止まる場所(ゴール、または次の通過と前後が逆)で終わるつなぎは、円弧のまま止まらず、
    最後に TAIL_CM の直進を足して直進で止まる候補にする(2026-11-01、止まって終わる円弧で約1cmずれたため)。"""
    out = []
    for opt in options(A, hA, B, hB, is_goal):
        m = _opt_motion(opt)
        stops = is_goal or (m is not None and m != d_next)
        if not stops or opt[-1][0] == 'pivot' or TAIL_CM <= 0:
            out.append(opt)
    if TAIL_CM > 0:
        for m in (1, -1):
            if not (is_goal or m != d_next):
                continue
            thB = math.radians(hB if m > 0 else hB + 180)
            Bm = B - TAIL_CM * tdir(thB)
            for opt in options(A, hA, Bm, hB, is_goal):
                if _opt_motion(opt) != m or opt[-1][0] == 'pivot':
                    continue
                out.append(opt + [('line', Bm, B, m)])
    return out


def legs_to_prims(legs, h0):
    prims, h = [], h0
    for a, b, d in legs:
        a, b = np.array(a), np.array(b)
        hh = ang(b - a) if d > 0 else nd(ang(b - a) + 180)
        prims.append(('pivot', a, h, hh))
        prims.append(('line', a, b, d))
        h = hh
    return prims, h


def main():
    route = json.load(open(ROUTE_IN))
    items = route['items']
    # 通過区間をプリミティブにする
    seq = []   # [('conn', legs) | ('visit', item)]
    h = START_H
    blocks = []
    for it in items:
        blocks.append(it)
    out_prims = []          # 区間ごとのプリミティブ列
    orig_prims = []
    cur_pose = (np.array(START), START_H)
    for i, it in enumerate(blocks):
        if it['kind'] == 'visit':
            pr, hend = legs_to_prims(it['legs'], cur_pose[1])
            # 通過の最初の旋回は、つなぎ側で処理するので外す(向きは合っている前提)
            pr = pr[1:]
            out_prims.append(('visit', it, pr))
            cur_pose = end_pose(pr[-1])
            continue
        # つなぎ: 前の終わりの姿勢から、次の通過の始点の姿勢へ
        if i + 1 < len(blocks):
            nxt = blocks[i + 1]
            a, b, d = nxt['legs'][0]
            hB = ang(np.array(b) - np.array(a)) if d > 0 else nd(ang(np.array(b) - np.array(a)) + 180)
            B = np.array(a); is_goal = False
        else:
            B, hB, is_goal = np.array(GOAL), GOAL_H, True
        A, hA = cur_pose
        orig, hlast = legs_to_prims(it['legs'], hA)
        orig.append(('pivot', B, hlast, hB))
        prev_tail = out_prims[-1][2][-1:] if out_prims else []
        next_head = [] if is_goal else legs_to_prims(blocks[i + 1]['legs'][:1], hB)[0][1:2]
        best = (time_of(prev_tail + orig + next_head), orig, 'orig')
        d_next = None if is_goal else d
        for opt in stop_safe_options(A, hA, B, hB, is_goal, d_next):
            if i == 0 and __import__('os').environ.get('CR_START_FWD') == '1' and any(
                    (p[0] == 'line' and p[3] < 0) or (p[0] == 'arc' and p[6] < 0) for p in opt):
                continue   # スタート直後は前進だけ(前の版と比べるため)
            if not all(prim_ok(p) for p in opt):
                continue
            c = time_of(prev_tail + opt + next_head)
            if c < best[0] - 1e-9:
                best = (c, opt, 'curve')
        out_prims.append(('conn', it, best[1]))
        orig_prims.append(orig)
        cur_pose = (B, hB)
    allp = [p for _, _, pr in out_prims for p in pr]
    total = time_of(allp)
    print(f'curved total {total:.1f}s')
    # 周回ごとの時刻
    t_acc = []
    acc = []
    for kind, it, pr in out_prims:
        acc += pr
        if kind == 'visit':
            t_acc.append((it['lap'], it['color'], round(time_of(acc), 1)))
    print(t_acc)
    # 停止の回数・旋回
    stops = sum(1 for p in allp if p[0] == 'pivot' and abs(nd(p[3] - p[2])) >= 0.3)
    print('pivots', stops, 'pivot_deg', round(sum(abs(nd(p[3] - p[2])) for p in allp if p[0] == 'pivot' and abs(nd(p[3] - p[2])) >= 0.3)))
    for kind, it, pr in out_prims[:8]:
        print(kind, it.get('color', ''), [(p[0], round(prim_len(p), 1) if p[0] != 'pivot' else round(abs(nd(p[3] - p[2])), 1),
                                           (p[3] if p[0] == 'line' else p[6]) if p[0] != 'pivot' else '', round(p[2], 1) if p[0] == 'arc' else '') for p in pr])

    def ser(p):
        if p[0] == 'line':
            return dict(t='line', a=p[1].tolist(), b=p[2].tolist(), m=p[3])
        if p[0] == 'arc':
            return dict(t='arc', pts=[q.tolist() for q, _ in arc_points(p, 2.0)], m=p[6], r=p[2], deg=math.degrees(p[5]), th0=math.degrees(p[3]), s=p[4])
        return dict(t='pivot', p=p[1].tolist(), deg=abs(nd(p[3] - p[2])), h_from=p[2], h_to=p[3])
    json.dump(dict(total=total, laps=t_acc, blocks=[dict(kind=k, lap=it.get('lap'), color=it.get('color'), prims=[ser(p) for p in pr]) for k, it, pr in out_prims]),
              open(OUT, 'w'))


main()
