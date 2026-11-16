"""curve_route.py の出力(c_*.json、右コースのplanner座標)を、実機の計画JSON(move/turn/arc)にする。
右コースのファイル実行では gyro目標 = -planner角(1031bと同じ規約)。
usage: to_plan.py c.json gates.json out_plan.json [laps]
"""
import json, math, sys

c = json.load(open(sys.argv[1]))
gates = json.load(open(sys.argv[2]))['gates']
nd = lambda a: (a + 180.0) % 360.0 - 180.0
# 2026-11-09: スタートの向き(planner角)を基準に gyro 目標を出す(新スタートは向き180)。ジャイロはスタートで0。
START_H0 = 180.0 if c.get('params', {}).get('start_at') in ('goal', 'goal_mid') else 0.0
garage = c.get('params', {}).get('goal_at') == 'garage'
PIVOT_BACK = float(c.get('params', {}).get('pivot_back_cm', 0.0) or 0.0)   # 2026-11-13: その場の旋回の中心が車軸より後ろの量(cm)
g = lambda h: round(nd(-(h - START_H0)), 2)

steps = []
h = START_H0     # planner frame robot heading
max_gap = 0.0
# 2026-11-11(ユーザー): ガレージへの最後のつなぎで、アームを下げても脚に当たらない点(hybrid_route の arm_down、
# 黄ゲートを抜けたあとの距離 cm)から下げ始める。その点で直進・円弧を2つに分け、後ろの区間に印 'garage-arm-down' を付ける。
ARM_DOWN = c.get('arm_down')


def _split_prims(prims, d0):
    out = []; cum = 0.0; done = False
    for p in prims:
        if done or p['t'] == 'pivot':
            out.append(dict(p)); continue
        if p['t'] == 'line':
            L = math.hypot(p['b'][0] - p['a'][0], p['b'][1] - p['a'][1])
        else:
            L = p['r'] * math.radians(p['deg'])
        if cum + L <= d0 + 1e-6:
            out.append(dict(p)); cum += L; continue
        f = max(0.0, (d0 - cum) / L) if L > 0 else 0.0
        if f * L < 0.05:
            q = dict(p); q['arm_down'] = True; out.append(q)
        elif p['t'] == 'line':
            m_ = [p['a'][0] + (p['b'][0] - p['a'][0]) * f, p['a'][1] + (p['b'][1] - p['a'][1]) * f]
            out.append(dict(p, b=m_)); out.append(dict(p, a=m_, arm_down=True))
        else:
            d1 = p['deg'] * f
            out.append(dict(p, deg=d1)); out.append(dict(p, deg=p['deg'] - d1, th0=p['th0'] + p['s'] * d1, arm_down=True))
        done = True
    return out


def _start_point(p):
    if p['t'] == 'line':
        return list(p['a'])
    if p['t'] == 'arc':
        return list(p['pts'][0])
    return list(p['p'])   # その場の旋回: 回り始める車軸の位置


def prims_to_steps(seq, h):
    """seq: [(prim, label, mark)]。mark(周の終わりの印など)は、そのプリミティブから最初にできたステップに付ける
    (できなければ次にできたステップへ)。戻り値 (steps, h, max_gap)。
    2026-11-13: 道具の出力は区間のつなぎ目で向きが最大3.4°とぶ(1周に2回ほど)。以前は直進を道具の向き、円弧を道具の角度のまま
    並べていたので、とびのたびに位置が約2mm ずれ、10周で約1cm たまっていた(ロボットは計画の座標をめがけて走るので、当たり判定を
    した経路から離れる)。いまの位置と向きを積算しながら、円弧は「いまの向き → 道具の円弧の終わりの向き」を回る角度にし、
    直進は「いまの位置 → 道具の直進の終わりの点」へ向かう向きと長さにする(直進のたびに道具の経路に戻る)。"""
    out = []; gap = 0.0; pending = None
    pos = None
    for p, label, mark in seq:
        if pos is None:
            pos = _start_point(p)
            # 始めの向きは、最初の区間の向き(切り上げのつなぎは、黄の抜けの向きから始まる)
            if p['t'] == 'line':
                h = math.degrees(math.atan2(p['b'][1] - p['a'][1], p['b'][0] - p['a'][0])) + (0 if p['m'] > 0 else 180)
            elif p['t'] == 'arc':
                h = p['th0'] + (0 if p['m'] > 0 else 180)
            else:
                h = p['h_from']
        if mark:
            pending = mark
        n0 = len(out)
        if p['t'] == 'pivot':
            if p['deg'] >= 0.3:
                gap = max(gap, abs(nd(p['h_from'] - h)))
                out.append({'type': 'turn', 'target_heading_deg': g(p['h_to']), 'label': None})
                h_to_r = START_H0 - out[-1]['target_heading_deg']   # 丸めた向き
                # その場の旋回では、回る中心が車軸の PIVOT_BACK 後ろにあり、車軸も動く(実測。経路を作る道具も同じ形)。
                # 次の直進を道具の点へ向けるとき、動いたあとの位置から数える。
                qx, qy = pos[0] - PIVOT_BACK * math.cos(math.radians(h)), pos[1] - PIVOT_BACK * math.sin(math.radians(h))
                pos = [qx + PIVOT_BACK * math.cos(math.radians(h_to_r)), qy + PIVOT_BACK * math.sin(math.radians(h_to_r))]
                h = h_to_r
            else:
                h = p['h_to']
        elif p['t'] == 'line':
            e = p['b']
            dx, dy = e[0] - pos[0], e[1] - pos[1]
            L = math.hypot(dx, dy) * 10
            if L >= 0.5:
                hh = math.degrees(math.atan2(dy, dx)) + (0 if p['m'] > 0 else 180)
                gap = max(gap, abs(nd(hh - h)))
                st_ = {'type': 'move', 'target_heading_deg': g(hh), 'distance_mm': round(L, 1), 'reverse': p['m'] < 0,
                       'stop': False, 'label': 'garage-arm-down' if p.get('arm_down') else label}
                out.append(st_)
                # 2026-11-13: ロボットが積算するのと同じく、丸めた向き・距離で位置を進める(丸めの誤差が周を重ねてたまらないように)
                h = START_H0 - st_['target_heading_deg']
                th_ = math.radians(h + (0 if p['m'] > 0 else 180))
                pos = [pos[0] + st_['distance_mm'] / 10 * math.cos(th_), pos[1] + st_['distance_mm'] / 10 * math.sin(th_)]
        else:
            if p['deg'] >= 0.05:
                h0 = p['th0'] + (0 if p['m'] > 0 else 180)
                gap = max(gap, abs(nd(h0 - h)))
                h1 = h0 + p['s'] * p['deg']
                sw = p['s'] * p['deg'] + nd(h0 - h)        # いまの向き h から h1 まで(向きのとびを円弧で吸収する)
            if p['deg'] >= 0.05 and abs(sw) >= 0.05:
                st_ = {'type': 'arc', 'target_heading_deg': g(h1), 'sweep_deg': round(-sw, 2),
                       'radius_mm': round(p['r'] * 10, 1), 'reverse': p['m'] < 0, 'stop': False,
                       'label': 'garage-arm-down' if p.get('arm_down') else None}
                out.append(st_)
                # ロボットと同じく、丸めた半径・角度で、いまの向きから積算する
                sw = -st_['sweep_deg']; r = st_['radius_mm'] / 10
                sg = 1.0 if sw > 0 else -1.0
                th0 = math.radians(h + (0 if p['m'] > 0 else 180))
                cx, cy = pos[0] - sg * r * math.sin(th0), pos[1] + sg * r * math.cos(th0)
                th1 = th0 + math.radians(sw)
                pos = [cx + sg * r * math.sin(th1), cy - sg * r * math.cos(th1)]
                h = h + sw
        if pending and len(out) > n0:
            out[n0]['marks'] = [pending]; pending = None
    return out, h, gap


SMOOTH_DEV_MM = 0.5    # 2026-11-14: 折れを大きめの円弧にするとき、元の経路から離れてよい量
SMOOTH_MAX_SWEEP = 12.0
SMOOTH_MAX_ARC_MM = 40.0
SMOOTH_MAX_R_MM = 1000.0


def smooth_corners(steps):
    """2026-11-14(ユーザー: 狭い直進の揺れを減らす): 「直進 → ごく短い円弧(数mm で数度)→ 直進」の折れを、同じ角度で
    半径の大きい円弧に置き換える(前後の直進を同じだけ短くし、2本の直線に接する円弧にする)。元の経路から離れるのは
    SMOOTH_DEV_MM 以内(半径 R の円弧と元の角の差 ≈ R・θ²/8)。数mm で数度曲がる円弧はロボットが曲がりきれず、直進に入ってから
    左右に振れていた(配置36の赤のあとの狭い直進)。印(marks)の付いた区間の始まりは動かさない(切り上げの道の始まりと合わせるため)。"""
    out = [dict(st) for st in steps]
    for i in range(len(out) - 2):
        a, b, c_ = out[i], out[i + 1], out[i + 2]
        if not (a['type'] == 'move' and b['type'] == 'arc' and c_['type'] == 'move'):
            continue
        if a.get('reverse') or b.get('reverse') or c_.get('reverse') or b.get('label') or b.get('marks') or c_.get('marks'):
            continue
        th = math.radians(abs(b['sweep_deg']))
        if th < 1e-4 or abs(b['sweep_deg']) > SMOOTH_MAX_SWEEP or b['radius_mm'] * th > SMOOTH_MAX_ARC_MM:
            continue
        r = b['radius_mm']
        R = min(SMOOTH_MAX_R_MM, max(r, 8.0 * SMOOTH_DEV_MM / (th * th)))
        t0, t1 = r * math.tan(th / 2), R * math.tan(th / 2)
        room = min(a['distance_mm'], c_['distance_mm']) - 10.0
        if t1 - t0 > room:
            t1 = t0 + max(0.0, room)
            R = t1 / math.tan(th / 2)
        d = t1 - t0
        if R <= r + 1.0 or d <= 0.5:
            continue
        a['distance_mm'] = round(a['distance_mm'] - d, 1)
        c_['distance_mm'] = round(c_['distance_mm'] - d, 1)
        b['radius_mm'] = round(R, 1)
        b['smoothed_from_mm'] = r
    return out


def finalize(steps, main):
    """止まる場所・止まって終わる円弧のあとの向きの直し・(ゴールなら)最後の向きの旋回。"""
    # 止まる場所: 旋回の前・前後の切り替えの前・最後
    for i, st in enumerate(steps):
        if st['type'] == 'turn':
            continue
        nxt = steps[i + 1] if i + 1 < len(steps) else None
        if nxt is None or nxt['type'] == 'turn' or nxt['reverse'] != st['reverse']:
            st['stop'] = True
    # 止まって終わる円弧のあとには、向きを直す旋回を入れる(止まる手前で終える分、円弧の終わりの向きが足りないため)。
    # 最後には、ゴールの向き(planner 90度 = gyro -90度)への旋回を入れる。
    fixed = []
    for i, st in enumerate(steps):
        fixed.append(st)
        nxt = steps[i + 1] if i + 1 < len(steps) else None
        if st['type'] == 'arc' and st['stop'] and (nxt is None or nxt['type'] != 'turn'):
            fixed.append({'type': 'turn', 'target_heading_deg': st['target_heading_deg'], 'label': 'arc-fix'})
    if garage or not main:
        pass   # ガレージは向きの決まりがないので、最後の旋回はしない
    elif fixed[-1]['type'] != 'turn' or fixed[-1]['label'] != 'arc-fix':
        fixed.append({'type': 'turn', 'target_heading_deg': g(90.0), 'label': 'goal'})
    else:
        fixed[-1] = {'type': 'turn', 'target_heading_deg': g(90.0), 'label': 'goal'}
    return fixed


LAPS = c.get('params', {}).get('laps', 4)
EXIT = c.get('exit') if garage else None
seq = []
carry_mark = None
blocks = c['blocks']


def _sig(b):
    """区間の形の比べ用(座標を丸めたもの)。"""
    out = []
    for p in b['prims']:
        if p['t'] == 'line':
            out.append(('l', round(p['a'][0], 2), round(p['a'][1], 2), round(p['b'][0], 2), round(p['b'][1], 2), p['m']))
        elif p['t'] == 'arc':
            out.append(('a', round(p['pts'][0][0], 2), round(p['pts'][0][1], 2), round(p['pts'][-1][0], 2), round(p['pts'][-1][1], 2), p['m']))
        else:
            out.append(('p', round(p['h_from'], 2), round(p['h_to'], 2)))
    return tuple(out)


# 2026-11-13 ユーザー: 切り上げに特別な道を作らず、本線の最後の周の道でガレージへ向かう。周ごとに、最後の周と同じ経路を走っている
# 区間(赤の前のつなぎ → 赤 → つなぎ → 青 → つなぎ → 黄)を前から比べ、分かれる区間の始めで判断する(印 lapN-end)。切り上げると
# 決めたら、その区間から先は最後の周と同じ(黄も最後の周と同じ通り方で抜けて、その周を完了してからガレージへ)。
FINAL_TAIL = None   # (区間の番号 J, 切り上げられる周のリスト, 最後の周のその区間の番号)


def _first(bi):
    """区間 bi の始まりの車軸の位置。空の区間(つなぎなし)は、次の区間の始まり。"""
    for k in range(bi, len(blocks)):
        for p_ in blocks[k]['prims']:
            sp_ = _start_point(p_)
            if sp_ is not None:
                return (round(sp_[0], 2), round(sp_[1], 2))
    return None
if garage:
    groups = {}
    for bi, b in enumerate(blocks):
        if b['kind'] == 'visit' and b['lap'] >= 1:
            g_ = groups.setdefault(b['lap'], [])
            if b['color'] == 'red' and bi > 0 and blocks[bi - 1]['kind'] == 'conn':
                g_.append(bi - 1)
            g_.append(bi)
            if b['color'] in ('red', 'blue') and bi + 1 < len(blocks) and blocks[bi + 1]['kind'] == 'conn':
                g_.append(bi + 1)
    F = max(groups) if groups else 0
    if F >= 2 and len(groups[F]) == 6:
        fs = [_sig(blocks[i]) for i in groups[F]]
        jn = {}
        for n in range(1, F):
            gn = groups.get(n, [])
            if len(gn) != 6:
                continue
            # 周の始め(赤の前のつなぎの始め)が最後の周と同じ位置か。違えば(1周目はスタートから始まる)、その周は切り上げられない
            start_same = _first(gn[0]) == _first(groups[F][0])
            # 赤の前のつなぎは、1周目だけスタートから始まるので違うことが多い。その場合は赤から先を比べる(分かれ目は赤より後)
            j = 0 if start_same else 1
            if not start_same and _first(gn[1]) != _first(groups[F][1]):
                continue
            # 区間の形に加えて、始まりの位置も同じ(空のつなぎどうしは形が同じでも、前後の位置が違うことがある)
            while j < 6 and _sig(blocks[gn[j]]) == fs[j] and _first(gn[j]) == _first(groups[F][j]):
                j += 1
            if not start_same and j < 1:
                continue
            jn[n] = j   # 0..6: 前から j 区間が最後の周と同じ(6 なら1周まるごと同じ)。赤の前が違う周は1以上
        best = None
        for J in range(0, 7):
            cov = [n for n, j in jn.items() if j >= J and (J >= 1 or _first(groups[n][0]) == _first(groups[F][0]))
                   and (J >= 6 or _first(groups[n][J]) == _first(groups[F][J]))]
            if cov and (best is None or (len(cov), J) > (len(best[1]), best[0])):
                best = (J, sorted(cov))
        if best is not None:
            FINAL_TAIL = best
for bi, b in enumerate(blocks):
    label = f"lap{b['lap']}-{b['color']}" if b['kind'] == 'visit' and b['lap'] >= 1 else None   # lap0 は赤の前に通る黄
    prims = b['prims']
    if garage and ARM_DOWN is not None and bi == len(blocks) - 1 and b['kind'] == 'conn':
        prims = _split_prims(prims, float(ARM_DOWN['cm']))
    # 2026-11-11: 途中で切り上げてガレージへ向かえる周の終わり(N周目の黄ゲートのあとの、赤へのつなぎの始め)に印 'lapN-end'
    mark = None
    if FINAL_TAIL is not None:
        J, cov = FINAL_TAIL
        for n in cov:
            gn = groups[n]
            # J == 6: 1周まるごと同じ → その周の黄のあとのつなぎ(次の周の赤の前、または最後のつなぎ)の始め
            at = gn[J] if J < 6 else gn[5] + 1
            if bi == at:
                mark = 'lap%d-end' % n
    elif (EXIT and b['kind'] == 'conn' and bi > 0 and blocks[bi - 1]['kind'] == 'visit' and blocks[bi - 1]['color'] == 'yellow'
            and blocks[bi - 1]['lap'] in EXIT.get('laps', []) and bi < len(blocks) - 1):
        mark = 'lap%d-end' % blocks[bi - 1]['lap']
    # 2026-11-13: 黄を抜けてそのまま赤へ一直線に入る配置(33)では、黄→赤のつなぎが空で印が消えていた
    # (ロボットが時間で切り上げられず全部の周を走る)。空のときは次の区間の最初へ持ち越す。
    if carry_mark and not mark:
        mark = carry_mark
    carry_mark = mark if (mark and not prims) else None
    for k, p in enumerate(prims):
        seq.append((p, label, mark if k == 0 else None))
steps, h, max_gap = prims_to_steps(seq, START_H0)
steps = smooth_corners(steps)
steps = finalize(steps, True)
exit_steps = None
if FINAL_TAIL is not None:
    J, cov = FINAL_TAIL
    start_b = groups[F][J] if J < 6 else groups[F][5] + 1
    ep = []
    for bi in range(start_b, len(blocks)):
        pr = blocks[bi]['prims']
        if ARM_DOWN is not None and bi == len(blocks) - 1 and blocks[bi]['kind'] == 'conn':
            pr = _split_prims(pr, float(ARM_DOWN['cm']))
        ep += list(pr)
    exit_steps, _, _ = prims_to_steps([(p, None, None) for p in ep], START_H0)
    exit_steps = finalize(smooth_corners(exit_steps), False)
    EXIT = dict(laps=cov, kind='final_tail', switch_block=J, cost=None)
elif EXIT and EXIT.get('prims'):
    ep = EXIT['prims']
    if EXIT.get('arm_down') is not None:
        ep = _split_prims(ep, float(EXIT['arm_down']['cm']))
    exit_steps, _, _ = prims_to_steps([(p, None, None) for p in ep], START_H0)
    exit_steps = finalize(exit_steps, False)
    # 2026-11-13: 黄を後退で戻る切り上げでは、後退で黄を通っている間はアームを動かさない(下げ始めは後退を終えてから)
    if exit_steps and exit_steps[0].get('reverse') and exit_steps[0].get('label') == 'garage-arm-down':
        exit_steps[0]['label'] = None
        nxt = next((st for st in exit_steps[1:] if st['type'] in ('move', 'arc')), None)
        if nxt is not None:
            nxt['label'] = 'garage-arm-down'
# 2026-11-11(ユーザー): ガレージで終わる計画では、最後の黄ゲートを通ったあとの最後の長い直進(150mm以上)に印を付ける。
# ロボットはこの直進からアームを下げ始める(下げたアームの分、ガレージエリアにかかる所が増える)。
# 下げたアームは車体の前に約5cm出る(経路を作る道具は、アームを下げた状態の当たり判定をしていない)ので、下げ始めるのは
# 車軸がスタート側の列の点から ARM_DOWN_X_CM より外に出てから(アームの先がどの脚の長方形よりも外)。そこで直進を2つに分ける。
ARM_DOWN_X_CM = 4 * 24.64 + 10.0
if garage and ARM_DOWN is None:
    # 経路のファイルに arm_down がない(古い道具)ときだけ、以前の決め方(フィールドを出てから)
    last_lap = max((i for i, st in enumerate(steps) if (st.get('label') or '').startswith('lap')), default=-1)
    # 計画をたどって、各ステップの始めの車軸の位置(経路を作る道具の座標 cm、コースの向きは 180 - ジャイロの目標)
    px, py, ph = 113.46, 86.24, START_H0
    starts = []
    for st in steps:
        starts.append((px, py, ph))
        if st['type'] == 'turn':
            ph = START_H0 - st['target_heading_deg']
        elif st['type'] == 'move':
            hh = START_H0 - st['target_heading_deg']; th = math.radians(hh + (180 if st.get('reverse') else 0))
            px += st['distance_mm'] / 10 * math.cos(th); py += st['distance_mm'] / 10 * math.sin(th); ph = hh
        else:
            sw = -st['sweep_deg']; r = st['radius_mm'] / 10; sg = 1.0 if sw > 0 else -1.0
            th0 = math.radians(ph + (180 if st.get('reverse') else 0))
            cx, cy = px - sg * r * math.sin(th0), py + sg * r * math.cos(th0); th1 = th0 + math.radians(sw)
            px, py = cx + sg * r * math.sin(th1), cy - sg * r * math.cos(th1); ph += sw
    cand = [i for i, st in enumerate(steps) if i > last_lap and st['type'] == 'move' and st.get('distance_mm', 0) >= 150.0
            and not st.get('reverse')]
    if cand:
        i = cand[-1]; st = steps[i]
        x0, y0, _ = starts[i]
        th = math.radians(START_H0 - st['target_heading_deg'])
        L = st['distance_mm'] / 10
        if x0 >= ARM_DOWN_X_CM:
            st['label'] = 'garage-arm-down'
        elif math.cos(th) > 0.3:
            k = (ARM_DOWN_X_CM - x0) / math.cos(th)
            if k < L - 0.5:
                first = dict(st, distance_mm=round(k * 10, 1), stop=False, label=None)
                second = dict(st, distance_mm=round((L - k) * 10, 1), label='garage-arm-down')
                steps[i:i + 1] = [first, second]
# 同じラベルが続く(往復など)ときは最初の1つだけ残す
seen = set()
for st in steps:
    if st.get('label') and st['label'].startswith('lap'):
        if st['label'] in seen:
            st['label'] = None
        seen.add(st['label'])
# 2026-11-14(ユーザー: 狭い直進の揺れを減らす): 脚とのすき間が NARROW_CM 未満の直進に、出力の上限 NARROW_POWER を書く
# (出力100に張り付くと、向きを直すとき遅い側を速くできず振れやすい)。すき間は検査(check_plan.py)と同じ当たり判定で測る。
NARROW_CM = 2.5
NARROW_POWER = 92


def mark_narrow(steps_, start_pose):
    import os, tempfile
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import check_plan as cpk
    import numpy as np
    tmp = tempfile.NamedTemporaryFile('w', suffix='.json', delete=False, encoding='utf-8')
    json.dump(c.get('params', {}), tmp); tmp.close()
    try:
        hr = cpk.load_tool(gates, tmp.name)
    finally:
        os.unlink(tmp.name)
    pts = cpk.sample(steps_, start_pose, pivot_back_cm=PIVOT_BACK)
    P = np.array([(q[0], q[1]) for q in pts]); H = np.radians([q[2] for q in pts]); I = np.array([q[3] for q in pts])
    hr._RO = float(c.get('rear_open') or 0.0)   # 2026-11-14: 狭い配置で後ろの端の部分は支柱だけ(hybrid_route の rear_open_cm)
    cl = hr.body_clear(P[:, 0], P[:, 1], H) if hasattr(hr, 'body_clear') else hr._rect_clear(P[:, 0], P[:, 1], H, -hr.REAR, hr.FRONT)
    n = 0
    for k, st in enumerate(steps_):
        if st['type'] != 'move' or st.get('distance_mm', 0) < 50.0:
            continue
        m = I == k
        if m.any() and float(cl[m].min()) < NARROW_CM:
            st['max_power'] = NARROW_POWER; n += 1
    return n


if garage and '--no-narrow' not in sys.argv:
    _first = next(p for b in blocks for p in b['prims'])
    _sp = _start_point(_first)
    n_narrow = mark_narrow(steps, (_sp[0], _sp[1], START_H0))
    if exit_steps:
        _ep = ep[0]
        _h = (math.degrees(math.atan2(_ep['b'][1] - _ep['a'][1], _ep['b'][0] - _ep['a'][0])) + (0 if _ep['m'] > 0 else 180)) if _ep['t'] == 'line' else ((_ep['th0'] + (0 if _ep['m'] > 0 else 180)) if _ep['t'] == 'arc' else _ep['h_from'])
        _s = _start_point(_ep)
        mark_narrow(exit_steps, (_s[0], _s[1], _h))
    print('narrow moves (clearance < %.1fcm, power cap %d): %d' % (NARROW_CM, NARROW_POWER, n_narrow))
dist = sum(s.get('distance_mm', 0) if s['type'] == 'move' else s['radius_mm'] * math.radians(abs(s['sweep_deg'])) if s['type'] == 'arc' else 0 for s in steps)
plan = {
    'gates': {'mode': 'random', 'seed': gates.get('seed'), 'course': 'right', **{k: gates[k] for k in ('red', 'blue', 'yellow')}},
    'heading_convention': 'target_heading_deg is the GYRO target for course=right file-mode execution (gyro = -planner angle), same as plan_seed9392783_right_1031b.json. arc: target_heading_deg is the end heading, sweep_deg the signed heading change in the same frame (start = end - sweep), radius_mm the radius of the robot-centre path.',
    'note_1101': '2026-11-01 試作: 往復・後退でのゲート通過を許した経路(4周)のつなぎ区間を、円弧+直進+円弧などに置き換えたもの(検討セッションのスクラッチパッドの shuttle_plan2.py / curve_route.py で作成)。近似式の見積もりで走行部分 %.1f 秒。' % c['total'],
    'laps': c.get('params', {}).get('laps', 4),
    'goal_at': 'garage' if garage else 'goal',
    'total_distance_mm': round(dist, 1),
    'move_count': sum(1 for s in steps if s['type'] == 'move'),
    'arc_count': sum(1 for s in steps if s['type'] == 'arc'),
    'turn_count': sum(1 for s in steps if s['type'] == 'turn'),
    'steps': steps,
}
if c.get('rear_open'):
    # 2026-11-14: 狭い配置で、後ろの端からこの長さ(cm)は脚の支柱とのすき間だけを見て作った(検査も同じ見方をする)
    plan['rear_open_cm'] = float(c['rear_open'])
if exit_steps:
    # 2026-11-11: 途中の周(印 lapN-end)で切り上げるときに、そこから走るガレージまでのつなぎ(--rally-time-budget)
    plan['exit'] = {'laps': EXIT.get('laps', []), 'steps': exit_steps, 'tool_cost_sec': EXIT.get('cost'),
                    'kind': EXIT.get('kind'), 'switch_block': EXIT.get('switch_block')}
# 2026-11-06: アームを下げたままだと脚に当たる最初の点(hybrid_route.py の arm、なければ書かない)。
# ロボットは、ここまでに上げ終わる見込みなら走りながら上げ、間に合わなければスタートで止まっている間に上げる。
if c.get('arm') is not None or 'arm' in c:
    arm = c.get('arm')
    plan['et_rally_arm'] = {
        'must_be_up_mm': None if arm is None else round(arm['cm'] * 10, 1),
        'must_be_up_sec': None if arm is None else arm['sec'],
        'arm_front_cm': c['params'].get('arm_front_cm'), 'arm_half_width_cm': c['params'].get('arm_half_width_cm'),
        'note': 'must_be_up_* が null なら最後まで当たらない。時間は経路を作る道具の近似式(止まる時間を入れない短めの見込み)。',
    }
json.dump(plan, open(sys.argv[3], 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
print('steps', len(steps), 'moves', plan['move_count'], 'arcs', plan['arc_count'], 'turns', plan['turn_count'],
      'dist', plan['total_distance_mm'], 'max heading gap', round(max_gap, 3),
      'exit', ('%d steps from laps %s' % (len(exit_steps), EXIT.get('laps'))) if exit_steps else 'none')
