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


def prims_to_steps(seq, h):
    """seq: [(prim, label, mark)]。mark(周の終わりの印など)は、そのプリミティブから最初にできたステップに付ける
    (できなければ次にできたステップへ)。戻り値 (steps, h, max_gap)。"""
    out = []; gap = 0.0; pending = None
    for p, label, mark in seq:
        if mark:
            pending = mark
        n0 = len(out)
        if p['t'] == 'pivot':
            if p['deg'] >= 0.3:
                gap = max(gap, abs(nd(p['h_from'] - h)))
                out.append({'type': 'turn', 'target_heading_deg': g(p['h_to']), 'label': None})
            h = p['h_to']
        elif p['t'] == 'line':
            a, e = p['a'], p['b']
            L = math.hypot(e[0] - a[0], e[1] - a[1]) * 10
            if L >= 0.5:
                hh = math.degrees(math.atan2(e[1] - a[1], e[0] - a[0])) + (0 if p['m'] > 0 else 180)
                gap = max(gap, abs(nd(hh - h)))
                out.append({'type': 'move', 'target_heading_deg': g(hh), 'distance_mm': round(L, 1), 'reverse': p['m'] < 0,
                            'stop': False, 'label': 'garage-arm-down' if p.get('arm_down') else label})
                h = hh
        else:
            if p['deg'] >= 0.05:
                h0 = p['th0'] + (0 if p['m'] > 0 else 180)
                gap = max(gap, abs(nd(h0 - h)))
                h1 = h0 + p['s'] * p['deg']
                out.append({'type': 'arc', 'target_heading_deg': g(h1), 'sweep_deg': round(-p['s'] * p['deg'], 2),
                            'radius_mm': round(p['r'] * 10, 1), 'reverse': p['m'] < 0, 'stop': False,
                            'label': 'garage-arm-down' if p.get('arm_down') else None})
                h = h1
        if pending and len(out) > n0:
            out[n0]['marks'] = [pending]; pending = None
    return out, h, gap


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
for bi, b in enumerate(blocks):
    label = f"lap{b['lap']}-{b['color']}" if b['kind'] == 'visit' and b['lap'] >= 1 else None   # lap0 は赤の前に通る黄
    prims = b['prims']
    if garage and ARM_DOWN is not None and bi == len(blocks) - 1 and b['kind'] == 'conn':
        prims = _split_prims(prims, float(ARM_DOWN['cm']))
    # 2026-11-11: 途中で切り上げてガレージへ向かえる周の終わり(N周目の黄ゲートのあとの、赤へのつなぎの始め)に印 'lapN-end'
    mark = None
    if (EXIT and b['kind'] == 'conn' and bi > 0 and blocks[bi - 1]['kind'] == 'visit' and blocks[bi - 1]['color'] == 'yellow'
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
steps = finalize(steps, True)
exit_steps = None
if EXIT and EXIT.get('prims'):
    ep = EXIT['prims']
    if EXIT.get('arm_down') is not None:
        ep = _split_prims(ep, float(EXIT['arm_down']['cm']))
    exit_steps, _, _ = prims_to_steps([(p, None, None) for p in ep], START_H0)
    exit_steps = finalize(exit_steps, False)
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
if exit_steps:
    # 2026-11-11: 途中の周(印 lapN-end)で切り上げるときに、そこから走るガレージまでのつなぎ(--rally-time-budget)
    plan['exit'] = {'laps': EXIT.get('laps', []), 'steps': exit_steps, 'tool_cost_sec': EXIT.get('cost')}
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
