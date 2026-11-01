"""curve_route.py の出力(c_*.json、右コースのplanner座標)を、実機の計画JSON(move/turn/arc)にする。
右コースのファイル実行では gyro目標 = -planner角(1031bと同じ規約)。
usage: to_plan.py c.json gates.json out_plan.json [laps]
"""
import json, math, sys

c = json.load(open(sys.argv[1]))
gates = json.load(open(sys.argv[2]))['gates']
nd = lambda a: (a + 180.0) % 360.0 - 180.0
g = lambda h: round(nd(-h), 2)

steps = []
h = 0.0          # planner frame robot heading
max_gap = 0.0
for b in c['blocks']:
    label = f"lap{b['lap']}-{b['color']}" if b['kind'] == 'visit' else None
    for p in b['prims']:
        if p['t'] == 'pivot':
            if p['deg'] >= 0.3:
                max_gap = max(max_gap, abs(nd(p['h_from'] - h)))
                steps.append({'type': 'turn', 'target_heading_deg': g(p['h_to']), 'label': None})
            h = p['h_to']
        elif p['t'] == 'line':
            a, e = p['a'], p['b']
            L = math.hypot(e[0] - a[0], e[1] - a[1]) * 10
            if L < 0.5:
                continue
            hh = math.degrees(math.atan2(e[1] - a[1], e[0] - a[0])) + (0 if p['m'] > 0 else 180)
            max_gap = max(max_gap, abs(nd(hh - h)))
            st = {'type': 'move', 'target_heading_deg': g(hh), 'distance_mm': round(L, 1), 'reverse': p['m'] < 0,
                  'stop': False, 'label': label}
            steps.append(st)
            h = hh
        else:
            if p['deg'] < 0.05:
                continue
            h0 = p['th0'] + (0 if p['m'] > 0 else 180)
            max_gap = max(max_gap, abs(nd(h0 - h)))
            h1 = h0 + p['s'] * p['deg']
            steps.append({'type': 'arc', 'target_heading_deg': g(h1), 'sweep_deg': round(-p['s'] * p['deg'], 2),
                          'radius_mm': round(p['r'] * 10, 1), 'reverse': p['m'] < 0, 'stop': False, 'label': None})
            h = h1
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
if fixed[-1]['type'] != 'turn' or fixed[-1]['label'] != 'arc-fix':
    fixed.append({'type': 'turn', 'target_heading_deg': g(90.0), 'label': 'goal'})
else:
    fixed[-1] = {'type': 'turn', 'target_heading_deg': g(90.0), 'label': 'goal'}
steps = fixed
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
    'laps': 4,
    'total_distance_mm': round(dist, 1),
    'move_count': sum(1 for s in steps if s['type'] == 'move'),
    'arc_count': sum(1 for s in steps if s['type'] == 'arc'),
    'turn_count': sum(1 for s in steps if s['type'] == 'turn'),
    'steps': steps,
}
json.dump(plan, open(sys.argv[3], 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
print('steps', len(steps), 'moves', plan['move_count'], 'arcs', plan['arc_count'], 'turns', plan['turn_count'],
      'dist', plan['total_distance_mm'], 'max heading gap', round(max_gap, 3))
