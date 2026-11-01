"""区間ごとに、計画上の終わりの位置と、ログのet_locの差(mm、et_locの座標)を出す。
usage: seg_err.py plan.json log...
et_loc = (-(planner y - start y), planner x - start x) [mm](右コース)。ゴールで+yを向くので、et_xの+は「後ろ」。
"""
import json, math, re, sys

P = 24.64
plan = json.load(open(sys.argv[1], encoding='utf-8'))['steps']
x0, y0 = -17.8, 3.5 * P
x, y, h = x0, y0, 0.0
pts = []
for s in plan:
    if s['type'] == 'turn':
        h = -s['target_heading_deg']
        continue
    rev = s.get('reverse', False)
    if s['type'] == 'move':
        hh = -s['target_heading_deg']
        th = math.radians(hh + (180 if rev else 0))
        L = s['distance_mm'] / 10
        x += L * math.cos(th); y += L * math.sin(th); h = hh
    else:
        sw = -s['sweep_deg']; r = s['radius_mm'] / 10
        th0 = math.radians(h + (180 if rev else 0)); sg = 1 if sw > 0 else -1
        cx, cy = x - sg * r * math.sin(th0), y + sg * r * math.cos(th0)
        th1 = th0 + math.radians(sw)
        x, y = cx + sg * r * math.sin(th1), cy - sg * r * math.cos(th1)
        h += sw
    pts.append((-(y - y0) * 10, (x - x0) * 10))

rows = {}
for f in sys.argv[2:]:
    segs = [l for l in open(f, encoding='utf-8') if 'EtRallyPathSegment.' in l and 'summary' in l]
    for i, l in enumerate(segs):
        m = re.search(r'et_loc=\(([-\d.]+),([-\d.]+)\)', l)
        name = re.search(r'et_rally (\w+)', l).group(1)
        e = (float(m.group(1)) - pts[i][0], float(m.group(2)) - pts[i][1])
        rows.setdefault(i, [name]).append('(%+6.1f,%+6.1f)' % e)
print('%-8s' % 'seg', ' '.join('%17s' % f.split('T')[1][:6] for f in sys.argv[2:]))
for i, r in rows.items():
    print('%-8s' % r[0], ' '.join('%17s' % c for c in r[1:]))
