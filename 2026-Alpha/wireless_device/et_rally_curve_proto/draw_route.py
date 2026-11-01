"""route.json(shuttle_plan.pyの出力)と、元のplan JSONの経路をSVGにする。"""
import json, math, sys

P = 24.64
plan = json.load(open(sys.argv[1], encoding='utf-8'))
route = json.load(open(sys.argv[2], encoding='utf-8'))
out = sys.argv[3]

X0, X1, Y0, Y1 = -36, 132, -40, 140
S = 3.2  # px/cm


def tx(x):
    return (x - X0) * S


def ty(y):
    return (Y1 - y) * S


COL = {'red': '#d64545', 'blue': '#3b6fd6', 'yellow': '#d6a21e'}


def base_svg():
    parts = []
    W, H = (X1 - X0) * S, (Y1 - Y0) * S
    parts.append(f'<svg viewBox="0 0 {W:.0f} {H:.0f}" xmlns="http://www.w3.org/2000/svg" class="map">')
    # 進入禁止
    mx = lambda x: 4 * P - x
    keep = [(-1e3, mx(-17.0), -1e3, -37.0), (-1e3, mx(4 * P + 17.8 + 14.0), -1e3, 1e3), (mx(-28.0), 1e3, 0.0, 1e3)]
    for x0, x1, y0, y1 in keep:
        x0, x1 = max(x0, X0), min(x1, X1); y0, y1 = max(y0, Y0), min(y1, Y1)
        parts.append(f'<rect x="{tx(x0):.1f}" y="{ty(y1):.1f}" width="{(x1-x0)*S:.1f}" height="{(y1-y0)*S:.1f}" class="keep"/>')
    for i in range(5):
        for j in range(5):
            parts.append(f'<circle cx="{tx(i*P):.1f}" cy="{ty(j*P):.1f}" r="{2.2*S:.1f}" class="dot"/>')
    for c in ('red', 'blue', 'yellow'):
        a = plan['gates'][c]['a']; b = plan['gates'][c]['b']
        pa = (a[0] * P, a[1] * P); pb = (b[0] * P, b[1] * P)
        dx, dy = pb[0] - pa[0], pb[1] - pa[1]; L = math.hypot(dx, dy); n = (-dy / L, dx / L)
        parts.append(f'<line x1="{tx(pa[0]):.1f}" y1="{ty(pa[1]):.1f}" x2="{tx(pb[0]):.1f}" y2="{ty(pb[1]):.1f}" stroke="{COL[c]}" stroke-width="2" stroke-dasharray="4 3" opacity=".7"/>')
        for p in (pa, pb):
            parts.append(f'<line x1="{tx(p[0]-n[0]*4):.1f}" y1="{ty(p[1]-n[1]*4):.1f}" x2="{tx(p[0]+n[0]*4):.1f}" y2="{ty(p[1]+n[1]*4):.1f}" stroke="{COL[c]}" stroke-width="{2*S:.1f}" stroke-linecap="round"/>')
    sx, sy = 4 * P - (4 * P + 17.8), 3.5 * P
    gx, gy = 4 * P + 14.9, 3.5 * P
    parts.append(f'<circle cx="{tx(sx):.1f}" cy="{ty(sy):.1f}" r="7" class="start"/><text x="{tx(sx)-10:.1f}" y="{ty(sy)-12:.1f}" class="lbl">S</text>')
    parts.append(f'<rect x="{tx(gx)-7:.1f}" y="{ty(gy)-7:.1f}" width="14" height="14" class="goal"/><text x="{tx(gx)-4:.1f}" y="{ty(gy)-12:.1f}" class="lbl">G</text>')
    return parts


def legs_svg(legs, cls):
    s = []
    for a, b, d in legs:
        s.append(f'<line x1="{tx(a[0]):.1f}" y1="{ty(a[1]):.1f}" x2="{tx(b[0]):.1f}" y2="{ty(b[1]):.1f}" class="{cls} {"fwd" if d > 0 else "rev"}" marker-end="url(#arr{"F" if d > 0 else "R"})"/>')
    return s


MARK = ('<defs><marker id="arrF" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="5" markerHeight="5" orient="auto"><path d="M0,0L10,5L0,10z" class="mf"/></marker>'
        '<marker id="arrR" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="5" markerHeight="5" orient="auto"><path d="M0,0L10,5L0,10z" class="mr"/></marker></defs>')


def old_route_legs():
    # 1031bのステップから軸の軌跡を復元(gyro = -planner角)
    x, y = 4 * P - (4 * P + 17.8), 3.5 * P
    legs = []
    for s in plan['steps']:
        if s['type'] == 'move':
            h = math.radians(-s['target_heading_deg'])
            nx, ny = x + s['distance_mm'] / 10 * math.cos(h), y + s['distance_mm'] / 10 * math.sin(h)
            legs.append(([x, y], [nx, ny], 1)); x, y = nx, ny
    return legs


# 新しい経路: 周回ごとに1枚
items = route['items']
panels = []
# 区切り: 各周 = conn+visit を3組。最初の周の前(スタート)と最後のゴールは端の周に入れる
groups = []
cur = []
lap_of = []
for it in items:
    cur.append(it)
    if it['kind'] == 'visit' and it['color'] == 'yellow':
        groups.append(cur); cur = []
groups[-1].extend(cur)

svgs = []
for gi, g in enumerate(groups):
    p = base_svg(); p.insert(1, MARK)
    for it in g:
        p += legs_svg(it['legs'], 'conn' if it['kind'] == 'conn' else 'pass')
    p.append('</svg>')
    svgs.append('\n'.join(p))
p = base_svg(); p.insert(1, MARK); p += legs_svg(old_route_legs(), 'old'); p.append('</svg>')
old_svg = '\n'.join(p)
json.dump(dict(new=svgs, old=old_svg), open(out, 'w', encoding='utf-8'))
print('ok', len(svgs))
