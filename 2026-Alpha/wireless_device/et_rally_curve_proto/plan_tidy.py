"""2026-11-17(ユーザー): 計画の手直し。経路の道具が作った計画の、同じ直線上の「後退→少し前進→また後退」(前進→少し後退→前進も)を、
前後の切り替えなしの1本にまとめる。

道具はゲートを1つ通るごとに「入りの直進→通過→抜けの直進」を別々に作るので、平行に隣の列に並んだゲートを続けて
同じ向きで通るとき(例: 配置 r3040-b4243-y3141 の黄→赤、間隔24.64cm)、前のゲートの抜けの終わりが次のゲートの入りの
始まりより先まで行き、その分を一度逆向きに戻ってから通っていた(1周に2回の前後の切り替え)。
真ん中の逆向きの区間がもとの向きと同じ直線上(向きのずれ max_dh 以内)で、次の区間がそれより長いときだけまとめる。
全4872配置で3配置だけ。
"""
import math


def _len(s):
    if s.get('distance_mm') is not None:
        return float(s['distance_mm'])
    return float(s.get('radius_mm', 0.0)) * math.radians(abs(float(s.get('sweep_deg', 0.0))))


def _dh(a, b):
    return abs(((float(a) - float(b) + 180.0) % 360.0) - 180.0)


def _arc_like(base, target, sweep, R, rev, stop, label, drop_marks=False):
    """base の直進の設定(max_power・marks など)を引き継いだ円弧。"""
    a = {k: v for k, v in base.items() if k not in ('distance_mm',)}
    if drop_marks:
        a.pop('marks', None)
    a.update(type='arc', target_heading_deg=round(target, 4), sweep_deg=round(sweep, 4), radius_mm=round(R, 1),
             reverse=rev, stop=stop, label=label)
    return a


def _integrate(steps):
    """steps を (0,0) 向き(コースの向き)h0 から積算した終わりの (x, y, h)。check_plan.sample と同じ式(mm)。"""
    x = y = 0.0
    h = None
    for s in steps:
        hh = 180.0 - float(s['target_heading_deg'])
        if s['type'] == 'move':
            if h is None:
                h = hh
            th = math.radians(hh + (180.0 if s.get('reverse') else 0.0))
            L = float(s['distance_mm'])
            x, y, h = x + L * math.cos(th), y + L * math.sin(th), hh
        elif s['type'] == 'arc':
            sw = -float(s['sweep_deg'])
            if h is None:
                h = hh - sw
            r = float(s['radius_mm']); sg = 1.0 if sw > 0 else -1.0
            th0 = math.radians(h + (180.0 if s.get('reverse') else 0.0))
            cx, cy = x - sg * r * math.sin(th0), y + sg * r * math.cos(th0)
            th1 = th0 + math.radians(sw)
            x, y, h = cx + sg * r * math.sin(th1), cy - sg * r * math.cos(th1), h + sw
        else:
            raise ValueError('turn in the middle chunk')
    return x, y


def _straight_to_same_end(chunk, nxt):
    """chunk のあと nxt(直進)で着く位置へ、nxt と同じ向き・同じ前後の向きで行く steps(リスト)を返す。
    横のずれ(1〜2mm)は、ごく緩いS字(同じ半径の円弧2つ、向きは元に戻る)で寄せる。向きを変えずに終わるので、
    そのあとの円弧の始まりの向きも元と同じ(向きを変えた直進にすると、後ろの円弧がずれて周ごとにたまった)。"""
    cx, cy = _integrate(chunk)
    H = float(nxt['target_heading_deg'])
    hh = 180.0 - H
    rev = bool(nxt.get('reverse'))
    th = math.radians(hh + (180.0 if rev else 0.0))
    L = float(nxt['distance_mm'])
    wx, wy = cx + L * math.cos(th), cy + L * math.sin(th)
    ux, uy = math.cos(th), math.sin(th)
    along = wx * ux + wy * uy
    lat = -wx * uy + wy * ux
    if abs(lat) < 0.2:
        return [dict(nxt, distance_mm=round(along, 1))]
    eps = 2.0 * math.atan2(abs(lat), along)
    R = along / (2.0 * math.sin(eps))
    for sg in (1.0, -1.0):
        d = math.degrees(eps) * sg
        a1 = _arc_like(nxt, H + d, d, R, rev, stop=False, label=None, drop_marks=True)
        a2 = _arc_like(nxt, H, -d, R, rev, stop=nxt.get('stop', False), label=nxt.get('label'), drop_marks=True)
        ex, ey = _integrate([dict(nxt, distance_mm=0.0), a1, a2])
        if math.hypot(ex - wx, ey - wy) < 0.5:
            return [a1, a2]
    raise ValueError('could not build the S curve')


def _straight_to_same_end_from(first, chunk, nxt):
    """first(直進)の始まりから、first → chunk → nxt で着く位置へ、同じ向き・同じ前後の向きで行く steps。
    横のずれは first+nxt の全体の長さにわたる緩いS字で寄せる。first の印(label)は最初の区間に残す。"""
    if first['type'] != 'move' or nxt['type'] != 'move':
        return None
    total = [first] + list(chunk) + [nxt]
    # first の始まりを原点に積算する(_integrate は最初の区間の向きを起点にする)
    ex, ey = _integrate(total)
    H = float(nxt['target_heading_deg'])
    hh = 180.0 - H
    rev = bool(nxt.get('reverse'))
    th = math.radians(hh + (180.0 if rev else 0.0))
    ux, uy = math.cos(th), math.sin(th)
    along = ex * ux + ey * uy
    lat = -ex * uy + ey * ux
    if along <= 0:
        return None
    if abs(lat) < 0.2:
        return [dict(nxt, distance_mm=round(along, 1), label=nxt.get('label') or first.get('label'), stop=nxt.get('stop', False))]
    eps = 2.0 * math.atan2(abs(lat), along)
    R = along / (2.0 * math.sin(eps))
    for sg in (1.0, -1.0):
        d = math.degrees(eps) * sg
        a1 = _arc_like(first, H + d, d, R, rev, stop=False, label=first.get('label'))
        a2 = _arc_like(nxt, H, -d, R, rev, stop=nxt.get('stop', False), label=nxt.get('label'))
        fx, fy = _integrate([dict(nxt, distance_mm=0.0), a1, a2])
        if math.hypot(fx - ex, fy - ey) < 0.5:
            return [a1, a2]
    return None


def merge_back_and_forth(steps, max_back_mm=250.0, max_dh=5.0):
    """戻り値: (新しい steps, まとめた数)。"""
    out, i, n, merged = [], 0, len(steps), 0
    while i < n:
        s = steps[i]
        if s['type'] == 'move':
            d0 = bool(s.get('reverse')); H = s['target_heading_deg']
            j, back, ok, labels = i + 1, 0.0, True, []
            while j < n and steps[j]['type'] in ('move', 'arc') and bool(steps[j].get('reverse')) != d0:
                if _dh(steps[j]['target_heading_deg'], H) > max_dh:
                    ok = False
                    break
                back += _len(steps[j])
                if steps[j].get('label'):
                    labels.append(steps[j]['label'])
                j += 1
            if (ok and j > i + 1 and j < n and steps[j]['type'] == 'move' and bool(steps[j].get('reverse')) == d0
                    and _dh(steps[j]['target_heading_deg'], H) <= max_dh and back <= max_back_mm
                    and float(steps[j]['distance_mm']) > back and not labels
                    # 周の区切りの印(marks、切り上げの判断の場所)が真ん中や次の区間にあると、まとめると場所が変わるのでまとめない
                    and not any(st.get('marks') for st in steps[i + 1:j + 1])):
                # 真ん中の区間で動いた分(横の小さな寄せも含む)を足した終わりの位置へ、まっすぐ1本で行く
                # (向きを少しだけ変える)。終わりの位置は元と同じになる(横の寄せを捨てると周ごとにたまる)。
                if _dh(s['target_heading_deg'], steps[j]['target_heading_deg']) < 0.5:
                    # 2026-11-17: 前の直進から まとめて1本にする(横の寄せを長い距離に薄く広げ、急なS字にしない)
                    whole = _straight_to_same_end_from(s, steps[i + 1:j], steps[j])
                    if whole is not None:
                        out.extend(whole)
                        merged += 1
                        i = j + 1
                        continue
                nxt = _straight_to_same_end(steps[i + 1:j], steps[j])
                # もとの計画では、この直進のあと向きを切り替えるので「止まる」(stop)が付いている。続けて走るので外す
                # (11/17 実機: 付いたままで、黄と赤の間で減速して止まっていた)。
                out.append(dict(s, stop=False))
                out.extend(nxt)
                merged += 1
                i = j + 1
                continue
        out.append(s)
        i += 1
    return out, merged


def tidy_plan(plan):
    """計画(dict)の steps と exit の steps をまとめる。戻り値: (計画, まとめた数)。"""
    p = dict(plan)
    p['steps'], m = merge_back_and_forth(plan['steps'])
    ex = plan.get('exit')
    if isinstance(ex, dict) and ex.get('steps'):
        p['exit'] = dict(ex)
        p['exit']['steps'], m2 = merge_back_and_forth(ex['steps'])
        m += m2
    if m:
        p['note_tidy_1117'] = 'plan_tidy: 同じ直線上の前後の切り替え %d か所をまとめた' % m
    return p, m
