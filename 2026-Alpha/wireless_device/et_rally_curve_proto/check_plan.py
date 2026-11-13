"""2026-11-13: ロボットに渡す計画ファイル(robot_program/tests/plan_*.json)を、経路を作る道具とは別に検査する。

経路は計画ファイルのステップから積算し直す(to_plan の変換やロボット側の読み方の誤りも見つける)。当たり判定の形(脚・進入禁止・
ガレージ)は hybrid_route.py と同じものを使う。11/13 に実機で見つかった不具合(切り上げの道がコースの外へ出た、周の終わりの印が
消えていた)を机上で見つけるため。

検査すること(どれか1つでも満たさなければ NG):
  1. 本線: 進入禁止に入らない(はみ出しが KEEPOUT_TOL_CM 以内なら注意)、脚とのすき間が --min-clear 以上、ゲートを決まった順(赤→青→黄、同じ色の重複はよい)で通る、
     終わりがガレージに入る(x・y とも食い込み量が道具の決まり以上)。
  2. 周の終わりの印 lapN-end が、計画の exit.laps の周すべてに1つずつある(切り上げられない周がない)。
  3. 各周の印から exit を走った場合: 1 と同じ(ゲートの順は、その周の黄のあとから数える)。
  4. 本線にも exit にもアームを下げる印 garage-arm-down がある。下げたあとは、車体の前をアームの分だけ伸ばしてすき間を見る。
  5. 持ち時間(--budget 秒)での切り上げ: ロボットと同じ式(estimate_steps と余裕2秒)で、何周目で切り上げ、何秒でガレージに着くか。
     着く時刻が持ち時間を超えたら NG。見込みには --ratio(実機の速さ/見込み、既定0.93)を掛ける。すき間が基準より1mm 以内だけ足りないときは NG でなく注意(形の計算の細かさより小さい差)。
     その場の旋回は、道具と同じく車軸より PIVOT_BACK_CM 後ろを中心に回す。

usage: python check_plan.py plan.json [params.json] [--budget 70] [--min-clear 1.5] [--json]
       params.json の既定は同じフォルダの params_day.json(進入禁止・ガレージの決まりを読む)。
"""
import argparse
import json
import math
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
KEEPOUT_TOL_CM = 0.5       # 進入禁止へのはみ出しが、この深さ以内なら注意(NG にしない)。変換で車体の角が数mm 振れる分
START_OVERHEAD_SEC = 3.2   # スタート位置に着いてから走り出すまで(取り込み・ジャイロ)。11/13 の実機で約3.2秒


def load_tool(gates, params_path):
    """hybrid_route を、この配置のゲートと設定で読み込む(当たり判定の関数だけ使う)。"""
    tmp = tempfile.NamedTemporaryFile('w', suffix='.json', delete=False, encoding='utf-8')
    json.dump({'gates': gates}, tmp); tmp.close()
    os.environ['HR_PARAMS'] = params_path
    argv = sys.argv
    sys.argv = [os.path.join(HERE, 'hybrid_route.py'), tmp.name, os.devnull]
    sys.path.insert(0, HERE)
    try:
        import importlib
        if 'hybrid_route' in sys.modules:
            hr = importlib.reload(sys.modules['hybrid_route'])
        else:
            import hybrid_route as hr
    finally:
        sys.argv = argv
        os.unlink(tmp.name)
    return hr


def sample(steps, start, step_cm=0.5, pivot_back_cm=0.0):
    """計画のステップを積算して、車軸の点 (x, y, 向き度, ステップの番号) を step_cm ごとに返す(コースの座標 cm)。
    向きの決まりは et_rally_start_pose._integrate と同じ(コースの向き = 180 − target_heading_deg)。"""
    x, y, h = start
    out = [(x, y, h, -1)]
    for i, s in enumerate(steps):
        t = s['type']
        if t == 'turn':
            # その場の旋回: 実測では回る中心が車軸より pivot_back_cm 後ろ(経路を作る道具と同じ形。車軸も少し動く)
            h1 = 180.0 - float(s['target_heading_deg'])
            d = (h1 - h + 180.0) % 360.0 - 180.0
            n = max(1, int(abs(d) / 2.0))
            px_, py_ = x - pivot_back_cm * math.cos(math.radians(h)), y - pivot_back_cm * math.sin(math.radians(h))
            for k in range(1, n + 1):
                hk = h + d * k / n
                out.append((px_ + pivot_back_cm * math.cos(math.radians(hk)), py_ + pivot_back_cm * math.sin(math.radians(hk)), hk, i))
            x, y = out[-1][0], out[-1][1]
            h = h + d
        elif t == 'move':
            hh = 180.0 - float(s['target_heading_deg'])
            th = math.radians(hh + (180.0 if s.get('reverse') else 0.0))
            L = float(s['distance_mm']) / 10.0
            n = max(1, int(L / step_cm))
            for k in range(1, n + 1):
                out.append((x + L * k / n * math.cos(th), y + L * k / n * math.sin(th), hh, i))
            x, y, h = x + L * math.cos(th), y + L * math.sin(th), hh
        elif t == 'arc':
            sw = -float(s['sweep_deg']); r = float(s['radius_mm']) / 10.0; sg = 1.0 if sw > 0 else -1.0
            th0 = math.radians(h + (180.0 if s.get('reverse') else 0.0))
            cx, cy = x - sg * r * math.sin(th0), y + sg * r * math.cos(th0)
            n = max(2, int(r * math.radians(abs(sw)) / step_cm), int(abs(sw)))
            for k in range(1, n + 1):
                th1 = th0 + math.radians(sw) * k / n
                out.append((cx + sg * r * math.sin(th1), cy - sg * r * math.cos(th1), h + sw * k / n, i))
            x, y, h = out[-1][0], out[-1][1], h + sw
        else:
            raise ValueError('unknown step type %r' % t)
    return out


def _cross(p, q, a, b):
    """線分 pq と ab が交わるか(端を含まない)。"""
    def o(u, v, w):
        return (v[0] - u[0]) * (w[1] - u[1]) - (v[1] - u[1]) * (w[0] - u[0])
    return o(p, q, a) * o(p, q, b) < 0 and o(a, b, p) * o(a, b, q) < 0


def gate_crossings(hr, pts):
    """車軸の軌跡が、各ゲートの2本の脚を結ぶ線を横切った順(色のリスト)。"""
    out = []
    for (x0, y0, _h0, _i0), (x1, y1, _h1, _i1) in zip(pts, pts[1:]):
        for c, g in hr.GATES.items():
            if _cross((x0, y0), (x1, y1), g.pa, g.pb):
                out.append(c)
    return out


ORDER = {'red': 'blue', 'blue': 'yellow', 'yellow': 'red'}


def order_ok(seq, prev=None):
    """ゲートの順: 同じ色の続けての通過はよい。それ以外は 赤→青→黄→赤。
    最初に赤をくぐるまでは、ほかの色をくぐってもよい(経路を作る道具の start_any_gate と同じ決まり。色の順番は赤から数える)。"""
    bad = []
    for i, c in enumerate(seq):
        if prev is None:
            if c != 'red':
                continue
        elif c != prev and c != ORDER[prev]:
            bad.append((i, c))
        prev = c
    return bad


def garage_overlap(hr, x, y, h):
    """止まった姿勢の車体の箱が、ガレージの長方形に x・y それぞれ何cm 食い込むか。"""
    gx0, gx1, gy0, gy1 = hr.GARAGE_BOX
    c, s = math.cos(math.radians(h)), math.sin(math.radians(h))
    xs, ys = [], []
    for a in (-hr.REAR, hr.FRONT):
        for b in (-hr.HALF_WIDTH, hr.HALF_WIDTH):
            xs.append(x + a * c - b * s); ys.append(y + a * s + b * c)
    return float(min(max(xs), gx1) - max(min(xs), gx0)), float(min(max(ys), gy1) - max(min(ys), gy0))


def check_path(hr, pts, steps, min_clear, arm_from=None):
    """すき間・進入禁止を調べる。arm_from: この番号のステップからアームが下がっている(前を伸ばす)。"""
    import numpy as np
    P = np.array([(p[0], p[1]) for p in pts]); H = np.radians([p[2] for p in pts]); I = np.array([p[3] for p in pts])
    clear = hr._rect_clear(P[:, 0], P[:, 1], H, -hr.REAR, hr.FRONT)
    if arm_from is not None:
        m = I >= arm_from
        if m.any():
            arm_front = float(hr.PARAMS.get('arm_front_cm', 5.0))
            ca = hr._rect_clear(P[m, 0], P[m, 1], H[m], -hr.REAR, hr.FRONT + arm_front,
                                hw=max(hr.HALF_WIDTH, float(hr.PARAMS.get('arm_half_width_cm', hr.HALF_WIDTH))))
            clear = clear.copy(); clear[m] = np.minimum(clear[m], ca)
    hr._EXEMPT_ON = True     # スタートの近くだけ、置いた時点でかかっている進入禁止を外す(道具と同じ)
    ko = hr.body_keepout_mask(P[:, 0], P[:, 1], np.degrees(H))
    ko_deep = ko
    if ko.any():
        # 2026-11-13: はみ出しの深さ。進入禁止の長方形を KEEPOUT_TOL_CM だけ内側へ縮めても当たるなら、深いはみ出し
        saved = hr.KEEPOUT, hr._KO_CORNERS, hr._START_EXEMPT
        t = KEEPOUT_TOL_CM
        hr.KEEPOUT = [(x0 + t, x1 - t, y0 + t, y1 - t) for x0, x1, y0, y1 in saved[0]]
        hr._KO_CORNERS = None; hr._START_EXEMPT = None
        ko_deep = hr.body_keepout_mask(P[:, 0], P[:, 1], np.degrees(H))
        hr.KEEPOUT, hr._KO_CORNERS, hr._START_EXEMPT = saved
    hr._EXEMPT_ON = False
    k = int(np.argmin(clear))
    return dict(min_clear=round(float(clear[k]), 2), min_clear_at=(round(float(P[k, 0]), 1), round(float(P[k, 1]), 1), int(I[k])),
                keepout_hits=int(ko_deep.sum()), keepout_shallow=int(ko.sum() - ko_deep.sum()), keepout_first=(None if not ko.any() else (round(float(P[ko][0, 0]), 1), round(float(P[ko][0, 1]), 1), int(I[ko][0]))),
                max_x=round(float(P[:, 0].max()), 1), min_y=round(float(P[:, 1].min()), 1))


def check(plan_path, params_path, budget, min_clear, ratio=0.93, tol=0.1):
    plan = json.load(open(plan_path, encoding='utf-8'))
    hr = load_tool(plan['gates'], params_path)
    sys.path.insert(0, ROOT)
    from robot_program.et_rally_time_budget import estimate_steps
    steps = plan['steps']
    start = (float(hr.START[0]), float(hr.START[1]), float(hr.START_H))
    res = dict(plan=os.path.basename(plan_path), ng=[], warn=[])
    # 1. 本線
    pb = float(getattr(hr, 'PIVOT_BACK_CM', 0.0))
    pts = sample(steps, start, pivot_back_cm=pb)
    arm_i = next((i for i, s in enumerate(steps) if s.get('label') == 'garage-arm-down'), None)
    if arm_i is None:
        res['ng'].append('main: no garage-arm-down label (the arm never goes down before the garage)')
    m = check_path(hr, pts, steps, min_clear, arm_from=arm_i)
    seq = gate_crossings(hr, pts)
    bad = order_ok(seq)
    end = pts[-1]
    ov = garage_overlap(hr, end[0], end[1], end[2])
    res['main'] = dict(m, gates=''.join(c[0].upper() for c in seq), order_bad=bad, end=(round(end[0], 1), round(end[1], 1), round(end[2] % 360, 1)),
                       garage_overlap=(round(ov[0], 1), round(ov[1], 1)))
    if m['min_clear'] < min_clear - tol:
        res['ng'].append('main: clearance %.2fcm < %.1f at %s' % (m['min_clear'], min_clear, m['min_clear_at']))
    elif m['min_clear'] < min_clear:
        res['warn'].append('main: clearance %.2fcm (within %.1fmm below %.1f) at %s' % (m['min_clear'], tol * 10, min_clear, m['min_clear_at']))
    if m['keepout_hits']:
        res['ng'].append('main: keepout %d samples, first %s' % (m['keepout_hits'], m['keepout_first']))
    elif m['keepout_shallow']:
        res['warn'].append('main: keepout within %.0fmm (%d samples) at %s' % (KEEPOUT_TOL_CM * 10, m['keepout_shallow'], m['keepout_first']))
    if bad:
        res['ng'].append('main: gate order %s' % res['main']['gates'])
    need_x, need_y = float(hr.PARAMS.get('garage_overlap_cm', 6.0)), float(hr.PARAMS.get('garage_overlap_y_cm', 8.0))
    if ov[0] < need_x - 0.5 or ov[1] < need_y - 0.5:
        res['ng'].append('main: garage overlap %.1f/%.1f < %.1f/%.1f' % (ov[0], ov[1], need_x, need_y))
    laps_total = int(plan.get('laps') or max([int(s['label'][3:].split('-')[0]) for s in steps if (s.get('label') or '').startswith('lap')] or [0]))
    res['laps'] = laps_total
    # 2. 周の終わりの印
    marks = {}
    for i, s in enumerate(steps):
        for mk in s.get('marks') or []:
            if mk.startswith('lap') and mk.endswith('-end'):
                marks[int(mk[3:-4])] = i
    ex = plan.get('exit')
    ex_laps = sorted((ex or {}).get('laps') or [])
    res['exit_laps'] = ex_laps
    res['marks'] = sorted(marks)
    if not ex or not ex.get('steps'):
        res['ng'].append('no exit (cannot stop early by time)')
    else:
        missing = [k for k in ex_laps if k not in marks]
        if missing:
            res['ng'].append('marks missing for exit laps %s' % missing)
        cannot = [k for k in range(1, laps_total) if k not in ex_laps]
        if cannot:
            res['warn'].append('no early exit after laps %s' % cannot)
    # 3. 各周の印から exit
    res['exits'] = {}
    if ex and ex.get('steps'):
        exs = ex['steps']
        ex_arm = next((i for i, s in enumerate(exs) if s.get('label') == 'garage-arm-down'), None)
        if ex_arm is None:
            res['ng'].append('exit: no garage-arm-down label (the arm never goes down before the garage)')
        for k in sorted(marks):
            mi = marks[k]
            pre = sample(steps[:mi], start, pivot_back_cm=pb)
            p0 = pre[-1]
            pre_seq = gate_crossings(hr, pre)
            e_pts = sample(exs, (p0[0], p0[1], p0[2]), pivot_back_cm=pb)
            em = check_path(hr, e_pts, exs, min_clear, arm_from=ex_arm)
            eseq = gate_crossings(hr, e_pts)
            ebad = order_ok(eseq, prev=(pre_seq[-1] if pre_seq else None))   # 印の手前で最後にくぐった色から数える
            ee = e_pts[-1]
            eov = garage_overlap(hr, ee[0], ee[1], ee[2])
            res['exits'][k] = dict(em, before=(pre_seq[-1][0].upper() if pre_seq else '-'), gates=''.join(c[0].upper() for c in eseq), order_bad=ebad, end=(round(ee[0], 1), round(ee[1], 1), round(ee[2] % 360, 1)),
                                   garage_overlap=(round(eov[0], 1), round(eov[1], 1)))
            if em['min_clear'] < min_clear - tol:
                res['ng'].append('exit lap%d: clearance %.2fcm at %s' % (k, em['min_clear'], em['min_clear_at']))
            elif em['min_clear'] < min_clear:
                res['warn'].append('exit lap%d: clearance %.2fcm (within %.1fmm below %.1f)' % (k, em['min_clear'], tol * 10, min_clear))
            if em['keepout_shallow'] and not em['keepout_hits']:
                res['warn'].append('exit lap%d: keepout within %.0fmm' % (k, KEEPOUT_TOL_CM * 10))
            if em['keepout_hits']:
                res['ng'].append('exit lap%d: keepout %d samples, first %s (max x %.1f, min y %.1f)' % (k, em['keepout_hits'], em['keepout_first'], em['max_x'], em['min_y']))
            if ebad:
                res['ng'].append('exit lap%d: gate order %s>%s' % (k, (pre_seq[-1][0].upper() if pre_seq else '-'), res['exits'][k]['gates']))
            if eov[0] < need_x - 0.5 or eov[1] < need_y - 0.5:
                res['ng'].append('exit lap%d: garage overlap %.1f/%.1f' % (k, eov[0], eov[1]))
    # 5. 持ち時間での切り上げ(ロボットの式: 経過 + 次の周 + そこからの切り上げ(最後の周の前は最後の周 + ガレージ) + 余裕 <= 持ち時間)
    def est(a, b, extra=None):
        ss = [dict(st, distance_mm=st.get('distance_mm') or float(st.get('radius_mm', 0.0)) * math.radians(abs(float(st.get('sweep_deg', 0.0))))) for st in steps[a:b]]
        if extra:
            ss += [dict(st, distance_mm=st.get('distance_mm') or float(st.get('radius_mm', 0.0)) * math.radians(abs(float(st.get('sweep_deg', 0.0))))) for st in extra]
        return estimate_steps(ss)
    exs = (ex or {}).get('steps') or []
    t_exit = est(0, 0, exs) if exs else None
    done_laps, finish = laps_total, START_OVERHEAD_SEC + ratio * est(0, len(steps))
    for k in sorted(marks):
        el = START_OVERHEAD_SEC + ratio * est(0, marks[k])
        if k + 1 < laps_total and (k + 1) in marks:
            need = ratio * (est(marks[k], marks[k + 1]) + t_exit)
        else:
            need = ratio * est(marks[k], len(steps))
        if el + need + 2.0 > budget and k in ex_laps:
            done_laps, finish = k, el + ratio * t_exit
            break
    res['budget'] = dict(budget=budget, ratio=ratio, laps=done_laps, finish=round(finish, 1), exit_sec=None if t_exit is None else round(t_exit, 1))
    if finish > budget:
        res['ng'].append('time: garage at %.1fs > budget %.0fs (%d laps)' % (finish, budget, done_laps))
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('plan')
    ap.add_argument('params', nargs='?', default=os.path.join(HERE, 'params_day.json'))
    ap.add_argument('--budget', type=float, default=70.0)
    ap.add_argument('--min-clear', type=float, default=1.5)
    ap.add_argument('--ratio', type=float, default=0.93, help='実機の速さ / 見込み(ロボットは1周ごとに合わせ直す。11/13 は0.92〜0.95)')
    ap.add_argument('--json', action='store_true')
    a = ap.parse_args()
    r = check(a.plan, a.params, a.budget, a.min_clear, a.ratio)
    if a.json:
        print(json.dumps(r, ensure_ascii=False))
        return
    print('%s: %s' % (r['plan'], 'OK' if not r['ng'] else 'NG'))
    mm = r['main']
    print('  main: gates %s, min clear %.2fcm, keepout %d, end %s, garage overlap %s' % (mm['gates'], mm['min_clear'], mm['keepout_hits'], mm['end'], mm['garage_overlap']))
    print('  laps %d, exit laps %s, marks %s' % (r['laps'], r['exit_laps'], r['marks']))
    for k, e in r['exits'].items():
        print('  exit lap%d: gates %s>%s, min clear %.2fcm, keepout %d, max x %.1f, end %s, garage overlap %s' % (
            k, e['before'], e['gates'], e['min_clear'], e['keepout_hits'], e['max_x'], e['end'], e['garage_overlap']))
    b = r['budget']
    print('  budget %.0fs: %d laps, garage at %.1fs (exit %.1fs)' % (b['budget'], b['laps'], b['finish'], b['exit_sec'] or 0))
    for s in r['ng']:
        print('  NG: ' + s)
    for s in r['warn']:
        print('  warn: ' + s)


if __name__ == '__main__':
    main()
