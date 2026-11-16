"""2026-11-15: 正解の分かっている画像をまとめて、カメラの取り付けを当てはめる(floor_fit.py の拡張)。

使うもの:
  - 止まって撮る試験(odo): 止まった場所どうしの動きはエンコーダとジャイロ(poses.txt)を正とし、スタートの置き場所 (x, y, 向き) は未知数。
  - 定規で測った止まった位置(ruler): 車軸の真ん中 (x, y) を定規の値に固定し、向きは未知数(手で付けた印の向きは±1°くらいの誤差)。
    止まったあとの画像(poses.txt で走った距離が変わらない最後の区間)を使う。ゲートの脚の点は使わない。
未知数(共通): カメラの左右の向き・左右の傾き・高さ・前後の位置・左右の位置、(任意)レンズの遠いところの伸び縮み k(下を参照)。
止まった場所ごとに下向き。
k: 床の上の点の、カメラの真下からの距離を (1 + k × (距離/600mm)^2) 倍に伸ばす(遠いところほど効く、レンズのゆがみの残りの近似)。

使い方(2026-Alpha で):
    python wireless_device/et_rally_camera_calib/floor_fit2.py --odo <dir> [...] --ruler <dir>:<x>:<y>:<向き>:<計画> [...] [--lens] [--write calib.json]
"""
import argparse
import json
import math
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, '..', '..')))
sys.path.insert(0, HERE)
from robot_program import et_rally_start_pose as sp  # noqa: E402
import floor_fit as ff  # noqa: E402


def leg_dots(gates):
    if not gates:
        return set()
    out = set()
    for c in ('red', 'blue', 'yellow'):
        g = gates.get(c)
        if g:
            for k in ('a', 'b'):
                out.add((round(g[k][0] * sp.P_MM, 1), round(g[k][1] * sp.P_MM, 1)))
    return out


def feats_from_frames(d, cam, frames):
    feats = []
    for m in frames:
        img = cv2.imread(os.path.join(d, m['file']))
        if img is None:
            continue
        dets = sp.detect_ground(cam, img)
        items = [(mk['xy'], 'M', 2.0) for mk in dets['markers']] + [(dt, 'D', 1.0) for dt in dets['dots']]
        if len(items) < 2:
            continue
        body = np.array([[x, y, 0.0] for (x, y), _, _ in items])
        px, _ = cv2.projectPoints(body, cam.rvec, cam.tvec, cam.K, cam.dist)
        und = cv2.undistortPoints(px.reshape(-1, 1, 2).astype(np.float64), cam.K, cam.dist).reshape(-1, 2)
        for (u, v), (_, kind, w) in zip(und, items):
            feats.append((float(u), float(v), kind, w))
    return feats


def load_ruler(spec, cam, settle=0.4, max_frames=10):
    d, x, y, h, plan = spec.split('|')
    P = np.loadtxt(os.path.join(d, 'poses.txt'))
    meta = [json.loads(l) for l in open(os.path.join(d, 'meta.jsonl'))]
    j = len(P) - 1
    while j > 0 and abs(P[j - 1, 4] - P[-1, 4]) < 1.0:
        j -= 1
    t0 = P[j, 0]
    fr = [m for m in meta if m['t'] >= t0 + settle and not m.get('start')]
    fr = fr[::max(1, len(fr) // max_frames)][:max_frames]
    gates = json.load(open(plan, encoding='utf-8')).get('gates')
    return dict(kind='ruler', name=os.path.basename(d.rstrip('/\\')), xy=(float(x), float(y)), h0=float(h),
                feats=feats_from_frames(d, cam, fr), legs=leg_dots(gates), nframes=len(fr))


class Model2:
    """p = [向き, 傾き, 高さ, 前, 左, k] + odo の run ごとに (x, y, 向き) + ruler ごとに 向き + 止まった場所ごとの下向き。"""
    NC = 6

    def __init__(self, cam, odo_runs, rulers, lens):
        self.R0, self.C0 = cam.R0, cam.C0.copy()
        self.odo, self.rulers, self.lens = odo_runs, rulers, lens
        self.stops = [(ri, s) for ri, run in enumerate(odo_runs) for s in run] + [(-1, r) for r in rulers]

    def n(self):
        return self.NC + 3 * len(self.odo) + len(self.rulers) + len(self.stops)

    def points(self, p):
        Rb = ff.rot_cam_body(math.radians(p[0]), math.radians(p[1]))
        C = self.C0 + np.array([p[3], p[4], p[2]])
        k_lens = p[5] if self.lens else 0.0
        out = []
        pi = self.NC + 3 * len(self.odo) + len(self.rulers)
        for si, (ri, st) in enumerate(self.stops):
            R = self.R0 @ ff.rot_y(math.radians(p[pi + si])) @ Rb.T
            if ri >= 0:
                name, (x, y, h), feats = st
                run = self.odo[ri]
                sx, sy = run[0][1][0], run[0][1][1]
                dx0, dy0, dth0 = p[self.NC + 3 * ri], p[self.NC + 3 * ri + 1], math.radians(p[self.NC + 3 * ri + 2])
                c, s = math.cos(dth0), math.sin(dth0)
                X = sx + c * (x - sx) - s * (y - sy) + dx0
                Y = sy + s * (x - sx) + c * (y - sy) + dy0
                H = math.radians(h) + dth0
                feats = [(u, v, kind, w) for u, v, kind, w, _ in feats]
                legs = set()
            else:
                k = self.rulers.index(st)
                X, Y = st['xy']
                H = math.radians(st['h0'] + p[self.NC + 3 * len(self.odo) + k])
                feats, legs = st['feats'], st['legs']
            ch, sh = math.cos(H), math.sin(H)
            pts = []
            for u, v, kind, w in feats:
                d = R.T @ np.array([u, v, 1.0])
                if d[2] >= -1e-9:
                    pts.append(None)
                    continue
                t = -C[2] / d[2]
                bx, by = C[0] + t * d[0], C[1] + t * d[1]
                if k_lens:
                    ox, oy = bx - C[0], by - C[1]
                    r = math.hypot(ox, oy) / 600.0
                    f = 1.0 + k_lens * r * r
                    bx, by = C[0] + ox * f, C[1] + oy * f
                pts.append((X + ch * bx - sh * by, Y + sh * bx + ch * by, kind, w))
            out.append((si, pts, legs))
        return out

    def associate(self, p, lim_m=60.0, lim_d=45.0):
        assign = []
        for si, pts, legs in self.points(p):
            asg = []
            for q in pts:
                if q is None:
                    asg.append(None)
                    continue
                ref = sp.CELLS if q[2] == 'M' else [dd for dd in sp.DOTS if (round(dd[0], 1), round(dd[1], 1)) not in legs]
                c = min(ref, key=lambda g: math.hypot(g[0] - q[0], g[1] - q[1]))
                near_leg = any(math.hypot(c[0] - l[0], c[1] - l[1]) < 60 for l in legs) if q[2] == 'M' else False
                ok = math.hypot(c[0] - q[0], c[1] - q[1]) <= (lim_m if q[2] == 'M' else lim_d) and not near_leg
                asg.append(c if ok else None)
            assign.append(asg)
        return assign

    def residuals(self, p, assign):
        r = []
        for (si, pts, _), asg in zip(self.points(p), assign):
            for q, a in zip(pts, asg):
                if a is None:
                    continue
                if q is None:
                    r += [100.0, 100.0]
                    continue
                w = math.sqrt(q[3])
                r += [w * (q[0] - a[0]), w * (q[1] - a[1])]
        return np.array(r)


def lm(model, p, assign, fixed=(), iters=80):
    r = model.residuals(p, assign)
    lam = 1e-2
    free = [i for i in range(len(p)) if i not in fixed]
    for _ in range(iters):
        J = np.zeros((len(r), len(free)))
        for jj, i in enumerate(free):
            q = p.copy(); q[i] += 1e-3
            J[:, jj] = (model.residuals(q, assign) - r) / 1e-3
        A = J.T @ J
        step = np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-9), -J.T @ r)
        q = p.copy(); q[free] += step
        rq = model.residuals(q, assign)
        if (rq ** 2).sum() < (r ** 2).sum():
            p, r = q, rq
            lam = max(lam / 3, 1e-7)
            if np.abs(step).max() < 1e-5:
                break
        else:
            lam *= 5
    return p, r


def report(model, p, assign, title):
    print(title)
    for (si, pts, _), asg in zip(model.points(p), assign):
        e = np.array([(q[0] - a[0], q[1] - a[1]) for q, a in zip(pts, asg) if a is not None and q is not None])
        ri, st = model.stops[si]
        name = ('odo%d %s' % (ri + 1, st[0])) if ri >= 0 else ('ruler %s' % st['name'])
        if len(e):
            print('  %-34s 点%3d  平均のずれ x%+6.1f y%+6.1f  ばらつき %.1f' % (name, len(e), e[:, 0].mean(), e[:, 1].mean(),
                                                                         float(np.sqrt(((e - e.mean(0)) ** 2).sum(1).mean()))))
        else:
            print('  %-34s 点なし' % name)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--odo', nargs='*', default=[])
    ap.add_argument('--ruler', nargs='*', default=[], help='dir|x|y|heading|plan')
    ap.add_argument('--calib')
    ap.add_argument('--lens', action='store_true')
    ap.add_argument('--write')
    a = ap.parse_args()
    sp.set_gate_legs(None)
    cam = sp.GroundCamera(a.calib, top_s=0.5, top_x=sp.START_WIDE_X, top_y=sp.START_WIDE_Y)
    odo = [ff.load_run(d, cam) for d in a.odo]
    rulers = [load_ruler(s, cam) for s in a.ruler]
    for r in rulers:
        print('ruler %s: %d frames, %d features' % (r['name'], r['nframes'], len(r['feats'])))
    m = Model2(cam, odo, rulers, a.lens)
    p = np.zeros(m.n())
    cam_idx = list(range(m.NC))
    # 1段目: カメラは今のまま
    assign = m.associate(p)
    p, r = lm(m, p, assign, fixed=cam_idx)
    assign = m.associate(p)
    p, r = lm(m, p, assign, fixed=cam_idx)
    report(m, p, assign, '今の較正のまま: 残り %.2f mm' % float(np.sqrt((r ** 2).mean())))
    fixed = () if a.lens else (5,)
    p1, r1 = lm(m, p.copy(), assign, fixed=fixed)
    assign = m.associate(p1)
    p1, r1 = lm(m, p1, assign, fixed=fixed)
    report(m, p1, assign, 'カメラの取り付けも当てはめ: 残り %.2f mm' % float(np.sqrt((r1 ** 2).mean())))
    print('カメラの補正: 左右の向き %+.2f°  傾き %+.2f°  高さ %+.1f  前 %+.1f  左 %+.1f mm%s' % (
        p1[0], p1[1], p1[2], p1[3], p1[4], ('  レンズ k %+.4f' % p1[5]) if a.lens else ''))
    for ri in range(len(odo)):
        b = m.NC + 3 * ri
        print('  odo%d の置き場所の補正: x %+.1f y %+.1f 向き %+.2f°' % (ri + 1, p1[b], p1[b + 1], p1[b + 2]))
    for k, r in enumerate(rulers):
        print('  ruler %s の向きの補正(定規の向きに足す): %+.2f°' % (r['name'], p1[m.NC + 3 * len(odo) + k]))
    if a.write:
        d = json.load(open(a.calib or sp.DEFAULT_CALIB, encoding='utf-8'))
        d['cam_yaw_fix_deg'] = float(d.get('cam_yaw_fix_deg', 0.0)) + float(p1[0])
        d['roll_fix_deg'] = float(d.get('roll_fix_deg', 0.0)) + float(p1[1])
        d['height_fix_mm'] = float(d.get('height_fix_mm', 0.0)) + float(p1[2])
        d['front_fix_mm'] = float(d.get('front_fix_mm', 0.0)) + float(p1[3])
        d['lat_fix_mm'] = float(d.get('lat_fix_mm', 0.0)) + float(p1[4])
        if a.lens:
            d['lens_far_k'] = float(p1[5])
        json.dump(d, open(a.write, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
        print('書いた:', a.write)


if __name__ == '__main__':
    main()
