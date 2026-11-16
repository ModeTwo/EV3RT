"""2026-11-15: 止まって撮る試験の画像から、カメラの取り付け(左右の向き・左右の傾き・高さ、止まった場所ごとの下向き)を当てはめ直す。

背景: 11/15 の止まって撮る試験(plan_test_selfcal_stops_1114.json、15cm 進んで止まる×5)で、エンコーダとジャイロでは
まっすぐ進んでいるのに、カメラの左右の位置が場所で18mm 飛び、スタートで見た向きとフィールドの中で見た向きが1.2〜1.9° 食い違った。
較正台は1つの距離でしか合わせないので、見る距離によって誤差が変わる(左右の向き・左右の傾き・高さの誤差は距離で効き方が違う)。

考え方:
  - 止まった場所どうしの動きは、エンコーダとジャイロ(poses.txt)を正しいとする(短い直進は1mm 以内)。
  - スタート(start_*.jpg)と止まった場所ごとの画像で見つけた格子の点・マーカーを、画像の画素に戻す。
  - 未知数: カメラの左右の向き・左右の傾き・高さの補正(全部の場所で共通)、止まった場所ごとの下向き(マットの傾きで変わる)、
    スタートの置き場所の補正(x, y, 向き)。点が格子の点・マスの中心に一番よく重なるように当てはめる(LM法)。
  - カメラの左右・前後の位置は、まっすぐ進むだけではスタートの置き場所と区別できないので当てはめない(今の値のまま)。

使い方:
    python floor_fit.py <run_logs/et_camera_...> [<もう1回分> ...] [--calib calib_xxx.json] [--write calib_new.json]
"""
import argparse
import glob
import json
import math
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, ROOT)
from robot_program import et_rally_start_pose as sp  # noqa: E402


def static_intervals(P, min_sec=0.8):
    out, i, n = [], 0, len(P)
    while i < n:
        j = i
        while j + 1 < n and abs(P[j + 1, 4] - P[i, 4]) < 1.0:
            j += 1
        if P[j, 0] - P[i, 0] >= min_sec:
            out.append((P[i, 0], P[j, 0], i))
        i = j + 1
    return out


def load_run(d, cam, per_stop=8, settle=0.5):
    """[(止まった場所の番号, odometry の姿勢 (x, y, h度), [(画素u, v, 種類, 重み), ...]), ...]"""
    P = np.loadtxt(os.path.join(d, 'poses.txt'))
    meta = [json.loads(l) for l in open(os.path.join(d, 'meta.jsonl'))]
    stops = []
    starts = [m for m in meta if m.get('start')]
    if starts:
        stops.append(('start', tuple(P[0, 1:4]), starts))
    for k, (t0, t1, i) in enumerate(static_intervals(P)):
        if P[i, 4] < 1.0 and starts:
            continue   # スタートの止まり(走り出す前)は start_*.jpg で数える
        fr = [m for m in meta if t0 + settle <= m['t'] <= t1 and not m.get('start')]
        if fr:
            step = max(1, len(fr) // per_stop)
            stops.append(('stop%d' % (k + 1), tuple(P[i, 1:4]), fr[::step][:per_stop]))
    out = []
    for name, pose, frames in stops:
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
            for (u, v), (_, kind, w), b in zip(und, items, body):
                feats.append((float(u), float(v), kind, w, (float(b[0]), float(b[1]))))
        if feats:
            out.append((name, pose, feats))
    return out


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_cam_body(yaw, roll):
    cz, sz = math.cos(yaw), math.sin(yaw)
    cr, sr = math.cos(roll), math.sin(roll)
    return np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]]) @ np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])


class Model:
    """p = [左右の向き, 左右の傾き, 高さ(mm), 置き場所 x, y, 向き(度)] (共通) + 止まった場所ごとの下向き(度)。角度はラジアン以外は度。"""

    def __init__(self, cam, runs):
        self.R0, self.C0 = cam.R0, cam.C0.copy()
        self.runs = runs   # [[(name, pose, feats), ...], ...]
        self.n_stops = sum(len(r) for r in runs)

    def n_params(self):
        return 3 + 3 * len(self.runs) + self.n_stops

    def world_points(self, p):
        yaw, roll, dh = math.radians(p[0]), math.radians(p[1]), p[2]
        Rb = rot_cam_body(yaw, roll)
        C = self.C0 + np.array([0.0, 0.0, dh])
        out = []
        k = 3 + 3 * len(self.runs)
        for ri, run in enumerate(self.runs):
            dx0, dy0, dth0 = p[3 + 3 * ri], p[4 + 3 * ri], math.radians(p[5 + 3 * ri])
            sx, sy = run[0][1][0], run[0][1][1]   # 置き場所の補正は最初の姿勢のまわりに回す
            for name, (x, y, h), feats in run:
                R = self.R0 @ rot_y(math.radians(p[k])) @ Rb.T
                k += 1
                # odometry の姿勢に置き場所の補正をかける
                c, s = math.cos(dth0), math.sin(dth0)
                X = sx + c * (x - sx) - s * (y - sy) + dx0
                Y = sy + s * (x - sx) + c * (y - sy) + dy0
                H = math.radians(h) + dth0
                ch, sh = math.cos(H), math.sin(H)
                pts = []
                for u, v, kind, w, _ in feats:
                    d = R.T @ np.array([u, v, 1.0])
                    if d[2] >= -1e-9:
                        pts.append(None)
                        continue
                    t = -C[2] / d[2]
                    bx, by = C[0] + t * d[0], C[1] + t * d[1]
                    pts.append((X + ch * bx - sh * by, Y + sh * bx + ch * by, kind, w))
                out.append((ri, name, pts))
        return out

    def residuals(self, p, assign):
        r = []
        for (ri, name, pts), asg in zip(self.world_points(p), assign):
            for q, a in zip(pts, asg):
                if a is None:
                    continue
                if q is None:
                    r += [100.0, 100.0]
                    continue
                w = math.sqrt(q[3])
                r += [w * (q[0] - a[0]), w * (q[1] - a[1])]
        return np.array(r)

    def associate(self, p, lim_m=60.0, lim_d=45.0):
        assign = []
        for ri, name, pts in self.world_points(p):
            asg = []
            for q in pts:
                if q is None:
                    asg.append(None)
                    continue
                ref = sp.CELLS if q[2] == 'M' else sp._dots_visible
                c = min(ref, key=lambda g: math.hypot(g[0] - q[0], g[1] - q[1]))
                asg.append(c if math.hypot(c[0] - q[0], c[1] - q[1]) <= (lim_m if q[2] == 'M' else lim_d) else None)
            assign.append(asg)
        return assign


def lm(model, p, assign, iters=60, fixed=()):
    r = model.residuals(p, assign)
    lam = 1e-2
    free = [i for i in range(len(p)) if i not in fixed]
    for _ in range(iters):
        J = np.zeros((len(r), len(free)))
        for jj, i in enumerate(free):
            q = p.copy()
            h = 1e-3 if i >= 3 else 1e-3
            q[i] += h
            J[:, jj] = (model.residuals(q, assign) - r) / h
        A = J.T @ J
        g = J.T @ r
        step = np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-9), -g)
        q = p.copy()
        q[free] += step
        rq = model.residuals(q, assign)
        if (rq ** 2).sum() < (r ** 2).sum():
            p, r = q, rq
            lam = max(lam / 3, 1e-7)
            if np.abs(step).max() < 1e-5:
                break
        else:
            lam *= 5
    return p, r


def per_stop_report(model, p, assign, title):
    print(title)
    k = 0
    for (ri, name, pts), asg in zip(model.world_points(p), assign):
        e = [(q[0] - a[0], q[1] - a[1]) for q, a in zip(pts, asg) if a is not None and q is not None]
        if not e:
            continue
        e = np.array(e)
        print('  run%d %-6s 点%2d  平均のずれ x%+6.1f y%+6.1f mm  ばらつき %.1f mm' % (
            ri + 1, name, len(e), e[:, 0].mean(), e[:, 1].mean(), float(np.sqrt(((e - e.mean(0)) ** 2).sum(1).mean()))))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('dirs', nargs='+')
    ap.add_argument('--calib', default=None)
    ap.add_argument('--gates', default=None, help='ゲートの位置(計画のファイル)。ゲートを外して撮ったなら不要')
    ap.add_argument('--write', default=None, help='当てはめた補正を足した較正ファイルを書く')
    a = ap.parse_args()
    if a.gates:
        sp.set_gate_legs(json.load(open(a.gates, encoding='utf-8')).get('gates'))
    else:
        sp.set_gate_legs(None)
    cam = sp.GroundCamera(a.calib, top_s=0.5, top_x=sp.START_WIDE_X, top_y=sp.START_WIDE_Y)
    runs = [load_run(d, cam) for d in a.dirs]
    for d, run in zip(a.dirs, runs):
        print('%s: %s' % (os.path.basename(d.rstrip('/\\')), ', '.join('%s(%d点)' % (n, len(f)) for n, _, f in run)))
    model = Model(cam, runs)
    p = np.zeros(model.n_params())
    pitch_idx = list(range(3 + 3 * len(runs), model.n_params()))
    # 1段目: カメラは今のまま、置き場所と下向きだけ
    assign = model.associate(p)
    p, r = lm(model, p, assign, fixed=(0, 1, 2))
    assign = model.associate(p)
    p, r = lm(model, p, assign, fixed=(0, 1, 2))
    rms0 = float(np.sqrt((r ** 2).mean()))
    per_stop_report(model, p, assign, '今の較正のまま(置き場所と下向きだけ当てはめ): 重みつきの残り %.2f mm' % rms0)
    # 2段目: カメラの向き・傾き・高さも
    p1, r1 = lm(model, p.copy(), assign)
    assign = model.associate(p1)
    p1, r1 = lm(model, p1, assign)
    rms1 = float(np.sqrt((r1 ** 2).mean()))
    per_stop_report(model, p1, assign, 'カメラの取り付けも当てはめ: 重みつきの残り %.2f mm' % rms1)
    print('カメラの補正: 左右の向き %+.2f°  左右の傾き %+.2f°  高さ %+.1f mm' % (p1[0], p1[1], p1[2]))
    for ri in range(len(runs)):
        print('  run%d の置き場所の補正: x %+.1f  y %+.1f mm  向き %+.2f°' % (ri + 1, p1[3 + 3 * ri], p1[4 + 3 * ri], p1[5 + 3 * ri]))
    print('  止まった場所ごとの下向き: ' + ' '.join('%+.2f' % p1[i] for i in pitch_idx))
    if a.write:
        d = json.load(open(a.calib or sp.DEFAULT_CALIB, encoding='utf-8'))
        d['cam_yaw_fix_deg'] = float(d.get('cam_yaw_fix_deg', 0.0)) + float(p1[0])
        d['roll_fix_deg'] = float(d.get('roll_fix_deg', 0.0)) + float(p1[1])
        d['height_fix_mm'] = float(d.get('height_fix_mm', 0.0)) + float(p1[2])
        d['note'] = (d.get('note', '') + ' / floor_fit %s: 左右の向き %+.2f° 傾き %+.2f° 高さ %+.1fmm(%s)' % (
            os.path.basename(a.write), p1[0], p1[1], p1[2], ', '.join(os.path.basename(x.rstrip('/\\')) for x in a.dirs))).strip(' /')
        out = a.write if os.path.isabs(a.write) else os.path.join(HERE, a.write)
        json.dump(d, open(out, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
        print('書いた: %s' % out)


if __name__ == '__main__':
    main()
