"""2026-11-13: 床での較正。ロボットを定規で測った位置に置いて撮った画像から、カメラの位置と向きの補正を求める。

較正台(recalibrate_day.py)で出したカメラの位置・向きは、床で使うと数mm ずれる(11/13: 止まった状態で前へ4〜6mm、
左右は場所によって右へ3〜8mm)。床の上の、位置が分かっている点・マーカーの写り方に一番合うように、次を一度に求める。
  全体: 前後の位置(front)、左右の位置(lat)、高さ(height)、左右の傾き(roll)、向き(yaw)
  画像ごと: 下向き(床の凹凸で変わる)、ロボットの向き(heading_known でない画像だけ。ジャイロの向きは0.5°ほどずれるため)
向き(yaw)は、ロボットの向きが分かっている画像(heading_known)がないと決まらない(ロボットの向きと区別できない)。

入力: JSON のリスト [{"image": パス, "pose": [x, y, 向き度](コースの座標 mm、向きはコースの向き), "gates": 計画の gates か null,
                       "heading_known": true/false}, ...]
使い方: python floor_calib.py samples.json [--calib 較正ファイル] [--write 新しい較正ファイル]
出力: 補正の値と、補正の前後の食い違い(点・マーカーの本当の位置との差 mm)。--write で、補正を入れた較正ファイルを書く
     (front_fix_mm・lat_fix_mm・height_fix_mm・roll_fix_deg・cam_yaw_fix_deg に足す。GroundCamera が読む)。
"""
import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, ROOT)


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('samples')
    ap.add_argument('--calib', default=None)
    ap.add_argument('--write', default=None)
    ap.add_argument('--dump', default=None, help='点ごとの食い違い(画素の位置、車体から見た前・右 mm)を CSV に書く(補正なし・補正ありの両方)')
    ap.add_argument('--fix', default='', help='求めない全体の補正(カンマ区切り: front,lat,height,roll,yaw)')
    a = ap.parse_args()
    import cv2
    from robot_program import et_rally_start_pose as sp

    def least_squares(fun, x0, f_scale=5.0, iters=60):
        """外れ値に強い(Huber の重み)レーベンバーグ・マーカート法。scipy がないため自前。"""
        x = np.array(x0, float)
        lam = 1e-3
        def cost(r):
            a_ = np.abs(r)
            return float(np.sum(np.where(a_ <= f_scale, 0.5 * r ** 2, f_scale * (a_ - 0.5 * f_scale))))
        r = fun(x); c = cost(r)
        for _ in range(iters):
            w = np.where(np.abs(r) <= f_scale, 1.0, f_scale / np.maximum(np.abs(r), 1e-9))
            J = np.zeros((len(r), len(x)))
            for j in range(len(x)):
                dx = np.zeros(len(x)); dx[j] = 1e-4
                J[:, j] = (fun(x + dx) - r) / 1e-4
            A = J.T @ (w[:, None] * J); gvec = J.T @ (w * r)
            while True:
                step = np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-9), -gvec)
                r2 = fun(x + step); c2 = cost(r2)
                if c2 < c:
                    x, r, c = x + step, r2, c2; lam = max(lam / 3, 1e-9); break
                lam *= 5
                if lam > 1e8:
                    break
            if np.abs(step).max() < 1e-7 or lam > 1e8:
                break
        class _S: pass
        s_ = _S(); s_.x = x
        return s_
    samples = json.load(open(a.samples, encoding='utf-8'))
    cam = sp.GroundCamera(a.calib, top_x=sp.START_WIDE_X, top_y=sp.START_WIDE_Y)
    R0, C0, K, dist = cam.R0, cam.C0, cam.K, cam.dist   # いまの較正(補正を入れたあと)のカメラ: 車体の座標 → カメラ
    cells = [((i + 0.5) * sp.P_MM, (j + 0.5) * sp.P_MM) for i in range(4) for j in range(4)]
    obs = []   # (画像の番号, 正規化した画素 (u, v), 本当の位置 (x, y) コースの座標)
    imgs = []
    for k, smp in enumerate(samples):
        sp.set_gate_legs(smp.get('gates'))
        img = cv2.imread(smp['image'])
        if img is None:
            print('cannot read', smp['image']); continue
        dets = sp.detect_ground(cam, img)
        x0, y0, h0 = smp['pose']
        hr = math.radians(h0)
        n0 = len(obs)
        items = [(m['xy'], cells, sp.ASSOC_MARKER_MM) for m in dets.get('markers', [])] + [(d, sp._dots_visible, sp.ASSOC_DOT_MM) for d in dets.get('dots', [])]
        for b, world, lim in items:
            # 車体の座標 (前, 左) → コースの座標(いまの較正での見え方)
            wx = x0 + b[0] * math.cos(hr) - b[1] * math.sin(hr); wy = y0 + b[0] * math.sin(hr) + b[1] * math.cos(hr)
            best = min(world, key=lambda w: (w[0] - wx) ** 2 + (w[1] - wy) ** 2)
            if math.hypot(best[0] - wx, best[1] - wy) > lim or sp._near_leg(best):
                continue
            # 床の点(車体の座標)を、いまの較正で画素へ戻す(検出した点の画素の位置)
            pc = R0 @ (np.array([b[0], b[1], 0.0]) - C0)
            uvn = pc[:2] / pc[2]
            pix = cv2.projectPoints(np.array([[b[0], b[1], 0.0]]), cam.rvec, cam.tvec, K, dist)[0].reshape(2)
            obs.append((len(imgs), uvn, best, (float(pix[0]), float(pix[1])), (float(b[0]), float(b[1]))))
        imgs.append(dict(name=os.path.basename(os.path.dirname(smp['image'])) + '/' + os.path.basename(smp['image']),
                         pose=(x0, y0, h0), known=bool(smp.get('heading_known')), n=len(obs) - n0))
    print('images %d, observations %d' % (len(imgs), len(obs)))
    names = ['front', 'lat', 'height', 'roll', 'yaw']
    fixed = {f.strip() for f in a.fix.split(',') if f.strip()}
    if not any(im['known'] for im in imgs):
        fixed.add('yaw')
    free = [nm for nm in names if nm not in fixed]
    nI = len(imgs)
    unk = [i for i, im in enumerate(imgs) if not im['known']]

    def model(p):
        g = dict(zip(free, p[:len(free)]))
        pitch = p[len(free):len(free) + nI]
        dh = dict(zip(unk, p[len(free) + nI:]))
        res = []
        for i, uvn, w, _pix, _b in obs:
            # 補正を入れたカメラ: 位置 C0 + (front, lat, height)、向き = R0 を 車体の軸まわりに yaw(上下軸)・roll(前後軸)・pitch(左右軸)で回す
            C = C0 + np.array([g.get('front', 0.0), g.get('lat', 0.0), g.get('height', 0.0)])
            Rb = rot_z(math.radians(g.get('yaw', 0.0))) @ rot_x(math.radians(g.get('roll', 0.0))) @ rot_y(math.radians(pitch[i]))
            R = R0 @ Rb.T
            d = R.T @ np.array([uvn[0], uvn[1], 1.0])
            if d[2] >= -1e-9:
                res += [100.0, 100.0]; continue
            s = -C[2] / d[2]
            bx, by = C[0] + s * d[0], C[1] + s * d[1]
            x0, y0, h0 = imgs[i]['pose']
            hr = math.radians(h0 + dh.get(i, 0.0))
            wx = x0 + bx * math.cos(hr) - by * math.sin(hr); wy = y0 + bx * math.sin(hr) + by * math.cos(hr)
            res += [wx - w[0], wy - w[1]]
        return np.array(res)

    p0 = np.zeros(len(free) + nI + len(unk))
    # 比べるための「補正なし」: 全体の補正は0のまま、画像ごとの下向き・向きだけ合わせた食い違い
    free_all = free
    free = []
    pb = least_squares(model, np.zeros(nI + len(unk)), f_scale=5.0).x
    r0 = model(pb)
    pb_dh = dict(zip(unk, pb[nI:]))
    free = free_all
    sol = least_squares(model, p0, f_scale=5.0)
    r1 = model(sol.x)
    g = dict(zip(free, sol.x[:len(free)]))
    print('補正(いまの較正に足す): ' + ', '.join('%s %+.2f%s' % (nm, g.get(nm, 0.0), 'mm' if nm in ('front', 'lat', 'height') else '°') for nm in names) + (' (固定: %s)' % ','.join(sorted(fixed)) if fixed else ''))

    def per_image(r):
        out = []
        k = 0
        for i in range(nI):
            n = imgs[i]['n']
            rr = r[2 * k:2 * (k + n)].reshape(-1, 2) if n else np.zeros((0, 2))
            out.append(float(np.sqrt((rr ** 2).sum(1).mean())) if n else float('nan'))
            k += n
        return out
    b0, b1 = per_image(r0), per_image(r1)
    pitch = sol.x[len(free):len(free) + nI]
    dh = dict(zip(unk, sol.x[len(free) + nI:]))
    for i, im in enumerate(imgs):
        print('  %-48s 点 %2d  食い違い %5.1f → %5.1f mm  下向き %+.2f°  向き%s' % (
            im['name'][-48:], im['n'], b0[i], b1[i], pitch[i], ' (固定)' if im['known'] else ' %+.2f°' % dh.get(i, 0.0)))
    print('全体の食い違い(点ごとの距離の真ん中): %.1f → %.1f mm' % (
        float(np.median(np.hypot(r0[0::2], r0[1::2]))), float(np.median(np.hypot(r1[0::2], r1[1::2])))))
    if a.dump:
        with open(a.dump, 'w', encoding='utf-8') as f:
            f.write('image,u,v,body_front,body_left,front0,right0,front1,right1\n')
            k = 0
            for (i, uvn, w, pix, b) in obs:
                x0, y0, h0 = imgs[i]['pose']
                out = []
                for r, hh in ((r0, pb_dh.get(i, 0.0)), (r1, dh.get(i, 0.0))):
                    ex, ey = r[2 * k], r[2 * k + 1]
                    hr = math.radians(h0 + hh)
                    out += [ex * math.cos(hr) + ey * math.sin(hr), ex * math.sin(hr) - ey * math.cos(hr)]
                f.write('%s,%.1f,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%.2f\n' % ((imgs[i]['name'], pix[0], pix[1], b[0], b[1]) + tuple(out)))
                k += 1
        print('書いた:', a.dump)
    if a.write:
        d = json.load(open(a.calib or sp.DEFAULT_CALIB, encoding='utf-8'))
        d['front_fix_mm'] = round(float(d.get('front_fix_mm', 0.0)) + g.get('front', 0.0), 2)
        d['lat_fix_mm'] = round(float(d.get('lat_fix_mm', 0.0)) + g.get('lat', 0.0), 2)
        d['height_fix_mm'] = round(float(d.get('height_fix_mm', 0.0)) + g.get('height', 0.0), 2)
        d['roll_fix_deg'] = round(float(d.get('roll_fix_deg', 0.0)) + g.get('roll', 0.0), 3)
        d['cam_yaw_fix_deg'] = round(float(d.get('cam_yaw_fix_deg', 0.0)) + g.get('yaw', 0.0), 3)
        d['note_floor_calib'] = 'floor_calib.py: %d 枚、点 %d から求めた補正 %s' % (len(imgs), len(obs), {k: round(v, 3) for k, v in g.items()})
        json.dump(d, open(a.write, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
        print('書いた:', a.write)


if __name__ == '__main__':
    main()
