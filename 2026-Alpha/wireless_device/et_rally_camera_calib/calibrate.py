"""2026-11-03: et_rally_camera_capture.py で撮った画像から、レンズの較正と、較正台でのカメラの位置・向きを計算する(PCで動かす)。

車体の座標: 原点は車軸(駆動輪の接地点の中点)、X=前、Y=左、Z=上(mm)。
較正台: 後ろの壁 → 車軸 D2、壁 → ボードの手前の黒いマスの端 D1(左右)、ボードの左右の真ん中のずれ D3(左が+)。
usage: python calibrate.py <lensフォルダ> <jigフォルダ> [--d1 420,420] [--d2 179] [--d3 0] [--sq 25] [--out calib.json]
"""
import argparse
import glob
import json
import math
import os

import cv2
import numpy as np

COLS, ROWS = 9, 6


def detect(path):
    img = cv2.imread(path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ok, c = cv2.findChessboardCornersSB(gray, (COLS, ROWS), cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
    if not ok:
        ok, c = cv2.findChessboardCorners(gray, (COLS, ROWS), cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not ok:
            return None, gray.shape[::-1]
        c = cv2.cornerSubPix(gray, c, (5, 5), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.001))
    return c.reshape(-1, 2).astype(np.float32), gray.shape[::-1]


def board_obj(sq):
    o = np.zeros((ROWS * COLS, 3), np.float32)
    o[:, :2] = np.mgrid[0:COLS, 0:ROWS].T.reshape(-1, 2) * sq
    return o


def lens(folder, sq):
    files = sorted(glob.glob(os.path.join(folder, '*.png')))
    objs, imgs, used, size = [], [], [], None
    for f in files:
        c, size = detect(f)
        if c is not None:
            objs.append(board_obj(sq)); imgs.append(c); used.append(os.path.basename(f))
    print('レンズ: %d枚中 %d枚でボードが見つかった' % (len(files), len(used)))
    flags = 0
    rms, K, dist, rv, tv = cv2.calibrateCamera(objs, imgs, size, None, None, flags=flags)
    per = []
    for o, i, r, t in zip(objs, imgs, rv, tv):
        p, _ = cv2.projectPoints(o, r, t, K, dist)
        per.append(float(np.sqrt(np.mean(np.sum((p.reshape(-1, 2) - i) ** 2, axis=1)))))
    # ずれの大きい画像を外してやり直す(手ぶれ・ボードのたわみなど)
    keep = [k for k, e in enumerate(per) if e < max(1.0, 3 * np.median(per))]
    if len(keep) < len(per):
        print('  ずれの大きい画像を除外:', [used[k] for k in range(len(used)) if k not in keep])
        objs = [objs[k] for k in keep]; imgs = [imgs[k] for k in keep]; used = [used[k] for k in keep]
        rms, K, dist, rv, tv = cv2.calibrateCamera(objs, imgs, size, None, None, flags=flags)
    # 画面のどこまでボードの角が来たか
    allp = np.concatenate(imgs)
    print('  使った枚数 %d、再投影のずれ %.3f画素' % (len(used), rms))
    print('  焦点距離 fx=%.1f fy=%.1f、中心 cx=%.1f cy=%.1f' % (K[0, 0], K[1, 1], K[0, 2], K[1, 2]))
    print('  ゆがみ', np.round(dist.ravel(), 4).tolist())
    print('  角が来た範囲: 横 %.0f〜%.0f、縦 %.0f〜%.0f (画面 %dx%d)' % (allp[:, 0].min(), allp[:, 0].max(), allp[:, 1].min(), allp[:, 1].max(), size[0], size[1]))
    return {'K': K.tolist(), 'dist': dist.ravel().tolist(), 'rms_px': rms, 'size': list(size), 'images': used}, K, dist


def order_for_robot(c):
    """見つかった角を「遠い行から近い行へ、各行は左から右へ」に並べ直す(画像の上=遠い、左=車体の左)。"""
    g = c.reshape(ROWS, COLS, 2)
    if g[0, :, 1].mean() > g[-1, :, 1].mean():
        g = g[::-1]
    if g[:, 0, 0].mean() > g[:, -1, 0].mean():
        g = g[:, ::-1]
    return g


def jig(folder, K, dist, sq, d1l, d1r, d2, d3):
    near_x = (d1l + d1r) / 2.0 - d2 + sq          # 手前の内側の角の行(黒いマスの端から1マス内側)
    skew = math.atan2(d1l - d1r, (COLS + 1) * sq)  # ボードの斜め(左が遠いと+)
    res = []
    for f in sorted(glob.glob(os.path.join(folder, '*.png'))):
        c, _ = detect(f)
        if c is None:
            print('  %s: ボードが見つからない' % os.path.basename(f)); continue
        g = order_for_robot(c)
        obj = []
        for r in range(ROWS):           # r=0 が一番遠い行
            for k in range(COLS):       # k=0 が一番左
                bx = (ROWS - 1 - r) * sq                 # 手前の行からの前方向
                by = ((COLS - 1) / 2.0 - k) * sq         # 左が+
                x = near_x + bx * math.cos(skew) - by * math.sin(skew)
                y = d3 + bx * math.sin(skew) + by * math.cos(skew)
                obj.append([x, y, 0.0])
        obj = np.array(obj, np.float32)
        img = g.reshape(-1, 2).astype(np.float32)
        ok, rv, tv = cv2.solvePnP(obj, img, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
        R, _ = cv2.Rodrigues(rv)
        C = (-R.T @ tv).ravel()
        ax = R.T @ np.array([0, 0, 1.0])      # 光軸
        xr = R.T @ np.array([1.0, 0, 0])      # 画像の右
        pitch = math.degrees(math.asin(-ax[2]))
        yaw = math.degrees(math.atan2(ax[1], ax[0]))
        roll = math.degrees(math.asin(xr[2]))
        p, _ = cv2.projectPoints(obj, rv, tv, K, dist)
        err = float(np.sqrt(np.mean(np.sum((p.reshape(-1, 2) - img) ** 2, axis=1))))
        res.append({'file': os.path.basename(f), 'cam_mm': C.tolist(), 'pitch_down_deg': pitch, 'yaw_left_deg': yaw,
                    'roll_deg': roll, 'reproj_px': err, 'rvec': rv.ravel().tolist(), 'tvec': tv.ravel().tolist()})
        print('  %s: カメラ 前%.1f 左%.1f 高さ%.1f mm、下向き%.2f° 左向き%.2f° 傾き%.2f°、ずれ%.2f画素'
              % (os.path.basename(f), C[0], C[1], C[2], pitch, yaw, roll, err))
    return res


def ground_table(K, dist, rv, tv, size):
    """画像の点 → 床(車体の座標)。行ごとの床の前方向の距離と、1画素あたりのmm。"""
    R, _ = cv2.Rodrigues(np.array(rv))
    t = np.array(tv).reshape(3)
    C = -R.T @ t
    out = []
    for v in (479, 440, 400, 360, 320, 280, 240, 200):
        row = []
        for u in (0, 160, 320, 480, 639):
            n = cv2.undistortPoints(np.array([[[u, v]]], np.float32), K, dist).reshape(2)
            d = R.T @ np.array([n[0], n[1], 1.0])
            if d[2] >= -1e-6:
                row.append(None); continue
            s = -C[2] / d[2]
            P = C + s * d
            row.append((round(P[0]), round(P[1])))
        out.append((v, row))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('lens'); ap.add_argument('jig')
    ap.add_argument('--d1', default='420,420'); ap.add_argument('--d2', type=float, default=179.0)
    ap.add_argument('--d3', type=float, default=0.0); ap.add_argument('--sq', type=float, default=25.0)
    ap.add_argument('--out', default='calib.json')
    a = ap.parse_args()
    d1l, d1r = [float(x) for x in a.d1.split(',')]
    L, K, dist = lens(a.lens, a.sq)
    print('較正台: D1 左%.0f 右%.0f、D2 %.0f、D3 %.0f、マス %.1f mm' % (d1l, d1r, a.d2, a.d3, a.sq))
    J = jig(a.jig, K, dist, a.sq, d1l, d1r, a.d2, a.d3)
    if J:
        for key in ('pitch_down_deg', 'yaw_left_deg', 'roll_deg'):
            v = [j[key] for j in J]
            print('  %s: 平均 %.2f、ばらつき(最大-最小) %.2f' % (key, np.mean(v), max(v) - min(v)))
        cams = np.array([j['cam_mm'] for j in J])
        print('  カメラ位置の平均 前%.1f 左%.1f 高さ%.1f、ばらつき %s' % (*cams.mean(0), np.round(cams.max(0) - cams.min(0), 1).tolist()))
        print('床の位置(画像の行v・列u → 車体の座標 前,左 mm)、%s で計算' % J[0]['file'])
        for v, row in ground_table(K, dist, J[0]['rvec'], J[0]['tvec'], L['size']):
            print('  v=%3d: ' % v + '  '.join('%s' % (r,) for r in row))
    json.dump({'lens': L, 'jig': J, 'jig_setup': {'d1': [d1l, d1r], 'd2': a.d2, 'd3': a.d3, 'sq': a.sq}},
              open(a.out, 'w'), indent=1)
    print('保存:', a.out)


if __name__ == '__main__':
    main()
