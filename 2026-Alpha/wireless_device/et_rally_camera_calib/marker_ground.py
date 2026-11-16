"""2026-11-03: マーカー(QRコード)を写した画像から、QRの4隅を見つけて、較正(calib_*.json)で床の位置(車体の座標)に直す(PCで動かす)。
車体の座標: 原点は車軸、X=前、Y=左(mm)。答え合わせ用に、床の上での辺の長さ(本当は約50mm)と角の角度も出す。
マーカーの探し方は find_qr_ground を参照(11/03: 21枚すべてで見つかった)。
usage: python marker_ground.py <calib.json> <画像フォルダ> [--debug 出力フォルダ]
"""
import argparse
import glob
import json
import math
import os

import cv2
import numpy as np


def load_calib(path):
    d = json.load(open(path))
    K = np.array(d['lens']['K']); dist = np.array(d['lens']['dist'])
    j = d['jig'][0]
    R, _ = cv2.Rodrigues(np.array(j['rvec']))
    t = np.array(j['tvec']).reshape(3)
    return K, dist, R, t


def _unused_to_ground(pts, K, dist, R, t):
    """画像の点(N,2) → 床の点(N,2)、車体の座標(前, 左)mm。"""
    C = -R.T @ t
    n = cv2.undistortPoints(np.asarray(pts, np.float32).reshape(-1, 1, 2), K, dist).reshape(-1, 2)
    out = []
    for x, y in n:
        d = R.T @ np.array([x, y, 1.0])
        s = -C[2] / d[2]
        P = C + s * d
        out.append((P[0], P[1]))
    return np.array(out)


TOP_S = 0.5                       # 真上から見た画像の1画素(mm)
TOP_X = (250.0, 900.0)            # 前方向の範囲(mm)
TOP_Y = (-350.0, 350.0)           # 左右の範囲(mm、左が+)


def top_map(K, dist, R, t):
    """真上から見た床の画像を作るための対応表。行=前(上が遠い)、列=左右(左が左)。"""
    rv, _ = cv2.Rodrigues(R)
    xs = np.arange(TOP_X[1], TOP_X[0], -TOP_S); ys = np.arange(TOP_Y[1], TOP_Y[0], -TOP_S)
    YY, XX = np.meshgrid(ys, xs)
    P = np.stack([XX.ravel(), YY.ravel(), np.zeros(XX.size)], 1)
    p, _ = cv2.projectPoints(P, rv, t, K, dist)
    return p.reshape(XX.shape + (2,)).astype(np.float32)


def top_to_mm(pts):
    pts = np.asarray(pts, np.float64).reshape(-1, 2)
    return np.stack([TOP_X[1] - pts[:, 1] * TOP_S, TOP_Y[1] - pts[:, 0] * TOP_S], 1)


QR_SIDE_MM = (44.0, 56.0)       # QRの1辺として受け入れる範囲(実物は50〜51mm)


def find_qr_ground(img, mp):
    """画像を真上から見た形に直してから、QRを「黒っぽい模様がつまった、1辺約5cmの正方形」として探す。
    QRの中身は読まない。QRの3つの角の四角が上下左右の縁に接しているので、黒い画素を囲む最小の長方形がQRの外枠になる。
    (OpenCVのQR検出器は、ぼやけた画像で半分くらいしか見つけられなかったため使わない。)
    戻り値: 床の4隅(mm、車体の座標)のリスト、真上から見た画像。"""
    top = cv2.remap(img, mp[..., 0], mp[..., 1], cv2.INTER_LINEAR, borderValue=(255, 255, 255))
    hsv = cv2.cvtColor(top, cv2.COLOR_BGR2HSV)
    v = hsv[..., 2].astype(np.int16)
    bg = cv2.medianBlur(hsv[..., 2], 61).astype(np.int16)          # まわりの明るさ(約30mm四方)
    dark = (((bg - v) > 25) & (hsv[..., 1] < 150)).astype(np.uint8) * 255   # まわりより暗く、色の薄い画素
    k = int(round(6.0 / TOP_S))                                     # 6mm: QRの模様をつなげて1つの塊にする
    blob = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (k, k)))
    n, lab, st, _ = cv2.connectedComponentsWithStats(blob)
    out = []
    for i in range(1, n):
        if not ((35.0 / TOP_S) ** 2 <= st[i, 4] <= (62.0 / TOP_S) ** 2):
            continue
        ys, xs = np.nonzero((lab == i) & (dark > 0))
        rect = cv2.minAreaRect(np.stack([xs, ys], 1).astype(np.float32))
        (cx, cy), (rw, rh), _ = rect
        side = (rw + rh) / 2.0
        if not (QR_SIDE_MM[0] <= side * TOP_S <= QR_SIDE_MM[1]) or max(rw, rh) / max(1e-6, min(rw, rh)) > 1.2:
            continue
        fill = len(xs) / (rw * rh)                                  # 黒い画素の割合(QRは半分くらい、ただの黒い四角は1に近い)
        if not (0.25 < fill < 0.9):
            continue
        c = np.array([cx, cy])
        box = c + (cv2.boxPoints(rect) - c) * ((side + 1.0) / side)  # 画素の中心どうし→外側の縁(半画素ずつ外へ)
        out.append(top_to_mm(box))
    return out, top


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('calib'); ap.add_argument('folder'); ap.add_argument('--debug', default=None)
    a = ap.parse_args()
    K, dist, R, t = load_calib(a.calib)
    if a.debug:
        os.makedirs(a.debug, exist_ok=True)
    rows = []
    mp = top_map(K, dist, R, t)
    for f in sorted(glob.glob(os.path.join(a.folder, '*.png'))):
        img = cv2.imread(f)
        name = os.path.basename(f)
        qrs, top = find_qr_ground(img, mp)
        if not qrs:
            print('%s: QRが見つからない' % name)
        for g in qrs:
            sides = [np.linalg.norm(g[(k + 1) % 4] - g[k]) for k in range(4)]
            ang = []
            for k in range(4):
                u = g[(k + 1) % 4] - g[k]; v = g[k - 1] - g[k]
                ang.append(math.degrees(math.acos(np.dot(u, v) / np.linalg.norm(u) / np.linalg.norm(v))))
            c = g.mean(0)
            dirs = []
            for k in range(4):
                e = g[(k + 1) % 4] - g[k]
                dirs.append(((math.degrees(math.atan2(e[1], e[0])) + 45) % 90) - 45)
            heading = float(np.mean(dirs))
            rows.append({'file': name, 'center_mm': [round(c[0], 1), round(c[1], 1)], 'sides_mm': [round(x, 1) for x in sides],
                         'angles_deg': [round(x, 1) for x in ang], 'rot_deg': round(heading, 2)})
            print('%s: 中心 前%.1f 左%.1f mm  辺 %s mm  角 %s°  回転 %.2f°'
                  % (name, c[0], c[1], ' '.join('%.1f' % x for x in sides), ' '.join('%.0f' % x for x in ang), heading))
        if a.debug:
            cv2.imwrite(os.path.join(a.debug, name), top)
    json.dump(rows, open(os.path.join(a.folder, 'marker_ground.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
