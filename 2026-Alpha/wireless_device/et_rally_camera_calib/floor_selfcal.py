"""2026-11-14: 本番の試走で、コースの上でカメラの向き・位置のずれを測り、較正ファイルの補正を出す(PCで動かす)。

較正台(recalibrate_day.py)だけでは、カメラの左右の向き(約2°)・左右・前後(数mm)のずれが残る(11/14: 較正から向き+1.9°・
左右14mm・前後-2〜-7mm、下向き+1°)。本番のコースには試走のときしか触れず、置き方も数mm ずれるので、次の2つを使う。

1. motion: 走行中にカメラが保存した画像(--rally-camera-log、試走では 1〜2 にする)から、カメラの見た向きと、
   カメラの位置から求めた「進んだ向き」を比べる。車輪は向いている方へ進むので、差がカメラの向きのずれ。
   置き方の正確さは要らない(スタートで取り込んだ向きのずれも、同じだけ両方に入るので消える)。
   2つの画像の間で、ロボットの向きの変化が2°以内・進んだ距離が min-d mm 以上の組を使う。
2. static: 止まった状態の画像(スタート位置に置いた試走の走り出し前の画像、または et_rally_camera_capture.py で撮ったもの)
   から、置いたはずの姿勢(--pose、既定はETラリーのスタート)との前後・左右の差を出す。置き方が数mm ずれれば、その分は残る。
   向きは置き方の誤差(約1°)が大きいので、motion の値を使う。

usage:
  python floor_selfcal.py motion <et_camera_dir> [--calib C] [--min-d 100]
  python floor_selfcal.py static <画像のフォルダ or et_camera_dir> [--calib C] [--pose x,y,h]
  python floor_selfcal.py write --yaw 1.9 --lat 10.5 --front -4.0 [--calib C] --out calib_new.json
     (--yaw: motion の「カメラの向き - 進んだ向き」、--lat: static の右へのずれ mm、--front: static の前へのずれ mm。
      それぞれを打ち消すように cam_yaw_fix_deg・lat_fix_mm・front_fix_mm を足し引きした較正ファイルを書く)
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


def _cam(calib, wide=True):
    if wide:
        return sp.GroundCamera(calib, top_x=sp.START_WIDE_X, top_y=sp.START_WIDE_Y)
    return sp.GroundCamera(calib, top_s=sp.FAST_TOP_S)


def _fit(cam, img, nominal, max_resid=6.0):
    dets = sp.detect_ground(cam, img)
    if len(dets['dots']) + len(dets['markers']) < 3:
        return None
    r = sp.fit_pose_pitch(cam, dets, nominal=nominal)
    if r is None or r['resid_mm'] > max_resid or r['n_dots'] + r['n_markers'] < 3:
        return None
    return r


def motion(args):
    D = args.dir
    cam = _cam(args.calib, wide=False)
    P = np.loadtxt(os.path.join(D, 'poses.txt'))
    meta = [json.loads(l) for l in open(os.path.join(D, 'meta.jsonl'))]
    hu = np.unwrap(np.radians(P[:, 3]))
    rows = []
    for m in meta:
        t = m['t']
        if t < P[0, 0] or t > P[-1, 0]:
            continue
        bx, by, bh = np.interp(t, P[:, 0], P[:, 1]), np.interp(t, P[:, 0], P[:, 2]), np.interp(t, P[:, 0], hu)
        img = cv2.imread(os.path.join(D, m['file']))
        if img is None:
            continue
        r = _fit(cam, img, (bx, by, math.degrees(bh)))
        if r is None:
            continue
        x, y, h = r['pose']
        rows.append((t, x, y, math.radians(h), bh))
    print('使えた画像 %d / %d' % (len(rows), len(meta)))
    est = []
    for i in range(len(rows)):
        a = rows[i]
        for j in range(i + 1, min(i + 40, len(rows))):
            b = rows[j]
            if b[0] - a[0] > 1.5 or abs(math.degrees(b[4] - a[4])) > 2.0:
                break
            d = math.hypot(b[1] - a[1], b[2] - a[2])
            if d < args.min_d:
                continue
            mv = math.atan2(b[2] - a[2], b[1] - a[1])
            hb = (a[4] + b[4]) / 2
            if abs(math.atan2(math.sin(mv - hb), math.cos(mv - hb))) > math.pi / 2:
                mv += math.pi      # 後退
            hc = math.atan2(math.sin(a[3]) + math.sin(b[3]), math.cos(a[3]) + math.cos(b[3]))
            est.append((math.degrees(math.atan2(math.sin(hc - mv), math.cos(hc - mv))), d))
            break
    if not est:
        print('組がない(画像を増やす: --rally-camera-log 1〜2、まっすぐ進む区間がある経路で)')
        return
    e = np.array(est)
    med = float(np.median(e[:, 0])); q1, q3 = np.percentile(e[:, 0], [25, 75])
    se = 1.2533 * float(np.std(e[:, 0])) / math.sqrt(len(e))
    print('カメラの向き - 進んだ向き: 中央値 %+.2f°(組 %d、四分位 %+.2f〜%+.2f、中央値の誤差の目安 ±%.2f°)' % (med, len(e), q1, q3, se))
    print('→ write --yaw %.2f' % med)


def static(args):
    cam = _cam(args.calib, wide=True)
    nom = tuple(float(v) for v in args.pose.split(',')) if args.pose else (sp.START_MM[0], sp.START_MM[1], sp.START_HEADING_DEG)
    files = sorted(glob.glob(os.path.join(args.dir, '*.png')) + glob.glob(os.path.join(args.dir, '*.jpg')))
    if os.path.exists(os.path.join(args.dir, 'poses.txt')):
        # 走行の画像: 走り出す前(車軸がまだ動いていない)の画像だけ
        P = np.loadtxt(os.path.join(args.dir, 'poses.txt'))
        moved = np.flatnonzero(np.hypot(P[:, 1] - P[0, 1], P[:, 2] - P[0, 2]) > 0.5)
        t_move = P[moved[0], 0] if len(moved) else P[-1, 0]
        meta = {json.loads(l)['file']: json.loads(l)['t'] for l in open(os.path.join(args.dir, 'meta.jsonl'))}
        files = [f for f in files if meta.get(os.path.basename(f), 1e18) < t_move]
    out = []
    c, s = math.cos(math.radians(nom[2])), math.sin(math.radians(nom[2]))
    for f in files:
        r = _fit(cam, cv2.imread(f), nom)
        if r is None:
            continue
        x, y, h = r['pose']
        fwd = (x - nom[0]) * c + (y - nom[1]) * s
        right = (x - nom[0]) * s - (y - nom[1]) * c
        out.append((fwd, right, sp._nd(h - nom[2]), r['pitch_deg'], r['resid_mm']))
        print('%s 前%+.1f 右%+.1f mm 向き%+.2f° 下向き%+.2f° 食い違い%.1fmm' % ((os.path.basename(f),) + out[-1]))
    if not out:
        print('使える画像がない')
        return
    o = np.median(np.array(out), axis=0)
    print('中央値: 前%+.1f 右%+.1f mm 向き%+.2f° 下向き%+.2f°(置き方のずれも入る)' % tuple(o[:4]))
    print('→ write --front %.1f --lat %.1f' % (o[0], o[1]))


def write(args):
    src = args.calib or str(sp.DEFAULT_CALIB)
    d = json.load(open(src, encoding='utf-8'))
    old = {k: float(d.get(k, 0.0)) for k in ('cam_yaw_fix_deg', 'lat_fix_mm', 'front_fix_mm')}
    # 符号(11/14 の画像で確かめた): cam_yaw_fix_deg を +a すると向きの読みが a 減る。lat_fix_mm を -a すると右へのずれが a 減る。
    # front_fix_mm を +a すると前へのずれが a 減る。
    d['cam_yaw_fix_deg'] = round(old['cam_yaw_fix_deg'] + (args.yaw or 0.0), 2)
    d['lat_fix_mm'] = round(old['lat_fix_mm'] - (args.lat or 0.0), 1)
    d['front_fix_mm'] = round(old['front_fix_mm'] + (args.front or 0.0), 1)
    d.setdefault('notes', [])
    if isinstance(d['notes'], list):
        d['notes'].append('floor_selfcal: yaw %+.2f lat %+.1f front %+.1f from %s' % (args.yaw or 0, args.lat or 0, args.front or 0, os.path.basename(src)))
    json.dump(d, open(args.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
    print('%s: cam_yaw_fix_deg %.2f→%.2f, lat_fix_mm %.1f→%.1f, front_fix_mm %.1f→%.1f' % (
        args.out, old['cam_yaw_fix_deg'], d['cam_yaw_fix_deg'], old['lat_fix_mm'], d['lat_fix_mm'], old['front_fix_mm'], d['front_fix_mm']))


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    a = sub.add_parser('motion'); a.add_argument('dir'); a.add_argument('--calib'); a.add_argument('--min-d', type=float, default=100.0)
    b = sub.add_parser('static'); b.add_argument('dir'); b.add_argument('--calib'); b.add_argument('--pose')
    c = sub.add_parser('write'); c.add_argument('--calib'); c.add_argument('--yaw', type=float); c.add_argument('--lat', type=float)
    c.add_argument('--front', type=float); c.add_argument('--out', required=True)
    args = ap.parse_args()
    {'motion': motion, 'static': static, 'write': write}[args.cmd](args)


if __name__ == '__main__':
    main()
