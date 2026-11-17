"""2026-11-15: 本番当日の床での較正を1本で行う(PCで動かす)。止まって撮る試験の画像から、
  (1) カメラの取り付けの補正(左右の向き・左右の傾き・高さ、floor_fit.py と同じ当てはめ)と、
  (2) スタートの景色で決まって出る読み違い start_fix(前・左・向き)
を求めて、較正ファイルに書く。

順番(大事): 較正台で較正し直した(recalibrate_day.py)あとに行う。recalibrate_day.py は取り付けの補正と start_fix を引き継がない
(較正台で較正し直すとカメラの見え方が変わるので、床での較正はやり直しが要る)。

止まって撮る試験(ゲートは外す、スタートに置く。置き方は数cm ずれてよい。2回以上、うち1回は左に3cm くらいずらすとよい):
    python3 alpha.py right --mission rally-drive --rally-start-pose on --rally-camera-log 1 --rally-camera-exposure 30 \
        --rally-straight-pause-sec 1.5 --rally-straight-pause-labels cal1,cal2,cal3,cal4,cal5 \
        --rally-plan-path robot_program/tests/plan_test_selfcal_stops_1114.json
PC で(2026-Alpha で、ログを取ってきてから):
    python wireless_device/et_rally_camera_calib/day_floor_calib.py <run_logs/et_camera_...> [<2回目> ...] [--set-default]
出力: wireless_device/et_rally_camera_calib/calib_<月日時分>_floor.json(元は今の DEFAULT_CALIB)。--set-default で既定にする。

start_fix の求め方: 当てはめで出た「本当のスタートの姿勢」(エンコーダとジャイロでつないだ止まった場所すべてと合うもの)と、
新しい較正でスタートの画像だけから測った姿勢の差(右コースの車体の座標で 前・左・向き)の、回ごとの平均。
11/15 の3回: 前 -1〜-7mm、左 +16〜+17.5mm、向き -1.24〜-1.39°(回ごとのばらつきが大きいときは警告)。
"""
import argparse
import glob
import json
import math
import os
import re
import sys
from datetime import datetime

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)
from robot_program import et_rally_start_pose as sp  # noqa: E402
import floor_fit as ff  # noqa: E402

START_POSE = os.path.join(ROOT, 'robot_program', 'et_rally_start_pose.py')


def fit_mounting(dirs, calib, still_mm=1.0):
    sp.set_gate_legs(None)
    cam = sp.GroundCamera(calib, top_s=0.5, top_x=sp.START_WIDE_X, top_y=sp.START_WIDE_Y)
    runs = [ff.load_run(d, cam, still_mm=still_mm) for d in dirs]
    for d, run in zip(dirs, runs):
        print('%s: %s' % (os.path.basename(d.rstrip('/\\')), ', '.join('%s(%d点)' % (n, len(f)) for n, _, f in run)))
        if not run or run[0][0] != 'start' or len(run) < 4:
            raise SystemExit('止まった場所かスタートの画像が足りない(スタート+止まった場所3か所以上が要る): %s' % d)
    model = ff.Model(cam, runs)
    p = np.zeros(model.n_params())
    assign = model.associate(p)
    p, r = ff.lm(model, p, assign, fixed=(0, 1, 2))
    assign = model.associate(p)
    p, r0 = ff.lm(model, p, assign, fixed=(0, 1, 2))
    p1, r1 = ff.lm(model, p.copy(), assign)
    assign = model.associate(p1)
    p1, r1 = ff.lm(model, p1, assign)
    print('点の重なりの残り: 今の較正 %.2f mm → 補正後 %.2f mm' % (float(np.sqrt((r0 ** 2).mean())), float(np.sqrt((r1 ** 2).mean()))))
    print('カメラの補正: 左右の向き %+.2f°  左右の傾き %+.2f°  高さ %+.1f mm' % (p1[0], p1[1], p1[2]))
    places = []
    for ri, d in enumerate(dirs):
        P = np.loadtxt(os.path.join(d, 'poses.txt'))
        a = P[0, 1:4]
        places.append((a[0] + p1[3 + 3 * ri], a[1] + p1[4 + 3 * ri], a[2] + p1[5 + 3 * ri]))
    return p1, places


def measure_start(d, calib):
    sp.set_gate_legs(None)
    cam = sp.GroundCamera(calib, top_s=sp.FAST_TOP_S, top_x=sp.START_WIDE_X, top_y=sp.START_WIDE_Y)
    poses = []
    for f in sorted(glob.glob(os.path.join(d, 'start_*.jpg'))):
        r = sp.choose_start_fit(cam, sp.detect_ground(cam, cv2.imread(f)))[0]
        if r:
            poses.append({'pose': r['pose']})
    return sp.median_pose(poses) if poses else None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('dirs', nargs='+', help='止まって撮る試験の run_logs/et_camera_* フォルダ')
    ap.add_argument('--base', default=None, help='元にする較正ファイル(既定: 今の DEFAULT_CALIB)')
    ap.add_argument('--out', default=None)
    ap.add_argument('--set-default', action='store_true')
    ap.add_argument('--still-mm', type=float, default=1.0, help='止まっているとみなす進んだ距離の幅(mm)。止まる間に少し進むロボットでは3')
    a = ap.parse_args()
    base_path = a.base or str(sp.DEFAULT_CALIB)
    base = json.load(open(base_path, encoding='utf-8'))
    # 取り付けの補正は、元のファイルの floor の補正を外した状態から当てはめる(二重に足さない)
    clean = dict(base)
    for k in ('cam_yaw_fix_deg', 'roll_fix_deg', 'height_fix_mm', 'start_fix'):
        clean.pop(k, None)
    tmp = os.path.join(HERE, '_day_floor_tmp.json')
    json.dump(clean, open(tmp, 'w', encoding='utf-8'), ensure_ascii=False)
    try:
        p1, places = fit_mounting(a.dirs, tmp, still_mm=a.still_mm)
        new = dict(clean, cam_yaw_fix_deg=float(p1[0]), roll_fix_deg=float(p1[1]), height_fix_mm=float(p1[2]))
        json.dump(new, open(tmp, 'w', encoding='utf-8'), ensure_ascii=False)
        diffs = []
        for d, truth in zip(a.dirs, places):
            m = measure_start(d, tmp)
            if m is None:
                print('%s: スタートの画像から位置が出ない(start_fix に使わない)' % os.path.basename(d))
                continue
            dx, dy = truth[0] - m[0], truth[1] - m[1]
            hh = math.radians(m[2])
            front = dx * math.cos(hh) + dy * math.sin(hh)
            left = -dx * math.sin(hh) + dy * math.cos(hh)
            dh = ((truth[2] - m[2] + 180) % 360) - 180
            diffs.append((front, left, dh))
            print('%s: スタートの読み違い(本当 − 測った値) 前 %+.1f 左 %+.1f mm 向き %+.2f°' % (os.path.basename(d.rstrip('/\\')), front, left, dh))
    finally:
        try:
            os.remove(tmp)
        except OSError:
            pass
    if not diffs:
        raise SystemExit('start_fix を出せる回がない')
    D = np.array(diffs)
    fix = D.mean(0)
    spread = D.max(0) - D.min(0) if len(D) > 1 else np.zeros(3)
    print('start_fix: 前 %+.1f 左 %+.1f mm 向き %+.2f°(回ごとの幅 %.1f / %.1f mm / %.2f°)' % (fix[0], fix[1], fix[2], *spread))
    warn = []
    if len(D) < 2:
        warn.append('1回だけ(2回以上がおすすめ)')
    if spread[0] > 10 or spread[1] > 8 or spread[2] > 0.6:
        warn.append('回ごとのばらつきが大きい')
    new['start_fix'] = {'front_mm': round(float(fix[0]), 1), 'left_mm': round(float(fix[1]), 1), 'heading_deg': round(float(fix[2]), 2),
                        'note': '%s day_floor_calib.py(%d回)' % (datetime.now().strftime('%Y-%m-%d %H:%M'), len(D))}
    new['note'] = ('%s day_floor_calib.py: 床での較正(取り付けの補正 向き %+.2f° 傾き %+.2f° 高さ %+.1fmm、start_fix)。元: %s。%s' % (
        datetime.now().strftime('%Y-%m-%d %H:%M'), p1[0], p1[1], p1[2], os.path.basename(base_path), ('警告: ' + '; '.join(warn)) if warn else ''))
    out = a.out or os.path.join(HERE, 'calib_%s_floor.json' % datetime.now().strftime('%m%d%H%M'))
    if not os.path.isabs(out):
        out = os.path.join(HERE, out)
    json.dump(new, open(out, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
    print('保存: %s' % out)
    for w in warn:
        print('警告: %s' % w)
    if a.set_default:
        s = open(START_POSE, encoding='utf-8').read()
        s2 = re.sub(r'"et_rally_camera_calib" / "[^"]+\.json"', '"et_rally_camera_calib" / "%s"' % os.path.basename(out), s, count=1)
        open(START_POSE, 'w', encoding='utf-8').write(s2)
        print('DEFAULT_CALIB を %s にした。git で commit・push して、ロボットで git pull してください。' % os.path.basename(out))


if __name__ == '__main__':
    main()
