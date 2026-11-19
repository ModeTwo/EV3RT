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

2026-11-19(ユーザー: 別のPC・ラズパイで、数字のやり取りも git もなしで済ませたい): ロボットの上で1本で済ませる --auto。
    python3 wireless_device/et_rally_camera_calib/day_floor_calib.py --auto        # 新しい止まって撮る試験(最大2回)を使う
    python3 wireless_device/et_rally_camera_calib/day_floor_calib.py --auto --runs 1
    python3 wireless_device/et_rally_camera_calib/day_floor_calib.py --clear       # 使うのをやめる(元の較正に戻す)
  run_logs/et_camera_* のうち、止まって撮る試験(poses.txt とスタートの画像がある)の新しいものを自動で選ぶ。
  合格(取り付けの補正・start_fix が今までの範囲で、回ごとにそろっている)なら LOCAL_CALIB.txt に名前を書き、
  次の走行から自動で使う(alpha.py のログに「ET rally camera calibration: calib_..._floor.json (LOCAL_CALIB.txt)」)。
  不合格なら使わない(今の較正のまま)。

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


def _use_plain_python():
    """2026-11-19: ロボットの python3 は PyPy で、cv2 がない・numpy がとても遅い。普通の Python(/usr/bin/python3、
    カメラの子と同じ)で動かし直す。LD_LIBRARY_PATH などはカメラの子(et_rally_camera_proc.child_env)と同じく外す。"""
    import platform
    if platform.python_implementation() != 'PyPy' or os.environ.get('_DAY_FLOOR_CALIB_REEXEC'):
        return
    plain = os.environ.get('ET_CAMERA_PYTHON') or '/usr/bin/python3'
    if not os.path.exists(plain):
        return
    env = dict(os.environ)
    for name in ('PYTHONPATH', 'PYTHONHOME', 'LD_LIBRARY_PATH'):
        env.pop(name, None)
    env['_DAY_FLOOR_CALIB_REEXEC'] = '1'
    print('(PyPy なので %s で動かし直す)' % plain, flush=True)
    os.execve(plain, [plain, os.path.abspath(__file__)] + sys.argv[1:], env)


_use_plain_python()
if '/usr/lib/python3/dist-packages' not in sys.path and os.path.isdir('/usr/lib/python3/dist-packages'):
    sys.path.append('/usr/lib/python3/dist-packages')   # apt の cv2・numpy(カメラの子と同じ)
import cv2  # noqa: E402
import numpy as np  # noqa: E402

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
    return p1, places, float(np.sqrt((r1 ** 2).mean()))


# 2026-11-19: --auto の合格の範囲。今までの床での較正(11/15〜11/18): 取り付けの補正 向き 0.0〜0.5°・傾き -0.4〜-0.6°・
# 高さ 1〜4mm、start_fix 前 -1〜-7・左 +15〜+17.5mm・向き -1.2〜-1.4°、点の重なりの残り 補正後 2〜4mm。
# それより十分広く取り、置き間違い・撮り間違い(大きく外れた値)だけを弾く。
LIMIT_MOUNT_DEG = 2.0       # 取り付けの左右の向き・傾き
LIMIT_HEIGHT_MM = 10.0
LIMIT_FIX_FRONT_MM = 20.0
LIMIT_FIX_LEFT_MM = 30.0
LIMIT_FIX_HEADING_DEG = 2.5
LIMIT_SPREAD = (10.0, 8.0, 0.6)   # 回ごとの幅(前・左 mm、向き °)。2回以上のとき
LIMIT_RESID_MM = 8.0


def _judge(p1, fix, spread, n, resid):
    why = []
    if abs(p1[0]) > LIMIT_MOUNT_DEG or abs(p1[1]) > LIMIT_MOUNT_DEG:
        why.append('カメラの取り付けの補正が大きい(向き %+.2f° 傾き %+.2f°、%.1f°まで)' % (p1[0], p1[1], LIMIT_MOUNT_DEG))
    if abs(p1[2]) > LIMIT_HEIGHT_MM:
        why.append('カメラの高さの補正が大きい(%+.1fmm、%.0fmmまで)' % (p1[2], LIMIT_HEIGHT_MM))
    if abs(fix[0]) > LIMIT_FIX_FRONT_MM or abs(fix[1]) > LIMIT_FIX_LEFT_MM or abs(fix[2]) > LIMIT_FIX_HEADING_DEG:
        why.append('スタートの読み違いが大きい(前 %+.1f 左 %+.1f mm 向き %+.2f°)' % tuple(fix))
    if n >= 2 and (spread[0] > LIMIT_SPREAD[0] or spread[1] > LIMIT_SPREAD[1] or spread[2] > LIMIT_SPREAD[2]):
        why.append('回ごとのばらつきが大きい(幅 %.1f / %.1f mm / %.2f°)' % tuple(spread))
    if resid > LIMIT_RESID_MM:
        why.append('点の重なりが合わない(残り %.1fmm、%.0fmmまで)' % (resid, LIMIT_RESID_MM))
    return (not why), why


def _repo_default():
    """git で配った既定の較正(et_rally_start_pose.py に書いてある名前)。"""
    s = open(START_POSE, encoding='utf-8').read()
    m = re.search(r'"et_rally_camera_calib" / "([^"]+\.json)"', s)
    return os.path.join(HERE, m.group(1)) if m else str(sp.DEFAULT_CALIB)


def _find_selfcal_runs(n):
    """run_logs/et_camera_* のうち、止まって撮る試験(poses.txt・スタートの画像・止まった所の画像)の新しい順 n 個。"""
    cands = []
    for d in glob.glob(os.path.join(ROOT, 'run_logs', 'et_camera_*')):
        if not os.path.isdir(d) or not os.path.exists(os.path.join(d, 'poses.txt')):
            continue
        if not glob.glob(os.path.join(d, 'start_*.jpg')):
            continue
        if len(glob.glob(os.path.join(d, '*.jpg'))) < 10:
            continue
        # 止まって撮る試験だけ(ふつうの走行で --rally-camera-log を付けた回を選ばない): 進んだのが 1.2m 未満で、
        # 走り出してから止まった所が3か所以上(止まる間に少し進むロボットもあるので 3mm の幅で数える)
        try:
            P = np.loadtxt(os.path.join(d, 'poses.txt'))
            if P.ndim != 2:
                continue
            P[:, 4] -= P[0, 4]   # full で走ると LAP からの距離が足されているので、ETラリーのスタートからにする
            if P[-1, 4] > 1200.0:
                continue
            if sum(1 for _, _, i in ff.static_intervals(P, still_mm=3.0) if P[i, 4] >= 1.0) < 3:
                continue
        except (OSError, ValueError, IndexError):
            continue
        cands.append((os.path.getmtime(d), d))
    import time
    old_n = len(cands)
    cands = [c for c in cands if time.time() - c[0] < 8 * 3600]   # 8時間より前の回(前の日・別のロボット)は使わない
    if old_n and not cands:
        raise SystemExit('止まって撮る試験は見つかったが、どれも8時間より前のもの(今日の回を走らせてください)')
    cands.sort(reverse=True)
    dirs = [d for _, d in cands[:n]]
    if not dirs:
        raise SystemExit('止まって撮る試験のフォルダが run_logs に見つからない(--rally-camera-log 1 を付けて走らせたか確認)')
    print('使う回: %s' % ', '.join(os.path.basename(d) for d in dirs))
    return dirs


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
    ap.add_argument('dirs', nargs='*', help='止まって撮る試験の run_logs/et_camera_* フォルダ(--auto なら省略)')
    ap.add_argument('--auto', action='store_true', help='新しい止まって撮る試験を自動で選び、合格なら LOCAL_CALIB.txt で次から使う')
    ap.add_argument('--runs', type=int, default=2, help='--auto で使う回数(新しい順、既定2)')
    ap.add_argument('--clear', action='store_true', help='LOCAL_CALIB.txt を消して元の較正に戻す')
    ap.add_argument('--base', default=None, help='元にする較正ファイル(既定: 今の DEFAULT_CALIB)')
    ap.add_argument('--out', default=None)
    ap.add_argument('--set-default', action='store_true')
    ap.add_argument('--still-mm', type=float, default=1.0, help='止まっているとみなす進んだ距離の幅(mm)。止まる間に少し進むロボットでは3')
    a = ap.parse_args()
    if a.clear:
        if sp.LOCAL_CALIB_FILE.exists():
            sp.LOCAL_CALIB_FILE.unlink()
            print('LOCAL_CALIB.txt を消した。次の走行から元の較正 %s を使う。' % os.path.basename(str(_repo_default())))
        else:
            print('LOCAL_CALIB.txt はない(元の較正のまま)。')
        return
    if a.auto:
        if not a.dirs:
            a.dirs = _find_selfcal_runs(a.runs)
        # 元は git で配った既定(LOCAL_CALIB.txt で前に選んだ床の較正の上に重ねない)
        if a.base is None:
            a.base = str(_repo_default())
    if not a.dirs:
        raise SystemExit('止まって撮る試験のフォルダを指定してください(または --auto)')
    base_path = a.base or str(sp.DEFAULT_CALIB)
    base = json.load(open(base_path, encoding='utf-8'))
    # 取り付けの補正は、元のファイルの floor の補正を外した状態から当てはめる(二重に足さない)
    clean = dict(base)
    for k in ('cam_yaw_fix_deg', 'roll_fix_deg', 'height_fix_mm', 'start_fix'):
        clean.pop(k, None)
    tmp = os.path.join(HERE, '_day_floor_tmp.json')
    json.dump(clean, open(tmp, 'w', encoding='utf-8'), ensure_ascii=False)
    try:
        try:
            p1, places, r_after = fit_mounting(a.dirs, tmp, still_mm=a.still_mm)
        except SystemExit:
            if not a.auto or a.still_mm >= 3.0:
                raise
            # ロボットによっては止まる間に1〜3mm 進み、止まった場所が足りなくなる(11/18 ロボットA)。幅を 3mm にしてやり直す
            print('止まった場所が足りないので、止まっているとみなす幅を 3mm にしてやり直す')
            a.still_mm = 3.0
            p1, places, r_after = fit_mounting(a.dirs, tmp, still_mm=a.still_mm)
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
    if a.auto:
        ok, why = _judge(p1, fix, spread, len(D), r_after)
        if ok:
            sp.LOCAL_CALIB_FILE.write_text(os.path.basename(out) + '\n', encoding='utf-8')
            print('')
            print('=== 合格: 次の走行から %s を自動で使う(LOCAL_CALIB.txt)。コマンドに何も付けなくてよい ===' % os.path.basename(out))
            print('    走ったログの最初に「ET rally camera calibration: %s (LOCAL_CALIB.txt)」と出る。' % os.path.basename(out))
            if warn:
                print('    (参考: %s)' % '; '.join(warn))
        else:
            print('')
            print('=== 不合格: この較正は使わない(今の較正のまま走る) ===')
            for w in why:
                print('    理由: %s' % w)
            print('    対処: 止まって撮る試験をもう1回走らせて、もう一度 --auto。直らなければこのまま走ってよい(床の補正なし)。')
    if a.set_default:
        s = open(START_POSE, encoding='utf-8').read()
        s2 = re.sub(r'"et_rally_camera_calib" / "[^"]+\.json"', '"et_rally_camera_calib" / "%s"' % os.path.basename(out), s, count=1)
        open(START_POSE, 'w', encoding='utf-8').write(s2)
        print('DEFAULT_CALIB を %s にした。git で commit・push して、ロボットで git pull してください。' % os.path.basename(out))


if __name__ == '__main__':
    main()
