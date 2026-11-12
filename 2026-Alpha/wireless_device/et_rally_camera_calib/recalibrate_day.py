"""2026-11-11: 本番当日など、カメラの角度を直したあとの較正し直しを1本で行う(PCで動かす)。

1. ロボットで較正台の画像を撮る(1枚ごとに置き直して3枚):
     cd ~/EV3RT/2026-Alpha && python3 et_rally_camera_capture.py --tag jig --count 1
2. PCで:
     python wireless_device/et_rally_camera_calib/recalibrate_day.py
   - ロボットから、新しい順に --count 個の *_jig フォルダの画像を取ってくる(ホストは fetch_run_logs のキャッシュ)。
     --local フォルダを指定すれば、取ってこずにその中の *.png を使う。
   - レンズ(K, dist)と較正台の置き方(jig_setup)、横のずれの補正(lat_fix_mm)、床との傾きの補正(pitch_fix_deg)は
     --base の較正ファイル(既定: 今ロボットが使っているもの)から引き継ぐ。レンズは計算し直さない。
     下向きは、走るたびにスタート位置の取り込みで当てはめ直し、走行中の補正にも使う(ee49f4d)ので、
     pitch_fix_deg が少しずれていても影響は小さい。
   - 画像ごとにカメラの位置と向きを出し、下向きが真ん中の1枚を先頭(ロボットが使う jig[0])にして保存する。
   - ばらつきが大きければ警告する(置き直しが雑・ボードがずれた・画像がぶれた)。
   - --set-default で、robot_program/et_rally_start_pose.py の DEFAULT_CALIB を新しいファイルに書き換える。
3. git で commit・push し、ロボットで git pull する。
4. 床で確かめる(2026-11-12 追加。較正台の下向きは撮るたびに約1°ずれることがあり、床で確かめないと危ない):
   スタート位置にまっすぐ置いて(ものさしは不要)、アームを動かすだけの計画で取り込みを1回する:
     python alpha.py right --mission rally-drive --rally-gyro-hub-recal-sec 2 --rally-gyro-drift-settle-sec 1.0 --rally-start-pose on --rally-camera-log 5 --rally-camera-exposure 30 --rally-arm-stop-ticks 3 --rally-arm-power 40 --rally-arm-travel-deg 260 --rally-plan-path robot_program/tests/plan_test_arm_only_1111.json
   ログの "start pose:" で、点が1つ以上見えていて(dots>=1)、計画のスタートからのずれ(plan start ...)が数mm〜1cm程度なら合格。
   数cmずれる・位置が出ないときは、較正台を置き直して撮り直す(このときは --set-default で書き換えた分を元に戻す)。
   合格した回の下向き(pitch_fix)はロボットが run_logs/et_rally_last_pitch.json に覚え、点が見えない回の走行中の補正に使う。
"""
import argparse
import glob
import json
import os
import re
import subprocess
import sys
import tempfile
from datetime import datetime

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, HERE)
import calibrate  # noqa: E402

START_POSE = os.path.join(ROOT, 'robot_program', 'et_rally_start_pose.py')
SPREAD_WARN = dict(pitch_down_deg=0.3, yaw_left_deg=0.5, roll_deg=0.3, cam_mm=3.0)


def current_default():
    s = open(START_POSE, encoding='utf-8').read()
    m = re.search(r'"et_rally_camera_calib" / "([^"]+\.json)"', s)
    return os.path.join(HERE, m.group(1)) if m else None


def robot_host():
    try:
        d = json.load(open(os.path.expanduser('~/.et_run_logs_robot.json'), encoding='utf-8'))
        return d.get('last')
    except Exception:
        return None


def fetch(host, count, dest):
    out = subprocess.run(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10', host,
                          'ls -dt ~/EV3RT/2026-Alpha/camera_calib/*_jig | head -%d' % count],
                         capture_output=True, text=True, timeout=60)
    folders = [l.strip() for l in out.stdout.splitlines() if l.strip().endswith('_jig')]
    if not folders:
        sys.exit('ロボットに *_jig フォルダが見つからない(%s)' % out.stderr.strip())
    n = 0
    for f in folders:
        for name in ('jig_001.png', 'jig_002.png', 'jig_003.png'):
            r = subprocess.run(['scp', '-q', '-o', 'BatchMode=yes', '%s:%s/%s' % (host, f, name),
                                os.path.join(dest, '%s_%s' % (os.path.basename(f), name))],
                               capture_output=True, text=True, timeout=60)
            if r.returncode == 0:
                n += 1
    print('ロボットから %d 枚(%s)' % (n, ', '.join(os.path.basename(f) for f in folders)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--count', type=int, default=3, help='取ってくる *_jig フォルダの数(新しい順)')
    ap.add_argument('--local', help='取ってこずに、このフォルダの *.png を使う')
    ap.add_argument('--host', default=None, help='ロボット(既定: fetch_run_logs のキャッシュ)')
    ap.add_argument('--base', default=None, help='レンズ・置き方・補正を引き継ぐ較正ファイル(既定: 今の DEFAULT_CALIB)')
    ap.add_argument('--out', default=None, help='保存するファイル名(既定: calib_<月日時分>.json)')
    ap.add_argument('--set-default', action='store_true', help='DEFAULT_CALIB を新しいファイルに書き換える')
    a = ap.parse_args()

    base_path = a.base or current_default()
    base = json.load(open(base_path, encoding='utf-8'))
    K = np.array(base['lens']['K']); dist = np.array(base['lens']['dist'])
    js = base.get('jig_setup', {})
    d1l, d1r = js.get('d1', [603.0, 603.0]); d2 = js.get('d2', 179.0); d3 = js.get('d3', 0.0)
    sq = js.get('sq', 25.0); h = js.get('h', 0.0)
    print('引き継ぎ元: %s(較正台 D1 %.0f/%.0f、D2 %.0f、D3 %.0f、h %.1f、横の補正 %+.1fmm、下向きの補正 %+.2f°)' % (
        os.path.basename(base_path), d1l, d1r, d2, d3, h, base.get('lat_fix_mm', 0.0), base.get('pitch_fix_deg', 0.0)))

    folder = a.local
    if folder is None:
        folder = tempfile.mkdtemp(prefix='jig_')
        fetch(a.host or robot_host(), a.count, folder)
    J = calibrate.jig(folder, K, dist, sq, d1l, d1r, d2, d3, h)
    if len(J) < 2:
        sys.exit('ボードが見つかった画像が %d 枚しかない。撮り直してください。' % len(J))
    warn = []
    for key in ('pitch_down_deg', 'yaw_left_deg', 'roll_deg'):
        v = [j[key] for j in J]
        sp = max(v) - min(v)
        print('  %s: 真ん中 %.2f、ばらつき %.2f' % (key, float(np.median(v)), sp))
        if sp > SPREAD_WARN[key]:
            warn.append('%s のばらつき %.2f > %.2f' % (key, sp, SPREAD_WARN[key]))
    cams = np.array([j['cam_mm'] for j in J])
    cs = float(np.max(cams.max(0) - cams.min(0)))
    print('  カメラ位置 前%.1f 左%.1f 高さ%.1f、ばらつき %.1fmm' % (*np.median(cams, 0), cs))
    if cs > SPREAD_WARN['cam_mm']:
        warn.append('カメラ位置のばらつき %.1fmm > %.1f' % (cs, SPREAD_WARN['cam_mm']))
    # 下向きが真ん中の1枚を先頭に(ロボットは jig[0] を使う)
    order = sorted(range(len(J)), key=lambda i: J[i]['pitch_down_deg'])
    mid = order[len(order) // 2]
    J = [J[mid]] + [J[i] for i in range(len(J)) if i != mid]
    b0 = base['jig'][0]
    print('引き継ぎ元との差: 下向き %+.2f°、左向き %+.2f°、傾き %+.2f°' % (
        J[0]['pitch_down_deg'] - b0['pitch_down_deg'], J[0]['yaw_left_deg'] - b0['yaw_left_deg'], J[0]['roll_deg'] - b0['roll_deg']))
    out = a.out or ('calib_%s.json' % datetime.now().strftime('%m%d%H%M'))
    path = out if os.path.isabs(out) else os.path.join(HERE, out)
    d = {'lens': base['lens'], 'jig': J, 'jig_setup': base.get('jig_setup'),
         'pitch_fix_deg': base.get('pitch_fix_deg', 0.0), 'lat_fix_mm': base.get('lat_fix_mm', 0.0),
         'note': '%s recalibrate_day.py: 較正台 %d 枚(下向きが真ん中の %s を先頭)、レンズ・置き方・補正は %s から引き継ぎ。%s' % (
             datetime.now().strftime('%Y-%m-%d %H:%M'), len(J), J[0]['file'], os.path.basename(base_path),
             ('警告: ' + '; '.join(warn)) if warn else '')}
    json.dump(d, open(path, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
    print('保存: %s' % path)
    for w in warn:
        print('警告: %s(置き直して撮り直すのがおすすめ)' % w)
    if a.set_default:
        s = open(START_POSE, encoding='utf-8').read()
        s2 = re.sub(r'"et_rally_camera_calib" / "[^"]+\.json"', '"et_rally_camera_calib" / "%s"' % os.path.basename(path), s, count=1)
        open(START_POSE, 'w', encoding='utf-8').write(s2)
        print('DEFAULT_CALIB を %s にした。git で commit・push して、ロボットで git pull してください。' % os.path.basename(path))


if __name__ == '__main__':
    main()
