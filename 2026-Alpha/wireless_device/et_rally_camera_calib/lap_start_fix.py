"""2026-11-18: 周回の試走のログから start_fix(スタートの画像で決まって出る読み違い)を直す(PCで動かす)。

止まって撮る試験(day_floor_calib.py)をスタートの行でできないときの代わり。スタートには目視で置いてよい
(置き方のずれは毎回スタートの画像で読むので効かない)。
走り出してすぐの「camera correct: diff」(走りながらカメラで読んだ位置 − 推定の位置)に、それまでに直した分(total)を
足し戻すと、スタートでの読み違いの残りになる。スタートの向きの読み違いは、進んだ距離に比例して横のずれにもなるので、
向き → 位置の順に分けて求める。回ごとの値の平均を、その回で使っていた start_fix に足す。

前後(front)は走りながら読むと前へ4〜6mm出る癖があり当てにならないので、既定では元の値のまま(--use-front で使う)。

使い方(2026-Alpha で、ロボットのログを取ってきてから):
    scp shunsuke09@10.56.69.191:EV3RT/2026-Alpha/run_logs/run_<日時>_*.log <置き場>/
    python wireless_device/et_rally_camera_calib/lap_start_fix.py <置き場>/run_*.log
    python wireless_device/et_rally_camera_calib/lap_start_fix.py <置き場>/run_*.log --out calib_C_lapfix.json [--set-default]
--out を付けたときだけ較正ファイルを書く(元はログで使っていた較正ファイル)。
"""
import argparse
import ast
import json
import math
import os
import re
import sys
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
START_POSE = os.path.join(ROOT, 'robot_program', 'et_rally_start_pose.py')

RE_T = re.compile(r' \+([0-9.]+)s ')
RE_START = re.compile(r'start pose: x=([-0-9.]+) y=([-0-9.]+) h=([-+0-9.]+) deg')
RE_FIX = re.compile(r'start_fix front ([-+0-9.]+) left ([-+0-9.]+) mm heading ([-+0-9.]+) deg applied')
RE_DIFF = re.compile(r'camera correct: diff \(([-+0-9.]+),([-+0-9.]+) mm ([-+0-9.]+) deg\) items D(\d+) M(\d+) resid ([0-9.]+)'
                     r' -> applied \(([-+0-9.]+),([-+0-9.]+) ([-+0-9.]+)\) total \(([-+0-9.]+),([-+0-9.]+) ([-+0-9.]+)\)')
RE_DIST = re.compile(r'RALLY_PWR source=\S+ dist=([-0-9.]+)')


def default_calib_name():
    s = open(START_POSE, encoding='utf-8').read()
    m = re.search(r'"et_rally_camera_calib" / "([^"]+\.json)"', s)
    return m.group(1) if m else None


def parse(path):
    run = dict(path=path, course=None, calib=None, start=None, fix=None, diffs=[], drive_dist=None)
    last_dist = None
    for line in open(path, encoding='utf-8', errors='replace'):
        if '[session] argv=' in line and run['course'] is None:
            try:
                argv = ast.literal_eval(line.split('argv=', 1)[1].strip())
                run['course'] = 'left' if 'left' in argv[1:2] else 'right'
                if '--rally-camera-calib' in argv:
                    run['calib'] = os.path.basename(argv[argv.index('--rally-camera-calib') + 1])
            except (ValueError, SyntaxError, IndexError):
                pass
            continue
        if run['start'] is None:
            m = RE_START.search(line)
            if m:
                run['start'] = tuple(float(v) for v in m.groups())
                continue
        if run['start'] is not None and run['fix'] is None:
            m = RE_FIX.search(line)
            if m:
                run['fix'] = tuple(float(v) for v in m.groups())
                continue
        if run['start'] is None:
            continue
        m = RE_DIST.search(line)
        if m:
            last_dist = float(m.group(1))
            if run['drive_dist'] is None:
                run['drive_dist'] = last_dist
            continue
        m = RE_DIFF.search(line)
        if m:
            t = RE_T.search(line)
            v = [float(x) for x in m.groups()]
            run['diffs'].append(dict(t=float(t.group(1)) if t else None, ex=v[0], ey=v[1], eh=v[2], nd=int(v[3]), nm=int(v[4]),
                                     resid=v[5], ax=v[6], ay=v[7], ah=v[8], tx=v[9], ty=v[10], th=v[11],
                                     dist=None if last_dist is None or run['drive_dist'] is None else last_dist - run['drive_dist']))
    return run


def estimate(run, window_sec, max_dist):
    """走り出してすぐの読み直しから、スタートの読み違いの残り (前, 左, 向き)(右コースの車体の座標)を求める。"""
    d = run['diffs']
    if not d or run['start'] is None:
        return None
    t0 = d[0]['t']
    use = [x for x in d if x['t'] is not None and x['t'] - t0 <= window_sec and (x['dist'] is None or x['dist'] <= max_dist)]
    if len(use) < 3:
        return None
    # 直す前の残り = 読み直しの差 + それまでに直した分
    heads = [x['eh'] + (x['th'] - x['ah']) for x in use]
    heads.sort()
    dh = heads[len(heads) // 2]
    h0 = math.radians(run['start'][2])
    r = math.radians(dh)
    ex, ey = [], []
    for x in use:
        px = x['ex'] + (x['tx'] - x['ax'])
        py = x['ey'] + (x['ty'] - x['ay'])
        s = x['dist'] or 0.0
        vx, vy = s * math.cos(h0), s * math.sin(h0)     # 進んだ分(向きはスタートの向きで近似)
        ex.append(px + r * vy)                          # 向きの読み違いで出た横のずれを除く
        ey.append(py - r * vx)
    ex.sort()
    ey.sort()
    mx, my = ex[len(ex) // 2], ey[len(ey) // 2]
    front = mx * math.cos(h0) + my * math.sin(h0)
    left = -mx * math.sin(h0) + my * math.cos(h0)
    if run['course'] == 'left':
        left, dh = -left, -dh
    spread = (max(heads) - min(heads))
    return dict(n=len(use), front=front, left=left, heading=dh, head_spread=spread,
                dist=max((x['dist'] or 0.0) for x in use))


def main():
    ap = argparse.ArgumentParser(description='周回の試走のログから start_fix を直す')
    ap.add_argument('logs', nargs='+')
    ap.add_argument('--window-sec', type=float, default=2.0, help='最初の読み直しから何秒までを使うか(既定2.0)')
    ap.add_argument('--max-dist', type=float, default=700.0, help='走り出してから何mmまでを使うか(既定700)')
    ap.add_argument('--use-front', action='store_true', help='前後も直す(既定は元の値のまま)')
    ap.add_argument('--out', help='書き出す較正ファイル名(付けたときだけ書く)')
    ap.add_argument('--set-default', action='store_true')
    a = ap.parse_args()

    rows, calibs = [], set()
    for p in a.logs:
        run = parse(p)
        name = os.path.basename(p)
        if run['start'] is None or run['fix'] is None:
            print('%s: スタートの姿勢か start_fix の行がない(使わない)' % name)
            continue
        e = estimate(run, a.window_sec, a.max_dist)
        if e is None:
            print('%s: 走り出しの読み直しが3枚未満(使わない)' % name)
            continue
        calib = run['calib'] or default_calib_name()
        calibs.add(calib)
        f = run['fix']
        new = (f[0] + e['front'], f[1] + e['left'], f[2] + e['heading'])
        rows.append(new)
        print('%s: %s %s 使った start_fix 前%+.1f 左%+.1f 向き%+.2f / 残り 前%+.1f 左%+.1f mm 向き%+.2f°(%d枚、%.0fmmまで、向きの幅%.2f°)'
              ' → 前%+.1f 左%+.1f 向き%+.2f'
              % (name, run['course'], calib, f[0], f[1], f[2], e['front'], e['left'], e['heading'], e['n'], e['dist'],
                 e['head_spread'], new[0], new[1], new[2]))
    if not rows:
        raise SystemExit('使える回がない')
    n = len(rows)
    mean = [sum(r[i] for r in rows) / n for i in range(3)]
    sd = [math.sqrt(sum((r[i] - mean[i]) ** 2 for r in rows) / (n - 1)) if n > 1 else float('nan') for i in range(3)]
    print('平均(%d回): 前 %+.1f 左 %+.1f mm 向き %+.2f°  回ごとのばらつき(標準偏差) %.1f / %.1f mm / %.2f°'
          % (n, mean[0], mean[1], mean[2], *sd))
    if len(calibs) != 1:
        print('警告: 回によって較正ファイルが違う: %s' % ', '.join(sorted(str(c) for c in calibs)))
    if n < 3:
        print('警告: %d回だけ(3回以上がおすすめ)' % n)
    if not a.out:
        return
    base_name = sorted(str(c) for c in calibs)[0]
    base_path = os.path.join(HERE, base_name)
    new = json.load(open(base_path, encoding='utf-8'))
    old = new.get('start_fix') or {}
    front = mean[0] if a.use_front else float(old.get('front_mm', 0.0))
    new['start_fix'] = {'front_mm': round(front, 1), 'left_mm': round(mean[1], 1), 'heading_deg': round(mean[2], 2),
                        'note': '%s lap_start_fix.py(周回のログ%d回、前後は%s)' % (
                            datetime.now().strftime('%Y-%m-%d %H:%M'), n, '直した' if a.use_front else '元のまま')}
    new['note'] = '%s lap_start_fix.py: start_fix を周回のログ%d回から直した。元: %s。%s' % (
        datetime.now().strftime('%Y-%m-%d %H:%M'), n, base_name, new.get('note', ''))
    out = a.out if os.path.isabs(a.out) else os.path.join(HERE, a.out)
    json.dump(new, open(out, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
    print('保存: %s(start_fix 前 %+.1f 左 %+.1f 向き %+.2f、元は 前 %+.1f 左 %+.1f 向き %+.2f)' % (
        out, front, mean[1], mean[2], float(old.get('front_mm', 0.0)), float(old.get('left_mm', 0.0)), float(old.get('heading_deg', 0.0))))
    if a.set_default:
        s = open(START_POSE, encoding='utf-8').read()
        s2 = re.sub(r'"et_rally_camera_calib" / "[^"]+\.json"', '"et_rally_camera_calib" / "%s"' % os.path.basename(out), s, count=1)
        open(START_POSE, 'w', encoding='utf-8').write(s2)
        print('DEFAULT_CALIB を %s にした。git で commit・push して、ロボットで git pull してください。' % os.path.basename(out))


if __name__ == '__main__':
    main()
