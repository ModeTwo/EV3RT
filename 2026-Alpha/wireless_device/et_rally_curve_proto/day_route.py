"""2026-11-12: 本番当日に、ゲートの配置から実機の計画ファイルまでを1本で作る(PCで動かす)。

使い方(右コース、公式の座標 GX-Y で各ゲートの2つの点):
    python wireless_device/et_rally_curve_proto/day_route.py --red G2-1,G3-1 --blue G1-2,G1-3 --yellow G3-2,G4-2 \
        --name day1 [--full-sec 300]

- 1段目: params_day.json(11/11の50配置と同じ探索の設定。1配置おおむね10〜60秒)で経路を作り、計画にする。
- 2段目(--full-sec 秒 を付けたとき): 斜めを±45°・5°刻みにし、遠回りの探索も長くして作り直す。狭い配置
  (脚とのすき間を3cm取れない配置)は、1段目で決まったすき間のまま斜めを全部使う(配置36ではこれで、
  黄の手前のその場旋回が要らない経路が見つかった)。指定した秒で打ち切り、道具の見込みが1段目より短いときだけ差し替える。
  1配置 数十秒〜6分ほど(PCの負荷で変わる)。
- 出力: robot_program/tests/plan_<name>.json と、走らせるコマンド。

座標: 道具の格子 (c, r) と公式の GX-Y は c = 5 - X、r = 5 - Y(右コースの図: Y1 が上、X5 が左)。
左コースの図は左右が逆(X1 が左)なので、公式の座標のまま入れれば同じ計画になり、走るときに向きの符号だけ逆になる
(--course left で表示するコマンドが left になる)。11/11 にユーザーと確認: seed 9392783 = 赤 G3-3/G4-3、
青 G2-3/G2-4、黄 G2-1/G3-1 → 道具の座標 赤(2,2)(1,2)・青(3,2)(3,1)・黄(3,4)(2,4)。
"""
import argparse
import json
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
TESTS = os.path.join(ROOT, 'robot_program', 'tests')


def parse_point(s):
    m = re.fullmatch(r'\s*G?(\d)-(\d)\s*', s, re.I)
    if not m:
        raise SystemExit('点は G<X>-<Y> の形で(例 G2-1): %r' % s)
    X, Y = int(m.group(1)), int(m.group(2))
    return [5 - X, 5 - Y]


def parse_gate(s):
    a, b = (parse_point(p) for p in s.split(','))
    if abs(a[0] - b[0]) + abs(a[1] - b[1]) != 1:
        raise SystemExit('ゲートの2つの点はとなり合う格子の点にしてください: %s' % s)
    return {'a': a, 'b': b}


def run_tool(gates_path, params, out_path, log_path, timeout):
    env = dict(os.environ, PYTHONUTF8='1', OMP_NUM_THREADS='1', NUMBA_NUM_THREADS='1')
    pf = out_path + '.params.json'
    json.dump(params, open(pf, 'w', encoding='utf-8'), ensure_ascii=False)
    env['HR_PARAMS'] = pf
    t0 = time.time()
    try:
        with open(log_path, 'w', encoding='utf-8') as log:
            subprocess.run([sys.executable, os.path.join(HERE, 'hybrid_route.py'), gates_path, out_path],
                           env=env, stdout=log, stderr=subprocess.STDOUT, timeout=timeout, check=True)
    except subprocess.TimeoutExpired:
        return None, time.time() - t0
    except subprocess.CalledProcessError:
        return None, time.time() - t0
    return json.load(open(out_path, encoding='utf-8')), time.time() - t0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--red', required=True)
    ap.add_argument('--blue', required=True)
    ap.add_argument('--yellow', required=True)
    ap.add_argument('--name', default=time.strftime('day_%m%d%H%M'))
    ap.add_argument('--course', choices=('right', 'left'), default='right')
    ap.add_argument('--full-sec', type=float, default=0.0, help='2段目に使う秒(0なら1段目だけ)')
    ap.add_argument('--first-sec', type=float, default=600.0, help='1段目の打ち切り(秒)')
    ap.add_argument('--budget-sec', type=float, default=70.0,
                    help='持ち時間(秒)。切り上げの道が作れなかったとき、全部の周がこれに収まる周回数で作り直す')
    a = ap.parse_args()

    work = os.path.join(ROOT, 'run_logs', 'day_route', a.name)
    os.makedirs(work, exist_ok=True)
    gates = {'course': 'right', 'red': parse_gate(a.red), 'blue': parse_gate(a.blue), 'yellow': parse_gate(a.yellow)}
    gp = os.path.join(work, 'gates.json')
    json.dump({'gates': gates}, open(gp, 'w', encoding='utf-8'))
    base = json.load(open(os.path.join(HERE, 'params_day.json'), encoding='utf-8'))

    r1, t1 = run_tool(gp, base, os.path.join(work, 'r1.json'), os.path.join(work, 'log1.txt'), a.first_sec)
    if r1 is None:
        raise SystemExit('1段目で経路が作れなかった(%s を見てください)' % os.path.join(work, 'log1.txt'))
    print('1段目: 道具の見込み %.1f秒、すき間 %.1fcm、計算 %.0f秒' % (r1['total'], r1.get('margin') or 0, t1))
    best, best_path = r1, os.path.join(work, 'r1.json')

    if a.full_sec > 0:
        full = dict(base)
        full.update(angles=[float(v) for v in range(-45, 50, 5)], angle_scan=[-45.0, 45.0, 1.0],
                    via_always=True, via_budget_sec=max(30.0, a.full_sec * 0.4))
        if r1.get('narrow') and r1.get('margin'):
            full['margin_list'] = [float(r1['margin'])]   # 狭い配置: 決まったすき間のまま、斜めを全部使う
        r2, t2 = run_tool(gp, full, os.path.join(work, 'r2.json'), os.path.join(work, 'log2.txt'), a.full_sec)
        if r2 is None:
            print('2段目: %.0f秒で終わらなかった(1段目を使う)' % t2)
        else:
            print('2段目: 道具の見込み %.1f秒、すき間 %.1fcm、計算 %.0f秒' % (r2['total'], r2.get('margin') or 0, t2))
            if r2['total'] < r1['total'] - 0.2:
                best, best_path = r2, os.path.join(work, 'r2.json')

    # 2026-11-12: 途中の周で切り上げる道(exit)がない経路は、ロボットが時間で切り上げられず全部の周を走る。
    # そのときは、全部の周が持ち時間に収まる周回数まで減らして作り直す(実機は道具の見込みの約1.08倍、余裕3秒)。
    if not best.get('exit'):
        print('注意: 途中の周でガレージへ切り上げる道が作れなかった。持ち時間 %.0f 秒に収まる周回数で作り直す' % a.budget_sec)
        for laps in range(int(base.get('laps', 10)) - 1, 2, -1):
            p = dict(json.load(open(best_path + '.params.json', encoding='utf-8')), laps=laps)
            rr, tt = run_tool(gp, p, os.path.join(work, 'r_l%d.json' % laps), os.path.join(work, 'log_l%d.txt' % laps), a.first_sec)
            if rr is None:
                continue
            est = rr['total'] * 1.08
            print('  %d周: 道具の見込み %.1f秒(実機の目安 %.1f秒)%s' % (laps, rr['total'], est, '、切り上げの道あり' if rr.get('exit') else ''))
            if rr.get('exit') or est <= a.budget_sec - 3.0:
                best, best_path = rr, os.path.join(work, 'r_l%d.json' % laps)
                break

    out = os.path.join(TESTS, 'plan_%s.json' % a.name)
    subprocess.run([sys.executable, os.path.join(HERE, 'to_plan.py'), best_path, gp, out],
                   env=dict(os.environ, PYTHONUTF8='1'), check=True)
    print('計画: %s(%s を使用)' % (out, os.path.basename(best_path)))
    # 2026-11-13: 計画ファイルを道具とは別に検査する(切り上げの道・コースの外・脚とのすき間・ガレージ・持ち時間)。
    # ユーザー: 当日は NG でも走らせる(走らないと0点)。検査は当日までの準備の抜けを見つけるためのもので、当日は結果を表示するだけ。
    chk = subprocess.run([sys.executable, os.path.join(HERE, 'check_plan.py'), out, os.path.join(HERE, 'params_day.json'),
                          '--budget', str(a.budget_sec)], env=dict(os.environ, PYTHONUTF8='1', HR_NO_NUMBA='1'),
                         capture_output=True, text=True)
    print(chk.stdout.rstrip())
    first_line = chk.stdout.splitlines()[0] if chk.stdout.strip() else ''
    if chk.returncode != 0 or not first_line.endswith(': OK'):
        print('注意: 検査で NG(または検査が動かなかった)。当日はこのまま走らせてよい(上の NG の行は、あとで準備の抜けとして見直す)')
        if chk.stderr:
            print(chk.stderr[-1500:])
    rel = os.path.relpath(out, ROOT).replace('\\', '/')
    print('走らせるコマンド(ロボットで git pull のあと。持ち時間は 120 - ETラリー開始までの秒):')
    print('python alpha.py %s --mission rally-drive --rally-gyro-hub-recal-sec 2 --rally-gyro-drift-settle-sec 1.0 '
          '--rally-move-short-lead 15 --rally-path-stop-lead 18 --rally-gyro-post-measure-sec 3 --rally-start-pose on '
          '--rally-camera-log 5 --rally-camera-exposure 30 --rally-camera-correct on --rally-arm-stop-ticks 3 '
          '--rally-arm-power 40 --rally-arm-travel-deg 260 --rally-arm-while-driving on --rally-arm-up-sec 0.9 '
          '--rally-time-budget 70 --rally-plan-path %s' % (a.course, rel))


if __name__ == '__main__':
    main()
