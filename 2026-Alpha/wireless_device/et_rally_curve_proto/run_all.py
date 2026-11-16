"""2026-11-16: 全配置(4872通り)の事前計算。run_layout.py の決定版の流れを、常駐する作業者で並べて回す。

使い方(2026-Alpha で):
    python wireless_device/et_rally_curve_proto/run_all.py <出力フォルダ> [--workers 12] [--keys 鍵,鍵,...] [--limit N]
出力: <出力フォルダ>/gates/<鍵>.json(配置)、<出力フォルダ>/<鍵>.plan.json ほか(run_layout.py と同じ)、
      <出力フォルダ>/progress.txt(終わった配置ごとに1行: 時刻・鍵・秒・選んだ理由)
- 空いた作業者が次の配置を取る(重い配置が残っても、ほかの CPU が遊ばない)。
- 作業者は常駐し、計画への変換と当たり判定はその中で行う(起動のたびの読み込みを省く)。道具の計算は別プロセス(時間切れあり)。
- <鍵>.pick.json がある配置は飛ばす(止まっても続きから再開できる)。
- 12本同時がこの PC(i5-1235U)で一番速かった(同じ12配置で 12本 158秒・6本 178秒・4本 205秒、11/16)。
"""
import argparse
import json
import multiprocessing as mp
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))


def _init():
    os.environ['RUN_LAYOUT_FAST'] = '1'
    os.environ['PYTHONUTF8'] = '1'
    sys.path.insert(0, HERE)
    import run_layout
    run_layout.INPROC = True


def _job(args):
    gates, out, key = args
    import run_layout
    t0 = time.time()
    try:
        run_layout.process(gates, out, key)
        why = json.load(open(os.path.join(out, key + '.pick.json'), encoding='utf-8')).get('why')
    except Exception as e:   # 1つの配置の失敗で全体を止めない
        why = 'ERROR %r' % (e,)
    return key, time.time() - t0, why


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('out')
    ap.add_argument('--workers', type=int, default=12)
    ap.add_argument('--keys', default=None)
    ap.add_argument('--limit', type=int, default=0)
    a = ap.parse_args()
    sys.path.insert(0, ROOT)
    from shared_communication.et_rally_layouts import all_layouts, layout_key
    out = os.path.abspath(a.out)
    gdir = os.path.join(out, 'gates')
    os.makedirs(gdir, exist_ok=True)
    want = set(a.keys.split(',')) if a.keys else None
    jobs = []
    for g in all_layouts():
        key = layout_key(g)
        if want is not None and key not in want:
            continue
        gp = os.path.join(gdir, key + '.json')
        if not os.path.exists(gp):
            json.dump({'gates': g}, open(gp, 'w', encoding='utf-8'))
        if os.path.exists(os.path.join(out, key + '.pick.json')):
            continue
        jobs.append((gp, out, key))
    if a.limit:
        jobs = jobs[:a.limit]
    print('%d layouts to compute, %d workers' % (len(jobs), a.workers), flush=True)
    t0 = time.time()
    with mp.Pool(a.workers, initializer=_init) as pool, open(os.path.join(out, 'progress.txt'), 'a', encoding='utf-8') as pf:
        for i, (key, sec, why) in enumerate(pool.imap_unordered(_job, jobs), 1):
            pf.write('%s %s %.0f %s\n' % (time.strftime('%m-%d %H:%M:%S'), key, sec, why)); pf.flush()
            el = time.time() - t0
            if i % 10 == 0 or i == len(jobs):
                print('%d/%d done, %.0f min elapsed, about %.1f h left' % (i, len(jobs), el / 60, el / i * (len(jobs) - i) / 3600), flush=True)


if __name__ == '__main__':
    main()
