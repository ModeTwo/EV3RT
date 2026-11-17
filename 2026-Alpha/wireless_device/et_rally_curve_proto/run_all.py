"""2026-11-16: 全配置(4872通り)の事前計算。run_layout.py の決定版の流れを、常駐する作業者で並べて回す。

使い方(2026-Alpha で):
    python wireless_device/et_rally_curve_proto/run_all.py <出力フォルダ> [--workers 12] [--keys 鍵,鍵,...] [--limit N]
出力: <出力フォルダ>/gates/<鍵>.json(配置)、<出力フォルダ>/<鍵>.plan.json ほか(run_layout.py と同じ)、
      <出力フォルダ>/progress.txt(終わった配置ごとに1行: 時刻・鍵・秒・選んだ理由)
- 空いた作業者が次の配置を取る(重い配置が残っても、ほかの CPU が遊ばない)。
- 作業者は常駐し、計画への変換と当たり判定はその中で行う(起動のたびの読み込みを省く)。道具の計算は別プロセス(時間切れあり)。
- <鍵>.pick.json がある配置は飛ばす(止まっても続きから再開できる)。
- 12本同時がこの PC(i5-1235U)で一番速かった(同じ12配置で 12本 158秒・6本 178秒・4本 205秒、11/16)。
- 2026-11-17: ただしメモリ 8GB で、ほかのアプリが約6GB 使うと、道具1本 約0.6GB で 12本は入らない(スワップで止まった)。--workers で調整する。
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
    # 2026-11-17: 既定は作業者の中で計画・当たり判定をしない(作業者が道具を読み込んで約430MB 常駐し、12本でメモリ 8GB を
    # 使い切ってスワップで計算がほぼ止まった)。RUN_ALL_INPROC=1 で前のやり方。
    run_layout.INPROC = os.environ.get('RUN_ALL_INPROC', '0') == '1'


def _job(args):
    gates, out, key = args
    import run_layout
    t0 = time.time()
    try:
        run_layout.process(gates, out, key)
        pp = os.path.join(out, key + '.pick.json')
        if os.path.exists(pp):
            why = json.load(open(pp, encoding='utf-8')).get('why')
        elif os.path.exists(os.path.join(out, key + '.failed.json')):
            why = 'FAILED (tool gave no route)'
        else:
            dp = os.path.join(out, key + '.deferred.json')
            why = 'DEFERRED ' + (json.load(open(dp, encoding='utf-8')).get('deferred') if os.path.exists(dp) else '?')
    except Exception as e:   # 1つの配置の失敗で全体を止めない
        why = 'ERROR %r' % (e,)
    return key, time.time() - t0, why


def _near_pairs_g(G):
    """ゲートどうしが1マス以内に近い組の数(0〜3)。重い配置ほど多い。"""
    pts = {c: [tuple(G[c]['a']), tuple(G[c]['b'])] for c in ('red', 'blue', 'yellow')}
    d = lambda p, q: ((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2) ** 0.5
    return sum(1 for x, y in (('red', 'blue'), ('blue', 'yellow'), ('yellow', 'red'))
               if min(d(p, q) for p in pts[x] for q in pts[y]) <= 1.0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('out')
    ap.add_argument('--workers', type=int, default=12)
    ap.add_argument('--keys', default=None)
    ap.add_argument('--limit', type=int, default=0)
    # 2026-11-17(ユーザー): 1回目は狭い配置を後回しにし(--defer-narrow)、重い配置は --cap 秒で打ち切って後回しにする。
    # 2回目はどちらも付けずに同じフォルダで回す(結果のない配置=後回しにした配置だけを計算する)。
    ap.add_argument('--defer-narrow', action='store_true')
    ap.add_argument('--cap', type=int, default=0)
    # 2026-11-17(クラウドと分ける): ゲートが1マス以内に近い組の数で絞る、n本に分けた i 本目だけ(--shard i/n、全配置の並び順で
    # 番号 % n == i)、ほかで計算済みの配置の一覧(1行1つ、またはカンマ区切り)を飛ばす
    ap.add_argument('--near-min', type=int, default=0)
    ap.add_argument('--near-max', type=int, default=3)
    ap.add_argument('--shard', default=None)
    ap.add_argument('--skip-file', default=None)
    # 2026-11-17: 計算する配置の一覧のファイル(1行1つ、またはカンマ区切り)。--keys と同じ意味(多いとき用)
    ap.add_argument('--keys-file', default=None)
    a = ap.parse_args()
    if a.keys_file:
        a.keys = ','.join(open(a.keys_file, encoding='utf-8').read().replace(',', ' ').split())
    if a.defer_narrow:
        os.environ['HR_DEFER_NARROW'] = '1'
    else:
        os.environ.pop('HR_DEFER_NARROW', None)
    if a.cap:
        os.environ['RUN_LAYOUT_TIMEOUT'] = str(a.cap)
    sys.path.insert(0, ROOT)
    from shared_communication.et_rally_layouts import all_layouts, layout_key
    out = os.path.abspath(a.out)
    gdir = os.path.join(out, 'gates')
    os.makedirs(gdir, exist_ok=True)
    want = set(a.keys.split(',')) if a.keys else None
    skip = set(open(a.skip_file, encoding='utf-8').read().replace(',', ' ').split()) if a.skip_file else set()
    si, sn = (int(x) for x in a.shard.split('/')) if a.shard else (0, 1)
    jobs = []
    idx = -1
    for g in all_layouts():
        key = layout_key(g)
        if want is not None and key not in want:
            continue
        if key in skip:
            continue
        if not (a.near_min <= _near_pairs_g(g) <= a.near_max):
            continue
        idx += 1
        if idx % sn != si:
            continue
        gp = os.path.join(gdir, key + '.json')
        if not os.path.exists(gp):
            json.dump({'gates': g}, open(gp, 'w', encoding='utf-8'))
        if os.path.exists(os.path.join(out, key + '.pick.json')):
            continue
        jobs.append((gp, out, key))
    # 2026-11-17: 重い配置から先に回す(最後に重い配置だけが残って CPU が遊ぶのを減らす)。100配置で、ゲートどうしが1マス以内に
    # 近い組が多いほど計算が重かった(近い組なし: 中央値26秒、2組以上: 208秒)。ゲートの位置だけで分かる。
    jobs.sort(key=lambda j: -_near_pairs_g(json.load(open(j[0], encoding='utf-8'))['gates']))
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
