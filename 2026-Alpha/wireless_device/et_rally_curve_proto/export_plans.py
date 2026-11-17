"""2026-11-15: 前もって作った計画(plan_*.json、計画の中の gates で配置が分かる)を、配置の鍵の名前 plan_<鍵>.json に並べ直し、
plans.zip にもまとめる。PC の計画の配り役(wireless_device/curve_plan_provider.py --curve-plans-dir)と、
ロボットの予備(robot_program/et_rally_plan_select.py、~/et_rally_plans)が使う。

使い方(2026-Alpha で):
    python wireless_device/et_rally_curve_proto/export_plans.py <計画のフォルダ> [...] --out <出力フォルダ> [--check-json results.jsonl]
  --check-json: batch の検査結果(results.jsonl)。NG・エラーの配置は書き出さない(予備の計画になる)。
"""
import argparse
import glob
import json
import os
import sys
import zipfile

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
sys.path.insert(0, ROOT)
from shared_communication.et_rally_layouts import check, layout_key, all_layouts  # noqa: E402


def _tidy_keeps_clearance(orig, tidied, tol_cm=0.1):
    """check_plan で、手直しした計画のすき間(本線と各周の切り上げ)が元より tol_cm 超え狭くならず、NG もないか。"""
    import tempfile
    import check_plan as cp
    params = os.path.join(cp.HERE, 'params_day.json')
    res = []
    for pl in (orig, tidied):
        fd, path = tempfile.mkstemp(suffix='.json')
        os.close(fd)
        try:
            json.dump(pl, open(path, 'w', encoding='utf-8'), ensure_ascii=False)
            res.append(cp.check(path, params, 70.0, 1.5))
        finally:
            os.remove(path)
    a, b = res
    if b['ng']:
        return False
    if b['main']['min_clear'] < a['main']['min_clear'] - tol_cm:
        return False
    for k, e in a.get('exits', {}).items():
        if k in b.get('exits', {}) and b['exits'][k]['min_clear'] < e['min_clear'] - tol_cm:
            return False
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('dirs', nargs='+')
    ap.add_argument('--out', required=True)
    ap.add_argument('--check-json', action='append', default=[])
    a = ap.parse_args()
    bad = set()
    for cj in a.check_json:
        for line in open(cj, encoding='utf-8'):
            try:
                r = json.loads(line)
            except ValueError:
                continue
            if r.get('ng') or r.get('error'):
                bad.add(os.path.basename(r.get('plan') or ('plan_%s.json' % r.get('name'))))
    os.makedirs(a.out, exist_ok=True)
    written, skipped, dup = {}, [], 0
    tidied, fallback = [], []
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from plan_tidy import tidy_plan
    for d in a.dirs:
        # 2026-11-16: run_all.py / run_layout.py の出力(<鍵>.plan.json、当たり判定は <鍵>.chk.json)も読む。NG の配置は書き出さない
        files = sorted(glob.glob(os.path.join(d, 'plan_*.json')))
        for f in sorted(glob.glob(os.path.join(d, '*.plan.json'))):
            # 2026-11-17: 選んだ計画は <鍵>.plan.json。run_layout v2 の途中の段(.s1/.s2)や .fast/.base は書き出さない
            # (並べ替えで .s1 が後に来て、選んだ計画を上書きしていた)。
            if os.path.basename(f)[:-len('.plan.json')].count('.') > 0:
                continue
            ck = f[:-len('.plan.json')] + '.chk.json'
            try:
                if json.load(open(ck, encoding='utf-8')).get('ng'):
                    bad.add(os.path.basename(f))
            except (OSError, ValueError):
                bad.add(os.path.basename(f))
            files.append(f)
        for f in files:
            if os.path.basename(f) in bad:
                skipped.append(os.path.basename(f))
                continue
            try:
                plan = json.load(open(f, encoding='utf-8'))
                key = layout_key(check(plan['gates']))
            except (OSError, ValueError, KeyError, TypeError) as error:
                skipped.append('%s (%s)' % (os.path.basename(f), error))
                continue
            # 2026-11-17(ユーザー): 同じ直線上の「後退→少し前進→後退」を1本にまとめる(plan_tidy.py、終わりの位置は同じ)
            orig = plan
            plan, n_tidy = tidy_plan(orig)
            n_plain = tidy_plan(orig, allow_marks=False)[1]
            if n_tidy > n_plain and not _tidy_keeps_clearance(orig, plan):
                # 2026-11-17: 周の区切りの印のある折り返しもまとめた版で、すき間(本線・切り上げのどれか)が 1mm 超え狭くなったら、
                # 印のある所はまとめない版にする(分けた所が S字の途中になり、切り上げの出発点が数mm横にずれるため)
                plan, n_tidy = tidy_plan(orig, allow_marks=False)
                fallback.append(key)
            if n_tidy:
                tidied.append('%s(%d)' % (key, n_tidy))
            if key in written:
                dup += 1
            written[key] = f
            json.dump(plan, open(os.path.join(a.out, 'plan_%s.json' % key), 'w', encoding='utf-8'), ensure_ascii=False)
    z = os.path.join(a.out, 'plans.zip')
    with zipfile.ZipFile(z, 'w', zipfile.ZIP_DEFLATED) as zf:
        for key in sorted(written):
            zf.write(os.path.join(a.out, 'plan_%s.json' % key), 'plan_%s.json' % key)
    total = len(all_layouts())
    print('wrote %d plans (%d duplicates replaced), skipped %d, zip %s (%.1f MB), covers %d of %d layouts' % (
        len(written), dup, len(skipped), z, os.path.getsize(z) / 1e6, len(written), total))
    for s in skipped[:10]:
        print('  skipped', s)
    if tidied:
        print('tidied (back-and-forth merged): %s' % ', '.join(tidied))
    if fallback:
        print('lap-end turnarounds left as they were (clearance would drop): %s' % ', '.join(fallback))


if __name__ == '__main__':
    main()
