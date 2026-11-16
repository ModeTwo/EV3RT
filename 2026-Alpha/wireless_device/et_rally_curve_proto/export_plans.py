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
    for d in a.dirs:
        for f in sorted(glob.glob(os.path.join(d, 'plan_*.json'))):
            if os.path.basename(f) in bad:
                skipped.append(os.path.basename(f))
                continue
            try:
                plan = json.load(open(f, encoding='utf-8'))
                key = layout_key(check(plan['gates']))
            except (OSError, ValueError, KeyError, TypeError) as error:
                skipped.append('%s (%s)' % (os.path.basename(f), error))
                continue
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


if __name__ == '__main__':
    main()
