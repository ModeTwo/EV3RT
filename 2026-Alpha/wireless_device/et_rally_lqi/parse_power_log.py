"""run_logsの実行ログから"RALLY_PWR"タグの行を取り出し、出力・バッテリー電圧/電流の
傾向を要約する(power_log.py、robot_program/power_log.pyが出すログに対応)。

DD-19較正の目的(5-5要素技術③: バッテリー出力に応じた調整の要否)のため、生ログを
毎回貼ってもらう代わりに、Pi上でこのスクリプトを実行して短いサマリだけを見る/共有する
想定。複数のログファイルを渡すと、ファイルごとの比較表も出す(5回実行した結果の比較等)。

使い方:
    python3 -m wireless_device.et_rally_lqi.parse_power_log run_logs/run1.log [run_logs/run2.log ...]
"""

import re
import statistics
import sys
from collections import defaultdict

TAG = "RALLY_PWR"
_KV_INT_RE = re.compile(r"(right_power|left_power|battery_mv|battery_ma|dist)=([+-]?\d+)")
_SOURCE_RE = re.compile(r"source=(\S+)")


def parse_line(line):
    index = line.find(TAG)
    if index < 0:
        return None
    rest = line[index + len(TAG):]
    source_match = _SOURCE_RE.search(rest)
    if source_match is None:
        return None
    record = {"source": source_match.group(1)}
    for key, value in _KV_INT_RE.findall(rest):
        record[key] = int(value)
    if "battery_mv" not in record:
        return None
    return record


def parse_file(path):
    records = []
    with open(path, encoding="utf-8", errors="replace") as file:
        for line in file:
            record = parse_line(line)
            if record is not None:
                records.append(record)
    return records


def summarize(records):
    if not records:
        return None
    mv = [r["battery_mv"] for r in records]
    ma = [r["battery_ma"] for r in records]
    return {
        "count": len(records),
        "mv_min": min(mv), "mv_max": max(mv), "mv_mean": statistics.mean(mv),
        "mv_start": mv[0], "mv_end": mv[-1], "mv_drop": mv[0] - mv[-1],
        "ma_min": min(ma), "ma_max": max(ma), "ma_mean": statistics.mean(ma),
    }


def print_report(path, records):
    print("=== %s ===" % path)
    summary = summarize(records)
    if summary is None:
        print("RALLY_PWR行が見つかりません(--rally-gyro-tire-calibration等の有無に関わらず、"
              "power_log.pyが有効な実行のログか確認すること)")
        return
    print("サンプル数: %d" % summary["count"])
    print("電圧(mV): 開始=%d 終了=%d 低下=%d  (最小=%d 最大=%d 平均=%.1f)" % (
        summary["mv_start"], summary["mv_end"], summary["mv_drop"],
        summary["mv_min"], summary["mv_max"], summary["mv_mean"]))
    print("電流(mA): 最小=%d 最大=%d 平均=%.1f" % (
        summary["ma_min"], summary["ma_max"], summary["ma_mean"]))

    by_source = defaultdict(list)
    for record in records:
        by_source[record["source"]].append(record)
    print("--- source別 ---")
    for source, recs in sorted(by_source.items()):
        mv = [r["battery_mv"] for r in recs]
        ma = [r["battery_ma"] for r in recs]
        print("%-30s n=%5d  mv平均=%.1f mv最小=%d  ma平均=%.1f ma最大=%d" % (
            source, len(recs), statistics.mean(mv), min(mv), statistics.mean(ma), max(ma)))
    print()


def print_comparison(path_summaries):
    print("=== 複数ログの比較 ===")
    print("%-45s %8s %8s %8s %10s" % ("ファイル", "mv開始", "mv終了", "mv低下", "ma平均"))
    for path, summary in path_summaries:
        if summary is None:
            print("%-45s %8s" % (path, "データなし"))
            continue
        print("%-45s %8d %8d %8d %10.1f" % (
            path, summary["mv_start"], summary["mv_end"], summary["mv_drop"], summary["ma_mean"]))


if __name__ == "__main__":
    paths = sys.argv[1:]
    if not paths:
        print("使い方: python3 -m wireless_device.et_rally_lqi.parse_power_log run_logs/run1.log [...]")
        sys.exit(1)
    summaries = []
    for path in paths:
        records = parse_file(path)
        print_report(path, records)
        summaries.append((path, summarize(records)))
    if len(paths) > 1:
        print_comparison(summaries)
