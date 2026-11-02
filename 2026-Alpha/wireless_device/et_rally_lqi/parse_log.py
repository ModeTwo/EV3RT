"""run_logsのテキストログから、"LQI_CAL"タグの行だけを取り出し、構造化した記録の列にする。

使い方(単体):
    python -m wireless_device.et_rally_lqi.parse_log path/to/run.log

ログの各行は、`et_rally_lqi_calibration.py`が出す形式:
    LQI_CAL <marker> key=value key=value ...
`<marker>`は、"step"/"threshold" のような単純な語、または
"HoldSymmetricPower.start"のような「クラス名.フェーズ名」のどちらか。
NOT_FOUND のような、値を伴わない語は、真偽フラグとして扱う。
"""

import re
import sys

TAG = "LQI_CAL"
_KV_RE = re.compile(r"^([A-Za-z_]+)=([+-]?[0-9.]+)$")


def parse_line(line):
    """1行から、LQI_CALタグ以降を取り出し、構造化したdictを返す。タグが無ければNone。"""
    index = line.find(TAG)
    if index < 0:
        return None
    rest = line[index + len(TAG):].strip().split()
    if not rest:
        return None
    marker = rest[0]
    record = {"marker": marker}
    if "." in marker:
        cls, phase = marker.split(".", 1)
        record["class"] = cls
        record["phase"] = phase
    else:
        record["phase"] = marker
    for token in rest[1:]:
        match = _KV_RE.match(token)
        if match:
            key, value = match.group(1), float(match.group(2))
            record[key] = value
        else:
            record[token] = True
    return record


def parse_file(path):
    """ファイル全体を読み、レコードのリストを返す。"""
    records = []
    with open(path, encoding="utf-8", errors="replace") as file:
        for line in file:
            record = parse_line(line)
            if record is not None:
                records.append(record)
    return records


def step_response_samples(records):
    """HoldSymmetricPowerの"step"レコード(t, heading, omega, power)だけを、時系列の表として返す。

    戻り値: [{"power": float, "t": float, "heading": float, "omega": float}, ...]
    複数回の試行(HoldSymmetricPowerの呼び出し1回ごと)は、"start"レコードの直後から
    次の"start"/"end"までを1試行としてまとめる想定だが、ここでは単純に全"step"レコードを
    時系列順のまま返す(呼び出し側で、必要なら"power"の値ごと・区切りごとに分ける)。
    """
    return [r for r in records if r.get("phase") == "step" and "omega" in r and "t" in r]


def fine_trim_samples(records):
    """LoggedFineTrimの"step"レコード(heading, error, power)だけを返す(検証用データ)。"""
    return [r for r in records if r.get("phase") == "step" and "error" in r and "omega" not in r]


def group_step_response_runs(records):
    """HoldSymmetricPowerの"start"/"step"/"end"を、呼び出し1回ごとの試行にまとめる。

    最小二乗法で(b, c)を求めるとき、隣り合うサンプルの差分は、同じ試行の中でだけ計算しないと
    いけない(試行の境界をまたぐと、出力が変わった/停止した瞬間を、あたかも連続した応答であるかの
    ように扱ってしまう)。この関数は、それを避けるため、試行ごとに区切って返す。

    戻り値: [{"power": 符号付き出力, "samples": [{"t":..,"heading":..,"omega":..}, ...]}, ...]
    """
    runs = []
    current = None
    for record in records:
        if record.get("class") == "HoldSymmetricPower":
            if record.get("phase") == "start":
                current = {"power": record["power"], "samples": []}
                runs.append(current)
            elif record.get("phase") == "end":
                current = None
            continue
        if current is not None and record.get("phase") == "step" and "omega" in record and "t" in record:
            current["samples"].append(record)
    return runs


def stiction_thresholds(records):
    """FindStictionThresholdが見つけた閾値(direction, power)の一覧を返す。"""
    return [r for r in records if r.get("phase") == "threshold" and "power" in r]


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("使い方: python -m wireless_device.et_rally_lqi.parse_log <ログファイル>", file=sys.stderr)
        raise SystemExit(1)
    all_records = parse_file(sys.argv[1])
    steps = step_response_samples(all_records)
    fine = fine_trim_samples(all_records)
    thresholds = stiction_thresholds(all_records)
    print("全レコード数: %d" % len(all_records))
    print("応答特性(step)のサンプル数: %d" % len(steps))
    print("仕上げ検証(fine_trim)のサンプル数: %d" % len(fine))
    print("不感帯の閾値: %s" % thresholds)
