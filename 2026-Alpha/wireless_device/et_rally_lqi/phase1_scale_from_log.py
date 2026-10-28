"""one_shot旋回のログから、phase1の向きごとの係数(--rally-turn-phase1-scale)を求める。

各旋回について、phase1開始時の向き・目標と、phase1後に静止してから測った誤差
("phase2 pulse#1 error=")から、phase1が実際に回った角度を出す:
    指令角度 D = 目標 - 開始時の向き(-180〜180に正規化)
    実際の回転 A = D - 静止後の誤差
新しい係数は、phase1が目標の margin 度手前で止まるように
    新係数 = 今の係数 * (|D| - margin) / |A|
とし、headingが増える向き(D>0)・減る向き(D<0)ごとに中央値をとる。
手前で止めるのは、1回目のパルスを常に回転と同じ向きにして、逆向きのパルスが
バックラッシュを詰めるだけで終わるのを避けるため(2026-10-28)。

開始時の向きはログに整数で出るため、1旋回ごとの値には約1度の誤差がある。
適応補正(lag_estimate_ratio)が落ち着く前の最初の旋回は --skip で除く。

使い方:
    python -m wireless_device.et_rally_lqi.phase1_scale_from_log <ログ> [--skip 10] [--margin 1.0]
"""

import argparse
import re
import statistics

_START = re.compile(
    r"(turn\d+)[^:]*: \S+ EtRallyTurnOneShot\.phase1 started at heading=(-?\d+) for (-?\d+) "
    r"\(target_motor_deg=[\d.]+\)(?: phase1_scale=([\d.]+))?")
# phase1後の静止時の誤差。1回目のパルスを打たずに終えた旋回(誤差が許容範囲内、
# 2026-10-28以降)は "done attempts=0 error=" に出る。
_PULSE1 = re.compile(
    r"(turn\d+)[^:]*: \S+ EtRallyTurnOneShot\.phase2 (?:pulse#1|done attempts=0) error=([-+\d.]+)")


def parse_turns(path):
    turns = []
    current = None
    with open(path, encoding="utf-8", errors="replace") as file:
        for line in file:
            match = _START.search(line)
            if match:
                current = {
                    "name": match.group(1),
                    "start": int(match.group(2)),
                    "target": int(match.group(3)),
                    "scale": float(match.group(4)) if match.group(4) else 1.0,
                }
                continue
            match = _PULSE1.search(line)
            if match and current is not None and match.group(1) == current["name"]:
                current["error"] = float(match.group(2))
                turns.append(current)
                current = None
    return turns


def suggest(turns, skip, margin, min_turn_deg):
    groups = {"pos": [], "neg": []}
    for turn in turns[skip:]:
        commanded = (turn["target"] - turn["start"] + 180.0) % 360.0 - 180.0
        if abs(commanded) < min_turn_deg:
            continue
        actual = commanded - turn["error"]
        if abs(actual) < 1e-6:
            continue
        new_scale = turn["scale"] * (abs(commanded) - margin) / abs(actual)
        key = "pos" if commanded > 0 else "neg"
        groups[key].append((turn["name"], commanded, actual, turn["scale"], new_scale))
    return groups


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("log")
    parser.add_argument("--skip", type=int, default=10, help="最初に除く旋回数(既定10)")
    parser.add_argument("--margin", type=float, default=1.0, help="phase1を目標の何度手前で止めるか(既定1.0)")
    parser.add_argument("--min-turn-deg", type=float, default=30.0, help="これより小さい旋回は除く(既定30)")
    args = parser.parse_args()

    turns = parse_turns(args.log)
    groups = suggest(turns, args.skip, args.margin, args.min_turn_deg)
    result = []
    for key, label in (("pos", "headingが増える向き"), ("neg", "headingが減る向き")):
        rows = groups[key]
        print("== %s (%d旋回) ==" % (label, len(rows)))
        for name, commanded, actual, scale, new_scale in rows:
            print("  %-7s 指令%+7.1f 実際%+7.1f 今の係数%.4f -> %.4f" % (name, commanded, actual, scale, new_scale))
        value = statistics.median(r[4] for r in rows) if rows else 1.0
        result.append(value)
        print("  中央値: %.4f" % value)
    print("\n--rally-turn-phase1-scale %.3f,%.3f" % tuple(result))


if __name__ == "__main__":
    main()
