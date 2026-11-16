"""通常のrally-drive走行ログ(TURN_CALの較正ミッションではなく、本番のEtRallySpinAroundByEncoder
が毎回出すログ)から、方向反転時のバックラッシュ先詰め(backlash_preload_deg、DD-22/23の②)の
目安値を推定する。

専用の較正ミッションを別途走らせなくても、通常の走行ログに元から出ている下記2行だけで
推定できる(et_rally_drive.pyのEtRallySpinAroundByEncoder、fine_trim有効時は毎回出る):

    ...encoder-spin started at heading=<開始方位> for <目標方位> (target_motor_deg=...)
    ...encoder-spin phase1 complete at heading=<phase1終了時点の実測方位>, entering gyro fine-trim

(--rally-turn-fine-trim offの診断時は代わりに
    ...encoder-spin ended without fine-trim at heading=<...>, residual=<...> loc=(...,...)
が出る。このスクリプトは両方のパターンに対応する。)

考え方:
  各旋回について、phase1終了時点でまだ残っている誤差(フェーズ2補正が入る前の残差)を、
  その旋回の進行方向を基準に符号を揃えて(正=行き過ぎ不足/undershoot、負=行き過ぎ/overshoot)
  集計する。前の旋回と同じ方向の旋回(non-reversal)と、逆方向に切り替わった旋回(reversal)の
  2グループに分け、両者の平均の差を取る。

  引き込まれ補正(①lag_compensation/adaptive_lag_compensation)は反転・非反転どちらの
  旋回にも同じように起きる現象なので、差分を取る時点でおおむね相殺される。バックラッシュ
  先詰め(②backlash_preload_deg)は反転した旋回だけに起きる現象なので、差分として残るのが
  ②の目安値になる。①を有効にして取ったログほど、この推定はきれいに②だけを反映する
  (①が既にphase1側で補正されているため)。①を無効のまま取ったログでは、①の影響が
  乗った分だけ推定値がやや過大・不安定になりうる。

  この残差は見出し(heading)度数なので、--rally-turn-backlash-preload-degが適用される
  target_motor_deg(エンコーダの回転度数)に換算するため、WHEEL_TREAD/タイヤ径の比率
  (et_rally_drive.pyの変換と同じ)をかける。

使い方:
    python -m wireless_device.et_rally_lqi.backlash_from_log run_logs/run1.log [run_logs/run2.log ...]
    python -m wireless_device.et_rally_lqi.backlash_from_log --tire-diameter 56.87 --verbose run_logs/run1.log

既知の限界:
  - 1本のログファイルに複数回分の実行(プロセス再起動)が連結されていると、前の実行の
    最後の旋回の方向を次の実行の1本目の反転判定に引き継んでしまう(ログに実行の境界を
    示す目印が無いため)。1実行1ファイルを基本にすること。
  - サンプル数が少ないうちは平均の差がノイズに埋もれやすい。十分な旋回数(反転・非反転
    それぞれ最低数十)が集まってから目安値を使うこと。
"""

import argparse
import re
import statistics
import sys

# et_rally_drive.py / py_etrobo_util/plotter.pyと同じ値。このスクリプトはPC単体で動かすため
# robot_program/py_etrobo_utilには依存しない。実機側の値を変えたら、こちらも合わせて直すか
# --tire-diameterで上書きすること。
WHEEL_TREAD_MM = 117.0
DEFAULT_TIRE_DIAMETER_MM = 56.87  # ET_RALLY_TIRE_DIAMETER (--rally-gyro-tire-calibration on時)

_STARTED_RE = re.compile(
    r"encoder-spin started at heading=(-?\d+) for (-?\d+) \(target_motor_deg=([\d.]+)\)")
_PHASE1_COMPLETE_RE = re.compile(
    r"encoder-spin phase1 complete at heading=(-?\d+), entering gyro fine-trim")
_ENDED_NO_FINE_TRIM_RE = re.compile(
    r"encoder-spin ended without fine-trim at heading=(-?\d+), residual=[+-][\d.]+ loc=\(")


def _signed_error(target, current):
    """target - currentを-180〜+180に正規化する(et_rally_drive.pyのerror計算と同じ式)。"""
    return (float(target) - float(current) + 180.0) % 360.0 - 180.0


def parse_file(path):
    """ログ1本から、旋回ごとのレコードの列を時系列順で返す。

    各要素: {"start_heading", "target_heading", "target_motor_deg", "phase1_heading",
             "direction", "phase1_residual", "residual_in_direction"}
    """
    turns = []
    pending = None
    with open(path, encoding="utf-8", errors="replace") as file:
        for line in file:
            started = _STARTED_RE.search(line)
            if started is not None:
                start_heading, target_heading, target_motor_deg = started.groups()
                pending = {
                    "start_heading": int(start_heading),
                    "target_heading": int(target_heading),
                    "target_motor_deg": float(target_motor_deg),
                }
                continue
            if pending is None:
                continue
            complete = _PHASE1_COMPLETE_RE.search(line)
            if complete is not None:
                pending["phase1_heading"] = int(complete.group(1))
                turns.append(pending)
                pending = None
                continue
            ended = _ENDED_NO_FINE_TRIM_RE.search(line)
            if ended is not None:
                pending["phase1_heading"] = int(ended.group(1))
                turns.append(pending)
                pending = None
                continue

    for turn in turns:
        error = _signed_error(turn["target_heading"], turn["start_heading"])
        turn["direction"] = 1 if error > 0 else -1
        turn["phase1_residual"] = _signed_error(turn["target_heading"], turn["phase1_heading"])
        # 進行方向に対して揃えた符号(正=行き過ぎ不足、負=行き過ぎ)。
        turn["residual_in_direction"] = turn["phase1_residual"] * turn["direction"]
    return turns


def classify_reversals(turns):
    """各旋回に、直前の旋回と方向が逆だったか(is_reversal)を付加する。

    et_rally_drive.pyの_last_turn_directionと同じロジック(前の旋回が無ければ判定不能=None)。
    """
    previous_direction = 0
    for turn in turns:
        direction = turn["direction"]
        turn["is_reversal"] = None if previous_direction == 0 else (direction != previous_direction)
        previous_direction = direction
    return turns


def summarize(turns):
    """classify_reversals済みのturnsから、反転/非反転グループの統計とバックラッシュ目安を返す。"""
    classified = [t for t in turns if t.get("is_reversal") is not None]
    reversal = [t["residual_in_direction"] for t in classified if t["is_reversal"]]
    non_reversal = [t["residual_in_direction"] for t in classified if not t["is_reversal"]]
    result = {
        "n_total": len(turns),
        "n_reversal": len(reversal),
        "n_non_reversal": len(non_reversal),
    }
    if reversal:
        result["reversal_mean_deg"] = statistics.mean(reversal)
        result["reversal_stdev_deg"] = statistics.stdev(reversal) if len(reversal) > 1 else 0.0
    if non_reversal:
        result["non_reversal_mean_deg"] = statistics.mean(non_reversal)
        result["non_reversal_stdev_deg"] = statistics.stdev(non_reversal) if len(non_reversal) > 1 else 0.0
    if reversal and non_reversal:
        result["backlash_estimate_heading_deg"] = result["reversal_mean_deg"] - result["non_reversal_mean_deg"]
    return result


def print_verbose_table(turns):
    print("--- 旋回ごとの内訳(*=反転, ?=判定不能(1本目)) ---")
    for i, turn in enumerate(turns):
        if turn["is_reversal"] is None:
            mark = "?"
        elif turn["is_reversal"]:
            mark = "*"
        else:
            mark = " "
        print("[%3d]%s start=%+4d target=%+4d target_motor_deg=%6.1f "
              "phase1_heading=%+4d residual=%+6.2f residual_in_dir=%+6.2f" % (
                  i, mark, turn["start_heading"], turn["target_heading"],
                  turn["target_motor_deg"], turn["phase1_heading"],
                  turn["phase1_residual"], turn["residual_in_direction"]))
    print()


def print_report(path, turns, tire_diameter_mm):
    print("=== %s ===" % path)
    summary = summarize(turns)
    if summary["n_total"] == 0:
        print("encoder-spinの旋回ログが見つかりません(spin_encoderでの走行ログか確認すること)")
        print()
        return
    print("旋回の総数: %d (reversal=%d, non-reversal=%d, 判定不能=1本目のみ除外)" % (
        summary["n_total"], summary["n_reversal"], summary["n_non_reversal"]))
    if "reversal_mean_deg" in summary:
        print("反転旋回のphase1残差(進行方向基準、正=行き過ぎ不足): 平均=%+.2f度 (stdev=%.2f, n=%d)" % (
            summary["reversal_mean_deg"], summary["reversal_stdev_deg"], summary["n_reversal"]))
    if "non_reversal_mean_deg" in summary:
        print("非反転旋回のphase1残差: 平均=%+.2f度 (stdev=%.2f, n=%d)" % (
            summary["non_reversal_mean_deg"], summary["non_reversal_stdev_deg"], summary["n_non_reversal"]))
    if "backlash_estimate_heading_deg" in summary:
        heading_deg = summary["backlash_estimate_heading_deg"]
        motor_deg = heading_deg * WHEEL_TREAD_MM / tire_diameter_mm
        print("バックラッシュ先詰めの目安(反転-非反転の差): %+.2f度(見出し) -> "
              "--rally-turn-backlash-preload-deg %.1f 相当(タイヤ径%.2fmm換算)" % (
                  heading_deg, max(motor_deg, 0.0), tire_diameter_mm))
        if heading_deg <= 0:
            print("(差が0以下のため、今回のログからはバックラッシュ先詰めの必要性は確認できない)")
    else:
        print("反転・非反転の両方のサンプルが無いため、差分を計算できません"
              "(同じ方向の旋回だけ、または異なる方向の旋回だけだった)")
    print()


def main(argv):
    parser = argparse.ArgumentParser(
        description="走行ログから backlash_preload_deg(DD-22/23の②)の目安値を推定する")
    parser.add_argument("logs", nargs="+", help="run_logsのログファイル(複数可)")
    parser.add_argument("--tire-diameter", type=float, default=DEFAULT_TIRE_DIAMETER_MM,
                         help="見出し度数->motor_deg換算に使うタイヤ径mm(既定: ET_RALLY_TIRE_DIAMETER=%.2f)" % (
                             DEFAULT_TIRE_DIAMETER_MM))
    parser.add_argument("--verbose", action="store_true", help="旋回ごとの内訳も表示する")
    args = parser.parse_args(argv)

    for path in args.logs:
        turns = parse_file(path)
        classify_reversals(turns)
        if args.verbose:
            print_verbose_table(turns)
        print_report(path, turns, args.tire_diameter)


if __name__ == "__main__":
    main(sys.argv[1:])
