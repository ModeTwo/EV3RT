"""実測ログ1つから、モデル(b, c)・不感帯・LQIのゲインKまでを、一気に計算して報告する。

`et_rally_lqi_calibration.py`(--mission et-rally-lqi-calibration)で走らせたログを、
run_logsからそのままこのスクリプトへ渡せば、`robot_program/behaviours/et_rally_turn_lqi.py`の
PLACEHOLDER_K_*・stiction_floorを差し替えるための値を、そのまま出力する。

使い方:
    python -m wireless_device.et_rally_lqi.run_pipeline path/to/run.log
    python -m wireless_device.et_rally_lqi.run_pipeline path/to/run.log --q-omega 0.05 --q-integral 0.02 --r 0.02
"""

import argparse
import sys

from .design_gains import design_lqi_gains
from .fit_model import fit_b_c, fit_b_c_by_direction
from .parse_log import fine_trim_samples, group_step_response_runs, parse_file, stiction_thresholds
from .simulate import compare_to_fine_trim_run


def run(log_path, q_theta=1.0, q_omega=0.05, q_integral=0.02, r=0.02):
    records = parse_file(log_path)
    runs = group_step_response_runs(records)
    if not runs:
        raise SystemExit(
            "応答特性(HoldSymmetricPower)のデータが見つからない。"
            "--mission et-rally-lqi-calibration のログのパスを確認すること。"
        )
    b, c, r_squared, n = fit_b_c(runs)
    by_direction = fit_b_c_by_direction(runs)
    thresholds = stiction_thresholds(records)
    fine_samples = fine_trim_samples(records)

    K, P, A, B = design_lqi_gains(b, c, q_theta=q_theta, q_omega=q_omega, q_integral=q_integral, r=r)

    report = {
        "b": b, "c": c, "r_squared": r_squared, "n_samples": n,
        "by_direction": by_direction, "thresholds": thresholds,
        "K": K, "fine_samples_count": len(fine_samples),
    }

    # 検証データ(LoggedFineTrim)があれば、収束ごとに区切って、モデルの答え合わせをする。
    # (LoggedFineTrimは、それぞれ"start"で始まるので、そこで区切る。)
    validations = []
    current = []
    for record in fine_samples:
        current.append(record)
    if fine_samples:
        # fine_trim_samples は "step" レコードだけを返すため、ここでは全体を1本として粗く扱う。
        # 厳密に試行ごとへ分けたい場合は、parse_log.pyのgroup_step_response_runsと同様の
        # 区切り関数を、LoggedFineTrim用にも追加すること(現状は簡易的な全体比較のみ)。
        try:
            max_err, rmse = compare_to_fine_trim_run(fine_samples, b, c, K)
            validations.append((max_err, rmse))
        except (ValueError, ZeroDivisionError):
            pass
    report["validations"] = validations
    return report


def print_report(report):
    print("=== 応答特性(全体、時計回り+反時計回り込み) ===")
    print("b = %.6f deg/s^2 per power" % report["b"])
    print("c = %.6f 1/s" % report["c"])
    print("R^2 = %.4f (サンプル数 %d)" % (report["r_squared"], report["n_samples"]))
    if report["r_squared"] < 0.8:
        print("注意: R^2が低い。データ・モデルを見直すこと。", file=sys.stderr)

    print()
    print("=== 方向別(左右非対称の目安。差が大きければ、単一モデルの妥当性を要検討) ===")
    for label, fit in report["by_direction"].items():
        if fit is None:
            print("%s: データ不足" % label)
        else:
            b_d, c_d, r2_d, n_d = fit
            print("%s: b=%.6f c=%.6f R^2=%.4f n=%d" % (label, b_d, c_d, r2_d, n_d))

    print()
    print("=== 不感帯(FindStictionThreshold) ===")
    if not report["thresholds"]:
        print("不感帯のデータが見つからない(FindStictionThresholdのログを確認すること)")
    else:
        for record in report["thresholds"]:
            print("direction=%+d power=%d moved=%.2f" % (
                record.get("direction", 0), record.get("power", 0), record.get("moved", 0)))
        floor_candidates = [record["power"] for record in report["thresholds"] if "power" in record]
        if floor_candidates:
            print("推奨stiction_floor(安全側、最大値+余裕3): %.1f" % (max(floor_candidates) + 3))

    print()
    print("=== 検証(LoggedFineTrimとの答え合わせ) ===")
    if report["validations"]:
        for max_err, rmse in report["validations"]:
            print("最大誤差=%.3f度  RMSE=%.3f度" % (max_err, rmse))
    else:
        print("検証データなし、またはモデルとの比較に失敗した(fine_trimのログを確認すること)")

    print()
    print("=== robot_program/behaviours/et_rally_turn_lqi.py へ貼り付ける値 ===")
    K = report["K"]
    print("PLACEHOLDER_K_THETA = %.6f" % K[0])
    print("PLACEHOLDER_K_OMEGA = %.6f" % K[1])
    print("PLACEHOLDER_K_INTEGRAL = %.6f" % K[2])
    print("(変数名のPLACEHOLDER_は、実測値に差し替えたら、意味に合わせて改名してよい)")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("log_path")
    parser.add_argument("--q-theta", type=float, default=1.0)
    parser.add_argument("--q-omega", type=float, default=0.05)
    parser.add_argument("--q-integral", type=float, default=0.02)
    parser.add_argument("--r", type=float, default=0.02)
    args = parser.parse_args()
    report = run(args.log_path, q_theta=args.q_theta, q_omega=args.q_omega,
                 q_integral=args.q_integral, r=args.r)
    print_report(report)
