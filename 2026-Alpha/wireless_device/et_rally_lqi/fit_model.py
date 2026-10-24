"""応答特性の記録(HoldSymmetricPowerのstep列)から、旋回のモデル(b, c)を最小二乗法で求める。

モデル(離散時間、状態=角度誤差theta_e・角速度omega):
    theta_e[k+1] = theta_e[k] - omega[k]*dt        (これは幾何的な関係で、必ず成り立つ)
    omega[k+1]   = omega[k] + (b*u[k] - c*omega[k])*dt

b: 出力(PWM相当)1あたりの角加速度への効き方(deg/s^2 / power)。
c: 粘性減衰(1/s)。値が大きいほど、出力を止めたときに早く減速する。

推定は、各試行(同じ出力を保持し続けた1回ぶん)の中で、隣り合うサンプルの
(omega[k], omega[k+1], t[k], t[k+1], power)から、
    y = (omega[k+1]-omega[k]) / dt  ≈  b*u - c*omega[k]
という関係を、全試行・全サンプルぶんプールして、numpyの最小二乗法(lstsq)で解く。

不感帯(静止摩擦)は、この線形モデルには含めない。不感帯は
`FindStictionThreshold`の結果を、別途、実装側のクランプ・かさ上げとして使う。

使い方(単体):
    python -m wireless_device.et_rally_lqi.fit_model path/to/run.log
"""

import sys

import numpy as np

from .parse_log import group_step_response_runs, parse_file


def fit_b_c(runs, min_omega_for_damping_term=0.0):
    """b, cを最小二乗法で求める。runsは group_step_response_runs() の戻り値。

    戻り値: (b, c, r_squared, n_samples)
    """
    rows = []
    targets = []
    for run in runs:
        samples = run["samples"]
        u = run["power"]
        for a, b_sample in zip(samples, samples[1:]):
            dt = b_sample["t"] - a["t"]
            if dt <= 1e-6:
                continue
            y = (b_sample["omega"] - a["omega"]) / dt
            rows.append([u, -a["omega"]])
            targets.append(y)
    if len(rows) < 2:
        raise ValueError("フィットに使えるサンプルが不足している(応答特性のログを確認すること)")
    design = np.array(rows, dtype=float)
    target = np.array(targets, dtype=float)
    (b_coef, c_coef), residuals, rank, _ = np.linalg.lstsq(design, target, rcond=None)
    predicted = design @ np.array([b_coef, c_coef])
    ss_res = float(np.sum((target - predicted) ** 2))
    ss_tot = float(np.sum((target - np.mean(target)) ** 2))
    r_squared = 1.0 - ss_res / ss_tot if ss_tot > 1e-9 else float("nan")
    return float(b_coef), float(c_coef), r_squared, len(rows)


def fit_from_log(path):
    records = parse_file(path)
    runs = group_step_response_runs(records)
    return fit_b_c(runs)


def fit_b_c_by_direction(runs):
    """時計回り(power>0)・反時計回り(power<0)を別々に当てはめ、左右非対称の目安を出す。

    このプロジェクトのEtRallyTurnLqiは、方向によらず単一のb・cしか使わない設計なので、
    ここで大きな差が見つかった場合は、その単純化が妥当かどうかを、別途判断すること
    (差が大きいまま単一モデルを使うと、どちらか一方の方向で、想定より効きが強い/弱いゲインになる)。
    """
    positive_runs = [r for r in runs if r["power"] > 0]
    negative_runs = [r for r in runs if r["power"] < 0]
    result = {}
    for label, subset in (("cw(power>0)", positive_runs), ("ccw(power<0)", negative_runs)):
        if len(subset) < 2:
            result[label] = None
            continue
        result[label] = fit_b_c(subset)
    return result


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("使い方: python -m wireless_device.et_rally_lqi.fit_model <ログファイル>", file=sys.stderr)
        raise SystemExit(1)
    b, c, r2, n = fit_from_log(sys.argv[1])
    print("b(出力->角加速度) = %.6f deg/s^2 per power" % b)
    print("c(粘性減衰)       = %.6f 1/s" % c)
    print("決定係数 R^2       = %.4f (サンプル数 %d)" % (r2, n))
    if r2 < 0.8:
        print("注意: R^2が低い。不感帯の影響が混ざっている(出力が低い試行を除く)、"
              "ノイズが大きい(ジャイロの平滑化を検討)、モデルが単純すぎる、等の可能性がある。",
              file=sys.stderr)
