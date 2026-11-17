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


def fit_b_c_with_voltage(runs, v_ref_mv=None):
    """uの代わりにu*(battery_mv/v_ref_mv)を使って最小二乗する(電圧補正版)。

    モーターはPWM駆動のDCモーターなので、同じpower(デューティ比)でも実際に
    かかる電圧はバッテリー電圧に比例する。較正中の電圧変動がそのままb・cの
    誤差(fit_b_c()のR^2の低さ)に混ざっている可能性を検証するため、2026-10-25追加。
    較正ログにbattery_mvが無い(旧ログ)場合はValueErrorになる。

    v_ref_mv: 基準電圧。省略時は使うサンプル全体の平均(bの値を電圧補正前と
        近いスケールに保つため)。

    戻り値: (b, c, r_squared, n_samples, v_ref_mv)
    """
    voltages = []
    for run in runs:
        for sample in run["samples"]:
            if "battery_mv" in sample and sample["battery_mv"] > 0:
                voltages.append(sample["battery_mv"])
    if not voltages:
        raise ValueError(
            "battery_mvが記録されていない較正ログ(旧ログ、または--check-tree実行)。"
            "et_rally_turn_calibration.pyの修正(2026-10-25)以降に取り直した"
            "ログを使うこと。"
        )
    if v_ref_mv is None:
        v_ref_mv = sum(voltages) / len(voltages)

    rows = []
    targets = []
    for run in runs:
        samples = run["samples"]
        u = run["power"]
        for a, b_sample in zip(samples, samples[1:]):
            if a.get("battery_mv", -1) <= 0:
                continue
            dt = b_sample["t"] - a["t"]
            if dt <= 1e-6:
                continue
            y = (b_sample["omega"] - a["omega"]) / dt
            u_v = u * (a["battery_mv"] / v_ref_mv)
            rows.append([u_v, -a["omega"]])
            targets.append(y)
    if len(rows) < 2:
        raise ValueError("フィットに使えるサンプルが不足している(較正ログを確認すること)")
    design = np.array(rows, dtype=float)
    target = np.array(targets, dtype=float)
    (b_coef, c_coef), residuals, rank, _ = np.linalg.lstsq(design, target, rcond=None)
    predicted = design @ np.array([b_coef, c_coef])
    ss_res = float(np.sum((target - predicted) ** 2))
    ss_tot = float(np.sum((target - np.mean(target)) ** 2))
    r_squared = 1.0 - ss_res / ss_tot if ss_tot > 1e-9 else float("nan")
    return float(b_coef), float(c_coef), r_squared, len(rows), v_ref_mv


def fit_b_c_trajectory(runs, c_min=0.05, c_max=300.0):
    """b, cを、隣り合うサンプルの差分(omega)ではなく、各試行のheading(t)の軌跡全体に当てはめて求める。

    headingの生の分解能が粗い(約1度刻み、2026-10-25の実機ログで確認)ため、fit_b_c()のように
    隣り合うサンプルの差分でomegaを作ると、本当の滑らかな角速度ではなく分解能の粗さがそのまま
    ノイズとして乗ってしまう。この関数は、1試行(同じ出力を保持し続ける1回のHoldSymmetricPower)
    ぶんのheading(t)の並び全体を、
        omega(t) = (b*u/c) * (1 - exp(-c*t))          (omega(0)=0から出発)
        theta(t) = theta0 + (b*u/c)*t - (b*u/c^2) * (1 - exp(-c*t))
    という解析解にまとめて当てはめる。cを1つ決めればthetaはbについて線形になるので、
    cだけを数値探索し、各cごとにbを最小二乗で解析的に求める(scipy不要)。

    戻り値: (b, c, r_squared, n_samples)
        r_squared はtheta(heading)の当てはめに対する値。fit_b_c()のr_squared(omegaの当てはめに
        対する値)とは対象が違うので、数値を直接比較しないこと。
    """
    t_list = []
    u_list = []
    theta_list = []
    for run in runs:
        samples = run["samples"]
        if len(samples) < 2:
            continue
        u = run["power"]
        theta0 = samples[0]["heading"]
        for sample in samples:
            t_list.append(sample["t"])
            u_list.append(u)
            theta_list.append(sample["heading"] - theta0)
    if len(t_list) < 2:
        raise ValueError("フィットに使えるサンプルが不足している(較正ログを確認すること)")

    t_arr = np.array(t_list, dtype=float)
    u_arr = np.array(u_list, dtype=float)
    theta_arr = np.array(theta_list, dtype=float)

    def basis_for_c(c):
        # theta(t)/b = (u/c)*t - (u/c^2)*(1-exp(-c*t))
        return (u_arr / c) * t_arr - (u_arr / (c * c)) * (1.0 - np.exp(-c * t_arr))

    def sse_for_c(c):
        basis = basis_for_c(c)
        denom = float(np.dot(basis, basis))
        if denom < 1e-12:
            return float("inf"), 0.0
        b = float(np.dot(basis, theta_arr) / denom)
        resid = theta_arr - b * basis
        return float(np.dot(resid, resid)), b

    best_c, best_sse, best_b = None, float("inf"), None

    def scan(lo, hi, steps):
        nonlocal best_c, best_sse, best_b
        for c in np.geomspace(lo, hi, steps):
            sse, b = sse_for_c(c)
            if sse < best_sse:
                best_sse, best_c, best_b = sse, float(c), b

    scan(c_min, c_max, 80)
    for _ in range(3):
        lo = max(best_c / 3.0, c_min)
        hi = min(best_c * 3.0, c_max)
        scan(lo, hi, 60)

    ss_tot = float(np.sum((theta_arr - np.mean(theta_arr)) ** 2))
    r_squared = 1.0 - best_sse / ss_tot if ss_tot > 1e-9 else float("nan")
    return float(best_b), float(best_c), r_squared, len(t_list)


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
    print("=== 電圧補正なし(従来) ===")
    print("b(出力->角加速度) = %.6f deg/s^2 per power" % b)
    print("c(粘性減衰)       = %.6f 1/s" % c)
    print("決定係数 R^2       = %.4f (サンプル数 %d)" % (r2, n))
    if r2 < 0.8:
        print("注意: R^2が低い。不感帯の影響が混ざっている(出力が低い試行を除く)、"
              "ノイズが大きい(ジャイロの平滑化を検討)、モデルが単純すぎる、等の可能性がある。",
              file=sys.stderr)

    records = parse_file(sys.argv[1])
    runs = group_step_response_runs(records)
    try:
        b_v, c_v, r2_v, n_v, v_ref = fit_b_c_with_voltage(runs)
    except ValueError as error:
        print("\n=== 電圧補正版 ===\n(スキップ: %s)" % error)
    else:
        print("\n=== 電圧補正版(2026-10-25追加) ===")
        print("基準電圧 v_ref     = %.0f mV (サンプル平均)" % v_ref)
        print("b(出力*電圧比->角加速度) = %.6f deg/s^2 per (power*mV/v_ref)" % b_v)
        print("c(粘性減衰)       = %.6f 1/s" % c_v)
        print("決定係数 R^2       = %.4f (サンプル数 %d)" % (r2_v, n_v))
        print("\n電圧補正なしR^2=%.4f -> 電圧補正版R^2=%.4f (%s)" % (
            r2, r2_v,
            "改善" if r2_v > r2 else "改善なし(床材一定なら電圧以外の要因が主、または電圧依存が小さい)"))

    b_t, c_t, r2_t, n_t = fit_b_c_trajectory(runs)
    print("\n=== 軌跡全体フィット版(2026-10-25追加) ===")
    print("(隣接サンプルの差分でomegaを作らず、各試行のheading(t)全体にモデルを当てはめる。"
          "R^2はthetaに対する値で、上のomega基準のR^2とは比較できない)")
    print("b(出力->角加速度) = %.6f deg/s^2 per power" % b_t)
    print("c(粘性減衰)       = %.6f 1/s" % c_t)
    print("決定係数 R^2       = %.4f (サンプル数 %d)" % (r2_t, n_t))
