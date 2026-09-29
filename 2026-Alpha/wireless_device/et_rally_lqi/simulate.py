"""モデル(b, c)とゲインKで、閉ループの動きをオフラインで試算する。

実機に載せる前に、Q・Rの候補をいくつか試して挙動を見比べる用途(方式選択・調整)と、
実測した検証データ(LoggedFineTrimのログ)と、モデルの予測を答え合わせする用途に使う。

使い方(単体、b=0.6, c=1.5, 初期誤差5度からの収束を試算):
    python -m wireless_device.et_rally_lqi.simulate --b 0.6 --c 1.5 --theta0 5.0
"""

import argparse

import numpy as np

from .design_gains import design_lqi_gains


def simulate_closed_loop(b, c, k, dt=0.02, theta0=5.0, omega0=0.0, integral0=0.0,
                          steps=300, stiction_floor=0.0, u_limit=None, disturbance=0.0):
    """状態[theta_e, omega, integral]を、u=-K.xの制御則で、steps回ぶん進める。

    stiction_floor: これより|u|が小さいときは、実機と同様に動かない(u実効=0)ものとして扱う
    (このモデルはあくまで近似。実機のスタック検知・出力の引き上げは、別途、制御則側に実装する)。
    u_limit: 出力の絶対値の上限(Noneなら無制限)。
    disturbance: 出力に依存しない、一定の外乱トルク(deg/s^2、例: 左右輪の摩擦差で、
    出力0でも車体が一定方向へ回ろうとする成分)。積分項(K3)は、この種の「一定の偏り」を
    打ち消すためのものなので、disturbanceを与えたときにだけ、積分の有無の差が意味を持つ
    (bの較正誤差のような、出力に比例する誤差=ゲインの誤差は、状態フィードバックの
    平衡点そのものをずらさないため、積分項の有無で差が出ない。これは数式上の性質であり、
    不具合ではない)。

    戻り値: {"t":[...], "theta_e":[...], "omega":[...], "integral":[...], "u":[...]}
    """
    k = np.asarray(k, dtype=float).flatten()
    theta, omega, integral = theta0, omega0, integral0
    t_list, theta_list, omega_list, integral_list, u_list = [], [], [], [], []
    for step in range(steps):
        x = np.array([theta, omega, integral])
        u = float(-k @ x)
        if u_limit is not None:
            u = max(-u_limit, min(u_limit, u))
        u_eff = u
        if stiction_floor > 0.0 and abs(u) < stiction_floor:
            u_eff = 0.0
        t_list.append(step * dt)
        theta_list.append(theta)
        omega_list.append(omega)
        integral_list.append(integral)
        u_list.append(u)
        theta = theta - omega * dt
        omega = omega + (b * u_eff - c * omega + disturbance) * dt
        integral = integral + dt * theta
    return {"t": t_list, "theta_e": theta_list, "omega": omega_list,
            "integral": integral_list, "u": u_list}


def compare_to_fine_trim_run(samples, b, c, k, dt=0.02):
    """LoggedFineTrimの実測サンプル(heading, error)列と、同じ初期誤差からのモデル予測を比べる。

    戻り値: (最大絶対誤差, 二乗平均平方根誤差) の組(理論値と実測値の、theta_eの差について)。
    """
    if not samples:
        raise ValueError("samples must be non-empty")
    theta0 = samples[0]["error"]
    result = simulate_closed_loop(b, c, k, dt=dt, theta0=theta0, steps=len(samples))
    predicted = np.array(result["theta_e"][:len(samples)])
    actual = np.array([s["error"] for s in samples])
    n = min(len(predicted), len(actual))
    diff = predicted[:n] - actual[:n]
    return float(np.max(np.abs(diff))), float(np.sqrt(np.mean(diff ** 2)))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--b", type=float, required=True)
    parser.add_argument("--c", type=float, required=True)
    parser.add_argument("--theta0", type=float, default=5.0)
    parser.add_argument("--q-omega", type=float, default=0.05)
    parser.add_argument("--r", type=float, default=0.02)
    parser.add_argument("--q-integral", type=float, default=0.0)
    args = parser.parse_args()
    k, _, _, _ = design_lqi_gains(args.b, args.c, q_omega=args.q_omega, r=args.r,
                                  q_integral=args.q_integral)
    result = simulate_closed_loop(args.b, args.c, k, theta0=args.theta0, steps=150)
    for i in range(0, len(result["t"]), 5):
        print("t=%.2f theta_e=%+.3f omega=%+.3f u=%+.1f" % (
            result["t"][i], result["theta_e"][i], result["omega"][i], result["u"][i]))
