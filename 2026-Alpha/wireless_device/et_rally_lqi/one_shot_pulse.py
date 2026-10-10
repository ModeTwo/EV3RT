"""旋回フェーズ2の「一発サイズの補正」用、PC側の計算ツール。

【位置づけ(2026-09、会話にもとづく優先順位の見直し)】
実機で確認された「旋回時、目標角度に向くまで細かく左右に動く」現象は、
フィードバックの賢さ(PID/LQIどちらのゲイン設計か)の問題ではなく、
最小出力(不感帯を超えるための下限パワー)の粒度が粗いことが原因である
可能性が高い、という診断にもとづく。

対策として、「毎tickフィードバックし続けて収束を待つ」のではなく、
「較正データ(b, c、および不感帯の下限パワー)から、残りの誤差ぴったりに
必要なパルス長(何tick、下限パワーを与え続けるか)を先に計算し、
1回だけそのパルスを与えて様子を見る」という設計にする。

b, c、下限パワーは、parse_log.py/fit_model.pyで、
--mission et-rally-lqi-calibration のログから、これまでと同じ手順で求める
(この一発補正の設計のためだけに、新しい較正手順を追加する必要はない)。

モデル(design_gains.pyと同じ、開ループ版):
    omega[k+1] = omega[k] + (b*u_eff - c*omega[k])*dt
    theta[k+1] = theta[k] + omega[k]*dt   (符号は「今の向き」の変化そのもの)

    u_eff = u - stiction_floor (u>0の場合。u<0なら符号反転)
    ※HoldSymmetricPower/fit_model.pyの定義に合わせ、不感帯分を引いた
      「効いている分」の出力だけがb*u_effとして角加速度に効く、とみなす。
"""
import math


def simulate_pulse(b, c, power, stiction_floor, dt=0.02, max_steps=2000):
    """一定のpowerを与え続けたときの、角度・角速度の時系列を返す。

    戻り値: [(tick, theta_deg, omega_deg_s), ...] (tick=0が開始時点、theta=0スタート)
    """
    sign = 1.0 if power >= 0 else -1.0
    u_eff = sign * max(abs(power) - stiction_floor, 0.0)
    theta = 0.0
    omega = 0.0
    trace = [(0, theta, omega)]
    for tick in range(1, max_steps + 1):
        omega = omega + (b * u_eff - c * omega) * dt
        theta = theta + omega * dt
        trace.append((tick, theta, omega))
    return trace


def ticks_for_target(b, c, power, stiction_floor, target_deg, dt=0.02, max_steps=2000):
    """target_deg(符号つき、度)ぴったり動かすのに必要なtick数を求める。

    「|theta|がtarget_degの絶対値に初めて到達するtick」を返す(オーバーシュートは
    見込まない、狙った量に最も近い側で止める設計)。到達できなければNone。

    戻り値: (ticks, actual_theta_deg, actual_omega_deg_s) または (None, None, None)
    """
    if abs(target_deg) < 1e-9:
        return 0, 0.0, 0.0
    # powerの大きさはそのまま使い、向きだけtarget_degに揃える。
    signed_power = math.copysign(abs(power), target_deg)
    trace = simulate_pulse(b, c, signed_power, stiction_floor, dt=dt, max_steps=max_steps)
    target_abs = abs(target_deg)
    prev_tick, prev_theta, prev_omega = trace[0]
    for tick, theta, omega in trace[1:]:
        if abs(theta) >= target_abs:
            # 目標をまたいだ直前・直後のうち、より近いtickを選ぶ
            if abs(abs(prev_theta) - target_abs) <= abs(abs(theta) - target_abs):
                return prev_tick, prev_theta, prev_omega
            return tick, theta, omega
        prev_tick, prev_theta, prev_omega = tick, theta, omega
    return None, None, None


def build_pulse_table(b, c, power, stiction_floor, max_target_deg=5.0, step_deg=0.25, dt=0.02):
    """0.25度刻みなどで、小さい目標角度ごとに必要なtick数を事前計算した表を作る。

    ロボット側(behaviours側)は、この表をそのまま定数として埋め込み、実行時は
    シミュレーションをやり直さず、一番近い目標角度のtick数を引くだけにする想定。

    戻り値: [(target_deg, ticks, actual_theta_deg), ...] target_deg昇順(0除く、正の値のみ。
        負の目標は呼び出し側でtarget_deg<0のときticksをそのまま使い、向きだけ反転させる)。
    """
    table = []
    steps = int(round(max_target_deg / step_deg))
    for i in range(1, steps + 1):
        target_deg = step_deg * i
        ticks, actual_theta, _ = ticks_for_target(b, c, power, stiction_floor, target_deg, dt=dt)
        if ticks is not None:
            table.append((round(target_deg, 4), ticks, round(actual_theta, 4)))
    return table


def print_pulse_table(table, power, stiction_floor, dt=0.02):
    print("=== 一発補正パルス表 (power=%d, stiction_floor=%.1f, dt=%.3fs) ===" % (
        power, stiction_floor, dt))
    print("目標角度(度) -> tick数 (実際の到達角度)")
    for target_deg, ticks, actual_theta in table:
        print("  %5.2f度 -> %3d tick (%.3fs)  実際=%.3f度" % (
            target_deg, ticks, ticks * dt, actual_theta))
