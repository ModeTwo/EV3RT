"""b, cのモデルと、重み(Q, R)から、LQI(積分付きLQR)のゲインKを求める。

離散代数リカッチ方程式(DARE)を、`scipy`を使わず、固定点反復(値反復)で解く
(`P_{k+1} = A^T P_k A - A^T P_k B (R+B^T P_k B)^-1 B^T P_k A + Q` を、収束するまで繰り返す。
系が可制御・可観測であれば収束することが知られている、教科書的な求め方)。

状態 x = [角度誤差theta_e(deg), 角速度omega(deg/s), 角度誤差の積分Integral(deg*s)]
    theta_e[k+1]  = theta_e[k] - omega[k]*dt
    omega[k+1]    = (1-c*dt)*omega[k] + b*dt*u[k]
    Integral[k+1] = Integral[k] + dt*theta_e[k]
制御則: u = -(K1*theta_e + K2*omega + K3*Integral)

使い方(単体、bとcを直接渡す):
    python -m wireless_device.et_rally_lqi.design_gains --b 0.6 --c 1.5
"""

import argparse

import numpy as np


def build_state_space(b, c, dt):
    A = np.array([
        [1.0, -dt, 0.0],
        [0.0, 1.0 - c * dt, 0.0],
        [dt, 0.0, 1.0],
    ])
    B = np.array([[0.0], [b * dt], [0.0]])
    return A, B


def solve_discrete_lqr(A, B, Q, R, iterations=5000, tol=1e-12):
    """離散LQRのP・Kを、固定点反復で求める。収束しなければRuntimeErrorにする。"""
    P = np.array(Q, dtype=float)
    for _ in range(iterations):
        BtP = B.T @ P
        S = R + BtP @ B  # (m x m)、mは入力の数(ここでは1)
        K = np.linalg.solve(S, BtP @ A)  # (m x n)
        P_next = A.T @ P @ A - A.T @ P @ B @ K + Q
        if np.max(np.abs(P_next - P)) < tol:
            return K, P_next
        P = P_next
    raise RuntimeError("リカッチ方程式が収束しなかった(iterationsを増やすか、Q/R/モデルを見直すこと)")


def design_lqi_gains(b, c, dt=0.02, q_theta=1.0, q_omega=0.05, q_integral=0.0, r=0.02):
    """LQIのゲインK=[K_theta, K_omega, K_integral]を返す。

    重みの目安: q_thetaを基準(=1)にし、q_omegaは「速度の揺れをどれだけ嫌うか」、
    q_integralは「定常偏差(較正誤差等)をどれだけ速く消すか」、rは「出力の大きさを
    どれだけ節約したいか(大きいほど穏やかな動きになる)」。
    """
    A, B = build_state_space(b, c, dt)
    Q = np.diag([q_theta, q_omega, q_integral])
    R = np.array([[r]])
    K, P = solve_discrete_lqr(A, B, Q, R)
    return K.flatten(), P, A, B


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--b", type=float, required=True)
    parser.add_argument("--c", type=float, required=True)
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument("--q-theta", type=float, default=1.0)
    parser.add_argument("--q-omega", type=float, default=0.05)
    parser.add_argument("--q-integral", type=float, default=0.0)
    parser.add_argument("--r", type=float, default=0.02)
    args = parser.parse_args()
    K, P, A, B = design_lqi_gains(
        args.b, args.c, dt=args.dt, q_theta=args.q_theta, q_omega=args.q_omega,
        q_integral=args.q_integral, r=args.r,
    )
    print("K = [K_theta=%.6f, K_omega=%.6f, K_integral=%.6f]" % (K[0], K[1], K[2]))
    eigenvalues = np.linalg.eigvals(A - B @ K.reshape(1, 3))
    print("閉ループの固有値(絶対値が全て1未満なら安定): %s" % np.abs(eigenvalues))
