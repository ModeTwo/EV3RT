"""ETラリー旋回の、LQI(積分付きLQR)仕上げ版。EtRallySpinAroundByEncoder(et_rally_drive.py)は
一切変更・継承せず、この1クラスに独立して実装する(実測データがまだ無く、検証前のため)。

構成(2026-09の設計検討にもとづく):
  フェーズ1: 既存クラスと同じ、エンコーダで左右対称に回す「その場」保証(この部分はモデル化・
             ゲイン計算の対象にしない。静止摩擦・出力上限が支配的で、線形モデルに向かないため)。
             ただし、目標の手前でdecel_powerへ減速し、フェーズ2へは完全停止せずに引き継ぐ
             (つなぎ目の無駄を無くす)。
  フェーズ2: 状態[角度誤差theta_e, 角速度omega, 角度誤差の積分Integral]に対する、
             状態フィードバック u = -(K1*theta_e + K2*omega + K3*Integral)。
             ゲインK1・K2・K3は、wireless_device/et_rally_lqi/design_gains.py で、
             実測データから求める。**ここにある既定値は、まだ実測前の仮の値**であり、
             実測が揃うまで、そのまま実機の本番経路には組み込まないこと
             (features/execute_strategy.py の steps_from_strategy は、まだこのクラスを使っていない)。

フェーズ2の完了条件は、角度誤差だけでなく角速度も見る(既存クラスとの違い。角度は合っているが
まだ回っている、という状態で終わらせない=揺れの原因を減らす狙い)。
"""

import math
import time

from py_trees.behaviour import Behaviour
from py_trees.common import Status

from py_etrobo_util import SymmetricClamper
from py_etrobo_util.plotter import WHEEL_TREAD

from ..gyro_scale import ensure_scaled_gyro
from ..runtime import runtime
from ..timing import CONTROL_INTERVAL_SEC as EXEC_INTERVAL
from ..types import HeadingType
from .et_rally_drive import TIRE_DIAMETER

# 2026-09時点、実測前の仮のゲイン。design_gains.pyで実測から求めた値に、必ず差し替えること。
# u = -(k_theta*theta_e + k_omega*omega + k_integral*Integral) の3つは、
# design_gains.design_lqi_gains()が返すK=[K_theta,K_omega,K_integral]を、そのまま代入する値。
# 符号は、モデル(b>0, c>0)から機械的に決まるもので、任意に選べる値ではない
# (このモデルでは、安定なK_thetaは負、K_omegaは正になる。simulate.pyの自己診断と同じ符号)。
PLACEHOLDER_K_THETA = -7.8
PLACEHOLDER_K_OMEGA = 3.2
PLACEHOLDER_K_INTEGRAL = -1.0


def _current_heading():
    return (-1) * runtime.course * runtime.gyro_sensor.get_angle()


class EtRallyTurnLqi(Behaviour):
    def __init__(self, name: str, target: float, target_type: HeadingType,
                 main_power: int,
                 decel_deg: float = 15.0, decel_power: int = 50,
                 k_theta: float = PLACEHOLDER_K_THETA, k_omega: float = PLACEHOLDER_K_OMEGA,
                 k_integral: float = PLACEHOLDER_K_INTEGRAL,
                 fine_max_power: int = 60, stiction_floor: float = 0.0,
                 omega_filter_alpha: float = 1.0,
                 fine_tolerance_deg: float = 0.5, fine_omega_tolerance_deg: float = 3.0,
                 max_fine_duration_sec: float = 3.0) -> None:
        super().__init__(name)
        self.target = target
        self.target_type = target_type
        self.main_power = main_power
        self.decel_deg = decel_deg
        self.decel_power = decel_power
        self.k_theta, self.k_omega, self.k_integral = k_theta, k_omega, k_integral
        self.fine_clamper = SymmetricClamper(0, fine_max_power)
        self.stiction_floor = stiction_floor
        # omega_filter_alpha=1.0で無濾波(生の差分そのまま)。0に近いほど強く平滑化する。
        # ジャイロのノイズが分かるまでは、無濾波を既定にしておく。
        self.omega_filter_alpha = omega_filter_alpha
        self.fine_tolerance_deg = fine_tolerance_deg
        self.fine_omega_tolerance_deg = fine_omega_tolerance_deg
        self.max_fine_duration_sec = max_fine_duration_sec
        self.running = False

    # --- フェーズ1: エンコーダ主導(EtRallySpinAroundByEncoderと同じ考え方の、独立実装) ---

    def _enter_phase1(self, current_heading):
        if self.target_type == HeadingType.RELATIVE:
            self.target_heading = current_heading + self.target
        else:
            self.target_heading = self.target
        error = (float(self.target_heading) - current_heading + 180.0) % 360.0 - 180.0
        arc_length_mm = math.radians(abs(error)) * (WHEEL_TREAD / 2.0)
        self.target_motor_deg = arc_length_mm / (math.pi * TIRE_DIAMETER) * 360.0
        self.direction = 1 if error > 0 else -1
        self.start_r = runtime.right_motor.get_count()
        self.start_l = runtime.left_motor.get_count()
        self.right_done = self.target_motor_deg < 1e-6
        self.left_done = self.target_motor_deg < 1e-6
        self.phase = 1
        self.logger.info(
            "%+06d %s.phase1 started at heading=%d for %d (target_motor_deg=%.1f)" % (
                runtime.plotter.get_distance(), self.__class__.__name__, current_heading,
                self.target_heading, self.target_motor_deg))

    def _tick_phase1(self):
        delta_r = abs(runtime.right_motor.get_count() - self.start_r)
        delta_l = abs(runtime.left_motor.get_count() - self.start_l)
        if not self.right_done and delta_r >= self.target_motor_deg:
            self.right_done = True
        if not self.left_done and delta_l >= self.target_motor_deg:
            self.left_done = True
        if self.right_done and self.left_done:
            return True

        use_decel = self.decel_deg > 0 and self.target_motor_deg >= 2 * self.decel_deg
        if not self.right_done:
            p = self.decel_power if use_decel and self.target_motor_deg - delta_r < self.decel_deg else self.main_power
            runtime.right_motor.set_power(runtime.course * self.direction * p)
        if not self.left_done:
            p = self.decel_power if use_decel and self.target_motor_deg - delta_l < self.decel_deg else self.main_power
            runtime.left_motor.set_power((-1) * runtime.course * self.direction * p)
        return False

    # --- フェーズ2: 状態フィードバック(LQI) ---

    def _enter_phase2(self, current_heading):
        # フェーズ1からフェーズ2へは、意図的にブレーキ・完全停止を挟まない(つなぎ目の無駄を無くす)。
        # フェーズ1終盤の減速(decel_power)により、この時点の実際の角速度はすでに小さいはず。
        self.phase = 2
        self.integral = 0.0
        self.prev_heading = current_heading
        self.omega_filtered = 0.0
        self.phase2_started_at = time.monotonic()
        self.logger.info(
            "%+06d %s.phase2(LQI) started at heading=%d" % (
                runtime.plotter.get_distance(), self.__class__.__name__, current_heading))

    def _tick_phase2(self, current_heading):
        # 状態の定義は、較正時(et_rally_lqi_calibration.py)・モデル(design_gains.py)と、
        # 必ず同じ符号規約に揃える: theta_e=目標-現在、omega=現在方位の変化率(current_headingの
        # 微分、符号反転しない)、Integral[k+1]=Integral[k]+dt*theta_e[k]。
        theta_e = (float(self.target_heading) - current_heading + 180.0) % 360.0 - 180.0
        raw_omega = ((current_heading - self.prev_heading + 180.0) % 360.0 - 180.0) / EXEC_INTERVAL
        self.prev_heading = current_heading
        self.omega_filtered += self.omega_filter_alpha * (raw_omega - self.omega_filtered)
        self.integral += EXEC_INTERVAL * theta_e

        if abs(theta_e) < self.fine_tolerance_deg and abs(self.omega_filtered) < self.fine_omega_tolerance_deg:
            self.logger.info(
                "%+06d %s.phase2 converged heading=%d error=%+.2f omega=%+.2f" % (
                    runtime.plotter.get_distance(), self.__class__.__name__,
                    current_heading, theta_e, self.omega_filtered))
            return "converged"
        if time.monotonic() - self.phase2_started_at >= self.max_fine_duration_sec:
            self.logger.warning(
                "%+06d %s.phase2 timeout error=%+.2f omega=%+.2f" % (
                    runtime.plotter.get_distance(), self.__class__.__name__, theta_e, self.omega_filtered))
            return "timeout"

        u = -(self.k_theta * theta_e + self.k_omega * self.omega_filtered + self.k_integral * self.integral)
        u = self.fine_clamper.clamp(u)
        if 0.0 < abs(u) < self.stiction_floor:
            u = math.copysign(self.stiction_floor, u)
        power = int(round(u))
        runtime.right_motor.set_power(runtime.course * power)
        runtime.left_motor.set_power((-1) * runtime.course * power)
        return "running"

    # --- Behaviour本体 ---

    def update(self) -> Status:
        runtime.require("plotter", "gyro_sensor", "right_motor", "left_motor")
        if not self.running:
            ensure_scaled_gyro()
            current_heading = _current_heading()
            self._enter_phase1(current_heading)
            self.running = True

        current_heading = _current_heading()
        if self.phase == 1:
            if self._tick_phase1():
                self._enter_phase2(current_heading)
            return Status.RUNNING

        result = self._tick_phase2(current_heading)
        if result == "running":
            return Status.RUNNING
        runtime.right_motor.set_power(0)
        runtime.right_motor.set_brake(True)
        runtime.left_motor.set_power(0)
        runtime.left_motor.set_brake(True)
        return Status.SUCCESS

    def terminate(self, new_status: Status) -> None:
        if runtime.right_motor is not None:
            runtime.right_motor.set_power(0)
        if runtime.left_motor is not None:
            runtime.left_motor.set_power(0)
        self.running = False
