"""DD-19較正・検証専用: 旋回の回転方向(時計回り/反時計回り)を明示的に指定できる、
EtRallySpinAroundByEncoder(et_rally_drive.py)の検証用コピー。

目的: 絶対方位の目標だけでは、旋回方向は「現在角度から目標角度への最短方向」が
自動で選ばれる。目標差がちょうど180度に近いと、実機の数度のブレでこの最短方向の
判定が不安定になり(2026-10-10に実機で確認)、「毎回方向を反転させる」ような
テストプランが意図通りに実行できない。この検証用クラスでは、呼び出し側が
direction(+1/-1)を明示したときはそれを使い、目標角度までその方向で回りきる
(最短方向と逆なら、360度からの残りを回る「長い方」になる)。

production(EtRallySpinAroundByEncoder)は変更せず、このクラスは検証目的のみで
使う(本番の旋回方式には使わない)。フェーズ1(エンコーダ主導)・フェーズ2
(ジャイロ仕上げ)の実装は、direction決定部分以外はet_rally_drive.pyと同一。
"""

import math

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from simple_pid import PID

from py_etrobo_util import SymmetricClamper
from py_etrobo_util.plotter import WHEEL_TREAD

from ..gyro_scale import ensure_scaled_gyro
from ..power_log import OscillationTracker, log_power_state
from ..runtime import runtime
from ..timing import CONTROL_INTERVAL_SEC as EXEC_INTERVAL
from ..types import HeadingType

_TUNED_INTERVAL_SEC = 0.03
_STALL_TICK_LIMIT = math.ceil(7 * _TUNED_INTERVAL_SEC / EXEC_INTERVAL - 1e-9)
_STALL_PROGRESS_DEG = 0.3 * EXEC_INTERVAL / _TUNED_INTERVAL_SEC


class EtRallySpinAroundDirectionTest(Behaviour):
    """forced_direction(+1/-1)を指定すると、その方向に回りきる(0なら自動=最短方向)。"""

    def __init__(self, name: str, target: int,
                 main_power: int, fine_max_power: int, fine_min_power: int,
                 pid_p: float, pid_i: float, pid_d: float, target_type: HeadingType,
                 fine_tolerance_deg: float = 2.0,
                 decel_deg: float = 0.0, decel_power: int = 50,
                 fine_trim: bool = True, forced_direction: int = 0) -> None:
        super(EtRallySpinAroundDirectionTest, self).__init__(name)
        self.decel_deg = decel_deg
        self.decel_power = decel_power
        self.target = target
        self.target_type = target_type
        self.main_power = main_power
        self.pid_p = pid_p
        self.pid_i = pid_i
        self.pid_d = pid_d
        self.fine_tolerance_deg = fine_tolerance_deg
        self.fine_trim = fine_trim
        self.forced_direction = forced_direction
        self.fine_clamper = SymmetricClamper(fine_min_power, fine_max_power)
        self.escalated_clamper = SymmetricClamper(max(int(main_power * 0.8), fine_min_power), main_power)
        self.running = False

    def update(self) -> Status:
        runtime.require("plotter", "gyro_sensor", "right_motor", "left_motor")
        if not self.running:
            ensure_scaled_gyro()
        current_heading = (-1) * runtime.course * runtime.gyro_sensor.get_angle()
        if not self.running:
            if self.target_type == HeadingType.RELATIVE:
                self.target_heading = current_heading + self.target
            else:
                self.target_heading = self.target
            error = (float(self.target_heading) - current_heading + 180.0) % 360.0 - 180.0

            if self.forced_direction == 0 or abs(error) < 1e-9:
                # 自動(最短方向)。既存のEtRallySpinAroundByEncoderと同じ挙動。
                self.direction = 1 if error > 0 else -1
                travel_deg = abs(error)
            else:
                self.direction = self.forced_direction
                shortest_direction = 1 if error > 0 else -1
                if self.direction == shortest_direction:
                    travel_deg = abs(error)
                else:
                    # 指定方向が最短方向と逆: 同じ目標角度に、長い方(360-最短距離)で回る。
                    travel_deg = 360.0 - abs(error)

            arc_length_mm = math.radians(travel_deg) * (WHEEL_TREAD / 2.0)
            self.target_motor_deg = arc_length_mm / (math.pi * runtime.plotter.tire_diameter) * 360.0

            self.start_r = runtime.right_motor.get_count()
            self.start_l = runtime.left_motor.get_count()
            self.right_done = (self.target_motor_deg < 1e-6)
            self.left_done = (self.target_motor_deg < 1e-6)
            self.phase = 1
            self.oscillation = OscillationTracker()

            self.running = True
            self.logger.info(
                "%+06d %s.encoder-spin started at heading=%d for %d direction=%+d "
                "(target_motor_deg=%.1f)" % (
                    runtime.plotter.get_distance(), self.__class__.__name__, current_heading,
                    self.target_heading, self.direction, self.target_motor_deg))

        if self.phase == 1:
            delta_r = abs(runtime.right_motor.get_count() - self.start_r)
            delta_l = abs(runtime.left_motor.get_count() - self.start_l)

            if not self.right_done and delta_r >= self.target_motor_deg:
                self.right_done = True
                runtime.right_motor.set_power(0)
                runtime.right_motor.set_brake(True)
            if not self.left_done and delta_l >= self.target_motor_deg:
                self.left_done = True
                runtime.left_motor.set_power(0)
                runtime.left_motor.set_brake(True)

            if self.right_done and self.left_done:
                if not self.fine_trim:
                    residual = (float(self.target_heading) - current_heading + 180.0) % 360.0 - 180.0
                    self.logger.info(
                        "%+06d %s.encoder-spin ended without fine-trim at heading=%d, residual=%+.1f" % (
                            runtime.plotter.get_distance(), self.__class__.__name__,
                            current_heading, residual))
                    return Status.SUCCESS
                self.phase = 2
                self.pid = PID(self.pid_p, self.pid_i, self.pid_d, setpoint=self.target_heading,
                                sample_time=EXEC_INTERVAL)
                self.stall_ticks = 0
                self.escalated = False
                self.prev_heading_for_stall = current_heading
                self.logger.info(
                    "%+06d %s.encoder-spin phase1 complete at heading=%d, entering gyro fine-trim" % (
                        runtime.plotter.get_distance(), self.__class__.__name__, current_heading))
                return Status.RUNNING

            use_decel = self.decel_deg > 0 and self.target_motor_deg >= 2 * self.decel_deg
            right_power = left_power = 0
            if not self.right_done:
                p = self.decel_power if use_decel and self.target_motor_deg - delta_r < self.decel_deg else self.main_power
                right_power = runtime.course * self.direction * p
                runtime.right_motor.set_power(right_power)
            if not self.left_done:
                p = self.decel_power if use_decel and self.target_motor_deg - delta_l < self.decel_deg else self.main_power
                left_power = (-1) * runtime.course * self.direction * p
                runtime.left_motor.set_power(left_power)
            log_power_state(self.__class__.__name__ + ".phase1", right_power, left_power)
            return Status.RUNNING

        error = (float(self.target_heading) - current_heading + 180.0) % 360.0 - 180.0
        if abs(error) < self.fine_tolerance_deg:
            runtime.right_motor.set_power(0)
            runtime.right_motor.set_brake(True)
            runtime.left_motor.set_power(0)
            runtime.left_motor.set_brake(True)
            self.logger.info("%+06d %s.encoder-spin ended at heading=%d reversals=%d samples=%d" % (
                runtime.plotter.get_distance(), self.__class__.__name__, current_heading,
                self.oscillation.reversal_count, self.oscillation.sample_count))
            return Status.SUCCESS

        heading_progress = abs((current_heading - self.prev_heading_for_stall + 180.0) % 360.0 - 180.0)
        if heading_progress < _STALL_PROGRESS_DEG:
            self.stall_ticks += 1
            if self.stall_ticks > _STALL_TICK_LIMIT:
                self.escalated = True
                self.logger.info("%+06d %s.stalled in fine-trim, escalating power" % (
                    runtime.plotter.get_distance(), self.__class__.__name__))
        else:
            self.stall_ticks = 0
        self.prev_heading_for_stall = current_heading

        self.pid.setpoint = current_heading + error
        clamper = self.escalated_clamper if self.escalated else self.fine_clamper
        power = int(clamper.clamp(self.pid(current_heading)))
        right_power = runtime.course * power
        left_power = (-1) * runtime.course * power
        runtime.right_motor.set_power(right_power)
        runtime.left_motor.set_power(left_power)
        log_power_state(self.__class__.__name__ + ".phase2", right_power, left_power)
        self.oscillation.update(power)
        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        if runtime.right_motor is not None:
            runtime.right_motor.set_power(0)
        if runtime.left_motor is not None:
            runtime.left_motor.set_power(0)
        self.running = False
