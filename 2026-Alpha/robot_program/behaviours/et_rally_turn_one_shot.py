"""ETラリー旋回の、「一発サイズの補正」仕上げ版。EtRallySpinAroundByEncoder
(et_rally_drive.py)・EtRallyTurnLqi(et_rally_turn_lqi.py)のどちらも変更・継承せず、
この1クラスに独立して実装する(実測データがまだ無く、検証前のため)。

【位置づけ(2026-09、会話にもとづく優先順位の見直し)】
実機で確認された「旋回時、細かく左右に動く」現象は、毎tickフィードバックし
続ける制御則(PID・LQIどちらも同じ)の問題というより、最小出力(不感帯を超える
下限パワー)の粒度が粗いことが原因と診断した。この仕上げ版は、フィードバックを
繰り返す代わりに、較正データから計算した「狙った量だけ動く一発のパルス」を
与えて止める、という発想で作る。

構成:
  フェーズ1: 既存クラスと同じ、エンコーダで左右対称に回す「その場」保証。
  フェーズ2: PULSE_TABLE(wireless_device/et_rally_lqi/one_shot_pulse.pyで、
             実測データから計算する)を引いて、残差にちょうど合うtick数だけ
             floor_powerを与えて停止する。1回で収まらなければ、残差を測り直して
             もう1回だけ試す(最大2回、それ以上は追いかけない=不感帯の粒度より
             細かい誤差を無理に追わない設計)。

PULSE_TABLE/FLOOR_POWERは、まだ実測前の仮の値。
wireless_device/et_rally_lqi/one_shot_pulse.py の build_pulse_table() で、
実測したb, c, stiction_floorから計算した表に、必ず差し替えること。
"""
import math

from py_trees.behaviour import Behaviour
from py_trees.common import Status

from py_etrobo_util.plotter import WHEEL_TREAD

from ..gyro_scale import ensure_scaled_gyro
from ..power_log import log_power_state
from ..runtime import runtime
from ..types import HeadingType
from .et_rally_drive import TIRE_DIAMETER

# 2026-09時点、実測前の仮の較正値・仮のパルス表。
# b=0.5, c=1.5, stiction_floor=22, power=25(floor_powerぎりぎり上)という仮定で、
# wireless_device/et_rally_lqi/one_shot_pulse.build_pulse_table()を実際に実行して
# 生成した値(仮のb, c, stiction_floorを使っているだけで、計算そのものは本物)。
# 実測後は、実測したb, c, stiction_floorで同関数を実行し、この表を丸ごと差し替えること。
#
# 【重要・未解決の論点】この仮の数値で計算すると、5度の残差を1発で補正するのに
# 約280tick(5.6秒)かかる計算になった(floor_powerが不感帯ぎりぎりで、実効トルクが
# 弱いため)。これでは「瞬間的」という目標に対して遅すぎる可能性が高い。
# 実測後、残差が大きい場合はfloor_powerより高いパワーを使う(段階的に、または
# 残差の大きさに応じてパワーを変える)設計に見直す必要があるかもしれない
# (このファイルの現状は、あくまで「一発サイズで狙って止める」という考え方の
# 実装であり、パワーの決め方そのものは未検証・要改善)。
FLOOR_POWER = 25  # 【要実測】不感帯を確実に超える最小パワー(FindStictionThreshold参照)
PULSE_TABLE = [
    (0.25, 33, 0.25), (0.5, 50, 0.4943), (0.75, 65, 0.7426), (1.0, 79, 0.9916),
    (1.25, 93, 1.2514), (1.5, 106, 1.4989), (1.75, 119, 1.7506), (2.0, 132, 2.0049),
    (2.25, 144, 2.2414), (2.5, 157, 2.4988), (2.75, 170, 2.757), (3.0, 182, 2.9959),
    (3.25, 195, 3.255), (3.5, 207, 3.4945), (3.75, 220, 3.7541), (4.0, 232, 3.9939),
    (4.25, 245, 4.2537), (4.5, 257, 4.4936), (4.75, 270, 4.7535), (5.0, 282, 4.9935),
]
MAX_PULSE_ATTEMPTS = 2  # これ以上は追いかけない(不感帯より細かい残差を無理に追わない)


def _current_heading():
    return (-1) * runtime.course * runtime.gyro_sensor.get_angle()


def _lookup_ticks(target_deg, table):
    """target_deg(符号つき)に一番近い表の行から、tick数を引く。0未満ならtick=0。"""
    target_abs = abs(target_deg)
    if target_abs < 1e-9 or not table:
        return 0
    best = min(table, key=lambda row: abs(row[0] - target_abs))
    return best[1]


class EtRallyTurnOneShot(Behaviour):
    def __init__(self, name: str, target: float, target_type: HeadingType,
                 main_power: int, decel_deg: float = 15.0, decel_power: int = 50,
                 floor_power: int = FLOOR_POWER, pulse_table=None,
                 fine_tolerance_deg: float = 0.5, max_pulse_attempts: int = MAX_PULSE_ATTEMPTS) -> None:
        super().__init__(name)
        self.target = target
        self.target_type = target_type
        self.main_power = main_power
        self.decel_deg = decel_deg
        self.decel_power = decel_power
        self.floor_power = floor_power
        self.pulse_table = pulse_table if pulse_table is not None else PULSE_TABLE
        self.fine_tolerance_deg = fine_tolerance_deg
        self.max_pulse_attempts = max_pulse_attempts
        self.running = False

    # --- フェーズ1: エンコーダ主導(EtRallySpinAroundByEncoder/EtRallyTurnLqiと同じ考え方) ---

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
        return False

    # --- フェーズ2: 一発サイズのパルス補正 ---

    def _enter_phase2(self, current_heading):
        self.phase = 2
        self.pulse_attempt = 0
        self._start_pulse(current_heading)

    def _start_pulse(self, current_heading):
        error = (float(self.target_heading) - current_heading + 180.0) % 360.0 - 180.0
        ticks = _lookup_ticks(error, self.pulse_table)
        self.pulse_ticks_remaining = ticks
        self.pulse_power = int(math.copysign(self.floor_power, error)) if ticks > 0 else 0
        self.pulse_attempt += 1
        self.logger.info(
            "%+06d %s.phase2 pulse#%d error=%+.2f ticks=%d power=%+d" % (
                runtime.plotter.get_distance(), self.__class__.__name__,
                self.pulse_attempt, error, ticks, self.pulse_power))
        if ticks == 0:
            self._stop_motors()

    def _stop_motors(self):
        runtime.right_motor.set_power(0)
        runtime.right_motor.set_brake(True)
        runtime.left_motor.set_power(0)
        runtime.left_motor.set_brake(True)

    def _tick_phase2(self, current_heading):
        if self.pulse_ticks_remaining > 0:
            right_power = runtime.course * self.pulse_power
            left_power = (-1) * runtime.course * self.pulse_power
            runtime.right_motor.set_power(right_power)
            runtime.left_motor.set_power(left_power)
            log_power_state(self.__class__.__name__ + ".phase2", right_power, left_power, interval_ticks=1)
            self.pulse_ticks_remaining -= 1
            return "running"

        self._stop_motors()
        error = (float(self.target_heading) - current_heading + 180.0) % 360.0 - 180.0
        if abs(error) < self.fine_tolerance_deg or self.pulse_attempt >= self.max_pulse_attempts:
            self.logger.info(
                "%+06d %s.phase2 done attempts=%d error=%+.2f" % (
                    runtime.plotter.get_distance(), self.__class__.__name__,
                    self.pulse_attempt, error))
            return "done"
        # 1回で収まらなかった場合、残差を測り直してもう1回だけ試す。
        self._start_pulse(current_heading)
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
        return Status.SUCCESS

    def terminate(self, new_status: Status) -> None:
        if runtime.right_motor is not None:
            runtime.right_motor.set_power(0)
        if runtime.left_motor is not None:
            runtime.left_motor.set_power(0)
        self.running = False
