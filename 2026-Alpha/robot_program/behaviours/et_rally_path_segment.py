"""ETラリー専用: 直進・円弧を、前進/後退・止まる/止まらずに続ける、の組み合わせで走るノード。

2026-11-01: 往復・後退でのゲート通過と、つなぎ区間の円弧化を試すために追加した
(計画JSONの move/arc に reverse・stop の指定があるときだけ execute_strategy が使う。
指定のない従来の計画は、これまでどおり EtRallyRunByGyro で走る)。

- 向きは進んだ距離に合わせて目標を動かし、EtRallyRunByGyroと同じくジャイロのPIDで追う。
  円弧では、半径から決まる左右の差(フィードフォワード)を足してから、PIDでずれだけを直す。
- 円弧では、外側の車輪が巡航の出力を超えないよう、中心の出力を 1/(1+半輪距/半径) に落とす。
- 止まらずに続く区間(stop=False)では、次の区間の距離を「この区間の計画上の終わり」から数える
  (区間の切り替わりの判定の刻み・行き過ぎが、区間ごとにたまらないように)。PIDも次の区間へ引き継ぐ。
- 減速は、止まるまでに続く区間全体の残り距離(remaining_after_mm を足したもの)で決める。
- stop=True の区間の終わりでは、ブレーキをかけて左右のエンコーダが止まるまで待ってから終える
  (直後に前後が入れ替わっても、惰性のまま逆へ出力をかけないため)。
"""

import math
import time

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from simple_pid import PID

from py_etrobo_util.plotter import WHEEL_TREAD

from ..gyro_scale import ensure_scaled_gyro
from ..et_rally_heading import et_pose_text
from ..power_log import log_power_state
from ..runtime import runtime
from ..timing import CONTROL_INTERVAL_SEC as EXEC_INTERVAL
from . import et_rally_drive

HALF_TREAD_MM = WHEEL_TREAD / 2.0
# 止まるとき: 左右のエンコーダがこのtick数だけ続けて変わらなければ止まったとみなす(one_shotのpre_stopと同じ)。
STOP_STABLE_TICKS = 2
STOP_MAX_TICKS = 25

# 止まらずに続く区間の受け渡し(次の区間の距離の起点と、引き継ぐPID)。
_chain_end_dist = None
_chain_pid = None


def reset_chain():
    global _chain_end_dist, _chain_pid
    _chain_end_dist = None
    _chain_pid = None


def _heading():
    return (-1) * runtime.course * runtime.gyro_sensor.get_angle()


class EtRallyPathSegment(Behaviour):
    def __init__(self, name: str, kind: str, end_heading_deg: float, length_mm: float, power: int,
                 pid_p: float, pid_i: float, pid_d: float,
                 sweep_deg: float = 0.0, radius_mm: float = 0.0, reverse: bool = False,
                 stop: bool = True, continue_from_prev: bool = False, remaining_after_mm: float = 0.0,
                 stop_lead_mm: float = 0.0, accel_mm: float = 0.0, accel_min_power: int = 40,
                 decel_mm: float = 0.0, decel_min_power: int = 40,
                 next_profile=(), lead_sec: float = 0.0, arc_pid_i: float = 0.6) -> None:
        super().__init__(name)
        if kind not in ("line", "arc"):
            raise ValueError("kind must be line or arc")
        self.kind = kind
        self.end_heading = float(end_heading_deg)
        self.sweep = float(sweep_deg) if kind == "arc" else 0.0
        self.start_heading = self.end_heading - self.sweep
        self.radius_mm = float(radius_mm)
        self.length_mm = float(length_mm)
        self.power = power
        self.pid_gains = (pid_p, pid_i, pid_d)
        self.reverse = reverse
        self.stop_at_end = stop
        self.continue_from_prev = continue_from_prev
        self.remaining_after_mm = remaining_after_mm
        # 止まる区間だけ、惰性の分を手前で終える(続く区間は計画どおりの距離で次へ渡す)。
        self.run_length_mm = max(self.length_mm - stop_lead_mm, 1.0) if stop else self.length_mm
        self.accel_mm = 0.0 if continue_from_prev else accel_mm
        self.accel_min_power = min(accel_min_power, power)
        self.decel_mm = decel_mm  # 続く区間でも、止まるまでの全体の残りが短ければ減速する
        self.decel_min_power = min(decel_min_power, power)
        if kind == "arc" and self.radius_mm <= 0:
            raise ValueError("radius_mm must be positive")
        # 先読み: 止まるまでに続く区間の形(kind, 長さ, 始めの向き, 向きの変化, 半径)。
        # 車輪の速さが出力に追いつくまでの遅れの分(lead_sec × 今の速さ)だけ先の曲がり具合で
        # 左右の差を出し、曲がり始め・曲がり終わりを早める(遅れたまま曲がると、その分だけ経路がずれる)。
        self.profile = [(kind, self.length_mm, self.start_heading, self.sweep, self.radius_mm)] + list(next_profile)
        self.lead_sec = lead_sec
        # 2026-11-01: 円弧の中だけ向きのPIDのIを弱くする(直進用の強いI(--rally-move-pid-i 8など)のままだと、
        # 目標の向きが動き続ける円弧で出力が約0.5秒周期で揺れた。run_20261101T061925)。
        self.arc_pid_i = arc_pid_i
        self.running = False
        self.stopping = False

    # --- 内部 ---
    def _profile_at(self, s):
        """この区間の始めから距離sの点の(目標の向き, 左右差の比, 出力の倍率)。続く区間へはみ出してもよい。"""
        last = len(self.profile) - 1
        for i, (kind, length, h0, sweep, radius) in enumerate(self.profile):
            if s > length and i < last:
                s -= length
                continue
            if s > length:
                # 止まるまでに続く区間の、さらに先: 最後の向きのまま、まっすぐ
                return h0 + sweep, 0.0, 1.0
            if kind == "arc" and radius > 0 and length > 0:
                ratio = max(0.0, s / length)
                return (h0 + sweep * ratio, math.copysign(HALF_TREAD_MM / radius, sweep),
                        1.0 / (1.0 + HALF_TREAD_MM / radius))
            return h0 + sweep, 0.0, 1.0
        return self.end_heading, 0.0, 1.0

    def _target_heading(self, s):
        if self.kind == "line" or self.length_mm <= 0:
            return self.end_heading
        return self.start_heading + self.sweep * max(0.0, min(1.0, s / self.length_mm))

    def _start(self):
        global _chain_pid
        ensure_scaled_gyro()
        now = runtime.plotter.get_distance()
        if self.continue_from_prev and _chain_end_dist is not None:
            self.orig_dist = _chain_end_dist
        else:
            self.orig_dist = now
        if self.continue_from_prev and _chain_pid is not None:
            self.pid = _chain_pid
            self.pid.output_limits = (-self.power, self.power)
        else:
            pid_i = self.pid_gains[1] if et_rally_drive._move_pid_i_override is None else et_rally_drive._move_pid_i_override
            self.pid = PID(self.pid_gains[0], pid_i, self.pid_gains[2], setpoint=self._target_heading(0.0),
                           sample_time=EXEC_INTERVAL, output_limits=(-self.power, self.power))
        _chain_pid = None
        # 区間をまたいで引き継いだPIDでも、Iの係数はこの区間の種類に合わせる(たまっているIの値はそのまま)。
        line_i = self.pid_gains[1] if et_rally_drive._move_pid_i_override is None else et_rally_drive._move_pid_i_override
        self.pid.Ki = self.arc_pid_i if self.kind == "arc" else line_i
        self.saturated_ticks = 0
        self.right_residual = 0.0
        self.left_residual = 0.0
        self.err_sum = 0.0
        self.err_n = 0
        self.err_max = 0.0
        self.lat_mm = 0.0
        self.prev_dist = now
        self.speed_mm_s = 0.0
        self.start_heading_actual = _heading()
        self.running = True
        self.logger.info("%+06d %s.%s started %s heading %.1f->%.1f length=%d radius=%d stop=%s cont=%s rest=%d loc=(%d,%d)%s" % (
            now, self.__class__.__name__, self.kind, "reverse" if self.reverse else "forward",
            self.start_heading, self.end_heading, self.length_mm, self.radius_mm, self.stop_at_end,
            self.continue_from_prev, self.remaining_after_mm,
            runtime.plotter.get_loc_x(), runtime.plotter.get_loc_y(), et_pose_text()))

    def _finish_log(self, now, tag=""):
        current = _heading()
        self.logger.info("%+06d %s.%s summary: dist=%d planned=%d end_heading=%.1f target=%.1f err_mean=%+.2f err_max=%.2f lat=%+.0fmm sat=%d loc=(%d,%d)%s%s" % (
            now, self.__class__.__name__, self.kind, now - self.orig_dist, self.run_length_mm,
            current, self.end_heading, self.err_sum / max(self.err_n, 1), self.err_max, self.lat_mm,
            self.saturated_ticks,
            runtime.plotter.get_loc_x(), runtime.plotter.get_loc_y(), et_pose_text(), tag))

    def update(self) -> Status:
        global _chain_end_dist, _chain_pid
        runtime.require("plotter", "gyro_sensor", "right_motor", "left_motor")
        if not self.running:
            self._start()

        if self.stopping:
            counts = (runtime.right_motor.get_count(), runtime.left_motor.get_count())
            self.stop_ticks += 1
            self.stop_stable = self.stop_stable + 1 if counts == self.stop_prev else 0
            self.stop_prev = counts
            if self.stop_stable < STOP_STABLE_TICKS and self.stop_ticks < STOP_MAX_TICKS:
                return Status.RUNNING
            self._finish_log(runtime.plotter.get_distance(), " stopped ticks=%d%s" % (
                self.stop_ticks, "" if self.stop_stable >= STOP_STABLE_TICKS else " (timeout)"))
            return Status.SUCCESS

        now = runtime.plotter.get_distance()
        s = now - self.orig_dist
        if s >= self.run_length_mm:
            if self.stop_at_end:
                _chain_end_dist = None
                _chain_pid = None
                runtime.right_motor.set_power(0)
                runtime.right_motor.set_brake(True)
                runtime.left_motor.set_power(0)
                runtime.left_motor.set_brake(True)
                self.stopping = True
                self.stop_ticks = 0
                self.stop_stable = 0
                self.stop_prev = (runtime.right_motor.get_count(), runtime.left_motor.get_count())
                self.reached_dist = now
                return Status.RUNNING
            # 続く区間: 次の区間の距離の起点を、計画上の終わりにする。モーターは止めない。
            _chain_end_dist = self.orig_dist + self.length_mm
            _chain_pid = self.pid
            self._finish_log(now)
            return Status.SUCCESS

        current = _heading()
        self.speed_mm_s += ((now - self.prev_dist) / EXEC_INTERVAL - self.speed_mm_s) * 0.3
        # 目標の向きは今の位置のもの。左右の差(フィードフォワード)と出力の倍率だけを先読みする
        # (車輪の速さが出力に追いつくまでの遅れを打ち消す。目標の向きまで先読みすると曲がり始めが早すぎる)。
        target, _, here_scale = self._profile_at(s)
        _, ff_ratio, look_scale = self._profile_at(s + self.lead_sec * max(self.speed_mm_s, 0.0))
        error = (target - current + 180.0) % 360.0 - 180.0
        self.pid.setpoint = current + error
        self.lat_mm += (now - self.prev_dist) * math.sin(math.radians(-error))
        self.prev_dist = now
        self.err_sum += error
        self.err_n += 1
        self.err_max = max(self.err_max, abs(error))

        # 出力: 巡航 → 円弧なら外側が巡航を超えない値 → 発進の加速・止まる前の減速の小さいほう
        power_now = self.power * min(look_scale, here_scale)
        if self.accel_mm > 0.0 and s < self.accel_mm:
            ratio = max(s, 0.0) / self.accel_mm
            power_now = min(power_now, self.accel_min_power + (self.power - self.accel_min_power) * ratio)
        if self.decel_mm > 0.0:
            remaining = (self.run_length_mm - s) + self.remaining_after_mm
            if remaining < self.decel_mm:
                ratio = max(remaining, 0.0) / self.decel_mm
                power_now = min(power_now, self.decel_min_power + (self.power - self.decel_min_power) * ratio)

        output = self.pid(current)
        ff = abs(power_now) * ff_ratio
        turn = max(-power_now, min(power_now, output)) + ff
        base = -power_now if self.reverse else power_now
        # 2026-11-01: 左右の出力を±100に収める。左右の差(曲がる分)を優先して残し、はみ出す分は全体の出力を下げる
        # (円弧の入りで右+131・左-29のように上限を超え、超えた分が削られて曲がり方が計画と変わっていた)。
        if abs(turn) > 100.0:
            turn = math.copysign(100.0, turn)
        headroom = 100.0 - abs(turn)
        if abs(base) > headroom:
            base = math.copysign(headroom, base)
            self.saturated_ticks += 1
        self.right_residual += base + runtime.course * turn
        self.left_residual += base - runtime.course * turn
        right_power = int(round(self.right_residual))
        left_power = int(round(self.left_residual))
        self.right_residual -= right_power
        self.left_residual -= left_power
        runtime.right_motor.set_power(right_power)
        runtime.left_motor.set_power(left_power)
        log_power_state(self.__class__.__name__ + "." + self.kind, right_power, left_power)
        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        # 続く区間ではモーターを止めない(次の区間がすぐ出力を上書きする)。中断(INVALID)のときだけ止める。
        if new_status == Status.INVALID:
            if runtime.right_motor is not None:
                runtime.right_motor.set_power(0)
            if runtime.left_motor is not None:
                runtime.left_motor.set_power(0)
            reset_chain()
        self.running = False
        self.stopping = False
