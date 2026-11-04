"""ETラリー旋回の、LQI(積分付きLQR)による仕上げ制御を設計するための、実測データ収集専用ツール。

ここにあるクラスは、いずれも「本番の走行」には使わない、較正専用の使い捨てビヘイビアである。
既存の EtRallySpinAroundByEncoder / EtRallyRunByGyro(behaviours/et_rally_drive.py)は、
一切変更・再利用(継承)しない。データの取り方だけを、ここで完結させる。

収集するデータは3種類:
  1. 応答特性(HoldSymmetricPower): 静止状態から、一定の出力を短時間与え続け、
     角度・角速度の変化を記録する。モデル(A・B)を最小二乗法で求めるための入力。
  2. 不感帯(FindStictionThreshold): 出力を少しずつ上げ、実際に動き出す最小出力を、
     時計回り・反時計回りそれぞれで特定する。
  3. 検証用(LoggedFineTrim): 小さい角度誤差からの収束を、毎ティックログに残す
     (1・2で作ったモデルとの答え合わせ専用。既存のEtRallySpinAroundByEncoderの
     フェーズ2と同じPID構成を、ログ目的だけのために、ここで独立して再実装する。
     既存クラスへは一切手を入れない)。

ログの読み方: すべての行に "LQI_CAL" という目印を付けている。
    grep "LQI_CAL" run_logs/xxxx.log
で抽出し、wireless_device/et_rally_lqi/parse_log.py で表に変換する。
"""

import math
import time

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence
from simple_pid import PID

from py_etrobo_util import SymmetricClamper

from ..behaviours.conditions import IsTimePassed
from ..behaviours.motor_control import StopNow
from ..gyro_scale import ensure_scaled_gyro
from ..runtime import runtime
from ..timing import CONTROL_INTERVAL_SEC as EXEC_INTERVAL

TAG = "LQI_CAL"


def _current_heading():
    # EtRallySpinAroundByEncoder/EtRallyRunByGyroと同じ符号規約(時計回り正)に揃える。
    # LQIの設計は、この関数で読んだ値をそのまま使う前提で行うため、規約を変えない。
    return (-1) * runtime.course * runtime.gyro_sensor.get_angle()


class HoldSymmetricPower(Behaviour):
    """静止状態から、左右対称の出力を、指定時間だけ与え続け、毎ティック状態をログに残す
    (応答特性データ、目的1)。durationが経過したら、ブレーキして止まりSUCCESSを返す。

    power: 正なら時計回り(direction=+1相当)に回す出力の大きさ(0以上)。
    direction: +1か-1。EtRallySpinAroundByEncoderの`direction`と同じ意味(時計回り正)。
    """

    def __init__(self, name: str, power: int, direction: int, duration_sec: float):
        super().__init__(name)
        if power < 0:
            raise ValueError("power must be non-negative; use direction for the sign")
        if direction not in (1, -1):
            raise ValueError("direction must be 1 or -1")
        self.power = power
        self.direction = direction
        self.duration_sec = duration_sec
        self.running = False

    def update(self) -> Status:
        runtime.require("plotter", "gyro_sensor", "right_motor", "left_motor")
        if not self.running:
            ensure_scaled_gyro()
            self.start_time = time.monotonic()
            self.prev_time = self.start_time
            self.prev_heading = _current_heading()
            self.running = True
            self.logger.info(
                "%s %s.start power=%+d duration=%.2f heading0=%.2f"
                % (TAG, self.__class__.__name__, self.direction * self.power,
                   self.duration_sec, self.prev_heading)
            )

        now = time.monotonic()
        heading = _current_heading()
        dt = max(now - self.prev_time, 1e-6)
        omega = (heading - self.prev_heading) / dt
        elapsed = now - self.start_time
        self.logger.info(
            "%s step power=%+d t=%.3f heading=%.3f omega=%.3f"
            % (TAG, self.direction * self.power, elapsed, heading, omega)
        )
        self.prev_time, self.prev_heading = now, heading

        if elapsed >= self.duration_sec:
            runtime.right_motor.set_power(0)
            runtime.right_motor.set_brake(True)
            runtime.left_motor.set_power(0)
            runtime.left_motor.set_brake(True)
            self.logger.info("%s %s.end heading=%.3f" % (TAG, self.__class__.__name__, heading))
            return Status.SUCCESS

        runtime.right_motor.set_power(runtime.course * self.direction * self.power)
        runtime.left_motor.set_power((-1) * runtime.course * self.direction * self.power)
        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        if runtime.right_motor is not None:
            runtime.right_motor.set_power(0)
        if runtime.left_motor is not None:
            runtime.left_motor.set_power(0)
        self.running = False


class FindStictionThreshold(Behaviour):
    """出力を少しずつ上げながら短時間ずつ保持し、実際に回転が確認できた出力を、
    「不感帯(静止摩擦で動けない範囲)の上限」としてログに残す(目的2)。

    1段階ごとに、settle_sec(静止で待つ)→hold_sec(その出力で保持し、角度変化を見る)
    を繰り返す。hold_sec の間の角度変化が move_threshold_deg を超えたら「動いた」とみなし、
    その出力を記録して終了する。max_power に達しても動かなければ、その旨をログに残して終了する
    (この場合はFAILUREにはせず、SUCCESSのまま次の較正へ進める)。
    """

    def __init__(self, name: str, direction: int, start_power: int = 20, power_step: int = 5,
                 max_power: int = 70, hold_sec: float = 0.3, settle_sec: float = 0.3,
                 move_threshold_deg: float = 1.0):
        super().__init__(name)
        if direction not in (1, -1):
            raise ValueError("direction must be 1 or -1")
        self.direction = direction
        self.start_power = start_power
        self.power_step = power_step
        self.max_power = max_power
        self.hold_sec = hold_sec
        self.settle_sec = settle_sec
        self.move_threshold_deg = move_threshold_deg
        self.running = False

    def _enter_level(self, power):
        self.level_power = power
        self.level_phase = "settle"
        self.phase_started_at = time.monotonic()
        self.level_start_heading = None

    def update(self) -> Status:
        runtime.require("plotter", "gyro_sensor", "right_motor", "left_motor")
        if not self.running:
            ensure_scaled_gyro()
            self.running = True
            self._enter_level(self.start_power)
            self.logger.info(
                "%s %s.start direction=%+d start_power=%d step=%d max=%d"
                % (TAG, self.__class__.__name__, self.direction, self.start_power,
                   self.power_step, self.max_power)
            )

        heading = _current_heading()
        now = time.monotonic()

        if self.level_phase == "settle":
            runtime.right_motor.set_power(0)
            runtime.right_motor.set_brake(True)
            runtime.left_motor.set_power(0)
            runtime.left_motor.set_brake(True)
            if now - self.phase_started_at >= self.settle_sec:
                self.level_phase = "hold"
                self.phase_started_at = now
                self.level_start_heading = heading
            return Status.RUNNING

        # level_phase == "hold": その出力を与え続け、角度変化を見る。
        runtime.right_motor.set_power(runtime.course * self.direction * self.level_power)
        runtime.left_motor.set_power((-1) * runtime.course * self.direction * self.level_power)
        moved_deg = abs(heading - self.level_start_heading)
        elapsed = now - self.phase_started_at
        if moved_deg >= self.move_threshold_deg:
            runtime.right_motor.set_power(0)
            runtime.right_motor.set_brake(True)
            runtime.left_motor.set_power(0)
            runtime.left_motor.set_brake(True)
            self.logger.info(
                "%s threshold direction=%+d power=%d moved=%.2f in %.3fs"
                % (TAG, self.direction, self.level_power, moved_deg, elapsed)
            )
            return Status.SUCCESS
        if elapsed >= self.hold_sec:
            if self.level_power + self.power_step > self.max_power:
                runtime.right_motor.set_power(0)
                runtime.right_motor.set_brake(True)
                runtime.left_motor.set_power(0)
                runtime.left_motor.set_brake(True)
                self.logger.warning(
                    "%s threshold direction=%+d NOT_FOUND up to max_power=%d"
                    % (TAG, self.direction, self.max_power)
                )
                return Status.SUCCESS
            self._enter_level(self.level_power + self.power_step)
        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        if runtime.right_motor is not None:
            runtime.right_motor.set_power(0)
        if runtime.left_motor is not None:
            runtime.left_motor.set_power(0)
        self.running = False


class LoggedFineTrim(Behaviour):
    """小さい角度誤差からの収束を、毎ティックログに残す(目的3、検証専用)。

    既存のEtRallySpinAroundByEncoderのフェーズ2(PID、SymmetricClamper、
    スタック検知によるエスカレーション)と、同じ構成をここで独立に再実装している
    (既存クラスは変更しない。ログの粒度が違うだけで、狙いは同じ動きを再現すること)。
    """

    def __init__(self, name: str, target_relative_deg: float,
                 fine_max_power: int = 60, fine_min_power: int = 50,
                 pid_p: float = 0.2, pid_i: float = 0.00075, pid_d: float = 0.03,
                 fine_tolerance_deg: float = 0.5, stall_deg_per_tick: float = 0.1,
                 stall_tick_limit: int = 10, timeout_sec: float = 3.0):
        super().__init__(name)
        self.target_relative_deg = target_relative_deg
        self.fine_clamper = SymmetricClamper(fine_min_power, fine_max_power)
        self.escalated_clamper = SymmetricClamper(max(int(fine_max_power * 0.8), fine_min_power), fine_max_power)
        self.pid_p, self.pid_i, self.pid_d = pid_p, pid_i, pid_d
        self.fine_tolerance_deg = fine_tolerance_deg
        self.stall_deg_per_tick = stall_deg_per_tick
        self.stall_tick_limit = stall_tick_limit
        self.timeout_sec = timeout_sec
        self.running = False

    def update(self) -> Status:
        runtime.require("plotter", "gyro_sensor", "right_motor", "left_motor")
        current_heading = _current_heading()
        if not self.running:
            ensure_scaled_gyro()
            current_heading = _current_heading()
            self.target_heading = current_heading + self.target_relative_deg
            self.pid = PID(self.pid_p, self.pid_i, self.pid_d,
                           setpoint=self.target_heading, sample_time=EXEC_INTERVAL)
            self.stall_ticks = 0
            self.escalated = False
            self.prev_heading_for_stall = current_heading
            self.start_time = time.monotonic()
            self.running = True
            self.logger.info(
                "%s %s.start target_relative=%+.2f heading0=%.2f"
                % (TAG, self.__class__.__name__, self.target_relative_deg, current_heading)
            )

        error = (float(self.target_heading) - current_heading + 180.0) % 360.0 - 180.0
        if abs(error) < self.fine_tolerance_deg:
            runtime.right_motor.set_power(0)
            runtime.right_motor.set_brake(True)
            runtime.left_motor.set_power(0)
            runtime.left_motor.set_brake(True)
            self.logger.info("%s %s.end heading=%.3f error=%+.3f" % (
                TAG, self.__class__.__name__, current_heading, error))
            return Status.SUCCESS
        if time.monotonic() - self.start_time >= self.timeout_sec:
            runtime.right_motor.set_power(0)
            runtime.right_motor.set_brake(True)
            runtime.left_motor.set_power(0)
            runtime.left_motor.set_brake(True)
            self.logger.warning("%s %s.timeout error=%+.3f" % (TAG, self.__class__.__name__, error))
            return Status.SUCCESS

        heading_progress = abs((current_heading - self.prev_heading_for_stall + 180.0) % 360.0 - 180.0)
        if heading_progress < self.stall_deg_per_tick:
            self.stall_ticks += 1
            if self.stall_ticks > self.stall_tick_limit:
                self.escalated = True
        else:
            self.stall_ticks = 0
        self.prev_heading_for_stall = current_heading

        self.pid.setpoint = current_heading + error
        clamper = self.escalated_clamper if self.escalated else self.fine_clamper
        power = int(clamper.clamp(self.pid(current_heading)))
        runtime.right_motor.set_power(runtime.course * power)
        runtime.left_motor.set_power((-1) * runtime.course * power)
        self.logger.info(
            "%s step heading=%.3f error=%+.3f power=%+d" % (TAG, current_heading, error, power)
        )
        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        if runtime.right_motor is not None:
            runtime.right_motor.set_power(0)
        if runtime.left_motor is not None:
            runtime.left_motor.set_power(0)
        self.running = False


# --- 較正シーケンス全体の組み立て --------------------------------------------

# 応答特性テスト(目的1)の条件。実測しながら調整してよい。
STEP_POWER_LEVELS = (40, 50, 60, 70)
STEP_REPEATS_PER_LEVEL = 3          # 各出力・各向き(交互)で3回ずつ = 1レベルあたり計6回
STEP_DURATION_SEC = 0.4
STEP_SETTLE_SEC = 0.5

# 検証テスト(目的3)の条件。
FINE_TRIM_TARGETS_DEG = (3.0, -3.0, 5.0, -5.0, 2.0, -2.0)


def build_et_rally_lqi_calibration_sequence() -> Sequence:
    root = Sequence(name="et_rally_lqi_calibration", memory=True)
    children = [
        FindStictionThreshold(name="stiction cw", direction=1),
        StopNow(name="settle after stiction cw"),
        IsTimePassed(name="wait after stiction cw", delta_time=STEP_SETTLE_SEC),
        FindStictionThreshold(name="stiction ccw", direction=-1),
        StopNow(name="settle after stiction ccw"),
        IsTimePassed(name="wait after stiction ccw", delta_time=STEP_SETTLE_SEC),
    ]
    for power in STEP_POWER_LEVELS:
        for rep in range(STEP_REPEATS_PER_LEVEL):
            for direction in (1, -1):
                children.append(HoldSymmetricPower(
                    name="step power=%d dir=%+d rep=%d" % (power, direction, rep),
                    power=power, direction=direction, duration_sec=STEP_DURATION_SEC,
                ))
                children.append(StopNow(name="settle power=%d dir=%+d rep=%d" % (power, direction, rep)))
                children.append(IsTimePassed(
                    name="wait power=%d dir=%+d rep=%d" % (power, direction, rep),
                    delta_time=STEP_SETTLE_SEC,
                ))
    for index, target in enumerate(FINE_TRIM_TARGETS_DEG):
        children.append(LoggedFineTrim(name="fine_trim %d target=%+.1f" % (index, target),
                                       target_relative_deg=target))
        children.append(StopNow(name="settle fine_trim %d" % index))
        children.append(IsTimePassed(name="wait fine_trim %d" % index, delta_time=STEP_SETTLE_SEC))
    children.append(StopNow(name="et_rally_lqi_calibration final stop"))
    root.add_children(children)
    return root
