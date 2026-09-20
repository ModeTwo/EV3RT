"""Reusable gyro drive behaviors."""

import time
import math
from typing import Callable, Union

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from simple_pid import PID

from py_etrobo_util import SymmetricClamper

from ..runtime import runtime
from ..types import HeadingType
from ..path_tracking import PathTracking, curvature_turn


from ..timing import CONTROL_INTERVAL_SEC as EXEC_INTERVAL


def _normalize_heading_error(error: float) -> float:
    # 角度差を-180度以上180度未満へ正規化する。
    return (error + 180.0) % 360.0 - 180.0


def _nearest_equivalent_heading(target: float, current: float) -> float:
    # 絶対方位には360度ごとに同じ向きが存在する。
    # 現在方位から最も近い等価角へ置き換え、PIDへ180度を超える角度差を渡さない。
    return current + _normalize_heading_error(target - current)


class SpinAround(Behaviour):
    # 指定した絶対角度または現在角度からの相対角度まで、その場で旋回する。
    def __init__(
        self,
        name: str,
        target: int,
        max_power: int,
        min_power: int,
        pid_p: float,
        pid_i: float,
        pid_d: float,
        target_type: HeadingType,
        tolerance: float = 2.0,
    ) -> None:
        super().__init__(name)
        self.target = target
        self.target_type = target_type
        self.pid_p = pid_p
        self.pid_i = pid_i
        self.pid_d = pid_d
        self.tolerance = tolerance
        self.clamper = SymmetricClamper(min_power, max_power)
        self.running = False
        self.target_heading = 0.0
        self.pid = None

    def update(self) -> Status:
        runtime.require("plotter", "gyro_sensor", "right_motor", "left_motor")
        current_heading = -runtime.course * runtime.gyro_sensor.get_angle()
        if not self.running:
            if self.target_type == HeadingType.RELATIVE:
                self.target_heading = current_heading + self.target
            else:
                self.target_heading = self.target
            self.pid = PID(
                self.pid_p,
                self.pid_i,
                self.pid_d,
                setpoint=self.target_heading,
                sample_time=EXEC_INTERVAL,
            )
            self.running = True
            self.logger.info(
                "%+06d %s.spin started at heading=%d for %d"
                % (
                    runtime.plotter.get_distance(),
                    self.__class__.__name__,
                    current_heading,
                    self.target_heading,
                )
            )

        error = _normalize_heading_error(self.target_heading - current_heading)
        if abs(error) < self.tolerance:
            self.logger.info(
                "%+06d %s.spin ended at heading=%d"
                % (
                    runtime.plotter.get_distance(),
                    self.__class__.__name__,
                    current_heading,
                )
            )
            return Status.SUCCESS

        # PIDの出力方向は維持しつつ、角度境界をまたぐ場合は正規化した誤差方向を採用する。
        raw_power = float(self.pid(current_heading))
        if raw_power == 0.0:
            raw_power = error
        elif raw_power * error < 0.0:
            raw_power = -raw_power
        power = int(self.clamper.clamp(raw_power))
        runtime.right_motor.set_power(runtime.course * power)
        runtime.left_motor.set_power(-runtime.course * power)
        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        # 旋回完了または中断時にモーター出力を残さない。
        if runtime.right_motor is not None:
            runtime.right_motor.set_power(0)
        if runtime.left_motor is not None:
            runtime.left_motor.set_power(0)
        self.running = False


class RunByGyro(Behaviour):
    """ジャイロで目標角を追従する走行命令。

    target=90.0: 従来どおり一定角度へ走る。
    target=heading_at: 毎周期 heading_at(開始からの距離mm) で目標角を得る。
    関数を渡す場合、ABSOLUTEはIMU方位角、RELATIVEは開始時0度の角度を目標にする。
    関数名には括弧を付けない。PIDは開始時に一度だけ生成する。
    """
    def __init__(
        self,
        name: str,
        target: Union[float, Callable[[float], float]],
        power: int,
        pid_p: float,
        pid_i: float,
        pid_d: float,
        target_type: HeadingType,
        *,
        distance_limit_mm: float = None,
        completion_condition: Behaviour = None,
        completion_min_mm: float = 0.0,
        feedforward_gain: float = 0.0,
        wheel_tread_mm: float = 110.0,
        cross_track_lookahead_mm: float = 0.0,
        max_heading_correction_deg: float = 8.0,
        profile_log_interval_sec: float = 1.0,
    ) -> None:
        super().__init__(name)
        self.target = target
        self.target_type = target_type
        self.power = power
        self.pid_p = pid_p
        self.pid_i = pid_i
        self.pid_d = pid_d
        self.last_log_time = None
        self.running = False
        self.target_heading = 0
        self.pid = None
        self.distance_target = callable(target)
        self.distance_limit_mm = distance_limit_mm
        self.completion_condition = completion_condition
        self.completion_min_mm = completion_min_mm
        # 追加補正は既定で無効。スタート～LAPのfeatureが明示的に有効化する。
        self.feedforward_gain = feedforward_gain
        self.wheel_tread_mm = wheel_tread_mm
        self.cross_track_lookahead_mm = cross_track_lookahead_mm
        self.max_heading_correction_deg = max_heading_correction_deg
        self.profile_log_interval_sec = profile_log_interval_sec
        self.path_tracker = None
        values = (feedforward_gain, wheel_tread_mm, cross_track_lookahead_mm,
                  max_heading_correction_deg, profile_log_interval_sec)
        if (not all(map(math.isfinite, values)) or feedforward_gain < 0 or wheel_tread_mm <= 0
                or cross_track_lookahead_mm < 0 or not 0 <= max_heading_correction_deg <= 45
                or profile_log_interval_sec <= 0):
            raise ValueError('Invalid path tracking settings')
        if not self.distance_target and (feedforward_gain != 0 or cross_track_lookahead_mm != 0):
            raise ValueError('path tracking requires a target function')
        if self.distance_target:
            # 距離関数モードだけの契約。固定角度を使う他工程の仕様は維持。
            if target_type not in (HeadingType.ABSOLUTE, HeadingType.RELATIVE):
                raise ValueError('distance target requires a valid HeadingType')
            if distance_limit_mm is None or not math.isfinite(distance_limit_mm) or distance_limit_mm <= 0:
                raise ValueError('distance_limit_mm must be positive and finite')
            if not math.isfinite(completion_min_mm) or not 0 <= completion_min_mm < distance_limit_mm:
                raise ValueError('completion window must be within distance_limit_mm')
            if not isinstance(power, int) or not 1 <= power <= 100:
                raise ValueError('power must be an integer from 1 to 100')
            if any(not math.isfinite(gain) for gain in (pid_p, pid_i, pid_d)):
                raise ValueError('PID gains must be finite')
        elif distance_limit_mm is not None or completion_condition is not None or completion_min_mm != 0:
            raise ValueError('distance completion options require a target function')

    def update(self) -> Status:
        # 距離プロファイルと固定角度では目標更新方法が異なるため処理を分ける。
        if self.distance_target:
            return self._update_distance_target()
        runtime.require("plotter", "gyro_sensor", "right_motor", "left_motor")
        current_heading = -runtime.course * runtime.gyro_sensor.get_angle()

        # 走行周期への影響を抑えるため、方位ログは1秒に1回だけ出力する。
        if self.last_log_time is None or time.time() - self.last_log_time >= 1.0:
            self.logger.info(
                "%+06d %s.current heading=%d"
                % (runtime.plotter.get_distance(), self.__class__.__name__, current_heading)
            )
            self.last_log_time = time.time()

        if not self.running:
            if self.target_type == HeadingType.RELATIVE:
                self.target_heading = current_heading + self.target
            else:
                # 例: 現在314度、受信目標-44度は円周上では約2度差である。
                # PIDへ-358度差を渡さず、現在値に近い316度として追従する。
                self.target_heading = _nearest_equivalent_heading(self.target, current_heading)
            self.pid = PID(
                self.pid_p,
                self.pid_i,
                self.pid_d,
                setpoint=self.target_heading,
                sample_time=EXEC_INTERVAL,
                output_limits=(-self.power, self.power),
            )
            self.logger.info(
                "%+06d %s.gyro run started at heading=%.1f requested=%.1f resolved=%.1f delta=%.1f"
                % (
                    runtime.plotter.get_distance(),
                    self.__class__.__name__,
                    current_heading,
                    self.target,
                    self.target_heading,
                    self.target_heading - current_heading,
                )
            )
            self.running = True

        turn = int(self.pid(current_heading))
        runtime.right_motor.set_power(self.power + runtime.course * turn)
        runtime.left_motor.set_power(self.power - runtime.course * turn)
        return Status.RUNNING


    def _update_distance_target(self) -> Status:
        """距離関数モードの1周期。順序: 計測→終了判定→目標角→出力。"""
        runtime.require('plotter', 'gyro_sensor', 'right_motor', 'left_motor')
        distance = float(runtime.plotter.get_distance())
        angle = float(runtime.gyro_sensor.get_angle())
        if runtime.course not in (-1, 1) or not all(map(math.isfinite, (distance, angle))):
            return self._finish_distance_run(Status.FAILURE)
        heading = -runtime.course * angle

        # 1. 開始時だけ原点とPIDを作る。目標角が変わってもPIDを作り直さない。
        if not self.running:
            self.origin_distance = distance
            self.previous_heading = heading
            self.continuous_heading = heading
            self.heading_origin = heading
            self.relative_heading = 0.0
            self.path_tracker = (PathTracking(self.cross_track_lookahead_mm, self.max_heading_correction_deg)
                                 if self.cross_track_lookahead_mm > 0 else None)
            self.last_log_time = None
            turn_limit = min(self.power, 100 - self.power)
            self.pid = PID(self.pid_p, self.pid_i, self.pid_d, setpoint=0,
                           sample_time=EXEC_INTERVAL,
                           output_limits=(-turn_limit, turn_limit))
            self.running = True
        progress = distance - self.origin_distance
        if progress < 0:
            return self._finish_distance_run(Status.FAILURE)
        # 359→0度等の折り返しを解除して開始からの連続角を得る。
        self.continuous_heading += _normalize_heading_error(heading - self.previous_heading)
        self.previous_heading = heading
        self.relative_heading = self.continuous_heading - self.heading_origin
        actual_heading = (self.continuous_heading
                          if self.target_type == HeadingType.ABSOLUTE
                          else self.relative_heading)

        # 2. 終了条件なし: 距離で成功。あり: 検知で成功、距離上限で失敗。
        if self.completion_condition is not None and progress >= self.completion_min_mm:
            self.completion_condition.tick_once()
            if self.completion_condition.status == Status.SUCCESS:
                return self._finish_distance_run(Status.SUCCESS)
        if progress >= self.distance_limit_mm:
            result = Status.SUCCESS if self.completion_condition is None else Status.FAILURE
            return self._finish_distance_run(result)

        # 3. featureから渡された関数をここで呼ぶ。単位はmm→度。
        nominal_heading = float(self.target(progress))
        if not math.isfinite(nominal_heading):
            return self._finish_distance_run(Status.FAILURE)
        # 推定横ずれがあるときは、計画の向きへ戻すため小さな追加角度を与える。
        correction = (self.path_tracker.correction(progress, actual_heading, self.target)
                      if self.path_tracker is not None else 0.0)
        self.target_heading = nominal_heading + correction
        turn_limit = min(self.power, 100-self.power)
        feedforward = curvature_turn(self.target, progress, self.power,
                                     self.wheel_tread_mm, self.feedforward_gain)
        feedforward = max(-turn_limit, min(turn_limit, feedforward))
        # 曲線に必要な出力を先に用意し、PIDは残った誤差を補正する。
        # FF分を差し引いた範囲に制限し、合計PWMの飽和時も積分を制限する。
        self.pid.output_limits = (-turn_limit-feedforward, turn_limit-feedforward)
        self.pid.setpoint = self.target_heading
        feedback = self.pid(actual_heading)
        turn = max(-turn_limit, min(turn_limit, feedforward+feedback))

        # 4. 基本旋回＋PID補正の合計を左右コースへ変換して出力する。
        runtime.right_motor.set_brake(False)
        runtime.left_motor.set_brake(False)
        runtime.right_motor.set_power(round(self.power + runtime.course * turn))
        runtime.left_motor.set_power(round(self.power - runtime.course * turn))
        now = time.monotonic()
        if self.last_log_time is None or now-self.last_log_time >= self.profile_log_interval_sec:
            p, i, d = self.pid.components
            xte = self.path_tracker.cross_track_mm if self.path_tracker else 0.0
            self.logger.info(
                'profile s=%.1f nominal=%.2f target=%.2f actual=%.2f error=%.2f '
                'xte_est=%.1f correction=%.2f ff=%.2f p=%.2f i=%.2f d=%.2f '
                'turn=%.2f left=%d right=%d' %
                (progress, nominal_heading, self.target_heading, actual_heading,
                 self.target_heading-actual_heading, xte, correction, feedforward, p, i, d,
                 turn, round(self.power-runtime.course*turn), round(self.power+runtime.course*turn)))
            self.last_log_time = now
        return Status.RUNNING

    def _finish_distance_run(self, status: Status) -> Status:
        # updateを直接呼ぶ場合も、終了時の出力0を保証する。
        self.terminate(status)
        return status


    def terminate(self, new_status: Status) -> None:
        # 惰性走行によるオーバーシュートを防ぐため即時ブレーキをかける。
        for motor in (runtime.left_motor, runtime.right_motor):
            if motor is not None:
                motor.set_power(0)
                motor.set_brake(True)
        self.running = False
