"""Feature 15: move from the rally exit to the sumo search position."""

import math
from typing import Callable, Union

from simple_pid import PID

from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time

from ..behaviours.conditions import IsColorDetected, IsDistanceEarned, IsTimePassed
from ..behaviours.gyro_drive import RunByGyro, SpinAround
from ..behaviours.motor_control import StopNow, RunAsInstructed
from ..path_tracking import PathTracking, curvature_turn
from ..timing import CONTROL_INTERVAL_SEC as EXEC_INTERVAL
from .sumo_bearing_motion import RunAtBearing, EncoderSpinToBearing as SpinToBearing, current_bearing


def _normalize_heading_error(error: float) -> float:
    # 角度差を-180度以上180度未満へ正規化する。
    return (error + 180.0) % 360.0 - 180.0


# マイナスジャイロでバックするクラス。No.15開始位置補正専用
# (元はstart_to_lap_gate.pyにあったものを、相撲開始位置への移動という
# 目的に合わせてこちらへ移した)。
class RunByGyroMinusBack(Behaviour):
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
                self.target_heading = self.target #_nearest_equivalent_heading(self.target, current_heading)
            limit = abs(self.power)
            self.pid = PID(
                self.pid_p,
                self.pid_i,
                self.pid_d,
                setpoint=self.target_heading,
                sample_time=EXEC_INTERVAL,
                output_limits=(-limit, limit),
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
            turn_limit = abs(self.power)
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


class ConfigureCameraRetreatPwm(Behaviour):
    # RunAsInstructedがcourse符号を掛ける前の値を調整し、両コースで実出力を後退にする。
    def __init__(self, name, motor_command, power):
        super().__init__(name)
        self.motor_command = motor_command
        self.power = power

    def update(self):
        runtime.require("right_motor", "left_motor")
        command_power = -self.power if runtime.course >= 0 else self.power
        self.motor_command.pwm_l = command_power
        self.motor_command.pwm_r = command_power
        return Status.SUCCESS


class IsBlackThenBrightSurface(Behaviour):
    # ET相撲開始位置専用。黒線を1回確認した後、明るい路面が継続したことを生V値で判定する。
    # 共通色分類の彩度条件や5サンプル多数決に依存させず、白地がUNKNOWNになる影響を避ける。
    def __init__(self, name, settings):
        super().__init__(name)
        self.settings = settings
        self.black_seen = False
        self.bright_started_at = None
        self.last_log_at = None

    def update(self):
        runtime.require("plotter", "color_sensor")
        h, s, v = runtime.color_sensor.get_raw_color_hsv()
        now = time.monotonic()

        # 実機調整時に閾値の妥当性を判断できるよう、生HSVと判定段階を定期出力する。
        if (
            self.last_log_at is None
            or now - self.last_log_at >= self.settings.line_sensor_log_interval_sec
        ):
            stage = "waiting_black" if not self.black_seen else "waiting_bright"
            self.logger.info(
                "%+06d %s.hsv=(%d,%d,%d) stage=%s"
                % (
                    runtime.plotter.get_distance(),
                    self.__class__.__name__,
                    h,
                    s,
                    v,
                    stage,
                )
            )
            self.last_log_at = now

        # 実行単位1：青円上などの明るい開始地点では成功させず、黒線を先に確認する。
        if not self.black_seen:
            if v <= self.settings.line_black_max_value:
                self.black_seen = True
                self.logger.info(
                    "%+06d %s.black line confirmed v=%d"
                    % (
                        runtime.plotter.get_distance(),
                        self.__class__.__name__,
                        v,
                    )
                )
            return Status.RUNNING

        # 実行単位2：黒線確認後、Vが閾値以上の状態が連続したら白地へ抜けたと判断する。
        if v >= self.settings.line_white_min_value:
            if self.bright_started_at is None:
                self.bright_started_at = now
            if now - self.bright_started_at >= self.settings.line_exit_white_duration_sec:
                self.logger.info(
                    "%+06d %s.bright surface confirmed v=%d for %.3fs"
                    % (
                        runtime.plotter.get_distance(),
                        self.__class__.__name__,
                        v,
                        self.settings.line_exit_white_duration_sec,
                    )
                )
                return Status.SUCCESS
        else:
            self.bright_started_at = None
        return Status.RUNNING


class InitializeSumoState(Behaviour):
    # No.16以降で使用する探索基準方位と実行状態を初期化する。
    def __init__(self, name, context):
        super().__init__(name)
        self.context = context

    def update(self):
        runtime.require("plotter", "gyro_sensor")
        state = self.context.sumo
        state.started_at = time.monotonic()
        state.search_bearing_deg = current_bearing(self.context)
        state.sonar_samples.clear()
        state.bottle_bearing_deg = None
        state.bottle_distance_mm = None
        state.approach_distance_mm = 0.0
        state.camera_capture_bearing_deg = None
        state.skipped = False
        state.bottle_captured = False
        state.bottle_pushed_out = False
        state.bottle_released = False
        state.transport_completed = False
        state.bottle_held_at_exit = False
        state.garage_line_found = False
        state.line_trace_ready = False
        state.failure_reason = None
        self.logger.info(
            "%+06d %s.camera capture bearing=%.1f"
            % (
                runtime.plotter.get_distance(),
                self.__class__.__name__,
                state.search_bearing_deg,
            )
        )
        return Status.SUCCESS


def build_move_to_sumo_start(context, config):
    # No.15：初期位置から350mm直進して停止し、土俵側へコース図の土俵方位へ旋回する。
    settings = config.sumo

    # LAPゲート終了直後の実際の停止位置が、この後のNo.15(下記drive以降)が
    # 前提とする初期位置からズレているため、後退→+90度旋回→前進(黒即停止/
    # 青+extra/距離上限)→180度旋回で相撲開始位置へ合わせ直す。絶対方位を
    # 直接使う旧来のgyro_drive方式で、他のNo.15ロジックが使うRunAtBearing/
    # SpinToBearingの方位抽象とは別系統(元はstart_to_lap_gate.py側にあった)。
    back_by_gyro = Parallel(
        name='back by gyro',
        policy=ParallelPolicy.SuccessOnOne(),
    )
    back_by_gyro.add_children([
        RunByGyroMinusBack(
            name="backward_50cm_abs0",
            target=0,  # 絶対角度0°
            power=-settings.reposition_backward_power,  # 負のパワーで後退
            pid_p=settings.reposition_backward_pid_p,
            pid_i=settings.reposition_backward_pid_i,
            pid_d=settings.reposition_backward_pid_d,
            target_type=HeadingType.ABSOLUTE,
        ),
        IsDistanceEarned(
            name="backward_50cm_done",
            delta_dist=settings.reposition_backward_distance_mm,
        ),
    ])

    turn_plus_90 = SpinAround(
        name='turn to plus 90 after backward',
        target=90.0,
        max_power=settings.reposition_turn_max_power,
        min_power=settings.reposition_turn_min_power,
        pid_p=settings.reposition_turn_pid_p,
        pid_i=settings.reposition_turn_pid_i,
        pid_d=settings.reposition_turn_pid_d,
        target_type=HeadingType.ABSOLUTE,
    )

    # +90°を保持したまま、黒検知で即停止、青検知はさらに20mm進んでから停止、
    # またはどちらも検知しないまま距離上限まで進んだら停止する。
    stop_after_blue_plus_20mm = Sequence(
        name='blue then 20mm more', memory=True
    )
    stop_after_blue_plus_20mm.add_children([
        IsColorDetected('detect blue after turn90', Color.BLUE),
        IsDistanceEarned(
            name='advance_after_turn90_blue_extra_distance',
            delta_dist=settings.reposition_advance_blue_extra_mm,
        ),
    ])

    advance_after_turn90 = Parallel(
        name='advance at plus 90 until black, blue plus 20mm, or distance limit',
        policy=ParallelPolicy.SuccessOnOne(),
    )
    advance_after_turn90.add_children([
        RunByGyro(
            name='advance_90deg_until_color_or_limit',
            target=90.0,
            power=settings.reposition_advance_power,
            pid_p=settings.reposition_advance_pid_p,
            pid_i=settings.reposition_advance_pid_i,
            pid_d=settings.reposition_advance_pid_d,
            target_type=HeadingType.ABSOLUTE,
        ),
        IsColorDetected('detect black after turn90', Color.BLACK),
        stop_after_blue_plus_20mm,
        IsDistanceEarned(
            name='advance_after_turn90_distance_limit',
            delta_dist=settings.reposition_advance_limit_mm,
        ),
    ])

    turn_to_180 = SpinAround(
        name='turn to 180 after advance',
        target=180.0,
        max_power=settings.reposition_turn_max_power,
        min_power=settings.reposition_turn_min_power,
        pid_p=settings.reposition_turn_pid_p,
        pid_i=settings.reposition_turn_pid_i,
        pid_d=settings.reposition_turn_pid_d,
        target_type=HeadingType.ABSOLUTE,
    )

    drive = Parallel(
        name="drive configured distance from rally exit",
        policy=ParallelPolicy.SuccessOnOne(),
    )
    drive.add_children(
        [
            RunAtBearing(
                name="run straight from rally exit",
                context=context,
                bearing=settings.entry_bearing_deg,
                power=settings.navigation_power,
                pid_p=settings.drive_pid_p,
                pid_i=settings.drive_pid_i,
                pid_d=settings.drive_pid_d,
            ),
            IsDistanceEarned(
                name="sumo initial straight distance",
                delta_dist=settings.start_straight_distance_mm,
            ),
        ]
    )

    camera_retreat_command = RunAsInstructed(
        name="reverse after ring turn",
        pwm_l=-settings.camera_retreat_power,
        pwm_r=-settings.camera_retreat_power,
    )
    camera_retreat = Parallel(
        name="reverse to widen sumo camera view",
        policy=ParallelPolicy.SuccessOnOne(),
    )
    camera_retreat.add_children(
        [
            camera_retreat_command,
            IsDistanceEarned(
                name="sumo camera retreat distance",
                delta_dist=settings.camera_retreat_distance_mm,
            ),
        ]
    )

    root = Sequence(name="move_to_sumo_start", memory=True)
    root.add_children(
        [
            # 実行順0：LAPゲート終了直後の実際の停止位置を、以降の固定進入方位の
            # 前提位置へ合わせ直す(後退→+90度旋回→前進→180度旋回)。
            StopNow(name="stop_before_backward"),
            IsTimePassed(name="wait_before_back", delta_time=0.2),
            back_by_gyro,
            StopNow(name="stop_after_backward"),
            turn_plus_90,
            StopNow(name="stop_after_turn90"),
            advance_after_turn90,
            StopNow(name="stop_after_advance_turn90"),
            turn_to_180,
            StopNow(name="stop_after_turn180"),
            # 実行順1：設定した進入方位を維持し、初期位置から合計350mm進む。色判定は行わない。
            drive,
            # 実行順2：距離到達後に制動し、この位置で旋回する。75mmの追加直進は入れない。
            StopNow(name="stop after sumo initial straight"),
            # 実行順3：Leftは方位270度、Rightは方位90度の土俵側を向く。
            SpinToBearing(
                name="turn 90 degrees toward sumo ring",
                context=context,
                bearing=lambda: (settings.ring_bearing_left_deg if runtime.course > 0
                                 else (-settings.ring_bearing_left_deg) % 360.0),
                max_power=settings.turn_max_power,
                min_power=settings.turn_min_power,
                pid_p=settings.turn_pid_p,
                pid_i=settings.turn_pid_i,
                pid_d=settings.turn_pid_d,
                tolerance=settings.heading_tolerance_deg,
            ),
            # 実行順4：旋回完了位置を確定してから、カメラ視野を広げる後退へ移る。
            StopNow(name="stop at sumo search position"),
            # 実行順5：左右コースにかかわらず、両輪が後退するPWMへ設定する。
            ConfigureCameraRetreatPwm(
                name="configure sumo camera retreat pwm",
                motor_command=camera_retreat_command,
                power=settings.camera_retreat_power,
            ),
            # 実行順6：設定距離だけ後退し、土俵全体と黒テープを画角へ入れやすくする。
            camera_retreat,
            # 実行順7：画像取得前に完全停止し、モーションブラーを抑える。
            StopNow(name="stop at sumo camera capture position"),
            # 実行順8：停止時の正面方位を、カメラ捕捉と死角進入後の基準として保存する。
            InitializeSumoState(name="initialize sumo camera capture", context=context),
        ]
    )
    return root
