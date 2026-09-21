"""スタート～LAPで「何を走らせるか」を組み立てる。

角度表: start_lap_profile_v1.py / 補間: heading_profile.py
速度・PID: config.py / モーター制御: behaviours/gyro_drive.py
"""

from py_trees.common import ParallelPolicy
from py_trees.composites import Parallel, Sequence
from py_trees.decorators import Timeout

from ..behaviours.conditions import IsColorDetected
from ..behaviours.camera_line_trace import RecoverLineByCamera
from ..behaviours.gyro_drive import RunByGyro, SpinAround
from ..behaviours.line_trace import TraceLine
from ..behaviours.section_motion import DriveDistance
from ..start_lap_calibration import calibrated_profile
from ..start_lap_profile_v1 import POINTS, BLUE_START_MM, LAP_GATE_MM
from ..types import HeadingType
from .bt_imports import Color, TraceSide
from ..behaviours.motor_control import StopNow
from ..behaviours.conditions import IsTimePassed
from ..behaviours.conditions import IsDistanceEarned
from py_trees.behaviour import Behaviour
import time
import math
from typing import Callable, Union
from py_trees.common import Status
from simple_pid import PID
from ..path_tracking import PathTracking, curvature_turn


from ..timing import CONTROL_INTERVAL_SEC as EXEC_INTERVAL

from ..runtime import runtime

# 終了位置の調整値（mm）。角度追従の調整でも終了位置は変更しない。
LAP_PASS_MARGIN_MM = 0.0       # LAP単体: ゲートの20mm先で停止

# 【案】後退→+90°旋回→前進→180°旋回の試作区間。実機未校正の試走初期値。
BACKWARD_DISTANCE_MM = 330.0            # 後退距離（50cm）
TURN_MAX_POWER = 60
TURN_MIN_POWER = 60
TURN_PID_P = 0.2
TURN_PID_I = 0.005
TURN_PID_D = 0.03
ADVANCE_AFTER_TURN90_POWER = 80
ADVANCE_AFTER_TURN90_PID_P = 1.1
ADVANCE_AFTER_TURN90_PID_I = 0.00075
ADVANCE_AFTER_TURN90_PID_D = 0.04
ADVANCE_AFTER_TURN90_LIMIT_MM = 550.0    # 前進距離上限（55cm）
ADVANCE_AFTER_TURN90_BLUE_EXTRA_MM = 20.0  # 青検知後にさらに進む距離（2cm）。黒検知は即停止。

def _normalize_heading_error(error: float) -> float:
    # 角度差を-180度以上180度未満へ正規化する。
    return (error + 180.0) % 360.0 - 180.0
#カーブで速度を落とすために、追加
def speed_profile(distance_mm: float, config) -> int:
     # カーブ区間のリスト（mm）
    curve_sections = [
        (418, 841),
        (1184, 1591),
        (1749, 2262),
        (3740, 4201),
    ]

    # どれかのカーブ区間に入っていたら速度を落とす
    for start, end in curve_sections:
        if start <= distance_mm <= end:
            return 70   # カーブの速度（共通）

    return config.start_lap_power            # 直線の速度 75?80?（config.start_lap_power と同じ）


#カーブでスピードを落とすためにRunByGyroを新しく定義
class RunByGyroSpeedDownatCurve(RunByGyro):
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
        config=None,   # ★ 追加
    ):
        # ★ super() には RunByGyro と同じ引数だけ渡す
        super().__init__(
            name,
            target,
            power,
            pid_p,
            pid_i,
            pid_d,
            target_type,
            distance_limit_mm=distance_limit_mm,
            completion_condition=completion_condition,
            completion_min_mm=completion_min_mm,
            feedforward_gain=feedforward_gain,
            wheel_tread_mm=wheel_tread_mm,
            cross_track_lookahead_mm=cross_track_lookahead_mm,
            max_heading_correction_deg=max_heading_correction_deg,
            profile_log_interval_sec=profile_log_interval_sec,
        )

        # ★ config を自分で保持する
        self.config = config

    def update(self) -> Status:
        if self.distance_target:
            return self._update_distance_target_speeddown()
        else:
            return super().update()

    def _update_distance_target_speeddown(self) -> Status:
        status = super()._update_distance_target()

        progress = runtime.plotter.get_distance() - self.origin_distance

        # ★ None ではなく self.config が入るようになる
        self.power = speed_profile(progress, self.config)

        return status


#マイナスジャイロでバックするクラスを追加
#output_limitsを絶対値つける
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



def build_start_to_lap_gate(context, config):
    # 旧方式は別ファイルへ保存。通常読む必要はない。
    if config.start_lap_mode == 'legacy':
        from .start_to_lap_gate_legacy import build_legacy_start_to_lap_gate
        return build_legacy_start_to_lap_gate(context, config)
    if config.start_lap_mode != 'profile':
        raise ValueError('start_lap_mode must be profile or legacy')

    # 1. 距離を渡すと目標角を返す関数を用意する。
    profile, blue_start_mm, lap_gate_mm = calibrated_profile(
        POINTS, BLUE_START_MM, LAP_GATE_MM,
        first_straight_mm=config.start_lap_first_straight_mm,
        route_scale=config.start_lap_route_scale,
    )

    # 2. 後続工程があれば、青検知開始位置で方位角走行からライン追従へ渡す。
    follows_bottle = config.enable_bottle_delivery or config.mission_mode in ('hint2', 'hint2-return')
    if follows_bottle:
        line_trace_start_mm = max(
            0.0, blue_start_mm - config.start_lap_camera_before_blue_mm
        )

        run_to_line_trace = RunByGyroSpeedDownatCurve(
            name='start_to_lap_gate gyro section',
            target=profile.heading_at,
            power=config.start_lap_power,
            pid_p=config.start_lap_pid_p,
            pid_i=config.start_lap_pid_i,
            pid_d=config.start_lap_pid_d,
            target_type=HeadingType.RELATIVE,
            distance_limit_mm= 5260.540, #line_trace_start_mm,
            feedforward_gain=config.start_lap_feedforward_gain,
            wheel_tread_mm=config.start_lap_wheel_tread_mm,
            cross_track_lookahead_mm=config.start_lap_cross_track_lookahead_mm,
            max_heading_correction_deg=config.start_lap_max_heading_correction_deg,
            profile_log_interval_sec=config.start_lap_log_interval_sec,

            config=config
        )

        camera_recovery = RecoverLineByCamera(
            name='recover line by camera before lap',
            power=config.start_lap_camera_power,
            pid_p=config.start_lap_camera_pid_p,
            pid_i=config.start_lap_camera_pid_i,
            pid_d=config.start_lap_camera_pid_d,
            max_camera_turn=config.start_lap_camera_max_turn,
            align_power=config.start_lap_camera_align_power,
            handoff_power=config.start_lap_camera_handoff_power,
            handoff_target_v=config.start_lap_line_target_v,
            handoff_pid_p=config.start_lap_camera_handoff_pid_p,
            handoff_turn_cap=config.start_lap_camera_handoff_turn_cap,
            handoff_v_tolerance=config.start_lap_camera_handoff_v_tolerance,
            handoff_stable_samples=config.start_lap_camera_handoff_stable_samples,
            line_v=config.start_lap_camera_rejoin_v,
            line_samples=config.start_lap_camera_rejoin_samples,
            trace_side=TraceSide.NORMAL,
            tilt_ff_gain=config.start_lap_camera_tilt_ff_gain,
            ff_cap=config.start_lap_camera_ff_cap,
            heading_tolerance_deg=config.start_lap_heading_tolerance_deg,
            stable_samples=config.start_lap_camera_stable_samples,
            gyro_heading_deg=0.0,
            gyro_kp=config.start_lap_camera_gyro_kp,
            gyro_turn_cap=config.start_lap_camera_gyro_turn_cap,
        )

        drive_after_gyro = Sequence(
            name='camera recovery then color line trace', memory=True
        )
        drive_after_gyro.add_children([
            camera_recovery,
            TraceLine(
                name='recover and trace line before lap',
                target=config.start_lap_line_target_v,
                power=config.start_lap_line_power,
                pid_p=config.start_lap_line_pid_p,
                pid_i=config.start_lap_line_pid_i,
                pid_d=config.start_lap_line_pid_d,
                trace_side=TraceSide.NORMAL,
            ),
        ])

        # 青はカメラ復帰中から監視し、検知した周期でATへ渡す。
        # 青から293mmの0度走行を始めるため、ここでは方位安定を待たない。
        recover_and_watch_blue = Parallel(
            name='recover line and watch lap blue marker',
            policy=ParallelPolicy.SuccessOnOne(),
        )
        recover_and_watch_blue.add_children([
            drive_after_gyro,
            IsColorDetected('lap blue marker', Color.BLUE),
        ])

        #バックでジャイロ処理を追加
        back_by_gyro = Parallel(
            name='back by gyro',
            policy=ParallelPolicy.SuccessOnOne(),
        )
        back_by_gyro.add_children([
             RunByGyroMinusBack(
                name="backward_50cm_abs0",
                target=0,                     # 絶対角度0°
                power=-80,                    # ★負のパワーで後退
                pid_p=0.8,                    # 後退用PID（前進より少し強めが安定）
                pid_i=0.0,
                pid_d=0.02,
                target_type=HeadingType.ABSOLUTE
            ),
            IsDistanceEarned(
                name="backward_50cm_done",
                delta_dist=BACKWARD_DISTANCE_MM   # 50cm後退
                ),
        ])

        # 【案】後退後に+90°へ旋回する。いつでも戻せるよう、旧来のsafe_blue_search
        # (camera_recovery経由でのカメラ復帰+青検知)は下で構築を残し、rootには繋がない。
        turn_plus_90 = SpinAround(
            name='turn to plus 90 after backward',
            target=90.0,
            max_power=TURN_MAX_POWER,
            min_power=TURN_MIN_POWER,
            pid_p=TURN_PID_P,
            pid_i=TURN_PID_I,
            pid_d=TURN_PID_D,
            target_type=HeadingType.ABSOLUTE,
        )

        # +90°を保持したまま、黒検知で即停止、青検知はさらに2cm進んでから停止、
        # またはどちらも検知しないまま55cm進んだら停止する。
        stop_after_blue_plus_20mm = Sequence(
            name='blue then 20mm more', memory=True
        )
        stop_after_blue_plus_20mm.add_children([
            IsColorDetected('detect blue after turn90', Color.BLUE),
            IsDistanceEarned(
                name='advance_after_turn90_blue_extra_distance',
                delta_dist=ADVANCE_AFTER_TURN90_BLUE_EXTRA_MM,
            ),
        ])

        advance_after_turn90 = Parallel(
            name='advance at plus 90 until black, blue plus 20mm, or 55cm',
            policy=ParallelPolicy.SuccessOnOne(),
        )
        advance_after_turn90.add_children([
            RunByGyro(
                name='advance_90deg_until_color_or_550mm',
                target=90.0,
                power=ADVANCE_AFTER_TURN90_POWER,
                pid_p=ADVANCE_AFTER_TURN90_PID_P,
                pid_i=ADVANCE_AFTER_TURN90_PID_I,
                pid_d=ADVANCE_AFTER_TURN90_PID_D,
                target_type=HeadingType.ABSOLUTE,
            ),
            IsColorDetected('detect black after turn90', Color.BLACK),
            stop_after_blue_plus_20mm,
            IsDistanceEarned(
                name='advance_after_turn90_distance_limit',
                delta_dist=ADVANCE_AFTER_TURN90_LIMIT_MM,
            ),
        ])

        turn_to_180 = SpinAround(
            name='turn to 180 after advance',
            target=180.0,
            max_power=TURN_MAX_POWER,
            min_power=TURN_MIN_POWER,
            pid_p=TURN_PID_P,
            pid_i=TURN_PID_I,
            pid_d=TURN_PID_D,
            target_type=HeadingType.ABSOLUTE,
        )

        # 旧来の経路（カメラ復帰で黒線に乗り、青検知まで見張る）。
        # 現在は下のroot.add_childrenから外しているだけなので、戻すときは
        # safe_blue_searchをroot.add_childrenの末尾へ差し戻せばよい。
        safe_blue_search = Timeout(
            name='lap blue marker emergency timeout',
            child=recover_and_watch_blue,
            duration=config.start_lap_blue_timeout_sec,
        )

        root = Sequence(name='start_to_lap_gate', memory=True)
        root.add_children([
            run_to_line_trace,
            StopNow(
                name="stop_before_backward"
            ),
            IsTimePassed(
                name="wait_before_back",
                delta_time=0.2
            ),
            back_by_gyro,
            StopNow(
                name="stop_after_backward"
            ),
            # IsTimePassed(
            #     name="wait_before_turn90",
            #     delta_time=0.2
            # ),
            turn_plus_90,
            StopNow(
                name="stop_after_turn90"
            ),
            advance_after_turn90,
            StopNow(
                name="stop_after_advance_turn90"
            ),
            turn_to_180,
            StopNow(
                name="stop_after_turn180"
            ),
        ])

        return root
    else:
        distance_limit_mm = 5160.540 #lap_gate_mm + LAP_PASS_MARGIN_MM

    # 3. devRE完成版と同じく、LAPまで一つのRunByGyroで走る。
    #    heading_atに括弧を付けず、距離から目標方位を得る関数として渡す。
    return RunByGyro(
        name='start_to_lap_gate',
        target=profile.heading_at,
        power=config.start_lap_power,
        pid_p=config.start_lap_pid_p,
        pid_i=config.start_lap_pid_i,
        pid_d=config.start_lap_pid_d,
        target_type=HeadingType.RELATIVE,
        distance_limit_mm=distance_limit_mm,
        feedforward_gain=config.start_lap_feedforward_gain,
        wheel_tread_mm=config.start_lap_wheel_tread_mm,
        cross_track_lookahead_mm=config.start_lap_cross_track_lookahead_mm,
        max_heading_correction_deg=config.start_lap_max_heading_correction_deg,
        profile_log_interval_sec=config.start_lap_log_interval_sec,
    )