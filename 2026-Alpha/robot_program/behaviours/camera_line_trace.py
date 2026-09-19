"""カメラでコースラインへ寄せ、カラーセンサーへ追従を引き渡す。"""

import time

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from simple_pid import PID
from py_etrobo_util import TargetInterested, TraceSide

from ..runtime import runtime
from ..timing import CONTROL_INTERVAL_SEC

ROE_DEGENERATE = 90
MIN_CURVE_ROWS = 15
PHASE_SEEK = 'SEEK'
PHASE_ALIGN = 'ALIGN'
PHASE_HANDOFF = 'HANDOFF'


class RecoverLineByCamera(Behaviour):
    """固定旋回を使わず、カメラで見えたラインへ前進しながら寄せる。"""

    def __init__(
        self,
        name: str,
        power: int,
        pid_p: float,
        pid_i: float,
        pid_d: float,
        max_camera_turn: int,
        align_power: int,
        handoff_power: int,
        handoff_target_v: int,
        handoff_pid_p: float,
        handoff_turn_cap: float,
        handoff_v_tolerance: int,
        handoff_stable_samples: int,
        line_v: int,
        line_samples: int,
        trace_side: TraceSide,
        gs_min: int = 0,
        gs_max: int = 55,
        tilt_ff_gain: float = 8.0,
        ff_cap: float = 8.0,
        heading_tolerance_deg: float = 5.0,
        stable_samples: int = 10,
        gyro_heading_deg: float = 0.0,
        gyro_kp: float = 0.3,
        gyro_turn_cap: float = 8.0,
        log_interval_sec: float = 0.25,
        seek_theta_tolerance_deg: float = 5.0,
        seek_stable_samples: int = 0,
        seek_distance_limit_mm=None,
    ) -> None:
        super().__init__(name)
        self.power = power
        self.max_camera_turn = max_camera_turn
        self.align_power = align_power
        self.handoff_power = handoff_power
        self.handoff_target_v = handoff_target_v
        self.handoff_turn_cap = handoff_turn_cap
        self.handoff_v_tolerance = handoff_v_tolerance
        self.handoff_stable_samples = handoff_stable_samples
        self.line_v = line_v
        self.line_samples = line_samples
        self.trace_side = trace_side
        self.gs_min = gs_min
        self.gs_max = gs_max
        self.tilt_ff_gain = tilt_ff_gain
        self.ff_cap = ff_cap
        self.heading_tolerance_deg = heading_tolerance_deg
        self.stable_samples = stable_samples
        self.gyro_heading_deg = gyro_heading_deg
        self.gyro_kp = gyro_kp
        self.gyro_turn_cap = gyro_turn_cap
        self.log_interval_sec = log_interval_sec
        # SEEKがカラーセンサーの黒検知(dark_count)だけに依存すると、カメラが
        # 中心だと見ている対象と実際の色センサーの位置がずれている場合に
        # 永久にSEEKへ留まってしまう(実機で複数回確認済み)。theta(カメラの
        # 中心誤差)が一定時間安定した場合、あるいは距離上限に達した場合も
        # ALIGNへ進めるフォールバックを追加する。seek_stable_samples<=0または
        # seek_distance_limit_mm=Noneでそれぞれ無効化され、既存呼び出し元
        # (LAP前のcamera_recovery)の挙動は変えない。
        self.seek_theta_tolerance_deg = seek_theta_tolerance_deg
        self.seek_stable_samples = seek_stable_samples
        self.seek_distance_limit_mm = seek_distance_limit_mm
        self.seek_stable_count = 0
        self.seek_start_dist = 0
        self.pid = PID(
            pid_p,
            pid_i,
            pid_d,
            setpoint=0,
            sample_time=CONTROL_INTERVAL_SEC,
            output_limits=(-max_camera_turn, max_camera_turn),
        )
        # 通常TraceLineへ渡す前だけ使う、穏やかな明度P制御。
        self.handoff_pid = PID(
            handoff_pid_p,
            0.0,
            0.0,
            setpoint=handoff_target_v,
            sample_time=CONTROL_INTERVAL_SEC,
            output_limits=(-handoff_turn_cap, handoff_turn_cap),
        )
        self.running = False
        self.dark_count = 0
        self.phase = PHASE_SEEK
        self.stable_count = 0
        self.last_log_at = None

    def update(self) -> Status:
        runtime.require(
            'video', 'plotter', 'color_sensor', 'gyro_sensor',
            'right_motor', 'left_motor'
        )
        if not self.running:
            runtime.video.set_thresholds(self.gs_min, self.gs_max)
            runtime.video.set_target_interested(TargetInterested.LINE)
            if self.trace_side == TraceSide.NORMAL:
                camera_side = TraceSide.RIGHT if runtime.course == -1 else TraceSide.LEFT
            elif self.trace_side == TraceSide.OPPOSITE:
                camera_side = TraceSide.LEFT if runtime.course == -1 else TraceSide.RIGHT
            else:
                camera_side = TraceSide.CENTER
            runtime.video.set_trace_side(camera_side)
            self.running = True
            self.seek_start_dist = runtime.plotter.get_distance()
            self.seek_stable_count = 0
            self.logger.info(
                '%+06d %s.camera recovery started side=%s'
                % (runtime.plotter.get_distance(), self.__class__.__name__, camera_side.name)
            )

        theta, frame_id, captured_at, _ = runtime.video.get_theta_stamped()
        insight = runtime.video.is_target_insight()
        tilt = runtime.video.get_line_tilt()
        edge_range = runtime.video.get_range_of_edges()
        tilt_ff = 0.0
        if (
            edge_range != 0
            and edge_range <= ROE_DEGENERATE
            and runtime.video.get_band_sep() >= MIN_CURVE_ROWS
        ):
            tilt_ff = max(
                -self.ff_cap,
                min(self.ff_cap, self.tilt_ff_gain * tilt),
            )
        # SEEKではカメラで短くラインへ入る。黒を捕捉したALIGN以降は
        # カメラ操舵を止め、反対向きのジャイロ補正だけで絶対0度へ戻す。
        camera_turn = (
            float(self.pid(theta)) + tilt_ff
            if insight and self.phase == PHASE_SEEK
            else 0.0
        )

        _, _, value = runtime.color_sensor.get_raw_color_hsv()
        self.dark_count = self.dark_count + 1 if value <= self.line_v else 0

        if self.phase == PHASE_SEEK:
            theta_stable = bool(insight) and abs(theta) <= self.seek_theta_tolerance_deg
            self.seek_stable_count = self.seek_stable_count + 1 if theta_stable else 0
            seek_dist = abs(runtime.plotter.get_distance() - self.seek_start_dist)
            advance_reason = None
            if self.dark_count >= self.line_samples:
                advance_reason = 'line acquired'
            elif self.seek_stable_samples > 0 and self.seek_stable_count >= self.seek_stable_samples:
                # 色センサーが黒を検知しなくても、カメラのtheta(中心誤差)が
                # 一定サンプル安定していれば十分近いと判断してALIGNへ進める。
                advance_reason = 'heading stable fallback'
            elif self.seek_distance_limit_mm is not None and seek_dist >= self.seek_distance_limit_mm:
                # dark_countもtheta安定も得られないまま一定距離を使い切った場合、
                # SEEKに留まり続けて1200mm予算を消費し尽くすよりは、ALIGN以降へ
                # 進めて色センサーへの引渡しを試みる(実機で複数回確認した滞留対策)。
                advance_reason = 'distance limit fallback'
            if advance_reason is not None:
                self.phase = PHASE_ALIGN
                self.stable_count = 0
                self.logger.info(
                    '%+06d %s.%s; switching SEEK to ALIGN heading=%.1f'
                    % (
                        runtime.plotter.get_distance(), self.__class__.__name__, advance_reason,
                        -runtime.course * float(runtime.gyro_sensor.get_angle()),
                    )
                )

        heading = -runtime.course * float(runtime.gyro_sensor.get_angle())
        heading_error = (self.gyro_heading_deg - heading + 180.0) % 360.0 - 180.0
        gyro_turn = 0.0
        if self.phase != PHASE_SEEK:
            gyro_turn = runtime.course * self.gyro_kp * heading_error
            gyro_turn = max(-self.gyro_turn_cap, min(self.gyro_turn_cap, gyro_turn))

        # ALIGNでは方位だけを0度へ戻す。安定したら、低速HANDOFFで
        # カラーセンサーを通常TraceLineの目標明度付近へ移す。
        if self.phase == PHASE_ALIGN:
            heading_stable = abs(heading_error) <= self.heading_tolerance_deg
            self.stable_count = self.stable_count + 1 if heading_stable else 0
            if self.stable_count >= self.stable_samples:
                self.phase = PHASE_HANDOFF
                self.stable_count = 0
                self.handoff_pid.reset()
                self.logger.info(
                    '%+06d %s.heading aligned; switching ALIGN to HANDOFF '
                    'v=%d heading=%.1f'
                    % (
                        runtime.plotter.get_distance(), self.__class__.__name__,
                        value, heading,
                    )
                )

        color_turn = 0.0
        if self.phase == PHASE_HANDOFF:
            # TraceLineのNORMAL側と同じ操舵を、このクラスの左右出力式へ変換する。
            color_turn = runtime.course * float(self.handoff_pid(value))

        if self.phase == PHASE_SEEK:
            base_power = self.power
        elif self.phase == PHASE_ALIGN:
            base_power = self.align_power
        else:
            base_power = self.handoff_power
        # PID、傾きFF、ジャイロを合算した最終値にも上限を適用する。
        if self.phase == PHASE_SEEK:
            turn_cap = self.max_camera_turn
            requested_turn = camera_turn
        elif self.phase == PHASE_ALIGN:
            turn_cap = self.gyro_turn_cap
            requested_turn = gyro_turn
        else:
            turn_cap = self.handoff_turn_cap
            requested_turn = color_turn + gyro_turn
        turn = int(max(-turn_cap, min(turn_cap, requested_turn)))
        right_power = max(-100, min(100, base_power + turn))
        left_power = max(-100, min(100, base_power - turn))
        runtime.right_motor.set_brake(False)
        runtime.left_motor.set_brake(False)
        runtime.right_motor.set_power(right_power)
        runtime.left_motor.set_power(left_power)

        if self.phase == PHASE_HANDOFF:
            handoff_stable = (
                abs(value - self.handoff_target_v) <= self.handoff_v_tolerance
                and abs(heading_error) <= self.heading_tolerance_deg
            )
            self.stable_count = self.stable_count + 1 if handoff_stable else 0
        now = time.monotonic()
        if self.last_log_at is None or now - self.last_log_at >= self.log_interval_sec:
            age_ms = max(0.0, (time.time() - captured_at) * 1000.0)
            self.logger.info(
                '%+06d camera line phase=%s fid=%d theta=%.1f insight=%d v=%d dark=%d '
                'tilt=%.2f ff=%.1f color=%.1f gyro=%.1f heading=%.1f stable=%d '
                'turn=%d left=%d right=%d age_ms=%.1f'
                % (
                    runtime.plotter.get_distance(),
                    self.phase,
                    frame_id, theta, int(insight),
                    value, self.dark_count, tilt, tilt_ff, color_turn, gyro_turn, heading,
                    self.stable_count, turn,
                    left_power, right_power, age_ms,
                )
            )
            self.last_log_at = now

        if self.phase == PHASE_HANDOFF and self.stable_count >= self.handoff_stable_samples:
            self.logger.info(
                '%+06d %s.line handed to color sensor v=%d theta=%.1f heading=%.1f'
                % (
                    runtime.plotter.get_distance(), self.__class__.__name__,
                    value, theta, heading,
                )
            )
            return Status.SUCCESS
        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        for motor in (runtime.left_motor, runtime.right_motor):
            if motor is not None:
                motor.set_power(0)
        self.running = False
        self.dark_count = 0
        self.phase = PHASE_SEEK
        self.stable_count = 0
        self.seek_stable_count = 0
        self.last_log_at = None
