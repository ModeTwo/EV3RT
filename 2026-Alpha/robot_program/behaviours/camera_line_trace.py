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
    ) -> None:
        super().__init__(name)
        self.power = power
        self.max_camera_turn = max_camera_turn
        self.align_power = align_power
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
        self.pid = PID(
            pid_p,
            pid_i,
            pid_d,
            setpoint=0,
            sample_time=CONTROL_INTERVAL_SEC,
            output_limits=(-max_camera_turn, max_camera_turn),
        )
        self.running = False
        self.dark_count = 0
        self.line_acquired = False
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
            if insight and not self.line_acquired
            else 0.0
        )

        _, _, value = runtime.color_sensor.get_raw_color_hsv()
        self.dark_count = self.dark_count + 1 if value <= self.line_v else 0
        if not self.line_acquired and self.dark_count >= self.line_samples:
            self.line_acquired = True
            self.stable_count = 0
            self.logger.info(
                '%+06d %s.line acquired; switching SEEK to ALIGN heading=%.1f'
                % (
                    runtime.plotter.get_distance(), self.__class__.__name__,
                    -runtime.course * float(runtime.gyro_sensor.get_angle()),
                )
            )

        heading = -runtime.course * float(runtime.gyro_sensor.get_angle())
        heading_error = (self.gyro_heading_deg - heading + 180.0) % 360.0 - 180.0
        gyro_turn = 0.0
        if self.line_acquired:
            gyro_turn = runtime.course * self.gyro_kp * heading_error
            gyro_turn = max(-self.gyro_turn_cap, min(self.gyro_turn_cap, gyro_turn))
        base_power = self.align_power if self.line_acquired else self.power
        # PID、傾きFF、ジャイロを合算した最終値にも上限を適用する。
        turn_cap = self.gyro_turn_cap if self.line_acquired else self.max_camera_turn
        turn = int(max(-turn_cap, min(turn_cap, camera_turn + gyro_turn)))
        right_power = max(-100, min(100, base_power + turn))
        left_power = max(-100, min(100, base_power - turn))
        runtime.right_motor.set_brake(False)
        runtime.left_motor.set_brake(False)
        runtime.right_motor.set_power(right_power)
        runtime.left_motor.set_power(left_power)

        stable = (
            self.line_acquired
            and abs(heading_error) <= self.heading_tolerance_deg
        )
        self.stable_count = self.stable_count + 1 if stable else 0
        now = time.monotonic()
        if self.last_log_at is None or now - self.last_log_at >= self.log_interval_sec:
            age_ms = max(0.0, (time.time() - captured_at) * 1000.0)
            self.logger.info(
                '%+06d camera line phase=%s fid=%d theta=%.1f insight=%d v=%d dark=%d '
                'tilt=%.2f ff=%.1f gyro=%.1f heading=%.1f stable=%d '
                'turn=%d left=%d right=%d age_ms=%.1f'
                % (
                    runtime.plotter.get_distance(),
                    'ALIGN' if self.line_acquired else 'SEEK',
                    frame_id, theta, int(insight),
                    value, self.dark_count, tilt, tilt_ff, gyro_turn, heading,
                    self.stable_count, turn,
                    left_power, right_power, age_ms,
                )
            )
            self.last_log_at = now

        if self.stable_count >= self.stable_samples:
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
        self.line_acquired = False
        self.stable_count = 0
        self.last_log_at = None
