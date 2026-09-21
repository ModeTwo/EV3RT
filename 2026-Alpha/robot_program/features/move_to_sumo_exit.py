"""Feature 18: release the captured sumo bottle and rejoin the garage-side line."""

from .bt_imports import (
    Behaviour,
    Color,
    Parallel,
    ParallelPolicy,
    Selector,
    Sequence,
    Status,
    TraceSide,
    runtime,
    time,
)

from ..behaviours.conditions import IsDistanceEarned, IsColorDetected
from .sumo_bearing_motion import (
    RunAtBearing,
    EncoderSpinToBearing as SpinToBearing,
    current_bearing,
)
from ..behaviours.line_trace import TraceLine
from ..behaviours.motor_control import RunAsInstructed, StopNow


class PlanGarageReturn(Behaviour):
    """Plan the single fixed escape path used after the sumo bottle is released.

    The old route-1/route-2 selection has been removed.
    Regardless of the black bottle's image position, the robot always turns
    40 degrees from the bottle push heading (mirrored by course) and then
    drives on that bearing until the garage-side black line is detected.
    """

    ESCAPE_TURN_DEG = 40.0

    def __init__(self, context, settings):
        super().__init__(name="plan garage return from push heading")
        self.context = context
        self.settings = settings
        self.push_bearing = None
        self.escape_bearing = None

    def update(self):
        # Capture the actual heading immediately after the 500 mm push.
        pushed = current_bearing(self.context)
        self.push_bearing = pushed

        # Fixed escape path:
        # Left / Right are mirrored by runtime.course.
        self.escape_bearing = (
            pushed - runtime.course * self.ESCAPE_TURN_DEG
        ) % 360.0

        self.logger.info(
            "SUMO escape plan push_bearing=%.1f turn=%.1f escape_bearing=%.1f"
            % (pushed, self.ESCAPE_TURN_DEG, self.escape_bearing)
        )
        return Status.SUCCESS

    def escape_bearing_target(self):
        if self.escape_bearing is None:
            raise RuntimeError("Escape bearing has not been planned")
        return self.escape_bearing


class SkipTransportWhenBottleWasNotCaptured(Behaviour):
    # 未検出などでボトルを捕捉していない場合は、保持運搬動作を省略する。
    def __init__(self, name, context):
        super().__init__(name)
        self.context = context

    def update(self):
        if not self.context.sumo.bottle_captured:
            runtime.require("plotter")
            self.logger.warning(
                "%+06d %s.transport skipped because bottle was not captured"
                % (runtime.plotter.get_distance(), self.__class__.__name__)
            )
            return Status.SUCCESS
        return Status.FAILURE


class AnnounceSumoTransportStage(Behaviour):
    # ボトル捕捉後の処理段階をログとBehavior Tree上で明示する。
    def __init__(self, name, message):
        super().__init__(name)
        self.message = message

    def update(self):
        runtime.require("plotter")
        self.logger.info(
            "%+06d %s.%s"
            % (runtime.plotter.get_distance(), self.__class__.__name__, self.message)
        )
        return Status.SUCCESS


class ConfigureCourseIndependentReversePwm(Behaviour):
    # RunAsInstructedは内部でcourse符号を掛けるため、両コースで物理的に後退する論理PWMを設定する。
    def __init__(self, name, motor_command, power):
        super().__init__(name)
        self.motor_command = motor_command
        self.power = abs(int(power))

    def update(self):
        runtime.require("right_motor", "left_motor")
        logical_reverse_power = -self.power if runtime.course >= 0 else self.power
        self.motor_command.pwm_l = logical_reverse_power
        self.motor_command.pwm_r = logical_reverse_power
        return Status.SUCCESS


class DetectDarkGarageLine(Behaviour):
    # 復帰用黒ラインは彩度に依存させず、カラーセンサーの生V値を連続確認する。
    def __init__(self, name, context, settings):
        super().__init__(name)
        self.context = context
        self.settings = settings
        self.dark_samples = 0
        self.last_log_at = None

    def update(self):
        runtime.require("plotter", "color_sensor", "gyro_sensor")
        now = time.monotonic()
        h, s, v = runtime.color_sensor.get_raw_color_hsv()
        if v <= self.settings.garage_line_black_max_value:
            self.dark_samples += 1
        else:
            self.dark_samples = 0

        if (
            self.last_log_at is None
            or now - self.last_log_at >= self.settings.garage_line_sensor_log_interval_sec
        ):
            self.last_log_at = now
            self.logger.info(
                "%+06d %s.hsv=(%d,%d,%d) dark_samples=%d/%d"
                % (
                    runtime.plotter.get_distance(),
                    self.__class__.__name__,
                    h,
                    s,
                    v,
                    self.dark_samples,
                    self.settings.garage_line_confirm_samples,
                )
            )

        if self.dark_samples >= self.settings.garage_line_confirm_samples:
            self.context.sumo.garage_line_found = True
            absolute_heading = current_bearing(self.context)
            self.logger.info(
                "%+06d %s.garage-side black line detected v=%d bearing=%.1f"
                % (
                    runtime.plotter.get_distance(),
                    self.__class__.__name__,
                    v,
                    absolute_heading,
                )
            )
            return Status.SUCCESS
        return Status.RUNNING


class WasGarageLineFound(Behaviour):
    # 最大探索距離と黒ライン検知のどちらが探索を終了させたかを後段で判別する。
    def __init__(self, name, context):
        super().__init__(name)
        self.context = context

    def update(self):
        if self.context.sumo.garage_line_found:
            return Status.SUCCESS
        return Status.FAILURE


class MarkSumoExitState(Behaviour):
    # 各物理工程の完了状態をContextへ記録し、後続工程とログ解析から参照可能にする。
    def __init__(self, name, context, event):
        super().__init__(name)
        self.context = context
        self.event = event

    def update(self):
        runtime.require("plotter")
        if self.event == "pushed_out":
            self.context.sumo.bottle_pushed_out = True
            message = "bottle pushed by configured distance"
        elif self.event == "released":
            self.context.sumo.bottle_released = True
            self.context.sumo.bottle_held_at_exit = False
            message = "bottle released after straight reverse"
        elif self.event == "line_trace_ready":
            self.context.sumo.transport_completed = True
            self.context.sumo.line_trace_ready = True
            message = "garage-side line trace is ready"
        else:
            raise ValueError("Unknown sumo exit event: " + str(self.event))
        self.logger.info(
            "%+06d %s.%s"
            % (runtime.plotter.get_distance(), self.__class__.__name__, message)
        )
        return Status.SUCCESS


def build_move_to_sumo_exit(context, config):
    # No.18:
    # The previous camera-capture stage has already completed the total 500 mm
    # bottle push.  From here: reverse to release -> fixed 40 degree escape ->
    # detect black line -> align to garage heading -> line trace to blue.
    settings = config.sumo
    return_plan = PlanGarageReturn(context, settings)

    # ======================================================
    # Final line trace: continue until the blue line is detected.
    # ======================================================
    blue_trace = Parallel(
        name="blue_trace",
        policy=ParallelPolicy.SuccessOnOne(),
    )
    blue_trace.add_children([
        TraceLine(
            name="sensor trace normal edge",
            target=65,
            power=70,
            power_min=33,
            pid_p=0.65,
            pid_i=0.000001,
            pid_d=0.045,
            err_lo=6,
            err_hi=16,
            decel_per_s=350,
            gains_slow=(0.65, 0.045),
            gains_fast=(0.55, 0.065),
            recover_v=97,
            recover_after=3,
            recover_turn=35,
            trace_side=TraceSide.NORMAL,
        ),
        IsColorDetected(name="check color", color=Color.BLUE),
    ])

    # ======================================================
    # Release the bottle by reversing straight.
    # ======================================================
    release_reverse_motor = RunAsInstructed(
        name="straight reverse release motor command",
        pwm_l=-settings.release_reverse_power,
        pwm_r=-settings.release_reverse_power,
    )

    release_reverse = Parallel(
        name="reverse straight to release sumo bottle",
        policy=ParallelPolicy.SuccessOnOne(),
    )
    release_reverse.add_children([
        release_reverse_motor,
        IsDistanceEarned(
            name="sumo bottle release reverse distance",
            delta_dist=settings.release_reverse_distance_mm,
        ),
    ])

    # ======================================================
    # Fixed escape path.
    #
    # Push heading -> turn 40 degrees -> keep that bearing and drive
    # forward until the garage-side black line is detected.
    # There is no route-1/route-2 selection anymore.
    # ======================================================
    escape_line_detector = DetectDarkGarageLine(
        name="detect black line during sumo escape",
        context=context,
        settings=settings,
    )

    escape_drive = Parallel(
        name="drive sumo escape until black line",
        policy=ParallelPolicy.SuccessOnOne(),
    )
    escape_drive.add_children([
        RunAtBearing(
            name="hold sumo escape bearing",
            context=context,
            bearing=return_plan.escape_bearing_target,
            power=settings.escape_power,
            pid_p=settings.drive_pid_p,
            pid_i=settings.drive_pid_i,
            pid_d=settings.drive_pid_d,
        ),
        escape_line_detector,
    ])

    escape = Sequence(
        name="sumo bottle escape",
        memory=True,
    )
    escape.add_children([
        SpinToBearing(
            name="turn 40deg for sumo escape",
            context=context,
            bearing=return_plan.escape_bearing_target,
            max_power=settings.turn_max_power,
            min_power=settings.turn_min_power,
            pid_p=settings.turn_pid_p,
            pid_i=settings.turn_pid_i,
            pid_d=settings.turn_pid_d,
            tolerance=settings.heading_tolerance_deg,
        ),
        StopNow(name="stop after sumo escape 40deg turn"),
        escape_drive,
        StopNow(name="stop after sumo escape black line detection"),
    ])

    # ======================================================
    # After black-line detection, move onto the line so that the
    # following heading alignment has enough room.
    # ======================================================
    enter_garage_line = Parallel(
        name="advance onto garage line before alignment",
        policy=ParallelPolicy.SuccessOnOne(),
    )
    enter_garage_line.add_children([
        RunAtBearing(
            name="run straight onto garage line",
            context=context,
            bearing=lambda: current_bearing(context),
            power=settings.garage_return_drive_power,
            pid_p=settings.drive_pid_p,
            pid_i=settings.drive_pid_i,
            pid_d=settings.drive_pid_d,
        ),
        IsDistanceEarned(
            name="garage line entry distance",
            delta_dist=settings.garage_line_entry_distance_mm,
        ),
    ])

    # Short line trace after alignment to stabilize the robot on the line.
    stabilize_line_trace = Parallel(
        name="stabilize on garage-side black line",
        policy=ParallelPolicy.SuccessOnOne(),
    )
    stabilize_line_trace.add_children([
        TraceLine(
            name="trace garage-side black line",
            target=settings.line_rejoin_trace_target_v,
            power=settings.line_rejoin_trace_power,
            pid_p=0.55,
            pid_i=0.0000009,
            pid_d=0.015,
            trace_side=TraceSide.NORMAL,
        ),
        IsDistanceEarned(
            name="garage-side line stabilization distance",
            delta_dist=settings.line_rejoin_trace_distance_mm,
        ),
    ])

    # ======================================================
    # Complete the garage-side line rejoin.
    # The escape drive has already detected the black line.
    # ======================================================
    complete_line_rejoin = Sequence(
        name="complete garage-side line rejoin",
        memory=True,
    )
    complete_line_rejoin.add_children([
        WasGarageLineFound(
            name="confirm garage-side black line was found",
            context=context,
        ),
        enter_garage_line,
        StopNow(name="stop after garage line entry"),
        SpinToBearing(
            name="align to garage absolute heading",
            context=context,
            bearing=settings.garage_bearing_deg,
            max_power=settings.turn_max_power,
            min_power=settings.turn_min_power,
            pid_p=settings.turn_pid_p,
            pid_i=settings.turn_pid_i,
            pid_d=settings.turn_pid_d,
            tolerance=settings.heading_tolerance_deg,
        ),
        StopNow(name="stop after absolute garage alignment"),
        stabilize_line_trace,
        StopNow(name="stop after garage-side line stabilization"),
        MarkSumoExitState(
            name="mark sumo line trace ready",
            context=context,
            event="line_trace_ready",
        ),
    ])

    # ======================================================
    # Main Feature 18 sequence.
    # ======================================================
    transport = Sequence(
        name="carry sumo bottle to exit",
        memory=True,
    )
    transport.add_children([
        # Save the push heading and calculate the fixed 40-degree escape bearing.
        return_plan,

        AnnounceSumoTransportStage(
            name="begin straight reverse release",
            message="reversing straight to release bottle from arm",
        ),
        ConfigureCourseIndependentReversePwm(
            name="configure straight reverse release pwm",
            motor_command=release_reverse_motor,
            power=settings.release_reverse_power,
        ),
        release_reverse,
        StopNow(name="stop after sumo bottle release reverse"),
        MarkSumoExitState(
            name="mark sumo bottle released",
            context=context,
            event="released",
        ),

        # Fixed 40-degree escape; no bottle-position route selection.
        escape,

        # Existing garage-line rejoin flow.
        complete_line_rejoin,

        # Continue tracing until blue.
        blue_trace,
    ])

    root = Selector(name="move_to_sumo_exit", memory=True)
    root.add_children([
        SkipTransportWhenBottleWasNotCaptured(
            name="skip sumo transport when bottle was not captured",
            context=context,
        ),
        transport,
    ])
    return root
