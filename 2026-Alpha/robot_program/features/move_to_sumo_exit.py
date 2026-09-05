"""Feature 18: carry the captured sumo bottle to the exit-side black line."""

from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time

from ..behaviours.conditions import IsColorDetected, IsDistanceEarned
from ..behaviours.gyro_drive import RunByGyro
from ..behaviours.motor_control import RunAsInstructed, StopNow


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


class ConfigureMirroredCarryCurvePwm(Behaviour):
    # ボトルを保持したまま、その場旋回せず出口方向へ向く緩い円弧を左右コースで鏡像化する。
    def __init__(self, name, motor_command, settings):
        super().__init__(name)
        self.motor_command = motor_command
        self.settings = settings

    def update(self):
        runtime.require("right_motor", "left_motor")
        if runtime.course >= 0:
            # Left：右車輪を速くし、物理的な左カーブで出口側へ向く。
            self.motor_command.pwm_l = self.settings.carry_curve_left_pwm
            self.motor_command.pwm_r = self.settings.carry_curve_right_pwm
        else:
            # Right：course符号を相殺して前進を保ち、物理的な右カーブへ鏡像化する。
            self.motor_command.pwm_l = -self.settings.carry_curve_right_pwm
            self.motor_command.pwm_r = -self.settings.carry_curve_left_pwm
        return Status.SUCCESS


class MarkTransportCompleted(Behaviour):
    # 出口側黒ライン到達をET相撲の運搬完了として記録する。
    def __init__(self, name, context):
        super().__init__(name)
        self.context = context

    def update(self):
        self.context.sumo.transport_completed = True
        return Status.SUCCESS


def build_move_to_sumo_exit(context, config):
    # No.18：ボトルをアーム内に保持し、緩い円弧と直進でET相撲終了位置へ運ぶ。
    settings = config.sumo

    carry_curve_motor_command = RunAsInstructed(
        name="course mirrored carry curve motor command",
        pwm_l=settings.carry_curve_left_pwm,
        pwm_r=settings.carry_curve_right_pwm,
    )
    carry_curve = Parallel(
        name="curve toward sumo exit while holding bottle",
        policy=ParallelPolicy.SuccessOnOne(),
    )
    carry_curve.add_children(
        [
            carry_curve_motor_command,
            IsDistanceEarned(
                name="sumo carry curve distance",
                delta_dist=settings.carry_curve_distance_mm,
            ),
        ]
    )
    find_black_line = Parallel(
        name="drive onto course-side black line",
        policy=ParallelPolicy.SuccessOnOne(),
    )
    find_black_line.add_children(
        [
            RunByGyro(
                name="run toward course-side black line",
                target=0,
                power=settings.carry_power,
                pid_p=settings.drive_pid_p,
                pid_i=settings.drive_pid_i,
                pid_d=settings.drive_pid_d,
                target_type=HeadingType.RELATIVE,
            ),
            IsColorDetected(
                name="detect course-side black line",
                color=Color.BLACK,
            ),
        ]
    )
    transport = Sequence(name="carry sumo bottle to exit", memory=True)
    transport.add_children(
        [
            # 実行順1：保持中のその場旋回を避け、調整可能な緩い円弧で出口方向へ向く。
            ConfigureMirroredCarryCurvePwm(
                "configure course mirrored sumo carry curve pwm",
                carry_curve_motor_command,
                settings,
            ),
            carry_curve,
            StopNow(name="stop after sumo carry curve"),
            # 実行順2：円弧終了時の方位をジャイロで維持し、出口側黒ラインまで運搬する。
            find_black_line,
            StopNow(name="stop at sumo exit with bottle"),
            MarkTransportCompleted("mark sumo bottle transport completed", context),
        ]
    )

    root = Selector(name="move_to_sumo_exit", memory=True)
    root.add_children(
        [
            SkipTransportWhenBottleWasNotCaptured(
                "skip sumo transport when bottle was not captured",
                context,
            ),
            transport,
        ]
    )
    return root
