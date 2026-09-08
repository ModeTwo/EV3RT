"""Feature 18: push out the captured bottle and rejoin the garage-side line."""

from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time

from ..behaviours.conditions import IsColorDetected, IsDistanceEarned
from ..behaviours.gyro_drive import RunByGyro
from ..behaviours.line_trace import TraceLine
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


class ConfigureMirroredGarageReturnPwm(Behaviour):
    # 離脱後の後退カーブを左右コースで鏡像化し、各コースのガレージ側へ寄せる。
    def __init__(self, name, motor_command, settings):
        super().__init__(name)
        self.motor_command = motor_command
        self.settings = settings

    def update(self):
        runtime.require("right_motor", "left_motor")
        left_power = abs(int(self.settings.garage_return_reverse_left_pwm))
        right_power = abs(int(self.settings.garage_return_reverse_right_pwm))
        if runtime.course >= 0:
            # Leftでは左輪を遅くして、後退しながらガレージ側へ緩く寄せる。
            self.motor_command.pwm_l = -left_power
            self.motor_command.pwm_r = -right_power
        else:
            # Rightでは左右差を反転し、course符号を相殺する正の論理PWMを渡す。
            self.motor_command.pwm_l = right_power
            self.motor_command.pwm_r = left_power
        return Status.SUCCESS


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
            message = "bottle pushed beyond black line"
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
    # No.18：黒ライン外へ押し出し、直線後退で離脱し、ガレージ側の黒ラインへ復帰する。
    settings = config.sumo

    # 捕捉時の方位を維持したまま、最初に見つかる黒ラインまでボトルを押して進む。
    find_push_out_line = Parallel(
        name="drive captured bottle to push-out black line",
        policy=ParallelPolicy.SuccessOnOne(),
    )
    find_push_out_line.add_children(
        [
            RunByGyro(
                name="run straight to push-out black line",
                target=0,
                power=settings.push_out_drive_power,
                pid_p=settings.drive_pid_p,
                pid_i=settings.drive_pid_i,
                pid_d=settings.drive_pid_d,
                target_type=HeadingType.RELATIVE,
            ),
            IsColorDetected(name="detect push-out black line", color=Color.BLACK),
        ]
    )

    # 黒ラインを検知した位置から30mm進み、ボトル全体を境界の外へ押し出す。
    push_beyond_line = Parallel(
        name="push bottle 30 mm beyond black line",
        policy=ParallelPolicy.SuccessOnOne(),
    )
    push_beyond_line.add_children(
        [
            RunByGyro(
                name="run straight after push-out line",
                target=0,
                power=settings.push_out_drive_power,
                pid_p=settings.drive_pid_p,
                pid_i=settings.drive_pid_i,
                pid_d=settings.drive_pid_d,
                target_type=HeadingType.RELATIVE,
            ),
            IsDistanceEarned(name="push-out margin distance", delta_dist=settings.push_out_after_line_distance_mm),
        ]
    )

    # ボトルを押した向きのまま直線後退し、アームから確実に離脱する。
    release_reverse_motor = RunAsInstructed(name="straight reverse release motor command", pwm_l=-settings.release_reverse_power, pwm_r=-settings.release_reverse_power)
    release_reverse = Parallel(
        name="reverse straight to release sumo bottle",
        policy=ParallelPolicy.SuccessOnOne(),
    )
    release_reverse.add_children(
        [
            release_reverse_motor,
            IsDistanceEarned(name="sumo bottle release reverse distance", delta_dist=settings.release_reverse_distance_mm),
        ]
    )

    # 離脱後は後退を続けつつ少しガレージ側へ曲がり、復帰用の黒ラインを探す。
    garage_return_motor = RunAsInstructed(name="garage-side reverse curve motor command", pwm_l=-settings.garage_return_reverse_left_pwm, pwm_r=-settings.garage_return_reverse_right_pwm)
    find_garage_side_line = Parallel(
        name="reverse curve to garage-side black line",
        policy=ParallelPolicy.SuccessOnOne(),
    )
    find_garage_side_line.add_children(
        [
            garage_return_motor,
            IsColorDetected(name="detect garage-side black line", color=Color.BLACK),
        ]
    )

    # 黒ラインを検知した後に短距離だけライントレースし、FINISHへ渡せる姿勢へ安定させる。
    stabilize_line_trace = Parallel(
        name="stabilize on garage-side black line",
        policy=ParallelPolicy.SuccessOnOne(),
    )
    stabilize_line_trace.add_children(
        [
            TraceLine(
                name="trace garage-side black line",
                target=settings.line_rejoin_trace_target_v,
                power=settings.line_rejoin_trace_power,
                pid_p=0.55,
                pid_i=0.0000009,
                pid_d=0.015,
                trace_side=TraceSide.NORMAL,
            ),
            IsDistanceEarned(name="garage-side line stabilization distance", delta_dist=settings.line_rejoin_trace_distance_mm),
        ]
    )

    transport = Sequence(name="carry sumo bottle to exit", memory=True)
    transport.add_children(
        [
            # 実行順1：捕捉したボトルを保持したまま、押し出し用黒ラインへ直進する。
            AnnounceSumoTransportStage(
                name="begin straight push-out with bottle held",
                message="driving straight to push-out black line; bottle remains held",
            ),
            find_push_out_line,
            StopNow(name="stop at push-out black line"),
            # 実行順2：黒ラインから30mm直進し、ボトルを確実に押し出す。
            push_beyond_line,
            StopNow(name="stop after push-out margin"),
            MarkSumoExitState("mark sumo bottle pushed out", context, "pushed_out"),
            # 実行順3：旋回せず、そのまま真っすぐ後退してアームからボトルを外す。
            AnnounceSumoTransportStage(
                name="begin straight reverse release",
                message="reversing straight to release bottle from arm",
            ),
            ConfigureCourseIndependentReversePwm("configure straight reverse release pwm", release_reverse_motor, settings.release_reverse_power),
            release_reverse,
            StopNow(name="stop after sumo bottle release reverse"),
            MarkSumoExitState("mark sumo bottle released", context, "released"),
            # 実行順4：少しガレージ側へそれる後退カーブで、復帰用黒ラインを検知する。
            ConfigureMirroredGarageReturnPwm("configure mirrored garage return pwm", garage_return_motor, settings),
            find_garage_side_line,
            StopNow(name="stop on garage-side black line"),
            # 実行順5：短距離のライントレースで姿勢を整え、FINISH工程へ引き渡す。
            stabilize_line_trace,
            StopNow(name="stop after garage-side line stabilization"),
            MarkSumoExitState("mark sumo line trace ready", context, "line_trace_ready"),
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
