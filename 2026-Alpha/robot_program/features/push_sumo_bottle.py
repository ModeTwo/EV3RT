"""Feature 17: approach and capture the sumo bottle with the lowered arm."""

from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time

from ..behaviours.conditions import IsDistanceEarned
from ..behaviours.gyro_drive import RunByGyro
from ..behaviours.motor_control import StopNow


class SkipUnavailableCapture(Behaviour):
    # 距離センサーで有効なボトル候補を得られなかった場合は捕捉を省略する。
    def __init__(self, name, context):
        super().__init__(name)
        self.context = context

    def update(self):
        state = self.context.sumo
        if state.skipped or state.bottle_distance_mm is None:
            runtime.require("plotter")
            self.logger.warning(
                "%+06d %s.capture skipped reason=%s"
                % (
                    runtime.plotter.get_distance(),
                    self.__class__.__name__,
                    state.failure_reason or "bottle distance unavailable",
                )
            )
            return Status.SUCCESS
        return Status.FAILURE


class ConfigureApproachDistance(Behaviour):
    # 測定距離から捕捉時のセンサー・ボトル間距離を引き、直進終了条件へ設定する。
    def __init__(self, name, context, settings, distance_condition):
        super().__init__(name)
        self.context = context
        self.settings = settings
        self.distance_condition = distance_condition

    def update(self):
        state = self.context.sumo
        if state.bottle_distance_mm is None:
            return Status.FAILURE
        state.approach_distance_mm = max(
            0.0,
            state.bottle_distance_mm
            - self.settings.bottle_front_target_distance_mm,
        )
        self.distance_condition.delta_dist = state.approach_distance_mm
        return Status.SUCCESS


class MarkBottleCaptured(Behaviour):
    # アームを下げた状態で規定位置まで進んだことを、オープンループの捕捉完了として記録する。
    def __init__(self, name, context):
        super().__init__(name)
        self.context = context

    def update(self):
        self.context.sumo.bottle_captured = True
        return Status.SUCCESS


def build_push_sumo_bottle(context, config):
    # No.17：距離センサーで正対したボトルへ低速直進し、下げたアーム内へ捕捉する。
    settings = config.sumo

    approach_distance = IsDistanceEarned(
        name="sumo bottle approach distance",
        delta_dist=0,
    )
    approach = Parallel(
        name="approach sumo bottle",
        policy=ParallelPolicy.SuccessOnOne(),
    )
    approach.add_children(
        [
            RunByGyro(
                name="run toward sumo bottle",
                target=0,
                power=settings.approach_power,
                pid_p=settings.drive_pid_p,
                pid_i=settings.drive_pid_i,
                pid_d=settings.drive_pid_d,
                target_type=HeadingType.RELATIVE,
            ),
            approach_distance,
        ]
    )

    execute = Sequence(name="execute sumo bottle capture", memory=True)
    execute.add_children(
        [
            # 実行順1：検出距離に応じて、ボトルをアーム内へ捕捉する直進距離を確定する。
            ConfigureApproachDistance(
                "configure sumo bottle approach distance",
                context,
                settings,
                approach_distance,
            ),
            # 実行順2：旋回は完了済みなので、下げたアームで挟むように低速直進する。
            approach,
            StopNow(name="stop after capturing sumo bottle"),
            # 実行順3：接触センサーは使わず、規定距離到達を捕捉成立として次工程へ渡す。
            MarkBottleCaptured("mark sumo bottle captured", context),
        ]
    )

    root = Selector(name="push_sumo_bottle", memory=True)
    root.add_children(
        [
            SkipUnavailableCapture("skip unavailable sumo capture", context),
            execute,
        ]
    )
    return root
