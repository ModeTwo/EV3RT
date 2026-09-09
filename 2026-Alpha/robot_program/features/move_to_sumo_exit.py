"""Feature 18: push out the captured bottle and rejoin the garage-side line."""

from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time

from ..behaviours.conditions import IsDistanceEarned
from .sumo_bearing_motion import RunAtBearing, SpinToBearing, current_bearing, search_bearing
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


class FailWhenGarageLineWasNotFound(Behaviour):
    # ライン未検出のままFINISHへ進ませず、停止済みの状態でミッションを失敗終了させる。
    def __init__(self, name, context, settings):
        super().__init__(name)
        self.context = context
        self.settings = settings

    def update(self):
        runtime.require("plotter")
        self.context.sumo.failure_reason = "garage_line_not_found"
        self.logger.error(
            "%+06d %s.garage-side black line not found within %.0f mm"
            % (
                runtime.plotter.get_distance(),
                self.__class__.__name__,
                self.settings.garage_line_search_max_distance_mm,
            )
        )
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
    # No.18：前工程で合計500mm走行済み。直線後退で離脱し、ガレージ側黒ラインへ復帰する。
    settings = config.sumo

    # キャッチと押し出しは前工程の合計500mmに含まれるため、ここでは直線後退から開始する。

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

    # 候補選択による旋回後の方位を保持し、直進で黒ラインを探す。
    garage_line_detector = DetectDarkGarageLine(
        name="detect garage-side black line by brightness",
        context=context,
        settings=settings,
    )
    find_garage_side_line = Parallel(
        name="drive straight to garage-side black line",
        policy=ParallelPolicy.SuccessOnOne(),
    )
    find_garage_side_line.add_children(
        [
            RunAtBearing(
                name="hold garage heading and drive straight",
                context=context,
                bearing=lambda: current_bearing(context),
                power=settings.garage_return_drive_power,
                pid_p=settings.drive_pid_p,
                pid_i=settings.drive_pid_i,
                pid_d=settings.drive_pid_d,
            ),
            garage_line_detector,
            IsDistanceEarned(
                name="garage-side black line search limit",
                delta_dist=settings.garage_line_search_max_distance_mm,
            ),
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

    # 黒検知後の進入量を調整し、旋回時に機体がラインへ十分乗り込むようにする。
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

    # 黒検知成功時のみ追加直進、絶対180度合わせ、ライントレースを行う。
    complete_line_rejoin = Sequence(name="complete garage-side line rejoin", memory=True)
    complete_line_rejoin.add_children(
        [
            WasGarageLineFound("confirm garage-side black line was found", context),
            enter_garage_line,
            StopNow(name="stop after garage line entry"),
            # 黒検知後はコース図の下向き（方位180度）へ向き直す。
            # 相撲専用アダプターがリセット基準のジャイロ目標へ変換する。
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
            # 方位合わせ終了後に制動し、ガレージ方向へのライントレースを開始する。
            StopNow(name="stop after absolute garage alignment"),
            stabilize_line_trace,
            StopNow(name="stop after garage-side line stabilization"),
            MarkSumoExitState("mark sumo line trace ready", context, "line_trace_ready"),
        ]
    )
    handle_line_search_result = Selector(name="handle garage-side line search result", memory=True)
    handle_line_search_result.add_children(
        [
            complete_line_rejoin,
            FailWhenGarageLineWasNotFound(
                "fail when garage-side black line was not found",
                context,
                settings,
            ),
        ]
    )

    transport = Sequence(name="carry sumo bottle to exit", memory=True)
    transport.add_children(
        [
            # 実行順2：旋回せず、そのまま真っすぐ後退してアームからボトルを外す。
            AnnounceSumoTransportStage(
                name="begin straight reverse release",
                message="reversing straight to release bottle from arm",
            ),
            ConfigureCourseIndependentReversePwm("configure straight reverse release pwm", release_reverse_motor, settings.release_reverse_power),
            release_reverse,
            StopNow(name="stop after sumo bottle release reverse"),
            MarkSumoExitState("mark sumo bottle released", context, "released"),
            # 実行順3：後退完了時の方位±50度のうち、ガレージ方位180度から遠い候補を選ぶ。
            SpinToBearing(
                name="turn toward garage before line search",
                context=context,
                bearing=lambda: search_bearing(context, settings),
                max_power=settings.turn_max_power,
                min_power=settings.turn_min_power,
                pid_p=settings.turn_pid_p,
                pid_i=settings.turn_pid_i,
                pid_d=settings.turn_pid_d,
                tolerance=settings.heading_tolerance_deg,
            ),
            # 実行順4：旋回完了位置で制動し、後続直進が保持する絶対方位を確定する。
            StopNow(name="stop after garage heading turn"),
            # 実行順5：旋回後の絶対方位を維持して直進し、復帰用黒ラインを検知する。
            find_garage_side_line,
            StopNow(name="stop after garage-side black line search"),
            # 実行順6：検知成功時だけ短距離ライントレースし、FINISH工程へ引き渡す。
            handle_line_search_result,
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
