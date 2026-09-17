"""Feature 18: push out the captured bottle and rejoin the garage-side line."""

import math

from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time

from ..behaviours.conditions import IsDistanceEarned
from .sumo_bearing_motion import RunAtBearing, SpinToBearing, current_bearing
from ..behaviours.line_trace import TraceLine
from ..behaviours.motor_control import RunAsInstructed, StopNow


class PlanGarageReturn(Behaviour):
    # 相撲開始方向(entry_bearing_deg)を0度、土俵側への旋回を正とする。
    # Leftは反時計回り、Rightは時計回りで同じ120度条件を使う。
    def __init__(self, context, settings):
        super().__init__(name="plan garage return from push heading")
        self.context = context
        self.settings = settings
        self.target_bearing = None
        self.search_limit_mm = None

    def update(self):
        # 押し出し完了直後、後退する前の実測方位で一度だけ計画する。
        pushed = current_bearing(self.context)
        relative = (-runtime.course * (pushed - self.settings.entry_bearing_deg)) % 360.0
        self.search_limit_mm = self.settings.garage_line_search_max_distance_mm
        if self.settings.garage_point_return_enabled:
            # 直線各区間の設定距離で後退後の位置を近似する試作。
            # 初期直進は+Y、土俵向き後退は-X。滑りや旋回中心の移動は未補正。
            capture = self.context.sumo.camera_capture_bearing_deg
            if capture is None:
                self.logger.error("Capture bearing missing; cannot plan point return")
                return Status.FAILURE
            capture_angle = math.radians((-runtime.course * (
                capture - self.settings.entry_bearing_deg)) % 360.0)
            reverse_angle = math.radians(relative)
            x = (-self.settings.camera_retreat_distance_mm
                 + self.settings.capture_and_push_distance_mm * math.sin(capture_angle)
                 - self.settings.release_reverse_distance_mm * math.sin(reverse_angle))
            y = (self.settings.start_straight_distance_mm
                 + self.settings.capture_and_push_distance_mm * math.cos(capture_angle)
                 - self.settings.release_reverse_distance_mm * math.cos(reverse_angle))
            target_x = self.settings.garage_line_offset_mm
            # 緑回避時は直線500mmの仮定ではなく、前進カーブを含む積分終点を使う。
            if self.settings.green_avoidance_enabled and self.context.sumo.push_end_position_mm is not None:
                end_x, end_y = self.context.sumo.push_end_position_mm
                x = end_x - self.settings.release_reverse_distance_mm * math.sin(reverse_angle)
                y = end_y - self.settings.release_reverse_distance_mm * math.cos(reverse_angle)
            target_y = self.settings.garage_blue_forward_mm - self.settings.garage_rejoin_before_blue_mm
            if x >= target_x or self.settings.garage_rejoin_before_blue_mm <= 0:
                self.logger.error("Point return geometry invalid; robot must remain before return line")
                return Status.FAILURE
            angle = math.degrees(math.atan2(target_x - x, target_y - y))
            distance = math.hypot(target_x - x, target_y - y)
            margin = self.settings.garage_search_margin_mm
            if not math.isfinite(distance) or not math.isfinite(margin) or margin < 0:
                self.logger.error("Invalid garage search distance or margin")
                return Status.FAILURE
            self.search_limit_mm = math.ceil(distance + margin)
            self.logger.info("Garage search distance=%.1f margin=%.1f limit=%d mm" % (
                distance, margin, self.search_limit_mm))
            self.target_bearing = (self.settings.entry_bearing_deg - runtime.course * angle) % 360.0
            self.logger.info("Point return estimated_after_reverse=(%.1f,%.1f) target=(%.1f,%.1f) bearing=%.1f distance=%.1f" % (
                x, y, target_x, target_y, self.target_bearing, math.hypot(target_x-x, target_y-y)))
            return Status.SUCCESS
        # 互換モード：120度ちょうども加算側。
        offset = -50.0 if relative > 120.0 else 50.0
        self.target_bearing = (self.settings.entry_bearing_deg
                               - runtime.course * (relative + offset)) % 360.0
        self.logger.info("garage return plan push_bearing=%.1f relative=%.1f target=%s rule=%s" % (
            pushed, relative, self.target_bearing,
            "subtract_50" if relative > 120.0 else "add_50"))
        return Status.SUCCESS

    def bearing(self):
        # 計画より先に走行しない。後退後のジャイロ値で再選択しない。
        if self.target_bearing is None:
            raise RuntimeError("Garage return bearing has not been planned")
        return self.target_bearing


class PlannedGarageDistance(IsDistanceEarned):
    # 探索開始時に計画済み上限を設定し、既存の距離判定を再利用する。
    def __init__(self, plan):
        super().__init__(name="garage-side black line search limit", delta_dist=0)
        self.plan = plan

    def update(self):
        if not self.running:
            if self.plan.search_limit_mm is None:
                raise RuntimeError("Garage return distance has not been planned")
            self.delta_dist = self.plan.search_limit_mm
        return super().update()


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
    def __init__(self, name, context, settings, plan=None):
        super().__init__(name)
        self.context = context
        self.settings = settings
        self.plan = plan

    def update(self):
        runtime.require("plotter")
        self.context.sumo.failure_reason = "garage_line_not_found"
        self.logger.error(
            "%+06d %s.garage-side black line not found within %.0f mm"
            % (
                runtime.plotter.get_distance(),
                self.__class__.__name__,
                self.plan.search_limit_mm if self.plan is not None else self.settings.garage_line_search_max_distance_mm,
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
    return_plan = PlanGarageReturn(context, settings)

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
                bearing=lambda: (return_plan.bearing() if settings.garage_point_return_enabled
                                 else current_bearing(context)),
                power=settings.garage_return_drive_power,
                pid_p=settings.drive_pid_p,
                pid_i=settings.drive_pid_i,
                pid_d=settings.drive_pid_d,
            ),
            garage_line_detector,
            PlannedGarageDistance(return_plan),
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
                return_plan,
            ),
        ]
    )

    transport = Sequence(name="carry sumo bottle to exit", memory=True)
    transport.add_children(
        [
            return_plan,
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
                bearing=return_plan.bearing,
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
