"""Feature 18: push out the captured bottle and rejoin the garage-side line."""

import math

from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time

from ..behaviours.conditions import IsDistanceEarned,IsColorDetected
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
        self.push_bearing = None
        self.avoid_bearing = None
        self.escape_route_2_bearing = None

    def update(self):
        # 押し出し完了直後、後退する前の実測方位で一度だけ計画する。
        pushed = current_bearing(self.context)
        # 押し出し完了時の方位を保存
        self.push_bearing = pushed
        relative = (-runtime.course * (pushed - self.settings.entry_bearing_deg)) % 360.0
        # 押し出した方向に対してコース外側へ90度向け、
        # 黒ライン探索前にボトルの進路から横へ退避する。
        self.avoid_bearing = (
            pushed - runtime.course * 90.0
        ) % 360.0
        # 退避ルート②の走行方位を計算する。
        # Left/Rightはruntime.courseによって鏡像にする。
        self.escape_route_2_bearing = (
            pushed
            - runtime.course * self.settings.escape_route_2_angle_deg
        ) % 360.0
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

            # Feature16で決定した退避ルートに応じて、
            # 退避後の推定位置を計算する。
            escape_route = self.context.sumo.escape_route

            if escape_route == 1:
                # ==================================================
                # 退避ルート①
                # 押し出し方向から90度横へ退避する。
                # ==================================================
                escape_bearing = self.avoid_bearing
                escape_distance_mm = self.settings.garage_avoid_distance_mm

            elif escape_route == 2:
                # ==================================================
                # 退避ルート②
                # 設定した角度の斜め方向へ退避する。
                # ==================================================
                escape_bearing = self.escape_route_2_bearing
                escape_distance_mm = self.settings.escape_route_2_distance_mm

            else:
                self.logger.error("Escape route is not selected: %s" % str(escape_route))
                return Status.FAILURE

            # 選択された退避方向を、
            # 相撲開始方向を基準としたコース正規化角度へ変換する。
            escape_relative = (
                -runtime.course
               * (escape_bearing - self.settings.entry_bearing_deg)
            ) % 360.0

            escape_angle = math.radians(escape_relative)

            # 退避走行後の推定座標を反映する。
            x += escape_distance_mm * math.sin(escape_angle)
            y += escape_distance_mm * math.cos(escape_angle)

            self.logger.info(
                "Escape route=%d bearing=%.1f distance=%.1f "
                "estimated_position=(%.1f,%.1f)"
                % (
                    escape_route,
                    escape_bearing,
                    escape_distance_mm,
                    x,
                    y,
                )
            )         
            
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

    def avoidance_bearing(self):
        if self.avoid_bearing is None:
            raise RuntimeError("Garage avoidance bearing has not been planned")
        return self.avoid_bearing

    def route_2_bearing(self):
        if self.escape_route_2_bearing is None:
            raise RuntimeError(
                "Escape route 2 bearing has not been planned"
            )

        return self.escape_route_2_bearing

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

class IsSumoEscapeRoute(Behaviour):
    """
    Feature 16で決定した退避ルートを確認する。
    route=1 : 退避ルート①
    route=2 : 退避ルート②
    """

    def __init__(self, name, context, route):
        super().__init__(name)
        self.context = context
        self.route = route

    def update(self):
        if self.context.sumo.escape_route == self.route:
            return Status.SUCCESS

        return Status.FAILURE

def build_move_to_sumo_exit(context, config):
    # No.18：前工程で合計500mm走行済み。直線後退で離脱し、ガレージ側黒ラインへ復帰する。
    settings = config.sumo
    return_plan = PlanGarageReturn(context, settings)
    
    blue_trace = Parallel(name="blue_trace", policy=ParallelPolicy.SuccessOnOne())
    blue_trace.add_children(
        [
            TraceLine(name="sensor trace normal edge", target=65,
                power=70, power_min=33,
                pid_p=0.65, pid_i=0.000001, pid_d=0.045,
                err_lo=6, err_hi=16, decel_per_s=350, gains_slow=(0.65, 0.045), gains_fast=(0.55, 0.065),
                recover_v=97, recover_after=3, recover_turn=35,
                trace_side=TraceSide.NORMAL),
            IsColorDetected(name="check color", color=Color.BLUE),
        ]
    )
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
    # ボトルから離脱後、黒ライン探索経路へ入る前に横へ退避する。
    garage_avoid_drive = Parallel(
        name="move sideways away from released sumo bottle",
        policy=ParallelPolicy.SuccessOnOne(),
    )

    garage_avoid_drive.add_children(
        [
            RunAtBearing(
                name="hold sumo bottle avoidance heading",
                context=context,
                bearing=return_plan.avoidance_bearing,
                power=settings.garage_avoid_power,
                pid_p=settings.drive_pid_p,
                pid_i=settings.drive_pid_i,
                pid_d=settings.drive_pid_d,
            ),
            IsDistanceEarned(
                name="sumo bottle avoidance distance",
                delta_dist=settings.garage_avoid_distance_mm,
            ),
        ]
    )

    # ======================================================
    # 退避ルート②：斜め方向へ一定距離走行する
    # ======================================================
    escape_route_2_drive = Parallel(
        name="drive sumo escape route 2",
        policy=ParallelPolicy.SuccessOnOne(),
    )

    escape_route_2_drive.add_children([
        # 計算した退避ルート②の方位を維持して走行する
        RunAtBearing(
            name="hold sumo escape route 2 bearing",
            context=context,
            bearing=return_plan.route_2_bearing,
            power=settings.escape_route_2_power,
            pid_p=settings.drive_pid_p,
            pid_i=settings.drive_pid_i,
            pid_d=settings.drive_pid_d,
        ),

        # 設定した距離まで進んだら終了する
        IsDistanceEarned(
            name="sumo escape route 2 distance",
            delta_dist=settings.escape_route_2_distance_mm,
        ),
    ])

    # ======================================================
    # 退避ルート➀
    # Feature16でescape_route=1と判定された場合に実行する
    # ======================================================
    escape_route_1 = Sequence(
        name="sumo escape route 1",
        memory=True,
    )

    escape_route_1.add_children([
        # Feature16で退避ルート①と判定されているか確認
        IsSumoEscapeRoute(
            name="check sumo escape route 1",
            context=context,
            route=1,
        ),

        # ボトルを押し出した方向から90度横へ向く
        SpinToBearing(
            name="escape route 1 turn sideways",
            context=context,
            bearing=return_plan.avoidance_bearing,
            max_power=settings.turn_max_power,
            min_power=settings.turn_min_power,
            pid_p=settings.turn_pid_p,
            pid_i=settings.turn_pid_i,
            pid_d=settings.turn_pid_d,
            tolerance=settings.heading_tolerance_deg,
        ),

        # 旋回終了後に停止
        StopNow(name="stop before escape route 1 drive"),
        # 横方向へ退避
        garage_avoid_drive,
        # 退避終了後に停止
        StopNow(name="stop after escape route 1 drive"),
    ])

    # ======================================================
    # 退避ルート➁
    # ======================================================
    escape_route_2 = Sequence(
        name="sumo escape route 2",
        memory=True,
    )

    escape_route_2.add_children([
        IsSumoEscapeRoute(
            name="check sumo escape route 2",
            context=context,
            route=2,
        ),

        # 押し出し方向から退避ルート②の方向へ旋回
        SpinToBearing(
            name="turn toward sumo escape route 2",
            context=context,
            bearing=return_plan.route_2_bearing,
            max_power=settings.turn_max_power,
            min_power=settings.turn_min_power,
            pid_p=settings.turn_pid_p,
            pid_i=settings.turn_pid_i,
            pid_d=settings.turn_pid_d,
            tolerance=settings.heading_tolerance_deg,
        ),

        # 旋回終了後に一度停止
        StopNow(name="stop before sumo escape route 2 drive"),

        # 退避ルート②を走行
        escape_route_2_drive,

        # 退避終了
        StopNow(name="stop after sumo escape route 2 drive"),
    ])

    # ======================================================
    # 退避ルート➀/➁を選択
    # ======================================================
    escape_route_selector = Selector(
        name="select sumo escape route",
        memory=True,
    )

    escape_route_selector.add_children([
        escape_route_1,
        escape_route_2,
    ])


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
           MarkSumoExitState(
                "mark sumo bottle released",
                context,
                "released",
            ),

            # Feature16で決定した退避ルート①/②を実行する
            escape_route_selector,

            # 退避完了後、ガレージ側黒ラインへ向く。
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
            blue_trace,
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
