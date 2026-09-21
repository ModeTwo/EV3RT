"""Features 16 and 17: find and capture the sumo bottle with the camera."""

from simple_pid import PID

from .bt_imports import Behaviour, BottleColor, HeadingType, Parallel, ParallelPolicy, Selector, Sequence, Status, runtime, time

from ..behaviours.conditions import IsDistanceEarned
from .sumo_bearing_motion import current_bearing, EncoderSpinToBearing as SpinToBearing
from ..behaviours.motor_control import StopNow
from ..behaviours.gyro_drive import RunByGyro
from ..timing import CONTROL_INTERVAL_SEC


def _normalize_heading(angle):
    # 方位差を-180度以上180度未満へ正規化し、0/360度境界で逆回転しないようにする。
    return (float(angle) + 180.0) % 360.0 - 180.0


class CaptureSumoBottleWithCamera(Behaviour):
    ACQUIRE = 0
    SEARCH_ADVANCE = 6
    PRE_APPROACH = 1
    APPROACH = 2
    ALIGN = 3
    SETTLE = 4
    REALIGN = 5

    def __init__(self, name, context, settings):
        super().__init__(name)
        self.context = context
        self.settings = settings
        self.phase = self.ACQUIRE
        self.session = None
        self.last_frame_id = -1
        self.confirmed_frames = 0
        self.lost_frames = 0
        self.last_bottom_row = 0
        self.last_theta = None
        self.target_bearing = None
        self.started_at = 0.0
        self.approach_started_at = None
        self.last_drive_log_at = None
        self.realign_count = 0
        self.max_realign_count = 1
        self.initial_alignment = False
        self.gyro_drive = None
        self.pre_approach_distance = None
        self.search_advance_drive = None
        self.search_advance_distance = None

        # 150mm前進を実施済みか
        # Trueになった後は、再び3秒経過しても追加前進しない
        self.search_advance_done = False
        self.pid = None

    def initialise(self):
        self.alignment_turn = None
        self.alignment_checked = False
        self.settle_until = 0.0
        runtime.require(
            "plotter", "video", "gyro_sensor", "right_motor", "left_motor"
        )
        # 全体ResetDevice直後に登録済みであることを、モーター走行前に確認する。
        current_bearing(self.context)
        self.phase = self.ACQUIRE
        self.session = runtime.video.begin_sumo_bottle_read()
        self.last_frame_id = -1
        self.confirmed_frames = 0
        self.lost_frames = 0
        self.last_bottom_row = 0
        self.last_theta = None
        self.target_bearing = None
        self.started_at = time.monotonic()
        self.approach_started_at = None
        self.last_drive_log_at = None
        self.realign_count = 0
        self.initial_alignment = False
        self.gyro_drive = None
        self.pre_approach_distance = None
        self.search_advance_drive = None
        self.search_advance_distance = None
        self.search_advance_done = False
        self.pid = PID(
            self.settings.camera_steer_gain,
            0.0,
            0.0,
            setpoint=0.0,
            sample_time=CONTROL_INTERVAL_SEC,
            output_limits=(
                -self.settings.camera_max_steer_power,
                self.settings.camera_max_steer_power,
            ),
        )

        self.remaining_distance = IsDistanceEarned(
            name="sumo remaining capture and push distance",
            delta_dist=self.settings.capture_and_push_distance_mm - 150.0,
        )
        self.context.sumo.bottle_captured = False
        self.context.sumo.bottle_pushed_out = False
        self.context.sumo.skipped = False
        self.context.sumo.failure_reason = None
        self.context.sumo.camera_capture_bearing_deg = None
        # 今回の黒ボトル位置から退避ルートを改めて決定するため、
        # 前回実行時の判定結果をリセットする。
        self.context.sumo.bottle_image_x_ratio = None
        self.context.sumo.escape_route = None
        # 黒テープ確定中は必ず静止し、同じフレームを複数回数えない。
        self._stop_motors()
        self.logger.info(
            "%+06d %s.black bottle acquisition started"
            % (runtime.plotter.get_distance(), self.__class__.__name__)
        )

    def _stop_motors(self):
        for motor in (runtime.right_motor, runtime.left_motor):
            motor.set_power(0)
            motor.set_brake(True)

    def _drive_toward_bottle(self, theta):
        # theta自体が画像上の左右方向を持つため、course符号による反転は行わない。
        turn = int(self.pid(theta))
        right_power = max(
            self.settings.camera_min_wheel_power,
            min(
                self.settings.camera_max_wheel_power,
                self.settings.camera_approach_power + turn,
            ),
        )
        left_power = max(
            self.settings.camera_min_wheel_power,
            min(
                self.settings.camera_max_wheel_power,
                self.settings.camera_approach_power - turn,
            ),
        )
        for motor in (runtime.right_motor, runtime.left_motor):
            motor.set_brake(False)
        runtime.right_motor.set_power(right_power)
        runtime.left_motor.set_power(left_power)
        return left_power, right_power

    def _current_bearing(self):
        return current_bearing(self.context)

    def _estimated_bottle_bearing(self, theta):
        # 画像右の角度は時計回りの方位差。カメラの左右はコースで反転しない。
        return (
            self._current_bearing() + float(theta)
        ) % 360.0

    def _drive_toward_locked_bearing(self):
        # 停止中に確定したボトル方位との差を画像角度相当に戻し、既存の高出力範囲で操舵する。
        if self.target_bearing is None:
            self._stop_motors()
            return 0, 0, 0.0
        heading_error = _normalize_heading(
            self.target_bearing - self._current_bearing()
        )
        # 方位目標を既存のジャイロ基準へ変換してから、従来の高出力操舵へ渡す。
        raw = runtime.gyro_sensor.get_angle()
        legacy_target = self.context.sumo.bearing_reference.legacy_target(self.target_bearing, raw, runtime.course)
        legacy_error = _normalize_heading(legacy_target - (-runtime.course * raw))
        equivalent_theta = -runtime.course * legacy_error
        left_power, right_power = self._drive_toward_bottle(equivalent_theta)
        return left_power, right_power, heading_error

    def _fail(self, reason):
        self.context.sumo.skipped = True
        self.context.sumo.failure_reason = reason
        self.logger.warning(
            "%+06d %s.%s"
            % (runtime.plotter.get_distance(), self.__class__.__name__, reason)
        )
        return Status.FAILURE
    
    def _decide_escape_route(self, cx, frame_width):
        """
        黒ボトルを確定した時点の画像上の位置から、
        押し出し後に使用する退避ルートを決定する。

        Leftコース:
            左40%  -> 退避ルート①
            右60%  -> 退避ルート②

        Rightコース:
            右40%  -> 退避ルート①
            左60%  -> 退避ルート②
        """

        if cx is None or frame_width <= 0:
            return False

        # 画像上のボトル位置を0.0～1.0へ変換
        x_ratio = float(cx) / float(frame_width)
        x_ratio = max(0.0, min(1.0, x_ratio))

        # Leftコース
        if runtime.course > 0:
            if x_ratio < 0.40:
                route = 1       # 退避ルート①
            else:
                route = 2       # 退避ルート②

        # Rightコース
        else:
            if x_ratio >= 0.60:
                route = 1       # 退避ルート①
            else:
                route = 2       # 退避ルート②

        # Feature 18で使用するため保存
        self.context.sumo.bottle_image_x_ratio = x_ratio
        self.context.sumo.escape_route = route

        course_name = "LEFT" if runtime.course > 0 else "RIGHT"

        self.logger.info(
            "SUMO ESCAPE ROUTE "
            "course=%s cx=%.1f width=%d ratio=%.3f route=%d"
            % (course_name,cx,frame_width,x_ratio,route,)
        )

        return True

    def _start_gyro_approach(self):
        """
        再確認後に確定した最終方位をRunByGyro用の絶対角度へ変換し、
        残り350mmの走行を開始する。
        """

        # 現在のジャイロ生値
        raw = runtime.gyro_sensor.get_angle()

        # 相撲用bearing → RunByGyroが使用する絶対角度へ変換
        gyro_target = self.context.sumo.bearing_reference.legacy_target(
            self.target_bearing,
            raw,
            runtime.course,
        )

        self.gyro_drive = RunByGyro(
            name="run remaining distance toward sumo bottle by gyro",
            target=gyro_target,
            power=self.settings.camera_approach_power,
            pid_p=self.settings.drive_pid_p,
            pid_i=self.settings.drive_pid_i,
            pid_d=self.settings.drive_pid_d,
            target_type=HeadingType.ABSOLUTE,
        )

        self.phase = self.APPROACH

        # 再確認後、残り350mmの計測を開始
        self.remaining_distance.update()

        # Feature18の位置計算で使用
        self.context.sumo.camera_capture_bearing_deg = self.target_bearing

        self.logger.info(
            "Starting remaining-distance RunByGyro "
            "bearing=%.1f gyro_target=%.1f"
            % (self.target_bearing,gyro_target,)
        )

        # 最初の1tick
        self.gyro_drive.tick_once()

    def _start_search_advance(self):
        """
        初回の黒ボトル探索で3秒間検知できなかった場合、
        現在向いている方向へRunByGyroで150mm前進する。
        """

        raw = runtime.gyro_sensor.get_angle()

        # 現在向いている方向を維持して直進する
        current_heading = -runtime.course * raw

        self.search_advance_drive = RunByGyro(
            name="run 150mm after black bottle search timeout",
            target=current_heading,
            power=self.settings.retry_advance_power,
            pid_p=self.settings.drive_pid_p,
            pid_i=self.settings.drive_pid_i,
            pid_d=self.settings.drive_pid_d,
            target_type=HeadingType.ABSOLUTE,
        )

        self.search_advance_distance = IsDistanceEarned(
            name="150mm after black bottle search timeout",
            delta_dist=150.0,
        )

        # この地点から150mmを計測
        self.search_advance_distance.update()

        self.phase = self.SEARCH_ADVANCE
        self.search_advance_done = True

        self.logger.info(
            "Black bottle not detected for 3.0s; "
            "starting 150mm search advance heading=%.1f"
            % current_heading
        )

        self.search_advance_drive.tick_once()

    def _start_pre_approach(self):
        """
        初回黒ボトル検知で確定した方位へ
        RunByGyroで150mm接近する。
        """

        raw = runtime.gyro_sensor.get_angle()

        gyro_target = self.context.sumo.bearing_reference.legacy_target(
            self.target_bearing,
            raw,
            runtime.course,
        )

        self.gyro_drive = RunByGyro(
            name="run first 150mm toward sumo bottle by gyro",
            target=gyro_target,
            power=self.settings.camera_approach_power,
            pid_p=self.settings.drive_pid_p,
            pid_i=self.settings.drive_pid_i,
            pid_d=self.settings.drive_pid_d,
            target_type=HeadingType.ABSOLUTE,
        )

        self.pre_approach_distance = IsDistanceEarned(
            name="sumo first 150mm approach distance",
            delta_dist=150.0,
        )

        # ここを150mm走行の開始地点にする
        self.pre_approach_distance.update()

        self.phase = self.PRE_APPROACH

        self.logger.info(
            "Starting first 150mm RunByGyro "
            "bearing=%.1f gyro_target=%.1f"
            % (
                self.target_bearing,
                gyro_target,
            )
        )

        self.gyro_drive.tick_once()

    def update(self):
        # ------------------------------------------------------
        # 初回探索で3秒未検出だった場合の150mm前進
        # ------------------------------------------------------
        if self.phase == self.SEARCH_ADVANCE:

            # 150mm到達
            if self.search_advance_distance.update() == Status.SUCCESS:

                if self.search_advance_drive is not None:
                    self.search_advance_drive.stop(Status.INVALID)

                self._stop_motors()
        
                # 150mm前進後、再び黒ボトル探索へ戻る
                self.phase = self.ACQUIRE
        
                # 連続検知カウントをリセット
                self.confirmed_frames = 0

                # ここから再探索開始
                self.started_at = time.monotonic()

                self.logger.info(
                    "150mm search advance completed; "
                    "restarting black bottle acquisition"
                )

                return Status.RUNNING

            # 150mm到達までRunByGyroを継続
            if self.search_advance_drive is not None:
                self.search_advance_drive.tick_once()

                if self.search_advance_drive.status == Status.FAILURE:
                    self._stop_motors()
                    return Status.FAILURE

            return Status.RUNNING

        # ------------------------------------------------------
        # 初回黒ボトル検知後、その方向へ150mm接近
        # ------------------------------------------------------
        if self.phase == self.PRE_APPROACH:

            # 150mm到達
            if self.pre_approach_distance.update() == Status.SUCCESS:

                if self.gyro_drive is not None:
                    self.gyro_drive.stop(Status.INVALID)

                self._stop_motors()

                # 150mm接近後は、再確認用にカウントをリセット
                self.confirmed_frames = 0

                # 黒ボトルをもう一度確認
                self.phase = self.REALIGN

                self.logger.info(
                    "First 150mm approach completed; "
                    "checking black bottle again"
                )

                return Status.RUNNING

            # 150mmに到達するまでRunByGyroを継続
            if self.gyro_drive is not None:
                self.gyro_drive.tick_once()

                if self.gyro_drive.status == Status.FAILURE:
                    self._stop_motors()
                    return Status.FAILURE

            return Status.RUNNING

        # ------------------------------------------------------
        # 再確認後、必要な場合だけその場旋回
        # ------------------------------------------------------
        if self.phase == self.ALIGN:
        # 前進せず既存のその場旋回Behaviorでボトル方位へ整列する。
            self.alignment_turn.tick_once()
            if self.alignment_turn.status == Status.FAILURE:
                self._stop_motors()
                return Status.FAILURE
            if self.alignment_turn.status == Status.SUCCESS:
                self._stop_motors()
                self.settle_until = time.monotonic() + self.settings.camera_alignment_settle_sec
                self.phase = self.SETTLE
            return Status.RUNNING

        if self.phase == self.SETTLE:
            self._stop_motors()

            if time.monotonic() >= self.settle_until:
                
                # --------------------------------------------------
                # 初回黒ボトル検知後の旋回が完了
                # → この方位のまま150mm接近
                # --------------------------------------------------
                if self.initial_alignment:
                    self.initial_alignment = False

                    self.logger.info(
                        "Initial bottle alignment completed; "
                        "starting first 150mm approach "
                        "target=%.1f"
                        % self.target_bearing
                    )

                    self._start_pre_approach()
                    return Status.RUNNING
                # --------------------------------------------------
                # 再補正を1回実施済み
                # → 2回目のカメラ確認をせずRunByGyroへ
                # --------------------------------------------------
                if self.realign_count >= self.max_realign_count:

                    self.logger.info(
                        "Realignment completed; "
                        "skipping second camera check and starting RunByGyro "
                        "target=%.1f"
                        % self.target_bearing
                    )
                    self._start_gyro_approach()
                    return Status.RUNNING

                # ==================================================
                # 初回旋回後
                # → 1回だけカメラで再確認
                # ==================================================
                self.phase = self.REALIGN
                self.confirmed_frames = 0

                self.logger.info(
                    "Alignment settled; checking bottle once before gyro drive"
                )

            return Status.RUNNING

        # ------------------------------------------------------
        # 旋回後に黒ボトルを再確認する
        # ------------------------------------------------------
        if self.phase == self.REALIGN:

            # 停止した状態で確認する
            self._stop_motors()

            session, frame_id, observation = runtime.video.get_bottle_observation()

            # 古いセッションや同じフレームは使用しない
            if session != self.session or frame_id <= self.last_frame_id:
                return Status.RUNNING

            self.last_frame_id = frame_id

            insight, color, cx, theta, bottom_row, area, in_blind = observation

            valid = (
                insight
                and color == BottleColor.BLACK
                and area >= self.settings.camera_min_area_px
            )

            # 黒ボトルが見つからなければ、次のフレームを待つ
            if not valid:
                self.confirmed_frames = 0
                return Status.RUNNING

            self.confirmed_frames += 1

            self.logger.info(
                "SUMO REALIGN DETECT "
                "frame=%d cx=%s theta=%.1f area=%.1f bearing=%.1f"
                % (
                    frame_id,
                    cx,
                    theta,
                    area,
                    self._current_bearing(),
                )
            )

            # 1フレームだけではなく、通常の検出と同じ回数確認する
            if self.confirmed_frames < self.settings.camera_confirm_frames:
                return Status.RUNNING

            # 確認完了
            self.confirmed_frames = 0

            # --------------------------------------------------
            # まだボトルが正面からずれている場合
            # --------------------------------------------------
            if (
                abs(theta) > self.settings.camera_alignment_tolerance_deg
                and self.realign_count < self.max_realign_count
            ):
                self.realign_count += 1

                # 「現在の方位 + 今回改めて測ったtheta」で
                # 新しいボトル方位を計算する
                self.target_bearing = self._estimated_bottle_bearing(theta)

                self.logger.info(
                    "Bottle realignment requested "
                    "count=%d theta=%.1f target=%.1f"
                    % (
                        self.realign_count,
                         theta,
                        self.target_bearing,
                    )
                )

                self.alignment_turn = SpinToBearing(
                    name="realign to camera bottle bearing",
                    context=self.context,
                    bearing=self.target_bearing,
                    max_power=self.settings.turn_max_power,
                    min_power=self.settings.turn_min_power,
                    pid_p=self.settings.turn_pid_p,
                    pid_i=self.settings.turn_pid_i,
                    pid_d=self.settings.turn_pid_d,
                    tolerance=self.settings.heading_tolerance_deg,
                )

                self.phase = self.ALIGN
                return Status.RUNNING

            # --------------------------------------------------
            # ボトルがほぼ正面、または最大補正回数に到達
            # → ここから350mm走行開始
            # --------------------------------------------------
            self.target_bearing = self._estimated_bottle_bearing(theta)

            self.logger.info(
                "Bottle alignment confirmed "
                "theta=%.1f target=%.1f; starting RunByGyro"
                "starting distance drive"
                % (theta,self.target_bearing,)
            )

            self._start_gyro_approach()
            return Status.RUNNING
        

        # 方位確定後は画像更新を待たず、毎制御周期で500mm到達を確認する。
        # キャッチ・押し出しを一つの距離へ含め、死角判定による追加走行は行わない。
        if self.phase == self.APPROACH:
            if self.remaining_distance.update() == Status.SUCCESS:
                if self.gyro_drive is not None:
                    self.gyro_drive.stop(Status.INVALID)
                self._stop_motors()
                self.logger.info("Capture and push distance completed; proceeding to reverse")
                return Status.SUCCESS
            # RunByGyroを毎制御周期実行
            if self.gyro_drive is not None:
                self.gyro_drive.tick_once()

                if self.gyro_drive.status == Status.FAILURE:
                    self._stop_motors()
                    return Status.FAILURE
    
            return Status.RUNNING

        now = time.monotonic()
        session, frame_id, observation = runtime.video.get_bottle_observation()

        # カメラモード切替前の結果と、制御周期内で再読した同一フレームは無視する。
        if session != self.session or frame_id <= self.last_frame_id:
            if self.phase == self.APPROACH:
                # カメラの新規フレーム待ちでも、20ms周期のジャイロ値で固定方位へ制御し続ける。
                self._drive_toward_locked_bearing()
            return Status.RUNNING
        self.last_frame_id = frame_id

        insight, color, cx, theta, bottom_row, area, in_blind = observation

        if insight and color == BottleColor.BLACK:
            self.logger.info(
                "SUMO BLACK DETECT "
                "frame=%d cx=%s theta=%.1f bottom=%s area=%.1f blind=%s bearing=%.1f"
                % (
                    frame_id,
                    cx,
                    theta,
                    bottom_row,
                    area,
                    in_blind,
                    self._current_bearing(),
                )
            )
        
        

        valid = (
            insight
            and color == BottleColor.BLACK
            and area >= self.settings.camera_min_area_px
        )

        # 実行単位1：静止したまま黒テープを連続した新規フレームで確認する。
        if self.phase == self.ACQUIRE:
            # --------------------------------------------------
            # 黒ボトルを3秒間確定できなかった場合
            # → 150mm前進してから再探索する
            #
            # search_advance_done=Trueなら、
            # 150mm前進はすでに実施済みなので再実行しない。
            # --------------------------------------------------
            if (
                not self.search_advance_done
                and time.monotonic() - self.started_at >= 3.0
            ):
                self.confirmed_frames = 0
                self._stop_motors()

                self.logger.info(
            "Black bottle acquisition timeout after 3.0s"
                )

                self._start_search_advance()
                return Status.RUNNING

            # 黒ボトルの連続検知
            self.confirmed_frames = (
                self.confirmed_frames + 1
                if valid
                else 0
            )

            if self.confirmed_frames < self.settings.camera_confirm_frames:
                return Status.RUNNING
            # ======================================================
            # 黒ボトルの位置から退避ルート①/②を決定
            # =====================================================

            frame_width = 320

            # 一度だけ退避ルートを決定する
            if self.context.sumo.escape_route is None:
                self._decide_escape_route(
                    cx=cx,
                    frame_width=frame_width,
                )


            # ======================================================
            # ボトル方位を確定
            # ======================================================

            self.target_bearing = self._estimated_bottle_bearing(theta)

            self.logger.info(
                "%+06d %s.black bottle confirmed "
                "frame=%d theta=%.1f area=%d target_bearing=%.1f; "
                "aligning before first 150mm approach"
                % (
                    runtime.plotter.get_distance(),
                    self.__class__.__name__,
                    frame_id,
                    theta,
                    area,
                    self.target_bearing,
                )
            )

            # ------------------------------------------------------
            # まずその場で黒ボトル方向を向く
            # ------------------------------------------------------
            self.initial_alignment = True
            self.alignment_turn = SpinToBearing(
                name="initial align to sumo bottle before 150mm approach",
                context=self.context,
                bearing=self.target_bearing,
                max_power=self.settings.turn_max_power,
                min_power=self.settings.turn_min_power,
                pid_p=self.settings.turn_pid_p,
                pid_i=self.settings.turn_pid_i,
                pid_d=self.settings.turn_pid_d,
                tolerance=self.settings.heading_tolerance_deg,
            )

            self.phase = self.ALIGN
            return Status.RUNNING


    def terminate(self, new_status):
        if getattr(self, "alignment_turn", None) is not None:
            self.alignment_turn.stop(Status.INVALID)
        # 成功、失敗、中断のどの場合も次のBehaviorへ出力を残さない。
        if runtime.right_motor is not None and runtime.left_motor is not None:
            self._stop_motors()


class MarkCameraCaptureCompleted(Behaviour):
    # 総距離走行を終えたことを、捕捉・押し出し完了の仮定として記録する。
    def __init__(self, name, context):
        super().__init__(name)
        self.context = context

    def update(self):
        self.context.sumo.bottle_captured = True
        self.context.sumo.bottle_pushed_out = True
        return Status.SUCCESS


class CompleteSkippedCameraCapture(Behaviour):
    # 未検出時はET相撲を失敗終了させず、後続の運搬を安全に省略できる状態へする。
    def __init__(self, name, context):
        super().__init__(name)
        self.context = context

    def update(self):
        return Status.SUCCESS if self.context.sumo.skipped else Status.FAILURE


def build_capture_sumo_bottle_camera(context, config):
    # No.16・17：画像で方位確定後、キャッチと押し出しを含む合計500mmを走る。
    # 追加の死角150mm・押し出し距離を足さず、No.18の直線後退へ引き渡す。
    capture = Sequence(name="capture sumo bottle by camera", memory=True)
    capture.add_children([
        CaptureSumoBottleWithCamera(
            name="locate and approach sumo bottle by camera",
            context=context,
            settings=config.sumo,
        ),
        StopNow(name="stop after total sumo capture and push distance"),
        MarkCameraCaptureCompleted(
            name="mark camera sumo capture completed",
            context=context,
        ),
    ])
    return capture
