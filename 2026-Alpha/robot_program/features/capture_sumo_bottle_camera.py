"""Features 16 and 17: find and capture the sumo bottle with the camera."""

from simple_pid import PID

from .bt_imports import (
    Behaviour,
    BottleColor,
    HeadingType,
    Sequence,
    Status,
    runtime,
    time,
)

from ..behaviours.conditions import IsDistanceEarned
from .sumo_bearing_motion import (
    current_bearing,
    EncoderSpinToBearing as SpinToBearing,
)
from ..behaviours.motor_control import StopNow
from ..behaviours.gyro_drive import RunByGyro
from ..timing import CONTROL_INTERVAL_SEC


def _normalize_heading(angle):
    # 方位差を-180度以上180度未満へ正規化し、
    # 0/360度境界で逆回転しないようにする。
    return (float(angle) + 180.0) % 360.0 - 180.0


class CaptureSumoBottleWithCamera(Behaviour):

    # ------------------------------------------------------
    # フェーズ
    # ------------------------------------------------------
    ACQUIRE = 0
    SEARCH_ADVANCE = 6
    PRE_APPROACH = 1
    APPROACH = 2
    ALIGN = 3
    SETTLE = 4
    REALIGN = 5

    # 黒ボトル検知のタイムアウト
    BOTTLE_DETECT_TIMEOUT_SEC = 2.0

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

        # 再補正
        self.realign_count = 0
        self.max_realign_count = 1
        self.initial_alignment = False

        # RunByGyro
        self.gyro_drive = None

        # 初回検知後150mm接近
        self.pre_approach_distance = None

        # 2秒未検知時の150mm探索前進
        self.search_advance_drive = None
        self.search_advance_distance = None

        # Trueになった後は、
        # 再び2秒経過しても追加の150mm前進は行わない。
        self.search_advance_done = False

        # 最終残距離
        self.remaining_distance = None

        self.pid = None

    def initialise(self):

        self.alignment_turn = None
        self.alignment_checked = False
        self.settle_until = 0.0

        runtime.require(
            "plotter",
            "video",
            "gyro_sensor",
            "right_motor",
            "left_motor",
        )

        # 全体ResetDevice直後に登録済みであることを、
        # モーター走行前に確認する。
        current_bearing(self.context)

        self.phase = self.ACQUIRE

        # 相撲ボトル認識開始
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

        self.remaining_distance = None

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

        # 相撲状態初期化
        self.context.sumo.bottle_captured = False
        self.context.sumo.bottle_pushed_out = False
        self.context.sumo.skipped = False
        self.context.sumo.failure_reason = None
        self.context.sumo.camera_capture_bearing_deg = None

        # 黒ボトル探索中は停止
        self._stop_motors()

        self.logger.info(
            "%+06d %s.black bottle acquisition started"
            % (
                runtime.plotter.get_distance(),
                self.__class__.__name__,
            )
        )

    # ======================================================
    # モーター停止
    # ======================================================

    def _stop_motors(self):

        for motor in (
            runtime.right_motor,
            runtime.left_motor,
        ):
            motor.set_power(0)
            motor.set_brake(True)

    # ======================================================
    # カメラ角度による操舵
    # ======================================================

    def _drive_toward_bottle(self, theta):

        # theta自体が画像上の左右方向を持つため、
        # course符号による反転は行わない。
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

        for motor in (
            runtime.right_motor,
            runtime.left_motor,
        ):
            motor.set_brake(False)

        runtime.right_motor.set_power(right_power)
        runtime.left_motor.set_power(left_power)

        return left_power, right_power

    # ======================================================
    # 現在方位
    # ======================================================

    def _current_bearing(self):
        return current_bearing(self.context)

    # ======================================================
    # カメラthetaからボトル絶対方位を求める
    # ======================================================

    def _estimated_bottle_bearing(self, theta):

        return (
            self._current_bearing()
            + float(theta)
        ) % 360.0

    # ======================================================
    # 確定済み方位へ向かって走る
    # ======================================================

    def _drive_toward_locked_bearing(self):

        if self.target_bearing is None:
            self._stop_motors()
            return 0, 0, 0.0

        heading_error = _normalize_heading(
            self.target_bearing
            - self._current_bearing()
        )

        raw = runtime.gyro_sensor.get_angle()

        legacy_target = (
            self.context.sumo.bearing_reference.legacy_target(
                self.target_bearing,
                raw,
                runtime.course,
            )
        )

        legacy_error = _normalize_heading(
            legacy_target
            - (-runtime.course * raw)
        )

        equivalent_theta = (
            -runtime.course * legacy_error
        )

        left_power, right_power = (
            self._drive_toward_bottle(
                equivalent_theta
            )
        )

        return (
            left_power,
            right_power,
            heading_error,
        )

    # ======================================================
    # 失敗
    # ======================================================

    def _fail(self, reason):

        self.context.sumo.skipped = True
        self.context.sumo.failure_reason = reason

        self.logger.warning(
            "%+06d %s.%s"
            % (
                runtime.plotter.get_distance(),
                self.__class__.__name__,
                reason,
            )
        )

        return Status.FAILURE

    # ======================================================
    # 最終残距離走行開始
    # ======================================================

    def _start_gyro_approach(self):
        """
        再確認後に確定した最終方位を
        RunByGyro用の絶対角度へ変換する。

        ET相撲工程の総前進距離を500mmにする。

        通常:
            検知後150mm
            + 残り350mm
            = 500mm

        2秒未検知:
            探索前進150mm
            + 検知後150mm
            + 残り200mm
            = 500mm
        """

        raw = runtime.gyro_sensor.get_angle()

        gyro_target = (
            self.context.sumo.bearing_reference.legacy_target(
                self.target_bearing,
                raw,
                runtime.course,
            )
        )

        self.gyro_drive = RunByGyro(
            name=(
                "run remaining distance "
                "toward sumo bottle by gyro"
            ),
            target=gyro_target,
            power=self.settings.camera_approach_power,
            pid_p=self.settings.drive_pid_p,
            pid_i=self.settings.drive_pid_i,
            pid_d=self.settings.drive_pid_d,
            target_type=HeadingType.ABSOLUTE,
        )

        # ------------------------------------------
        # 総前進距離500mmになるよう残距離を計算
        # ------------------------------------------

        if self.search_advance_done:

            # 150mm 探索前進
            # + 150mm 検知後接近
            # = 300mm
            #
            # 500 - 300 = 200mm
            remaining_mm = (
                self.settings.capture_and_push_distance_mm
                - 300.0
            )

        else:

            # 150mm 検知後接近
            #
            # 500 - 150 = 350mm
            remaining_mm = (
                self.settings.capture_and_push_distance_mm
                - 150.0
            )

        self.remaining_distance = (
            IsDistanceEarned(
                name=(
                    "sumo remaining "
                    "capture and push distance"
                ),
                delta_dist=remaining_mm,
            )
        )

        self.logger.info(
            "SUMO remaining distance=%.1fmm "
            "search_advance_done=%s"
            % (
                remaining_mm,
                self.search_advance_done,
            )
        )

        self.phase = self.APPROACH

        # ここを残距離の計測開始地点にする
        self.remaining_distance.update()

        # Feature18で使用
        self.context.sumo.camera_capture_bearing_deg = (
            self.target_bearing
        )

        self.logger.info(
            "Starting remaining-distance RunByGyro "
            "bearing=%.1f "
            "gyro_target=%.1f "
            "remaining=%.1fmm"
            % (
                self.target_bearing,
                gyro_target,
                remaining_mm,
            )
        )

        # 最初の1tick
        self.gyro_drive.tick_once()

    # ======================================================
    # 2秒未検知時の150mm探索前進
    # ======================================================

    def _start_search_advance(self):
        """
        初回の黒ボトル探索で
        2秒間検知できなかった場合、
        現在向いている方向へ
        RunByGyroで150mm前進する。
        """

        raw = runtime.gyro_sensor.get_angle()

        # 現在向いている方向を維持
        current_heading = (
            -runtime.course * raw
        )

        self.search_advance_drive = RunByGyro(
            name=(
                "run 150mm after "
                "black bottle search timeout"
            ),
            target=current_heading,
            power=self.settings.retry_advance_power,
            pid_p=self.settings.drive_pid_p,
            pid_i=self.settings.drive_pid_i,
            pid_d=self.settings.drive_pid_d,
            target_type=HeadingType.ABSOLUTE,
        )

        self.search_advance_distance = (
            IsDistanceEarned(
                name=(
                    "150mm after "
                    "black bottle search timeout"
                ),
                delta_dist=150.0,
            )
        )

        # この地点から150mmを計測
        self.search_advance_distance.update()

        self.phase = self.SEARCH_ADVANCE

        # 探索前進済み
        self.search_advance_done = True

        self.logger.info(
            "Black bottle not detected for %.1fs; "
            "starting 150mm search advance heading=%.1f"
            % (
                self.BOTTLE_DETECT_TIMEOUT_SEC,
                current_heading,
            )
        )

        self.search_advance_drive.tick_once()

    # ======================================================
    # 初回検知後150mm接近
    # ======================================================

    def _start_pre_approach(self):
        """
        初回黒ボトル検知で確定した方位へ
        RunByGyroで150mm接近する。
        """

        raw = runtime.gyro_sensor.get_angle()

        gyro_target = (
            self.context.sumo.bearing_reference.legacy_target(
                self.target_bearing,
                raw,
                runtime.course,
            )
        )

        self.gyro_drive = RunByGyro(
            name=(
                "run first 150mm "
                "toward sumo bottle by gyro"
            ),
            target=gyro_target,
            power=self.settings.camera_approach_power,
            pid_p=self.settings.drive_pid_p,
            pid_i=self.settings.drive_pid_i,
            pid_d=self.settings.drive_pid_d,
            target_type=HeadingType.ABSOLUTE,
        )

        self.pre_approach_distance = (
            IsDistanceEarned(
                name=(
                    "sumo first "
                    "150mm approach distance"
                ),
                delta_dist=150.0,
            )
        )

        # ここを150mm走行開始地点にする
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

    # ======================================================
    # メイン更新処理
    # ======================================================

    def update(self):

        # ==================================================
        # 2秒未検知時の150mm探索前進
        # ==================================================

        if self.phase == self.SEARCH_ADVANCE:

            if (
                self.search_advance_distance.update()
                == Status.SUCCESS
            ):

                if (
                    self.search_advance_drive
                    is not None
                ):
                    self.search_advance_drive.stop(
                        Status.INVALID
                    )

                self._stop_motors()

                # 150mm前進後、再探索
                self.phase = self.ACQUIRE

                self.confirmed_frames = 0

                # 再探索開始時間
                self.started_at = time.monotonic()

                self.logger.info(
                    "150mm search advance completed; "
                    "restarting black bottle acquisition"
                )

                return Status.RUNNING

            if (
                self.search_advance_drive
                is not None
            ):
                self.search_advance_drive.tick_once()

                if (
                    self.search_advance_drive.status
                    == Status.FAILURE
                ):
                    self._stop_motors()
                    return Status.FAILURE

            return Status.RUNNING

        # ==================================================
        # 初回検知後150mm接近
        # ==================================================

        if self.phase == self.PRE_APPROACH:

            if (
                self.pre_approach_distance.update()
                == Status.SUCCESS
            ):

                if self.gyro_drive is not None:
                    self.gyro_drive.stop(
                        Status.INVALID
                    )

                self._stop_motors()

                # 再確認用カウントリセット
                self.confirmed_frames = 0

                # 2回目の黒ボトル検知タイムアウト計測開始
                self.started_at = time.monotonic()

                self.phase = self.REALIGN

                self.logger.info(
                    "First 150mm approach completed; "
                    "checking black bottle again "
                    "(timeout=%.1fs)"
                    % self.BOTTLE_DETECT_TIMEOUT_SEC
                )

                return Status.RUNNING

                self.logger.info(
                    "First 150mm approach completed; "
                    "checking black bottle again"
                )

                return Status.RUNNING

            if self.gyro_drive is not None:

                self.gyro_drive.tick_once()

                if (
                    self.gyro_drive.status
                    == Status.FAILURE
                ):
                    self._stop_motors()
                    return Status.FAILURE

            return Status.RUNNING

        # ==================================================
        # その場旋回
        # ==================================================

        if self.phase == self.ALIGN:

            self.alignment_turn.tick_once()

            if (
                self.alignment_turn.status
                == Status.FAILURE
            ):
                self._stop_motors()
                return Status.FAILURE

            if (
                self.alignment_turn.status
                == Status.SUCCESS
            ):

                self._stop_motors()

                self.settle_until = (
                    time.monotonic()
                    + self.settings.camera_alignment_settle_sec
                )

                self.phase = self.SETTLE

            return Status.RUNNING

        # ==================================================
        # 旋回後静止
        # ==================================================

        if self.phase == self.SETTLE:

            self._stop_motors()

            if (
                time.monotonic()
                >= self.settle_until
            ):

                # ------------------------------------------
                # 初回旋回完了
                # → 150mm接近
                # ------------------------------------------

                if self.initial_alignment:

                    self.initial_alignment = False

                    self.logger.info(
                        "Initial bottle alignment "
                        "completed; "
                        "starting first 150mm approach "
                        "target=%.1f"
                        % self.target_bearing
                    )

                    self._start_pre_approach()

                    return Status.RUNNING

                # ------------------------------------------
                # 再補正を1回実施済み
                # → そのまま残距離走行
                # ------------------------------------------

                if (
                    self.realign_count
                    >= self.max_realign_count
                ):

                    self.logger.info(
                        "Realignment completed; "
                        "skipping second camera check "
                        "and starting RunByGyro "
                        "target=%.1f"
                        % self.target_bearing
                    )

                    self._start_gyro_approach()

                    return Status.RUNNING

                # ------------------------------------------
                # 旋回後、カメラで1回再確認
                # ------------------------------------------

                self.phase = self.REALIGN
                self.confirmed_frames = 0

                self.logger.info(
                    "Alignment settled; "
                    "checking bottle once "
                    "before gyro drive"
                )

            return Status.RUNNING

        # ==================================================
        # 150mm接近後の再確認
        # ==================================================

        if self.phase == self.REALIGN:

            # 停止状態で確認
            self._stop_motors()

            # --------------------------------------------------
            # 2回目の黒ボトル検知タイムアウト
            # --------------------------------------------------
            if (
                time.monotonic() - self.started_at
                >= self.BOTTLE_DETECT_TIMEOUT_SEC
            ):
                self.logger.warning(
                    "Second black bottle detection timed out "
                    "after %.1fs; "
                    "continuing with previous target bearing=%.1f"
                    % (
                        self.BOTTLE_DETECT_TIMEOUT_SEC,
                        self.target_bearing,
                    )
                )

                # 1回目の検知で確定した方位を使用して
                # 残り距離の走行へ進む
                self._start_gyro_approach()

                return Status.RUNNING

            (
                session,
                frame_id,
                observation,
            ) = runtime.video.get_bottle_observation()

            # 古いセッション・同一フレームは無視
            if (
                session != self.session
                or frame_id <= self.last_frame_id
            ):
                return Status.RUNNING

            self.last_frame_id = frame_id

            (
                insight,
                color,
                cx,
                theta,
                bottom_row,
                area,
                in_blind,
            ) = observation

            valid = (
                insight
                and color == BottleColor.BLACK
                and area
                >= self.settings.camera_min_area_px
            )

            if not valid:

                self.confirmed_frames = 0

                return Status.RUNNING

            self.confirmed_frames += 1

            self.logger.info(
                "SUMO REALIGN DETECT "
                "frame=%d cx=%s "
                "theta=%.1f area=%.1f "
                "bearing=%.1f"
                % (
                    frame_id,
                    cx,
                    theta,
                    area,
                    self._current_bearing(),
                )
            )

            # 複数フレームで確認
            if (
                self.confirmed_frames
                < self.settings.camera_confirm_frames
            ):
                return Status.RUNNING

            self.confirmed_frames = 0

            # ------------------------------------------
            # まだ正面からずれている場合
            # → 最大1回再補正
            # ------------------------------------------

            if (
                abs(theta)
                > self.settings.camera_alignment_tolerance_deg
                and self.realign_count
                < self.max_realign_count
            ):

                self.realign_count += 1

                self.target_bearing = (
                    self._estimated_bottle_bearing(
                        theta
                    )
                )

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
                    name=(
                        "realign to "
                        "camera bottle bearing"
                    ),
                    context=self.context,
                    bearing=self.target_bearing,
                    max_power=self.settings.turn_max_power,
                    min_power=self.settings.turn_min_power,
                    pid_p=self.settings.turn_pid_p,
                    pid_i=self.settings.turn_pid_i,
                    pid_d=self.settings.turn_pid_d,
                    tolerance=(
                        self.settings.heading_tolerance_deg
                    ),
                )

                self.phase = self.ALIGN

                return Status.RUNNING

            # ------------------------------------------
            # 正面に入った、または補正上限
            # → 残距離を走行
            # ------------------------------------------

            self.target_bearing = (
                self._estimated_bottle_bearing(
                    theta
                )
            )

            self.logger.info(
                "Bottle alignment confirmed "
                "theta=%.1f target=%.1f; "
                "starting distance drive"
                % (
                    theta,
                    self.target_bearing,
                )
            )

            self._start_gyro_approach()

            return Status.RUNNING

        # ==================================================
        # 最終残距離走行
        # ==================================================

        if self.phase == self.APPROACH:

            if (
                self.remaining_distance.update()
                == Status.SUCCESS
            ):

                if self.gyro_drive is not None:
                    self.gyro_drive.stop(
                        Status.INVALID
                    )

                self._stop_motors()

                self.logger.info(
                    "Capture and push distance "
                    "completed; proceeding to reverse"
                )

                return Status.SUCCESS

            # RunByGyroを毎制御周期実行
            if self.gyro_drive is not None:

                self.gyro_drive.tick_once()

                if (
                    self.gyro_drive.status
                    == Status.FAILURE
                ):
                    self._stop_motors()
                    return Status.FAILURE

            return Status.RUNNING

        # ==================================================
        # 初回黒ボトル探索
        # ==================================================

        now = time.monotonic()

        (
            session,
            frame_id,
            observation,
        ) = runtime.video.get_bottle_observation()

        # カメラモード切替前の結果、
        # 同一フレームは無視
        if (
            session != self.session
            or frame_id <= self.last_frame_id
        ):
            return Status.RUNNING

        self.last_frame_id = frame_id

        (
            insight,
            color,
            cx,
            theta,
            bottom_row,
            area,
            in_blind,
        ) = observation

        # 黒ボトル検知ログ
        if (
            insight
            and color == BottleColor.BLACK
        ):

            self.logger.info(
                "SUMO BLACK DETECT "
                "frame=%d cx=%s "
                "theta=%.1f bottom=%s "
                "area=%.1f blind=%s "
                "bearing=%.1f"
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
            and area
            >= self.settings.camera_min_area_px
        )

        # ==================================================
        # ACQUIRE
        # ==================================================

        if self.phase == self.ACQUIRE:

            # ------------------------------------------
            # 2秒間黒ボトルを確定できなかった
            # → 150mm前進して再探索
            #
            # 150mm前進は1回だけ
            # ------------------------------------------

            if (
                not self.search_advance_done
                and time.monotonic() - self.started_at >= self.BOTTLE_DETECT_TIMEOUT_SEC
            ):
                self._start_search_advance()
                return Status.RUNNING

            # ------------------------------------------
            # 黒ボトル連続検知
            # ------------------------------------------

            if valid:
                self.confirmed_frames += 1
            else:
                self.confirmed_frames = 0

            if (
                self.confirmed_frames
                < self.settings.camera_confirm_frames
            ):
                return Status.RUNNING

            # ======================================================
            # 退避ルートは黒ボトルの位置に関係なく常に➁を使用
            # ======================================================

            self.logger.info(
                "SUMO ESCAPE ROUTE fixed route=2"
            )

            # ==========================================
            # ボトル方位確定
            # ==========================================

            self.target_bearing = (
                self._estimated_bottle_bearing(
                    theta
                )
            )

            self.logger.info(
                "%+06d %s.black bottle confirmed "
                "frame=%d theta=%.1f "
                "area=%d "
                "target_bearing=%.1f; "
                "aligning before "
                "first 150mm approach"
                % (
                    runtime.plotter.get_distance(),
                    self.__class__.__name__,
                    frame_id,
                    theta,
                    area,
                    self.target_bearing,
                )
            )

            # ==========================================
            # 黒ボトル方向へその場旋回
            # ==========================================

            self.initial_alignment = True

            self.alignment_turn = SpinToBearing(
                name=(
                    "initial align to sumo bottle "
                    "before 150mm approach"
                ),
                context=self.context,
                bearing=self.target_bearing,
                max_power=self.settings.turn_max_power,
                min_power=self.settings.turn_min_power,
                pid_p=self.settings.turn_pid_p,
                pid_i=self.settings.turn_pid_i,
                pid_d=self.settings.turn_pid_d,
                tolerance=(
                    self.settings.heading_tolerance_deg
                ),
            )

            self.phase = self.ALIGN

            return Status.RUNNING

        return Status.RUNNING

    # ======================================================
    # 終了処理
    # ======================================================

    def terminate(self, new_status):

        if (
            getattr(
                self,
                "alignment_turn",
                None,
            )
            is not None
        ):
            self.alignment_turn.stop(
                Status.INVALID
            )

        if (
            self.search_advance_drive
            is not None
        ):
            self.search_advance_drive.stop(
                Status.INVALID
            )

        if self.gyro_drive is not None:
            self.gyro_drive.stop(
                Status.INVALID
            )

        # 成功・失敗・中断のいずれでも停止
        if (
            runtime.right_motor is not None
            and runtime.left_motor is not None
        ):
            self._stop_motors()


# ==========================================================
# 相撲ボトル捕捉完了
# ==========================================================

class MarkCameraCaptureCompleted(Behaviour):

    def __init__(
        self,
        name,
        context,
    ):
        super().__init__(name)

        self.context = context

    def update(self):

        self.context.sumo.bottle_captured = True
        self.context.sumo.bottle_pushed_out = True

        return Status.SUCCESS


# ==========================================================
# スキップ完了
# ==========================================================

class CompleteSkippedCameraCapture(Behaviour):

    def __init__(
        self,
        name,
        context,
    ):
        super().__init__(name)

        self.context = context

    def update(self):

        if self.context.sumo.skipped:
            return Status.SUCCESS

        return Status.FAILURE


# ==========================================================
# Behaviour Tree生成
# ==========================================================

def build_capture_sumo_bottle_camera(
    context,
    config,
):
    """
    No.16・17

    カメラで黒ボトルを検知し、
    ボトル方向へ接近して押し出す。

    総前進距離:
        通常
            150mm + 350mm = 500mm

        2秒未検知時
            150mm探索前進
            + 150mm接近
            + 200mm
            = 500mm
    """

    capture = Sequence(
        name="capture sumo bottle by camera",
        memory=True,
    )

    capture.add_children(
        [
            CaptureSumoBottleWithCamera(
                name=(
                    "locate and approach "
                    "sumo bottle by camera"
                ),
                context=context,
                settings=config.sumo,
            ),

            StopNow(
                name=(
                    "stop after total "
                    "sumo capture and push distance"
                )
            ),

            MarkCameraCaptureCompleted(
                name=(
                    "mark camera "
                    "sumo capture completed"
                ),
                context=context,
            ),
        ]
    )

    return capture