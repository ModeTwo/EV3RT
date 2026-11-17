"""No floor calibration: measured relative alignment + explicit push distance.

今回の標準実行。規定接近・画像正対は親へ任せ、正対後の捕捉→戻り→押しを状態別メソッドで進める。NC_は較正JSONなし方式の記録名で、既存ログ互換のため維持する。"""

import math
import time
from py_trees.common import Status
from py_trees.behaviour import Behaviour
from simple_pid import PID
from ..runtime import runtime
from ..behaviours.gyro_drive import RunByGyro
from ..timing import CONTROL_INTERVAL_SEC
from .catch_sumo_bottle_test import AlignAndCatchTest
from .align_sumo_bottle import AlignSumoBottle
from .sumo_bearing_motion import current_bearing, SpinToBearing
from .sumo_push_geometry import alignment_push_policy, CaptureEvidence, delta
from .sumo_push_route import BoundedRunByGyro
from ..types import HeadingType

from .sumo_relative_push_settings import RelativePushSettings


class BoundedBackoffRunByGyro(RunByGyro):
    """後退時も左右出力が-100..100を超えないようPID補正を制限する。"""

    def update(self):
        if not self.running:
            heading = -runtime.course * float(runtime.gyro_sensor.get_angle())
            self.target_heading = heading + delta(self.target, heading)
            limit = min(abs(self.power), 100 - abs(self.power))
            self.pid = PID(self.pid_p, self.pid_i, self.pid_d,
                           setpoint=self.target_heading,
                           sample_time=CONTROL_INTERVAL_SEC,
                           output_limits=(-limit, limit))
            self.running = True
        return super().update()


class ContinuousReturnTurn(SpinToBearing):
    """成功時は出力を残して同一周期に押し直進へ渡す。中断時は通常停止。"""

    def terminate(self, new_status):
        if new_status == Status.SUCCESS:
            self.running = False
        else:
            super().terminate(new_status)


class ForwardReturnArc(Behaviour):
    """両輪50以上の前進で方位を戻し、同じ周期に押しへ渡す。"""

    MIN_POWER = 50
    MAX_POWER = 75
    TOLERANCE_DEG = 2.0
    MAX_DISTANCE_MM = 250.0
    TIMEOUT_SEC = 3.0

    def __init__(self, name, owner, bearing):
        super().__init__(name)
        self.owner = owner
        self.target_bearing = float(bearing) % 360.0

    def initialise(self):
        self.started_at = time.monotonic()
        self.origin = self.owner._encoder_pair()
        self.initial_error = delta(
            self.target_bearing, current_bearing(self.owner.context)
        )
        self.last_log_at = None
        self.logger.info(
            "SUMO_RETURN_ARC START target=%.2f error=%.2f "
            "power=50..75 max_distance=250.0"
            % (self.target_bearing, self.initial_error)
        )

    def _fail(self, reason):
        self.logger.error("SUMO_RETURN_ARC FAILURE: " + reason)
        return Status.FAILURE

    def update(self):
        now = time.monotonic()
        bearing = current_bearing(self.owner.context)
        distance = self.owner._forward_mm_from(self.origin)
        error = delta(self.target_bearing, bearing)
        if not all(math.isfinite(v) for v in (
            now, bearing, distance, error, self.initial_error
        )):
            return self._fail("invalid gyro/encoder")

        # カーブ中も押し出し終了位置を越えて走り続けない。
        if self.owner._fixed_enabled():
            self.owner._fixed_sample()
            if self.owner._fixed_remaining() <= 0.0:
                self.logger.info(
                    "SUMO_RETURN_ARC ENDPOINT_REACHED distance=%.1f error=%.2f"
                    % (distance, error)
                )
                return Status.SUCCESS

        crossed = self.initial_error * error < 0.0
        if abs(error) <= self.TOLERANCE_DEG or crossed:
            self.logger.info(
                "SUMO_RETURN_ARC COMPLETE bearing=%.2f error=%.2f distance=%.1f"
                % (bearing, error, distance)
            )
            return Status.SUCCESS

        elapsed = now - self.started_at
        if distance < -2.0:
            return self._fail("reverse distance bound")
        if elapsed > 0.6 and distance < 2.0:
            return self._fail("no forward progress")
        if distance >= self.MAX_DISTANCE_MM or elapsed > self.TIMEOUT_SEC:
            reason = "distance" if distance >= self.MAX_DISTANCE_MM else "time"
            self.logger.info(
                "SUMO_RETURN_ARC LIMIT_HANDOFF reason=%s distance=%.1f "
                "bearing=%.2f remaining_error=%.2f aligned=False"
                % (reason, distance, bearing, error)
            )
            # 補正を終了し、現在の方位で押し出しへ渡す。
            return Status.SUCCESS

        extra = int(round(
            min(25.0, max(12.0, 1.5 * abs(error)))
        ))
        outer = min(self.MAX_POWER, self.MIN_POWER + extra)

        # 床面方位の正方向は時計回り。コースでは反転しない。
        left, right = (
            (outer, self.MIN_POWER) if error > 0.0
            else (self.MIN_POWER, outer)
        )
        runtime.left_motor.set_power(left)
        runtime.right_motor.set_power(right)

        if self.last_log_at is None or now - self.last_log_at >= 0.2:
            self.logger.info(
                "SUMO_RETURN_ARC DRIVE error=%.2f distance=%.1f left=%d right=%d"
                % (error, distance, left, right)
            )
            self.last_log_at = now
        return Status.RUNNING

    def terminate(self, new_status):
        if new_status == Status.SUCCESS:
            return  # 同一周期で押し出し、または後退開始へ渡す。
        for motor in (runtime.left_motor, runtime.right_motor):
            if motor is not None:
                motor.set_power(0)
                motor.set_brake(True)


class RelativeAlignAndPush(AlignAndCatchTest):
    """較正なし相対方式。正対までは親、正対後はこのクラスが担当する。

    NC_CAPTURE: 候補接近 / NC_CAPTURE_SETTLE: 停止中の捕捉確認。
    NC_TURN: 必要な戻り旋回 / NC_TURN_SETTLE: 戻り後の停止確認。
    NC_PUSH: 押し出し / NC_BACK*: 押し・接近区間を逆順に戻る。
    FINALIZING: 親の録画終了処理。状態名・ログ形式は従来互換。"""

    def __init__(
        self,
        name,
        context,
        settings,
        motion_settings,
        catch_settings,
        push_settings,
        execute,
    ):
        if execute and push_settings.push_distance_mm is None:
            raise ValueError(
                "no-calibration execution requires --sumo-push-distance-mm"
            )
        super().__init__(name, context, settings, motion_settings, catch_settings)
        self.push_settings = push_settings
        self.execute = execute

    def _fail(self, reason):
        recoverable = (
            "alignment turn failed",
            "alignment turn timeout",
            "alignment turn limit reached without alignment",
            "fine alignment timeout; approach inhibited",
        )
        if (
            reason in recoverable
            and not self.catch_started
            and self.phase != "FINALIZING"
        ):
            return self._continue_after_alignment_failure(reason)
        self._black_fine_started = None
        return super()._fail(reason)

    def _continue_after_alignment_failure(self, reason):
        self._stop_child("turn")
        self._stop_child("drive")
        self._brake()
        self._black_fine_started = None

        if (
            self.selected_track_id is None
            or self.selected_capture_at is None
            or self.selected_frame_id is None
        ):
            return super()._fail(
                "missing bottle evidence for alignment continuation"
            )

        bearing = current_bearing(self.context)
        gyro = float(runtime.gyro_sensor.get_angle())
        if not all(math.isfinite(v) for v in (bearing, gyro)):
            return super()._fail(
                "invalid alignment continuation pose"
            )

        self.context.sumo.bottle_aligned = False
        self.observation_decision = "alignment-incomplete-continue"
        self.catch_started = True
        self.catch_track = self.selected_track_id
        self.logger.info(
            "SUMO_ALIGN CONTINUE_UNALIGNED reason=%s bearing=%.2f track=%s"
            % (reason, bearing, self.catch_track)
        )
        return self._start_catch_drive()

    def initialise(self):
        """実行ごとの状態を初期化する。機器登録・タッチ開始はalphaが担当する。"""
        self._black_fine_started = None
        self._black_fine_track = None
        self.nc_samples = []
        self.fixed_pose = None
        self.fixed_push_end = None
        self.angle_policy = None
        self.context.sumo.rally_turn_reference = None
        self.approach_bearing = None
        self.capture_assumed = False
        self.latest_raw = None
        self.next_sensor_at = 0.0
        self.push_start_bearing = None
        self.back_heading = None
        self.back_progress_mm = None
        self.capture_leg_mm = None
        self.push_leg_mm = None
        self.capture_turn_performed = False
        self.context.sumo.rally_avoidance_performed = None
        self.capture_candidate_at = None
        self.capture_candidate_progress = None
        self.nc_evidence = CaptureEvidence(self.push_settings)
        self.nc_outcome = None
        runtime.require("sonar_sensor")
        super().initialise()

    def _fixed_enabled(self):
        return bool(getattr(self, "push_endpoint_enabled", False))

    def _fixed_begin(self):
        if not self._fixed_enabled():
            return
        bearing = current_bearing(self.context)
        self.fixed_pose = (0.0, 0.0, self._encoder_pair(), bearing)
        self.fixed_push_end = None
        self.logger.info(
            "SUMO_FIXED ORIGIN approach=%.2f circle_far=1590.0 "
            "green=1895.0 bottle_offset=115.0 "
            "bottle_radius=32.5 clearance=300.0"
            % self.approach_bearing
        )

    def _fixed_sample(self):
        pose = getattr(self, "fixed_pose", None)
        if not self._fixed_enabled() or pose is None:
            return
        x, y, previous, old_bearing = pose
        counts = self._encoder_pair()
        bearing = current_bearing(self.context)
        diameter = float(runtime.plotter.tire_diameter)
        if (
            not math.isfinite(diameter) or diameter <= 0
            or not math.isfinite(bearing)
        ):
            raise ValueError("invalid fixed-endpoint sensors")

        distance = (
            math.pi * diameter
            * sum(c - p for c, p in zip(counts, previous)) / 720.0
        )
        if not math.isfinite(distance) or abs(distance) > 100.0:
            raise ValueError("fixed-endpoint encoder discontinuity")

        mid = old_bearing + delta(bearing, old_bearing) / 2.0
        angle = math.radians(delta(mid, self.approach_bearing))
        self.fixed_pose = (
            x + distance * math.cos(angle),
            y + distance * math.sin(angle),
            counts, bearing,
        )
        # RALLY_AXLE_POSE_EXPORT
        reference = getattr(self.context.sumo, "rally_turn_reference", None)
        if isinstance(reference, dict) and reference.get("course") == runtime.course:
            reference["axle_pose"] = dict(
                x=self.fixed_pose[0], y=self.fixed_pose[1],
                left=counts[0], right=counts[1], bearing=bearing,
            )

    def _fixed_remaining(self):
        pose = getattr(self, "fixed_pose", None)
        if pose is None:
            raise ValueError("fixed-endpoint origin missing")
        cosine = math.cos(math.radians(
            delta(pose[3], self.approach_bearing)
        ))
        if cosine < 0.5:
            raise ValueError("push heading outside endpoint direction bound")

        # ボトル後端を円の奥縁から200mm外へ出す。
        rear = pose[0] + 115.0 * cosine - 32.5
        remaining = (1590.0 + 300.0 - rear) / cosine
        if remaining > 1500.0:
            raise ValueError("fixed-endpoint remaining distance too large")
        return remaining

    def _start_drive(self, now):
        # 最初の直進で指示する絶対目標を床面方位に変換して保持する。
        # 開始時の実姿勢ではなく、直進に使用する目標そのものを基準にする。
        # 配送後の接近方位は開始時にコースの符号をそろえる。
        # bottle-sumo: LEFTの右側、RIGHTの左側は共に床面方位180度。
        # BOTTLE_APPROACH_OFFSET_FIXED
        if self.continuous_entry and getattr(self, "delivery_green_on_right", False):
            if runtime.course not in (-1, 1):
                raise ValueError("unsupported course for bottle approach")
            self.approach_offset_from_reset_deg = -90.0 * runtime.course
            self.logger.info(
                "SUMO_ALIGN APPROACH_REFERENCE course=%s raw_offset=%.1f"
                % (runtime.course, self.approach_offset_from_reset_deg)
            )
        if getattr(self, "delivery_green_on_right", False):
            from dataclasses import replace
            self.push_settings = replace(
                self.push_settings, green_bearing_deg=180.0
            )
        ref = self.context.sumo.bearing_reference
        if ref.reset_gyro_deg is None:
            raise RuntimeError("sumo gyro reference is not registered")
        self.approach_bearing = ref.bearing(
            ref.reset_gyro_deg + self.approach_offset_from_reset_deg
        )
        self._fixed_begin()
        super()._start_drive(now)

    def _start_catch_drive(self):
        """親の正対完了フック。押す向きを決め、捕捉接近を開始する。"""
        self._black_fine_started = None
        self.angle_policy = alignment_push_policy(
            self.push_settings, self.approach_bearing, current_bearing(self.context)
        )
        self.context.sumo.rally_turn_reference = {
            "course": runtime.course,
            "approach_bearing": self.approach_bearing,
            "bottle_delta": self.angle_policy["actual_alignment_delta_deg"],
        }
        self.logger.info("SUMO_RELATIVE ANGLE " + str(self.angle_policy))
        if not self.execute:
            return AlignSumoBottle._finish_recording(
                self, Status.SUCCESS, "no-calibration-shadow-angle-recorded"
            )
        super()._start_catch_drive()
        if self.phase != "CATCH_DRIVE":
            return Status.RUNNING
        self.phase = "NC_CAPTURE"
        self.nc_heading = current_bearing(self.context)
        self.aligned_heading = self.nc_heading
        self.capture_encoder_origin = self._encoder_pair()
        if self.catch_track is None or self.selected_capture_at is None:
            return self._nc_fail("missing capture camera handoff")
        self.capture_camera_heading = -runtime.course * self.catch_heading
        self.capture_camera_last_at = self.selected_capture_at
        self.capture_camera_next_at = 0.0
        self.catch_last_frame = self.selected_frame_id
        self.drive.target = self._capture_camera_target
        return Status.RUNNING

    def _on_alignment_turn_limit(self, now, observation, target):
        return self._continue_after_alignment_failure(
            "alignment coarse-turn limit"
        )


    def _black_fine_tick(self, now):
        # 失敗後は既存の録画終了処理へ進める。
        if self.phase not in ("OBSERVE", "SETTLE", "BLACK_FINE_PULSE"):
            self._black_fine_started = None
            return None
        try:
            if now - self._black_fine_started >= 3.0:
                return self._fail(
                    "fine alignment timeout; approach inhibited"
                )
            if self.phase != "BLACK_FINE_PULSE":
                return None

            bearing = current_bearing(self.context)
            if not math.isfinite(bearing):
                return self._fail("invalid fine alignment gyro")
            old_t, old_b = self._black_fine_previous
            dt = now - old_t
            velocity = (
                delta(bearing, old_b) / dt if dt > 0.0001 else 0.0
            )
            self._black_fine_previous = (now, bearing)
            remaining = delta(self._black_fine_target, bearing)
            toward = max(0.0, velocity * self._black_fine_sign)
            allowance = min(2.0, max(0.5, toward * 0.025))
            if (
                remaining * self._black_fine_sign <= allowance
                or now - self._black_fine_pulse_at >= 0.150
            ):
                self.logger.info(
                    "SUMO_ALIGN FINE_GYRO_BRAKE remaining=%.2f velocity=%.1f"
                    % (remaining, velocity)
                )
                self._settle(now, after_turn=True)
            return Status.RUNNING
        except Exception as exc:
            return self._fail("fine alignment error: %s" % exc)

    def _update_bottle_observation(self, now):
        if self._black_fine_started is not None:
            obs = runtime.video.get_sumo_alignment_observation(
                self.session
            )
            if (
                obs
                and obs.get("confirmed")
                and obs.get("candidate")
                and obs.get("session") == self.session
                and obs.get("t", -1) >= self.observations_after
                and 0 <= now - obs["t"]
                <= self.settings.max_observation_age_sec
                and obs.get("track_id") != self._black_fine_track
            ):
                return self._fail(
                    "bottle identity changed during fine alignment"
                )
        return super()._update_bottle_observation(now)


    def _capture_camera_target(self, distance_mm):
        # Return an equivalent angle near the drive's unwrapped heading.
        current = self.drive.continuous_heading
        return current + delta(self.capture_camera_heading, current)

    def _update_capture_camera(self):
        now = time.monotonic()
        settings = self.push_settings
        observation = runtime.video.get_sumo_alignment_observation(self.session)
        if (
            observation is not None
            and now >= self.capture_camera_next_at
        ):
            try:
                frame = observation["frame_id"]
                t = float(observation["t"])
                gyro = float(observation["gyro_deg"])
                cx = float(observation["candidate"]["cx"])
                valid = (
                    observation.get("session") == self.session
                    and observation.get("confirmed")
                    and self.catch_track is not None
                    and observation.get("track_id") == self.catch_track
                    and frame > self.catch_last_frame
                    and all(math.isfinite(v) for v in (t, gyro, cx))
                    and 0 <= cx < 320
                    and 0 <= now - t <= settings.capture_camera_max_age_sec
                    and t > self.capture_camera_last_at
                )
            except (KeyError, TypeError, ValueError):
                valid = False
            if valid:
                current_gyro = float(runtime.gyro_sensor.get_angle())
                if not math.isfinite(current_gyro):
                    return self._nc_fail("invalid capture camera control gyro")
                # Correct capture-time pixel error for rotation since capture.
                error = (
                    (cx - self.settings.target_cx)
                    / self.settings.visual_px_per_degree
                    - delta(current_gyro, gyro)
                )
                limit = settings.capture_camera_max_correction_deg
                correction = max(-limit, min(limit, error))
                desired = -runtime.course * (current_gyro + correction)
                self.capture_camera_heading += (
                    settings.capture_camera_smoothing_alpha
                    * delta(desired, self.capture_camera_heading)
                )
                self.capture_camera_last_at = t
                self.capture_camera_hold_logged = False
                self.capture_camera_next_at = (
                    now + settings.capture_camera_update_interval_sec
                )
                self.catch_last_frame = frame
                self.catch_error = cx - self.settings.target_cx
                self.catch_age = now - t
                self.logger.info(
                    "SUMO_RELATIVE CAPTURE_CAMERA frame=%s error=%.2f "
                    "correction=%.2f target=%.2f"
                    % (frame, error, correction, self.capture_camera_heading)
                )
        # 画像を失っても最後の目標方位で接近を継続する。
        # 接近距離上限と超音波判定は既存処理で維持する。
        age = now - self.capture_camera_last_at
        if not math.isfinite(age) or age < 0:
            return self._nc_fail("invalid capture camera timestamp")
        if age > settings.capture_camera_loss_timeout_sec:
            if not getattr(self, "capture_camera_hold_logged", False):
                self.logger.info(
                    "SUMO_RELATIVE CAPTURE_CAMERA_LOST_HOLD_HEADING "
                    "age=%.3f target=%.2f"
                    % (age, self.capture_camera_heading)
                )
                self.capture_camera_hold_logged = True
        return None

    def _encoder_pair(self):
        left = float(runtime.left_motor.get_count())
        right = float(runtime.right_motor.get_count())
        if not all(map(math.isfinite, (left, right))):
            raise ValueError("invalid wheel encoder")
        return left, right

    def _forward_mm_from(self, origin):
        left, right = self._encoder_pair()
        diameter = float(runtime.plotter.tire_diameter)
        if not math.isfinite(diameter) or diameter <= 0:
            raise ValueError("invalid tire diameter")
        return math.pi * diameter * ((left - origin[0]) + (right - origin[1])) / 720.0

    def _record_snapshot(self):
        """動画フレームと対応付ける現在値。ここで新しく測距しない。"""
        row = super()._record_snapshot()
        row.update(
            route_sonar_raw=self.latest_raw,
            route_capture_assumed=self.capture_assumed,
            route_push_start_bearing_deg=self.push_start_bearing,
            route_angle_decision=(
                self.angle_policy["decision"] if self.angle_policy else None
            ),
            route_alignment_delta_deg=(
                self.angle_policy["actual_alignment_delta_deg"]
                if self.angle_policy
                else None
            ),
            route_policy_bearing_deg=(
                self.angle_policy["push_bearing_deg"] if self.angle_policy else None
            ),
        )
        pose = getattr(self, "fixed_pose", None)
        if self._fixed_enabled() and pose is not None:
            row.update(fixed_x_mm=pose[0], fixed_y_mm=pose[1])
        return row

    def _recording_details(self):
        """終了summaryへ設定・判断・制御標本をまとめる。物理成功と推定を区別する。"""
        result = super()._recording_details()
        result.update(
            kind="sumo-relative-push-v1",
            route=dict(
                calibration_required=False,
                geometry_stop_enabled=False,
                pose_method="existing Plotter distance and gyro; no floor pose",
                angle_policy=self.angle_policy,
                execute=self.execute,
                capture_assumed=self.capture_assumed,
                physical_capture_confirmed=False,
                physical_push_out_confirmed=False,
                capture_candidate_at=self.capture_candidate_at,
                capture_candidate_progress_mm=self.capture_candidate_progress,
                requested_push_distance_mm=self.push_settings.push_distance_mm,
                actual_push_distance_mm=getattr(
                    self, "actual_push_distance_mm", None
                ),
                total_forward_target_mm=800.0,
                power=self.push_settings.power,
                capture_max_mm=self.push_settings.capture_max_mm,
                turn_power=self.push_settings.turn_power,
                post_return_correction=False,
                return_angle_gate=False,
                push_heading_origin="actual_bearing_at_handoff",
                push_start_bearing_deg=self.push_start_bearing,
                back_heading_deg=getattr(self, "back_heading", None),
                back_progress_mm=getattr(self, "back_progress_mm", None),
                capture_leg_mm=self.capture_leg_mm,
                push_leg_mm=self.push_leg_mm,
                aligned_heading_deg=getattr(self, "aligned_heading", None),
                capture_turn_performed=self.capture_turn_performed,
                outcome=self.nc_outcome,
                samples=list(self.nc_samples),
            ),
        )
        if self._fixed_enabled():
            result["route"].update(
                geometry_stop_enabled=True,
                pose_method="signed encoders and midpoint gyro from approach start",
                total_forward_target_mm=None,
                fixed_circle_far_mm=1590.0,
                fixed_green_boundary_mm=1895.0,
                fixed_bottle_offset_mm=115.0,
                fixed_bottle_radius_mm=32.5,
                fixed_clearance_mm=300.0,
                fixed_push_end_xy_mm=self.fixed_push_end,
            )
        return result

    def _nc_fail(self, reason):
        """正対後の中止理由を記録し、親の制動・録画終了処理へ渡す。"""
        self.nc_outcome = "aborted: " + reason
        return self._fail(reason)

    def _hold(self, phase, now):
        """子の出力を止めて制動し、停止後のセンサー確認を新しく始める。"""
        self._stop_child("drive")
        self._stop_child("turn")
        self._brake()
        self.phase = phase
        self.phase_started_at = now
        self.nc_evidence = CaptureEvidence(self.push_settings)

    def _push(self, now):
        """押し直進を開始する。目標は引継ぎ時の実際の方位。"""
        self.phase = "NC_PUSH"
        # 有効時は開始位置基準。他の実行方法は従来の距離方式。
        if self._fixed_enabled():
            self._fixed_sample()
            self.actual_push_distance_mm = max(
                0.0, self._fixed_remaining()
            )
            self.logger.info(
                "SUMO_FIXED PLAN x=%.1f y=%.1f remaining=%.1f bearing=%.2f"
                % (
                    self.fixed_pose[0], self.fixed_pose[1],
                    self.actual_push_distance_mm, self.fixed_pose[3],
                )
            )
        else:
            self.actual_push_distance_mm = max(
                0.0, 800.0 - self.capture_leg_mm
            )

        self.logger.info(
            "SUMO_RELATIVE PUSH_DISTANCE "
            "capture=%.1f target=%.1f total=%.1f"
            % ( self.capture_leg_mm,
                self.actual_push_distance_mm,
                self.capture_leg_mm + self.actual_push_distance_mm,
            )) 
        self.phase_started_at = now
        self.catch_origin = float(runtime.plotter.get_distance())
        self.push_encoder_origin = self._encoder_pair()
        self.progress_anchor = 0.0
        self.progress_at = now
        self.nc_heading = current_bearing(self.context)
        self.push_start_bearing = self.nc_heading
        self.logger.info(
            "SUMO_RELATIVE PUSH_ACTUAL_BEARING planned=%.3f actual=%.3f difference=%.3f no-post-return-correction"
            % (
                self.angle_policy["push_bearing_deg"],
                self.nc_heading,
                delta(self.nc_heading, self.angle_policy["push_bearing_deg"]),
            )
        )
        target = self.context.sumo.bearing_reference.legacy_target(
            self.nc_heading, runtime.gyro_sensor.get_angle(), runtime.course
        )
        self.drive = BoundedRunByGyro(
            "relative fixed-distance push",
            target=target,
            target_type=HeadingType.ABSOLUTE,
            power=self.push_settings.power,
            pid_p=self.motion_settings.drive_pid_p,
            pid_i=self.motion_settings.drive_pid_i,
            pid_d=self.motion_settings.drive_pid_d,
        )
        return Status.RUNNING

    def _start_backoff(self, now):
        """押し出し完了後、同じ方位で150mm後退する。"""
        if self._fixed_enabled():
            self._fixed_sample()
            self.fixed_push_end = self.fixed_pose[:2]
            cosine = math.cos(math.radians(
                delta(self.fixed_pose[3], self.approach_bearing)
            ))
            rear = self.fixed_pose[0] + 115.0 * cosine - 32.5
            self.logger.info(
                "SUMO_FIXED END x=%.1f y=%.1f estimated_bottle_rear=%.1f "
                "circle_far=1590.0 green=1895.0 physical_success=unverified"
                % (self.fixed_pose[0], self.fixed_pose[1], rear)
            )
        self._hold("NC_BACK", now)
        self.push_leg_mm = max(0.0, self._forward_mm_from(self.push_encoder_origin))

        # 押し出し距離に関係なく150mm後退する
        self.back_target_mm = 120.0
        self.back_left_origin = float(runtime.left_motor.get_count())
        self.back_right_origin = float(runtime.right_motor.get_count())
        self.back_heading = current_bearing(self.context)
        self.back_origin_distance = float(runtime.plotter.get_distance())
        self.back_progress_anchor = 0.0
        self.back_progress_mm = 0.0
        self.back_progress_at = now
        target = self.context.sumo.bearing_reference.legacy_target(
            self.back_heading, runtime.gyro_sensor.get_angle(), runtime.course
        )
        self.drive = BoundedBackoffRunByGyro(
            name="sumo reverse after push", target=target,
            target_type=HeadingType.ABSOLUTE, power=-75,
            pid_p=self.motion_settings.drive_pid_p,
            pid_i=self.motion_settings.drive_pid_i,
            pid_d=self.motion_settings.drive_pid_d,
        )
        self.logger.info("SUMO_RELATIVE BACK_START segment=push distance=%.1f power=-75 bearing=%.2f" % (self.back_target_mm, self.back_heading))
        return Status.RUNNING

    def _update_backoff(self, now):
        left = float(runtime.left_motor.get_count()) - self.back_left_origin
        right = float(runtime.right_motor.get_count()) - self.back_right_origin
        diameter = float(runtime.plotter.tire_diameter)
        if not all(map(math.isfinite, (left, right, diameter))) or diameter <= 0:
            return self._nc_fail("invalid backoff encoder")
        progress = -math.pi * diameter * (left + right) / 720.0
        self.back_progress_mm = progress
        if progress >= self.back_target_mm:
            self.logger.info("SUMO_RELATIVE BACK_SEGMENT_COMPLETE phase=%s progress=%.1f target=%.1f" % (self.phase, progress, self.back_target_mm))
            # 押し出し分の後退が完了したら、そのまま停止へ進む。
            self.logger.info("SUMO_RELATIVE BACK_COMPLETE progress=%.1f" % progress)
            if getattr(self, "immediate_rally_handoff", False):
                self.context.sumo.rally_avoidance_performed = (
                    self.capture_turn_performed
                )
                self.nc_outcome = "push-and-backoff-complete"
                # Request asynchronous recording close, then hand off now.
                result = AlignSumoBottle._finish_recording(
                    self, Status.SUCCESS, self.nc_outcome
                )
                if result == Status.RUNNING:
                    return Status.SUCCESS
                return result
            self._hold("NC_DONE", now)
            return Status.RUNNING
        if progress - self.back_progress_anchor >= self.catch_settings.stall_progress_mm:
            self.back_progress_anchor, self.back_progress_at = progress, now
        elif now - self.back_progress_at > self.catch_settings.stall_timeout_sec:
            return self._nc_fail("no backward progress")
        for motor in (runtime.left_motor, runtime.right_motor):
            motor.set_brake(False)
        self.drive.tick_once()
        if self.drive.status == Status.FAILURE:
            return self._nc_fail("backward gyro drive failed")
        return Status.RUNNING

    def _start_alignment_return_turn(self, now):
        """押し区間を戻った後、捕捉時に旋回していた場合は元の向きへ戻す。"""
        self._hold("NC_BACK_TURN", now)
        if not self.capture_turn_performed:
            return self._start_approach_backoff(now)
        self.turn = SpinToBearing(
            "sumo return to aligned heading", self.context, self.aligned_heading,
            max_power=self.push_settings.turn_power, min_power=self.push_settings.turn_power,
            pid_p=self.motion_settings.turn_pid_p, pid_i=self.motion_settings.turn_pid_i,
            pid_d=self.motion_settings.turn_pid_d, tolerance=1.0,
        )
        self.logger.info("SUMO_RELATIVE BACK_TURN target=%.2f" % self.aligned_heading)
        return Status.RUNNING

    def _start_approach_backoff(self, now):
        self._hold("NC_BACK_APPROACH", now)
        self.back_left_origin, self.back_right_origin = self._encoder_pair()
        self.back_target_mm = max(0.0, self.capture_leg_mm)
        self.back_progress_anchor = 0.0
        self.back_progress_at = now
        self.back_heading = current_bearing(self.context)
        target = self.context.sumo.bearing_reference.legacy_target(
            self.aligned_heading, runtime.gyro_sensor.get_angle(), runtime.course
        )
        self.drive = BoundedBackoffRunByGyro(
            name="sumo reverse to alignment position", target=target,
            target_type=HeadingType.ABSOLUTE, power=-80,
            pid_p=self.motion_settings.drive_pid_p, pid_i=self.motion_settings.drive_pid_i,
            pid_d=self.motion_settings.drive_pid_d,
        )
        self.logger.info("SUMO_RELATIVE BACK_START segment=approach distance=%.1f bearing=%.2f" % (self.back_target_mm, self.aligned_heading))
        return Status.RUNNING

    def _update_alignment_return_turn(self, now):
        self.turn.tick_once()
        if self.turn.status == Status.FAILURE:
            return self._nc_fail("alignment return turn failed")
        if self.turn.status == Status.SUCCESS:
            return self._start_approach_backoff(now)
        return Status.RUNNING

    def update(self):
        """制御周期1回分。計測・終了優先判定・状態別処理・前進の順に進める。"""
        if self._black_fine_started is not None:
            result = self._black_fine_tick(time.monotonic())
            if result is not None:
                return result
        try:
            self._fixed_sample()
        except (ValueError, TypeError, OverflowError) as error:
            return self._nc_fail("fixed-endpoint odometry: " + str(error))
        if not self.phase.startswith("NC_"):
            return super().update()
        push_settings = self.push_settings
        now = time.monotonic()
        try:
            recording = runtime.video.get_sumo_alignment_recording_status()
            if not recording or recording["error"]:
                return self._nc_fail("recording failed")
            if now - self.started_at > push_settings.overall_timeout_sec:
                return self._nc_fail("relative mission timeout")
            if now - self.phase_started_at > push_settings.phase_timeout_sec:
                return self._nc_fail("relative phase timeout")
            distance = float(runtime.plotter.get_distance())
            bearing = current_bearing(self.context)
            if not all(math.isfinite(x) for x in (distance, bearing)):
                return self._nc_fail("invalid distance/gyro")
            progress = distance - self.catch_origin
            self.catch_progress = progress
            # Distance first: do not read/command another forward tick at the endpoint.
            if self.phase == "NC_CAPTURE" and progress >= push_settings.capture_max_mm:
                return self._push_without_capture(now, progress)
            push_done = False
            if self.phase == "NC_PUSH":
                push_done = (
                    self._fixed_remaining() <= 0.0
                    if self._fixed_enabled()
                    else progress >= self.actual_push_distance_mm
                )
            if push_done:
                return self._start_backoff(now)
            # None: この周期は未読取 / False: 範囲外 / True: 一回で成立。
            evidence = None
            # 捕捉が成立した後は距離センサーを制御条件に使わない。
            if self.phase in ("NC_CAPTURE", "NC_CAPTURE_SETTLE") and now >= self.next_sensor_at:
                self.next_sensor_at = now + push_settings.sensor_interval_sec
                raw = runtime.sonar_sensor.get_distance()
                if (
                    not isinstance(raw, bool)
                    and isinstance(raw, (int, float))
                    and math.isfinite(raw)
                ):
                    self.latest_raw = raw
                else:
                    self.latest_raw = None
                # 一度の有効な測距を捕捉成立として扱う。無効・範囲外は成立しない。
                evidence = self.latest_raw is not None and 0 < self.latest_raw <= push_settings.capture_raw_max
                distance = float(runtime.plotter.get_distance())
                if not math.isfinite(distance):
                    return self._nc_fail("invalid post-read distance")
                progress = distance - self.catch_origin
                if (
                    self.phase == "NC_CAPTURE"
                    and progress >= push_settings.capture_max_mm
                ):
                    if evidence:
                        return self._update_capture_approach(now, progress, True)
                    return self._push_without_capture(now, progress)
            self.nc_samples.append(
                dict(
                    t=now,
                    phase=self.phase,
                    distance_mm=distance,
                    progress_mm=progress,
                    bearing_deg=bearing,
                    sonar_raw=self.latest_raw,
                )
            )
            phase_result = self._update_post_alignment_phase(
                now, progress, bearing, evidence
            )
            if phase_result is not None:
                return phase_result
            return self._drive_forward(now, progress, bearing)
        except Exception as error:
            return self._nc_fail("relative push IO failure: " + str(error))

    def _update_post_alignment_phase(self, now, progress, bearing, evidence):
        """正対後の処理順の入口。Noneは、この周期に共通直進処理へ進むことを表す。"""
        if self.phase == "NC_CAPTURE":
            return self._update_capture_approach(now, progress, evidence)
        if self.phase in ("NC_CAPTURE_SETTLE", "NC_TURN_SETTLE"):
            return self._update_stationary_confirmation(
                now, bearing, progress, evidence
            )
        if self.phase == "NC_TURN":
            return self._update_return_turn(now)
        if self.phase == "NC_PUSH":
            return None  # 押し直進の共通処理に進む。
        if self.phase in ("NC_BACK", "NC_BACK_APPROACH"):
            return self._update_backoff(now)
        if self.phase == "NC_BACK_TURN":
            return self._update_alignment_return_turn(now)
        if self.phase == "NC_DONE":
            return self._update_post_push_recording(now)
        return None

    def _update_capture_approach(self, now, progress, evidence):
        """接近中の一度の有効測距で捕捉成立し、その周期に次動作へ渡す。"""
        if not evidence:
            return None
        self.capture_candidate_at = now
        self.capture_candidate_progress = progress
        self.capture_assumed = True
        self.capture_leg_mm = max(0.0, self._forward_mm_from(self.capture_encoder_origin))
        self.logger.info(
            "SUMO_RELATIVE CAPTURE_CONFIRMED_MOVING raw=%s progress=%.1f"
            % (self.latest_raw, progress)
        )
        return self._start_capture_push_transition(now)

    def _start_capture_push_transition(self, now):
        """捕獲確認の有無によらず、既存の回避判定に従う。"""
        self._release_capture_drive()
        bearing = current_bearing(self.context)
        if (not self.angle_policy["toward_green"] or
                abs(delta(self.angle_policy["push_bearing_deg"], bearing)) <= 1.0):
            self._push(now)
            return self._drive_forward(now, 0.0, bearing)
        self.turn = ForwardReturnArc(
            ("forward return after capture" if self.capture_assumed
             else "forward return without capture confirmation"),
            self,
            self.angle_policy["push_bearing_deg"],
        )
        self.phase = "NC_TURN"
        self.capture_turn_performed = True
        self.phase_started_at = now
        return self._update_return_turn(now)

    def _release_capture_drive(self):
        """正常な引継ぎでは直進Behaviorの終了ブレーキを挟まない。"""
        drive = self.drive
        terminate = drive.terminate
        try:
            drive.terminate = lambda status: setattr(drive, "running", False)
            drive.stop(Status.INVALID)
        finally:
            drive.terminate = terminate
        self.drive = None

    def _push_without_capture(self, now, progress):
        """捕獲未確認でも緑床回避を経由して押し出しへ続行する。"""
        self.capture_assumed = False
        self.capture_leg_mm = max(
            0.0, self._forward_mm_from(self.capture_encoder_origin)
        )
        self.logger.info(
            "SUMO_RELATIVE CAPTURE_NOT_CONFIRMED_CONTINUE progress=%.1f"
            % progress
        )
        return self._start_capture_push_transition(now)

    def _update_stationary_confirmation(self, now, bearing, progress, evidence):
        """捕捉成立までは測距を確認し、旋回後は停止待ちのみ行う。"""
        push_settings = self.push_settings
        self._brake()
        if now - self.phase_started_at < push_settings.settle_sec:
            return Status.RUNNING
        if self.phase == "NC_CAPTURE_SETTLE":
            if not evidence:
                return Status.RUNNING
            self.capture_assumed = True
            self.logger.info(
                "SUMO_RELATIVE CAPTURE_CONFIRMED_STOPPED raw=%s progress=%.1f"
                % (self.latest_raw, progress)
            )
        target = self.angle_policy["push_bearing_deg"]
        gap = abs(delta(target, bearing))
        if self.phase == "NC_TURN_SETTLE":
            return self._push(now)
        if not self.angle_policy["toward_green"]:
            self.logger.info("SUMO_RELATIVE SKIP_RETURN_TURN: not toward green")
            return self._push(now)
        if gap <= 1.0:
            return self._push(now)
        self.turn = ForwardReturnArc(
            ("forward return after capture" if self.capture_assumed
             else "forward return without capture confirmation"),
            self,
            self.angle_policy["push_bearing_deg"],
        )
        self.phase = "NC_TURN"
        self.capture_turn_performed = True
        self.phase_started_at = now
        return Status.RUNNING

    def _update_return_turn(self, now):
        """捕捉成立後の戻り旋回を1周期進める。"""
        for m in (runtime.left_motor, runtime.right_motor):
            m.set_brake(False)
        self.turn.tick_once()
        if self.turn.status == Status.FAILURE:
            return self._nc_fail("return turn failed")
        if self.turn.status == Status.SUCCESS:
            self.turn = None
            self._push(now)
            return self._drive_forward(now, 0.0, self.nc_heading)
        return Status.RUNNING

    def _update_post_push_recording(self, now):
        """押し出し後に停止し、録画を終了する。"""
        self._brake()

        if now - self.phase_started_at < 1.0:
            return Status.RUNNING

        # ET相撲で回避旋回したかどうかを③工程に渡す
        self.context.sumo.rally_avoidance_performed = (
            self.capture_turn_performed
        )

        self.nc_outcome = "push-and-backoff-complete"

        return AlignSumoBottle._finish_recording(
            self,
            Status.SUCCESS,
            self.nc_outcome,
        )

    def _drive_forward(self, now, progress, bearing):
        """方位はジャイロで補正し、走行中の角度ずれでは失敗しない。"""
        if self.phase == "NC_PUSH" and self._fixed_enabled():
            self._fixed_sample()
            remaining = self._fixed_remaining()
            if remaining <= 0.0:
                return self._start_backoff(now)

            # 残り200mmで基本出力から50へ徐々に下げる。
            base = self.push_settings.power
            floor = min(50, base)
            self.drive.power = int(round(
                floor + (base - floor) * min(1.0, remaining / 200.0)
            ))
            if self.drive.running:
                limit = min(self.drive.power, 100 - self.drive.power)
                self.drive.pid.output_limits = (-limit, limit)
        if progress < 0:
            return self._nc_fail("negative forward progress")
        if progress - self.progress_anchor >= self.catch_settings.stall_progress_mm:
            self.progress_anchor = progress
            self.progress_at = now
        elif now - self.progress_at > self.catch_settings.stall_timeout_sec:
            return self._nc_fail("no forward progress")
        if self.phase == "NC_CAPTURE":
            result = self._update_capture_camera()
            if result is not None:
                return result
        self.drive.tick_once()
        if self.drive.status == Status.FAILURE:
            return self._nc_fail("gyro drive failed")
        return Status.RUNNING

    def terminate(self, new_status):
        """取消・終了でも子動作と記録を閉じ、モーター出力を残さない。"""
        if (
            getattr(self, "phase", "").startswith("NC_")
            and new_status == Status.INVALID
        ):
            self.nc_outcome = "interrupted"
        super().terminate(new_status)


def build_relative_push(context, config):
    from dataclasses import replace

    # 捕獲確認は 0 < raw <= 90。無効値の除外条件は維持。
    push_settings = replace(config.sumo_relative_push, capture_raw_max=90)
    catch = replace(
        config.sumo_catch,
        distance_mm=push_settings.capture_max_mm,
        power=push_settings.power,
    )
    return RelativeAlignAndPush(
        "align capture relative return and fixed-distance push",
        context,
        config.sumo_align,
        config.sumo,
        catch,
        push_settings,
        config.sumo_push_execute,
    )
