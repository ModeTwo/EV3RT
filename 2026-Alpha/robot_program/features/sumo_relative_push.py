"""No floor calibration: measured relative alignment + explicit push distance.

今回の標準実行。規定接近・画像正対は親へ任せ、正対後の捕捉→戻り→押しを状態別メソッドで進める。NC_は較正JSONなし方式の記録名で、既存ログ互換のため維持する。"""

import math
import time
from py_trees.common import Status
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

    def initialise(self):
        """実行ごとの状態を初期化する。機器登録・タッチ開始はalphaが担当する。"""
        self.nc_samples = []
        self.angle_policy = None
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
        self.capture_candidate_at = None
        self.capture_candidate_progress = None
        self.nc_evidence = CaptureEvidence(self.push_settings)
        self.nc_outcome = None
        runtime.require("sonar_sensor")
        super().initialise()

    def _start_drive(self, now):
        # 最初の直進で指示する絶対目標を床面方位に変換して保持する。
        # 開始時の実姿勢ではなく、直進に使用する目標そのものを基準にする。
        ref = self.context.sumo.bearing_reference
        if ref.reset_gyro_deg is None:
            raise RuntimeError("sumo gyro reference is not registered")
        self.approach_bearing = ref.bearing(
            ref.reset_gyro_deg + self.approach_offset_from_reset_deg
        )
        super()._start_drive(now)

    def _start_catch_drive(self):
        """親の正対完了フック。押す向きを決め、捕捉接近を開始する。"""
        self.angle_policy = alignment_push_policy(
            self.push_settings, self.approach_bearing, current_bearing(self.context)
        )
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
        return Status.RUNNING

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
        """押し区間の実走距離を同じ方位で逆走する。"""
        self._hold("NC_BACK", now)
        self.push_leg_mm = max(0.0, self._forward_mm_from(self.push_encoder_origin))
        self.back_target_mm = self.push_leg_mm
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
            target_type=HeadingType.ABSOLUTE, power=-80,
            pid_p=self.motion_settings.drive_pid_p,
            pid_i=self.motion_settings.drive_pid_i,
            pid_d=self.motion_settings.drive_pid_d,
        )
        self.logger.info("SUMO_RELATIVE BACK_START segment=push distance=%.1f power=-80 bearing=%.2f" % (self.back_target_mm, self.back_heading))
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
            if self.phase == "NC_BACK":
                return self._start_alignment_return_turn(now)
            self.logger.info("SUMO_RELATIVE BACK_COMPLETE progress=%.1f" % progress)
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
            if self.phase == "NC_PUSH" and progress >= push_settings.push_distance_mm:
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
        self._release_capture_drive()
        bearing = current_bearing(self.context)
        if (not self.angle_policy["toward_green"] or
                abs(delta(self.angle_policy["push_bearing_deg"], bearing)) <= 1.0):
            self._push(now)
            return self._drive_forward(now, 0.0, bearing)
        self.turn = ContinuousReturnTurn(
            "relative return after capture", self.context,
            self.angle_policy["push_bearing_deg"],
            max_power=self.push_settings.turn_power,
            min_power=self.push_settings.turn_power,
            pid_p=self.motion_settings.turn_pid_p,
            pid_i=self.motion_settings.turn_pid_i,
            pid_d=self.motion_settings.turn_pid_d, tolerance=1.0,
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
        """未捕捉の上限到達は旋回せず、指定距離の押しと後退へ進む。"""
        self.capture_assumed = False
        self.capture_leg_mm = max(0.0, self._forward_mm_from(self.capture_encoder_origin))
        self.logger.info("SUMO_RELATIVE CAPTURE_NOT_CONFIRMED_CONTINUE progress=%.1f" % progress)
        self._release_capture_drive()
        bearing = current_bearing(self.context)
        self._push(now)
        return self._drive_forward(now, 0.0, bearing)

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
        self.turn = SpinToBearing(
            "relative return after capture",
            self.context,
            target,
            max_power=push_settings.turn_power,
            min_power=push_settings.turn_power,
            pid_p=self.motion_settings.turn_pid_p,
            pid_i=self.motion_settings.turn_pid_i,
            pid_d=self.motion_settings.turn_pid_d,
            tolerance=1.0,
        )
        self.phase = "NC_TURN"
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
        """押し距離終了後は制動を維持し、1秒の録画を完了する。"""
        self._brake()
        if now - self.phase_started_at < 1.0:
            return Status.RUNNING
        self.nc_outcome = "specified-push-and-alignment-return-complete-outside-unconfirmed"
        return AlignSumoBottle._finish_recording(self, Status.SUCCESS, self.nc_outcome)

    def _drive_forward(self, now, progress, bearing):
        """方位はジャイロで補正し、走行中の角度ずれでは失敗しない。"""
        if progress < 0:
            return self._nc_fail("negative forward progress")
        if progress - self.progress_anchor >= self.catch_settings.stall_progress_mm:
            self.progress_anchor = progress
            self.progress_at = now
        elif now - self.progress_at > self.catch_settings.stall_timeout_sec:
            return self._nc_fail("no forward progress")
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

    push_settings = config.sumo_relative_push
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
