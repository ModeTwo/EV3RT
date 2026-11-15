"""Align, make one explicit low-power approach, brake, and record for inspection.

No physical capture sensor is available: completing this test never sets the
bottle_captured flag. The normal sumo-align mission remains align-and-stop.


正対後に一定距離だけ接近する比較用試験。正対終了フックと、接近・制動・録画の責務を分ける。"""

import math
import time
from py_trees.common import Status
from ..behaviours.conditions import IsDistanceEarned
from ..behaviours.gyro_drive import RunByGyro
from ..runtime import runtime
from ..types import HeadingType
from .align_sumo_bottle import AlignSumoBottle
from .sumo_bearing import bearing_delta


class AlignAndCatchTest(AlignSumoBottle):
    def __init__(self, name, context, settings, motion_settings, catch_settings):
        catch_settings.require_distance()
        super().__init__(name, context, settings, motion_settings)
        self.catch_settings = catch_settings

    def initialise(self):
        self.catch_started = False
        self.catch_finished = False
        self.catch_origin = self.catch_heading = None
        self.catch_track = None
        self.catch_progress = 0.0
        self.catch_error = self.catch_age = None
        self.catch_last_frame = -1
        self.catch_samples = []
        self.catch_stop_reason = None
        self.catch_outcome = None
        self.catch_aligned_at = None
        self.catch_stop_at = None
        self.catch_stop_progress = None
        self.catch_ready_frames = 0
        super().initialise()

    def _recording_details(self):
        result = super()._recording_details()
        result.update(
            kind="sumo-catch-test-v3",
            catch=dict(
                requested_distance_mm=self.catch_settings.distance_mm,
                base_power=self.catch_settings.power,
                aligned_at=self.catch_aligned_at,
                origin_distance_mm=self.catch_origin,
                heading_gyro_deg=self.catch_heading,
                progress_mm=self.catch_progress,
                stop_progress_mm=self.catch_stop_progress,
                stop_at=self.catch_stop_at,
                stop_reason=self.catch_stop_reason,
                outcome=self.catch_outcome,
                physical_capture_confirmed=False,
                image_gate_after_alignment=False,
                samples=list(self.catch_samples),
            ),
        )
        return result

    def _record_snapshot(self):
        result = super()._record_snapshot()
        result.update(
            catch_progress_mm=self.catch_progress,
            catch_camera_error_px=self.catch_error,
            catch_image_age_sec=self.catch_age,
            catch_track_id=self.catch_track,
            catch_base_power=self.catch_settings.power,
            catch_outcome=self.catch_outcome,
        )
        return result

    def _finish_recording(self, status, reason):
        # Reuse the verified align stage but keep its recorder/session alive.
        if status == Status.SUCCESS and reason == "aligned" and not self.catch_started:
            self.catch_started = True
            self.catch_aligned_at = time.monotonic()
            self.catch_track = self.selected_track_id
            return self._start_catch_drive()
        return super()._finish_recording(status, reason)

    def _start_catch_drive(self):
        # Alignment already required two new centered images. Do not ask the
        # near-field camera to confirm again or gate subsequent forward motion.
        now = time.monotonic()
        distance = float(runtime.plotter.get_distance())
        gyro = float(runtime.gyro_sensor.get_angle())
        if not all(map(math.isfinite, (distance, gyro))):
            return self._fail("invalid catch start pose")
        self.catch_origin, self.catch_heading = distance, gyro
        self.progress_anchor, self.progress_at = 0.0, now
        self.distance = IsDistanceEarned(
            "catch test additional distance", self.catch_settings.distance_mm
        )
        self.distance.tick_once()
        self.drive = RunByGyro(
            name="low-power catch test approach",
            # 画像正対後の実停止方位を、共通走行の絶対角度に変換して保持。
            target=lambda d: -runtime.course * gyro,
            target_type=HeadingType.ABSOLUTE,
            power=self.catch_settings.power,
            pid_p=self.motion_settings.drive_pid_p,
            pid_i=self.motion_settings.drive_pid_i,
            pid_d=self.motion_settings.drive_pid_d,
            distance_limit_mm=self.catch_settings.distance_mm,
        )
        self.phase = "CATCH_DRIVE"
        self.phase_started_at = now
        self.logger.info(
            "SUMO_CATCH DRIVE origin=%.1f heading=%.2f distance=%.1f base_power=%d image_gate=disabled"
            % (
                distance,
                gyro,
                self.catch_settings.distance_mm,
                self.catch_settings.power,
            )
        )
        return Status.RUNNING

    def _catch_stop(self, now, status, reason):
        try:
            self._stop_child("drive")
        finally:
            self._brake()
        self.catch_stop_at = now
        self.catch_stop_progress = self.catch_progress
        self.catch_stop_reason = reason
        if status == Status.SUCCESS:
            self.catch_outcome = "motion-complete-capture-unverified"
        else:
            self.catch_outcome = "aborted-capture-unverified"
        self.catch_result = status
        self.phase = "CATCH_HOLD"
        self.phase_started_at = now
        if status == Status.FAILURE:
            self.context.sumo.failure_reason = reason
        self.context.sumo.bottle_captured = False
        self.logger.info(
            "SUMO_CATCH STOP reason=%s progress=%.1f capture=unverified"
            % (reason, self.catch_progress)
        )
        return Status.RUNNING

    def terminate(self, new_status):
        if self.catch_started and self.phase in ("CATCH_DRIVE", "CATCH_HOLD"):
            self.catch_outcome = "interrupted-capture-unverified"
            self.catch_stop_reason = "interrupted"
            self.catch_stop_at = time.monotonic()
        super().terminate(new_status)

    def update(self):
        if self.phase not in ("CATCH_DRIVE", "CATCH_HOLD"):
            return super().update()
        now = time.monotonic()
        recording = runtime.video.get_sumo_alignment_recording_status()
        if recording is None or recording["error"]:
            return self._fail("catch recording failed")
        distance = float(runtime.plotter.get_distance())
        gyro = float(runtime.gyro_sensor.get_angle())
        if not all(map(math.isfinite, (distance, gyro))):
            return self._fail("invalid catch control pose")
        if self.catch_origin is not None:
            self.catch_progress = distance - self.catch_origin
        if len(self.catch_samples) < 800:
            self.catch_samples.append(
                dict(
                    t=now,
                    phase=self.phase,
                    distance_mm=distance,
                    progress_mm=self.catch_progress,
                    gyro_deg=gyro,
                )
            )
        if self.phase == "CATCH_HOLD":
            if now - self.phase_started_at < self.catch_settings.post_record_sec:
                return Status.RUNNING
            self.catch_finished = True
            return super()._finish_recording(self.catch_result, self.catch_stop_reason)
        if self.phase == "CATCH_DRIVE":
            # Distance first: never send an additional forward tick at/over the limit.
            self.distance.tick_once()
            if self.catch_progress >= self.catch_settings.distance_mm:
                return self._catch_stop(
                    now, Status.SUCCESS, "test-distance-complete-capture-unverified"
                )
            if self.catch_progress < 0:
                return self._catch_stop(
                    now, Status.FAILURE, "negative-approach-progress"
                )
            if now - self.phase_started_at > self.catch_settings.drive_timeout_sec:
                return self._catch_stop(now, Status.FAILURE, "catch-drive-timeout")
            if (
                abs(bearing_delta(gyro, self.catch_heading))
                > self.catch_settings.max_heading_error_deg
            ):
                return self._catch_stop(now, Status.FAILURE, "catch-heading-error")
            if (
                self.catch_progress - self.progress_anchor
                >= self.catch_settings.stall_progress_mm
            ):
                self.progress_anchor, self.progress_at = self.catch_progress, now
            elif now - self.progress_at > self.catch_settings.stall_timeout_sec:
                return self._catch_stop(now, Status.FAILURE, "catch-no-progress")
        self.drive.tick_once()
        if self.drive.status == Status.FAILURE:
            return self._catch_stop(now, Status.FAILURE, "catch-gyro-drive-failed")
        return Status.RUNNING


def build_sumo_catch_test(context, config):
    return AlignAndCatchTest(
        "align and low-power catch test only",
        context,
        config.sumo_align,
        config.sumo,
        config.sumo_catch,
    )
