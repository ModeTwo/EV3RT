"""Stationary labelled raw sonar test using alpha's device/touch lifecycle.

静止した配置の距離センサーを採取する。機器初期化はalphaが行い、この動作は車輪を走行させない。"""

from datetime import datetime, timezone
import math
from pathlib import Path
import time
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from ..runtime import runtime
from ..features.sumo_sensor_data import decode_distance, numeric_stats


class SumoSensorTest(Behaviour):
    def __init__(self, name, context, settings, motion_settings):
        super().__init__(name)
        settings.require_label()
        self.context, self.settings, self.motion_settings = (
            context,
            settings,
            motion_settings,
        )
        self.session = None

    def _brake(self):
        errors = []
        for motor in (runtime.left_motor, runtime.right_motor):
            for method, value in (("set_power", 0), ("set_brake", True)):
                try:
                    getattr(motor, method)(value)
                except Exception as error:
                    errors.append(error)
        if errors:
            raise errors[0]

    def initialise(self):
        runtime.require(
            "sonar_sensor",
            "video",
            "gyro_sensor",
            "plotter",
            "left_motor",
            "right_motor",
            "arm_motor",
        )
        self.samples = []
        self.latest = None
        self.phase = "RECORD_START"
        self.started_at = time.monotonic()
        self.measure_started_at = None
        self.next_sample_at = None
        self.final_details = None
        self.context.sumo.failure_reason = None
        self._brake()
        self.session = runtime.video.begin_sumo_alignment_read(
            gyro_reader=runtime.gyro_sensor.get_angle,
            distance_reader=runtime.plotter.get_distance,
        )
        runtime.video.begin_sumo_alignment_recording(
            self._record_snapshot,
            output_dir=Path(__file__).resolve().parents[2] / "sumo_sensor_test_runs",
        )
        self.logger.info(
            "SUMO_SENSOR READY label=%s forward=%s lateral=%s duration=%.1f interval=%.3f no-drive"
            % (
                self.settings.label,
                self.settings.forward_mm,
                self.settings.lateral_mm,
                self.settings.duration_sec,
                self.settings.interval_sec,
            )
        )

    def _record_snapshot(self):
        latest = self.latest or {}
        return dict(
            phase=self.phase,
            decision="stationary-labelled-sonar",
            requested_session=self.session,
            target_cx=160,
            probe_sample_id=latest.get("sample_id"),
            probe_read_started_monotonic_sec=latest.get("read_started_monotonic_sec"),
            probe_read_ended_monotonic_sec=latest.get("read_ended_monotonic_sec"),
            probe_raw_value=latest.get("raw_value"),
            probe_raw_repr=latest.get("raw_repr"),
            probe_raw_status=latest.get("raw_status"),
            probe_numeric_mm=latest.get("numeric_mm"),
            probe_label=self.settings.label,
            probe_forward_mm=self.settings.forward_mm,
            probe_lateral_mm=self.settings.lateral_mm,
            probe_notes=self.settings.notes,
            probe_sample_age_at_telemetry_sec=(
                time.monotonic() - latest["read_ended_monotonic_sec"]
                if latest
                else None
            ),
        )

    def _sample(self):
        started = time.monotonic()
        raw = None
        error = None
        try:
            raw = runtime.sonar_sensor.get_distance()
        except Exception as caught:
            error = caught
        ended = time.monotonic()
        decoded = decode_distance(
            raw,
            self.settings.mm_per_unit,
            self.motion_settings.sonar_min_distance_mm,
            self.motion_settings.sonar_max_distance_mm,
            error,
        )
        pose = dict(
            gyro_deg=float(runtime.gyro_sensor.get_angle()),
            odo_mm=float(runtime.plotter.get_distance()),
            left_count=float(runtime.left_motor.get_count()),
            right_count=float(runtime.right_motor.get_count()),
            arm_count=float(runtime.arm_motor.get_count()),
        )
        if not all(math.isfinite(v) for v in pose.values()):
            raise ValueError("invalid sensor-test control pose")
        same = bool(
            self.samples
            and decoded["raw_value"] is not None
            and self.samples[-1]["raw_value"] == decoded["raw_value"]
        )
        sample = dict(
            sample_id=len(self.samples) + 1,
            read_started_monotonic_sec=started,
            read_ended_monotonic_sec=ended,
            read_duration_ms=(ended - started) * 1000,
            utc=datetime.now(timezone.utc).isoformat(),
            label=self.settings.label,
            forward_mm=self.settings.forward_mm,
            lateral_mm=self.settings.lateral_mm,
            notes=self.settings.notes,
            same_as_previous_read=same,
            same_value_run_reads=(
                self.samples[-1]["same_value_run_reads"] + 1 if same else 1
            ),
            **decoded,
            **pose
        )
        self.samples.append(sample)
        self.latest = sample
        if len(self.samples) == 1 or len(self.samples) % 10 == 0:
            self.logger.info(
                "SUMO_SENSOR SAMPLE id=%d raw=%s mm=%s status=%s"
                % (
                    sample["sample_id"],
                    sample["raw_repr"],
                    sample["numeric_mm"],
                    sample["raw_status"],
                )
            )

    def _details(self):
        return dict(
            kind="sumo-sensor-test-v1",
            placement=dict(
                label=self.settings.label,
                forward_mm=self.settings.forward_mm,
                lateral_mm=self.settings.lateral_mm,
                notes=self.settings.notes,
                coordinate_origin="floor projection of midpoint between front arm tips; forward positive, robot right positive",
            ),
            duration_sec=self.settings.duration_sec,
            interval_sec=self.settings.interval_sec,
            mm_per_unit=self.settings.mm_per_unit,
            measure_started_monotonic_sec=self.measure_started_at,
            sample_count=len(self.samples),
            sensor_samples=list(self.samples),
            positive_numeric_mm=numeric_stats(
                s["numeric_mm"]
                for s in self.samples
                if s["raw_status"] == "positive_numeric"
            ),
            raw_status_counts={
                key: sum(s["raw_status"] == key for s in self.samples)
                for key in sorted({s["raw_status"] for s in self.samples})
            },
            pose_ranges={
                key: numeric_stats(s[key] for s in self.samples)
                for key in (
                    "gyro_deg",
                    "odo_mm",
                    "left_count",
                    "right_count",
                    "arm_count",
                )
            },
            sample_timing="API read start/end; hardware freshness/exposure synchronization not known",
            raw_status_meaning="numeric classification only; not manufacturer-valid or capture-confirmed",
            physical_capture_confirmed=False,
            no_drive=True,
        )

    def _finish(self, status, reason):
        self._brake()
        if status == Status.FAILURE:
            self.context.sumo.failure_reason = reason
        self.final_details = self._details()
        runtime.video.end_sumo_alignment_recording(
            status.name, reason, details=self.final_details
        )
        self.final_status = status
        self.phase = "FINALIZING"
        self.finalize_at = time.monotonic()
        self.logger.info(
            "SUMO_SENSOR RESULT samples=%d label=%s reason=%s"
            % (len(self.samples), self.settings.label, reason)
        )
        return Status.RUNNING

    def update(self):
        now = time.monotonic()
        recording = runtime.video.get_sumo_alignment_recording_status()
        if self.phase == "FINALIZING":
            if recording and recording["finished"]:
                if recording["error"]:
                    self.context.sumo.failure_reason = (
                        "sensor recording failed: " + recording["error"]
                    )
                    return Status.FAILURE
                return self.final_status
            if now - self.finalize_at > 5:
                self.context.sumo.failure_reason = "sensor recording finalize timeout"
                return Status.FAILURE
            return Status.RUNNING
        if recording is None or recording["error"]:
            return self._finish(Status.FAILURE, "sensor recording unavailable")
        if self.phase == "RECORD_START":
            if not recording["ready"]:
                if now - self.started_at > 5:
                    return self._finish(Status.FAILURE, "sensor camera startup timeout")
                return Status.RUNNING
            self.phase = "SENSOR_SAMPLE"
            self.measure_started_at = now
            self.next_sample_at = now
        if now - self.measure_started_at >= self.settings.duration_sec:
            return self._finish(
                Status.SUCCESS,
                "labelled stationary measurement complete; not capture detection",
            )
        if now >= self.next_sample_at:
            try:
                self._sample()
            except ValueError as error:
                return self._finish(Status.FAILURE, str(error))
            self.next_sample_at = time.monotonic() + self.settings.interval_sec
        return Status.RUNNING

    def terminate(self, new_status):
        try:
            self._brake()
        finally:
            try:
                runtime.video.end_sumo_alignment_recording(
                    new_status.name,
                    "sensor test ended",
                    details=self.final_details or self._details(),
                )
            finally:
                if self.session is not None:
                    runtime.video.end_sumo_alignment_read(self.session)
                    self.session = None


def build_sumo_sensor_test(context, config):
    if config.sumo_sensor.interactive:
        from .sumo_sensor_interactive import InteractiveSumoSensorTest

        return InteractiveSumoSensorTest(
            "interactive stationary sensor measurement only",
            context,
            config.sumo_sensor,
            config.sumo,
        )
    return SumoSensorTest(
        "stationary bottle sensor measurement only",
        context,
        config.sumo_sensor,
        config.sumo,
    )
