"""Measure an unchanged EncoderSpinToBearing, without any forward mission.

旋回出力の単体試験。指令角・実角・制動後の記録を比較するための補助機能。"""

from pathlib import Path
import time

from py_trees.behaviour import Behaviour
from py_trees.common import Status

from ..runtime import runtime
from ..features.sumo_bearing import bearing_delta
from ..features.sumo_bearing_motion import current_bearing, EncoderSpinToBearing


def sensor_stability(samples):
    """Cover 0.2s, including the sample just before its boundary.

    A strict inside-window filter can leave only 0.14s after sampling jitter.
    Do not mistake this loss of time coverage for actual movement.
    """
    if not samples:
        return dict(settled=False, sample_count=0, coverage_sec=0, max_gap_sec=None)
    end = samples[-1]["t"]
    cutoff = end - 0.2
    start = 0
    for index, sample in enumerate(samples):
        if sample["t"] <= cutoff + 1e-9:
            start = index
        else:
            break
    tail = samples[start:]
    coverage = end - tail[0]["t"]
    gaps = [b["t"] - a["t"] for a, b in zip(tail, tail[1:])]
    max_gap = max(gaps, default=0)
    angles = [(s["gyro_deg"] - tail[-1]["gyro_deg"] + 180) % 360 - 180 for s in tail]
    gyro_span = max(angles) - min(angles)
    left_span = max(s["left_count"] for s in tail) - min(s["left_count"] for s in tail)
    right_span = max(s["right_count"] for s in tail) - min(
        s["right_count"] for s in tail
    )
    time_ok = (
        len(tail) >= 3
        and 0.2 - 1e-9 <= coverage <= 0.35 + 1e-9
        and max_gap <= 0.15 + 1e-9
        and all(gap >= 0 for gap in gaps)
    )
    settled = time_ok and gyro_span <= 1.1 and left_span <= 1 and right_span <= 1
    return dict(
        settled=settled,
        sample_count=len(tail),
        coverage_sec=coverage,
        max_gap_sec=max_gap,
        gyro_span_deg=gyro_span,
        left_count_span=left_span,
        right_count_span=right_span,
        time_coverage_ok=time_ok,
    )


class _CommandProbe:
    """Forward API calls and record their values; this is not motor feedback."""

    def __init__(self, motor, name, report):
        self.motor, self.name, self.report = motor, name, report

    def __getattr__(self, name):
        return getattr(self.motor, name)

    def set_power(self, value):
        self.motor.set_power(value)
        self.report(self.name, "power", value)

    def set_brake(self, value):
        self.motor.set_brake(value)
        self.report(self.name, "brake", bool(value))


class SumoTurnTest(Behaviour):
    def __init__(self, name, context, settings, motion_settings):
        super().__init__(name)
        self.context, self.settings, self.motion_settings = (
            context,
            settings,
            motion_settings,
        )
        self.session = None
        self.turn = None

    def _command(self, motor, kind, value):
        if len(self.commands) < 2000:
            self.commands.append(
                dict(
                    t=time.monotonic(),
                    motor=motor,
                    kind=kind,
                    value=value,
                    phase=self.phase,
                    turn_stage=getattr(self.turn, "phase", None),
                )
            )
        else:
            self.trace_truncated = True
        if kind == "power":
            self.last_powers[motor] = value

    def _brake(self):
        errors = []
        for probe in self.probes:
            for method, value in (("set_power", 0), ("set_brake", True)):
                try:
                    getattr(probe, method)(value)
                except Exception as error:
                    errors.append(error)
        if errors:
            raise errors[0]

    def _with_probes(self, callback):
        # Only around this diagnostic's child tick. Restore even on exceptions;
        # no probe survives into another mission or the normal shutdown handler.
        previous = runtime.left_motor, runtime.right_motor
        runtime.left_motor, runtime.right_motor = self.probes
        try:
            return callback()
        finally:
            runtime.left_motor, runtime.right_motor = previous

    def _stop_turn(self):
        if self.turn is not None:
            self._with_probes(lambda: self.turn.stop(Status.INVALID))
            self.turn = None

    def _sample(self, now):
        gyro = runtime.gyro_sensor.get_angle()
        left, right = (probe.get_count() for probe in self.probes)
        velocity = None
        if self.samples and now > self.samples[-1]["t"]:
            velocity = bearing_delta(gyro, self.samples[-1]["gyro_deg"]) / (
                now - self.samples[-1]["t"]
            )
        sample = dict(
            t=now,
            gyro_deg=gyro,
            left_count=left,
            right_count=right,
            phase=self.phase,
            turn_stage=getattr(self.turn, "phase", None),
            left_command=self.last_powers["left"],
            right_command=self.last_powers["right"],
            angular_velocity_deg_s=velocity,
        )
        if len(self.samples) < 1000:
            self.samples.append(sample)
        else:
            self.trace_truncated = True
        self.snapshot = dict(
            phase=self.phase,
            turns=int(self.turn_started_at is not None),
            decision="turn-calibration",
            requested_session=self.session,
            target_cx=160,
            odo_mm=runtime.plotter.get_distance(),
            gyro_deg=gyro,
            control_sample_monotonic_sec=now,
            sampled_angular_velocity_deg_s=velocity,
            phase_entered_monotonic_sec=self.phase_started_at,
            requested_turn_step_deg=self.settings.angle_deg,
            requested_turn_bearing_deg=self.target_bearing,
            turn_stage=sample["turn_stage"],
            turn_main_power_setting=self.settings.power,
            test_control_gyro_deg=gyro,
            test_left_count=left,
            test_right_count=right,
            test_left_command=sample["left_command"],
            test_right_command=sample["right_command"],
        )
        return sample

    def _record_snapshot(self):
        return dict(self.snapshot)

    def _stable_tail(self):
        return sensor_stability(self.samples)["settled"]

    def initialise(self):
        runtime.require("video", "gyro_sensor", "plotter", "left_motor", "right_motor")
        self.phase = "RECORD_START"
        self.started_at = self.phase_started_at = time.monotonic()
        self.commands = []
        self.samples = []
        self.trace_truncated = False
        self.snapshot = {}
        self.last_powers = dict(left=0, right=0)
        self.probes = (
            _CommandProbe(runtime.left_motor, "left", self._command),
            _CommandProbe(runtime.right_motor, "right", self._command),
        )
        self.turn = None
        self.turn_started_at = None
        self.turn_completed_at = None
        self.start_gyro = None
        self.completed_gyro = None
        self.target_bearing = None
        self.context.sumo.failure_reason = None
        self.context.sumo.bottle_aligned = False
        current_bearing(self.context)  # Registered after the existing ResetDevice.
        self._brake()
        self._sample(self.started_at)
        self.session = runtime.video.begin_sumo_alignment_read(
            gyro_reader=runtime.gyro_sensor.get_angle,
            distance_reader=runtime.plotter.get_distance,
        )
        self._sample(time.monotonic())
        runtime.video.begin_sumo_alignment_recording(
            self._record_snapshot,
            output_dir=Path(__file__).resolve().parents[2] / "sumo_turn_test_runs",
        )

    def _details(self):
        last = self.samples[-1]
        if self.start_gyro is not None:
            actual = bearing_delta(last["gyro_deg"], self.start_gyro)
        else:
            actual = None
        settled = self._stable_tail()
        return dict(
            kind="sumo-turn-test",
            requested_angle_deg=self.settings.angle_deg,
            main_power=self.settings.power,
            fine_min_power=min(50, self.settings.power),
            fine_max_power=min(60, self.settings.power),
            start_gyro_deg=self.start_gyro,
            completed_gyro_deg=self.completed_gyro,
            final_gyro_deg=last["gyro_deg"],
            actual_angle_deg=actual,
            final_error_deg=(
                bearing_delta(actual, self.settings.angle_deg)
                if actual is not None
                else None
            ),
            rotation_after_completion_deg=(
                bearing_delta(last["gyro_deg"], self.completed_gyro)
                if self.completed_gyro is not None
                else None
            ),
            turn_duration_sec=(
                self.turn_completed_at - self.turn_started_at
                if self.turn_completed_at is not None
                else None
            ),
            turn_started_monotonic_sec=self.turn_started_at,
            turn_completed_monotonic_sec=self.turn_completed_at,
            sensor_settled_at_end=settled,
            stability_window=sensor_stability(self.samples),
            hold_sec=self.settings.hold_sec,
            trace_truncated=self.trace_truncated,
            command_meaning="successful motor API calls, not measured motor output",
            control_samples=list(self.samples),
            motor_commands=list(self.commands),
        )

    def _finish(self, status, reason):
        self._stop_turn()
        self._brake()
        if status == Status.FAILURE:
            self.context.sumo.failure_reason = reason
            self.logger.error("SUMO_TURN_TEST FAILED: " + reason)
        details = self._details()
        self.logger.info(
            "SUMO_TURN_TEST RESULT requested=%.2f actual=%s error=%s after_completion=%s settled=%s"
            % (
                self.settings.angle_deg,
                details["actual_angle_deg"],
                details["final_error_deg"],
                details["rotation_after_completion_deg"],
                details["sensor_settled_at_end"],
            )
        )
        runtime.video.end_sumo_alignment_recording(status.name, reason, details=details)
        self.final_status = status
        self.phase = "FINALIZING"
        self.finalize_at = time.monotonic()
        return Status.RUNNING

    def update(self):
        now = time.monotonic()
        recording = runtime.video.get_sumo_alignment_recording_status()
        if self.phase == "FINALIZING":
            if recording and recording["finished"]:
                if recording["error"]:
                    self.context.sumo.failure_reason = (
                        "recording failed: " + recording["error"]
                    )
                    return Status.FAILURE
                self.logger.info(
                    "SUMO_TURN_TEST RECORD CLOSED frames=%s dropped=%s"
                    % (recording.get("written"), recording.get("dropped"))
                )
                return self.final_status
            if now - self.finalize_at > 5:
                self.context.sumo.failure_reason = (
                    "turn-test recording finalize timeout"
                )
                return Status.FAILURE
            return Status.RUNNING
        sample = self._sample(now)
        if self.trace_truncated:
            return self._finish(Status.FAILURE, "turn-test trace capacity exceeded")
        if recording is None or recording["error"]:
            return self._finish(Status.FAILURE, "turn-test recording unavailable")
        if now - self.started_at > self.settings.overall_timeout_sec:
            return self._finish(Status.FAILURE, "turn-test overall timeout")
        if (
            self.start_gyro is not None
            and abs(bearing_delta(sample["gyro_deg"], self.start_gyro))
            > abs(self.settings.angle_deg) + 12
        ):
            return self._finish(Status.FAILURE, "turn-test angular excursion limit")
        if self.phase == "RECORD_START":
            if not recording["ready"]:
                if now - self.phase_started_at > 5:
                    return self._finish(
                        Status.FAILURE, "turn-test camera startup timeout"
                    )
                return Status.RUNNING
            self.phase = "BASELINE"
            self.phase_started_at = now
        if self.phase == "BASELINE":
            if now - self.phase_started_at < self.settings.baseline_sec:
                return Status.RUNNING
            if not self._stable_tail():
                if now - self.phase_started_at > 2:
                    return self._finish(Status.FAILURE, "turn-test baseline not stable")
                return Status.RUNNING
            self.start_gyro = sample["gyro_deg"]
            self.turn_started_at = now
            self.target_bearing = (
                current_bearing(self.context) + self.settings.angle_deg
            ) % 360
            self.turn = EncoderSpinToBearing(
                "measured sumo turn",
                self.context,
                self.target_bearing,
                max_power=self.settings.power,
                min_power=self.motion_settings.turn_min_power,
                pid_p=self.motion_settings.turn_pid_p,
                pid_i=self.motion_settings.turn_pid_i,
                pid_d=self.motion_settings.turn_pid_d,
                tolerance=1.0,
            )
            self.phase = "TURN"
            self.phase_started_at = now
            self.logger.info(
                "SUMO_TURN_TEST START angle=%.2f power=%d"
                % (self.settings.angle_deg, self.settings.power)
            )
        if self.phase == "TURN":
            if now - self.turn_started_at > self.settings.turn_timeout_sec:
                return self._finish(Status.FAILURE, "turn-test turn timeout")
            self._with_probes(self.turn.tick_once)
            if self.turn.status == Status.FAILURE:
                return self._finish(Status.FAILURE, "turn-test turn failed")
            if self.turn.status == Status.SUCCESS:
                self.turn_completed_at = time.monotonic()
                self.completed_gyro = self._sample(self.turn_completed_at)["gyro_deg"]
                self._stop_turn()
                self._brake()
                self.phase = "POST_HOLD"
                self.phase_started_at = self.turn_completed_at
            return Status.RUNNING
        if self.phase == "POST_HOLD":
            if now - self.phase_started_at < self.settings.hold_sec:
                return Status.RUNNING
            if not self._details()["sensor_settled_at_end"]:
                return self._finish(
                    Status.FAILURE, "turn-test sensors not settled after hold"
                )
            return self._finish(
                Status.SUCCESS, "turn measurement completed; not bottle alignment"
            )
        return Status.RUNNING

    def terminate(self, new_status):
        try:
            self._stop_turn()
        finally:
            try:
                self._brake()
            finally:
                try:
                    runtime.video.end_sumo_alignment_recording(
                        new_status.name, "turn-test ended", details=self._details()
                    )
                finally:
                    if self.session is not None:
                        runtime.video.end_sumo_alignment_read(self.session)
                        self.session = None


def build_sumo_turn_test(context, config):
    return SumoTurnTest(
        "sumo turn measurement only", context, config.sumo_turn_test, config.sumo
    )
