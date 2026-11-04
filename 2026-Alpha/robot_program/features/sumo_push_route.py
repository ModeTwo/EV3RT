"""Opt-in calibrated sumo capture and green-area-avoiding push.

Reuses alpha lifecycle, visual alignment, gyro drive and absolute bearing turns.
No image gating after alignment. Physical capture/push flags stay unconfirmed.


較正指定時の実行器と、共有の出力制限付き直進部品。較正なしの通常実行は末尾のbuild関数から相対方式へ委譲する。"""

import math
import time
from dataclasses import asdict
from pathlib import Path
from simple_pid import PID
from py_trees.common import Status
from ..behaviours.conditions import IsDistanceEarned
from ..behaviours.gyro_drive import RunByGyro
from ..runtime import runtime
from ..types import HeadingType
from ..timing import CONTROL_INTERVAL_SEC
from .align_sumo_bottle import AlignSumoBottle
from .sumo_bearing_motion import current_bearing, SpinToBearing
from .sumo_push_geometry import (
    PushCalibration,
    FloorPose,
    CaptureEvidence,
    advance,
    delta,
    plan_push,
    alignment_push_policy,
)


class EncoderDistance(IsDistanceEarned):
    """Same BT distance contract; signed encoder mean, not Plotter total distance."""

    def __init__(self, owner, distance):
        super().__init__("sumo signed encoder distance", distance)
        self.owner = owner
        self.origin = owner.pose.signed_mm

    def update(self):
        if self.owner.pose.signed_mm - self.origin >= self.delta_dist:
            return Status.SUCCESS
        else:
            return Status.RUNNING


class BoundedRunByGyro(RunByGyro):
    """Reuse fixed-heading drive; bound differential PWM before the first output."""

    def update(self):
        if not self.running:
            heading = -runtime.course * float(runtime.gyro_sensor.get_angle())
            self.target_heading = heading + delta(self.target, heading)
            limit = min(self.power, 100 - self.power)
            self.pid = PID(
                self.pid_p,
                self.pid_i,
                self.pid_d,
                setpoint=self.target_heading,
                sample_time=CONTROL_INTERVAL_SEC,
                output_limits=(-limit, limit),
            )
            self.running = True
        for m in (runtime.left_motor, runtime.right_motor):
            m.set_brake(False)
        return super().update()


class AlignAndPushRoute(AlignSumoBottle):
    def __init__(
        self, name, context, settings, motion_settings, calibration, execute=False
    ):
        super().__init__(name, context, settings, motion_settings)
        self.calibration = calibration
        self.execute = execute

    def initialise(self):
        c = self.calibration
        self.pose = None
        self.plan = None
        self.angle_policy = None
        self.approach_bearing = None
        self.route_samples = []
        self.route_events = []
        self.capture_assumed = False
        self.push_out_estimated = False
        self.route_reason = None
        self.latest_raw = None
        self.last_sonar_at = None
        self.route_started = False
        self.route_snapshot = {}
        self.next_sensor_at = 0.0
        self.evidence = CaptureEvidence(c)
        runtime.require("left_motor", "right_motor", "gyro_sensor", "sonar_sensor")
        self._brake()
        if runtime.course != (1 if c.course == "left" else -1):
            raise ValueError("runtime course does not match floor calibration")
        now = time.monotonic()
        gyro = float(runtime.gyro_sensor.get_angle())
        if (
            abs(delta(current_bearing(self.context), c.start_bearing_deg))
            > c.heading_error_deg
        ):
            raise ValueError(
                "registered initial bearing does not match floor calibration"
            )
        vals = [float(m.get_count()) for m in (runtime.left_motor, runtime.right_motor)]
        if not all(math.isfinite(v) for v in vals + [gyro]):
            raise ValueError("invalid initial encoders/gyro")
        self.pose = FloorPose(c, *c.start_xy_mm, c.start_bearing_deg, gyro, *vals, now)
        super().initialise()
        self.route_origin = self.pose.signed_mm
        self.progress_at = now
        self.progress_anchor = self.pose.signed_mm
        self._event("REGISTER", execute=self.execute, calibration_id=c.calibration_id)

    def _event(self, event, **kwargs):
        self.route_events.append(dict(t=time.monotonic(), event=event, **kwargs))
        self.logger.info("SUMO_PUSH " + event + " " + str(kwargs))

    def _make_drive(self, bearing):
        gyro = runtime.gyro_sensor.get_angle()
        target = self.context.sumo.bearing_reference.legacy_target(
            bearing, gyro, runtime.course
        )
        return BoundedRunByGyro(
            name="sumo guarded gyro straight",
            target=target,
            target_type=HeadingType.ABSOLUTE,
            power=self.calibration.drive_power,
            pid_p=self.motion_settings.drive_pid_p,
            pid_i=self.motion_settings.drive_pid_i,
            pid_d=self.motion_settings.drive_pid_d,
        )

    def _start_drive(self, now):
        self.approach_bearing = self.pose.bearing
        self.phase = "DRIVE"
        self.phase_started_at = now
        self.progress_anchor = self.pose.signed_mm
        self.progress_at = now
        self.distance = EncoderDistance(self, self.settings.distance_mm)
        # Preserve the requested initial approach PWM; post-align PWM is calibrated separately.
        self.drive = self._make_drive(self.pose.bearing)
        self.drive.power = self.settings.power

    def _finish_recording(self, status, reason):
        if status == Status.SUCCESS and reason == "aligned" and not self.route_started:
            self.route_started = True
            c = self.calibration
            self.capture_bearing = self.pose.bearing
            self.angle_policy = alignment_push_policy(
                c, self.approach_bearing, self.capture_bearing
            )
            self._event("ANGLE_POLICY", **self.angle_policy)
            self.capture_end = advance(
                self.pose.xy, self.capture_bearing, c.capture_max_mm
            )
            self._event("CAPTURE_CORRIDOR", end=self.capture_end)
            if not self.execute:
                return super()._finish_recording(
                    Status.SUCCESS, "shadow-angle-policy-recorded-no-capture-or-push"
                )
            self.phase = "ROUTE_CAPTURE"
            self.phase_started_at = time.monotonic()
            self.route_origin = self.pose.signed_mm
            self.evidence = CaptureEvidence(c)
            self.progress_anchor = self.pose.signed_mm
            self.progress_at = self.phase_started_at
            self.drive = self._make_drive(self.capture_bearing)
            return Status.RUNNING
        return super()._finish_recording(status, reason)

    def _route_fail(self, reason):
        self.route_reason = reason
        self._event("ABORT", reason=reason)
        return self._fail(reason)

    def _record_snapshot(self):
        result = super()._record_snapshot()
        result.update(self.route_snapshot)
        return result

    def _recording_details(self):
        result = super()._recording_details()
        result.update(
            kind="sumo-green-push-v3",
            route=dict(
                calibration=asdict(self.calibration),
                execute=self.execute,
                geometry_stop_enabled=False,
                plan=self.plan,
                angle_policy=self.angle_policy,
                events=list(self.route_events),
                samples=list(self.route_samples),
                capture_assumed=self.capture_assumed,
                physical_capture_confirmed=False,
                push_out_estimated=self.push_out_estimated,
                physical_push_out_confirmed=False,
                failure_reason=self.route_reason,
                image_gate_after_alignment=False,
                pose_method="signed wheel encoder mean + registered scaled gyro; not Plotter distance",
            ),
        )
        return result

    def _sample_sonar(self, now):
        if now < self.next_sensor_at:
            return None
        c = self.calibration
        self.next_sensor_at = now + c.sensor_interval_sec
        started = time.monotonic()
        try:
            raw = runtime.sonar_sensor.get_distance()
        except Exception as exc:
            raise ValueError("sonar API read failed: " + str(exc)) from exc
        ended = time.monotonic()
        if ended - started > c.max_cycle_sec:
            raise ValueError("sonar read exceeded reaction-time bound")
        if (
            not isinstance(raw, bool)
            and isinstance(raw, (int, float))
            and math.isfinite(raw)
        ):
            self.latest_raw = raw
        else:
            self.latest_raw = None
        self.last_sonar_at = ended
        return self.evidence.read(raw, ended)

    def _stop_for(self, phase, now):
        self._stop_child("drive")
        self._stop_child("turn")
        self._brake()
        self.phase = phase
        self.phase_started_at = now
        self.evidence = CaptureEvidence(self.calibration)

    def _start_push(self, now):
        # Use the settled actual pose. Never execute an old endpoint after turn drift.
        target = self.angle_policy["push_bearing_deg"]
        if abs(delta(self.pose.bearing, target)) > self.calibration.turn_tolerance_deg:
            return self._route_fail("actual push bearing not at angle-policy target")
        plan = plan_push(
            self.calibration, self.pose, only_bearing=target, check_clearance=False
        )
        if plan is None or plan["kind"] != "DIRECT":
            return self._route_fail(
                "push distance plan invalid after loaded turn/settle"
            )
        self.plan = plan
        self._event("PUSH", plan=plan)
        self.phase = "ROUTE_PUSH"
        self.phase_started_at = now
        self.route_origin = self.pose.signed_mm
        self.progress_anchor = self.pose.signed_mm
        self.progress_at = now
        self.drive = self._make_drive(plan["bearing_deg"])
        return Status.RUNNING

    def update(self):
        if self.phase == "FINALIZING":
            return super().update()
        c = self.calibration
        now = time.monotonic()
        try:
            self.pose.update(
                float(runtime.left_motor.get_count()),
                float(runtime.right_motor.get_count()),
                float(runtime.gyro_sensor.get_angle()),
                now,
            )
            if self.pose.signed_mm - self.progress_anchor >= 2:
                self.progress_anchor = self.pose.signed_mm
                self.progress_at = now
            if self.phase in ("DRIVE", "ROUTE_CAPTURE", "ROUTE_PUSH"):
                if now - self.progress_at > c.stall_timeout_sec:
                    return self._route_fail("signed encoder no progress")
            self.route_snapshot = dict(
                route_x_mm=self.pose.x,
                route_y_mm=self.pose.y,
                route_bearing_deg=self.pose.bearing,
                route_error_mm=self.pose.error,
                route_capture_assumed=self.capture_assumed,
                route_sonar_raw=self.latest_raw,
                route_kind=self.plan["kind"] if self.plan else None,
                route_alignment_delta_deg=(
                    self.angle_policy["actual_alignment_delta_deg"]
                    if self.angle_policy
                    else None
                ),
                route_angle_decision=(
                    self.angle_policy["decision"] if self.angle_policy else None
                ),
                route_policy_bearing_deg=(
                    self.angle_policy["push_bearing_deg"] if self.angle_policy else None
                ),
            )
            self.route_samples.append(
                dict(
                    t=now,
                    phase=self.phase,
                    signed_mm=self.pose.signed_mm,
                    **self.route_snapshot
                )
            )
            if now - self.started_at > c.mission_timeout_sec:
                return self._route_fail("push mission timeout")
            if not self.phase.startswith("ROUTE_"):
                if self.phase == "DRIVE":
                    if (
                        abs(delta(self.pose.bearing, c.start_bearing_deg))
                        > c.heading_error_deg
                    ):
                        return self._route_fail("approach heading outside bounds")
                return super().update()
            recording = runtime.video.get_sumo_alignment_recording_status()
            if recording is None or recording["error"]:
                return self._route_fail("push recording failed")
            if now - self.phase_started_at > c.phase_timeout_sec:
                return self._route_fail("push phase timeout: " + self.phase)
            if self.phase != "ROUTE_HOLD":
                evidence = self._sample_sonar(now)
            else:
                evidence = None
            # The sensor read may block briefly. Recheck motion/reaction bounds before any output.
            self.pose.update(
                float(runtime.left_motor.get_count()),
                float(runtime.right_motor.get_count()),
                float(runtime.gyro_sensor.get_angle()),
                time.monotonic(),
            )
            if self.phase == "ROUTE_CAPTURE":
                progress = self.pose.signed_mm - self.route_origin
                if progress < 0:
                    return self._route_fail("reverse capture progress")
                if progress >= c.capture_max_mm:
                    return self._route_fail(
                        "capture distance reached without sensor evidence"
                    )
                if evidence:
                    self.capture_assumed = True
                    self._event(
                        "CAPTURE_CANDIDATE", raw=self.latest_raw, progress_mm=progress
                    )
                    self._stop_for("ROUTE_CAPTURE_SETTLE", now)
                    return Status.RUNNING
                if (
                    abs(delta(self.pose.bearing, self.capture_bearing))
                    > c.heading_error_deg
                ):
                    return self._route_fail("capture heading outside bounds")
            elif self.phase == "ROUTE_CAPTURE_SETTLE":
                self._brake()
                if now - self.phase_started_at < c.turn_settle_sec or not evidence:
                    return Status.RUNNING
                self.plan = plan_push(
                    c,
                    self.pose,
                    only_bearing=self.angle_policy["push_bearing_deg"],
                    check_clearance=False,
                )
                if self.plan is None:
                    return self._route_fail(
                        "angle-policy push distance exceeds configured limit"
                    )
                self._event("PLAN", plan=self.plan)
                if self.plan["kind"] == "DIRECT":
                    return self._start_push(now)
                self.turn = SpinToBearing(
                    "loaded sumo redirect",
                    self.context,
                    self.plan["bearing_deg"],
                    max_power=c.turn_power,
                    min_power=c.turn_power,
                    pid_p=self.motion_settings.turn_pid_p,
                    pid_i=self.motion_settings.turn_pid_i,
                    pid_d=self.motion_settings.turn_pid_d,
                    tolerance=c.turn_tolerance_deg,
                )
                self.phase = "ROUTE_TURN"
                self.phase_started_at = now
                return Status.RUNNING
            elif self.phase == "ROUTE_TURN":
                if (
                    self.latest_raw is None
                    or not 0 < self.latest_raw <= c.capture_raw_max
                ):
                    return self._route_fail("capture evidence lost during loaded turn")
                for m in (runtime.left_motor, runtime.right_motor):
                    m.set_brake(False)
                self.turn.tick_once()
                if self.turn.status == Status.FAILURE:
                    return self._route_fail("loaded turn failed")
                if self.turn.status == Status.SUCCESS:
                    self._stop_for("ROUTE_TURN_SETTLE", now)
                return Status.RUNNING
            elif self.phase == "ROUTE_TURN_SETTLE":
                self._brake()
                if now - self.phase_started_at < c.turn_settle_sec or not evidence:
                    return Status.RUNNING
                if (
                    abs(delta(self.pose.bearing, self.plan["bearing_deg"]))
                    > c.heading_error_deg
                ):
                    return self._route_fail("loaded turn overshoot")
                return self._start_push(now)
            elif self.phase == "ROUTE_PUSH":
                progress = self.pose.signed_mm - self.route_origin
                if progress < 0:
                    return self._route_fail("reverse push progress")
                if (
                    self.latest_raw is None
                    or not 0 < self.latest_raw <= c.capture_raw_max
                ):
                    return self._route_fail("capture evidence lost while pushing")
                if progress >= self.plan["distance_mm"]:
                    self._stop_for("ROUTE_HOLD", now)
                    return Status.RUNNING
                if (
                    abs(delta(self.pose.bearing, self.plan["bearing_deg"]))
                    > c.heading_error_deg
                ):
                    return self._route_fail("push heading outside bounds")
            elif self.phase == "ROUTE_HOLD":
                self._brake()
                if now - self.phase_started_at < 1.0:
                    return Status.RUNNING
                # Whole bottle outside circle, with measured hold and pose uncertainty.
                p = advance(self.pose.xy, self.pose.bearing, c.held_bottle_offset_mm)
                required = (
                    c.ring_radius_mm
                    + c.bottle_radius_mm
                    + c.hold_uncertainty_mm
                    + c.push_out_margin_mm
                    + self.pose.error
                    + c.held_bottle_offset_mm
                    * math.sin(math.radians(c.heading_error_deg))
                )
                self.push_out_estimated = math.dist(p, c.ring_center_mm) >= required
                if self.push_out_estimated:
                    reason = (
                        "push-distance-complete-outside-estimated-capture-unverified"
                    )
                else:
                    reason = "push-distance-complete-outside-unconfirmed"
                return super()._finish_recording(Status.SUCCESS, reason)
            self.drive.tick_once()
            if self.drive.status == Status.FAILURE:
                return self._route_fail("guarded gyro drive failed")
            return Status.RUNNING
        except (ValueError, TypeError, OverflowError) as error:
            return self._route_fail(str(error))


def load_push_calibration(config):
    if not config.sumo_push_calibration:
        raise ValueError("sumo-push-route-test requires --sumo-push-calibration")
    p = Path(config.sumo_push_calibration)
    if not p.is_absolute():
        p = Path(__file__).resolve().parents[2] / p
    return PushCalibration.load(p)


def build_sumo_push_route(context, config):
    if not config.sumo_push_calibration:
        from .sumo_relative_push import build_relative_push

        return build_relative_push(context, config)
    return AlignAndPushRoute(
        "align capture and green-avoiding push",
        context,
        config.sumo_align,
        config.sumo,
        load_push_calibration(config),
        config.sumo_push_execute,
    )
