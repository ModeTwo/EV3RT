"""Pure image/gyro turn policy. Positive power means physical clockwise.

Frame t/gyro describe capture pose, not the pose at policy execution. All
durations use monotonic seconds. A prediction never substitutes for a detection.


画像・ジャイロを入力し旋回出力を決める純粋な方針。モーターIOはsumo_visual_turnへ分離する。"""

import math


def delta(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


class VisualTurnMemory:
    """Stationary sign brackets shared by correction turns in one mission."""

    def __init__(self, settings):
        self.settings = settings
        self.origin = None
        self.track = None
        self.source = None
        self.lower = self.upper = None
        self.sign = 0
        self.reversals = 0
        self.last_frame = None

    def observe(self, gyro, error, observation):
        identity = observation.get("track_id")
        source = observation.get("mask_source")
        if identity != self.track or source != self.source:
            self.origin, self.track, self.source = gyro, identity, source
            self.lower = self.upper = None
            self.sign = 0
            self.last_frame = None
            # Do not restore fast output after an actual overshoot in this mission.
        key = (observation.get("session"), observation["frame_id"])
        if key == self.last_frame:
            return
        self.last_frame = key
        heading = delta(gyro, self.origin)
        if error > self.settings.center_tolerance_px + 2:
            sign = 1
        elif error < -self.settings.center_tolerance_px - 2:
            sign = -1
        else:
            sign = 0
        if sign and self.sign and sign != self.sign:
            self.reversals += 1
        if sign:
            self.sign = sign
            if sign > 0:
                if self.lower is None:
                    self.lower = heading
                else:
                    self.lower = max(self.lower, heading)
            else:
                if self.upper is None:
                    self.upper = heading
                else:
                    self.upper = min(self.upper, heading)
        if (
            self.lower is not None
            and self.upper is not None
            and self.lower >= self.upper
        ):
            self.lower = self.upper = (
                None  # Inconsistent visual evidence: discard bracket.
            )

    def limit(self, gyro):
        cap = max(0.75, self.settings.max_turn_step_deg * 0.5**self.reversals)
        if self.lower is not None and self.upper is not None:
            midpoint = (self.lower + self.upper) / 2
            cap = min(cap, max(0.5, abs(midpoint - delta(gyro, self.origin))))
        return cap


class VisualTurnPolicy:
    """One bounded turn, including brake/quiet detection before any reversal."""

    def __init__(self, settings, memory, session, now, gyro):
        self.s, self.memory, self.session = settings, memory, session
        self.started = self.last_t = now
        self.start_gyro = self.last_gyro = gyro
        self.samples = [(now, gyro)]
        self.velocity_samples = []
        self.stage = "READY"
        self.power = 0
        self.direction = 0
        self.brake_at = now
        self.pulse_at = now
        self.moving_at = now
        self.moving_gyro = gyro
        self.kick_until = None
        self.kick_used = False
        self.pulses = 0
        self.last_good_at = now
        self.track = memory.track
        self.source = memory.source
        self.error = self.predicted = self.age = None
        self.velocity = 0.0
        self.reason = ""
        self.frame_id = self.capture_at = self.capture_gyro = None
        self.trace = []
        self.finish_after_brake = False

    def _brake(self, now, reason, finish=False):
        if self.stage != "BRAKING":
            self.brake_at = now
        self.stage, self.power, self.reason = "BRAKING", 0, reason
        self.finish_after_brake = self.finish_after_brake or finish

    def _quiet(self, now):
        cutoff = now - 0.12
        before = [i for i, (t, _) in enumerate(self.samples) if t <= cutoff]
        if before:
            window = self.samples[before[-1] :]
        else:
            window = self.samples
        return (
            len(window) >= 3
            and now - window[0][0] >= 0.10
            and max(g for _, g in window) - min(g for _, g in window) <= 0.6
        )

    def _valid(self, observation, now):
        try:
            return (
                observation is not None
                and observation.get("session") == self.session
                and observation.get("confirmed")
                and observation.get("track_id") is not None
                and observation.get("candidate") is not None
                and all(
                    math.isfinite(float(v))
                    for v in (
                        observation["t"],
                        observation["gyro_deg"],
                        observation["candidate"]["cx"],
                    )
                )
                and 0 <= now - observation["t"] <= self.s.visual_max_image_age_sec
            )
        except (KeyError, TypeError, ValueError):
            return False

    def step(self, now, gyro, observation):
        if not math.isfinite(gyro) or now < self.last_t:
            self.stage, self.power, self.reason = "FAILED", 0, "invalid-control-pose"
            return self.snapshot(now)
        if now > self.last_t:
            speed = delta(gyro, self.last_gyro) / (now - self.last_t)
            self.velocity_samples.append((now, max(-200.0, min(200.0, speed))))
            self.velocity_samples = [
                (t, v) for t, v in self.velocity_samples if now - t <= 0.08
            ]
            # Quantized gyro: retain recent velocity in the current travel direction.
            self.velocity = max(self.velocity_samples, key=lambda item: abs(item[1]))[1]
        self.last_t, self.last_gyro = now, gyro
        self.samples.append((now, gyro))
        self.samples = self.samples[-64:]
        excursion = abs(delta(gyro, self.start_gyro))
        if (
            excursion > self.s.max_turn_step_deg + 8
            or now - self.started > self.s.turn_timeout_sec
        ):
            self.stage, self.power, self.reason = "FAILED", 0, "turn-bound-or-timeout"
        elif self.stage in ("DONE", "FAILED"):
            self.power = 0
        else:
            self._control(now, gyro, observation, excursion)
        row = self.snapshot(now)
        if len(self.trace) < 512:
            self.trace.append(dict(row))
        return row

    def _control(self, now, gyro, observation, excursion):
        valid = self._valid(observation, now)
        if valid:
            self.frame_id = observation["frame_id"]
            self.capture_at = observation["t"]
            self.capture_gyro = observation["gyro_deg"]
            if self.track is not None and observation["track_id"] != self.track:
                self.stage, self.power, self.reason = (
                    "FAILED",
                    0,
                    "target-identity-changed",
                )
                return
            self.track = observation["track_id"]
            self.last_good_at = now
            self.age = now - observation["t"]
            self.error = (
                observation["candidate"]["cx"]
                - self.s.target_cx
                - self.s.visual_px_per_degree * delta(gyro, observation["gyro_deg"])
            )
            self.predicted = (
                self.error
                - self.velocity
                * self.s.visual_stop_horizon_sec
                * self.s.visual_px_per_degree
            )
        else:
            self.error = self.predicted = self.age = None
            self._brake(now, "missing-or-stale-image")
            if now - self.last_good_at > self.s.visual_loss_timeout_sec:
                self.stage, self.reason = "FAILED", "image-loss-timeout"
            return
        if self.stage == "BRAKING":
            if now - self.brake_at < self.s.visual_brake_hold_sec or not self._quiet(
                now
            ):
                return
            # A repeated pre-brake frame cannot confirm the stopped pose.
            if observation["t"] <= self.brake_at:
                return
            self.memory.observe(gyro, self.error, observation)
            if self.finish_after_brake or abs(self.error) <= self.s.center_tolerance_px:
                self.stage, self.reason = "DONE", "stopped-with-fresh-image"
                return
            if now - self.brake_at < self.s.visual_pulse_off_sec:
                return
            self.stage = "READY"
        if self.stage == "READY":
            if not self._quiet(now):
                self._brake(now, "waiting-for-quiet")
                return
            self.memory.observe(gyro, self.error, observation)
            if abs(self.error) <= self.s.center_tolerance_px:
                self._brake(now, "centered", finish=True)
                return
            if self.error > 0:
                self.direction = 1
            else:
                self.direction = -1
            self.moving_at, self.moving_gyro = now, gyro
            self.pulse_at = now
            self.pulses += 1
            self.budget = min(self.s.max_turn_step_deg, self.memory.limit(gyro))
            self.stage = "MOVING"
        travel = abs(delta(gyro, self.moving_gyro))
        stop_deg = (
            abs(self.velocity) * self.s.visual_stop_horizon_sec
            + self.s.visual_stop_margin_deg
        )
        remaining = self.error * self.direction / self.s.visual_px_per_degree
        approaching = self.velocity * self.direction > 0
        source_changed = (
            self.source is not None and observation.get("mask_source") != self.source
        )
        self.source = observation.get("mask_source")
        if (
            excursion >= self.s.max_turn_step_deg
            or travel >= self.budget
            or source_changed
            or remaining <= 0
            or abs(self.error) <= self.s.center_tolerance_px
            or approaching
            and (remaining <= stop_deg or self.predicted * self.direction <= 0)
        ):
            self._brake(
                now,
                "predicted-center-or-bound",
                finish=travel >= self.budget or excursion >= self.s.max_turn_step_deg,
            )
            return
        fine = remaining <= self.s.visual_fine_error_deg or self.memory.reversals > 0
        if fine and now - self.pulse_at >= self.s.visual_pulse_on_sec:
            self._brake(now, "fine-pulse-end")
            return
        fast_allowed = (
            not self.memory.reversals
            and self.age <= self.s.visual_fast_image_age_sec
            and observation.get("mask_source") == "value"
        )
        fast = fast_allowed and remaining >= max(
            self.s.visual_fast_error_deg, stop_deg + 3
        )
        if (
            fast_allowed
            and not self.kick_used
            and remaining >= 5
            and now - self.moving_at >= 0.08
            and travel < 0.6
        ):
            self.kick_used = True
            self.kick_until = now + self.s.visual_kick_sec
        kick = (
            fast_allowed
            and self.kick_until is not None
            and now < self.kick_until
            and remaining > stop_deg + 2
        )
        self.power = self.direction * (
            self.s.visual_fast_power if fast or kick else self.s.visual_slow_power
        )
        if kick:
            self.reason = "kick"
        elif fast:
            self.reason = "fast"
        elif fine:
            self.reason = "fine-pulse"
        else:
            self.reason = "slow"

    def snapshot(self, now):
        return dict(
            visual_time=now,
            visual_stage=self.stage,
            visual_reason=self.reason,
            visual_frame_id=self.frame_id,
            visual_capture_time=self.capture_at,
            visual_capture_gyro_deg=self.capture_gyro,
            visual_control_gyro_deg=self.last_gyro,
            visual_power=self.power,
            visual_error_px=self.error,
            visual_predicted_error_px=self.predicted,
            visual_velocity_deg_s=self.velocity,
            visual_image_age_sec=self.age,
            visual_reversals=self.memory.reversals,
            visual_lower_deg=self.memory.lower,
            visual_upper_deg=self.memory.upper,
            visual_pulses=self.pulses,
        )
