"""Drive a prescribed distance, face the black sumo bottle, and stop.

No device registration, reset, dispatch, capture thread, catching or pushing here.
alpha.py owns those lifecycle responsibilities.


規定距離直進→制動待機→画像観測→必要な画像旋回→正対停止。alphaの機器初期化や捕捉・押し出しは扱わない。"""

import time
import math

from py_trees.behaviour import Behaviour
from py_trees.common import Status

from ..behaviours.conditions import IsDistanceEarned
from ..behaviours.gyro_drive import RunByGyro
from ..runtime import runtime
from ..types import HeadingType
from .sumo_bearing import bearing_delta
from .sumo_bearing_motion import current_bearing, SpinToBearing
from .sumo_visual_turn import VisualBottleTurn
from .sumo_visual_policy import VisualTurnMemory


class AlignSumoBottle(Behaviour):
    """規定距離接近と画像正対の共通動作。

    RECORD_START → DRIVE → SETTLE → OBSERVE。
    未正対なら TURN → SETTLE → OBSERVE を繰り返す。
    正対完了は _finish_recording へ通知し、派生クラスの捕捉へ接続できる。"""

    def __init__(self, name, context, settings, motion_settings):
        super().__init__(name)
        self.context = context
        self.settings = settings
        self.motion_settings = motion_settings
        # 全体スタート時の生ジャイロ基準からの物理角度。単体試験は0°。
        self.approach_offset_from_reset_deg = 0.0
        self.drive = None
        self.turn = None
        self.session = None

    def _brake(self):
        self.brake_command_at = time.monotonic()  # Command time, not physical stop.
        # Attempt both wheels even when one hardware call raises.
        errors = []
        for motor in (runtime.left_motor, runtime.right_motor):
            if motor is None:
                continue
            for method, value in (("set_power", 0), ("set_brake", True)):
                try:
                    getattr(motor, method)(value)
                except Exception as error:
                    errors.append(error)
        if errors:
            raise errors[0]

    def _stop_child(self, name):
        child = getattr(self, name)
        if child is not None:
            child.stop(Status.INVALID)
            if name == "turn" and hasattr(child, "result_details"):
                self.completed_visual_turns.append(child.result_details())
            setattr(self, name, None)

    def _new_session(self):
        self.session = runtime.video.begin_sumo_alignment_read(
            gyro_reader=runtime.gyro_sensor.get_angle,
            distance_reader=runtime.plotter.get_distance,
            preserve_history=self.session is not None,
        )
        self.last_frame_id = -1
        self.stable = 0
        self.previous_stable_at = None
        self.observation_decision = "new-session"

    def _record_snapshot(self):
        """動画フレームと対応付ける現在値。ここで新しく測距しない。"""
        now = time.monotonic()
        gyro = runtime.gyro_sensor.get_angle()
        previous = self.last_telemetry_pose
        velocity = None
        if previous is not None and now > previous[0]:
            velocity = ((gyro - previous[1] + 180) % 360 - 180) / (now - previous[0])
        self.last_telemetry_pose = (now, gyro)
        row = dict(
            phase=self.phase,
            turns=self.turns,
            decision=self.observation_decision,
            requested_session=self.session,
            target_cx=self.settings.target_cx,
            odo_mm=runtime.plotter.get_distance(),
            gyro_deg=gyro,
            control_sample_monotonic_sec=now,
            sampled_angular_velocity_deg_s=velocity,
            phase_entered_monotonic_sec=self.phase_started_at,
            brake_command_monotonic_sec=self.brake_command_at,
            selected_frame_id=self.selected_frame_id,
            selected_capture_monotonic_sec=self.selected_capture_at,
            observation_selected_monotonic_sec=self.observation_selected_at,
            selected_observation_age_ms=(
                (self.observation_selected_at - self.selected_capture_at) * 1000
                if self.observation_selected_at is not None
                else None
            ),
            requested_turn_bearing_deg=self.requested_turn_bearing,
            requested_turn_step_deg=self.requested_turn_step,
            measured_heading_error_deg=self.measured_heading_error,
            camera_error_px=self.camera_error_px,
            turn_stage=getattr(self.turn, "phase", None),
            turn_main_power_setting=getattr(self.turn, "main_power", None),
        )
        row.update(getattr(self.turn, "telemetry", {}))
        return row

    def _start_drive(self, now):
        self.phase = "DRIVE"
        self.phase_started_at = now
        self.distance = IsDistanceEarned(
            "sumo approach distance", self.settings.distance_mm
        )
        self.distance.tick_once()
        reference = self.context.sumo.bearing_reference
        if reference.reset_gyro_deg is None:
            raise RuntimeError("sumo gyro reference is not registered")
        # RunByGyroは -course * raw gyro の座標を使う。開始値ではなく
        # ResetDevice後に登録された全体スタート基準から絶対目標を作る。
        raw_target = reference.reset_gyro_deg + self.approach_offset_from_reset_deg
        current = -runtime.course * float(runtime.gyro_sensor.get_angle())
        requested = -runtime.course * raw_target
        target = current + (requested - current + 180.0) % 360.0 - 180.0
        self.drive = RunByGyro(
            name="sumo prescribed straight approach",
            target=lambda distance_mm: target,
            target_type=HeadingType.ABSOLUTE,
            power=self.settings.power,
            pid_p=self.motion_settings.drive_pid_p,
            pid_i=self.motion_settings.drive_pid_i,
            pid_d=self.motion_settings.drive_pid_d,
            distance_limit_mm=self.settings.distance_mm,
        )
        self.logger.info(
            "SUMO_ALIGN DRIVE distance=%.1f power=%d"
            % (self.settings.distance_mm, self.settings.power)
        )

    def initialise(self):
        """実行ごとの状態を初期化する。機器登録・タッチ開始はalphaが担当する。"""
        self.visual_memory = VisualTurnMemory(self.settings)
        self.completed_visual_turns = []
        self.approach_observations = []
        self.approach_recovery_attempted = False
        self.settle_for = self.settings.settle_sec
        self.selected_track_id = None
        self.last_telemetry_pose = None
        self.selected_frame_id = self.selected_capture_at = (
            self.observation_selected_at
        ) = None
        self.requested_turn_bearing = self.requested_turn_step = None
        self.measured_heading_error = self.camera_error_px = None
        runtime.require("video", "plotter", "gyro_sensor", "left_motor", "right_motor")
        self._brake()
        self.context.sumo.bottle_aligned = False
        self.context.sumo.bottle_captured = False
        self.context.sumo.bottle_pushed_out = False
        self.context.sumo.failure_reason = None
        self.context.sumo.camera_capture_bearing_deg = None
        self.context.sumo.camera_capture_heading_deg = None
        self.drive = None
        self.turn = None
        self.started_at = time.monotonic()
        self.phase_started_at = self.started_at
        self.turns = 0
        self.phase = "RECORD_START"
        self.observation_decision = "waiting-for-recorder"
        self.last_frame_log_at = None
        # Validate registered bearing before commanding any forward motion.
        current_bearing(self.context)
        self._new_session()
        runtime.video.begin_sumo_alignment_recording(self._record_snapshot)

    def _settle(self, now, after_turn=False):
        if after_turn:
            self.settle_for = self.settings.visual_post_settle_sec
        else:
            self.settle_for = self.settings.settle_sec
        self._brake()
        self.phase = "SETTLE"
        self.phase_started_at = now
        self.stable = 0

    def _fail(self, reason):
        self.context.sumo.failure_reason = reason
        self.logger.error("SUMO_ALIGN FAILED: " + reason)
        self._stop_child("drive")
        self._stop_child("turn")
        self._brake()
        return self._finish_recording(Status.FAILURE, reason)

    def _recording_details(self):
        """終了summaryへ設定・判断・制御標本をまとめる。物理成功と推定を区別する。"""
        return dict(
            kind="sumo-visual-align-v1",
            visual_turns=self.completed_visual_turns,
            visual_px_per_degree=self.settings.visual_px_per_degree,
            visual_stop_horizon_sec=self.settings.visual_stop_horizon_sec,
        )

    def _finish_recording(self, status, reason):
        self._brake()
        runtime.video.end_sumo_alignment_recording(
            status.name, reason, details=self._recording_details()
        )
        self.final_status = status
        self.phase = "FINALIZING"
        self.finalize_started_at = time.monotonic()
        recording = runtime.video.get_sumo_alignment_recording_status()
        if recording and recording["finished"] and recording["error"]:
            self.context.sumo.failure_reason = "recording failed: " + recording["error"]
            return Status.FAILURE
        if recording and recording["finished"]:
            return status
        else:
            return Status.RUNNING

    def update(self):
        """制御周期1回分。計測・終了優先判定・状態別処理・前進の順に進める。"""
        now = time.monotonic()
        recording = runtime.video.get_sumo_alignment_recording_status()
        if self.phase == "FINALIZING":
            return self._update_recording_close(now, recording)
        if recording is None or recording["error"]:
            return self._fail(
                "recording failed: %s"
                % (recording["error"] if recording else "not started")
            )
        if self.phase == "RECORD_START":
            if not recording["ready"]:
                if now - self.phase_started_at > 5.0:
                    return self._fail("recording/camera startup timeout")
                return Status.RUNNING
            self._start_drive(now)
        if now - self.started_at > self.settings.overall_timeout_sec:
            return self._fail("overall timeout")
        if self.phase == "DRIVE":
            return self._update_prescribed_approach(now)
        if self.phase == "SETTLE":
            return self._update_alignment_settle(now)
        if self.phase == "TURN":
            return self._update_alignment_turn(now)
        return self._update_bottle_observation(now)

    def _update_recording_close(self, now, recording):
        """録画終了を待ち、最終結果を返す。"""
        if recording and recording["finished"]:
            self.logger.info(
                "SUMO_ALIGN RECORD CLOSED frames=%s dropped=%s error=%s"
                % (
                    recording.get("written"),
                    recording.get("dropped"),
                    recording["error"],
                )
            )
            if recording["error"]:
                self.context.sumo.failure_reason = (
                    "recording failed: " + recording["error"]
                )
                return Status.FAILURE
            return self.final_status
        if now - self.finalize_started_at > 5.0:
            self.context.sumo.failure_reason = "recording finalize timeout"
            self.logger.error("SUMO_ALIGN FAILED: recording finalize timeout")
            return Status.FAILURE
        return Status.RUNNING

    def _update_prescribed_approach(self, now):
        """規定距離の終了を先に判定し、未到達なら直進・画像追跡を進める。"""
        self.distance.tick_once()
        if self.distance.status == Status.SUCCESS:
            self._stop_child("drive")
            self._settle(now)
            return Status.RUNNING
        if now - self.phase_started_at > self.settings.drive_timeout_sec:
            return self._fail("approach distance timeout")
        self.drive.tick_once()
        if self.drive.status == Status.FAILURE:
            return self._fail("gyro approach failed")
        observation = runtime.video.get_sumo_alignment_observation(self.session)
        if observation and observation["frame_id"] > self.last_frame_id:
            self.last_frame_id = observation["frame_id"]
            c = observation["candidate"]
            if c and observation["confirmed"]:
                self._remember_approach_observation(observation, now)
                self.logger.info(
                    "SUMO_ALIGN TRACK frame=%d odo=%.1f cx=%d"
                    % (observation["frame_id"], observation["d"], c["cx"])
                )
        return Status.RUNNING

    def _remember_approach_observation(self, observation, now):
        """直進中の新鮮な確定検出だけを追跡IDごとに保持する。"""
        try:
            track = observation.get("track_id")
            t = float(observation["t"])
            distance = float(observation["d"])
            gyro = float(observation["gyro_deg"])
            cx = float(observation["candidate"]["cx"])
            if track is None or not all(map(math.isfinite, (t, distance, gyro, cx))):
                return
            if not (0 <= now - t <= self.settings.max_observation_age_sec and 0 <= cx < 320):
                return
            if self.approach_observations and self.approach_observations[-1]["track"] != track:
                self.approach_observations = []
            self.approach_observations.append(dict(track=track, t=t, d=distance, gyro=gyro, cx=cx))
            self.approach_observations = self.approach_observations[-8:]
        except (KeyError, TypeError, ValueError):
            return

    def _start_approach_recovery(self, now):
        """直進中の履歴から限定的に推定旋回し、画像の再取得を試す。"""
        if self.turns >= self.settings.max_turns:
            return False
        rows = self.approach_observations
        if self.approach_recovery_attempted or len(rows) < 3:
            return False
        a, b = rows[0], rows[-1]
        if not (0 <= now - b["t"] <= 2.0 and 0 < b["d"] - a["d"] <= 250):
            return False
        distance = float(runtime.plotter.get_distance())
        bearing = current_bearing(self.context)
        if not math.isfinite(distance):
            return False
        remaining = max(0.0, min(60.0, distance - b["d"]))
        slope = max(-1.0, min(1.0, (b["cx"] - a["cx"]) / (b["d"] - a["d"])))
        predicted_cx = b["cx"] + max(-30.0, min(30.0, slope * remaining))
        captured_bearing = self.context.sumo.bearing_reference.bearing(b["gyro"])
        target = (captured_bearing + (predicted_cx - self.settings.target_cx) / self.settings.visual_px_per_degree) % 360.0
        error = bearing_delta(target, bearing)
        if abs(error) < 1.0:
            return False
        step = max(-self.settings.max_turn_step_deg, min(self.settings.max_turn_step_deg, error))
        self.approach_recovery_attempted = True
        self.requested_turn_step = step
        self.requested_turn_bearing = (bearing + step) % 360.0
        self.logger.info("SUMO_ALIGN APPROACH_RECOVERY track=%s frames=%d last_cx=%.1f predicted_cx=%.1f last_d=%.1f now_d=%.1f target=%.2f step=%.2f" % (b["track"], len(rows), b["cx"], predicted_cx, b["d"], distance, target, step))
        self.turn = SpinToBearing("sumo last-seen recovery", self.context, self.requested_turn_bearing, max_power=self.settings.visual_slow_power, min_power=self.settings.visual_slow_power, pid_p=self.motion_settings.turn_pid_p, pid_i=self.motion_settings.turn_pid_i, pid_d=self.motion_settings.turn_pid_d, tolerance=1.0)
        self.turns += 1
        self.phase = "TURN"
        self.phase_started_at = now
        return True

    def _update_alignment_settle(self, now):
        """制動後の待機を終えたら新しい画像世代で観測を開始する。"""
        if now - self.phase_started_at < self.settle_for:
            return Status.RUNNING
        # New read generation rejects frames captured before/during the turn.
        self._new_session()
        self.phase = "OBSERVE"
        self.phase_started_at = now
        self.observations_after = now
        self.logger.info("SUMO_ALIGN OBSERVE session=%s" % self.session)
        return Status.RUNNING

    def _update_alignment_turn(self, now):
        """画像正対旋回を1周期進め、完了後は制動待機へ移る。"""
        if now - self.phase_started_at > self.settings.turn_timeout_sec:
            return self._fail("alignment turn timeout")
        self.turn.tick_once()
        if self.turn.status == Status.FAILURE:
            return self._fail("alignment turn failed")
        if self.turn.status == Status.SUCCESS:
            self._stop_child("turn")
            self._settle(now, after_turn=True)
        return Status.RUNNING

    def _on_alignment_turn_limit(self, now, observation, target):
        # Align-only / calibrated routes must not claim alignment success.
        return self._fail("alignment turn limit reached without alignment")

    def _update_bottle_observation(self, now):
        """停止後の新しい画像を評価し、正対完了または次の画像旋回を選ぶ。"""
        if now - self.phase_started_at > self.settings.observation_timeout_sec:
            return self._fail("no fresh stable bottle after stopping")
        observation = runtime.video.get_sumo_alignment_observation(self.session)
        if (not observation or not observation.get("candidate") or not observation.get("confirmed")) and now - self.phase_started_at >= 0.5:
            if self._start_approach_recovery(now):
                return Status.RUNNING
        if observation is None or observation["frame_id"] <= self.last_frame_id:
            if observation is None:
                self.observation_decision = "no-observation"
            else:
                self.observation_decision = "same-frame"
            # A long gap breaks the stable sequence even if the last image repeats.
            if (
                self.previous_stable_at is not None
                and now - self.previous_stable_at > 0.5
            ):
                self.stable = 0
                self.previous_stable_at = None
            return Status.RUNNING
        self.last_frame_id = observation["frame_id"]
        c = observation["candidate"]
        if not c:
            reason = "no-candidate"
        elif not observation["confirmed"]:
            reason = "unconfirmed"
        elif observation["t"] < self.observations_after:
            reason = "before-observe"
        elif not 0 <= now - observation["t"] <= self.settings.max_observation_age_sec:
            reason = "stale-observation"
        else:
            reason = ""
        self.observation_decision = reason or "fresh-confirmed"
        if reason:
            if self.last_frame_log_at is None or now - self.last_frame_log_at >= 1.0:
                self.logger.info(
                    "SUMO_ALIGN FRAME frame=%d candidate=%d confirmed=%d age=%.3f reason=%s"
                    % (
                        observation["frame_id"],
                        int(c is not None),
                        int(observation["confirmed"]),
                        now - observation["t"],
                        reason,
                    )
                )
                self.last_frame_log_at = now
            self.stable = 0
            self.previous_stable_at = None
            return Status.RUNNING
        bearing = current_bearing(self.context)
        self.selected_frame_id = observation["frame_id"]
        self.selected_track_id = observation.get("track_id")
        self.selected_capture_at = observation["t"]
        self.observation_selected_at = now
        self.camera_error_px = c["cx"] - self.settings.target_cx
        capture_bearing = self.context.sumo.bearing_reference.bearing(
            observation["gyro_deg"]
        )
        # Camera right is physical clockwise on both courses. Do not mirror theta.
        target = (
            capture_bearing
            + (c["cx"] - self.settings.target_cx) / self.settings.visual_px_per_degree
        ) % 360.0
        error = bearing_delta(target, bearing)
        self.measured_heading_error = error
        pose_drift = abs(bearing_delta(capture_bearing, bearing))
        centered = (
            abs(c["cx"] - self.settings.target_cx) <= self.settings.center_tolerance_px
            and pose_drift <= self.settings.max_stationary_heading_error_deg
        )
        self.logger.info(
            "SUMO_ALIGN OBS frame=%d cx=%d error=%.2f drift=%.2f"
            % (observation["frame_id"], c["cx"], error, pose_drift)
        )
        if centered:
            if (
                self.previous_stable_at is None
                or not 0 < observation["t"] - self.previous_stable_at <= 0.5
            ):
                self.stable = 0
            self.stable += 1
            self.previous_stable_at = observation["t"]
            if self.stable >= self.settings.stable_frames:
                self.context.sumo.bottle_aligned = True
                self.context.sumo.camera_capture_bearing_deg = target
                self.context.sumo.camera_capture_heading_deg = (
                    runtime.gyro_sensor.get_angle()
                )
                self.context.sumo.bottle_bearing_deg = (
                    capture_bearing
                    + (c["cx"] - 160.0) / self.settings.visual_px_per_degree
                ) % 360
                self._brake()
                self.logger.info(
                    "SUMO_ALIGN ALIGNED bearing=%.2f turns=%d" % (target, self.turns)
                )
                return self._finish_recording(Status.SUCCESS, "aligned")
            return Status.RUNNING
        self.stable = 0
        self.previous_stable_at = None
        if self.turns >= self.settings.max_turns:
            # Reached only after a new, fresh, confirmed stopped observation.
            return self._on_alignment_turn_limit(now, observation, target)
        if abs(error) < min(self.motion_settings.heading_tolerance_deg, 1.0):
            # Wait for a stationary image rather than accepting a stale pose or
            # launching a turn smaller than the shared turn's completion tolerance.
            return Status.RUNNING
        step = max(
            -self.settings.max_turn_step_deg,
            min(self.settings.max_turn_step_deg, error),
        )
        self.requested_turn_step = step
        self.requested_turn_bearing = (bearing + step) % 360
        self.turn = VisualBottleTurn(
            name="face sumo bottle",
            context=self.context,
            bearing=(bearing + step) % 360,
            session=self.session,
            settings=self.settings,
            memory=self.visual_memory,
        )
        self.turns += 1
        self.phase = "TURN"
        self.phase_started_at = now
        return Status.RUNNING

    def terminate(self, new_status):
        """取消・終了でも子動作と記録を閉じ、モーター出力を残さない。"""
        try:
            self._stop_child("drive")
        finally:
            try:
                self._stop_child("turn")
            finally:
                try:
                    self._brake()
                finally:
                    try:
                        runtime.video.end_sumo_alignment_recording(
                            new_status.name,
                            self.context.sumo.failure_reason
                            or (
                                "aligned"
                                if new_status == Status.SUCCESS
                                else "interrupted"
                            ),
                            details=self._recording_details(),
                        )
                    finally:
                        if self.session is not None:
                            runtime.video.end_sumo_alignment_read(self.session)
                            self.session = None


def build_align_sumo_bottle(context, config):
    return AlignSumoBottle(
        "approach and face sumo bottle only", context, config.sumo_align, config.sumo
    )
