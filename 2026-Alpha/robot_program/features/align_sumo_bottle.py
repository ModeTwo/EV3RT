"""Drive a prescribed distance, face the black sumo bottle, and stop.

No device registration, reset, dispatch, capture thread, catching or pushing here.
alpha.py owns those lifecycle responsibilities.
"""
import time

from py_trees.behaviour import Behaviour
from py_trees.common import Status

from ..behaviours.conditions import IsDistanceEarned
from ..behaviours.gyro_drive import RunByGyro
from ..runtime import runtime
from ..types import HeadingType
from .sumo_bearing import bearing_delta
from .sumo_bearing_motion import current_bearing, EncoderSpinToBearing
from .sumo_black_detector import pixel_bearing


class AlignSumoBottle(Behaviour):
    def __init__(self, name, context, settings, motion_settings):
        super().__init__(name)
        self.context = context
        self.settings = settings
        self.motion_settings = motion_settings
        self.drive = None
        self.turn = None
        self.session = None

    def _brake(self):
        # Attempt both wheels even when one hardware call raises.
        errors = []
        for motor in (runtime.left_motor, runtime.right_motor):
            if motor is None:
                continue
            for method, value in (('set_power', 0), ('set_brake', True)):
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
            setattr(self, name, None)

    def _new_session(self):
        self.session = runtime.video.begin_sumo_alignment_read(
            gyro_reader=runtime.gyro_sensor.get_angle,
            distance_reader=runtime.plotter.get_distance)
        self.last_frame_id = -1
        self.stable = 0
        self.previous_stable_at = None

    def initialise(self):
        runtime.require('video', 'plotter', 'gyro_sensor', 'left_motor', 'right_motor')
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
        self.phase = 'DRIVE'
        # Validate registered bearing before commanding any forward motion.
        current_bearing(self.context)
        self._new_session()
        self.distance = IsDistanceEarned('sumo approach distance', self.settings.distance_mm)
        self.distance.tick_once()  # Latch odometry origin before the first drive tick.
        self.drive = RunByGyro(
            name='sumo prescribed straight approach', target=lambda distance_mm: 0.0,
            target_type=HeadingType.RELATIVE, power=self.settings.power,
            pid_p=self.motion_settings.drive_pid_p,
            pid_i=self.motion_settings.drive_pid_i,
            pid_d=self.motion_settings.drive_pid_d,
            distance_limit_mm=self.settings.distance_mm)
        self.logger.info('SUMO_ALIGN DRIVE distance=%.1f power=%d' % (
            self.settings.distance_mm, self.settings.power))

    def _settle(self, now):
        self._brake()
        self.phase = 'SETTLE'
        self.phase_started_at = now
        self.stable = 0

    def _fail(self, reason):
        self.context.sumo.failure_reason = reason
        self.logger.error('SUMO_ALIGN FAILED: ' + reason)
        self._stop_child('drive')
        self._stop_child('turn')
        self._brake()
        return Status.FAILURE

    def update(self):
        now = time.monotonic()
        if now - self.started_at > self.settings.overall_timeout_sec:
            return self._fail('overall timeout')
        if self.phase == 'DRIVE':
            # Distance has priority: never issue another forward tick at/over D.
            self.distance.tick_once()
            if self.distance.status == Status.SUCCESS:
                self._stop_child('drive')
                self._settle(now)
                return Status.RUNNING
            if now - self.phase_started_at > self.settings.drive_timeout_sec:
                return self._fail('approach distance timeout')
            self.drive.tick_once()
            if self.drive.status == Status.FAILURE:
                return self._fail('gyro approach failed')
            observation = runtime.video.get_sumo_alignment_observation(self.session)
            if observation and observation['frame_id'] > self.last_frame_id:
                self.last_frame_id = observation['frame_id']
                c = observation['candidate']
                if c and observation['confirmed']:
                    self.logger.info('SUMO_ALIGN TRACK frame=%d odo=%.1f cx=%d' % (
                        observation['frame_id'], observation['d'], c['cx']))
            return Status.RUNNING
        if self.phase == 'SETTLE':
            if now - self.phase_started_at < self.settings.settle_sec:
                return Status.RUNNING
            # New read generation rejects frames captured before/during the turn.
            self._new_session()
            self.phase = 'OBSERVE'
            self.phase_started_at = now
            self.observations_after = now
            self.logger.info('SUMO_ALIGN OBSERVE session=%s' % self.session)
            return Status.RUNNING
        if self.phase == 'TURN':
            if now - self.phase_started_at > self.settings.turn_timeout_sec:
                return self._fail('alignment turn timeout')
            self.turn.tick_once()
            if self.turn.status == Status.FAILURE:
                return self._fail('alignment turn failed')
            if self.turn.status == Status.SUCCESS:
                self._stop_child('turn')
                self._settle(now)
            return Status.RUNNING
        if now - self.phase_started_at > self.settings.observation_timeout_sec:
            return self._fail('no fresh stable bottle after stopping')
        observation = runtime.video.get_sumo_alignment_observation(self.session)
        if observation is None or observation['frame_id'] <= self.last_frame_id:
            # A long gap breaks the stable sequence even if the last image repeats.
            if self.previous_stable_at is not None and now - self.previous_stable_at > .5:
                self.stable = 0
                self.previous_stable_at = None
            return Status.RUNNING
        self.last_frame_id = observation['frame_id']
        c = observation['candidate']
        if (not c or not observation['confirmed'] or observation['t'] < self.observations_after
                or not 0 <= now - observation['t'] <= self.settings.max_observation_age_sec):
            self.stable = 0
            self.previous_stable_at = None
            return Status.RUNNING
        bearing = current_bearing(self.context)
        capture_bearing = self.context.sumo.bearing_reference.bearing(observation['gyro_deg'])
        # Camera right is physical clockwise on both courses. Do not mirror theta.
        target = (capture_bearing + pixel_bearing(c['cx'])
                  - pixel_bearing(self.settings.target_cx)) % 360.0
        error = bearing_delta(target, bearing)
        pose_drift = abs(bearing_delta(capture_bearing, bearing))
        centered = (abs(c['cx'] - self.settings.target_cx) <= self.settings.center_tolerance_px
                    and pose_drift <= self.settings.max_stationary_heading_error_deg)
        self.logger.info('SUMO_ALIGN OBS frame=%d cx=%d error=%.2f drift=%.2f' % (
            observation['frame_id'], c['cx'], error, pose_drift))
        if centered:
            if self.previous_stable_at is None or not 0 < observation['t'] - self.previous_stable_at <= .5:
                self.stable = 0
            self.stable += 1
            self.previous_stable_at = observation['t']
            if self.stable >= self.settings.stable_frames:
                self.context.sumo.bottle_aligned = True
                self.context.sumo.camera_capture_bearing_deg = target
                self.context.sumo.camera_capture_heading_deg = runtime.gyro_sensor.get_angle()
                self.context.sumo.bottle_bearing_deg = (capture_bearing + pixel_bearing(c['cx'])) % 360
                self._brake()
                self.logger.info('SUMO_ALIGN ALIGNED bearing=%.2f turns=%d' % (target, self.turns))
                return Status.SUCCESS
            return Status.RUNNING
        self.stable = 0
        self.previous_stable_at = None
        if self.turns >= self.settings.max_turns:
            return self._fail('alignment correction limit reached')
        if abs(error) < min(self.motion_settings.heading_tolerance_deg, 1.0):
            # Wait for a stationary image rather than accepting a stale pose or
            # launching a turn smaller than the shared turn's completion tolerance.
            return Status.RUNNING
        step = max(-self.settings.max_turn_step_deg, min(self.settings.max_turn_step_deg, error))
        self.turn = EncoderSpinToBearing(
            name='face sumo bottle', context=self.context, bearing=(bearing + step) % 360,
            max_power=self.motion_settings.turn_max_power,
            min_power=self.motion_settings.turn_min_power,
            pid_p=self.motion_settings.turn_pid_p,
            pid_i=self.motion_settings.turn_pid_i,
            pid_d=self.motion_settings.turn_pid_d,
            tolerance=min(self.motion_settings.heading_tolerance_deg, 1.0))
        self.turns += 1
        self.phase = 'TURN'
        self.phase_started_at = now
        return Status.RUNNING

    def terminate(self, new_status):
        try:
            self._stop_child('drive')
        finally:
            try:
                self._stop_child('turn')
            finally:
                try:
                    self._brake()
                finally:
                    if self.session is not None:
                        runtime.video.end_sumo_alignment_read(self.session)
                        self.session = None


def build_align_sumo_bottle(context, config):
    return AlignSumoBottle('approach and face sumo bottle only', context,
                           config.sumo_align, config.sumo)
