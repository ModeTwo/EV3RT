"""PC tests: real BT/drive/detector, fake robot IO. Optional --overlay for staging."""
import argparse
from dataclasses import replace
import importlib
import json
from pathlib import Path
import sys
from types import ModuleType
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np

ARGS = argparse.ArgumentParser()
ARGS.add_argument('--repository', type=Path, default=Path(__file__).resolve().parents[1])
ARGS.add_argument('--overlay', type=Path)
OPTIONS, TEST_ARGS = ARGS.parse_known_args()
REPO = OPTIONS.repository.resolve()
sys.path.insert(0, str(REPO))
if OPTIONS.overlay:
    for name in ('robot_program', 'robot_program.features', 'py_etrobo_util'):
        module = ModuleType(name)
        rel = Path(*name.split('.'))
        module.__path__ = [str(OPTIONS.overlay / rel), str(REPO / rel)]
        sys.modules[name] = module
    # Load existing utility exports, resolving video.py from the staged overlay.
    util = sys.modules['py_etrobo_util']
    util.__file__ = str(REPO / 'py_etrobo_util/__init__.py')
else:
    util = None
if not hasattr(cv2, 'wechat_qrcode_WeChatQRCode'):
    # Optional QR backend is absent on this PC; no QR paths are exercised.
    cv2.wechat_qrcode_WeChatQRCode = Mock
if util:
    exec(compile(Path(util.__file__).read_text(encoding='utf-8'), util.__file__, 'exec'), util.__dict__)

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from robot_program.context import RaceContext
from robot_program.config import config_for_mission, mission_requires_camera, mission_requires_qr
from robot_program.features import align_sumo_bottle as target
from robot_program.features.sumo_align_settings import SumoAlignSettings
from robot_program.features.sumo_black_detector import SumoBlackDetector, pixel_bearing
from robot_program.features.sumo_bearing import initial_sumo_bearing
from robot_program.runtime import runtime
from py_etrobo_util.video import Video


class IO:
    def __init__(self):
        self.power = 0
        self.brake = False
        self.outputs = []

    def set_power(self, power):
        self.power = power
        self.outputs.append(power)

    def set_brake(self, brake):
        self.brake = brake


class FakeVideo:
    def __init__(self):
        self.session = 0
        self.observation = None
        self.ends = []
        self.record_status = dict(ready=True, finished=True, error=None)
        self.record_ends = []

    def begin_sumo_alignment_recording(self, telemetry_reader):
        self.telemetry_reader = telemetry_reader

    def get_sumo_alignment_recording_status(self):
        return self.record_status

    def end_sumo_alignment_recording(self, result, reason, **kwargs):
        self.record_ends.append((result, reason))

    def begin_sumo_alignment_read(self, **readers):
        self.session += 1
        self.observation = None
        return self.session

    def get_sumo_alignment_observation(self, session):
        return self.observation if session == self.session else None

    def end_sumo_alignment_read(self, session):
        self.ends.append(session)


class AlignTests(unittest.TestCase):
    def test_selected_observation_and_brake_command_are_separate_telemetry(self):
        self.start_observing()
        self.now.return_value = .6
        self.observation(10, cx=210, age=.1)
        self.node.tick_once()
        row = self.node._record_snapshot()
        self.assertEqual(row['selected_frame_id'], 10)
        self.assertAlmostEqual(row['selected_observation_age_ms'], 100)
        self.assertEqual(row['brake_command_monotonic_sec'], .1)
        self.assertAlmostEqual(row['requested_turn_step_deg'], (210-160)/self.settings.visual_px_per_degree)
        self.assertEqual(row['camera_error_px'], 50)
        self.now.return_value = .8
        self.gyro = 5
        self.assertAlmostEqual(self.node._record_snapshot()['sampled_angular_velocity_deg_s'], 25)

    def setUp(self):
        self.clock = patch.object(target.time, 'monotonic', return_value=0)
        self.now = self.clock.start()
        self.addCleanup(self.clock.stop)
        self.d = 0.0
        self.gyro = 0.0
        self.video = FakeVideo()
        runtime.left_motor, runtime.right_motor = IO(), IO()
        runtime.gyro_sensor = Mock(get_angle=lambda: self.gyro)
        runtime.plotter = Mock(get_distance=lambda: self.d)
        runtime.video = self.video
        runtime.course = 1
        self.context = RaceContext()
        self.context.sumo.bearing_reference.register(0, 0)
        self.config = config_for_mission('sumo-align')
        self.settings = replace(self.config.sumo_align, distance_mm=100)
        self.node = target.AlignSumoBottle('test', self.context, self.settings, self.config.sumo)

    def start_observing(self):
        self.node.tick_once()
        self.assertGreater(runtime.left_motor.power, 0)
        self.d = 100
        self.now.return_value = .1
        self.node.tick_once()
        self.assertEqual(self.node.phase, 'SETTLE')
        self.assert_stopped()
        self.now.return_value = .5
        self.node.tick_once()
        self.assertEqual(self.node.phase, 'OBSERVE')

    def observation(self, frame_id, cx=160, confirmed=True, age=0, gyro=None):
        t = self.now.return_value - age
        self.video.observation = dict(session=self.video.session, frame_id=frame_id,
            candidate=dict(cx=cx), confirmed=confirmed, t=t, track_id=1, mask_source='value',
            gyro_deg=self.gyro if gyro is None else gyro, d=self.d)

    def assert_stopped(self):
        for motor in (runtime.left_motor, runtime.right_motor):
            self.assertEqual(motor.power, 0)
            self.assertTrue(motor.brake)

    def test_distance_stops_before_another_forward_output(self):
        self.node.tick_once()
        index = len(runtime.left_motor.outputs)
        self.d = 100
        self.node.tick_once()
        self.assertTrue(all(v == 0 for v in runtime.left_motor.outputs[index:]))
        self.assert_stopped()

    def test_repeated_frame_never_completes_alignment(self):
        self.start_observing()
        self.now.return_value = .6
        self.observation(10)
        self.node.tick_once()
        for _ in range(8):
            self.node.tick_once()
        self.assertEqual(self.node.status, Status.RUNNING)
        self.assertFalse(self.context.sumo.bottle_aligned)
        self.now.return_value = .8
        self.observation(11)
        self.node.tick_once()
        self.assertEqual(self.node.status, Status.SUCCESS)
        self.assertTrue(self.context.sumo.bottle_aligned)
        self.assertFalse(self.context.sumo.bottle_captured)
        self.assertFalse(self.context.sumo.bottle_pushed_out)
        self.assert_stopped()
        self.assertEqual(self.video.ends, [self.video.session])

    def test_stale_or_pre_settle_image_does_not_turn(self):
        self.start_observing()
        self.now.return_value = 1.1
        self.observation(10, cx=230, age=.6)
        self.node.tick_once()
        self.assertEqual(self.node.phase, 'OBSERVE')
        self.assertEqual(self.node.stable, 0)
        self.assert_stopped()

    def test_stale_pose_must_not_count_as_centered(self):
        self.start_observing()
        self.now.return_value = .6
        self.gyro = 10
        self.observation(10, gyro=0)
        self.node.tick_once()
        self.assertEqual(self.node.phase, 'TURN')
        self.assertAlmostEqual(self.node.turn.bearing, 0)

    def test_rightward_error_is_same_physical_bearing_on_both_courses(self):
        for course in (-1, 1):
            with self.subTest(course=course):
                runtime.course = course
                self.node = target.AlignSumoBottle('test', self.context, self.settings, self.config.sumo)
                self.d = 0
                self.now.return_value = 0
                self.start_observing()
                self.now.return_value = .6
                self.observation(20, cx=210)
                self.node.tick_once()
                self.assertEqual(self.node.phase, 'TURN')
                self.assertAlmostEqual(self.node.turn.bearing, (210-160)/self.settings.visual_px_per_degree)
                self.node.stop(Status.INVALID)
                self.assert_stopped()

    def test_missing_images_fail_and_stop(self):
        self.start_observing()
        self.now.return_value = 4
        self.node.tick_once()
        self.assertEqual(self.node.status, Status.FAILURE)
        self.assert_stopped()

    def test_drive_timeout_fails_and_stops(self):
        self.node.tick_once()
        self.now.return_value = 16
        self.node.tick_once()
        self.assertEqual(self.node.status, Status.FAILURE)
        self.assert_stopped()

    def test_correction_limit_is_failure_not_alignment(self):
        self.start_observing()
        self.node.turns = self.settings.max_turns
        self.now.return_value = .6
        self.observation(20, cx=220)
        self.node.tick_once()
        self.assertEqual(self.node.status, Status.FAILURE)
        self.assertFalse(self.context.sumo.bottle_aligned)
        self.assert_stopped()

    def test_invalidated_drive_stops(self):
        self.node.tick_once()
        self.node.stop(Status.INVALID)
        self.assert_stopped()

    def test_drive_power_is_bounded_under_heading_error(self):
        self.node.tick_once()
        self.gyro = 60
        self.now.return_value = .1
        self.node.tick_once()
        self.assertTrue(0 <= runtime.left_motor.power <= 100)
        self.assertTrue(0 <= runtime.right_motor.power <= 100)

    def test_restart_latches_new_distance_origin(self):
        self.node.tick_once()
        self.node.stop(Status.INVALID)
        self.d = 500
        self.now.return_value = .2
        self.node.tick_once()
        self.assertEqual(self.node.distance.orig_dist, 500)
        self.assertEqual(self.node.phase, 'DRIVE')

    def test_turn_completion_requires_new_stopped_images(self):
        test = self

        class FakeTurn(Behaviour):
            def __init__(self, name, context, bearing, **kwargs):
                super().__init__(name)
                self.bearing = bearing

            def update(self):
                test.gyro = self.bearing
                return Status.SUCCESS

        self.start_observing()
        self.now.return_value = .6
        self.observation(10, cx=210)
        previous_session = self.video.session
        with patch.object(target, 'VisualBottleTurn', FakeTurn):
            self.node.tick_once()
        self.assertEqual(self.node.phase, 'TURN')
        self.now.return_value = .7
        self.node.tick_once()
        self.assertEqual(self.node.phase, 'SETTLE')
        self.now.return_value = 1.1
        self.node.tick_once()
        self.assertGreater(self.video.session, previous_session)
        self.assertIsNone(self.video.observation)
        self.now.return_value = 1.2
        self.observation(11)
        self.node.tick_once()
        self.assertEqual(self.node.status, Status.RUNNING)
        self.now.return_value = 1.4
        self.observation(12)
        self.node.tick_once()
        self.assertEqual(self.node.status, Status.SUCCESS)
        self.assert_stopped()

    def test_no_motion_until_recording_is_ready(self):
        self.video.record_status['ready'] = False
        self.node.tick_once()
        self.assertEqual(self.node.phase, 'RECORD_START')
        self.assert_stopped()
        self.d = 50
        self.now.return_value = .5
        self.video.record_status['ready'] = True
        self.node.tick_once()
        self.assertEqual(self.node.distance.orig_dist, 50)
        self.assertGreater(runtime.left_motor.power, 0)

    def test_recording_error_stops_and_finishes_recording(self):
        self.node.tick_once()
        self.video.record_status['error'] = 'disk full'
        self.now.return_value = .1
        self.node.tick_once()
        self.assertEqual(self.node.status, Status.FAILURE)
        self.assert_stopped()
        self.assertEqual(self.video.record_ends[-1][0], 'FAILURE')
        self.assertIn('disk full', self.video.record_ends[-1][1])

    def test_failed_mission_waits_for_recording_to_close_with_motors_stopped(self):
        self.start_observing()
        self.video.record_status['finished'] = False
        self.now.return_value = 4
        self.node.tick_once()
        self.assertEqual(self.node.phase, 'FINALIZING')
        self.assertEqual(self.node.status, Status.RUNNING)
        self.assert_stopped()
        self.video.record_status['finished'] = True
        self.now.return_value = 4.2
        self.node.tick_once()
        self.assertEqual(self.node.status, Status.FAILURE)
        self.assert_stopped()

    def test_recording_start_timeout_does_not_drive(self):
        self.video.record_status['ready'] = False
        self.node.tick_once()
        self.now.return_value = 5.1
        self.node.tick_once()
        self.assertEqual(self.node.status, Status.FAILURE)
        self.assert_stopped()

    def test_turn_timeout_stops(self):
        self.start_observing()
        self.now.return_value = .6
        self.observation(10, cx=210)
        self.node.tick_once()
        self.assertEqual(self.node.phase, 'TURN')
        self.now.return_value = 5
        self.node.tick_once()
        self.assertEqual(self.node.status, Status.FAILURE)
        self.assert_stopped()


class DetectionTests(unittest.TestCase):
    @staticmethod
    def bottle_image(x=150):
        image = np.full((180, 320, 3), 240, np.uint8)
        image[65:105, x:x+24] = 0
        return image

    def test_shape_rejected_background_does_not_veto_bottle(self):
        image = self.bottle_image()
        image[10:28, 5:145] = 0
        c = SumoBlackDetector().step(image, 0, 1, 1)['candidate']
        self.assertIsNotNone(c)
        self.assertGreater(c['cy'], 60)

    def test_horizontal_edge_fragment_cannot_seed_but_can_be_tracked(self):
        detector = SumoBlackDetector()
        self.assertIsNone(detector.step(self.bottle_image(306), 0, 1, 1)['candidate'])
        detector.step(self.bottle_image(280), 0, 1.1, 2)
        self.assertTrue(detector.step(self.bottle_image(296), 0, 1.2, 3)['confirmed'])

    def test_larger_unrelated_blob_cannot_steal_recent_identity(self):
        detector = SumoBlackDetector()
        image = self.bottle_image()
        first = detector.step(image, 0, 1, 1)
        detector.step(image, 0, 1.1, 2)
        image[60:120, 250:295] = 0
        result = detector.step(image, 0, 1.2, 3)
        self.assertEqual(result['track_id'], first['track_id'])
        self.assertLess(result['candidate']['cx'], 200)

    def test_inherited_history_requires_two_new_images(self):
        detector = SumoBlackDetector()
        image = self.bottle_image()
        detector.step(image, 0, 1, 1)
        old = detector.step(image, 0, 1.1, 2)
        detector.new_observation_session()
        one = detector.step(image, 0, 1.2, 3)
        self.assertEqual(old['track_id'], one['track_id'])
        self.assertFalse(one['confirmed'])
        self.assertTrue(detector.step(image, 0, 1.3, 4)['confirmed'])

    def test_missing_image_never_publishes_predicted_box(self):
        detector = SumoBlackDetector()
        image = self.bottle_image()
        detector.step(image, 0, 1, 1)
        detector.step(image, 0, 1.1, 2)
        blank = np.full_like(image, 240)
        missing = detector.step(blank, 0, 1.2, 3)
        self.assertIsNone(missing['candidate'])
        self.assertFalse(missing['confirmed'])
        self.assertEqual(missing['track_status'], 'missing')
        self.assertFalse(detector.step(image, 0, 1.3, 4)['confirmed'])

    def test_expired_identity_reacquires_with_new_id(self):
        detector = SumoBlackDetector()
        image = self.bottle_image()
        first = detector.step(image, 0, 1, 1)
        result = detector.step(image, 0, 2, 2)
        self.assertGreater(result['track_id'], first['track_id'])
        self.assertTrue(result['history_expired'])
        self.assertFalse(result['confirmed'])

    def test_dual_mask_can_see_saturated_near_black(self):
        image = np.full((180, 320, 3), 240, np.uint8)
        image[65:105, 150:174] = (1, 4, 1)
        self.assertIsNone(SumoBlackDetector('hsv').step(image, 0, 1, 1)['candidate'])
        self.assertEqual(SumoBlackDetector('dual').step(image, 0, 1, 1)['mask_source'], 'value')

    def test_patch_can_promote_to_containing_full_dark_tape(self):
        image = np.full((180, 320, 3), 240, np.uint8)
        image[60:125, 140:185] = (1, 4, 1)
        image[78:103, 152:168] = (3, 3, 3)
        detector = SumoBlackDetector()
        first = detector.step(image, 0, 1, 1)
        second = detector.step(image, 0, 1.1, 2)
        self.assertEqual(first['mask_source'], 'hsv')
        self.assertEqual(second['mask_source'], 'value')
        self.assertEqual(first['track_id'], second['track_id'])

    def test_gyro_compensation_associates_rotation_without_reset(self):
        detector = SumoBlackDetector()
        detector.step(self.bottle_image(190), 0, 1, 1, gyro_deg=0)
        result = detector.step(self.bottle_image(115), 0, 1.1, 2, gyro_deg=15)
        self.assertTrue(result['confirmed'])
        self.assertLess(result['candidate']['cx'], 160)

    def test_video_preserves_identity_but_not_confirmation_between_reads(self):
        with patch.object(Video, '_open_cap'):
            video = Video(preview_enabled=False)
        self.addCleanup(video.close)
        image = self.bottle_image()
        session = video.begin_sumo_alignment_read(gyro_reader=lambda: 0, distance_reader=lambda: 0)
        for frame in (1, 2):
            video.frame_id = frame
            video._process_sumo_alignment(image.copy(), session, (frame*.1, 0, 0))
        old = video.get_sumo_alignment_observation(session)
        self.assertTrue(old['confirmed'])
        new = video.begin_sumo_alignment_read(gyro_reader=lambda: 0, distance_reader=lambda: 0,
                                              preserve_history=True)
        self.assertIsNone(video.get_sumo_alignment_observation(new))
        video.frame_id = 3
        video._process_sumo_alignment(image.copy(), new, (.3, 0, 0))
        result = video.get_sumo_alignment_observation(new)
        self.assertEqual(old['track_id'], result['track_id'])
        self.assertFalse(result['confirmed'])
        self.assertGreaterEqual(result['detector_duration_ms'], 0)
        video.frame_id = 4
        video._process_sumo_alignment(image.copy(), new, (.4, 0, 0))
        self.assertTrue(video.get_sumo_alignment_observation(new)['confirmed'])

    def test_video_capture_process_publishes_metadata_and_confirmed_detection(self):
        with patch.object(Video, '_open_cap'):
            video = Video(preview_enabled=False)
        self.addCleanup(video.close)
        frame = np.full((480, 640, 3), 240, np.uint8)
        frame[140:200, 300:340] = 0
        video.cap = Mock()
        video.cap.read.side_effect = lambda: (True, frame.copy())
        session = video.begin_sumo_alignment_read(gyro_reader=lambda: 12.5, distance_reader=lambda: 321)
        with patch.object(Video, '_open_cap'), patch.object(target.time, 'monotonic', return_value=10) as clock:
            video.process(None, None, None, None, None, None, None, None)
            first = video.get_sumo_alignment_observation(session)
            self.assertFalse(first['confirmed'])
            clock.return_value = 10.2
            video.process(None, None, None, None, None, None, None, None)
        second = video.get_sumo_alignment_observation(session)
        self.assertTrue(second['confirmed'])
        self.assertEqual(second['gyro_deg'], 12.5)
        self.assertEqual(second['d'], 321)
        self.assertEqual(second['t'], 10.2)
        video.begin_bottle_read()
        self.assertIsNone(video.get_sumo_alignment_observation(session))

    def test_confirmation_loss_duplicate_and_reacquisition(self):
        detector = SumoBlackDetector()
        image = np.full((180, 320, 3), 240, np.uint8)
        image[40:70, 150:170] = 0
        self.assertFalse(detector.step(image, 0, 0, 1)['confirmed'])
        self.assertIsNone(detector.step(image, 0, .1, 1))
        self.assertTrue(detector.step(image, 0, .2, 2)['confirmed'])
        blank = np.full_like(image, 240)
        detector.step(blank, 0, .3, 3)
        self.assertIsNone(detector.step(blank, 0, .35, 3))
        self.assertTrue(detector.locked)
        detector.step(blank, 0, .4, 4)
        self.assertFalse(detector.locked)
        self.assertFalse(detector.step(image, 0, .5, 5)['confirmed'])
        self.assertTrue(detector.step(image, 0, .6, 6)['confirmed'])

    def test_video_read_generation_rejects_old_publish(self):
        with patch.object(Video, '_open_cap'):
            video = Video(preview_enabled=False)
        self.addCleanup(video.close)
        first = video.begin_sumo_alignment_read(gyro_reader=lambda: 0, distance_reader=lambda: 0)
        image = np.full((180, 320, 3), 240, np.uint8)
        image[40:70, 150:170] = 0
        video.frame_id = 1
        video._process_sumo_alignment(image.copy(), first, (1.0, 0, 0))
        self.assertIsNotNone(video.get_sumo_alignment_observation(first))
        second = video.begin_sumo_alignment_read(gyro_reader=lambda: 0, distance_reader=lambda: 0)
        video.frame_id = 2
        video._process_sumo_alignment(image.copy(), first, (1.2, 0, 0))
        self.assertIsNone(video.get_sumo_alignment_observation(first))
        self.assertIsNone(video.get_sumo_alignment_observation(second))
        video.frame_id = 3
        video._process_sumo_alignment(image.copy(), second, (1.4, 0, 0))
        self.assertFalse(video.get_sumo_alignment_observation(second)['confirmed'])

    def test_mission_is_isolated_and_uses_camera_without_qr(self):
        from robot_program.tree_builder import build_mission_children
        c = config_for_mission('sumo-align')
        self.assertTrue(mission_requires_camera(c))
        self.assertFalse(mission_requires_qr(c))
        nodes = build_mission_children(RaceContext(), c)
        self.assertEqual(len(nodes), 2)
        self.assertIsInstance(nodes[-1], target.AlignSumoBottle)
        self.assertEqual(initial_sumo_bearing('sumo-align'), 0)
        self.assertEqual(initial_sumo_bearing('sumo-align', 270), 270)

    def test_parameter_validation(self):
        for kwargs in (dict(distance_mm=0), dict(distance_mm=float('nan')), dict(power=101),
                       dict(power=True), dict(stable_frames=1), dict(target_cx=320)):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                SumoAlignSettings(**kwargs)

    def test_alpha_cli_check_tree_keeps_calibration_and_isolates_alignment(self):
        import contextlib
        import io
        script = (OPTIONS.overlay or REPO) / 'alpha.py'
        if not script.is_file():
            script = REPO / 'alpha.py'
        spec = importlib.util.spec_from_file_location('alpha_sumo_align_check', script)
        alpha = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(alpha)
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            result = alpha.main(['left', '--mission', 'sumo-align', '--check-tree',
                                 '--sumo-distance-mm', '1000', '--sumo-power', '80'])
        self.assertEqual(result, 0)
        text = output.getvalue()
        for part in ('arm up', 'arm down', 'device reset', 'touch start',
                     'approach and face sumo bottle only'):
            self.assertIn(part, text)
        for part in ('move to sumo start', 'capture and push', 'move to sumo exit'):
            self.assertNotIn(part, text)


if __name__ == '__main__':
    unittest.main(argv=[sys.argv[0]] + TEST_ARGS)
