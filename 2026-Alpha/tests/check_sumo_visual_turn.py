"""PC policy/IO tests. Plant tests are synthetic, not field success evidence."""
import math
import unittest
from dataclasses import replace
from unittest.mock import Mock, patch
import check_sumo_align as boot
from robot_program.features.sumo_visual_policy import VisualTurnPolicy, VisualTurnMemory
from robot_program.features.sumo_visual_turn import VisualBottleTurn


class PolicyTests(unittest.TestCase):
    def setUp(self):
        self.s = boot.SumoAlignSettings()
        self.m = VisualTurnMemory(self.s)
        self.p = VisualTurnPolicy(self.s, self.m, 1, 0., 0.)

    def obs(self, t, cx=220, gyro=0, **kw):
        result = dict(session=1, frame_id=int(t*1000), t=t, gyro_deg=gyro,
                      candidate=dict(cx=cx), confirmed=True, track_id=1, mask_source='value')
        result.update(kw)
        return result

    def ready(self, cx=220):
        for i in range(8):
            t = i*.02
            row = self.p.step(t, 0., self.obs(t, cx))
        return row

    def test_far_fast_near_slow(self):
        self.assertEqual(self.ready(300)['visual_power'], 75)
        self.setUp()
        self.assertEqual(self.ready(220)['visual_power'], 50)

    def test_capture_pose_compensation(self):
        row = self.p.step(.10, 3., self.obs(.08, cx=220, gyro=1))
        self.assertAlmostEqual(row['visual_error_px'], 45)

    def test_predictive_brake_precedes_center(self):
        self.ready(300)
        self.p.step(.16, 1., self.obs(.16, 230, 1))
        row = self.p.step(.18, 2., self.obs(.18, 200, 2))
        self.assertGreater(row['visual_error_px'], self.s.center_tolerance_px)
        self.assertEqual(row['visual_power'], 0)

    def test_loss_stale_future_nan_unconfirmed_and_session_brake(self):
        for replacement in (None, self.obs(-1), self.obs(2), self.obs(.16, cx=math.nan),
                            self.obs(.16, confirmed=False), self.obs(.16, session=2)):
            self.setUp()
            self.ready(300)
            row = self.p.step(.16, 0, replacement)
            self.assertEqual(row['visual_power'], 0)
        self.assertEqual(self.p.step(.9, 0, None)['visual_stage'], 'FAILED')

    def test_identity_change_fails(self):
        self.ready()
        row = self.p.step(.16, 0, self.obs(.16, track_id=2))
        self.assertEqual(row['visual_stage'], 'FAILED')
        self.assertEqual(row['visual_power'], 0)

    def test_no_reverse_before_quiet(self):
        self.ready(300)
        row = self.p.step(.16, 1, self.obs(.16, 130, 1))
        self.assertEqual(row['visual_power'], 0)
        row = self.p.step(.18, 2, self.obs(.18, 120, 2))
        self.assertEqual(row['visual_power'], 0)

    def test_stopped_sign_flip_shrinks_and_disables_fast(self):
        self.m.observe(0, 100, self.obs(0, 260))
        self.m.observe(15, -25, self.obs(1, 135, 15))
        self.assertEqual(self.m.reversals, 1)
        self.assertLessEqual(self.m.limit(15), 7.5)
        self.p = VisualTurnPolicy(self.s, self.m, 1, 0, 15)
        for i in range(9):
            t = i*.02
            row = self.p.step(t, 15, self.obs(t, 20, 15))
            self.assertNotEqual(abs(row['visual_power']), 75)

    def test_source_change_discards_bracket(self):
        self.m.observe(0, 100, self.obs(0))
        self.m.observe(15, -25, self.obs(1))
        self.m.observe(14, 30, self.obs(2, mask_source='hsv'))
        self.assertIsNone(self.m.upper)
        self.assertEqual(self.m.reversals, 1)

    def test_duplicate_stationary_frame_does_not_reverse_twice(self):
        self.m.observe(0, 100, self.obs(0))
        observation = self.obs(1)
        self.m.observe(15, -25, observation)
        self.m.observe(14, 30, observation)
        self.assertEqual(self.m.reversals, 1)

    def test_repeated_prebrake_image_cannot_finish(self):
        self.ready(300)
        self.p.step(.16, 0, self.obs(.16, 160))
        observation = self.obs(.14, 160)
        for i in range(9, 18):
            row = self.p.step(i*.02, 0, observation)
            self.assertNotEqual(row['visual_stage'], 'DONE')

    def test_fresh_center_finishes_only_after_quiet(self):
        self.ready(160)
        for i in range(8, 18):
            t = i*.02
            row = self.p.step(t, 0, self.obs(t, 160))
        self.assertEqual(row['visual_stage'], 'DONE')
        self.assertEqual(row['visual_power'], 0)

    def test_fine_pulse_is_bounded_and_never_kicks(self):
        self.ready(180)
        powers = []
        for i in range(8, 35):
            t=i*.02
            powers.append(self.p.step(t, 0, self.obs(t, 180))['visual_power'])
        self.assertIn(0, powers)
        self.assertIn(50, powers)
        self.assertNotIn(75, powers)

    def test_kick_has_bounded_duration(self):
        self.ready(220)
        kicked = []
        for i in range(8, 24):
            t=i*.02
            if self.p.step(t, 0, self.obs(t, 220))['visual_power'] == 75:
                kicked.append(t)
        self.assertTrue(kicked)
        self.assertLessEqual(max(kicked)-min(kicked), self.s.visual_kick_sec+.001)

    def test_turn_travel_and_timeout_bounds(self):
        self.ready(300)
        row=self.p.step(.16, 29, self.obs(.16, 300, 29))
        self.assertEqual(row['visual_stage'], 'FAILED')
        self.setUp()
        self.assertEqual(self.p.step(4.1, 0, self.obs(4.1))['visual_stage'], 'FAILED')

    def test_invalid_settings(self):
        for kw in (dict(visual_fast_power=101), dict(visual_max_image_age_sec=2),
                   dict(visual_px_per_degree=0), dict(visual_max_pulses=2.5)):
            with self.assertRaises(ValueError):
                replace(self.s, **kw)

    def test_synthetic_closed_loop_with_delay_and_coast(self):
        # Camera 25Hz, 40ms read delay; motor speed has first-order response.
        # Assert bounded commands/stopped convergence on explicitly modeled plants.
        for gain in (6., 7.5, 9.):
            for target in (-14., -3., 3., 14.):
                with self.subTest(gain=gain, target=target):
                    memory=VisualTurnMemory(self.s)
                    policy=VisualTurnPolicy(self.s, memory, 1, 0, 0)
                    angle=velocity=0.
                    poses=[(0., 0.)]
                    observation=None
                    final=None
                    for i in range(390):
                        t=i*.01
                        if i%4 == 0 and i>=4:
                            capture_t, capture_gyro=poses[-5]
                            observation=self.obs(capture_t, 160+(target-capture_gyro)*gain, capture_gyro)
                        final=policy.step(t, angle, observation)
                        power=final['visual_power']
                        desired=0 if not power else math.copysign(35 if abs(power)==50 else 65, power)
                        velocity+=(desired-velocity)*.01/(.035 if power else .025)
                        angle+=velocity*.01
                        poses.append((t, angle))
                        if final['visual_stage'] in ('DONE','FAILED'):
                            break
                    self.assertEqual(final['visual_stage'], 'DONE', final)
                    self.assertLessEqual(abs((target-angle)*gain), 10.)


class AdapterTests(unittest.TestCase):
    def setUp(self):
        self.s=boot.SumoAlignSettings()
        self.left,self.right=boot.IO(),boot.IO()
        boot.runtime.left_motor,boot.runtime.right_motor=self.left,self.right
        boot.runtime.gyro_sensor=Mock(get_angle=lambda:0.)
        boot.runtime.video=Mock(get_sumo_alignment_observation=lambda session:None)
        self.node=VisualBottleTurn('test', None, 10, 1, self.s, VisualTurnMemory(self.s))

    def test_physical_clockwise_not_course_mirrored(self):
        for course in (-1,1):
            boot.runtime.course=course
            self.node.last_power=None
            self.node._command(50)
            self.assertEqual((self.left.power,self.right.power),(50,-50))
            self.node._command(-50)
            self.assertEqual((self.left.power,self.right.power),(-50,50))

    def test_terminate_brakes_both_wheels(self):
        self.node._command(50)
        self.node.terminate(boot.Status.INVALID)
        self.assertEqual((self.left.power,self.right.power),(0,0))
        self.assertTrue(self.left.brake and self.right.brake)

    def test_brake_attempts_right_even_when_left_fails(self):
        self.left.set_power=Mock(side_effect=RuntimeError('test'))
        with self.assertRaises(RuntimeError):
            self.node._command(0)
        self.assertEqual(self.right.power,0)
        self.assertTrue(self.right.brake)

    def test_motor_exception_brakes_other_wheel(self):
        self.right.set_power=Mock(side_effect=RuntimeError('test'))
        with self.assertRaises(RuntimeError):
            self.node._command(50)
        self.assertEqual(self.left.power,0)
        self.assertTrue(self.left.brake)

    def test_telemetry_csv_and_details_are_serializable(self):
        import json
        from robot_program.features.sumo_align_recording import FIELDS
        self.node.initialise()
        self.node.update()
        self.assertTrue(set(self.node.telemetry).issubset(FIELDS))
        json.dumps(self.node.result_details(), allow_nan=False)

    def test_normal_align_tree_uses_visual_turn_and_stops(self):
        for course in (-1, 1):
            fixture=boot.AlignTests('test_distance_stops_before_another_forward_output')
            fixture.setUp()
            try:
                boot.runtime.course=course
                fixture.start_observing()
                velocity=0.
                for i in range(1, 350):
                    now=.5+i*.02
                    fixture.now.return_value=now
                    fixture.observation(100+i, cx=160+(8-fixture.gyro)*7.5)
                    fixture.node.tick_once()
                    if fixture.node.phase=='TURN':
                        self.assertIsInstance(fixture.node.turn, VisualBottleTurn)
                    power=boot.runtime.left_motor.power
                    desired=math.copysign(35 if abs(power)==50 else 65, power) if power else 0
                    velocity+=(desired-velocity)*.02/(.04 if power else .03)
                    fixture.gyro+=velocity*.02
                    if fixture.node.status in (boot.Status.SUCCESS, boot.Status.FAILURE):
                        break
                self.assertEqual(fixture.node.status, boot.Status.SUCCESS)
                self.assertTrue(fixture.context.sumo.bottle_aligned)
                self.assertTrue(fixture.node.completed_visual_turns)
                fixture.assert_stopped()
                self.assertFalse(fixture.context.sumo.bottle_captured)
                self.assertFalse(fixture.context.sumo.bottle_pushed_out)
            finally:
                fixture.doCleanups()


if __name__=='__main__':
    unittest.main(argv=[__file__]+boot.TEST_ARGS)
