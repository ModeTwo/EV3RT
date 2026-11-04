"""Real align/drive/CLI with fake IO: no robot moves during these PC checks."""
import contextlib
import argparse
from dataclasses import replace
import importlib.util
import io
import json
import tempfile
from pathlib import Path
import unittest
import sys
parser=argparse.ArgumentParser(add_help=False)
parser.add_argument('--repository',type=Path,default=Path(__file__).resolve().parents[1])
options,_=parser.parse_known_args()
sys.path.insert(0,str(options.repository/'tests'))
import check_sumo_align as boot
from robot_program.features.catch_sumo_bottle_test import AlignAndCatchTest
from robot_program.features.sumo_catch_settings import SumoCatchSettings
from robot_program.features.sumo_align_recording import SumoAlignRecording, FIELDS
from robot_program.config import config_for_mission,mission_requires_camera,mission_requires_qr
from robot_program.tree_builder import build_mission_children
from robot_program.features.sumo_bearing import initial_sumo_bearing


class Video(boot.FakeVideo):
    def __init__(self):
        super().__init__()
        self.details=None
    def end_sumo_alignment_recording(self,result,reason,**kwargs):
        super().end_sumo_alignment_recording(result,reason,**kwargs)
        if self.details is None:self.details=kwargs.get('details')


class CatchTests(unittest.TestCase):
    def setUp(self):
        self.f=boot.AlignTests('test_distance_stops_before_another_forward_output')
        self.f.setUp()
        self.addCleanup(self.f.doCleanups)
        self.f.video=Video()
        boot.runtime.video=self.f.video
        self.catch=SumoCatchSettings(distance_mm=50)
        self.f.node=AlignAndCatchTest('catch test',self.f.context,self.f.settings,
                                     self.f.config.sumo,self.catch)

    def ready(self):
        self.f.start_observing()
        for t,i in ((.6,10),(.64,11)):
            self.f.now.return_value=t
            self.f.observation(i)
            self.f.node.tick_once()
        self.assertEqual(self.f.node.phase,'CATCH_DRIVE')

    def drive(self):
        self.ready()
        for t,i in ((.66,12),(.68,13)):
            self.f.now.return_value=t
            self.f.observation(i)
            self.f.node.tick_once()
        self.assertEqual(self.f.node.phase,'CATCH_DRIVE')

    def test_normal_alignment_finishes_without_catch(self):
        nodes=build_mission_children(self.f.context,self.f.config)
        self.assertNotIsInstance(nodes[-1],AlignAndCatchTest)

    def test_recording_continues_from_align_into_catch(self):
        self.ready()
        self.assertTrue(self.f.context.sumo.bottle_aligned)
        self.assertEqual(self.f.video.record_ends,[])
        self.assertFalse(self.f.context.sumo.bottle_captured)

    def test_no_additional_camera_frames_required_after_alignment(self):
        self.ready()
        self.f.video.get_sumo_alignment_observation=lambda session: (_ for _ in ()).throw(AssertionError('post-align camera read'))
        self.f.now.return_value=.66
        self.f.node.tick_once()
        self.assertEqual(self.f.node.phase,'CATCH_DRIVE')
        self.assertGreater(boot.runtime.left_motor.power,0)

    def test_offset_image_does_not_gate_approach(self):
        self.ready()
        self.f.now.return_value=.66;self.f.observation(12,cx=300);self.f.node.tick_once()
        self.assertEqual(self.f.node.phase,'CATCH_DRIVE')
        self.assertGreater(boot.runtime.left_motor.power,0)

    def test_changed_image_identity_is_diagnostic_only_after_alignment(self):
        self.ready()
        self.f.now.return_value=.66;self.f.observation(12)
        self.f.video.observation['track_id']=999
        self.f.node.tick_once()
        self.assertEqual(self.f.node.phase,'CATCH_DRIVE')
        self.assertEqual(self.f.node.catch_track,1)
        self.assertGreater(boot.runtime.left_motor.power,0)

    def test_drive_timeout_has_priority_over_next_motor_tick(self):
        self.f.node.catch_settings=replace(self.catch,stall_timeout_sec=10)
        self.drive()
        self.f.now.return_value=8.8;self.f.node.tick_once()
        self.assertEqual(self.f.node.catch_stop_reason,'catch-drive-timeout')
        self.f.assert_stopped()

    def test_alignment_failure_cannot_enter_catch(self):
        self.f.start_observing()
        self.f.now.return_value=4.
        self.f.video.observation=None
        self.f.node.tick_once()
        self.assertEqual(self.f.node.status,boot.Status.FAILURE)
        self.assertFalse(self.f.node.catch_started)
        self.f.assert_stopped()

    def test_distance_stops_before_additional_forward_tick(self):
        self.drive()
        left=boot.runtime.left_motor
        marker=len(left.outputs)
        self.f.d=150
        self.f.now.return_value=.8
        self.f.node.tick_once()
        self.assertTrue(all(p==0 for p in left.outputs[marker:]))
        self.assertEqual(self.f.node.phase,'CATCH_HOLD')
        self.f.assert_stopped()

    def test_both_courses_hold_the_post_align_heading(self):
        for course in (-1,1):
            if course==1:
                self.f.node.stop(boot.Status.INVALID)
                self.f.now.return_value=0;self.f.d=0
            boot.runtime.course=course
            self.drive()
            self.assertEqual(boot.runtime.left_motor.power,25)
            self.assertEqual(boot.runtime.right_motor.power,25)

    def test_stop_hold_retains_recording_and_coast_measurement(self):
        self.drive()
        self.f.d=150;self.f.now.return_value=.8;self.f.node.tick_once()
        self.assertEqual(self.f.video.record_ends,[])
        self.f.d=153;self.f.now.return_value=1.3;self.f.node.tick_once()
        self.assertEqual(self.f.video.record_ends,[])
        self.f.now.return_value=1.81;self.f.node.tick_once()
        self.assertEqual(self.f.node.status,boot.Status.SUCCESS)
        self.assertFalse(self.f.context.sumo.bottle_captured)
        self.assertFalse(self.f.context.sumo.bottle_pushed_out)
        details=self.f.video.details
        self.assertEqual(details['kind'],'sumo-catch-test-v3')
        self.assertEqual(details['catch']['stop_progress_mm'],50)
        self.assertEqual(details['catch']['progress_mm'],53)
        self.assertFalse(details['catch']['physical_capture_confirmed'])
        json.dumps(details,allow_nan=False)

    def test_missing_and_stale_images_do_not_stop_but_heading_still_does(self):
        self.drive()
        self.f.now.return_value=.8;self.f.video.observation=None;self.f.node.tick_once()
        self.assertEqual(self.f.node.phase,'CATCH_DRIVE')
        self.assertGreater(boot.runtime.left_motor.power,0)
        self.f.now.return_value=.82;self.f.observation(14,cx=300,age=10)
        self.f.video.observation['confirmed']=False
        self.f.node.tick_once()
        self.assertEqual(self.f.node.phase,'CATCH_DRIVE')
        self.f.gyro=5;self.f.now.return_value=.84;self.f.node.tick_once()
        self.assertEqual(self.f.node.catch_stop_reason,'catch-heading-error')
        self.f.assert_stopped()

    def test_stall_stops_without_power_boost(self):
        self.drive();self.f.now.return_value=1.7;self.f.observation(14);self.f.node.tick_once()
        self.assertEqual(self.f.node.catch_stop_reason,'catch-no-progress')
        self.f.assert_stopped()

    def test_negative_odometry_stops(self):
        self.drive();self.f.d=99;self.f.now.return_value=.8;self.f.node.tick_once()
        self.assertEqual(self.f.node.catch_stop_reason,'negative-approach-progress')
        self.f.assert_stopped()

    def test_500mm_approach_continues_past_217mm_without_images(self):
        self.f.node.catch_settings=replace(self.catch,distance_mm=500)
        self.drive()
        self.f.video.observation=None
        self.f.d=317;self.f.now.return_value=.9;self.f.node.tick_once()
        self.assertEqual(self.f.node.catch_progress,217)
        self.assertEqual(self.f.node.phase,'CATCH_DRIVE')
        self.assertGreater(boot.runtime.left_motor.power,0)
        self.f.d=600;self.f.now.return_value=1.2;self.f.node.tick_once()
        self.assertEqual(self.f.node.phase,'CATCH_HOLD')
        self.assertEqual(self.f.node.catch_stop_progress,500)
        self.assertEqual(self.f.node.catch_result,boot.Status.SUCCESS)
        self.f.assert_stopped()

    def test_abort_brake_and_record_details_on_interrupt(self):
        self.drive();self.f.node.stop(boot.Status.INVALID)
        self.f.assert_stopped()
        self.assertEqual(self.f.video.details['catch']['outcome'],'interrupted-capture-unverified')

    def test_recorder_error_stops_immediately(self):
        self.drive();self.f.video.record_status['error']='disk failure';self.f.node.tick_once()
        self.assertEqual(self.f.node.status,boot.Status.FAILURE)
        self.f.assert_stopped()

    def test_invalid_pose_stops(self):
        self.drive();self.f.d=float('nan');self.f.node.tick_once()
        self.assertEqual(self.f.node.status,boot.Status.FAILURE)
        self.f.assert_stopped()

    def test_settings_and_missing_distance(self):
        for options in (dict(distance_mm=0),dict(distance_mm=-1),dict(distance_mm=float('inf')),dict(distance_mm=float('nan')),
                        dict(power=0),dict(power=True),dict(power=101),dict(post_record_sec=.1)):
            with self.assertRaises(ValueError):SumoCatchSettings(**options)
        with self.assertRaises(ValueError):SumoCatchSettings().require_distance()

    def test_positive_distance_has_no_upper_cap(self):
        for value in (400,10000,.5):
            self.assertEqual(SumoCatchSettings(distance_mm=value).distance_mm,value)
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(self.alpha().main(['left','--mission','sumo-catch-test',
                '--sumo-catch-distance-mm','400','--check-tree']),0)

    def test_motor_power_cap_is_device_limit_not_40(self):
        for power in (41,80,100):
            self.assertEqual(SumoCatchSettings(distance_mm=500,power=power).power,power)
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(self.alpha().main(['left','--mission','sumo-catch-test',
                '--sumo-catch-distance-mm','500','--sumo-catch-power','100','--check-tree']),0)
        for power in (80,100):
            self.f.node.catch_settings=replace(self.catch,power=power)
            self.drive()
            self.assertEqual(boot.runtime.left_motor.power,power)
            self.assertEqual(boot.runtime.right_motor.power,power)
            self.f.node.stop(boot.Status.INVALID)
            self.f.now.return_value=0;self.f.d=0

    def alpha(self):
        script=(boot.OPTIONS.overlay or boot.REPO)/'alpha.py'
        if not script.is_file():script=boot.REPO/'alpha.py'
        spec=importlib.util.spec_from_file_location('alpha_catch_check',script)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        return module

    def test_cli_preserves_initialization_and_feature_scope(self):
        with contextlib.redirect_stdout(io.StringIO()) as out:
            result=self.alpha().main(['left','--mission','sumo-catch-test','--sumo-catch-distance-mm','50','--check-tree'])
        self.assertEqual(result,0)
        for text in ('arm up','arm down','device reset','touch start','align and low-power catch test only'):
            self.assertIn(text,out.getvalue())
        self.assertNotIn('move to sumo exit',out.getvalue())
        config=replace(config_for_mission('sumo-catch-test'),sumo_catch=self.catch)
        self.assertTrue(mission_requires_camera(config));self.assertFalse(mission_requires_qr(config))
        self.assertEqual(initial_sumo_bearing('sumo-catch-test'),0)
        self.assertIsInstance(build_mission_children(self.f.context,config)[-1],AlignAndCatchTest)

    def test_cli_requires_explicit_distance_and_rejects_wrong_mission(self):
        for args in (['--mission','sumo-catch-test'],
                     ['--mission','sumo-align','--sumo-catch-distance-mm','50'],
                     ['--mission','sumo-catch-test','--sumo-catch-distance-mm','-1']):
            with contextlib.redirect_stderr(io.StringIO()),contextlib.redirect_stdout(io.StringIO()),self.assertRaises(SystemExit) as error:
                self.alpha().main(['left','--check-tree']+args)
            self.assertEqual(error.exception.code,2)

    def test_actual_writer_preserves_catch_csv_and_summary(self):
        import numpy as np
        import csv
        with tempfile.TemporaryDirectory(prefix='sumo-catch-check-') as directory:
            recorder=SumoAlignRecording(output_dir=directory)
            recorder.submit(np.zeros((480,640,3),np.uint8),np.zeros((180,320,3),np.uint8),
                dict(frame_id=1,capture_monotonic_sec=1,phase='CATCH_HOLD',catch_progress_mm=50))
            recorder.request_stop('SUCCESS','test-completed',details=dict(kind='sumo-catch-test-v1',physical_capture_confirmed=False))
            recorder.close()
            summary=json.loads(recorder.paths['summary'].read_text(encoding='utf-8'))
            self.assertEqual(summary['frames_written'],1)
            self.assertFalse(summary['details']['physical_capture_confirmed'])
            with recorder.paths['csv'].open(encoding='utf-8',newline='') as source:
                self.assertEqual(next(csv.DictReader(source))['catch_progress_mm'],'50')


if __name__=='__main__':unittest.main(argv=[__file__]+boot.TEST_ARGS)
