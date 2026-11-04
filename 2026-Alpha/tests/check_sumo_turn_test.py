"""Opt-in turn measurement tests using real feature/recorder and fake hardware."""
import argparse
import contextlib
from dataclasses import replace
import importlib.util
import io
import json
from pathlib import Path
import runpy
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np

parser=argparse.ArgumentParser(add_help=False)
parser.add_argument('--repository',type=Path,default=Path(__file__).resolve().parents[1])
options,_=parser.parse_known_args()
BOOT=runpy.run_path(str(options.repository/'tests/check_sumo_align.py'))
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from robot_program.config import config_for_mission,mission_requires_camera,mission_requires_qr
from robot_program.context import RaceContext
from robot_program.runtime import runtime
from robot_program.diagnostics import sumo_turn_test as target
from robot_program.diagnostics.sumo_turn_test_settings import SumoTurnTestSettings
from robot_program.features.sumo_bearing import initial_sumo_bearing
from robot_program.features.sumo_align_recording import SumoAlignRecording


class Video:
    def __init__(self):
        self.state=dict(ready=True,finished=True,error=None)
        self.details=None;self.ends=[];self.reads=0
    def begin_sumo_alignment_read(self,**kwargs):self.reads+=1;return self.reads
    def end_sumo_alignment_read(self,session):self.ends.append(session)
    def begin_sumo_alignment_recording(self,reader,**kwargs):self.reader=reader
    def get_sumo_alignment_recording_status(self):return self.state
    def end_sumo_alignment_recording(self,result,reason,**kwargs):
        if kwargs.get('details') is not None:self.details=kwargs['details']


class FakeTurn(Behaviour):
    status_to_return=Status.RUNNING
    def __init__(self,name,context,bearing,**kwargs):
        super().__init__(name);self.bearing=bearing;self.phase=1
    def update(self):
        runtime.left_motor.set_power(50);runtime.right_motor.set_power(-50)
        return self.status_to_return
    def terminate(self,status):
        runtime.left_motor.set_power(0);runtime.right_motor.set_power(0)


class TurnTests(unittest.TestCase):
    def setUp(self):
        self.now=0.;self.gyro=0.
        self.left,self.right=BOOT['IO'](),BOOT['IO']()
        self.left.get_count=lambda:0;self.right.get_count=lambda:0
        runtime.left_motor,runtime.right_motor=self.left,self.right
        runtime.gyro_sensor=Mock(get_angle=lambda:self.gyro)
        runtime.plotter=Mock(get_distance=lambda:0)
        runtime.video=self.video=Video();runtime.course=1
        self.context=RaceContext();self.context.sumo.bearing_reference.register(0,0)
        self.config=config_for_mission('sumo-turn-test')
        self.settings=replace(self.config.sumo_turn_test,baseline_sec=.2,hold_sec=.5)
        self.clock=patch.object(target.time,'monotonic',side_effect=lambda:self.now);self.clock.start();self.addCleanup(self.clock.stop)
        FakeTurn.status_to_return=Status.RUNNING
        self.turn_patch=patch.object(target,'EncoderSpinToBearing',FakeTurn);self.turn_patch.start();self.addCleanup(self.turn_patch.stop)
        self.node=target.SumoTurnTest('test',self.context,self.settings,self.config.sumo)

    def tick(self,t,gyro=None):
        self.now=t
        if gyro is not None:self.gyro=gyro
        self.node.tick_once()

    def start_turn(self):
        for t in (0,.05,.1,.15,.2,.25):self.tick(t)
        self.assertEqual(self.node.phase,'TURN')

    def test_no_motion_before_camera_and_stable_baseline(self):
        self.video.state['ready']=False
        self.tick(0);self.tick(.5)
        self.assertEqual(self.left.power,0);self.assertEqual(self.right.power,0)
        self.assertIsNone(self.node.turn)

    def test_never_creates_straight_drive_and_restores_motor_references(self):
        self.start_turn()
        self.assertIs(runtime.left_motor,self.left);self.assertIs(runtime.right_motor,self.right)
        powers=[c for c in self.node.commands if c['kind']=='power' and c['value']]
        self.assertTrue(powers)
        self.assertFalse(hasattr(self.node,'drive'))

    def test_post_hold_measures_rotation_after_completion(self):
        self.start_turn();FakeTurn.status_to_return=Status.SUCCESS
        self.tick(.3,5)
        self.assertEqual(self.node.phase,'POST_HOLD')
        for t in (.35,.4,.45,.5,.55,.6,.65,.7,.75,.8,.85,.9):
            self.tick(t,7)
            if self.node.status!=Status.RUNNING:break
        self.assertEqual(self.node.status,Status.SUCCESS)
        self.assertAlmostEqual(self.video.details['actual_angle_deg'],7)
        self.assertAlmostEqual(self.video.details['rotation_after_completion_deg'],2)
        self.assertAlmostEqual(self.video.details['final_error_deg'],2)
        self.assertTrue(self.video.details['sensor_settled_at_end'])
        self.assertFalse(self.context.sumo.bottle_aligned)
        self.assertEqual(self.left.power,0);self.assertEqual(self.right.power,0)

    def test_turn_timeout_stops_both_motors(self):
        self.start_turn();self.tick(5)
        self.tick(5.1)
        self.assertEqual(self.node.status,Status.FAILURE)
        self.assertEqual(self.left.power,0);self.assertEqual(self.right.power,0)
        self.assertIn('timeout',self.context.sumo.failure_reason)

    def test_camera_timeout_never_starts_turn(self):
        self.video.state['ready']=False;self.tick(0);self.tick(6);self.tick(6.1)
        self.assertEqual(self.node.status,Status.FAILURE)
        self.assertIsNone(self.node.start_gyro)

    def test_excursion_limit_stops_instead_of_chasing(self):
        self.start_turn();self.tick(.3,30);self.tick(.4,30)
        self.assertEqual(self.node.status,Status.FAILURE)
        self.assertIn('excursion',self.context.sumo.failure_reason)
        self.assertEqual(self.left.power,0)

    def test_exception_during_child_tick_restores_motor_references(self):
        self.start_turn()
        with self.assertRaises(RuntimeError):
            self.node._with_probes(lambda:(_ for _ in ()).throw(RuntimeError('test')))
        self.assertIs(runtime.left_motor,self.left);self.assertIs(runtime.right_motor,self.right)
        self.node.stop(Status.INVALID)
        self.assertEqual(self.left.power,0);self.assertEqual(self.right.power,0)

    def test_command_trace_contains_api_values_and_timestamps(self):
        self.start_turn()
        self.assertTrue(all('t' in c and 'phase' in c and 'kind' in c for c in self.node.commands))
        self.assertTrue(any(c['motor']=='left' and c['kind']=='power' and c['value']==50 for c in self.node.commands))
        self.assertIn('test_control_gyro_deg',self.video.reader())

    def test_unsettled_post_hold_is_failure(self):
        self.start_turn();FakeTurn.status_to_return=Status.SUCCESS;self.tick(.3,5)
        for i,t in enumerate((.35,.4,.45,.5,.55,.6,.65,.7,.75,.8,.85,.9)):
            self.tick(t,5+(i%2)*4)
            if self.node.phase=='FINALIZING':break
        self.tick(1,5)
        self.assertEqual(self.node.status,Status.FAILURE)
        self.assertIn('not settled',self.context.sumo.failure_reason)

    def test_real_adapter_targets_clockwise_on_both_courses(self):
        from robot_program.features.sumo_bearing_motion import EncoderSpinToBearing
        for course in (-1,1):
            for angle in (-5,5):
                runtime.course=course
                turn=EncoderSpinToBearing('real',self.context,angle%360,max_power=75,min_power=60,
                                         pid_p=.4,pid_i=.001,pid_d=.03,tolerance=1)
                turn.tick_once()
                self.assertAlmostEqual(turn.target_motor_deg,5*117/56.87)
                self.assertGreater(self.left.power*angle,0);self.assertLess(self.right.power*angle,0)
                turn.stop(Status.INVALID)

    def test_interruption_keeps_trace_and_stops(self):
        self.start_turn();self.node.stop(Status.INVALID)
        self.assertEqual(self.left.power,0);self.assertEqual(self.right.power,0)
        self.assertTrue(self.video.details['control_samples'])
        self.assertTrue(self.video.ends)

    def test_unstable_baseline_refuses_to_turn(self):
        for i in range(46):
            self.tick(i*.05,(i%2)*4)
            if self.node.phase=='FINALIZING':break
        self.assertEqual(self.node.phase,'FINALIZING')
        self.assertIsNone(self.node.start_gyro)
        self.assertEqual(self.left.power,0)

    def test_recording_error_stops_active_turn(self):
        self.start_turn();self.video.state['error']='disk full';self.tick(.3);self.tick(.4)
        self.assertEqual(self.node.status,Status.FAILURE)
        self.assertEqual(self.left.power,0);self.assertEqual(self.right.power,0)


class IntegrationTests(unittest.TestCase):
    @staticmethod
    def stability_samples(times,gyros=None):
        return [dict(t=t,gyro_deg=gyros[i] if gyros else 0,left_count=0,right_count=0)
                for i,t in enumerate(times)]

    def test_jittered_window_covers_boundary_instead_of_false_failure(self):
        samples=self.stability_samples([0,.06,.13,.21,.24,.27,.30,.34])
        window=target.sensor_stability(samples)
        self.assertTrue(window['settled'])
        self.assertGreaterEqual(window['coverage_sec'],.2)

    def test_boundary_sample_with_motion_is_not_discarded(self):
        samples=self.stability_samples([0,.06,.13,.21,.24,.27,.30,.34],[0,0,2,0,0,0,0,0])
        self.assertFalse(target.sensor_stability(samples)['settled'])
        self.assertGreater(target.sensor_stability(samples)['gyro_span_deg'],1.1)

    def test_sparse_measurements_do_not_prove_stability(self):
        samples=self.stability_samples([0,.5,.52,.54])
        window=target.sensor_stability(samples)
        self.assertFalse(window['settled'])
        self.assertFalse(window['time_coverage_ok'])

    def test_settings_reject_invalid_values(self):
        for kwargs in (dict(angle_deg=0),dict(angle_deg=21),dict(angle_deg=float('nan')),
                       dict(power=True),dict(power=81),dict(power=19),dict(hold_sec=.1),dict(hold_sec=float('inf'))):
            with self.subTest(kwargs=kwargs),self.assertRaises(ValueError):SumoTurnTestSettings(**kwargs)

    def test_mission_isolated_camera_enabled_without_qr(self):
        from robot_program.tree_builder import build_mission_children
        c=config_for_mission('sumo-turn-test')
        nodes=build_mission_children(RaceContext(),c)
        self.assertEqual(len(nodes),2);self.assertIsInstance(nodes[-1],target.SumoTurnTest)
        self.assertTrue(mission_requires_camera(c));self.assertFalse(mission_requires_qr(c))
        self.assertEqual(initial_sumo_bearing('sumo-turn-test'),0)

    def alpha(self):
        script=(BOOT['OPTIONS'].overlay or BOOT['REPO'])/'alpha.py'
        if not script.is_file():script=BOOT['REPO']/'alpha.py'
        spec=importlib.util.spec_from_file_location('alpha_turn_check',script);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        return module

    def test_alpha_preserves_original_calibration_and_touch_sequence(self):
        with contextlib.redirect_stdout(io.StringIO()) as out:
            result=self.alpha().main(['right','--mission','sumo-turn-test','--check-tree','--sumo-turn-angle','-5','--sumo-turn-power','50'])
        self.assertEqual(result,0)
        for text in ('arm up','arm down','device reset','touch start','sumo turn measurement only'):self.assertIn(text,out.getvalue())
        self.assertNotIn('approach and face sumo bottle only',out.getvalue())

    def test_test_flags_cannot_change_normal_alignment(self):
        with contextlib.redirect_stderr(io.StringIO()),contextlib.redirect_stdout(io.StringIO()),self.assertRaises(SystemExit) as error:
            self.alpha().main(['right','--mission','sumo-align','--check-tree','--sumo-turn-angle','5'])
        self.assertEqual(error.exception.code,2)

    def test_recorder_preserves_diagnostics_after_duplicate_stop(self):
        with tempfile.TemporaryDirectory(prefix='sumo-turn-test-') as tmp:
            recorder=SumoAlignRecording(output_dir=tmp)
            raw=np.full((480,640,3),220,np.uint8);det=cv2.resize(raw,(320,180))
            recorder.submit(raw,det,dict(frame_id=1,capture_monotonic_sec=1,phase='TURN'))
            recorder.request_stop('SUCCESS','measured',details=dict(kind='sumo-turn-test',actual_angle_deg=7))
            recorder.request_stop('INTERRUPTED','later')
            recorder.close()
            data=json.loads(recorder.paths['summary'].read_text(encoding='utf-8'))
            self.assertEqual(data['result'],'SUCCESS');self.assertEqual(data['details']['actual_angle_deg'],7)


if __name__=='__main__':unittest.main(argv=[sys.argv[0]]+BOOT['TEST_ARGS'])
