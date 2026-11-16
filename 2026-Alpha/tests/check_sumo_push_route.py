"""Synthetic geometry and real BT/drive with fake IO; no hardware movement."""
import sys,json,tempfile,importlib.util
from pathlib import Path
from dataclasses import replace,asdict
from unittest.mock import Mock
import unittest
import argparse
import contextlib,io
args,_=argparse.ArgumentParser(add_help=False).parse_known_args()
# Existing bootstrap supports --repository and --overlay and loads the real code.
repo_parser=argparse.ArgumentParser(add_help=False)
repo_parser.add_argument('--repository',type=Path,default=Path(__file__).resolve().parents[1])
opts,_=repo_parser.parse_known_args();sys.path.insert(0,str(opts.repository/'tests'))
import check_sumo_align as boot
from robot_program.features.sumo_push_geometry import PushCalibration,FloorPose,CaptureEvidence,clearance,plan_push,advance,alignment_push_policy
from robot_program.features.sumo_push_route import AlignAndPushRoute
from robot_program.config import config_for_mission,mission_requires_camera
from robot_program.tree_builder import build_mission_children


def calibration(**kwargs):
    # Synthetic values for tests only. Never exported as an operating calibration.
    d=dict(calibration_id='SYNTHETIC-NOT-FOR-ROBOT',course='left',calibrated=True,
      start_xy_mm=(0.,0.),start_bearing_deg=0.,ring_center_mm=(0.,0.),ring_radius_mm=500.,
      green_polygons_mm=((( -5000.,1000.),(5000.,1000.),(5000.,5000.),(-5000.,5000.)),),
      workspace_bounds_mm=(-6000.,6000.,-6000.,6000.),body_half_width_mm=50.,body_front_mm=60.,body_rear_mm=40.,
      held_bottle_offset_mm=80.,bottle_radius_mm=30.,hold_uncertainty_mm=5.,initial_position_error_mm=2.,heading_error_deg=2.,
      drift_mm_per_mm=.01,drift_mm_per_turn_deg=.1,braking_distance_mm=10.,max_speed_mm_s=100.,max_wheel_speed_mm_s=400.,
      max_turn_rate_deg_s=500.,max_cycle_sec=.2,mm_per_encoder_degree=1.,left_forward_sign=1,right_forward_sign=1,
      capture_max_mm=100.,push_max_mm=800.,push_out_margin_mm=10.,turn_power=50)
    d.update(green_bearing_deg=90.);d.update(kwargs);return PushCalibration(**d)


def pose(c):return FloorPose(c,*c.start_xy_mm,c.start_bearing_deg,0.,0.,0.,0.)


class Geometry(unittest.TestCase):
    def test_segment_crosses_thin_obstacle_between_endpoints(self):
        self.assertEqual(clearance((0,0),(100,0),(((49,-1),(51,-1),(51,1),(49,1)),)),0)
    def test_touching_is_not_safe(self):
        c=calibration();r=c.radius+c.stop_allowance
        self.assertFalse(c.safe_segment((0,1000-r),(0,1000-r),0))
    def test_safe_direct_has_no_turn(self):
        c=calibration();p=plan_push(c,pose(c));self.assertEqual(p['kind'],'DIRECT');self.assertGreater(p['distance_mm'],400)
    def test_green_near_top_redirects_into_white(self):
        c=calibration(green_polygons_mm=(((-5000,250),(5000,250),(5000,5000),(-5000,5000)),))
        p=plan_push(c,pose(c));self.assertIsNotNone(p);self.assertEqual(p['kind'],'REDIRECT');self.assertGreater(abs(p['turn_deg']),5)
    def test_no_feasible_route_never_falls_back(self):
        c=calibration(green_polygons_mm=(((-5000,-5000),(5000,-5000),(5000,5000),(-5000,5000)),))
        self.assertIsNone(plan_push(c,pose(c)))
    def test_body_only_collision(self):
        c=calibration(body_half_width_mm=1000.);self.assertFalse(c.safe_segment((0,0),(0,0),0))
    def test_bottle_offset_only_collision(self):
        c=calibration(held_bottle_offset_mm=1000.);self.assertFalse(c.safe_segment((0,0),(0,0),0))
    def test_more_uncertainty_rejects_route(self):
        c=calibration(initial_position_error_mm=1000.);self.assertIsNone(plan_push(c,pose(c)))
    def test_workspace_boundary_rejects(self):
        c=calibration(workspace_bounds_mm=(-100,100,-100,100));self.assertIsNone(plan_push(c,pose(c)))
    def test_uncalibrated_null_and_bad_polygon_rejected(self):
        for kw in [dict(calibrated=False),dict(body_front_mm=None),dict(max_speed_mm_s=float('nan')),
                   dict(green_polygons_mm=()),dict(green_polygons_mm=(((0,0),(2,2),(0,2),(2,0)),)),dict(left_forward_sign=0)]:
            with self.subTest(kw=kw),self.assertRaises((ValueError,TypeError)):calibration(**kw)
    def test_right_uses_explicit_floor_coordinates_without_second_mirror(self):
        a=calibration();b=calibration(course='right');self.assertEqual(plan_push(a,pose(a)),plan_push(b,pose(b)))
    def test_outside_means_whole_bottle_with_uncertainty(self):
        c=calibration();p=plan_push(c,pose(c));end=advance(p['end_xy_mm'],p['bearing_deg'],c.held_bottle_offset_mm)
        self.assertGreaterEqual(__import__('math').hypot(*end),c.ring_radius_mm+c.bottle_radius_mm+c.hold_uncertainty_mm+c.push_out_margin_mm)
    def test_encoder_pose_ignores_absolute_count_origin(self):
        c=calibration();p=pose(c);p.left=100;p.right=200;p.update(110,210,0,.1);self.assertEqual(p.signed_mm,10);self.assertEqual(p.y,10)
    def test_backward_encoder_is_signed(self):
        c=calibration();p=pose(c);p.update(-10,-10,0,.1);self.assertEqual(p.y,-10)
    def test_configured_encoder_polarity(self):
        c=calibration(left_forward_sign=-1);p=pose(c);p.update(-10,10,0,.1);self.assertEqual(p.y,10)
    def test_rotation_does_not_become_translation(self):
        c=calibration();p=pose(c);p.update(10,-10,20,.1);self.assertEqual(p.signed_mm,0);self.assertEqual(p.xy,(0,0))
    def test_cycle_and_encoder_jumps_rejected(self):
        c=calibration()
        for left,right,g,t in [(0,0,0,.3),(1000,0,0,.01),(0,0,100,.01),(float('nan'),0,0,.1)]:
            with self.subTest(t=t),self.assertRaises(ValueError):pose(c).update(left,right,g,t)
    def test_70_candidate_82_and_zero_not_capture(self):
        c=calibration();e=CaptureEvidence(c)
        for i in range(8):self.assertFalse(e.read(82,i*.05))
        for raw in [0,-1,True,None,float('nan'),'59']:
            for i in range(8):self.assertFalse(CaptureEvidence(c).read(raw,i*.05))
        e=CaptureEvidence(c);results=[e.read(59,i*.05) for i in range(7)]
        self.assertFalse(results[0]);self.assertTrue(results[-1])
    def test_invalid_breaks_capture_continuity(self):
        e=CaptureEvidence(calibration());e.read(59,0);e.read(59,.1);e.read(0,.15);self.assertFalse(e.read(59,.2))
    def test_long_sample_gap_breaks_continuity(self):
        e=CaptureEvidence(calibration());e.read(59,0);self.assertFalse(e.read(59,1))


class Mission(unittest.TestCase):
    def setUp(self):
        self.f=boot.AlignTests('test_distance_stops_before_another_forward_output');self.f.setUp();self.addCleanup(self.f.doCleanups)
        self.count=0.;boot.runtime.left_motor.get_count=lambda:self.count;boot.runtime.right_motor.get_count=lambda:self.count
        boot.runtime.sonar_sensor=Mock(get_distance=lambda:59)
        self.c=calibration()
        self.f.node=AlignAndPushRoute('green push',self.f.context,self.f.settings,self.f.config.sumo,self.c,True)
    def tick(self,t):self.f.now.return_value=t;self.f.node.tick_once()
    def aligned(self):
        self.tick(0);self.f.node._stop_child('drive');self.f.node._brake()
        self.f.node._finish_recording(boot.Status.SUCCESS,'aligned')
    def capture(self):
        self.aligned()
        for t in [.05,.10,.16,.22,.28]:self.tick(t)
        self.assertEqual(self.f.node.phase,'ROUTE_CAPTURE_SETTLE')
    def test_shadow_stops_at_aligned_corridor_check(self):
        self.f.node.execute=False;self.aligned();self.assertEqual(self.f.node.phase,'FINALIZING');self.f.assert_stopped()
    def test_distance_uses_encoder_not_plotter_drift(self):
        self.tick(0);self.f.d=10000;self.tick(.1);self.assertEqual(self.f.node.phase,'DRIVE')
    def test_capture_evidence_stops_before_next_output(self):
        self.capture();self.f.assert_stopped();self.assertTrue(self.f.node.capture_assumed);self.assertFalse(self.f.context.sumo.bottle_captured)
    def test_no_post_align_camera_read(self):
        self.aligned();self.f.video.get_sumo_alignment_observation=lambda _: (_ for _ in ()).throw(AssertionError('image gate'))
        self.tick(.05);self.assertGreater(boot.runtime.left_motor.power,0)
    def test_capture_max_distance_has_priority(self):
        self.aligned();self.f.node.calibration=replace(self.c,capture_max_mm=5)
        self.count=5;self.tick(.1);self.f.assert_stopped();self.assertEqual(self.f.node.phase,'FINALIZING')
    def test_invalid_sensor_does_not_report_capture(self):
        self.aligned();boot.runtime.sonar_sensor.get_distance=lambda:0
        for t in [.05,.1,.16,.22,.28]:self.tick(t)
        self.assertEqual(self.f.node.phase,'ROUTE_CAPTURE');self.assertFalse(self.f.node.capture_assumed)
    def test_control_gap_brakes(self):
        self.aligned();self.tick(1.);self.f.assert_stopped();self.assertIn('cycle',self.f.context.sumo.failure_reason)
    def test_recording_error_brakes(self):
        self.aligned();self.f.video.record_status['error']='disk failed';self.tick(.05);self.f.assert_stopped()
    def test_needs_fresh_post_stop_sensor_sequence(self):
        self.capture();self.tick(.33);self.assertEqual(self.f.node.phase,'ROUTE_CAPTURE_SETTLE')
        for t in [.39,.45,.51,.57]:self.tick(t)
        self.assertEqual(self.f.node.phase,'ROUTE_PUSH')
    def test_pwm_with_large_heading_error_stays_bounded(self):
        self.aligned();self.f.node.drive.target+=30;self.tick(.05)
        self.assertTrue(all(-100<=v<=100 for m in (boot.runtime.left_motor,boot.runtime.right_motor) for v in m.outputs))
    def test_cancellation_brakes_and_leaves_flags_unconfirmed(self):
        self.aligned();self.tick(.05);self.f.node.stop(boot.Status.INVALID);self.f.assert_stopped();self.assertFalse(self.f.context.sumo.bottle_pushed_out)
    def test_route_summary_has_calibration_and_unconfirmed_flags(self):
        self.capture();d=self.f.node._recording_details()['route'];self.assertTrue(d['capture_assumed']);self.assertFalse(d['physical_capture_confirmed']);self.assertEqual(d['calibration']['capture_raw_max'],70)
    def test_new_mission_camera_and_builder(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'cal.json';p.write_text(json.dumps(asdict(self.c)),encoding='utf-8')
            config=replace(config_for_mission('sumo-push-route-test'),sumo_push_calibration=str(p))
            self.assertTrue(mission_requires_camera(config));self.assertIsInstance(build_mission_children(self.f.context,config)[-1].engine,AlignAndPushRoute)
    def test_missing_calibration_selects_relative_mode(self):
        from robot_program.features.sumo_relative_push import RelativeAlignAndPush
        self.assertIsInstance(build_mission_children(self.f.context,config_for_mission('sumo-push-route-test'))[-1].engine,RelativeAlignAndPush)
    def pushing(self):
        self.capture()
        for t in [.33,.39,.45,.51,.57]:self.tick(t)
        self.assertEqual(self.f.node.phase,'ROUTE_PUSH')
    def test_end_to_end_direct_push_checks_distance_before_output(self):
        self.pushing();n=self.f.node
        distance=n.plan['distance_mm'];time=.57
        while self.count<distance:
            time+=.1;self.count=min(distance,self.count+10);self.tick(time)
            self.assertNotEqual(n.phase,'FINALIZING',self.f.context.sumo.failure_reason)
        self.assertEqual(n.phase,'ROUTE_HOLD');self.f.assert_stopped()
        for _ in range(11):
            time+=.1;self.tick(time)
            if n.phase=='FINALIZING':break
        self.assertEqual(n.phase,'FINALIZING');self.assertTrue(n.push_out_estimated)
        self.assertFalse(self.f.context.sumo.bottle_pushed_out)
    def test_sensor_loss_stops_push(self):
        self.pushing();boot.runtime.sonar_sensor.get_distance=lambda:-1
        self.tick(.63);self.f.assert_stopped();self.assertIn('evidence lost',self.f.context.sumo.failure_reason)
    def test_post_turn_replans_actual_pose(self):
        self.pushing();n=self.f.node;n.phase='ROUTE_TURN_SETTLE';n.phase_started_at=.57
        n.plan['bearing_deg']=30;self.f.gyro=0
        for t in [.63,.69,.75,.81,.87]:
            self.tick(t)
            if n.phase=='FINALIZING':break
        self.f.assert_stopped();self.assertIn('overshoot',self.f.context.sumo.failure_reason)
    def test_loaded_turn_stops_on_lost_sensor_evidence(self):
        self.pushing();n=self.f.node;n.phase='ROUTE_TURN';boot.runtime.sonar_sensor.get_distance=lambda:0
        self.tick(.63);self.f.assert_stopped();self.assertIn('loaded turn',self.f.context.sumo.failure_reason)
    def test_keepout_position_does_not_stop_capture_drive(self):
        self.aligned();n=self.f.node
        n.pose.y=1000-n.calibration.radius-n.calibration.stop_allowance
        self.tick(.05)
        self.assertEqual(n.phase,'ROUTE_CAPTURE');self.assertGreater(boot.runtime.left_motor.power,0)

    def test_initial_approach_is_not_rejected_by_collision_geometry(self):
        self.f.node.calibration=replace(self.c,initial_position_error_mm=2000)
        self.tick(0);self.assertEqual(self.f.node.phase,'DRIVE')
        self.assertGreater(boot.runtime.left_motor.power,0)

    def test_redirect_turn_and_settle_then_straight_push(self):
        self.c=replace(self.c,green_bearing_deg=0.,green_polygons_mm=(((-5000,350),(5000,350),(5000,5000),(-5000,5000)),))
        n=self.f.node;n.calibration=self.c
        self.capture()
        # Initial approach was along the white-side tangent; final alignment is up.
        n.angle_policy=alignment_push_policy(self.c,90.,0.)
        for t in [.33,.39,.45,.51,.57]:self.tick(t)
        self.assertEqual(n.phase,'ROUTE_TURN');goal=n.plan['bearing_deg']
        t=.57
        # First loaded turn output is opposite on the two wheels.
        self.tick(.63);t=.63
        self.assertEqual(boot.runtime.left_motor.power,-boot.runtime.right_motor.power)
        self.assertNotEqual(boot.runtime.left_motor.power,0)
        while abs((goal-self.f.gyro+180)%360-180)>0.01:
            gap=(goal-self.f.gyro+180)%360-180
            self.f.gyro+=max(-10,min(10,gap));t+=.06;self.tick(t)
        self.assertEqual(n.phase,'ROUTE_TURN_SETTLE');self.f.assert_stopped()
        for _ in range(6):
            t+=.06;self.tick(t)
            if n.phase=='ROUTE_PUSH':break
        self.assertEqual(n.phase,'ROUTE_PUSH')
        self.assertEqual(n.plan['kind'],'DIRECT')
    def alpha(self):
        script=(boot.OPTIONS.overlay or boot.REPO)/'alpha.py'
        if not script.is_file():script=boot.REPO/'alpha.py'
        spec=importlib.util.spec_from_file_location('alpha_green_check',script)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module
    def test_cli_keeps_alpha_reset_arm_and_touch(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'measured.json';p.write_text(json.dumps(asdict(self.c)),encoding='utf-8')
            with contextlib.redirect_stdout(io.StringIO()) as out:
                result=self.alpha().main(['left','--mission','sumo-push-route-test','--sumo-push-calibration',str(p),'--check-tree'])
            self.assertEqual(result,0)
            for text in ['arm up','arm down','device reset','touch start','green-avoiding push']:
                self.assertIn(text,out.getvalue())
    def test_cli_rejects_uncalibrated_course_mismatch_and_wrong_mission(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'measured.json'
            for edits,argv in [(dict(calibrated=False),['left','--mission','sumo-push-route-test']),
                               (dict(course='right'),['left','--mission','sumo-push-route-test']),
                               ({},['left','--mission','sumo-align'])]:
                p.write_text(json.dumps(dict(asdict(self.c),**edits)),encoding='utf-8')
                with contextlib.redirect_stdout(io.StringIO()),contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit) as e:
                    self.alpha().main(argv+['--sumo-push-calibration',str(p),'--check-tree'])
                self.assertEqual(e.exception.code,2)
    def test_route_fields_and_samples_saved_by_real_writer(self):
        import csv
        import numpy as np
        from robot_program.features.sumo_align_recording import SumoAlignRecording
        self.capture()
        with tempfile.TemporaryDirectory() as tmp:
            writer=SumoAlignRecording(output_dir=tmp)
            row=dict(self.f.node._record_snapshot(),frame_id=1,capture_monotonic_sec=.3)
            writer.submit(np.zeros((480,640,3),np.uint8),np.zeros((180,320,3),np.uint8),row)
            writer.request_stop('SUCCESS','unit synthetic',details=self.f.node._recording_details());writer.close()
            summary=json.loads(writer.paths['summary'].read_text(encoding='utf-8'))
            self.assertTrue(summary['details']['route']['samples'])
            with writer.paths['csv'].open(encoding='utf-8') as f:r=next(csv.DictReader(f))
            self.assertIn('route_x_mm',r);self.assertEqual(float(r['route_sonar_raw']),59)




class AnglePolicyTests(unittest.TestCase):
    def test_left_greenward_returns_measured_approach(self):
        c=calibration(green_bearing_deg=0);r=alignment_push_policy(c,270,300)
        self.assertEqual(r['actual_alignment_delta_deg'],30);self.assertEqual(r['push_bearing_deg'],270)
        self.assertTrue(r['toward_green'])
    def test_left_away_keeps_final_heading(self):
        r=alignment_push_policy(calibration(green_bearing_deg=0),270,240)
        self.assertFalse(r['toward_green']);self.assertEqual(r['push_bearing_deg'],240)
    def test_right_greenward_uses_opposite_delta_sign(self):
        r=alignment_push_policy(calibration(green_bearing_deg=0),90,60)
        self.assertTrue(r['toward_green']);self.assertEqual(r['actual_alignment_delta_deg'],-30)
        self.assertEqual(r['push_bearing_deg'],90)
    def test_right_away_keeps_final_heading(self):
        r=alignment_push_policy(calibration(green_bearing_deg=0),90,120)
        self.assertFalse(r['toward_green']);self.assertEqual(r['push_bearing_deg'],120)
    def test_positive_small_green_component_in_deadband(self):
        c=calibration(green_bearing_deg=0)
        for b in [270,271,272]:self.assertFalse(alignment_push_policy(c,270,b)['toward_green'])
        self.assertTrue(alignment_push_policy(c,270,272.01)['toward_green'])
    def test_wraparound_uses_signed_shortest_final_angle(self):
        r=alignment_push_policy(calibration(green_bearing_deg=90),350,10)
        self.assertEqual(r['actual_alignment_delta_deg'],20)
    def test_tilted_greenward_initial_pose_returns_nearest_tangent(self):
        r=alignment_push_policy(calibration(green_bearing_deg=0),280,300)
        self.assertEqual(r['push_bearing_deg'],270)
    def test_rotated_green_direction(self):
        r=alignment_push_policy(calibration(green_bearing_deg=180),90,120)
        self.assertTrue(r['toward_green']);self.assertEqual(r['push_bearing_deg'],90)
    def test_geometry_cannot_substitute_another_direction(self):
        c=calibration(green_polygons_mm=(((-5000,250),(5000,250),(5000,5000),(-5000,5000)),))
        p=pose(c);self.assertIsNotNone(plan_push(c,p))
        self.assertIsNone(plan_push(c,p,only_bearing=0))
    def test_geometry_honors_approved_target(self):
        c=calibration();p=pose(c);r=plan_push(c,p,only_bearing=90)
        self.assertEqual(r['bearing_deg'],90);self.assertEqual(r['turn_deg'],90)
    def test_invalid_deadband_rejected(self):
        for x in [0,45,True,float('nan')]:
            with self.assertRaises(ValueError):calibration(green_turn_deadband_deg=x)
    def test_backward_compatible_calibration_defaults(self):
        c=asdict(calibration());del c['green_bearing_deg'];del c['green_turn_deadband_deg']
        self.assertEqual(PushCalibration(**c).green_bearing_deg,0)
    def test_final_bearing_not_requested_first_turn_is_latched(self):
        f=boot.AlignTests('test_distance_stops_before_another_forward_output');f.setUp();self.addCleanup(f.doCleanups)
        boot.runtime.left_motor.get_count=lambda:0.;boot.runtime.right_motor.get_count=lambda:0.
        boot.runtime.sonar_sensor=Mock(get_distance=lambda:59)
        c=calibration();n=AlignAndPushRoute('angle latch',f.context,f.settings,f.config.sumo,c,False)
        n.tick_once();n._stop_child('drive');n._brake()
        n.requested_turn_step=20.;n.pose.bearing=-10.
        n._finish_recording(boot.Status.SUCCESS,'aligned')
        self.assertEqual(n.angle_policy['actual_alignment_delta_deg'],-10.)
        self.assertEqual(n.angle_policy['push_bearing_deg'],350.)
        self.assertEqual(n._recording_details()['route']['angle_policy'],n.angle_policy)
    def test_green_return_is_required_even_if_aligned_direct_would_be_safe(self):
        c=calibration(green_bearing_deg=0);p=pose(c);p.bearing=300
        self.assertEqual(plan_push(c,p)['kind'],'DIRECT')
        policy=alignment_push_policy(c,270,300)
        result=plan_push(c,p,only_bearing=policy['push_bearing_deg'])
        self.assertEqual(result['kind'],'REDIRECT');self.assertEqual(result['bearing_deg'],270)
    def test_collision_geometry_does_not_override_angle_policy(self):
        fixture=Mission('test_capture_evidence_stops_before_next_output');fixture.setUp();self.addCleanup(fixture.doCleanups)
        fixture.c=replace(fixture.c,green_polygons_mm=(((-5000,350),(5000,350),(5000,5000),(-5000,5000)),))
        n=fixture.f.node;n.calibration=fixture.c
        fixture.capture()
        for t in [.33,.39,.45,.51,.57]:fixture.tick(t)
        self.assertEqual(n.phase,'ROUTE_PUSH');self.assertEqual(n.plan['bearing_deg'],0)
        self.assertFalse(n._recording_details()['route']['geometry_stop_enabled'])

    def test_new_policy_fields_are_in_frame_csv_schema(self):
        from robot_program.features.sumo_align_recording import FIELDS
        for key in ['route_alignment_delta_deg','route_angle_decision','route_policy_bearing_deg']:
            self.assertIn(key,FIELDS)



    def test_unchecked_distance_plan_ignores_keepout(self):
        c=calibration(green_polygons_mm=(((-5000,-5000),(5000,-5000),(5000,5000),(-5000,5000)),))
        self.assertIsNone(plan_push(c,pose(c),only_bearing=0))
        self.assertIsNotNone(plan_push(c,pose(c),only_bearing=0,check_clearance=False))
    def test_unchecked_plan_retains_distance_limit(self):
        c=calibration(push_max_mm=10)
        self.assertIsNone(plan_push(c,pose(c),only_bearing=0,check_clearance=False))
    def test_unchecked_plan_requires_explicit_angle_target(self):
        with self.assertRaises(ValueError):plan_push(calibration(),pose(calibration()),check_clearance=False)
    def test_mission_never_calls_collision_checker(self):
        from unittest.mock import patch
        fixture=Mission('test_capture_evidence_stops_before_next_output');fixture.setUp();self.addCleanup(fixture.doCleanups)
        with patch.object(PushCalibration,'safe_segment',side_effect=AssertionError('geometry veto called')):
            fixture.pushing()
            fixture.tick(.63)
            self.assertEqual(fixture.f.node.phase,'ROUTE_PUSH')




class RelativePushTests(unittest.TestCase):
    def setUp(self):
        from robot_program.features.sumo_relative_push_settings import RelativePushSettings
        self.f=boot.AlignTests('test_distance_stops_before_another_forward_output');self.f.setUp();self.addCleanup(self.f.doCleanups)
        self.f.context.sumo.bearing_reference.register(270,0)
        boot.runtime.sonar_sensor=Mock(get_distance=lambda:59)
        self.p=RelativePushSettings(push_distance_mm=100,capture_max_mm=50,power=40)
        config=replace(self.f.config,mission_mode='sumo-push-route-test',sumo_push_calibration=None,
                       sumo_push_execute=True,sumo_relative_push=self.p,sumo_align=self.f.settings)
        self.f.node=build_mission_children(self.f.context,config)[-1].engine
    def tick(self,t):self.f.now.return_value=t;self.f.node.tick_once()
    def ready(self,gyro=0):
        self.f.start_observing();self.f.gyro=gyro
        for t,i in [(.6,10),(.64,11)]:
            self.f.now.return_value=t;self.f.observation(i);self.f.node.tick_once()
        self.assertEqual(self.f.node.phase,'NC_CAPTURE')
    def capture(self,gyro=0):
        self.ready(gyro)
        for t in [.7,.76,.82,.88,.94]:
            self.tick(t)
            if self.f.node.phase=='NC_CAPTURE_SETTLE':break
        self.assertEqual(self.f.node.phase,'NC_CAPTURE_SETTLE');self.f.assert_stopped()
    def pushing(self):
        self.capture()
        for t in [1.,1.06,1.12,1.18,1.24]:self.tick(t)
        self.assertEqual(self.f.node.phase,'NC_PUSH')
    def test_no_calibration_full_distance_sequence(self):
        self.pushing();self.f.d+=100;self.tick(1.3)
        self.assertEqual(self.f.node.phase,'NC_DONE');self.f.assert_stopped()
        self.tick(2.31);self.assertEqual(self.f.node.phase,'FINALIZING')
        self.assertFalse(self.f.node._recording_details()['route']['calibration_required'])
        self.assertFalse(self.f.context.sumo.bottle_pushed_out)
    def test_no_post_align_image_gate(self):
        self.ready();self.f.video.get_sumo_alignment_observation=lambda _: (_ for _ in ()).throw(AssertionError('camera gate'))
        self.tick(.7);self.assertEqual(self.f.node.phase,'NC_CAPTURE_SETTLE');self.f.assert_stopped()
    def test_capture_ceiling_before_next_forward_output(self):
        self.ready();self.f.d+=50;self.tick(.7);self.f.assert_stopped();self.assertEqual(self.f.node.phase,'FINALIZING')
    def test_invalid_sensor_is_not_capture(self):
        self.ready();boot.runtime.sonar_sensor.get_distance=lambda:0
        for t in [.7,.76,.82,.88,.94]:self.tick(t)
        self.assertEqual(self.f.node.phase,'NC_CAPTURE')
    def test_push_sensor_loss_brakes(self):
        self.pushing();boot.runtime.sonar_sensor.get_distance=lambda:-1;self.tick(1.3);self.f.assert_stopped()
    def test_greenward_alignment_returns_to_measured_approach(self):
        self.capture(30)
        for t in [1.,1.06,1.12,1.18,1.24]:self.tick(t)
        self.assertEqual(self.f.node.phase,'NC_TURN')
        self.assertEqual(self.f.node.angle_policy['push_bearing_deg'],270)
        self.tick(1.3);self.assertEqual(boot.runtime.left_motor.power,-boot.runtime.right_motor.power)
        self.f.gyro=0;self.tick(1.36);self.assertEqual(self.f.node.phase,'NC_TURN_SETTLE');self.f.assert_stopped()
        for t in [1.42,1.48,1.54,1.6,1.66]:self.tick(t)
        self.assertEqual(self.f.node.phase,'NC_PUSH')
    def test_recording_fault_brakes(self):
        self.ready();self.f.video.record_status['error']='disk';self.tick(.7);self.f.assert_stopped()
    def test_cancellation_brakes(self):
        self.ready();self.tick(.7);self.f.node.stop(boot.Status.INVALID);self.f.assert_stopped()
    def test_explicit_distance_and_settings_validation(self):
        from robot_program.features.sumo_relative_push_settings import RelativePushSettings
        for x in [0,-1,float('nan'),True]:
            with self.assertRaises(ValueError):RelativePushSettings(push_distance_mm=x)
        config=replace(self.f.config,mission_mode='sumo-push-route-test',sumo_push_execute=True)
        with self.assertRaisesRegex(ValueError,'push-distance'):build_mission_children(self.f.context,config)
    def alpha(self):
        script=(boot.OPTIONS.overlay or boot.REPO)/'alpha.py'
        if not script.exists():script=boot.REPO/'alpha.py'
        spec=importlib.util.spec_from_file_location('alpha_relative_check',script)
        m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
    def test_cli_without_json_preserves_startup(self):
        for course in ['left','right']:
            with contextlib.redirect_stdout(io.StringIO()) as out:
                result=self.alpha().main([course,'--mission','sumo-push-route-test','--sumo-push-distance-mm','100','--sumo-push-execute','--check-tree'])
            self.assertEqual(result,0)
            for text in ['arm up','arm down','device reset','touch start','relative return']:self.assertIn(text,out.getvalue())
    def test_cli_requires_distance_only_when_executing(self):
        with contextlib.redirect_stdout(io.StringIO()),contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit) as e:
            self.alpha().main(['left','--mission','sumo-push-route-test','--sumo-push-execute','--check-tree'])
        self.assertEqual(e.exception.code,2)
        with contextlib.redirect_stdout(io.StringIO()):self.assertEqual(self.alpha().main(['left','--mission','sumo-push-route-test','--check-tree']),0)
    def test_relative_options_cannot_change_other_missions(self):
        with contextlib.redirect_stdout(io.StringIO()),contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
            self.alpha().main(['left','--mission','sumo-align','--sumo-push-distance-mm','100','--check-tree'])
    def test_sensor_read_crossing_distance_limit_stops_before_output(self):
        self.ready()
        def read():self.f.d+=50;return 59
        boot.runtime.sonar_sensor.get_distance=read
        before=len(boot.runtime.left_motor.outputs);self.tick(.7)
        self.f.assert_stopped();self.assertTrue(all(v==0 for v in boot.runtime.left_motor.outputs[before:]))
    def test_sensor_exception_brakes(self):
        self.ready();boot.runtime.sonar_sensor.get_distance=Mock(side_effect=OSError('sensor error'))
        self.tick(.7);self.f.assert_stopped();self.assertIn('IO failure',self.f.context.sumo.failure_reason)
    def test_stall_remains_a_stop(self):
        self.ready();boot.runtime.sonar_sensor.get_distance=lambda:0
        self.tick(1.7);self.f.assert_stopped();self.assertIn('no forward progress',self.f.context.sumo.failure_reason)
    def test_phase_timeout_remains_a_stop(self):
        self.ready();self.f.node.push_settings=replace(self.p,phase_timeout_sec=.1)
        self.tick(.8);self.f.assert_stopped();self.assertIn('phase timeout',self.f.context.sumo.failure_reason)
    def test_arbitrary_explicit_push_distance_is_used(self):
        self.pushing();self.f.node.push_settings=replace(self.p,push_distance_mm=2.)
        self.f.d+=2;self.tick(1.3);self.assertEqual(self.f.node.phase,'NC_DONE');self.f.assert_stopped()




class ActualReturnHeadingTests(unittest.TestCase):
    def setUp(self):
        self.fixture=RelativePushTests('test_no_calibration_full_distance_sequence');self.fixture.setUp();self.addCleanup(self.fixture.doCleanups)
    def settled(self,error):
        f=self.fixture;f.pushing();n=f.f.node;n._hold('NC_TURN_SETTLE',1.24);f.f.gyro=error
        for t in [1.30,1.36,1.42,1.48,1.54]:
            f.tick(t)
            if n.phase=='NC_PUSH':break
        return f,n
    def test_positive_recorded_error_pushes_in_actual_heading_without_correction(self):
        f,n=self.settled(4.0254);self.assertEqual(n.phase,'NC_PUSH')
        self.assertAlmostEqual(n.nc_heading,274.0254)
        f.f.d+=3;before=len(boot.runtime.left_motor.outputs);f.tick(1.6)
        self.assertEqual(n.phase,'NC_PUSH');self.assertAlmostEqual(n.drive.target,-4.0254)
        self.assertEqual(boot.runtime.left_motor.power,40);self.assertEqual(boot.runtime.right_motor.power,40)
        self.assertTrue(all(x>=0 for x in boot.runtime.left_motor.outputs[before:]))
    def test_negative_recorded_error_uses_actual_heading(self):
        f,n=self.settled(-4.0254);self.assertAlmostEqual(n.nc_heading,265.9746)
        f.f.d+=3;f.tick(1.6);self.assertEqual(n.phase,'NC_PUSH')
    def test_overshoot_does_not_enter_old_correction_or_failure(self):
        f,n=self.settled(6.0381);self.assertEqual(n.phase,'NC_PUSH');self.assertAlmostEqual(n.nc_heading,276.0381)
    def test_four_degree_guard_uses_new_origin(self):
        f,n=self.settled(4.0254);f.f.gyro=9.05715;f.tick(1.6)
        f.f.assert_stopped();self.assertIn('heading deviation',f.f.context.sumo.failure_reason)
    def test_sensor_reconfirmation_is_still_required(self):
        f=self.fixture;f.pushing();n=f.f.node;n._hold('NC_TURN_SETTLE',1.24)
        boot.runtime.sonar_sensor.get_distance=lambda:0
        for t in [1.30,1.36,1.42,1.48,1.54]:f.tick(t)
        self.assertEqual(n.phase,'NC_TURN_SETTLE');f.f.assert_stopped()
    def test_summary_and_csv_field_distinguish_original_and_actual_target(self):
        f,n=self.settled(4.0254);r=n._recording_details()['route']
        self.assertFalse(r['post_return_correction']);self.assertFalse(r['return_angle_gate'])
        self.assertEqual(r['angle_policy']['push_bearing_deg'],270.)
        self.assertAlmostEqual(r['push_start_bearing_deg'],274.0254)
        self.assertAlmostEqual(n._record_snapshot()['route_push_start_bearing_deg'],274.0254)
        from robot_program.features.sumo_align_recording import FIELDS
        self.assertIn('route_push_start_bearing_deg',FIELDS)
    def test_distance_stop_remains(self):
        f,n=self.settled(4.0254);f.f.d+=100;f.tick(1.6)
        self.assertEqual(n.phase,'NC_DONE');f.f.assert_stopped()




class StopFirstCaptureTests(unittest.TestCase):
    def setUp(self):
        self.f=RelativePushTests('test_no_calibration_full_distance_sequence');self.f.setUp();self.addCleanup(self.f.doCleanups)
    def test_late_near_sample_brakes_before_distance_budget_is_consumed(self):
        self.f.ready();n=self.f.f.node;self.f.f.d+=49
        before=len(boot.runtime.left_motor.outputs);self.f.tick(.7)
        self.assertEqual(n.phase,'NC_CAPTURE_SETTLE');self.f.f.assert_stopped()
        self.assertFalse(n.capture_assumed);self.assertEqual(n.capture_candidate_progress,49)
        self.assertTrue(all(x==0 for x in boot.runtime.left_motor.outputs[before:]))
        # Coast to the cap while stopped must not request any further forward motion.
        self.f.f.d+=2
        for t in [.76,.82,.88,.94]:
            self.f.tick(t);self.f.f.assert_stopped();self.assertFalse(n.capture_assumed)
        self.f.tick(1.)
        self.assertTrue(n.capture_assumed);self.assertEqual(n.phase,'NC_PUSH')
    def test_one_transient_candidate_is_not_capture_confirmation(self):
        self.f.ready();n=self.f.f.node;self.f.tick(.7)
        boot.runtime.sonar_sensor.get_distance=lambda:0
        for t in [.76,.82,.88,.94,1.]:self.f.tick(t)
        self.assertEqual(n.phase,'NC_CAPTURE_SETTLE');self.assertFalse(n.capture_assumed);self.f.f.assert_stopped()
    def test_zero_and_negative_do_not_trigger_candidate_brake(self):
        self.f.ready();n=self.f.f.node
        for t,raw in [(.7,0),(.76,-1),(.82,82)]:
            boot.runtime.sonar_sensor.get_distance=lambda v=raw:v
            self.f.tick(t);self.assertEqual(n.phase,'NC_CAPTURE')
    def test_distance_cap_still_precedes_near_sample(self):
        self.f.ready();self.f.f.d+=50;self.f.tick(.7)
        self.assertEqual(self.f.f.node.phase,'FINALIZING');self.f.f.assert_stopped()
    def test_unconfirmed_candidate_times_out_without_resuming_forward(self):
        self.f.ready();n=self.f.f.node;self.f.tick(.7)
        boot.runtime.sonar_sensor.get_distance=lambda:0
        self.f.tick(16.);self.f.f.assert_stopped();self.assertFalse(n.capture_assumed)
        self.assertEqual(n.phase,'FINALIZING')




class SumoPushTreeTests(unittest.TestCase):
    def setUp(self):
        self.fixture=RelativePushTests('test_no_calibration_full_distance_sequence');self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        from robot_program.features.sumo_push_tree import SumoPushSequence
        self.tree=SumoPushSequence(self.fixture.f.node)
        # Existing fixture actions tick the actual Sequence; the shared engine
        # remains available for observing original state/telemetry.
        self.fixture.f.node.tick_once=lambda:self.tree.tick_once()
    def test_real_sequence_contains_ordered_executable_stages(self):
        from py_trees.composites import Sequence
        self.assertIsInstance(self.tree,Sequence)
        self.assertEqual([x.name for x in self.tree.children],['規定距離を直進してボトルへ正対','捕捉接近と判定','必要な場合だけ戻り旋回','押し出し・正対位置へ戻る・録画終了'])
        self.fixture.ready();self.assertEqual(self.fixture.f.node.phase,'NC_CAPTURE')
        self.assertEqual(self.tree.children[0].status,boot.Status.SUCCESS)
        self.assertEqual(self.tree.children[1].status,boot.Status.RUNNING)
    def test_direct_push_skips_return_stage_and_completes(self):
        f=self.fixture;f.pushing();self.assertEqual(self.tree.children[2].status,boot.Status.SUCCESS)
        f.f.d+=100;f.tick(1.3);self.assertEqual(f.f.node.phase,'NC_DONE');f.f.assert_stopped()
        f.tick(2.31);self.assertEqual(self.tree.status,boot.Status.SUCCESS)
    def test_needed_return_turn_is_an_active_tree_node(self):
        f=self.fixture;f.capture(30)
        for t in [1.,1.06,1.12,1.18,1.24]:f.tick(t)
        self.assertEqual(f.f.node.phase,'NC_TURN');self.assertEqual(self.tree.children[2].status,boot.Status.RUNNING)
    def test_cancel_tree_brakes_shared_controller(self):
        f=self.fixture;f.ready();f.tick(.7);self.tree.stop(boot.Status.INVALID);f.f.assert_stopped()
        self.assertEqual(f.f.node.status,boot.Status.INVALID)
    def test_same_cycle_stage_transition_does_not_tick_controller_twice(self):
        f=self.fixture;f.f.start_observing()
        for t,i in [(.6,10),(.64,11)]:
            f.f.now.return_value=t;f.f.observation(i);f.f.node.tick_once()
        self.assertEqual(f.f.node.phase,'NC_CAPTURE')
        self.assertEqual(len(f.f.node.nc_samples),0)
        self.assertEqual(self.tree.children[1].status,boot.Status.RUNNING)
    def test_failure_remains_a_failure_and_stops(self):
        f=self.fixture;f.ready();f.f.d+=50;f.tick(.7)
        self.assertEqual(self.tree.status,boot.Status.FAILURE);f.f.assert_stopped()
    def test_completed_tree_reinitializes_engine_on_next_run(self):
        f=self.fixture;f.pushing();f.f.d+=100;f.tick(1.3);f.tick(2.31)
        self.assertEqual(self.tree.status,boot.Status.SUCCESS)
        f.tick(3.);self.assertEqual(self.tree.status,boot.Status.RUNNING)
        self.assertEqual(f.f.node.phase,'DRIVE');self.assertGreater(boot.runtime.left_motor.power,0)
    def test_check_tree_displays_all_four_stages(self):
        with contextlib.redirect_stdout(io.StringIO()) as out:
            result=self.fixture.alpha().main(['right','--mission','sumo-push-route-test','--sumo-push-distance-mm','100','--sumo-push-execute','--check-tree'])
        self.assertEqual(result,0)
        for name in ['規定距離を直進してボトルへ正対','捕捉接近と判定','必要な場合だけ戻り旋回','押し出し・正対位置へ戻る・録画終了']:
            self.assertIn(name,out.getvalue())


if __name__=='__main__':unittest.main(argv=[sys.argv[0]]+boot.TEST_ARGS)
