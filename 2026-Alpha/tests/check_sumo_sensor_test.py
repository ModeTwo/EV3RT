"""Stationary probe tests with real recorder/CLI and fake robot IO."""
import argparse,contextlib,importlib.util,io,json,sys,tempfile,unittest,csv
from dataclasses import replace
from pathlib import Path
from unittest.mock import Mock,patch
import numpy as np
p=argparse.ArgumentParser(add_help=False);p.add_argument('--repository',type=Path,default=Path(__file__).resolve().parents[1])
options,_=p.parse_known_args();sys.path.insert(0,str(options.repository/'tests'))
import check_sumo_align as boot
from robot_program.features import sumo_sensor_test as target
from robot_program.features.sumo_sensor_settings import SumoSensorSettings
from robot_program.features.sumo_sensor_data import decode_distance
from robot_program.features.sumo_align_recording import SumoAlignRecording
from robot_program.config import config_for_mission,mission_requires_camera,mission_requires_qr
from robot_program.tree_builder import build_mission_children
from robot_program.features.sumo_bearing import initial_sumo_bearing


class Motor(boot.IO):
    def get_count(self):return 0
    def set_power(self,power):
        if power!=0:raise AssertionError('Stationary test commanded wheel motion')
        super().set_power(power)


class Video(boot.FakeVideo):
    def __init__(self):super().__init__();self.details=None;self.folder=None
    def begin_sumo_alignment_recording(self,reader,**kwargs):
        super().begin_sumo_alignment_recording(reader);self.folder=kwargs['output_dir']
    def end_sumo_alignment_recording(self,result,reason,**kwargs):
        super().end_sumo_alignment_recording(result,reason,**kwargs)
        if self.details is None:self.details=kwargs.get('details')


class SensorTests(unittest.TestCase):
    def setUp(self):
        self.clock=patch.object(target.time,'monotonic',return_value=0.)
        self.now=self.clock.start();self.addCleanup(self.clock.stop)
        self.context=boot.RaceContext();self.video=Video()
        boot.runtime.video=self.video
        boot.runtime.left_motor,boot.runtime.right_motor,boot.runtime.arm_motor=Motor(),Motor(),Motor()
        boot.runtime.sonar_sensor=Mock(get_distance=Mock(return_value=35))
        boot.runtime.gyro_sensor=Mock(get_angle=Mock(return_value=0))
        boot.runtime.plotter=Mock(get_distance=Mock(return_value=0))
        self.settings=SumoSensorSettings(label='inside',duration_sec=1,forward_mm=-30,lateral_mm=0)
        self.config=replace(config_for_mission('sumo-sensor-test'),sumo_sensor=self.settings)
        self.node=target.SumoSensorTest('probe',self.context,self.settings,self.config.sumo)

    def test_never_commands_nonzero_wheel_output(self):
        self.node.tick_once()
        self.now.return_value=.11;self.node.tick_once()
        self.node.stop(boot.Status.INVALID)
        for motor in (boot.runtime.left_motor,boot.runtime.right_motor):
            self.assertTrue(motor.brake);self.assertTrue(all(v==0 for v in motor.outputs))

    def test_preserves_short_distance_below_legacy_minimum(self):
        self.node.tick_once();sample=self.node.samples[0]
        self.assertEqual(sample['raw_value'],35)
        self.assertEqual(sample['numeric_mm'],35)
        self.assertEqual(sample['raw_status'],'positive_numeric')
        self.assertFalse(sample['legacy_window_match'])

    def test_label_and_signed_positions_survive(self):
        self.node.tick_once();sample=self.node.samples[0]
        self.assertEqual((sample['label'],sample['forward_mm'],sample['lateral_mm']),('inside',-30,0))
        self.assertEqual(self.video.folder.name,'sumo_sensor_test_runs')

    def test_optional_unmeasured_positions_stay_none(self):
        self.node.settings=replace(self.settings,forward_mm=None,lateral_mm=None)
        self.node.tick_once();self.assertIsNone(self.node.samples[0]['forward_mm'])

    def test_sampling_is_rate_limited_not_each_control_tick(self):
        self.node.tick_once()
        for t in (.01,.02,.05,.09):self.now.return_value=t;self.node.tick_once()
        self.assertEqual(boot.runtime.sonar_sensor.get_distance.call_count,1)
        self.now.return_value=.11;self.node.tick_once()
        self.assertEqual(boot.runtime.sonar_sensor.get_distance.call_count,2)

    def test_repeated_api_values_marked_not_called_independent(self):
        self.node.tick_once();self.now.return_value=.11;self.node.tick_once()
        self.assertTrue(self.node.samples[1]['same_as_previous_read'])
        self.assertEqual(self.node.samples[1]['same_value_run_reads'],2)
        self.assertIn('freshness',self.node._details()['sample_timing'])

    def test_sensor_exception_is_recorded_and_measurement_continues(self):
        boot.runtime.sonar_sensor.get_distance.side_effect=RuntimeError('read failed')
        self.node.tick_once()
        self.assertEqual(self.node.status,boot.Status.RUNNING)
        self.assertEqual(self.node.samples[0]['raw_status'],'read_error')
        self.assertEqual(self.node.samples[0]['raw_repr'],'read failed')

    def test_invalid_raw_values_are_json_safe_and_retained(self):
        for raw,status in ((None,'none'),(0,'non_positive'),(-1,'non_positive'),
                           (float('nan'),'non_finite'),(float('inf'),'non_finite'),('bad','non_numeric'),(True,'non_numeric')):
            with self.subTest(raw=raw):
                decoded=decode_distance(raw,1,50,800)
                self.assertEqual(decoded['raw_status'],status)
                json.dumps(decoded,allow_nan=False)

    def test_scale_and_legacy_match_are_metadata_not_filter(self):
        result=decode_distance(20,10,50,800)
        self.assertEqual(result['numeric_mm'],200);self.assertTrue(result['legacy_window_match'])
        result=decode_distance(65535,1,50,800)
        self.assertEqual(result['raw_value'],65535);self.assertFalse(result['legacy_window_match'])

    def test_measurement_complete_is_not_capture_success(self):
        self.node.tick_once();self.now.return_value=1.01;self.node.tick_once()
        self.assertEqual(self.node.phase,'FINALIZING')
        self.now.return_value=1.03;self.node.tick_once()
        self.assertEqual(self.node.status,boot.Status.SUCCESS)
        self.assertFalse(self.video.details['physical_capture_confirmed'])
        self.assertTrue(self.video.details['no_drive'])
        self.assertEqual(self.video.details['kind'],'sumo-sensor-test-v1')

    def test_video_telemetry_never_reads_sonar(self):
        self.node.tick_once();calls=boot.runtime.sonar_sensor.get_distance.call_count
        self.now.return_value=.05
        row=self.node._record_snapshot()
        self.assertEqual(row['probe_sample_id'],1)
        self.assertAlmostEqual(row['probe_sample_age_at_telemetry_sec'],.05)
        self.assertEqual(boot.runtime.sonar_sensor.get_distance.call_count,calls)

    def test_camera_start_timeout_brakes_without_sampling(self):
        self.video.record_status['ready']=False
        self.node.tick_once();self.now.return_value=5.1;self.node.tick_once()
        self.assertEqual(self.node.final_status,boot.Status.FAILURE)
        self.assertFalse(self.node.samples)
        self.assertEqual(boot.runtime.left_motor.power,0)

    def test_recording_error_stops(self):
        self.node.tick_once();self.video.record_status['error']='disk'
        self.node.tick_once();self.assertEqual(self.node.final_status,boot.Status.FAILURE)

    def test_finalize_timeout_reports_failure(self):
        self.node.tick_once();self.video.record_status['finished']=False
        self.now.return_value=1.01;self.node.tick_once()
        self.now.return_value=6.02;self.node.tick_once()
        self.assertEqual(self.node.status,boot.Status.FAILURE)

    def test_interruption_keeps_partial_samples(self):
        self.node.tick_once();self.node.stop(boot.Status.INVALID)
        self.assertEqual(len(self.video.details['sensor_samples']),1)
        self.assertEqual(self.video.ends,[1])

    def test_settings_validate_missing_labels_and_nonfinite_positions(self):
        with self.assertRaises(ValueError):SumoSensorSettings().require_label()
        for kwargs in (dict(label=''),dict(label='bad\nlabel'),dict(forward_mm=float('nan')),
                       dict(interval_sec=0),dict(duration_sec=61),dict(mm_per_unit=0)):
            with self.assertRaises(ValueError):SumoSensorSettings(**kwargs)

    def alpha(self):
        script=(boot.OPTIONS.overlay or boot.REPO)/'alpha.py'
        if not script.is_file():script=boot.REPO/'alpha.py'
        spec=importlib.util.spec_from_file_location('alpha_probe_check',script)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module

    def test_cli_keeps_initialization_and_no_drive_feature(self):
        with contextlib.redirect_stdout(io.StringIO()) as output:
            result=self.alpha().main(['left','--mission','sumo-sensor-test','--sumo-sensor-label','inside',
                '--sumo-sensor-forward-mm','-30','--sumo-sensor-lateral-mm','0','--check-tree'])
        self.assertEqual(result,0)
        for word in ('arm up','arm down','device reset','touch start','stationary bottle sensor measurement only'):
            self.assertIn(word,output.getvalue())
        self.assertNotIn('approach and face',output.getvalue())
        self.assertTrue(mission_requires_camera(self.config));self.assertFalse(mission_requires_qr(self.config))
        self.assertIsInstance(build_mission_children(self.context,self.config)[-1],target.SumoSensorTest)
        self.assertEqual(initial_sumo_bearing('sumo-sensor-test'),0)

    def test_cli_rejects_missing_label_wrong_mission_and_bad_values(self):
        for args in (['--mission','sumo-sensor-test'],['--mission','sumo-align','--sumo-sensor-label','inside'],
                     ['--mission','sumo-sensor-test','--sumo-sensor-label','inside','--sumo-sensor-sec','0']):
            with contextlib.redirect_stdout(io.StringIO()),contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit) as error:
                self.alpha().main(['left','--check-tree']+args)
            self.assertEqual(error.exception.code,2)

    def test_actual_writer_exports_one_row_per_read_and_frame_link(self):
        self.node.tick_once();details=self.node._details()
        with tempfile.TemporaryDirectory(prefix='sumo-sensor-check-') as tmp:
            recorder=SumoAlignRecording(output_dir=tmp)
            row=dict(self.node._record_snapshot(),frame_id=1,capture_monotonic_sec=.01)
            recorder.submit(np.zeros((480,640,3),np.uint8),np.zeros((180,320,3),np.uint8),row)
            recorder.request_stop('SUCCESS','measured',details=details)
            recorder.request_stop('INTERRUPTED','later')
            recorder.close()
            summary=json.loads(recorder.paths['summary'].read_text(encoding='utf-8'))
            self.assertIsNone(summary['writer_error'])
            with Path(summary['paths']['sensor_samples']).open(encoding='utf-8',newline='') as source:
                rows=list(csv.DictReader(source))
            self.assertEqual(len(rows),1);self.assertEqual(rows[0]['raw_value'],'35.0')
            self.assertEqual(rows[0]['label'],'inside')
            with recorder.paths['csv'].open(encoding='utf-8',newline='') as source:
                self.assertEqual(next(csv.DictReader(source))['probe_sample_id'],'1')


if __name__=='__main__':unittest.main(argv=[__file__]+boot.TEST_ARGS)
