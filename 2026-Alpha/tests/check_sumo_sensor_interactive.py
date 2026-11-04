"""Keyboard segments and terminal restoration without robot motion."""
import argparse,contextlib,io,sys,unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch
p=argparse.ArgumentParser(add_help=False);p.add_argument('--repository',type=Path,default=Path(__file__).resolve().parents[1])
options,_=p.parse_known_args();sys.path.insert(0,str(options.repository/'tests'))
import check_sumo_sensor_test as base
from robot_program.diagnostics import sumo_sensor_interactive as target
from robot_program.diagnostics import sumo_sensor_keys as keys_module
from robot_program.diagnostics.sumo_sensor_test import build_sumo_sensor_test


class FakeKeys:
    def __init__(self):self.queue=[];self.opened=False;self.closed=False
    def open(self):self.opened=True
    def close(self):self.closed=True
    def poll(self):result=self.queue[:];self.queue=[];return result


class Video(base.Video):
    def __init__(self):super().__init__();self.segments=[];self.starts=0
    def begin_sumo_alignment_recording(self,reader,**kwargs):
        self.starts+=1;self.details=None;super().begin_sumo_alignment_recording(reader,**kwargs)
    def end_sumo_alignment_recording(self,result,reason,**kwargs):
        if self.details is None and kwargs.get('details') is not None:self.segments.append(kwargs['details'])
        super().end_sumo_alignment_recording(result,reason,**kwargs)


class InteractiveTests(unittest.TestCase):
    def setUp(self):
        self.f=base.SensorTests('test_never_commands_nonzero_wheel_output')
        self.f.setUp();self.addCleanup(self.f.doCleanups)
        self.video=Video();base.boot.runtime.video=self.video
        self.keys=FakeKeys();self.patch=patch.object(target,'TerminalKeys',return_value=self.keys)
        self.patch.start();self.addCleanup(self.patch.stop)
        self.settings=replace(self.f.settings,label=None,interactive=True)
        self.node=target.InteractiveSumoSensorTest('interactive',self.f.context,self.settings,self.f.config.sumo)
        self.tick(0)

    def tick(self,t,keys=()):
        self.f.now.return_value=t;self.keys.queue.extend(keys);self.node.tick_once()

    def start(self):self.tick(.1,['5',' '])
    def complete(self):self.tick(1.11);self.tick(1.12)

    def test_idle_opens_keys_without_measurement_or_files(self):
        self.assertTrue(self.keys.opened);self.assertEqual(self.node.label,'empty')
        self.assertEqual(self.video.starts,0)
        self.assertEqual(base.boot.runtime.sonar_sensor.get_distance.call_count,0)

    def test_digit_only_selects_does_not_measure(self):
        self.tick(.1,['6'])
        self.assertEqual(self.node.label,'left-out');self.assertEqual(self.video.starts,0)

    def test_start_and_completion_return_to_idle(self):
        self.start();self.assertEqual(self.node.child.settings.label,'inside')
        self.complete()
        self.assertIsNone(self.node.child);self.assertEqual(self.node.status,base.boot.Status.RUNNING)
        self.assertEqual(self.node.saved_segments,1)
        self.assertTrue(self.video.segments[0]['interactive']['segment_complete'])

    def test_repeat_without_restart_has_distinct_repetition(self):
        self.start();self.complete();self.tick(1.13,[' ']);self.tick(2.14);self.tick(2.15)
        self.assertEqual(self.video.starts,2);self.assertEqual(self.node.saved_segments,2)
        self.assertEqual([x['interactive']['repeat'] for x in self.video.segments],[1,2])
        self.assertEqual(len({x['interactive']['session_id'] for x in self.video.segments}),1)

    def test_mid_measurement_phase_switch_is_ignored(self):
        self.start();self.tick(.2,['6',' '])
        self.assertEqual(self.node.label,'inside');self.assertEqual(self.video.starts,1)
        self.assertTrue(all(x['label']=='inside' for x in self.node.child.samples))

    def test_select_after_save_clears_prior_coordinates(self):
        self.start();self.complete();self.tick(1.13,['7',' '])
        child=self.node.child
        self.assertEqual(child.settings.label,'right-out')
        self.assertIsNone(child.settings.forward_mm);self.assertIsNone(child.settings.lateral_mm)

    def test_partial_stop_saves_and_allows_next_measurement(self):
        self.start();self.tick(.3,['p'])
        self.assertIsNone(self.node.child)
        self.assertFalse(self.video.segments[0]['interactive']['segment_complete'])
        self.assertIn('partial',self.video.segments[0]['interactive']['segment_end_reason'])
        self.tick(.4,[' ']);self.assertEqual(self.video.starts,2)

    def test_quit_during_measurement_saves_partial_and_restores_keys(self):
        self.start();self.tick(.3,['q'])
        self.assertEqual(self.node.status,base.boot.Status.SUCCESS);self.assertTrue(self.keys.closed)
        self.assertEqual(self.node.saved_segments,1)
        self.assertFalse(self.video.segments[0]['interactive']['segment_complete'])

    def test_quit_idle_does_not_create_empty_recording(self):
        self.tick(.1,['q']);self.assertEqual(self.node.status,base.boot.Status.SUCCESS)
        self.assertEqual(self.video.starts,0);self.assertTrue(self.keys.closed)

    def test_quit_waits_for_writer_completion(self):
        self.start();self.video.record_status['finished']=False
        self.tick(.3,['q']);self.assertEqual(self.node.status,base.boot.Status.RUNNING)
        self.assertFalse(self.keys.closed)
        self.video.record_status['finished']=True;self.tick(.4)
        self.assertEqual(self.node.status,base.boot.Status.SUCCESS);self.assertTrue(self.keys.closed)

    def test_interrupt_restores_terminal_and_preserves_partial(self):
        self.start();self.node.stop(base.boot.Status.INVALID)
        self.assertTrue(self.keys.closed)
        self.assertFalse(self.video.segments[0]['interactive']['segment_complete'])
        self.assertEqual(base.boot.runtime.left_motor.power,0)

    def test_recording_failure_stops_interactive_session(self):
        self.start();self.video.record_status['error']='disk'
        self.tick(.3);self.tick(.32)
        self.assertEqual(self.node.status,base.boot.Status.FAILURE);self.assertTrue(self.keys.closed)

    def test_repetition_is_in_sensor_and_video_rows(self):
        self.start();child=self.node.child
        self.assertEqual(child.samples[0]['repeat'],1)
        self.assertEqual(child._record_snapshot()['probe_repeat'],1)
        self.assertEqual(child.samples[0]['interactive_session_id'],self.node.batch)

    def test_actual_writer_keeps_interactive_repetition_columns(self):
        import tempfile,csv,json
        self.start();child=self.node.child
        with tempfile.TemporaryDirectory(prefix='sensor-key-writer-') as directory:
            writer=base.SumoAlignRecording(output_dir=directory)
            row=dict(child._record_snapshot(),frame_id=1,capture_monotonic_sec=.1)
            writer.submit(base.np.zeros((480,640,3),base.np.uint8),base.np.zeros((180,320,3),base.np.uint8),row)
            writer.request_stop('SUCCESS','partial',details=child._details());writer.close()
            summary=json.loads(writer.paths['summary'].read_text(encoding='utf-8'))
            self.assertFalse(summary['details']['interactive']['segment_complete'])
            with Path(summary['paths']['sensor_samples']).open(encoding='utf-8',newline='') as source:
                sample=next(csv.DictReader(source))
            self.assertEqual(sample['repeat'],'1');self.assertEqual(sample['interactive_session_id'],self.node.batch)
            with writer.paths['csv'].open(encoding='utf-8',newline='') as source:
                self.assertEqual(next(csv.DictReader(source))['probe_repeat'],'1')

    def test_cli_interactive_does_not_require_label(self):
        with contextlib.redirect_stdout(io.StringIO()) as out:
            result=self.f.alpha().main(['left','--mission','sumo-sensor-test','--sumo-sensor-interactive','--check-tree'])
        self.assertEqual(result,0)
        self.assertIn('interactive stationary sensor measurement only',out.getvalue())
        config=replace(self.f.config,sumo_sensor=self.settings)
        self.assertIsInstance(build_sumo_sensor_test(self.f.context,config),target.InteractiveSumoSensorTest)

    def test_interactive_flag_cannot_enable_other_missions(self):
        with contextlib.redirect_stdout(io.StringIO()),contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit) as error:
            self.f.alpha().main(['left','--mission','sumo-catch-test','--sumo-catch-distance-mm','500','--sumo-sensor-interactive','--check-tree'])
        self.assertEqual(error.exception.code,2)


class TerminalTests(unittest.TestCase):
    def test_non_terminal_is_rejected(self):
        keys=keys_module.TerminalKeys(Mock(isatty=lambda:False))
        with self.assertRaisesRegex(RuntimeError,'terminal stdin'):keys.open()

    def test_linux_poll_nonblocking_and_terminal_restored_once(self):
        term=SimpleNamespace(tcgetattr=Mock(return_value=['old']),tcsetattr=Mock(),tcflush=Mock(),TCIFLUSH=0,TCSADRAIN=1)
        tty=SimpleNamespace(setcbreak=Mock())
        stream=Mock(isatty=lambda:True,fileno=lambda:10)
        with patch.object(keys_module.sys,'platform','linux'),patch.dict(sys.modules,{'termios':term,'tty':tty}),\
             patch.object(keys_module.select,'select',return_value=([10],[],[])) as poll,\
             patch.object(keys_module.os,'read',return_value=b'5 q'):
            keys=keys_module.TerminalKeys(stream);keys.open()
            self.assertEqual(keys.poll(),['5',' ','q']);poll.assert_called_once_with([10],[],[],0)
            keys.close();keys.close()
        tty.setcbreak.assert_called_once_with(10);term.tcsetattr.assert_called_once_with(10,1,['old'])

    def test_setup_error_restores_original_terminal(self):
        term=SimpleNamespace(tcgetattr=Mock(return_value=['old']),tcsetattr=Mock(),tcflush=Mock(side_effect=RuntimeError('setup')),TCIFLUSH=0,TCSADRAIN=1)
        tty=SimpleNamespace(setcbreak=Mock())
        with patch.object(keys_module.sys,'platform','linux'),patch.dict(sys.modules,{'termios':term,'tty':tty}):
            keys=keys_module.TerminalKeys(Mock(isatty=lambda:True,fileno=lambda:10))
            with self.assertRaises(RuntimeError):keys.open()
        term.tcsetattr.assert_called_once_with(10,1,['old'])

    def test_eof_requests_graceful_quit(self):
        keys=keys_module.TerminalKeys();keys.fd=10
        with patch.object(keys_module.select,'select',return_value=([10],[],[])),patch.object(keys_module.os,'read',return_value=b''):
            self.assertEqual(keys.poll(),['q'])


if __name__=='__main__':unittest.main(argv=[__file__]+base.boot.TEST_ARGS)
