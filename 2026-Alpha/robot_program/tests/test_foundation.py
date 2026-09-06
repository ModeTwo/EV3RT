"""Foundation checks in isolated processes; no camera or motor is opened."""

import os
from pathlib import Path
import subprocess
import sys
import unittest


ROOT = Path(__file__).resolve().parents[2]
BOOTSTRAP = r'''
import importlib.util, sys, types
from enum import Enum
from pathlib import Path
from unittest.mock import Mock, patch
root = Path.cwd()
spec = importlib.util.spec_from_file_location('actual_util', root / 'py_etrobo_util/util.py')
util = importlib.util.module_from_spec(spec)
spec.loader.exec_module(util)
fake = types.ModuleType('py_etrobo_util')
for name in ('Color', 'ColorClassifier', 'SymmetricClamper', 'LowPassFilter'):
    setattr(fake, name, getattr(util, name))
fake.TraceSide = Enum('TraceSide', 'NORMAL OPPOSITE CENTER LEFT RIGHT')
fake.TargetInterested = Enum('TargetInterested', 'LINE QRCODE BOTTLE')
fake.BottleColor = Enum('BottleColor', 'NONE RED BLUE YELLOW BLACK')
fake.HintType = Enum('HintType', 'HINT1 HINT2 UNKNOWN')
fake.Hint = Mock()
fake.Video = Mock(side_effect=AssertionError('Camera must not open in this check'))
fake.Plotter = Mock()
sys.modules['py_etrobo_util'] = fake
import alpha
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees.behaviours import Failure
from robot_program.runtime import runtime
'''


class FoundationTest(unittest.TestCase):
    def run_case(self, code):
        result = subprocess.run([sys.executable, '-B', '-c', BOOTSTRAP + code],
                                cwd=ROOT, capture_output=True, text=True,
                                env={**os.environ, 'PYTHONDONTWRITEBYTECODE': '1'})
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_full_tree_and_preflight_never_open_hardware(self):
        self.run_case('''
assert alpha.robot_runtime is runtime
with patch.object(alpha, 'initialize_etrobo', side_effect=AssertionError('No hardware')):
    assert alpha.main(['left', '--mission', 'hint2', '--check-tree']) == 0
    assert alpha.main(['right', '--mission', 'hint2', '--check-tree']) == 0
    assert alpha.main(['left', '--mission', 'lap', '--check-tree']) == 0
    assert alpha.main(['right', '--mission', 'sumo', '--check-tree']) == 0
    assert alpha.main(['left', '--check-tree']) == 0
    assert alpha.main(['left', '--mission', 'full', '--check-tree']) == 0
fake.Video.assert_not_called()
''')

    def test_configured_switches_and_each_single_mission(self):
        self.run_case('''
from robot_program.config import RaceConfig, config_for_mission, mission_requires_qr
from robot_program.context import RaceContext
from robot_program.tree_builder import build_mission_children
def names(config):
    return [node.name for node in build_mission_children(RaceContext(), config)]
off = RaceConfig(mission_mode='configured', lapgate=False,
    enable_bottle_delivery=False, enable_et_rally=False, et_rally_laps=0,
    enable_et_sumo=False, enable_finish=False)
assert names(off) == []
sumo_only = RaceConfig(mission_mode='configured', lapgate=False,
    enable_bottle_delivery=False, enable_et_rally=False, et_rally_laps=0,
    enable_et_sumo=True, enable_finish=False)
assert names(sumo_only) == ['et_sumo']
expected = {
    'lap': ['start_to_lap_gate'],
    'bottle': ['bottle_and_rally_preparation'],
    'rally': ['bottle_and_rally_preparation', 'et_rally'],
    'sumo': ['et_sumo'],
    'finish': ['finish'],
    'full': ['start_to_lap_gate', 'bottle_and_rally_preparation',
             'et_rally', 'et_sumo', 'finish'],
}
for profile, result in expected.items():
    config = config_for_mission(profile)
    assert names(config) == result, (profile, names(config))
assert not mission_requires_qr(config_for_mission('lap'))
assert not mission_requires_qr(config_for_mission('sumo'))
assert mission_requires_qr(config_for_mission('rally'))
assert mission_requires_qr(config_for_mission('hint2'))
''')

    def test_initialization_precedes_camera_and_failure_latches_stop(self):
        self.run_case('''
tree = Failure('mission failure')
handler = alpha.TraverseBehaviourTree(tree)
motors = [Mock(), Mock(), Mock()]
devices = [Mock(), *motors, Mock(), Mock(), Mock(), Mock()]
def check_refs():
    assert runtime.right_motor is motors[1]
    assert runtime.plotter is not None
with patch.object(alpha, 'start_video_thread', side_effect=check_refs) as start:
    handler(*devices)
    start.assert_called_once()
try:
    handler(*devices)
    raise AssertionError('Failure must terminate dispatch')
except RuntimeError as e:
    assert 'Mission failed' in str(e)
for motor in motors:
    motor.set_power.assert_called_with(0)
    motor.set_brake.assert_called_with(True)
''')

    def test_shutdown_continues_after_one_motor_error(self):
        self.run_case('''
from robot_program.services.execution_safety import stop_motors
runtime.right_motor = Mock()
runtime.right_motor.set_power.side_effect = OSError('disconnected')
runtime.left_motor = Mock()
runtime.arm_motor = Mock()
errors = stop_motors(runtime)
assert len(errors) == 1
runtime.right_motor.set_brake.assert_called_with(True)
runtime.left_motor.set_power.assert_called_with(0)
runtime.arm_motor.set_brake.assert_called_with(True)
alpha.cleanup_thread()
alpha.cleanup_thread()
''')

    def test_dispatch_exception_and_partial_camera_setup_cleanup(self):
        self.run_case('''
for fail_setup in (False, True):
    empty = Sequence('ready', memory=True)
    def setup():
        alpha.g_video = Mock()
        if fail_setup:
            raise OSError('camera initialization failed')
    driver = Mock()
    driver.dispatch.side_effect = OSError('dispatch failed')
    with patch.object(alpha, 'build_behaviour_tree', return_value=empty), \
         patch.object(alpha, 'setup_thread', side_effect=setup), \
         patch.object(alpha, 'initialize_etrobo', return_value=driver), \
         patch.object(alpha, 'stop_motors', return_value=[]) as stop, \
         patch.object(alpha, 'cleanup_thread') as cleanup:
        try:
            alpha.main(['left'])
            raise AssertionError('Expected error')
        except OSError:
            pass
        stop.assert_called_once_with(runtime)
        cleanup.assert_called_once()
''')

    def test_camera_error_is_reported_to_dispatch(self):
        self.run_case('''
thread = alpha.VideoThread()
with patch.object(thread, 'process_frames', side_effect=OSError('camera failed')):
    thread.run()
assert isinstance(thread.error, OSError)
alpha.g_video_thread = thread
handler = alpha.TraverseBehaviourTree(Failure('unused'))
handler.running = True
try:
    handler(*([None] * 8))
    raise AssertionError('Camera failure must propagate')
except RuntimeError as e:
    assert 'Camera processing failed' in str(e)
''')

    def test_handoff_origin_preserves_devices_and_mirrors_heading(self):
        self.run_case('''
from robot_program.behaviours.handoff import CaptureAtToHandoff
from robot_program.context import RaceContext
for course, gyro in ((1, -35), (-1, 35)):
    ctx = RaceContext()
    runtime.course = course
    runtime.gyro_sensor = Mock()
    runtime.gyro_sensor.get_angle.return_value = gyro
    runtime.plotter = Mock()
    runtime.plotter.get_distance.return_value = 1250
    node = CaptureAtToHandoff('AT_TO', ctx)
    assert node.update() == Status.SUCCESS
    assert ctx.at_to.absolute_heading(90) == 125
    assert ctx.at_to.distance_mm == 1250
    runtime.gyro_sensor.reset.assert_not_called()
    runtime.plotter.get_distance.assert_called_once()
ctx = RaceContext()
try:
    ctx.at_to.absolute_heading(0)
    raise AssertionError('Missing handoff must not silently use zero')
except RuntimeError:
    pass
''')

    def test_pending_skips_and_garage_has_explicit_stop(self):
        self.run_case('''
from robot_program.placeholder import PendingFeature
from robot_program.features.stop_in_garage import build_stop_in_garage
from robot_program.behaviours.motor_control import StopNow
from robot_program.config import RaceConfig
from robot_program.context import RaceContext
assert PendingFeature('unfinished').update() == Status.SUCCESS
garage = build_stop_in_garage(RaceContext(), RaceConfig())
assert isinstance(garage.children[-1], StopNow)
''')

    def test_re_route_contains_each_segment_once_and_keeps_source_settings(self):
        self.run_case('''
from robot_program.features.start_to_lap_gate import build_start_to_lap_gate
from robot_program.config import RaceConfig
from robot_program.context import RaceContext
from robot_program.behaviours.line_trace import TraceLine
from robot_program.behaviours.conditions import IsColorDetected
root = build_start_to_lap_gate(RaceContext(), RaceConfig())
assert [n.name for n in root.children] == ['square', 'lap2_1', 'lap2_2', 'lap2_3']
square = root.children[0]
assert len(square.children) == 5
assert [(n.children[0].target, n.children[0].power, n.children[1].delta_dist)
        for n in square.children] == [(0,70,500),(-45,70,200),(-90,70,550),(-135,70,230),(-180,60,300)]
assert len(list(root.iterate())) == len({id(n) for n in root.iterate()})
for node in root.iterate():
    if isinstance(node, TraceLine):
        assert node.pid.sample_time == alpha.EXEC_INTERVAL
        assert node.lpf is None
        assert node.target == 65
assert isinstance(root.children[-1].children[-1], IsColorDetected)
''')
