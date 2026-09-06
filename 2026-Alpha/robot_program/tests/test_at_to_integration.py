"""Contract and directed-tick integration checks with actual motion behaviours."""
import os
from pathlib import Path
import subprocess
import sys
import unittest
from .test_foundation import BOOTSTRAP

ROOT = Path(__file__).resolve().parents[2]
SETUP = r'''
from dataclasses import replace
from robot_program.config import RaceConfig
from robot_program.context import RaceContext
from robot_program.integration_settings import IntegrationSettings
from robot_program.behaviours.section_motion import DriveDistance, LocalSpin, LocalDrive
from robot_program.behaviours.detect_bottle_color import DetectBottleColor
from robot_program.behaviours.hint_reader import ReadHintCard
from robot_program.behaviours.conditions import IsDistanceEarned, IsColorDetected
from robot_program.behaviours.gyro_drive import SpinAround, RunByGyro
from robot_program.behaviours.line_trace import TraceLine
from robot_program.timing import CONTROL_INTERVAL_SEC
from robot_program.phases.hint_collection import build_hint_collection_phase
from robot_program.tree_builder import build_mission_children
spec = importlib.util.spec_from_file_location('sessions', root/'py_etrobo_util/vision_sessions.py')
sessions = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sessions)
class Motor:
    def __init__(self): self.power, self.brake, self.writes = 0, True, []
    def set_power(self, v): self.power=v; self.writes.append(v)
    def set_brake(self, v): self.brake=v
class Video:
    def __init__(self): self.store=sessions.VisionSessions(); self.frame=0
    def set_target_interested(self, v):
        self.store.start({fake.TargetInterested.QRCODE:'qr',fake.TargetInterested.BOTTLE:'bottle'}.get(v,'line'))
    def begin_qr_read(self): return self.store.start('qr')
    def get_qr_observation(self): return self.store.get_qr()
    def begin_bottle_read(self): return self.store.start('bottle')
    def get_bottle_observation(self):
        g,f,o=self.store.get_bottle()
        return g,f,o or (False,fake.BottleColor.NONE,0,0,0,0,False)
    def bottle(self,color):
        self.frame+=1
        self.store.publish_bottle(self.store.generation,self.frame,(True,color,0,0,0,200,False))
    def qr(self,text):
        self.frame+=1
        self.store.publish_qr(self.store.generation,self.frame,text)
runtime.left_motor, runtime.right_motor, runtime.arm_motor=Motor(),Motor(),Motor()
runtime.video=Video()
runtime.plotter=Mock()
runtime.plotter.get_distance.return_value=2000
runtime.gyro_sensor=Mock()
runtime.gyro_sensor.get_angle.return_value=-15
runtime.color_sensor=Mock()
runtime.color_sensor.get_raw_color_hsv.return_value=(0,0,65)
runtime.course=1
ctx=RaceContext()
'''


class AtToIntegrationTest(unittest.TestCase):
    def run_case(self, code):
        result = subprocess.run([sys.executable, '-B', '-c', BOOTSTRAP + SETUP + code],
            cwd=ROOT, capture_output=True, text=True, env={**os.environ, 'PYTHONDONTWRITEBYTECODE':'1'})
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_at_forward_reverse_and_stop_on_both_courses(self):
        self.run_case('''
for course in (1,-1):
    runtime.course=course
    for power,distance in ((60,100),(-60,200)):
        runtime.plotter.get_distance.return_value=900
        n=DriveDistance('AT test',distance,power)
        n.tick_once()
        assert n.status==Status.RUNNING
        assert runtime.left_motor.power==power and not runtime.left_motor.brake
        runtime.plotter.get_distance.return_value=900+(distance if power>0 else -distance)
        n.tick_once()
        assert n.status==Status.SUCCESS
        assert runtime.left_motor.power==runtime.right_motor.power==0
        assert runtime.left_motor.brake and runtime.right_motor.brake
''')

    def test_bottle_requires_same_color_three_distinct_frames(self):
        self.run_case('''
n=DetectBottleColor('AT color',ctx,IntegrationSettings())
n.tick_once()
runtime.video.bottle(fake.BottleColor.RED); n.tick_once()
for _ in range(5): n.tick_once()
assert n.status==Status.RUNNING and ctx.bottle_color is None
runtime.video.bottle(fake.BottleColor.BLUE); n.tick_once()
runtime.video.bottle(fake.BottleColor.BLUE); n.tick_once()
assert n.status==Status.RUNNING
runtime.video.bottle(fake.BottleColor.BLUE); n.tick_once()
assert n.status==Status.SUCCESS and ctx.bottle_color==fake.BottleColor.BLUE.value
assert runtime.left_motor.power==0 and runtime.left_motor.brake
''')

    def test_qr2_rejects_previous_session_and_same_payload(self):
        self.run_case('''
first=ReadHintCard('first',1,ctx)
first.tick_once(); old_session=first.session
runtime.video.qr('hint-one'); first.tick_once()
assert ctx.hint1=='hint-one'
second=ReadHintCard('second',2,ctx)
second.tick_once()
assert not runtime.video.store.publish_qr(old_session,999,'old-late-result')
second.tick_once(); assert second.status==Status.RUNNING
runtime.video.qr('hint-one'); second.tick_once()
assert second.status==Status.RUNNING and ctx.hint2 is None
runtime.video.qr('hint-two'); second.tick_once()
assert second.status==Status.SUCCESS and ctx.hint2=='hint-two'
''')

    def test_recognition_timeouts_do_not_advance_to_motion(self):
        self.run_case('''
now=[0.0]
with patch('time.monotonic',side_effect=lambda: now[0]):
    n=DetectBottleColor('AT timeout',ctx,IntegrationSettings(bottle_timeout_sec=1))
    n.tick_once(); now[0]=2; n.tick_once()
    assert n.status==Status.FAILURE and runtime.left_motor.power==0
    q=ReadHintCard('QR timeout',1,ctx,timeout_sec=1)
    q.tick_once(); now[0]=4; q.tick_once()
    assert q.status==Status.FAILURE and ctx.hint1 is None
''')

    def test_common_period_used_by_every_built_pid(self):
        self.run_case('''
assert alpha.EXEC_INTERVAL==alpha.VIDEO_INTERVAL==CONTROL_INTERVAL_SEC
ctx.at_to.heading_deg=15
tree=alpha.build_behaviour_tree()
for n in tree.iterate():
    if isinstance(n,TraceLine):
        assert n.pid.sample_time==CONTROL_INTERVAL_SEC
    elif isinstance(n,(RunByGyro,SpinAround)):
        if hasattr(n,'context'): n.context.at_to.heading_deg=15
        n.update()
        assert n.pid.sample_time==CONTROL_INTERVAL_SEC
        n.terminate(Status.INVALID)
''')

    def test_at_to_complete_directed_ticks_and_optional_exit(self):
        self.run_case('''
for mode in ('hint2','hint2-return'):
  for course in (1,-1):
    runtime.course=course
    runtime.video=Video()
    runtime.plotter.get_distance.return_value=2000
    runtime.gyro_sensor.get_angle.return_value=-course*15
    ctx=RaceContext()
    tree=Sequence('RE_AT_TO integration',memory=True)
    tree.add_children(build_mission_children(ctx,RaceConfig(mission_mode=mode)))
    now=[0.0]
    with patch('time.monotonic',side_effect=lambda:now[0]):
      for step in range(200):
        tree.tick_once()
        assert tree.status!=Status.FAILURE
        if tree.status==Status.SUCCESS: break
        tip=tree.tip()
        if isinstance(tip,DriveDistance):
            runtime.plotter.get_distance.return_value=tip.start_distance+(tip.distance_mm if tip.power>0 else -tip.distance_mm)
        elif isinstance(tip,IsDistanceEarned):
            runtime.plotter.get_distance.return_value=tip.orig_dist+tip.delta_dist
        elif isinstance(tip,SpinAround):
            runtime.gyro_sensor.get_angle.return_value=-course*tip.target_heading
        elif isinstance(tip,IsColorDetected):
            runtime.color_sensor.get_raw_color_hsv.return_value=(210,90,65)
        elif isinstance(tip,DetectBottleColor): runtime.video.bottle(fake.BottleColor.RED)
        elif isinstance(tip,ReadHintCard): runtime.video.qr('hint-'+str(tip.hint_number))
        now[0]+=0.2
      assert tree.status==Status.SUCCESS, tree.tip().name
    assert ctx.hint1=='hint-1' and ctx.hint2=='hint-2'
    assert ctx.bottle_color==fake.BottleColor.RED.value
    assert ctx.at_to.distance_mm==6700  # 2000+RE4340+AT100-200+460
    assert ctx.at_to.heading_deg==15
    assert runtime.left_motor.power==runtime.right_motor.power==0
    assert runtime.left_motor.brake and runtime.right_motor.brake
    runtime.gyro_sensor.reset.assert_not_called()
''')

    def test_transfer_adjustment_changes_only_named_distance(self):
        self.run_case('''
cfg=RaceConfig(integration=IntegrationSettings(at_to_transfer_trace_mm=480,to_first_black_limit_mm=580))
t=build_hint_collection_phase(ctx,cfg)
distances={n.name:n.delta_dist for n in t.iterate() if isinstance(n,IsDistanceEarned)}
assert distances['AT_TO transfer trace distance']==480
assert distances['TO first approach distance']==580
assert distances['TO after hint1 straight distance']==385
assert distances['TO hint2 approach trace distance']==1000
''')

    def test_motion_timeout_interrupts_motor_and_prevents_next_segment(self):
        self.run_case('''
from robot_program.features.catch_bottle import build_catch_bottle
t=build_catch_bottle(ctx,RaceConfig(integration=IntegrationSettings(motion_timeout_sec=0.5)))
now=[0.0]
with patch('time.monotonic',side_effect=lambda:now[0]):
    t.tick_once(); assert runtime.left_motor.power==60
    now[0]=1; t.tick_once()
    assert t.status==Status.FAILURE
assert runtime.left_motor.power==runtime.right_motor.power==0
assert runtime.video.store.mode=='line' and ctx.bottle_color is None
''')

    def test_actual_video_worker_rejects_inflight_old_qr_and_closes(self):
        self.run_case('''
import ast, threading, time
source=ast.parse((root/'py_etrobo_util/video.py').read_text(encoding='utf-8-sig'))
cls=next(n for n in source.body if isinstance(n,ast.ClassDef) and n.name=='Video')
ns={'threading':threading,'time':time,'VisionSessions':sessions.VisionSessions,
    'TargetInterested':fake.TargetInterested,'BottleColor':fake.BottleColor,
    'cv2':Mock(),'zxingcpp':object(),'_CAP_CONFIG':{},
    **{name:object for name in ['Plotter','Hub','Motor','ColorSensor','SonarSensor','GyroSensor','TraceSide']}}
exec(compile(ast.Module(body=[cls],type_ignores=[]),'video.py','exec'),ns)
v=ns['Video'].__new__(ns['Video'])
v._vision_sessions=sessions.VisionSessions()
v._worker_stop=threading.Event(); v._closed=False
v._frame_lock=threading.Lock(); v._result_lock=threading.Lock()
v._pending_lock=threading.Lock(); v._latest_gray=None
v._cap_cfg=None; v.cap=Mock()
entered=threading.Event(); release=threading.Event(); published=threading.Event()
def decode(gray):
    if gray=='old':
        entered.set(); assert release.wait(2)
    return gray, None
v._detect_qr=decode
original_publish=v._vision_sessions.publish_qr
def publish(*a,**kw):
    result=original_publish(*a,**kw); published.set(); return result
v._vision_sessions.publish_qr=publish
try:
    first=v.begin_qr_read(); worker=v._detection_thread
    with v._frame_lock: v._latest_gray=('old',first,1)
    assert entered.wait(2)
    v.set_target_interested(fake.TargetInterested.LINE)
    second=v.begin_qr_read()
    assert v._detection_thread is worker
    release.set(); assert published.wait(2)
    assert v.get_qr_observation()==(second,-1,'')
    published.clear()
    with v._frame_lock: v._latest_gray=('new',second,2)
    assert published.wait(2)
    assert v.get_qr_observation()==(second,2,'new')
finally:
    release.set(); v.close()
assert not worker.is_alive()
v.close(); v.cap.release.assert_called_once()
''')
