"""Check PWM100 mixer, smooth handoff and fail-stop with fake devices."""
import enum,sys,threading,types,os
from unittest.mock import patch
os.environ['LAP_FAR_ENTRY_POLICY']='fixed60'
class Status(enum.Enum):RUNNING=1;SUCCESS=2;FAILURE=3
class Log:
 def info(self,*x):pass
 def error(self,*x):pass
 def warning(self,*x):pass
class Node:
 def __init__(self,*a,**k):self.logger=Log();self.children=[];self.args=a;self.kwargs=k;self.name=k.get("name", a[0] if a else "node")
 def add_children(self,x):self.children.extend(x)
 def tick_once(self):
  if not getattr(self,"_tick_started",False):self.initialise();self._tick_started=True
  self.status=self.update()
class Parallel(Node):pass
class Sequence(Node):pass
class Timeout(Node):pass
class Policy:
 class SuccessOnOne:pass
mods={n:types.ModuleType(n) for n in ('py_trees','py_trees.behaviour','py_trees.common','py_trees.composites','py_trees.decorators')}
mods['py_trees'].__path__=[];mods['py_trees.behaviour'].Behaviour=Node
mods['py_trees.common'].Status=Status;mods['py_trees.common'].ParallelPolicy=Policy
mods['py_trees.composites'].Parallel=Parallel;mods['py_trees.composites'].Sequence=Sequence;mods['py_trees.decorators'].Timeout=Timeout
sys.modules.update(mods)
class PID:
 def __init__(self,*a,**k):self.setpoint=k.get('setpoint',0);self.output_limits=k.get('output_limits',(-100,100));self.components=(0,0,0)
 def __call__(self,current):return max(self.output_limits[0],min(self.output_limits[1],2*(self.setpoint-current)))
p=types.ModuleType('simple_pid');p.PID=PID;sys.modules['simple_pid']=p
class Side(enum.Enum):NORMAL=1;OPPOSITE=2
import importlib.util
from pathlib import Path
spec=importlib.util.spec_from_file_location('real_color',Path(__file__).resolve().parents[1]/'py_etrobo_util'/'util.py');real_color=importlib.util.module_from_spec(spec);spec.loader.exec_module(real_color)
plotter_stub=types.ModuleType('py_etrobo_util.plotter');plotter_stub.IMU_HEADING_SIGN=1.0;sys.modules['py_etrobo_util.plotter']=plotter_stub
u=types.ModuleType('py_etrobo_util');u.__path__=[];u.TraceSide=Side;u.SymmetricClamper=object;u.Color=real_color.Color;u.ColorClassifier=real_color.ColorClassifier;sys.modules['py_etrobo_util']=u
def stub(module,**classes):
 m=types.ModuleType(module)
 for n in classes:setattr(m,n,type(n,(Node,),{}))
 sys.modules[module]=m
stub('robot_program.behaviours.line_trace',TraceLine=0)
stub('robot_program.behaviours.camera_process_benchmark',CaptureProcess=0,StartCaptureProcess=0,GuardCaptureProcess=0,FinishCaptureProcess=0)
stub('robot_program.behaviours.motor_control',StopNow=0)
stub('robot_program.gyro_scale',EnableGyroScale=0)
stub('robot_program.features.camera_merge_demo',FinishBottleColor=0)

bt=types.ModuleType('robot_program.features.bt_imports');bt.Color=real_color.Color;bt.TraceSide=Side;sys.modules[bt.__name__]=bt
stub('robot_program.behaviours.camera_line_trace',RecoverLineByCamera=0)
stub('robot_program.behaviours.conditions',IsColorDetected=0)
stub('robot_program.behaviours.section_motion',DriveDistance=0)
from robot_program.runtime import runtime
from robot_program.types import HeadingType
from robot_program.behaviours.gyro_drive import RunByGyro
stub('robot_program.behaviours.detect_bottle_color',DetectBottleColor=0)
from robot_program.features.lap_camera_recovery import Peak100ContinuousLap
class Motor:
 def __init__(self):self.power=None
 def set_power(self,x):self.power=x
 def set_brake(self,x):pass
distance=[0.];gyro=[0.]
runtime.left_motor=Motor();runtime.right_motor=Motor();runtime.course=-1
runtime.plotter=types.SimpleNamespace(get_distance=lambda:distance[0],loc_x=0.,loc_y=0.);runtime.gyro_sensor=types.SimpleNamespace(get_angle=lambda:gyro[0]);runtime.color_sensor=types.SimpleNamespace(get_raw_color_hsv=lambda:(200,20,99))
from robot_program.start_lap_calibration import calibrated_profile
from robot_program.start_lap_profile_v1 import POINTS, BLUE_START_MM, LAP_GATE_MM
from robot_program.features.start_to_lap_gate import RunByGyroSpeedDownatCurve
profile,_,_=calibrated_profile(POINTS,BLUE_START_MM,LAP_GATE_MM,first_straight_mm=600,route_scale=1.0)
kw=dict(name='lap',target=profile.heading_at,power=100,pid_p=1.8,pid_i=0,pid_d=.03,target_type=HeadingType.RELATIVE,distance_limit_mm=5290.54,feedforward_gain=1.0,wheel_tread_mm=110.,cross_track_lookahead_mm=300.,max_heading_correction_deg=8.,profile_log_interval_sec=.2)
for course in (-1,1):
 for speed in (False,True):
  runtime.course=course;distance[0]=0;gyro[0]=0
  cfg=types.SimpleNamespace(start_lap_power=100)
  a=RunByGyroSpeedDownatCurve(config=cfg,**kw) if speed else RunByGyro(**kw)
  b=Peak100ContinuousLap(**kw);b.devtk_speed_config=cfg if speed else None;b.initialise()
  for s in (0, 50, 400, 500, 600, 840, 900, 1200, 1590, 1700, 2300, 3000, 3700, 4200, 4400, 4449):
   distance[0]=s;gyro[0]=s*.01
   a.update();expected=(runtime.left_motor.power,runtime.right_motor.power,a.target_heading,a.power)
   b.update();actual=(runtime.left_motor.power,runtime.right_motor.power,b.target_heading,b.power)
   assert actual==expected,(course,speed,s,expected,actual)
print('PASS devTK actual profile vs trial same PWM and heading: both courses and drive classes, 0..4449 mm')


from robot_program.config import config_for_mission
import robot_program.features.lap_camera_recovery as v
from robot_program.features.start_to_lap_gate import build_start_to_lap_gate
config=config_for_mission('lap')
context=types.SimpleNamespace(bottle_color=None,lap_bottle_approach_result=None)
root=build_start_to_lap_gate(context,config)
assert root.children[0].__class__.__name__ == '_StartLapObservations'
run=root.children[1].kwargs['child']
assert run.children[1].lap is run.children[2].lap
lap=run.children[1].lap
assert lap.line_entry=='cross_rejoin' and lap.profile_white_rejoin
assert lap.confirmed_fallback_after_mm==200 and lap.completion_condition.allow_weak_blue
print('PASS normal LAP tree contains shared video observers and 200 mm gate approach')

runtime.video=types.SimpleNamespace(set_lap_observation=lambda *a,**k:None)
for ref,expected in (('160','RIGHT'),('186','LEFT')):
 session=types.SimpleNamespace(lock=threading.Lock(),latest=None)
 lap=types.SimpleNamespace(running=True,origin_distance=0.,live_camera_recovery=False)
 observer=v.RecentSide(session,lap);observer.initialise();distance[0]=4400
 with patch.dict(os.environ,{'LAP_CAMERA_REFERENCE_X':ref}),patch.object(v.time,'monotonic',return_value=10.0):
  for frame in (1,2,3):
   session.latest={'stream_id':'s','session':1,'frame':frame,'capture_time':10.,'lap_line':{'valid':True,'track_state':'CANDIDATE','x_px':175.,'image_width':320}}
   observer.update()
  assert observer.snapshot['side']==expected,(ref,observer.snapshot)
print('PASS native observer x=175 votes RIGHT at 160 and LEFT at 186')

for fallback,gate,expected in ((False,True,'confirmed'),(True,False,'fallback')):
 lap=types.SimpleNamespace(fallback_done=fallback,gate_latched=gate)
 ctx=types.SimpleNamespace(bottle_color='red',lap_bottle_approach_result=None)
 assert v._FinishLapApproach(ctx,lap).update()==Status.SUCCESS
 assert ctx.lap_bottle_approach_result==expected
print('PASS only completed native phase marks confirmed or fallback')

class FakeBottle(Node):
 def __init__(self,*a,**kw): super().__init__(*a,**kw);self.ticks=0;self.stops=0;self.status=None
 def tick_once(self): self.ticks+=1;self.status=Status.RUNNING
 def stop(self,status): self.stops+=1
v.DetectBottleColor=FakeBottle
lap=types.SimpleNamespace(running=True,origin_distance=0.)
detector=v._DeferredBottleObservation(types.SimpleNamespace(bottle_color=None),lap)
detector.initialise();distance[0]=4299;assert detector.update()==Status.RUNNING and detector.child.ticks==0
distance[0]=4300;assert detector.update()==Status.RUNNING and detector.child.ticks==1
distance[0]=4301;detector.update();assert detector.child.ticks==2
detector.terminate(Status.SUCCESS);assert detector.child.stops==1
print('PASS bottle recognition starts once at 4300 mm, not at START')
# Once started, the normal bottle detector must run during the post-gate phase.
lap=types.SimpleNamespace(running=True,origin_distance=0.)
detector=v._DeferredBottleObservation(types.SimpleNamespace(bottle_color=None),lap)
detector.initialise();distance[0]=4300;detector.update();lap.running=False
detector.update();assert detector.child.ticks==2
print('PASS bottle recognition continues after LAP boundary')
floor=[(212,96,62)]
runtime.color_sensor=types.SimpleNamespace(get_raw_color_hsv=lambda:floor[0])
gate=v.BlueThenBlackGate(types.SimpleNamespace(origin_distance=0.0,heading_origin=0.0),allow_weak_blue=True)
gate.initialise()
for s in (4999,5006,5014):
 distance[0]=s;assert gate.update()==Status.RUNNING
distance[0]=5084;floor[0]=(210,45,9);assert gate.update()==Status.SUCCESS
assert gate.confirmed and gate.black_progress==5084
print('PASS dark-blue three positions then black confirms gate')

class Projection:
 traveled=0.
 def update(self,plotter):return self.traveled
projection=Projection()
lap=types.SimpleNamespace(heading_origin=0.,origin_distance=0.,trace_motor_samples=__import__('collections').deque(),trace_motor_total=0,gate_stop_measure=False)
side=types.SimpleNamespace(lap=lap)
post=v.RollingCameraMerge(types.SimpleNamespace(),side,post_gate_steer='gyro')
post.initialise();post.post_gate_gyro_active=True;post.post_trace_projection=projection;post.last_trace_log=0
runtime.course=-1;gyro[0]=0.;distance[0]=5100
assert post._update_projected_trace()==Status.RUNNING
assert (runtime.left_motor.power,runtime.right_motor.power)==(40,40)
projection.traveled=199.;assert post._update_projected_trace()==Status.RUNNING
projection.traveled=200.;assert post._update_projected_trace()==Status.SUCCESS
assert (runtime.left_motor.power,runtime.right_motor.power)==(0,0)
print('PASS confirmed approach PWM40 until projected 200 mm then stops')
for mission in ('hint2','hint2-return'):
 hint=build_start_to_lap_gate(context,config_for_mission(mission))
 assert hint.__class__.__name__=='Sequence'
 assert len(hint.children)==1 and isinstance(hint.children[0],RunByGyroSpeedDownatCurve)
print('PASS hint2 routes retain original devTK profile')

# Exercise the actual transition method with recorded blue and entry-bound cases.
class GateStub:
 status=Status.RUNNING; saw_blue=True; blue_progress=5259.5; black_progress=None
 last_hsv=(208,87,72);last_hsv_time=10.;measure_projection=None
 def tick_once(self):pass
class FakeProjection:
 def update(self,plotter):return 0.
def setup_transition(phase):
 b=Peak100ContinuousLap(**kw);b.initialise();b.transition_phase=phase
 b.trace_projection=FakeProjection();b.trace_projection_base=0.;b.origin_distance=0.
 b.projected_progress=lambda:distance[0];b.completion_condition=GateStub()
 b.gate_latched=False;b.confirmed_fallback_slow=True;b.fallback_gate_base=None
 b.fallback_estimated_gate_mm=5290.5;b.heading_origin=0.;b._diag_context=None
 b.transition_start=0.;b.transition_time=0.;b.entry_seek_parent=None
 b.side_source=None;b.frozen_camera_frame=None;b.frozen_camera_stream=None
 return b
runtime.course=-1;gyro[0]=0.;distance[0]=5263.
b=setup_transition('FALLBACK')
with patch.object(v.time,'monotonic',return_value=10.):
 assert b._transition_update()==Status.RUNNING
 assert (runtime.left_motor.power,runtime.right_motor.power)==(40,40)
 assert b.fallback_gate_base is None and b.fallback_blue_slow_logged
print('PASS recorded blue during fallback slows without claiming black confirmation')

distance[0]=4808.;floor[0]=(200,20,99)
b=setup_transition('ENTRY_SEEK');b.transition_start=4716.;b.transition_time=9.8
b.completion_condition.saw_blue=False
b.side_source=types.SimpleNamespace(snapshot={'side':'RIGHT','frame':10,'stream':'s','capture_time':9.99,'x':180.,'vote_counts':{'RIGHT':3}})
with patch.object(v.time,'monotonic',return_value=10.):
 assert b._transition_update()==Status.RUNNING
 assert b.transition_phase=='RECOVER' and b.recovery_attempts==1
print('PASS entry bound on white uses a fresh directional vote to retry once')

b=setup_transition('ENTRY_SEEK');b.transition_start=4716.;b.transition_time=9.8
b.completion_condition.saw_blue=False;b.side_source=None
b._start_fallback=lambda reason:reason
with patch.object(v.time,'monotonic',return_value=10.):
 assert b._transition_update()=='entry_seek_bound'
print('PASS entry bound without fresh direction keeps fallback')

# Recorded dark ENTRY_SEEK sample with a newer right-side vote.
distance[0]=4792.;floor[0]=(210,40,11);gyro[0]=-8.
b=setup_transition('ENTRY_SEEK');b.transition_start=4717.;b.transition_time=9.8
b.completion_condition.saw_blue=False;b.completion_condition.weak_blue_progress=None;b.completion_condition.weak_blue_samples=0
b.side_source=types.SimpleNamespace(snapshot={'side':'RIGHT','frame':1068,'stream':'s','capture_time':9.98,'x':221.5,'vote_counts':{'RIGHT':3}})
called=[]
b._start_line_cross=lambda *args:called.append(args)
b._update_line_cross=lambda *args:Status.RUNNING
with patch.object(v.time,'monotonic',return_value=10.):
 assert b._transition_update()==Status.RUNNING
 assert called and called[0][3:]==(1,'dark_entry',1068)
print('PASS recorded dark entry and fresh right vote select bounded line crossing')

# A second recovery requires a newer vote; a third attempt is rejected.
distance[0]=5100.;gyro[0]=0.
b=setup_transition('TRACE');b.completion_condition.saw_blue=False;b.recovery_attempts=1
b.frozen_camera_frame=20;b.frozen_camera_stream='s'
b.side_source=types.SimpleNamespace(snapshot={'side':'LEFT','frame':20,'stream':'s','capture_time':10.,'x':100.,'vote_counts':{'LEFT':3}})
b._transition_drive=lambda *a,**k:None
with patch.object(v.time,'monotonic',return_value=10.):
 assert b._start_camera_recovery(5100.,10.)==Status.RUNNING and b.transition_phase=='REACQUIRE'
 b.side_source.snapshot['frame']=21
 assert b._start_camera_recovery(5100.,10.)==Status.RUNNING and b.recovery_attempts==2
 assert b.frozen_camera_frame==21
 b._start_fallback=lambda reason:reason
 assert b._start_camera_recovery(5100.,10.)=='recovery_attempt_limit'
print('PASS second recovery requires a newer frame; third falls back')

# Recorded run 4: first merge ends at 5055 mm, white persists, RIGHT vote at 5092.
b=setup_transition(None);b.initial_merge_end=5055.;b.camera_direction=1
b.profile_target=lambda d:0.;b.frozen_camera_frame=673;b.frozen_camera_stream='s'
b.distance_limit_mm=5290.5;b.completion_condition.saw_blue=False
b.completion_condition.weak_blue_progress=None;b.completion_condition.weak_blue_samples=0
b.side_source=types.SimpleNamespace(snapshot=None)
for dist in (5063.,5078.,5082.):
 assert not b._maybe_start_late_merge(dist,(0,5,99),10.)
b.side_source.snapshot={'side':'RIGHT','frame':700,'stream':'s','capture_time':9.98,'x':218.}
assert b._maybe_start_late_merge(5092.,(0,3,99),10.)
assert b.target(5092.)==0. and abs(b.target(5182.)-12.)<1e-6 and abs(b.target(5272.))<1e-6
assert not b._maybe_start_late_merge(5100.,(0,3,99),10.)
print('PASS recorded far-white run permits one bounded same-side merge after original ends')

for bad in ({'side':'RIGHT','frame':673,'stream':'s','capture_time':9.98,'x':218.},
            {'side':'LEFT','frame':700,'stream':'s','capture_time':9.98,'x':102.},
            {'side':'RIGHT','frame':700,'stream':'s','capture_time':8.,'x':218.},
            {'side':'RIGHT','frame':700,'stream':'s','capture_time':9.98,'x':165.}):
 b=setup_transition(None);b.initial_merge_end=5055.;b.camera_direction=1;b.profile_target=lambda d:0.
 b.frozen_camera_frame=673;b.frozen_camera_stream='s';b.distance_limit_mm=5290.5
 b.completion_condition.saw_blue=False;b.completion_condition.weak_blue_progress=None;b.completion_condition.weak_blue_samples=0
 b.side_source=types.SimpleNamespace(snapshot=bad)
 for dist in (5063.,5078.,5092.):assert not b._maybe_start_late_merge(dist,(0,5,99),10.)
 assert not b.late_merge_used
print('PASS old, wrong-side, stale and near-center votes cannot start a late merge')

b=setup_transition(None);b.initial_merge_end=5055.;b.camera_direction=1;b.profile_target=lambda d:0.
b.frozen_camera_frame=673;b.frozen_camera_stream='s';b.distance_limit_mm=5290.5
b.completion_condition.saw_blue=True;b.completion_condition.weak_blue_progress=None;b.completion_condition.weak_blue_samples=0
b.side_source=types.SimpleNamespace(snapshot={'side':'RIGHT','frame':700,'stream':'s','capture_time':9.98,'x':218.})
for dist in (5063.,5078.,5092.):assert not b._maybe_start_late_merge(dist,(0,5,99),10.)
b.completion_condition.saw_blue=False
for dist in (5141.,5151.,5161.):assert not b._maybe_start_late_merge(dist,(0,5,99),10.)
assert not b.late_merge_used
print('PASS blue detection and insufficient remaining distance block a late merge')

# Actual update/target path: successful and far initial image positions from logs.
for image_x,expected_far in ((200.,False),(184.,False),(182.5,False),(234.5,True),(243.,True),(225.5,True)):
 runtime.course=-1;distance[0]=4452.;gyro[0]=0.;floor[0]=(0,5,99)
 b=Peak100ContinuousLap(**kw);b.initialise();b.running=True;b.origin_distance=0.;b.profile_target=lambda d:0.
 b.side_source=types.SimpleNamespace(snapshot={'side':'RIGHT','frame':10,'stream':'s','capture_time':10.,'x':image_x,'vote_counts':{'RIGHT':5},'vote_frames':5})
 b.completion_condition=GateStub();b.completion_condition.saw_blue=False;b.completion_condition.weak_blue_progress=None;b.completion_condition.weak_blue_samples=0
 b.profile_white_rejoin=False;b._profile_update=lambda:Status.RUNNING
 with patch.object(v.time,'monotonic',return_value=10.):
  assert b.update()==Status.RUNNING and b.far_entry_hold==expected_far
  distance[0]=5052.;assert b.update()==Status.RUNNING
  assert abs(b.target(5052.)-(10. if expected_far else 0.))<1e-6
  if expected_far:
   assert abs(b.target(b.distance_limit_mm))<1e-6
   b.completion_condition.saw_blue=True;b.update();assert not b.far_entry_hold and b.target(5052.)==0.
print('PASS actual update keeps near cases unchanged and holds far entry until blue or bounded return')

runtime.course=-1;distance[0]=4452.;floor[0]=(0,5,99)
b=Peak100ContinuousLap(**kw);b.initialise();b.running=True;b.origin_distance=0.;b.profile_target=lambda d:0.
b.side_source=types.SimpleNamespace(snapshot={'side':'RIGHT','frame':10,'stream':'s','capture_time':10.,'x':234.5,'vote_counts':{'RIGHT':5},'vote_frames':5})
b.completion_condition=GateStub();b.completion_condition.saw_blue=False;b.completion_condition.weak_blue_progress=None;b.completion_condition.weak_blue_samples=0
b.profile_white_rejoin=True;b._profile_update=lambda:Status.RUNNING
with patch.object(v.time,'monotonic',return_value=10.):
 b.update();distance[0]=4864.
 assert b._maybe_start_profile_rejoin(4864.,(0,5,99)) is None and b.transition_phase is None
with patch.object(v.time,'monotonic',return_value=11.):
 b.update();assert not b.far_entry_hold and b.far_entry_disabled_reason=='camera_or_vote_unavailable' and b.target(4864.)==0.
print('PASS far white defers reversal and stale camera cancels hold')

# Real py_trees initialises the gate only on its first tick. The far-entry
# guard can run earlier, immediately after creating the first camera target.
runtime.course=-1;distance[0]=4457.;floor[0]=(0,5,99)
b=Peak100ContinuousLap(**kw);b.initialise();b.running=True;b.origin_distance=0.;b.profile_target=lambda d:0.
b.side_source=types.SimpleNamespace(snapshot={'side':'RIGHT','frame':840,'stream':'s','capture_time':10.,'x':276.5,'vote_counts':{'RIGHT':5},'vote_frames':5})
b.completion_condition=v.BlueThenBlackGate(b,allow_weak_blue=True)
assert not hasattr(b.completion_condition,'saw_blue')
b.profile_white_rejoin=True;b._profile_update=lambda:Status.RUNNING
with patch.object(v.time,'monotonic',return_value=10.):
 assert b.update()==Status.RUNNING and b.far_entry_hold
print('PASS real gate before its first tick does not crash far-entry update')

# Recorded reverse-vote events: white and still heading towards the initial line.
for initial_x,raw_progress,recorded_heading in ((234.5,4864.,11.),(243.,4879.,9.),(225.5,4867.,11.),(207.5,4896.,10.)):
 runtime.course=-1;distance[0]=raw_progress;gyro[0]=recorded_heading
 b=Peak100ContinuousLap(**kw);b.initialise();b.running=True;b.origin_distance=0.;b.heading_origin=0.
 b.camera_direction=1;b.frozen_camera_frame=10;b.frozen_camera_stream='s';b.far_entry_hold_end=5110.5
 b.profile_rejoin_source=types.SimpleNamespace(snapshot={'side':'LEFT','frame':20,'stream':'s','capture_time':9.98,'x':125.})
 b.completion_condition=GateStub();b.completion_condition.saw_blue=False;b.completion_condition.weak_blue_progress=None;b.completion_condition.weak_blue_samples=0
 b.profile_white_samples=3;b.profile_white_progress=raw_progress-1
 with patch.dict(os.environ,{'LAP_FAR_ENTRY_POLICY':'evidence'}),patch.object(v.time,'monotonic',return_value=10.):
  assert b._maybe_start_profile_rejoin(raw_progress,(0,5,99)) is None and b.far_entry_hold
print('PASS four recorded white/heading/opposite-vote cases defer reversal without initial pixel cutoff')

runtime.course=-1;gyro[0]=10.;distance[0]=4864.
b=Peak100ContinuousLap(**kw);b.initialise();b.running=True;b.origin_distance=0.;b.heading_origin=0.
b.camera_direction=1;b.frozen_camera_frame=10;b.frozen_camera_stream='s';b.far_entry_hold_end=5110.5
b.profile_rejoin_source=types.SimpleNamespace(snapshot={'side':'LEFT','frame':20,'stream':'s','capture_time':8.,'x':125.})
b.completion_condition=GateStub();b.completion_condition.saw_blue=False;b.completion_condition.weak_blue_progress=None;b.completion_condition.weak_blue_samples=0
b.profile_white_samples=3;b.profile_white_progress=4863.
with patch.dict(os.environ,{'LAP_FAR_ENTRY_POLICY':'evidence'}),patch.object(v.time,'monotonic',return_value=10.):
 assert b._maybe_start_profile_rejoin(4864.,(0,5,99)) is None and not b.far_entry_hold
print('PASS stale opposite vote cannot trigger evidence hold')

def grace_lap():
 runtime.course=-1;distance[0]=4457.;gyro[0]=0.;floor[0]=(0,5,99)
 b=Peak100ContinuousLap(**kw);b.initialise();b.running=True;b.origin_distance=0.;b.profile_target=lambda d:0.
 session=types.SimpleNamespace(lock=threading.Lock(),latest={'stream_id':'s','session':1,'frame':10,'capture_time':10.})
 b.side_source=types.SimpleNamespace(snapshot={'side':'RIGHT','frame':10,'stream':('s',1),'capture_time':10.,'x':245.5,'vote_counts':{'RIGHT':5},'vote_frames':5},session=session,last_reason='majority_ready')
 b.completion_condition=GateStub();b.completion_condition.saw_blue=False;b.completion_condition.weak_blue_progress=None;b.completion_condition.weak_blue_samples=0
 b.profile_white_rejoin=False;b._profile_update=lambda:Status.RUNNING
 with patch.dict(os.environ,{'LAP_FAR_ENTRY_POLICY':'fixed60'}),patch.object(v.time,'monotonic',return_value=10.):
  b.update();assert b.far_entry_hold
 return b,session

b,session=grace_lap();b.side_source.snapshot=None;b.side_source.last_reason='no_majority'
distance[0]=4810.;session.latest={'stream_id':'s','session':1,'frame':11,'capture_time':10.01}
with patch.object(v.time,'monotonic',return_value=10.02):
 b.update();assert b.far_entry_hold and b.far_vote_grace_start==10.02
distance[0]=4825.;session.latest={'stream_id':'s','session':1,'frame':12,'capture_time':10.06}
with patch.object(v.time,'monotonic',return_value=10.07):
 b.update();assert b.far_entry_hold
b.side_source.snapshot={'side':'LEFT','frame':13,'stream':('s',1),'capture_time':10.08,'x':100.}
with patch.object(v.time,'monotonic',return_value=10.09):
 b.update();assert b.far_entry_hold and b.far_vote_grace_start is None
print('PASS fresh advancing images keep hold through brief vote gap and restored vote clears grace')

for kind in ('frozen','wrong_stream','blue','distance','duration','reverse_frame'):
 b,session=grace_lap();b.side_source.snapshot=None;b.side_source.last_reason='no_majority';distance[0]=4810.
 session.latest={'stream_id':'s','session':1,'frame':11,'capture_time':10.01}
 with patch.object(v.time,'monotonic',return_value=10.02):b.update()
 if kind=='frozen':session.latest['capture_time']=9.8
 if kind=='wrong_stream':session.latest['stream_id']='other'
 if kind=='blue':b.completion_condition.saw_blue=True
 if kind=='distance':distance[0]=4851.
 if kind=='reverse_frame':session.latest['frame']=10
 t=10.13 if kind=='duration' else 10.05
 if kind=='duration':session.latest['capture_time']=10.12;session.latest['frame']=12
 with patch.object(v.time,'monotonic',return_value=t):b.update()
 assert not b.far_entry_hold,(kind,b.far_entry_disabled_reason)
print('PASS frozen/wrong stream/blue/40mm/100ms/reversed frame cancel hold')

# Recorded run 4: blue at 5204 mm, black never arrives during fallback.
for cap in (300,350,450):
 runtime.course=-1;gyro[0]=0.;distance[0]=5204.+cap-1
 b=setup_transition('FALLBACK');b.completion_condition.blue_progress=5204.
 b._finish_distance_run=lambda status:status
 with patch.dict(os.environ,{'LAP_BLUE_UNCONFIRMED_LIMIT_MM':str(cap)}):
  assert b._transition_update()==Status.RUNNING and not b.fallback_done
  distance[0]=5204.+cap
  assert b._transition_update()==Status.SUCCESS and b.fallback_done and not b.gate_latched
print('PASS unconfirmed blue is bounded at 300/350/450 without asserting black')

# Confirmed black keeps its 200 mm approach, even if the unconfirmed limit passes.
b=setup_transition('FALLBACK');b.completion_condition.blue_progress=5204.;b.gate_latched=True
b.fallback_gate_base=5290.;b.confirmed_fallback_after_mm=200.;b.completion_condition.measure_projection=Projection()
b.completion_condition.measure_projection.traveled=199.;distance[0]=5555.;b._finish_distance_run=lambda status:status
with patch.dict(os.environ,{'LAP_BLUE_UNCONFIRMED_LIMIT_MM':'300'}):
 assert b._transition_update()==Status.RUNNING
 b.completion_condition.measure_projection.traveled=200.
 assert b._transition_update()==Status.SUCCESS and b.fallback_done
print('PASS confirmed black preserves existing 200mm projected approach')

# 150 mm separates ordinary blue->black readings from the recorded late cases.
for cap in (300,350):
 with patch.dict(os.environ,{'LAP_BLUE_UNCONFIRMED_LIMIT_MM':str(cap)}):
  for gap in (39.4, 78.4, 100.8, 138.8, 150.):
   value,reason=v.confirmed_black_approach_mm(types.SimpleNamespace(blue_progress=5204.,black_progress=5204.+gap),200.)
   assert value==200. and reason=='normal_black',(cap,gap,value,reason)
  value,reason=v.confirmed_black_approach_mm(types.SimpleNamespace(blue_progress=5204.,black_progress=5456.1),200.)
  assert abs(value-(cap-252.1))<1e-6 and reason=='late_black'
print('PASS normal black retains 200mm; late black respects 300/350 total')

b=setup_transition('FALLBACK');b.completion_condition.blue_progress=5204.;b.completion_condition.black_progress=5456.1
b.gate_latched=True;b.fallback_gate_base=5456.1;b.confirmed_fallback_after_mm=200.
b.completion_condition.measure_projection=Projection();b.completion_condition.measure_projection.traveled=97.
distance[0]=5555.;b._finish_distance_run=lambda status:status
with patch.dict(os.environ,{'LAP_BLUE_UNCONFIRMED_LIMIT_MM':'350'}):
 assert b._transition_update()==Status.RUNNING
 b.completion_condition.measure_projection.traveled=98.
 assert b._transition_update()==Status.SUCCESS and b.fallback_done
print('PASS actual fallback stops at calculated late-black distance')

g=types.SimpleNamespace(confirmed=True,blue_progress=5204.,black_progress=5456.1,measure_projection=Projection())
lap=types.SimpleNamespace(heading_origin=0.,origin_distance=0.,fallback_done=False,completion_condition=g,
                          confirmed_fallback_after_mm=200.,camera_direction=1)
post=v.RollingCameraMerge(types.SimpleNamespace(),types.SimpleNamespace(lap=lap,snapshot=None),post_gate_steer='gyro')
post.initialise();post._update_projected_trace=lambda:Status.RUNNING
with patch.dict(os.environ,{'LAP_BLUE_UNCONFIRMED_LIMIT_MM':'350'}):
 assert post.update()==Status.RUNNING and abs(post.post_trace_target_mm-97.9)<1e-6
print('PASS post-gate gyro path uses late-black total limit')

# Missing final zero segment never selects a camera vote or stops the robot early.
b=Peak100ContinuousLap(**kw);b.initialise();b.running=True;b.origin_distance=0.
b.camera_start_mm=None;b.camera_deadline_mm=None;b.observation_start_mm=None
b._profile_update=lambda:Status.RUNNING;b._start_fallback=lambda reason:reason
distance[0]=5200.;assert b.update()==Status.RUNNING
distance[0]=5291.;assert b.update()=='no_final_zero_window'
print('PASS missing final zero window preserves profile until bounded fallback')

# Entry timing depends on profile distance and fresh vote, not actual gyro.
for actual_gyro in (-30., 0., 30.):
 runtime.course=-1;distance[0]=4450.;gyro[0]=actual_gyro;floor[0]=(0,5,99)
 b=Peak100ContinuousLap(**kw);b.initialise();b.running=True;b.origin_distance=0.;b.profile_target=lambda d:0.
 b.side_source=types.SimpleNamespace(snapshot={'side':'RIGHT','frame':10,'stream':'s','capture_time':10.,'x':200.,'vote_counts':{'RIGHT':5},'vote_frames':5})
 b.completion_condition=GateStub();b.completion_condition.saw_blue=False;b.completion_condition.weak_blue_progress=None;b.completion_condition.weak_blue_samples=0
 b.profile_white_rejoin=False;b._profile_update=lambda:Status.RUNNING
 with patch.object(v.time,'monotonic',return_value=10.):
  assert b.update()==Status.RUNNING and b.camera_direction==1
print('PASS first decision does not gate on measured gyro angle')

# The 84 px boundary is provisional and preserves the recorded gate-positive
# initial offsets (maximum 79.5 px); it is not a floor distance.
def adaptive_probe(offset, peak='12', side='RIGHT', age=0.02):
 b=Peak100ContinuousLap(**kw);b.initialise()
 b.adaptive_initial_progress=4545.;b.adaptive_initial_offset=offset
 b.camera_direction=1;b.frozen_camera_frame=10;b.frozen_camera_stream='s'
 b.completion_condition=GateStub();b.completion_condition.saw_blue=False
 b.completion_condition.weak_blue_progress=None;b.completion_condition.weak_blue_samples=0
 b.profile_rejoin_source=types.SimpleNamespace(snapshot=None)
 with patch.dict(os.environ,{'LAP_FAR_PEAK_DEG':peak}):
  for p in (4550.,4560.,4570.):b._update_adaptive_peak(p,(0,3,99),10.)
  b.profile_rejoin_source.snapshot={'side':side,'stream':'s','frame':11,'capture_time':10.-age}
  b._update_adaptive_peak(4722.,(0,3,99),10.)
 return b

for offset in (0.,20.,50.,60.,73.5,75.,77.5,79.5):
 assert adaptive_probe(offset).adaptive_boost_start is None,offset
for offset in (84.,88.5):
 assert adaptive_probe(offset).adaptive_boost_start==4722.,offset
assert adaptive_probe(88.5,peak='10').adaptive_boost_start is None
assert adaptive_probe(88.5,side='LEFT').adaptive_boost_start is None
assert adaptive_probe(88.5,age=1.).adaptive_boost_start is None
print('PASS adaptive peak preserves recorded gate-positive offsets and requires fresh same-side vote')

b=adaptive_probe(88.5)
b._update_adaptive_peak(4730.,(0,4,70),10.)
assert b.adaptive_boost_cancelled
b=adaptive_probe(88.5);b.completion_condition.saw_blue=True
b._update_adaptive_peak(4730.,(200,90,70),10.)
assert b.adaptive_boost_cancelled
print('PASS adaptive peak ends on nonwhite floor or blue candidate')

# Exercise the actual profile target, including the gradual 100 mm ramp.
import math
runtime.course=-1;distance[0]=4545.;gyro[0]=0.;floor[0]=(0,3,99)
b=Peak100ContinuousLap(**kw);b.initialise();b.running=True;b.origin_distance=0.;b.profile_target=lambda d:0.
b.camera_start_mm=4545.;b.camera_deadline_mm=4700.;b.observation_start_mm=4545.
b.side_source=types.SimpleNamespace(snapshot={'side':'RIGHT','frame':10,'stream':'s','capture_time':10.,'x':248.5,'vote_counts':{'RIGHT':5},'vote_frames':5})
b.profile_rejoin_source=types.SimpleNamespace(snapshot=None)
b.completion_condition=GateStub();b.completion_condition.saw_blue=False;b.completion_condition.weak_blue_progress=None;b.completion_condition.weak_blue_samples=0
b.profile_white_rejoin=False;b._profile_update=lambda:Status.RUNNING
with patch.dict(os.environ,{'LAP_FAR_PEAK_DEG':'12','LAP_FAR_ENTRY_POLICY':'evidence'}),patch.object(v.time,'monotonic',return_value=10.):
 b.update()
 for p in (4550.,4560.,4570.):distance[0]=p;b.update()
 b.profile_rejoin_source.snapshot={'side':'RIGHT','frame':11,'stream':'s','capture_time':10.}
 distance[0]=4722.;b.update()
 assert b.adaptive_boost_start==4722.
 assert abs(b.target(4722.)-10.*math.sin(math.pi*(4722.-4545.)/600.)**2)<1e-6
 assert abs(b.target(4845.)-12.)<1e-6
print('PASS actual camera target ramps from 10 to 12 degrees')

# A dark-entry edge is accepted only with a new same-side vote, bounded time
# and distance, and a heading that has started toward the selected direction.
def dark_edge_probe(mode='on', side='LEFT', frame=11, age=.02, delta=79., elapsed=.2, heading=-1., weak=False, source='dark_entry'):
 runtime.course=-1;distance[0]=4612.+delta
 b=Peak100ContinuousLap(**kw);b.initialise();b.origin_distance=0.;b.heading_origin=0.;b.line_entry='cross_rejoin'
 b.completion_condition=GateStub();b.completion_condition.saw_blue=False;b.completion_condition.weak_blue_progress=1. if weak else None;b.completion_condition.weak_blue_samples=0
 b.frozen_camera_frame=10;b.frozen_camera_stream='s'
 b.side_source=types.SimpleNamespace(snapshot={'side':side,'frame':frame,'stream':'s','capture_time':10.+elapsed-age})
 b._start_pre_gate_trace=lambda hsv,d:'trace';b._transition_drive=lambda *a,**kw:None
 b._start_line_cross(4612.,10.,(210,32,24),-1,source,10)
 with patch.dict(os.environ,{'LAP_DARK_ENTRY_EDGE':mode,'LAP_TRACE_TARGET_V':'40'}):
  return b._update_line_cross(distance[0],10.+elapsed,(208,29,45),heading,-heading)
assert dark_edge_probe()=='trace'
for args in ({'mode':'off'},{'side':'RIGHT'},{'frame':10},{'age':1.},{'delta':121.},{'elapsed':.41},{'heading':1.},{'weak':True},{'source':'camera'}):
 assert dark_edge_probe(**args)!= 'trace',args
print('PASS dark-entry edge bounds, new matching vote, heading, blue guard and mode')
with patch.dict(os.environ, {}, clear=True):
 assert v.dark_entry_edge_mode() == 'off'
 assert v.far_entry_policy() == 'evidence'
 assert v.decision_offset_mm() == 145.21
print('PASS production defaults keep experimental dark edge off')

# Production tree selects the earlier decision without moving observation or deadline.
windows=[]
for offset in ('145.21','100'):
 with patch.dict(os.environ,{'LAP_DECISION_OFFSET_MM':offset}):
  root=build_start_to_lap_gate(context,config)
  lap=root.children[1].kwargs['child'].children[1].lap
  windows.append((lap.observation_start_mm,lap.camera_start_mm,lap.camera_deadline_mm))
assert windows[0][0]==windows[1][0] and windows[0][2]==windows[1][2]
assert abs(windows[0][1]-windows[1][1]-45.21)<1e-6
print('PASS production tree early decision retains observation and deadline')

# Only a successful continuous handoff may retain the motor command.
class StopProbe:
 def __init__(self):self.commands=[]
 def set_power(self,value):self.commands.append(('power',value))
 def set_brake(self,value):self.commands.append(('brake',value))
for continuous, expected_stop in ((False,True),(True,False)):
 runtime.left_motor=StopProbe();runtime.right_motor=StopProbe()
 lap=types.SimpleNamespace(gate_stop_measure=False)
 post=v.RollingCameraMerge(types.SimpleNamespace(),types.SimpleNamespace(lap=lap,snapshot=None),post_gate_steer='gyro',continuous_exit=continuous)
 post.initialise();post.post_trace_projection=types.SimpleNamespace(update=lambda plotter:200.,traveled=200.)
 post.post_trace_target_mm=200.;runtime.color_sensor=types.SimpleNamespace(get_raw_color_hsv=lambda:(0,0,99));gyro[0]=0.
 assert post._update_projected_trace()==Status.SUCCESS
 post.terminate(Status.SUCCESS)
 assert bool(runtime.left_motor.commands)==expected_stop,(continuous,runtime.left_motor.commands)
 if continuous:
  post.terminate(Status.FAILURE)
  assert ('power',0) in runtime.left_motor.commands
print('PASS continuous success retains power; normal success and cancellation stop')
