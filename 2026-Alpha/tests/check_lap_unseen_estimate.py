"""Test real LAP decision methods with fake sensors and recorded landmarks."""
import ast,math,pathlib,types
p=pathlib.Path(__file__).resolve().parents[1]/'robot_program'/'features'/'lap_camera_recovery.py';t=ast.parse(p.read_text(encoding='utf8'));c=next(x for x in t.body if isinstance(x,ast.ClassDef) and x.name=='Peak100ContinuousLap'); wanted={'_transition_update','_unseen_estimate_plan','_unseen_blue_evidence','update'};methods=[x for x in c.body if isinstance(x,ast.FunctionDef) and x.name in wanted];module=ast.fix_missing_locations(ast.Module(body=[ast.ClassDef(name='Probe',bases=[],keywords=[],decorator_list=[],body=methods)],type_ignores=[]))
class Projection:
 def __init__(self):self.traveled=0
 def update(self,*a):return self.traveled
class Gate:
 def __init__(self):self.saw_blue=False;self.weak_blue_progress=None;self.weak_blue_samples=0;self.status='RUNNING';self.last_hsv=(0,0,99);self.last_hsv_time=10.;self.blue_progress=None;self.black_progress=None;self.measure_projection=Projection()
 def tick_once(self):pass
runtime=types.SimpleNamespace(course=-1,plotter=types.SimpleNamespace(get_distance=lambda:pos[0]),gyro_sensor=types.SimpleNamespace(get_angle=lambda:0),color_sensor=types.SimpleNamespace(get_raw_color_hsv=lambda:(0,0,99)))
mode=[160.];pos=[5000.]
ns={'runtime':runtime,'math':math,'time':types.SimpleNamespace(monotonic=lambda:10.),'Status':types.SimpleNamespace(SUCCESS='SUCCESS',FAILURE='FAILURE',RUNNING='RUNNING'),'unseen_estimate_distance':lambda:mode[0],'UNSEEN_ESTIMATE_SLOW_BEFORE_GATE_MM':190.,'UNSEEN_ESTIMATE_MIN_SLOW_MM':50.,'UNSEEN_GATE_FALLBACK_AFTER_MM':100.,'FALLBACK_AFTER_GATE_MM':150.,'BLUE_TO_BLACK_LIMIT_MM':300.,'TRACE_POWER':40,'blue_unconfirmed_limit_mm':lambda:350.,'confirmed_black_approach_mm':lambda gate,n:(n,'normal_black')};exec(compile(module,str(p),'exec'),ns)
def setup():
 o=ns['Probe']();o.transition_phase='FALLBACK';o.trace_projection=Projection();o.origin_distance=0.;o.fallback_estimated_gate_mm=5290.54;o.camera_start_mm=4283.;o.gate_latched=False;o.fallback_gate_base=None;o.unseen_estimate_started=False;o.unseen_estimate_cancelled=False;o.confirmed_fallback_slow=True;o.fallback_gate_slow=False;o.confirmed_fallback_after_mm=150.;o.fallback_done=False;o.heading_origin=0.;o.completion_condition=Gate();o.logger=types.SimpleNamespace(info=lambda *x:None,warning=lambda *x:None);o.projected_progress=lambda:pos[0];o._capture_transition_context=lambda *x:None;o._transition_drive=lambda *a,**kw:setattr(o,'last_slow',kw.get('slow'));o._finish_distance_run=lambda status:status;o._start_fallback=lambda reason:'FALLBACK:'+reason;return o
def tick(o,n):pos[0]=n;return o._transition_update()
for option in (160.,200.):
 mode[0]=option;o=setup();assert tick(o,5100)=='RUNNING' and not o.last_slow;assert tick(o,5101)=='RUNNING' and o.last_slow;assert tick(o,5100.54+option-.1)=='RUNNING';assert tick(o,5100.54+option)=='SUCCESS' and o.fallback_done
 print('estimate',option,'slows after',5100.54,'ends at',5100.54+option)
mode[0]=160.;o=setup();tick(o,5101);g=o.completion_condition;g.saw_blue=True;g.blue_progress=5130.;assert tick(o,5130)=='RUNNING' and o.unseen_estimate_cancelled and o.last_slow;g.status='SUCCESS';g.black_progress=5220.;g.measure_projection.traveled=149.;assert tick(o,5369)=='RUNNING';g.measure_projection.traveled=150.;assert tick(o,5370)=='SUCCESS';print('late blue/black cancels estimate and uses 150mm')
o=setup();o.completion_condition.weak_blue_samples=1;assert tick(o,5101)=='RUNNING' and not o.unseen_estimate_started;print('pending weak blue preserves detection')
o=setup();assert tick(o,5295)=='RUNNING' and o.unseen_estimate_cancelled and not o.unseen_estimate_started and not o.last_slow;print('late fallback retains baseline rather than immediate stop')
mode[0]=None;o=setup();assert tick(o,5101)=='RUNNING' and not o.unseen_estimate_started and not o.last_slow;print('default off unchanged')
mode[0]=160.;o=setup();o.transition_phase='CROSS';assert tick(o,5101)=='FALLBACK:unseen_estimate';print('active transition switches at estimate when no blue')

mode[0]=160.;pos[0]=5101.;o=setup();o.running=True;o.transition_phase=None;o.trace_projection=None;o.camera_start_mm=4283.;assert o.update()=='FALLBACK:unseen_estimate';print('active profile checks gate and starts estimated fallback')
o=setup();o.running=True;o.transition_phase=None;o.trace_projection=None;o.camera_start_mm=4283.;o.completion_condition.tick_once=lambda:setattr(o.completion_condition,'status','SUCCESS');assert o.update()=='SUCCESS' and o.gate_latched;print('gate confirmation wins before estimated fallback')


