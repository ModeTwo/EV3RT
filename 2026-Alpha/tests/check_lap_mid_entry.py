"""Exercise the actual candidate and existing white recovery methods."""
import ast,math,pathlib,types
path=pathlib.Path(__file__).resolve().parents[1]/'robot_program'/'features'/'lap_camera_recovery.py';tree=ast.parse(path.read_text(encoding='utf8'));cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Peak100ContinuousLap');names={'_start_line_cross','_cross_needs_recovery','_recovery_snapshot_valid'};methods=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in names];module=ast.fix_missing_locations(ast.Module(body=[ast.ClassDef(name='Probe',bases=[],keywords=[],decorator_list=[],body=methods)],type_ignores=[]));ns={'LINE_CROSS_DARK_V':35,'LINE_CROSS_RECOVERY_WHITE_SAMPLES':3,'CAMERA_MAX_AGE_SEC':.25,'trace_target_v':lambda:40,'math':math};exec(compile(module,str(path),'exec'),ns)
def setup(hsv):
 o=ns['Probe']();o.origin_distance=0.;o.live_camera_recovery=False;o.recovery_attempts=0;o.completion_condition=types.SimpleNamespace(weak_blue_progress=None);o.logger=types.SimpleNamespace(info=lambda *a:None);o.frozen_camera_frame=1260;o.frozen_camera_stream='s';o.side_source=types.SimpleNamespace(snapshot=None)
 def start(phase,distance,now):o.transition_phase=phase;o.transition_start=distance;o.transition_time=now
 o._transition_start=start;o._start_line_cross(4400.,10.,hsv,-1,'camera',1260);return o
for label,hsv,expected in [('success1',(208,29,76),False),('failure2',(208,27,55),True),('success3',(220,37,9),False)]:
 o=setup(hsv);assert o.cross_initial_mid_tone==expected;print(label,'entry hsv',hsv,'candidate',o.cross_initial_mid_tone)
o=setup((208,27,55));assert not o._cross_needs_recovery(4401,10.01,(0,10,99))
for d,t in ((4402,10.02),(4403,10.03)):assert not o._cross_needs_recovery(d,t,(0,10,99))
o.side_source.snapshot={'side':'RIGHT','frame':1261,'stream':'s','capture_time':10.02};assert o._cross_needs_recovery(4404,10.04,(0,10,99))
for kind in ('same_side','stale','dark','no_fresh'):
 o=setup((208,27,55));o.side_source.snapshot={'side':'LEFT' if kind=='same_side' else 'RIGHT','frame':1260 if kind=='no_fresh' else 1261,'stream':'s','capture_time':9.0 if kind=='stale' else 10.02}
 if kind=='dark':o._cross_needs_recovery(4401,10.01,(200,20,20))
 for d,t in ((4402,10.02),(4403,10.03),(4404,10.04)):assert not o._cross_needs_recovery(d,t,(0,10,99)),kind
print('PASS white+fresh opposite vote; same side/stale/real dark blocked')

