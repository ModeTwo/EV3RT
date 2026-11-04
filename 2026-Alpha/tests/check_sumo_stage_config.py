import ast, pathlib, types, unittest, dataclasses
root=pathlib.Path(__file__).resolve().parents[1]
def load_functions(path, names, env):
 t=ast.parse(path.read_text(encoding='utf-8-sig'))
 nodes=[n for n in t.body if isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name in names]
 exec(compile(ast.fix_missing_locations(ast.Module(body=nodes,type_ignores=[])),str(path),'exec'),env)
 return env
settings=load_functions(root/'robot_program/features/sumo_stage_test_settings.py',{'SumoStageTestSettings'},dict(dataclass=dataclasses.dataclass,Optional=__import__('typing').Optional,math=__import__('math')))['SumoStageTestSettings']
@dataclasses.dataclass(frozen=True)
class FakeAlign:
 distance_mm:float=0
 power:int=0
@dataclasses.dataclass(frozen=True)
class FakePush:
 push_distance_mm:float=None
 capture_max_mm:float=0
 power:int=0
 turn_power:int=0
@dataclasses.dataclass(frozen=True)
class Config:
 mission_mode:str
 sumo_stage_test:object
 sumo_align:object=FakeAlign()
 sumo_push_calibration:object=None
 sumo_push_execute:bool=False
 sumo_relative_push:object=None
env=dict(replace=dataclasses.replace,RelativePushSettings=FakePush)
for name in ['build_et_sumo_after_delivery_phase','build_move_delivery_to_sumo_straight','build_sumo_push_tree','build_move_sumo_to_rally_start']:
 env[name]=(lambda n:lambda c,x:n)(name)
load_functions(root/'robot_program/features/sumo_stage_test.py',{'configured_sumo_test','build_sumo_stage_test'},env)
class Test(unittest.TestCase):
 def test_each_builder(self):
  for mode,expected in [('sumo-stage-all','build_et_sumo_after_delivery_phase'),('sumo-stage-entry','build_move_delivery_to_sumo_straight'),('sumo-stage-push','build_sumo_push_tree'),('sumo-stage-exit','build_move_sumo_to_rally_start')]:
   c=Config(mode,settings(push_distance_mm=300));c=env['configured_sumo_test'](c);self.assertEqual(env['build_sumo_stage_test'](None,c),expected);self.assertEqual(c.sumo_relative_push.push_distance_mm,300);self.assertEqual(c.sumo_align.distance_mm,1000);self.assertTrue(c.sumo_push_execute)
 def test_missing_distance(self):
  for mode in ('sumo-stage-all','sumo-stage-push'):
   with self.assertRaises(ValueError):env['configured_sumo_test'](Config(mode,settings()))
  for mode in ('sumo-stage-entry','sumo-stage-exit'):env['configured_sumo_test'](Config(mode,settings()))
 def test_validation(self):
  with self.assertRaises(ValueError):settings(initial_bearing_left_deg=float('nan'))
if __name__=='__main__':unittest.main()
