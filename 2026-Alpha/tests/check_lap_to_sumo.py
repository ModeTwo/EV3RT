"""Check full integration order, forced status propagation and stop boundary."""
import contextlib,io
import time
import cv2
if not hasattr(cv2,'wechat_qrcode_WeChatQRCode'):
 cv2.wechat_qrcode_WeChatQRCode=lambda:object()
import alpha
from py_trees.common import Status
from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence
from py_trees.decorators import Timeout
from robot_program.behaviours.continue_after_failure import ContinueAfterFailure
from robot_program.config import config_for_mission,mission_requires_camera,mission_requires_qr
class Child(Behaviour):
 def __init__(self,status):super().__init__('failure probe');self.result=status
 def update(self):return self.result
for original,expected in ((Status.FAILURE,Status.SUCCESS),(Status.SUCCESS,Status.SUCCESS),(Status.RUNNING,Status.RUNNING)):
 node=ContinueAfterFailure(name='probe',child=Child(original));node.tick_once();assert node.status==expected
timed = ContinueAfterFailure(name='force after timeout', child=Timeout(name='test timeout', child=Child(Status.RUNNING), duration=.001))
root = Sequence(name='timeout test', memory=True, children=[timed])
root.tick_once();assert root.status==Status.RUNNING
time.sleep(.01);root.tick_once();assert root.status==Status.SUCCESS
c=config_for_mission('lap-to-sumo')
assert c.lapgate and c.enable_bottle_delivery and c.enable_et_sumo
assert not c.enable_et_rally and not c.enable_finish
assert mission_requires_camera(c) and mission_requires_qr(c)
for side in ('right','left'):
 out=io.StringIO()
 with contextlib.redirect_stdout(out):
  assert alpha.main([side,'--mission','lap-to-sumo','--no-preview','--check-tree'])==0
 text=out.getvalue()
 ordered=['continue after LAP failure','LAP recovery and bottle approach','AT require recognised bottle color','AT_TO capture boundary','continue after TO failure','Tantou Section','continue after delivery failure','bottle_delivery_final','move_to_rally_ready','finish after sumo failure','align capture relative return and fixed-distance push','LAP to route complete brake']
 positions=[text.index(s) for s in ordered];assert positions==sorted(positions),(side,positions)
 assert 'LAP approach stop' not in text
 assert text.index('dump transition diagnostics') > text.index('LAP to route complete brake')
 assert 'AT gyro straight after blue' not in text
print('PASS LAP -> direct TO -> production delivery -> sumo -> stop, forced failures, both courses')
cfg=config_for_mission('full')
assert cfg.mission_mode == 'full' and cfg.enable_et_rally and cfg.et_rally_laps > 0
for side in ('right','left'):
 out=io.StringIO()
 with contextlib.redirect_stdout(out):
  assert alpha.main([side,'--mission','full','--no-preview','--check-tree'])==0
 text=out.getvalue()
 ordered=['continue after LAP failure','AT_TO capture boundary','continue after TO failure',
          'continue after delivery failure','finish after sumo failure',
          'continue after sumo-to-rally failure','③ 相撲終了位置からETラリー開始位置へ','et_rally']
 positions=[text.index(s) for s in ordered];assert positions==sorted(positions),(side,positions)
 assert 'LAP approach stop' not in text
print('PASS full order LAP -> TO -> delivery -> sumo -> rally transition -> rally, both courses')
