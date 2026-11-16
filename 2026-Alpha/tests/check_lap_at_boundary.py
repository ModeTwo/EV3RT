"""AT must not repeat an approach completed by LAP, including fallback."""
import cv2
# PC OpenCV may lack QR support; this test exercises no camera or QR code.
if not hasattr(cv2, 'wechat_qrcode_WeChatQRCode'):
    cv2.wechat_qrcode_WeChatQRCode = lambda: object()
from py_trees.common import Status
from robot_program.context import RaceContext
from robot_program.config import config_for_mission
from robot_program.features.catch_bottle import LapBottleApproachCompleted, build_catch_bottle

for result, standalone, expected in (
    (None, False, Status.FAILURE),
    ('confirmed', False, Status.SUCCESS),
    ('fallback', False, Status.SUCCESS),
    ('confirmed', True, Status.FAILURE),
    ('fallback', True, Status.FAILURE),
):
    context = RaceContext(bottle_color='RED', lap_bottle_approach_result=result)
    guard = LapBottleApproachCompleted(context, standalone)
    assert guard.update() == expected, (result, standalone)
    assert context.bottle_color == 'RED'
try:
    LapBottleApproachCompleted(RaceContext(lap_bottle_approach_result='unknown'), False).update()
except ValueError:
    pass
else:
    raise AssertionError('unexpected result must not trigger unbounded AT drive')

for mode in ('at', 'lap'):
    root = build_catch_bottle(RaceContext(), config_for_mission(mode))
    approach = next(child for child in root.children if child.name == 'LAP completed approach or AT approach')
    assert len(approach.children) == 2
    assert isinstance(approach.children[0], LapBottleApproachCompleted)
    assert approach.children[0].standalone == (mode == 'at')
    assert approach.children[1].name == 'gyro drive and detect bottle for configured distance'
    assert root.children[-1].name == 'AT_TO capture boundary'
    assert root.children[-2].name == 'AT require recognised bottle color'
print('PASS: LAP result gates duplicate AT movement, standalone preserved, color and TO handoff retained')

# Tick a real selector and verify a completed LAP cannot issue any AT drive.
from py_trees.behaviour import Behaviour
from py_trees.composites import Selector
class DriveProbe(Behaviour):
    def __init__(self):
        super().__init__('drive probe')
        self.calls = 0
    def update(self):
        self.calls += 1
        return Status.RUNNING
for result, expected_calls in ((None, 1), ('confirmed', 0), ('fallback', 0)):
    context = RaceContext(lap_bottle_approach_result=result)
    selector = Selector(name='boundary test', memory=True)
    drive = DriveProbe()
    selector.add_children([LapBottleApproachCompleted(context, False), drive])
    selector.tick_once()
    assert drive.calls == expected_calls, (result, drive.calls)
print('PASS: real selector never ticks AT drive after LAP confirmed/fallback completion')
