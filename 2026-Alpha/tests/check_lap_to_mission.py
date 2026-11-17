"""Verify the real production LAP/AT/TO tree and bounded test end."""
import contextlib
import io
import cv2
if not hasattr(cv2, 'wechat_qrcode_WeChatQRCode'):
    cv2.wechat_qrcode_WeChatQRCode = lambda: object()
import alpha
from robot_program.config import config_for_mission, mission_requires_camera, mission_requires_qr

c = config_for_mission('lap-to')
assert c.lapgate and c.enable_bottle_delivery and not c.enable_et_rally
assert not c.enable_et_sumo and not c.enable_finish
assert mission_requires_camera(c) and mission_requires_qr(c)
for side in ('right', 'left'):
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        assert alpha.main([side, '--mission', 'lap-to', '--no-preview', '--check-tree']) == 0
    text = out.getvalue()
    expected = ('LAP recovery and bottle approach', 'AT require recognised bottle color',
                'AT_TO capture boundary', 'Tantou Section', 'stop_final',
                'LAP to route complete brake')
    indices = [text.index(s) for s in expected]
    assert indices == sorted(indices), (side, indices)
    for forbidden in ('blue and bottle test', 'AT gyro straight after blue', 'bottle_delivery_final', 'execute strategy', 'ET Sumo'):
        assert forbidden not in text, (side, forbidden)
print('PASS LAP -> color and TO origin -> current full TO -> stop, both courses')

