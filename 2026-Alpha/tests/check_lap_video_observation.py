"""Exercise real VideoThread processing with synthetic camera frames."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock,patch
import numpy as np
import cv2
if not hasattr(cv2, 'wechat_qrcode_WeChatQRCode'):
 cv2.wechat_qrcode_WeChatQRCode = Mock
from py_etrobo_util.video import Video
from py_etrobo_util import TargetInterested

class VideoLapObservationTest(unittest.TestCase):
 def test_bottle_and_line_from_same_frame_and_session_switch(self):
  with patch.object(Video,'_open_cap'):
   v=Video(preview_enabled=False)
   self.addCleanup(v.close)
   img=np.full((480,640,3),240,np.uint8)
   img[285:420,420:455]=0
   img[150:220,120:180]=(0,0,255)
   v.cap=Mock();v.cap.read.side_effect=lambda:(True,img.copy())
   session=v.begin_bottle_read()
   v.set_lap_observation(True)
   v.process(None,None,None,None,None,None,None,None)
   obs=v.get_lap_observation()
   self.assertEqual(obs['session'],session)
   self.assertEqual(obs['frame'],v.frame_id)
   self.assertTrue(obs['lap_line']['valid'])
   self.assertTrue(obs['bottle_insight'])
   self.assertEqual(obs['bottle_color'],'Red')
   v.set_target_interested(TargetInterested.LINE)
   self.assertIsNone(v.get_lap_observation())
   v.process(None,None,None,None,None,None,None,None)
   obs2=v.get_lap_observation()
   self.assertNotEqual(obs['session'],obs2['session'])
   self.assertFalse(obs2['bottle_insight'])
   v.set_lap_observation(False)
   self.assertIsNone(v.get_lap_observation())
 def test_image_saver_flushes_at_close(self):
  with tempfile.TemporaryDirectory() as tmp:
   with patch.object(Video,'_open_cap'),patch('py_etrobo_util.video.Path',lambda _p:Path(tmp)/'py_etrobo_util'/'video.py'):
    v=Video(preview_enabled=False)
    img=np.full((480,640,3),240,np.uint8);img[285:420,420:455]=0
    v.cap=Mock();v.cap.read.return_value=(True,img)
    v.begin_bottle_read();v.set_lap_observation(True,save_images=True)
    v.process(None,None,None,None,None,None,None,None)
    path=Path(v.get_lap_observation()['image']);v.close()
    self.assertTrue(path.is_file())
    self.assertGreater(path.stat().st_size,0)

if __name__=='__main__':unittest.main()
