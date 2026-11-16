"""PC checks for delivery-to-sumo tree composition and settings."""
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import cv2
if not hasattr(cv2, 'wechat_qrcode_WeChatQRCode'):
    cv2.wechat_qrcode_WeChatQRCode = lambda: None

import alpha
from py_etrobo_util import BottleColor
from robot_program.config import config_for_mission, mission_requires_qr
from robot_program.runtime import runtime
from robot_program.features.sumo_push_tree import SumoPushSequence


class DeliverySumoTests(unittest.TestCase):
    def test_final_delivery_then_continuous_sumo_both_courses(self):
        for course in (-1, 1):
            for color in (BottleColor.RED, BottleColor.BLUE, BottleColor.YELLOW):
                runtime.course = course
                cfg = config_for_mission('bottle-sumo')
                self.assertFalse(mission_requires_qr(cfg))
                tree = alpha.build_behaviour_tree(cfg, bottle_color=color)
                nodes = list(tree.iterate())
                names = [n.name for n in nodes]
                curve = next(n for n in nodes if n.name == 'curve to rally entry')
                sumo = next(n for n in nodes if isinstance(n, SumoPushSequence))
                self.assertFalse(curve.stop_on_success)
                self.assertTrue(sumo.engine.continuous_entry)
                self.assertEqual(sumo.engine.settings.distance_mm, 1000)
                self.assertEqual(sumo.engine.push_settings.push_distance_mm, 1000)
                self.assertTrue(sumo.engine.execute)
                self.assertEqual(sumo.engine.approach_offset_from_reset_deg, -90 * course)
                self.assertLess(names.index('BOTTLE'), names.index(sumo.name))
                self.assertIn('Integration range complete brake', names)

    def test_full_order_and_unimplemented_sumo_to_rally_guard(self):
        runtime.course = -1
        tree = alpha.build_behaviour_tree(config_for_mission('full'))
        nodes = list(tree.iterate())
        bottle = next(n for n in nodes if n.name == 'bottle_and_rally_preparation')
        sumo = next(n for n in nodes if isinstance(n, SumoPushSequence))
        rally = next(n for n in nodes if n.name == 'et_rally')
        names = [n.name for n in nodes]
        self.assertLess(names.index(bottle.name), names.index(sumo.name))
        self.assertLess(names.index(sumo.name), names.index(rally.name))
        self.assertTrue(any('経路未実装' in n.name for n in nodes))

    def test_color_cli(self):
        for name in ('red', 'blue', 'yellow'):
            self.assertIsNotNone(alpha.read_bottle_color_for_final_mission('bottle-sumo', name, False))


if __name__ == '__main__':
    unittest.main()
