"""ETラリー開始時のエラーで使う固定ルート(et_rally_fallback.py)の試験。

実行(2026-Alphaで): python -m unittest robot_program.tests.test_et_rally_fallback -v
ロボット実機用のライブラリが無い環境でも動くよう、無いモジュールは代用品に置き換える。
"""

import importlib.util
import math
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import MagicMock

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

if "etrobo_python" not in sys.modules:
    stub = types.ModuleType("etrobo_python")
    for _name in ("ETRobo", "Hub", "Motor", "TouchSensor", "ColorSensor", "SonarSensor", "GyroSensor"):
        setattr(stub, _name, type(_name, (), {}))
    sys.modules["etrobo_python"] = stub
for _name in ("cv2", "numpy", "pyzbar", "pyzbar.pyzbar", "PIL"):
    try:
        __import__(_name)
    except ImportError:
        sys.modules[_name] = MagicMock()

from py_trees.common import Status  # noqa: E402

from robot_program import et_rally_fallback as fb  # noqa: E402
from robot_program.context import RaceContext  # noqa: E402
from robot_program.features.execute_strategy import DeferredStrategySequence  # noqa: E402
from robot_program.features.receive_strategy import WaitForStrategy  # noqa: E402


def _load_planner_config():
    path = ROOT / "wireless_device" / "et_rally_planner" / "config.py"
    spec = importlib.util.spec_from_file_location("planner_config_for_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FallbackRouteTest(unittest.TestCase):
    def test_constants_match_planner_config(self):
        cfg = _load_planner_config()
        for mine, theirs in ((fb.START_POS_CM, cfg.START_POS_CM), (fb.GOAL_POS_CM, cfg.GOAL_POS_CM),
                             (fb.POINT_C_CM, cfg.KEEP_OUT_GOAL_LANE_POINT_CM)):
            self.assertAlmostEqual(mine[0], theirs[0], places=9)
            self.assertAlmostEqual(mine[1], theirs[1], places=9)
        self.assertAlmostEqual(fb.START_HEADING_DEG, cfg.START_HEADING_DEG)
        self.assertAlmostEqual(fb.GATE_EXIT_OFFSET_CM, cfg.GATE_EXIT_OFFSET_CM)

    def test_route_points(self):
        pts = fb.fallback_waypoints()
        self.assertEqual([tuple(round(v, 1) for v in p) for p in pts],
                         [(116.9, 84.1), (98.4, 86.1), (-22.0, 86.1), (-22.0, 0.0), (-75.8, -28.9)])

    def test_planner_steps_reach_the_goal(self):
        # step列から座標を復元して、ゴールに着くこと・総距離を確認する
        heading = fb.START_HEADING_DEG
        x, y = fb.START_POS_CM
        total = 0.0
        for step in fb.fallback_planner_steps():
            heading = fb.START_HEADING_DEG + step["target_heading_deg"]
            if step["type"] == "move":
                d = step["distance_mm"] / 10.0
                total += d
                x += d * math.cos(math.radians(heading))
                y += d * math.sin(math.radians(heading))
        self.assertAlmostEqual(x, fb.GOAL_POS_CM[0], delta=0.5)
        self.assertAlmostEqual(y, fb.GOAL_POS_CM[1], delta=0.5)
        self.assertAlmostEqual(total, 286.2, delta=0.2)

    def test_strategy_is_course_normalized(self):
        steps = fb.fallback_strategy()
        self.assertEqual(steps[-1]["label"], "goal")
        # 最初の向きはラリー開始の向き(-90度)に、スタート向きからのずれを足したもの
        self.assertAlmostEqual(steps[0]["target_heading_deg"], -90.0 + (-6.17), places=1)


class _Config:
    et_rally_fallback_enabled = True


class WaitForStrategyTest(unittest.TestCase):
    def test_failed_status_uses_fallback(self):
        ctx = RaceContext(hint1="45,55", hint2_gate_info="51,52/23,33")
        ctx.strategy_status, ctx.strategy_error = "failed", "Strategy response timed out"
        node = WaitForStrategy(ctx, fallback_enabled=True)
        self.assertEqual(node.update(), Status.SUCCESS)
        self.assertEqual(ctx.strategy_status, "ready")
        self.assertTrue(ctx.strategy_fallback_used)
        self.assertTrue(ctx.strategy)

    def test_failed_status_without_fallback_still_fails(self):
        ctx = RaceContext(hint1="45,55", hint2_gate_info="51,52/23,33")
        ctx.strategy_status, ctx.strategy_error = "failed", "x"
        self.assertEqual(WaitForStrategy(ctx, fallback_enabled=False).update(), Status.FAILURE)

    def test_missing_hints_uses_fallback(self):
        ctx = RaceContext()
        self.assertEqual(WaitForStrategy(ctx, fallback_enabled=True).update(), Status.SUCCESS)
        self.assertTrue(ctx.strategy_fallback_used)

    def test_waiting_and_ready_are_unchanged(self):
        ctx = RaceContext(hint1="45,55", hint2_gate_info="51,52/23,33")
        ctx.strategy_status = "pending"
        self.assertEqual(WaitForStrategy(ctx, fallback_enabled=True).update(), Status.RUNNING)
        ctx.strategy_status = "ready"
        self.assertEqual(WaitForStrategy(ctx, fallback_enabled=True).update(), Status.SUCCESS)
        self.assertFalse(ctx.strategy_fallback_used)


class LoaderFallbackTest(unittest.TestCase):
    def test_invalid_strategy_falls_back(self):
        def bad_loader():
            return [{"type": "move", "target_heading_deg": 0.0}]   # distance_mmが無い

        seq = DeferredStrategySequence(
            "t", bad_loader, fallback_loader=lambda: fb.fallback_planner_steps())
        seq._load_children()
        self.assertGreaterEqual(len(seq.children), 4)

    def test_invalid_strategy_without_fallback_is_failure_node(self):
        seq = DeferredStrategySequence("t", lambda: [{"type": "move"}])
        seq._load_children()
        self.assertEqual([type(c).__name__ for c in seq.children], ["Failure"])


if __name__ == "__main__":
    unittest.main()
