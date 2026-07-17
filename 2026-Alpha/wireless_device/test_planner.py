"""ハードウェアなしで実行できるヒント解析・経路計算の単体テスト。"""

import unittest
from pathlib import Path

from .planner import PlannerConfig, RoutePlanner
from .protocol import GateHint, parse_hint_set


class ProtocolTest(unittest.TestCase):
    def test_rule_examples_are_parsed_in_red_blue_yellow_order(self) -> None:
        hints = parse_hint_set("25,35", "53,54/12,22")
        self.assertEqual([gate.color for gate in hints.ordered_gates()], ["red", "blue", "yellow"])
        self.assertEqual(hints.red.encoded(), "25,35")
        self.assertEqual(hints.blue.encoded(), "53,54")
        self.assertEqual(hints.yellow.encoded(), "12,22")

    def test_non_adjacent_gate_posts_are_rejected(self) -> None:
        with self.assertRaises(ValueError):
            GateHint.parse("red", "11,55")

    def test_gate_orientation_must_follow_competition_rules(self) -> None:
        with self.assertRaisesRegex(ValueError, "赤ゲートは横向き"):
            parse_hint_set("25,24", "53,54/12,22")

    def test_reversed_duplicate_gate_is_rejected(self) -> None:
        """端点の記載順だけを逆にした同一ゲートも重複として拒否する。"""
        with self.assertRaisesRegex(ValueError, "同じ位置を重複"):
            parse_hint_set("11,21", "53,54/21,11")


class PlannerTest(unittest.TestCase):
    def setUp(self) -> None:
        config_path = Path(__file__).with_name("planner_config.json")
        self.config = PlannerConfig.load(config_path)
        self.planner = RoutePlanner(self.config)

    def test_official_course_scale_and_reference_positions_are_loaded(self) -> None:
        """公式50%図面を実寸換算した基準値が設定から読み込まれることを確認する。"""
        self.assertEqual(self.config.cell_mm, 250.0)
        self.assertEqual(self.config.obstacle_clearance_mm, 100.0)
        self.assertEqual((self.config.start_mm.x, self.config.start_mm.y), (1178.0, 857.0))
        self.assertIsNotNone(self.config.finish_mm)
        assert self.config.finish_mm is not None
        self.assertEqual((self.config.finish_mm.x, self.config.finish_mm.y), (-148.0, 876.0))

    def test_three_non_empty_laps_are_generated(self) -> None:
        route = self.planner.plan(parse_hint_set("25,35", "53,54/12,22"))
        self.assertEqual(route["type"], "route")
        self.assertEqual(route["gate_order"], ["red", "blue", "yellow"])
        self.assertEqual(len(route["laps"]), 3)
        for lap in route["laps"]:
            self.assertTrue(lap["segments"])
            self.assertTrue(all(length > 0 for _, length in lap["segments"]))

    def test_routes_are_generated_for_adjacent_gate_arrangements(self) -> None:
        """隣接ゲートの安全領域がつながりやすい配置でも3周経路を生成する。"""
        cases = (
            ("14,24", "34,35/15,25"),
            ("23,33", "31,32/12,22"),
            ("24,34", "43,44/33,43"),
            ("12,22", "32,33/13,23"),
            ("21,31", "11,12/22,32"),
            ("13,23", "12,13/22,32"),
        )
        for hint1, hint2 in cases:
            with self.subTest(hint1=hint1, hint2=hint2):
                route = self.planner.plan(parse_hint_set(hint1, hint2))
                self.assertEqual(len(route["laps"]), 3)
                self.assertTrue(all(lap["segments"] for lap in route["laps"]))


if __name__ == "__main__":
    unittest.main()
