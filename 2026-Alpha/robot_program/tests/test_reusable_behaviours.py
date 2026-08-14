"""Tests for reusable behaviors migrated from 2026base."""

import sys
import types
import unittest
from enum import Enum


# テスト環境にOpenCVがなくてもBehaviorの契約を確認できるよう、画像依存の型だけを置き換える。
class BottleColor(Enum):
    NONE = "None"
    RED = "Red"
    BLUE = "Blue"
    YELLOW = "Yellow"


class TargetInterested(Enum):
    LINE = "Line"
    QRCODE = "QR Code"
    BOTTLE = "Bottle"


class Color(Enum):
    UNKNOWN = "unknown"


class ColorClassifier:
    def classify(self, h, s, v):
        return Color.UNKNOWN


class SymmetricClamper:
    def __init__(self, min_val, max_val) -> None:
        self.min_val = min_val
        self.max_val = max_val

    def clamp(self, value):
        if value > 0:
            return max(self.min_val, min(value, self.max_val))
        if value < 0:
            return min(-self.min_val, max(value, -self.max_val))
        return 0


fake_util = types.ModuleType("py_etrobo_util")
fake_util.BottleColor = BottleColor
fake_util.TargetInterested = TargetInterested
fake_util.Color = Color
fake_util.ColorClassifier = ColorClassifier
fake_util.SymmetricClamper = SymmetricClamper
sys.modules["py_etrobo_util"] = fake_util

from py_trees.common import Status

from robot_program.behaviours.bottle import HasCaughtBottle, IsBottleInsight
from robot_program.behaviours.conditions import IsTimePassed
from robot_program.behaviours.gyro_drive import SpinAround
from robot_program.behaviours.hint_reader import ReadHintCard
from robot_program.behaviours.motor_control import RunAsInstructed, StopNow
from robot_program.context import RaceContext
from robot_program.features.behaviour_smoke_test import build_behaviour_smoke_test
from robot_program.runtime import runtime
from robot_program.types import HeadingType


class FakeMotor:
    def __init__(self) -> None:
        self.power = 0
        self.brake = False
        self.count = 0

    def set_power(self, power) -> None:
        self.power = power

    def set_brake(self, brake) -> None:
        self.brake = brake

    def get_count(self) -> int:
        return self.count


class FakePlotter:
    def __init__(self) -> None:
        self.distance = 0

    def get_distance(self) -> int:
        return self.distance


class FakeGyro:
    def __init__(self) -> None:
        self.angle = 0

    def get_angle(self) -> int:
        return self.angle


class FakeVideo:
    def __init__(self) -> None:
        self.target = TargetInterested.LINE
        self.qr_text = ""
        self.bottle_snapshot = (
            False,
            BottleColor.NONE,
            0,
            0.0,
            0,
            0,
            False,
        )

    def set_target_interested(self, target) -> None:
        self.target = target

    def get_QR_text(self) -> str:
        return self.qr_text

    def get_bottle_stamped(self):
        return self.bottle_snapshot


class ReusableBehaviourTest(unittest.TestCase):
    def setUp(self) -> None:
        # 各テストで同じFake実機をruntimeへ設定し、実機なしで出力と終了条件を確認する。
        runtime.right_motor = FakeMotor()
        runtime.left_motor = FakeMotor()
        runtime.arm_motor = FakeMotor()
        runtime.plotter = FakePlotter()
        runtime.gyro_sensor = FakeGyro()
        runtime.video = FakeVideo()
        runtime.course = 1

    def test_stop_now_stops_and_brakes_both_motors(self) -> None:
        runtime.right_motor.power = 50
        runtime.left_motor.power = 50
        behaviour = StopNow(name="stop")

        self.assertEqual(behaviour.update(), Status.SUCCESS)
        self.assertEqual(runtime.right_motor.power, 0)
        self.assertEqual(runtime.left_motor.power, 0)
        self.assertTrue(runtime.right_motor.brake)
        self.assertTrue(runtime.left_motor.brake)

    def test_run_as_instructed_outputs_requested_power(self) -> None:
        behaviour = RunAsInstructed(name="drive", pwm_l=30, pwm_r=40)

        self.assertEqual(behaviour.update(), Status.RUNNING)
        self.assertEqual(runtime.left_motor.power, 30)
        self.assertEqual(runtime.right_motor.power, 40)
        behaviour.terminate(Status.INVALID)
        self.assertEqual(runtime.left_motor.power, 0)
        self.assertEqual(runtime.right_motor.power, 0)

    def test_spin_around_finishes_at_target_heading(self) -> None:
        behaviour = SpinAround(
            name="spin",
            target=90,
            max_power=57,
            min_power=47,
            pid_p=0.4,
            pid_i=0.001,
            pid_d=0.03,
            target_type=HeadingType.ABSOLUTE,
        )

        self.assertEqual(behaviour.update(), Status.RUNNING)
        runtime.gyro_sensor.angle = -90
        self.assertEqual(behaviour.update(), Status.SUCCESS)

    def test_zero_second_wait_succeeds(self) -> None:
        behaviour = IsTimePassed(name="wait", delta_time=0.0)
        self.assertEqual(behaviour.update(), Status.SUCCESS)

    def test_bottle_insight_uses_consecutive_frames(self) -> None:
        runtime.video.bottle_snapshot = (
            True,
            BottleColor.RED,
            0,
            0.0,
            0,
            200,
            False,
        )
        behaviour = IsBottleInsight(
            name="find red bottle",
            color=BottleColor.RED,
            min_area=150,
            min_frames=2,
        )

        self.assertEqual(behaviour.update(), Status.FAILURE)
        self.assertEqual(behaviour.update(), Status.SUCCESS)
        self.assertEqual(runtime.video.target, TargetInterested.BOTTLE)

    def test_has_caught_bottle_reads_context(self) -> None:
        context = RaceContext(bottle_color="Red")
        behaviour = HasCaughtBottle(
            name="check red bottle",
            color=BottleColor.RED,
            context=context,
        )
        self.assertEqual(behaviour.update(), Status.SUCCESS)

    def test_read_hint_card_stores_raw_text_and_restores_line_mode(self) -> None:
        context = RaceContext()
        runtime.video.qr_text = "raw-encrypted-hint"
        behaviour = ReadHintCard(
            name="read hint2",
            hint_number=2,
            context=context,
        )

        self.assertEqual(behaviour.update(), Status.SUCCESS)
        self.assertEqual(context.hint2, "raw-encrypted-hint")
        self.assertEqual(runtime.video.target, TargetInterested.LINE)

    def test_behaviour_smoke_test_tree_can_be_built(self) -> None:
        # 実機を動かさず、全Behaviorのimportとツリーへの組み込みだけを確認する。
        root = build_behaviour_smoke_test(RaceContext(), object())
        self.assertEqual(root.name, "behaviour_smoke_test")
        self.assertEqual(len(root.children), 9)


if __name__ == "__main__":
    unittest.main()
