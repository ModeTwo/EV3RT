"""Tests for reusable behaviors migrated from 2026base."""

import sys
import types
import unittest
from dataclasses import replace
from enum import Enum
from unittest.mock import patch


# テスト環境にOpenCVがなくてもBehaviorの契約を確認できるよう、画像依存の型だけを置き換える。
class BottleColor(Enum):
    NONE = "None"
    RED = "Red"
    BLUE = "Blue"
    YELLOW = "Yellow"
    BLACK = "Black"


class TargetInterested(Enum):
    LINE = "Line"
    QRCODE = "QR Code"
    BOTTLE = "Bottle"


class Color(Enum):
    UNKNOWN = "unknown"
    BLUE = "blue"
    BLACK = "black"
    WHITE = "white"


class TraceSide(Enum):
    NORMAL = "Normal"


class ColorClassifier:
    def classify(self, h, s, v):
        if v <= 10:
            return Color.BLACK
        if v >= 90:
            return Color.WHITE
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
fake_util.TraceSide = TraceSide
sys.modules["py_etrobo_util"] = fake_util

from py_trees.common import Status

from robot_program.behaviours.bottle import HasCaughtBottle, IsBottleInsight
from robot_program.behaviours.conditions import IsColorDetected, IsColorTransitionDetected, IsDistanceEarned, IsTimePassed
from robot_program.behaviours.device_control import ResetDevice
from robot_program.behaviours.gyro_drive import RunByGyro, SpinAround
from robot_program.behaviours.hint_reader import ReadHintCard
from robot_program.behaviours.motor_control import RunAsInstructed, StopNow
from robot_program.context import RaceContext
from robot_program.config import RaceConfig
from robot_program.features.locate_sumo_bottle import (
    SampleSonarAtAngle,
    SelectNearestBottleAndConfigureAlignment,
)
from robot_program.features.move_to_sumo_exit import (
    ConfigureMirroredCarryCurvePwm,
    build_move_to_sumo_exit,
)
from robot_program.features.move_to_sumo_start import build_move_to_sumo_start
from robot_program.features.push_sumo_bottle import build_push_sumo_bottle
from robot_program.phases.et_sumo import build_et_sumo_phase
from robot_program.runtime import runtime
from robot_program.types import HeadingType
from robot_program.sumo_types import SumoSettings, SumoSonarSample


class FakeMotor:
    def __init__(self) -> None:
        self.power = 0
        self.brake = False
        self.count = 0
        self.reset_count_calls = 0

    def set_power(self, power) -> None:
        self.power = power

    def set_brake(self, brake) -> None:
        self.brake = brake

    def get_count(self) -> int:
        return self.count

    def reset_count(self) -> None:
        self.count = 0
        self.reset_count_calls += 1


class FakePlotter:
    def __init__(self) -> None:
        self.distance = 0

    def get_distance(self) -> int:
        return self.distance

    def get_loc_x(self) -> int:
        return 0

    def get_loc_y(self) -> int:
        return 0


class FakeGyro:
    def __init__(self) -> None:
        self.angle = 0
        self.reset_calls = 0

    def get_angle(self) -> int:
        return self.angle

    def reset(self) -> None:
        self.angle = 0
        self.reset_calls += 1


class FakeHub:
    def __init__(self) -> None:
        self.stationary = True

    def hub_imu_is_stationary(self) -> bool:
        return self.stationary


class FakeSonar:
    def __init__(self) -> None:
        self.readings = [100]
        self.index = 0

    def get_distance(self) -> int:
        value = self.readings[min(self.index, len(self.readings) - 1)]
        self.index += 1
        return value


class FakeColorSensor:
    def __init__(self) -> None:
        self.readings = [(0, 0, 50)]
        self.index = 0

    def get_raw_color_hsv(self):
        value = self.readings[min(self.index, len(self.readings) - 1)]
        self.index += 1
        return value


class FakeVideo:
    def __init__(self) -> None:
        self.target = TargetInterested.LINE
        self.thresholds = None
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

    def set_thresholds(self, gs_min, gs_max) -> None:
        self.thresholds = (gs_min, gs_max)

    def get_QR_text(self) -> str:
        return self.qr_text

    def get_bottle_stamped(self):
        return self.bottle_snapshot

    def set_bottle_color(self, color) -> None:
        self.bottle_color = color


class ReusableBehaviourTest(unittest.TestCase):
    def setUp(self) -> None:
        # 各テストで同じFake実機をruntimeへ設定し、実機なしで出力と終了条件を確認する。
        runtime.right_motor = FakeMotor()
        runtime.left_motor = FakeMotor()
        runtime.arm_motor = FakeMotor()
        runtime.hub = FakeHub()
        runtime.plotter = FakePlotter()
        runtime.gyro_sensor = FakeGyro()
        runtime.sonar_sensor = FakeSonar()
        runtime.color_sensor = FakeColorSensor()
        runtime.video = FakeVideo()
        runtime.course = 1

    def test_reset_device_resets_shared_devices_once_and_waits_for_stationary_imu(self) -> None:
        # 1回目のtickでデバイスをリセットし、静止確認が規定回数に達するまで待機する。
        behaviour = ResetDevice(name="reset", stationary_samples=2)

        self.assertEqual(behaviour.update(), Status.RUNNING)
        self.assertEqual(runtime.arm_motor.reset_count_calls, 1)
        self.assertEqual(runtime.right_motor.reset_count_calls, 1)
        self.assertEqual(runtime.left_motor.reset_count_calls, 1)
        self.assertEqual(runtime.gyro_sensor.reset_calls, 1)
        self.assertEqual(runtime.video.thresholds, (0, 55))
        self.assertEqual(runtime.video.target, TargetInterested.LINE)

        # 2回目の静止確認で完了しても、物理デバイスのリセットは繰り返さない。
        self.assertEqual(behaviour.update(), Status.SUCCESS)
        self.assertEqual(runtime.arm_motor.reset_count_calls, 1)
        self.assertEqual(runtime.right_motor.reset_count_calls, 1)
        self.assertEqual(runtime.left_motor.reset_count_calls, 1)
        self.assertEqual(runtime.gyro_sensor.reset_calls, 1)

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

    def test_distance_condition_uses_existing_fixed_distance_contract(self) -> None:
        # 共通距離条件の従来仕様である固定数値をそのまま使用する。
        behaviour = IsDistanceEarned(name="fixed distance", delta_dist=100)
        self.assertEqual(behaviour.update(), Status.RUNNING)
        runtime.plotter.distance = 100
        self.assertEqual(behaviour.update(), Status.SUCCESS)

    def test_sumo_start_drives_through_black_to_white_then_turns_to_ring(self) -> None:
        # No.15は黒→白を時間判定し、クリアランス直進後に土俵方向へ90度旋回する。
        feature = build_move_to_sumo_start(RaceContext(), RaceConfig())
        self.assertEqual(feature.children[0].name, "drive across black line to white area")
        transition = next(
            child for child in feature.children[0].children
            if isinstance(child, IsColorTransitionDetected)
        )
        self.assertEqual(transition.from_color, Color.BLACK)
        self.assertEqual(transition.to_color, Color.WHITE)
        self.assertEqual(transition.from_duration_sec, 0.5)
        self.assertEqual(transition.to_duration_sec, 0.5)
        self.assertIsInstance(feature.children[1], StopNow)
        self.assertEqual(
            feature.children[2].name,
            "drive clearance distance after leaving black line",
        )
        clearance_distance = next(
            child for child in feature.children[2].children
            if isinstance(child, IsDistanceEarned)
        )
        self.assertEqual(clearance_distance.delta_dist, 100.0)
        self.assertIsInstance(feature.children[3], StopNow)
        self.assertIsInstance(feature.children[4], SpinAround)
        self.assertEqual(feature.children[4].target, 90)
        self.assertEqual(feature.children[4].target_type, HeadingType.RELATIVE)
        self.assertIsInstance(feature.children[5], StopNow)

    def test_sumo_ring_turn_is_left_on_left_and_right_on_right_course(self) -> None:
        # 同じ正角度指定がLeftでは物理左、Rightでは物理右へ鏡像化されることを確認する。
        left_feature = build_move_to_sumo_start(RaceContext(), RaceConfig())
        left_turn = next(child for child in left_feature.children if isinstance(child, SpinAround))

        self.assertEqual(left_turn.update(), Status.RUNNING)
        self.assertGreater(runtime.right_motor.power, 0)
        self.assertLess(runtime.left_motor.power, 0)
        left_turn.terminate(Status.INVALID)

        right_feature = build_move_to_sumo_start(RaceContext(), RaceConfig())
        runtime.course = -1
        right_turn = next(child for child in right_feature.children if isinstance(child, SpinAround))

        self.assertEqual(right_turn.update(), Status.RUNNING)
        self.assertLess(runtime.right_motor.power, 0)
        self.assertGreater(runtime.left_motor.power, 0)
        right_turn.terminate(Status.INVALID)

    def test_color_transition_requires_black_before_white(self) -> None:
        # 青円上の白では終了せず、黒0.5秒の後に白が0.5秒続いた場合だけ成功する。
        runtime.color_sensor.readings = [
            (0, 0, 100),
            (0, 0, 0),
            (0, 0, 0),
            (0, 0, 100),
            (0, 0, 100),
        ]
        behaviour = IsColorTransitionDetected(
            "black to white",
            from_color=Color.BLACK,
            to_color=Color.WHITE,
            from_duration_sec=0.5,
            to_duration_sec=0.5,
        )
        with patch(
            "robot_program.behaviours.conditions.time.monotonic",
            side_effect=[0.0, 0.1, 0.6, 0.7, 1.3],
        ):
            self.assertEqual(behaviour.update(), Status.RUNNING)
            self.assertEqual(behaviour.update(), Status.RUNNING)
            self.assertEqual(behaviour.update(), Status.RUNNING)
            self.assertEqual(behaviour.update(), Status.RUNNING)
            self.assertEqual(behaviour.update(), Status.SUCCESS)

    def test_sumo_sonar_sampling_uses_no_camera_and_records_median(self) -> None:
        # No.16は画像を参照せず、同じ角度の距離センサー中央値を保存する。
        context = RaceContext()
        settings = replace(
            SumoSettings(),
            sonar_samples_per_angle=3,
            sonar_max_attempts_per_angle=3,
            sonar_settle_time_sec=0.0,
            sonar_sample_interval_sec=0.0,
            sonar_mm_per_unit=1.0,
        )
        runtime.sonar_sensor.readings = [300, 280, 320]
        original_video_target = runtime.video.target
        behaviour = SampleSonarAtAngle("sample", context, settings, 20.0)

        self.assertEqual(behaviour.update(), Status.RUNNING)
        self.assertEqual(behaviour.update(), Status.RUNNING)
        self.assertEqual(behaviour.update(), Status.SUCCESS)
        self.assertEqual(context.sumo.sonar_samples[0].angle_offset_deg, 20.0)
        self.assertEqual(context.sumo.sonar_samples[0].distance_mm, 300.0)
        self.assertEqual(runtime.video.target, original_video_target)

    def test_sumo_sonar_diagnostic_timing_and_single_retry_are_configured(self) -> None:
        # 診断段階では100ms間隔で測定し、未検出時の前進再探索は1回だけ構成する。
        settings = SumoSettings()
        self.assertEqual(settings.sonar_samples_per_angle, 3)
        self.assertEqual(settings.sonar_settle_time_sec, 0.10)
        self.assertEqual(settings.sonar_sample_interval_sec, 0.10)
        self.assertEqual(settings.sonar_mm_per_unit, 1.0)
        self.assertEqual(settings.retry_advance_distance_mm, 100.0)

        feature = build_locate_sumo_bottle(RaceContext(), RaceConfig())

        def descendants(node):
            result = [node]
            for child in getattr(node, "children", []):
                result.extend(descendants(child))
            return result

        nodes = descendants(feature)
        self.assertEqual(
            sum(node.name == "run closer before sumo sonar retry" for node in nodes),
            1,
        )
        self.assertEqual(
            sum(node.name == "first sumo sonar scan" for node in nodes),
            1,
        )
        self.assertEqual(
            sum(node.name == "second sumo sonar scan" for node in nodes),
            1,
        )

    def test_sumo_alignment_selects_nearest_sonar_direction(self) -> None:
        # 全角度のうち最短距離方向を選び、保存した探索中心からの絶対方位を設定する。
        context = RaceContext()
        context.sumo.search_heading_deg = 88.0
        context.sumo.sonar_samples = [
            SumoSonarSample(30.0, 500.0),
            SumoSonarSample(-10.0, 240.0),
            SumoSonarSample(-40.0, 410.0),
        ]
        spin = SpinAround(
            "align",
            0,
            57,
            47,
            0.4,
            0.001,
            0.03,
            HeadingType.ABSOLUTE,
        )
        behaviour = SelectNearestBottleAndConfigureAlignment(
            "select", context, spin, -60.0
        )

        self.assertEqual(behaviour.update(), Status.SUCCESS)
        self.assertEqual(context.sumo.bottle_bearing_deg, -10.0)
        self.assertEqual(context.sumo.bottle_distance_mm, 240.0)
        self.assertEqual(spin.target, 78.0)

    def test_sumo_capture_approaches_without_turning_while_near_bottle(self) -> None:
        # No.17は正対済みのボトルへ直進し、捕捉直前にその場旋回しない。
        context = RaceContext()
        feature = build_push_sumo_bottle(context, RaceConfig())
        execute = feature.children[1]
        spins = [child for child in execute.children if isinstance(child, SpinAround)]
        self.assertEqual(spins, [])
        self.assertTrue(any(child.name == "approach sumo bottle" for child in execute.children))

    def test_sumo_carry_curve_is_left_on_left_and_right_on_right_course(self) -> None:
        # Leftは左カーブ、Rightは前進を維持した右カーブになることを確認する。
        settings = SumoSettings()
        left_command = RunAsInstructed("left curve", 0, 0)
        left_configure = ConfigureMirroredCarryCurvePwm(
            "configure left curve", left_command, settings
        )

        self.assertEqual(left_configure.update(), Status.SUCCESS)
        self.assertEqual(left_command.update(), Status.RUNNING)
        self.assertEqual(runtime.left_motor.power, settings.carry_curve_left_pwm)
        self.assertEqual(runtime.right_motor.power, settings.carry_curve_right_pwm)
        self.assertGreater(runtime.right_motor.power, runtime.left_motor.power)
        left_command.terminate(Status.INVALID)

        right_command = RunAsInstructed("right curve", 0, 0)
        right_configure = ConfigureMirroredCarryCurvePwm(
            "configure right curve", right_command, settings
        )
        runtime.course = -1

        self.assertEqual(right_configure.update(), Status.SUCCESS)
        self.assertEqual(right_command.update(), Status.RUNNING)
        self.assertEqual(runtime.left_motor.power, settings.carry_curve_right_pwm)
        self.assertEqual(runtime.right_motor.power, settings.carry_curve_left_pwm)
        self.assertGreater(runtime.left_motor.power, runtime.right_motor.power)

    def test_sumo_exit_carries_bottle_by_curve_then_black_line(self) -> None:
        # No.18は保持中のその場旋回を使わず、円弧後に出口側黒ラインまで運ぶ。
        context = RaceContext()
        context.sumo.bottle_captured = True
        feature = build_move_to_sumo_exit(context, RaceConfig())
        transport = feature.children[1]
        self.assertEqual(RaceConfig().sumo.carry_curve_distance_mm, 120.0)
        self.assertFalse(any(isinstance(child, SpinAround) for child in transport.children))
        self.assertTrue(
            any(
                child.name == "drive onto course-side black line"
                for child in transport.children
            )
        )

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

    def test_et_sumo_phase_builds_four_feature_subtrees(self) -> None:
        phase = build_et_sumo_phase(RaceContext(), RaceConfig())
        self.assertEqual(phase.name, "et_sumo")
        self.assertEqual(len(phase.children), 4)

if __name__ == "__main__":
    unittest.main()
