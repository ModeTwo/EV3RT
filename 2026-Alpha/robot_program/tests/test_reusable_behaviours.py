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
    build_locate_sumo_bottle,
    ConfirmSonarCandidate,
    ContinuousSonarSweep,
    SampleSonarAtAngle,
    SelectNearestBottleAndConfigureAlignment,
)
from robot_program.features.capture_sumo_bottle_camera import (
    CaptureSumoBottleWithCamera,
    build_capture_sumo_bottle_camera,
)
from robot_program.features.move_to_sumo_exit import (
    ConfigureCourseIndependentReversePwm,
    ConfigureMirroredGarageReturnPwm,
    MarkSumoExitState,
    build_move_to_sumo_exit,
)
from robot_program.features.move_to_sumo_start import (
    ConfigureCameraRetreatPwm,
    IsBlackThenBrightSurface,
    build_move_to_sumo_start,
)
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
        self.bottle_session = 1
        self.bottle_frame_id = 0

    def set_target_interested(self, target) -> None:
        self.target = target

    def set_thresholds(self, gs_min, gs_max) -> None:
        self.thresholds = (gs_min, gs_max)

    def begin_qr_read(self):
        self.target = TargetInterested.QRCODE
        return 1

    def get_qr_observation(self):
        return (1, 1, self.qr_text)

    def get_QR_text(self) -> str:
        return self.qr_text

    def get_bottle_stamped(self):
        return self.bottle_snapshot

    def set_bottle_color(self, color) -> None:
        self.bottle_color = color

    def begin_sumo_bottle_read(self):
        self.bottle_session += 1
        self.target = TargetInterested.BOTTLE
        self.bottle_color = BottleColor.BLACK
        return self.bottle_session

    def get_bottle_observation(self):
        return self.bottle_session, self.bottle_frame_id, self.bottle_snapshot


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

    def test_reset_device_allows_camera_free_sumo(self) -> None:
        runtime.video = None
        behaviour = ResetDevice(name="reset without camera", stationary_samples=1)
        self.assertEqual(behaviour.update(), Status.SUCCESS)
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
        # No.15は黒を1回、白を連続判定し、クリアランス直進後に90度旋回する。
        feature = build_move_to_sumo_start(RaceContext(), RaceConfig())
        self.assertEqual(feature.children[0].name, "drive across black line to white area")
        transition = next(
            child for child in feature.children[0].children
            if isinstance(child, IsBlackThenBrightSurface)
        )
        self.assertEqual(transition.settings.line_black_max_value, 45)
        self.assertEqual(transition.settings.line_white_min_value, 65)
        self.assertEqual(transition.settings.line_exit_white_duration_sec, 0.5)
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
        self.assertEqual(feature.children[7].name, "reverse to widen sumo camera view")
        retreat_distance = next(
            child for child in feature.children[7].children
            if isinstance(child, IsDistanceEarned)
        )
        self.assertEqual(retreat_distance.delta_dist, 50.0)
        self.assertIsInstance(feature.children[8], StopNow)

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

    def test_sumo_camera_retreat_is_backward_on_both_courses(self) -> None:
        # RunAsInstructedのcourse変換後も、左右コースとも両輪が負出力になる。
        for course in (1, -1):
            runtime.course = course
            command = RunAsInstructed("camera retreat", 0, 0)
            configure = ConfigureCameraRetreatPwm(
                "configure retreat", command, SumoSettings().camera_retreat_power
            )
            self.assertEqual(configure.update(), Status.SUCCESS)
            self.assertEqual(command.update(), Status.RUNNING)
            self.assertEqual(runtime.left_motor.power, -60)
            self.assertEqual(runtime.right_motor.power, -60)
            command.terminate(Status.INVALID)

    def test_color_transition_requires_black_before_white(self) -> None:
        # 青円上の白では終了せず、黒を1回確認した後に白が0.5秒続けば成功する。
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
            from_duration_sec=0.0,
            to_duration_sec=0.5,
        )
        with patch(
            "robot_program.behaviours.conditions.time.monotonic",
            side_effect=[0.0, 0.1, 0.2, 0.3, 0.9],
        ):
            self.assertEqual(behaviour.update(), Status.RUNNING)
            self.assertEqual(behaviour.update(), Status.RUNNING)
            self.assertEqual(behaviour.update(), Status.RUNNING)
            self.assertEqual(behaviour.update(), Status.RUNNING)
            self.assertEqual(behaviour.update(), Status.SUCCESS)

    def test_sumo_line_exit_uses_raw_brightness_after_black(self) -> None:
        # 彩度によるWHITE分類に依存せず、黒確認後の明度継続で白地退出を判定する。
        runtime.color_sensor.readings = [
            (0, 80, 80),
            (0, 70, 40),
            (0, 90, 66),
            (0, 90, 68),
        ]
        behaviour = IsBlackThenBrightSurface("sumo line exit", SumoSettings())
        with patch(
            "robot_program.features.move_to_sumo_start.time.monotonic",
            side_effect=[0.0, 0.1, 0.2, 0.8],
        ):
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

    def test_sumo_continuous_scan_and_three_retries_are_configured(self) -> None:
        # 連続走査後に静止確認し、未検出時は3段階で接近する。
        settings = SumoSettings()
        self.assertEqual(settings.sonar_samples_per_angle, 3)
        self.assertEqual(settings.sonar_settle_time_sec, 0.10)
        self.assertEqual(settings.sonar_sample_interval_sec, 0.10)
        self.assertEqual(settings.sonar_mm_per_unit, 1.0)
        self.assertEqual(settings.continuous_sample_interval_sec, 0.02)
        self.assertEqual(settings.continuous_scan_power, 50)
        self.assertEqual(settings.retry_advance_distances_mm, (100.0, 70.0, 50.0))

        feature = build_locate_sumo_bottle(RaceContext(), RaceConfig())

        def descendants(node):
            result = [node]
            for child in getattr(node, "children", []):
                result.extend(descendants(child))
            return result

        nodes = descendants(feature)
        self.assertEqual(
            sum(node.name.startswith("run closer before sumo sonar retry") for node in nodes),
            3,
        )
        self.assertEqual(
            sum(node.name == "first sumo sonar scan" for node in nodes),
            1,
        )
        self.assertEqual(
            sum(isinstance(node, ContinuousSonarSweep) for node in nodes),
            4,
        )

    def test_continuous_scan_completes_when_inner_boundary_is_crossed(self) -> None:
        # PWM50で目標角を飛び越しても、±3度への収束を待たず120度走査で完了する。
        for course in (1, -1):
            context = RaceContext()
            context.sumo.search_heading_deg = 90.0
            runtime.course = course
            runtime.gyro_sensor.angle = -course * 150
            node = ContinuousSonarSweep("continuous", context, SumoSettings())
            with patch("robot_program.features.locate_sumo_bottle.time.monotonic", return_value=1.0):
                node.tick_once()
            self.assertEqual(node.status, Status.RUNNING)
            runtime.gyro_sensor.angle = -course * 28
            with patch("robot_program.features.locate_sumo_bottle.time.monotonic", return_value=1.1):
                node.tick_once()
            self.assertEqual(node.status, Status.SUCCESS)
            self.assertGreaterEqual(node.swept_angle_deg, 120.0)
            self.assertEqual(runtime.left_motor.power, 0)
            self.assertEqual(runtime.right_motor.power, 0)

    def test_continuous_candidate_is_confirmed_while_stopped(self) -> None:
        context = RaceContext()
        context.sumo.bottle_distance_mm = 108.0
        context.sumo.skipped = False
        runtime.sonar_sensor.readings = [108, 110, 109]
        settings = replace(
            SumoSettings(), sonar_settle_time_sec=0, sonar_sample_interval_sec=0
        )
        node = ConfirmSonarCandidate("confirm", context, settings)
        self.assertEqual(node.update(), Status.RUNNING)
        self.assertEqual(node.update(), Status.RUNNING)
        self.assertEqual(node.update(), Status.SUCCESS)
        self.assertEqual(context.sumo.bottle_distance_mm, 109.0)

    def test_et_sumo_motor_settings_are_at_least_fifty(self) -> None:
        settings = SumoSettings()
        values = (
            settings.navigation_power, settings.approach_power,
            settings.carry_power, settings.continuous_scan_power,
            settings.scan_step_turn_min_power, settings.scan_step_turn_max_power,
            settings.retry_advance_power, settings.push_out_drive_power,
            settings.release_reverse_power, settings.garage_return_reverse_left_pwm,
            settings.garage_return_reverse_right_pwm, settings.line_rejoin_trace_power,
            settings.turn_min_power,
            settings.turn_max_power, settings.camera_retreat_power,
            settings.camera_approach_power, settings.camera_min_wheel_power,
            settings.camera_max_wheel_power,
        )
        self.assertTrue(all(value >= 50 for value in values))

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

    def test_sumo_garage_return_curve_reverses_and_mirrors_by_course(self) -> None:
        # 両コースで後退し、ガレージ側へ寄せる左右差だけが鏡像になることを確認する。
        settings = SumoSettings()
        left_command = RunAsInstructed("left garage return", 0, 0)
        left_configure = ConfigureMirroredGarageReturnPwm(
            "configure left garage return", left_command, settings
        )

        self.assertEqual(left_configure.update(), Status.SUCCESS)
        self.assertEqual(left_command.update(), Status.RUNNING)
        self.assertEqual(runtime.left_motor.power, -settings.garage_return_reverse_left_pwm)
        self.assertEqual(runtime.right_motor.power, -settings.garage_return_reverse_right_pwm)
        left_command.terminate(Status.INVALID)

        right_command = RunAsInstructed("right garage return", 0, 0)
        right_configure = ConfigureMirroredGarageReturnPwm(
            "configure right garage return", right_command, settings
        )
        runtime.course = -1

        self.assertEqual(right_configure.update(), Status.SUCCESS)
        self.assertEqual(right_command.update(), Status.RUNNING)
        self.assertEqual(runtime.left_motor.power, -settings.garage_return_reverse_right_pwm)
        self.assertEqual(runtime.right_motor.power, -settings.garage_return_reverse_left_pwm)

    def test_sumo_release_reverse_runs_backward_on_both_courses(self) -> None:
        # RunAsInstructedのcourse補正後も、Left／Rightの両方で左右輪が後退する。
        settings = SumoSettings()
        for course in (1, -1):
            runtime.course = course
            command = RunAsInstructed("release reverse", 0, 0)
            configure = ConfigureCourseIndependentReversePwm(
                "configure release reverse", command, settings.release_reverse_power
            )
            self.assertEqual(configure.update(), Status.SUCCESS)
            self.assertEqual(command.update(), Status.RUNNING)
            self.assertEqual(runtime.left_motor.power, -settings.release_reverse_power)
            self.assertEqual(runtime.right_motor.power, -settings.release_reverse_power)
            command.terminate(Status.INVALID)

    def test_sumo_exit_pushes_releases_and_rejoins_line(self) -> None:
        # No.18は黒ライン越え、直線後退、ガレージ側復帰、ライントレースの順で構成する。
        context = RaceContext()
        context.sumo.bottle_captured = True
        feature = build_move_to_sumo_exit(context, RaceConfig())
        transport = feature.children[1]
        self.assertEqual(RaceConfig().sumo.push_out_after_line_distance_mm, 30.0)
        self.assertFalse(any(isinstance(child, SpinAround) for child in transport.children))
        expected_names = (
            "drive captured bottle to push-out black line",
            "push bottle 30 mm beyond black line",
            "reverse straight to release sumo bottle",
            "reverse curve to garage-side black line",
            "stabilize on garage-side black line",
        )
        child_names = [child.name for child in transport.children]
        self.assertTrue(all(name in child_names for name in expected_names))

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

    def test_sumo_camera_capture_uses_black_only_and_no_search_spin(self) -> None:
        # 新方式は距離センサー探索を実行木から外し、黒テープだけをカメラ追跡する。
        context = RaceContext()
        feature = build_capture_sumo_bottle_camera(context, RaceConfig())
        descendants = []

        def collect(node):
            descendants.append(node)
            for child in getattr(node, "children", []):
                collect(child)

        collect(feature)
        self.assertFalse(any(isinstance(node, SpinAround) for node in descendants))
        camera_node = next(
            node for node in descendants
            if isinstance(node, CaptureSumoBottleWithCamera)
        )

        camera_node.tick_once()
        self.assertEqual(runtime.video.target, TargetInterested.BOTTLE)
        self.assertEqual(runtime.video.bottle_color, BottleColor.BLACK)
        self.assertEqual(runtime.right_motor.power, 0)
        self.assertEqual(runtime.left_motor.power, 0)

    def test_sumo_camera_confirmation_counts_only_new_frames(self) -> None:
        # 制御周期が画像周期より速くても、同じ画像を3回検出として数えない。
        context = RaceContext()
        node = CaptureSumoBottleWithCamera(
            "camera capture", context, RaceConfig().sumo
        )
        runtime.video.bottle_snapshot = (
            True, BottleColor.BLACK, 220, 10.0, 80, 300, False
        )

        node.tick_once()
        self.assertEqual(node.confirmed_frames, 1)
        node.tick_once()
        self.assertEqual(node.confirmed_frames, 1)
        runtime.video.bottle_frame_id = 1
        node.tick_once()
        self.assertEqual(node.confirmed_frames, 2)
        runtime.video.bottle_frame_id = 2
        node.tick_once()
        self.assertEqual(node.phase, node.APPROACH)
        self.assertGreaterEqual(runtime.right_motor.power, 50)
        self.assertGreaterEqual(runtime.left_motor.power, 50)
        self.assertNotEqual(runtime.right_motor.power, runtime.left_motor.power)

    def test_sumo_exit_state_records_push_release_and_line_trace_ready(self) -> None:
        # 押し出し、離脱、ライン復帰の状態を後続工程から個別に確認できる。
        context = RaceContext()
        context.sumo.bottle_held_at_exit = True
        self.assertEqual(MarkSumoExitState("mark push", context, "pushed_out").update(), Status.SUCCESS)
        self.assertEqual(MarkSumoExitState("mark release", context, "released").update(), Status.SUCCESS)
        self.assertEqual(MarkSumoExitState("mark ready", context, "line_trace_ready").update(), Status.SUCCESS)
        self.assertTrue(context.sumo.bottle_pushed_out)
        self.assertTrue(context.sumo.bottle_released)
        self.assertTrue(context.sumo.transport_completed)
        self.assertFalse(context.sumo.bottle_held_at_exit)
        self.assertTrue(context.sumo.line_trace_ready)

    def test_et_sumo_phase_builds_three_feature_subtrees(self) -> None:
        phase = build_et_sumo_phase(RaceContext(), RaceConfig())
        self.assertEqual(phase.name, "et_sumo")
        self.assertEqual(len(phase.children), 3)
        self.assertEqual(
            [child.name for child in phase.children],
            ["move_to_sumo_start", "capture_sumo_bottle_camera", "move_to_sumo_exit"],
        )

if __name__ == "__main__":
    unittest.main()
