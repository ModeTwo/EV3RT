"""Feature 16: locate the sumo bottle with the distance sensor."""

import statistics

from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time

from ..behaviours.conditions import IsDistanceEarned
from ..behaviours.gyro_drive import RunByGyro, SpinAround
from ..behaviours.motor_control import StopNow
from ..sumo_types import SumoSonarSample


class SampleSonarAtAngle(Behaviour):
    # 停止角度ごとに距離センサーを複数回読み、中央値を探索結果へ保存する。
    def __init__(self, name, context, settings, angle_offset_deg):
        super().__init__(name)
        self.context = context
        self.settings = settings
        self.angle_offset_deg = float(angle_offset_deg)
        self.attempts = 0
        self.valid_distances = []
        self.started_at = None
        self.next_sample_at = None

    def update(self):
        runtime.require("plotter", "gyro_sensor", "sonar_sensor")
        now = time.monotonic()
        if self.started_at is None:
            self.started_at = now
            self.next_sample_at = now + self.settings.sonar_settle_time_sec
            self.logger.info(
                "%+06d %s.waiting %.0fms before sonar sampling"
                % (
                    runtime.plotter.get_distance(),
                    self.__class__.__name__,
                    self.settings.sonar_settle_time_sec * 1000.0,
                )
            )

        # BT自体は20ms周期でも、距離センサーへの問い合わせは設定周期まで間引く。
        if now < self.next_sample_at:
            return Status.RUNNING

        self.next_sample_at = now + self.settings.sonar_sample_interval_sec
        self.attempts += 1
        current_heading = (
            -runtime.course * runtime.gyro_sensor.get_angle()
        ) % 360.0
        raw_distance = runtime.sonar_sensor.get_distance()
        distance_mm = None
        result = "invalid:none"
        if raw_distance is None:
            result = "invalid:none"
        elif raw_distance <= 0:
            result = "invalid:non-positive"
        else:
            distance_mm = float(raw_distance) * self.settings.sonar_mm_per_unit
            if distance_mm < self.settings.sonar_min_distance_mm:
                result = "invalid:below-min"
            elif distance_mm > self.settings.sonar_max_distance_mm:
                result = "invalid:above-max"
            else:
                self.valid_distances.append(distance_mm)
                result = "valid"

        # 実機ログだけで、測定方位、生値、換算結果、破棄理由、有効値の蓄積状況を追跡可能にする。
        self.logger.info(
            "%+06d %s.sonar offset=%+.1f heading=%.1f raw=%s mm=%s "
            "result=%s valid=%d/%d attempt=%d/%d"
            % (
                runtime.plotter.get_distance(),
                self.__class__.__name__,
                self.angle_offset_deg,
                current_heading,
                raw_distance,
                "None" if distance_mm is None else "%.1f" % distance_mm,
                result,
                len(self.valid_distances),
                self.settings.sonar_samples_per_angle,
                self.attempts,
                self.settings.sonar_max_attempts_per_angle,
            )
        )

        # 規定数の有効値が集まった時点で、外れ値の影響を抑えるため中央値を採用する。
        if len(self.valid_distances) >= self.settings.sonar_samples_per_angle:
            median_distance = statistics.median(self.valid_distances)
            self.context.sumo.sonar_samples.append(
                SumoSonarSample(
                    angle_offset_deg=self.angle_offset_deg,
                    distance_mm=median_distance,
                )
            )
            self.logger.info(
                "%+06d %s.accepted sonar offset=%+.1f median=%.1fmm samples=%d"
                % (
                    runtime.plotter.get_distance(),
                    self.__class__.__name__,
                    self.angle_offset_deg,
                    median_distance,
                    len(self.valid_distances),
                )
            )
            return Status.SUCCESS

        # 無効値しか返らない角度でも探索全体を止めず、次の首振り角度へ進む。
        if self.attempts >= self.settings.sonar_max_attempts_per_angle:
            self.logger.warning(
                "%+06d %s.no accepted sonar offset=%+.1f valid=%d attempts=%d"
                % (
                    runtime.plotter.get_distance(),
                    self.__class__.__name__,
                    self.angle_offset_deg,
                    len(self.valid_distances),
                    self.attempts,
                )
            )
            return Status.SUCCESS
        return Status.RUNNING


class SelectNearestBottleAndConfigureAlignment(Behaviour):
    # 有効な測定のうち最短距離をボトル候補とし、その角度へ戻る旋回量を設定する。
    def __init__(self, name, context, alignment_spin, last_scan_offset_deg):
        super().__init__(name)
        self.context = context
        self.alignment_spin = alignment_spin
        self.last_scan_offset_deg = float(last_scan_offset_deg)

    def update(self):
        runtime.require("plotter", "gyro_sensor")
        state = self.context.sumo
        current_heading = (
            -runtime.course * runtime.gyro_sensor.get_angle()
        ) % 360.0
        if not state.sonar_samples:
            state.skipped = True
            state.failure_reason = "no valid sonar return during sumo scan"
            state.bottle_bearing_deg = None
            state.bottle_distance_mm = None
            # 相対角度の累積誤差を使わず、保存済みの探索中心へ絶対方位で戻す。
            self.alignment_spin.target = state.search_heading_deg
            self.logger.warning(
                "%+06d %s.no bottle candidate reference=%.1f current=%.1f "
                "accepted_angles=0 fallback_absolute=%.1f"
                % (
                    runtime.plotter.get_distance(),
                    self.__class__.__name__,
                    state.search_heading_deg,
                    current_heading,
                    self.alignment_spin.target,
                )
            )
            return Status.SUCCESS

        nearest = min(state.sonar_samples, key=lambda sample: sample.distance_mm)
        state.bottle_bearing_deg = nearest.angle_offset_deg
        state.bottle_distance_mm = nearest.distance_mm
        state.skipped = False
        state.failure_reason = None
        self.alignment_spin.target = (
            state.search_heading_deg + nearest.angle_offset_deg
        ) % 360.0
        self.logger.info(
            "%+06d %s.selected bottle offset=%+.1f distance=%.1fmm "
            "reference=%.1f current=%.1f accepted_angles=%d align_absolute=%.1f"
            % (
                runtime.plotter.get_distance(),
                self.__class__.__name__,
                nearest.angle_offset_deg,
                nearest.distance_mm,
                state.search_heading_deg,
                current_heading,
                len(state.sonar_samples),
                self.alignment_spin.target,
            )
        )
        return Status.SUCCESS


class HasBottleCandidate(Behaviour):
    # 1回目の探索で候補を得た場合は、前進再探索を行わずFeatureを完了する。
    def __init__(self, name, context):
        super().__init__(name)
        self.context = context

    def update(self):
        state = self.context.sumo
        if not state.skipped and state.bottle_distance_mm is not None:
            return Status.SUCCESS
        return Status.FAILURE


class PrepareSonarRetry(Behaviour):
    # 前進後の実方位を2回目の探索中心として保存し、1回目の探索結果を消去する。
    def __init__(self, name, context):
        super().__init__(name)
        self.context = context

    def update(self):
        runtime.require("plotter", "gyro_sensor")
        state = self.context.sumo
        state.search_heading_deg = (
            -runtime.course * runtime.gyro_sensor.get_angle()
        ) % 360.0
        state.sonar_samples.clear()
        state.bottle_bearing_deg = None
        state.bottle_distance_mm = None
        state.skipped = False
        state.failure_reason = None
        self.logger.info(
            "%+06d %s.retry search center heading=%.1f"
            % (
                runtime.plotter.get_distance(),
                self.__class__.__name__,
                state.search_heading_deg,
            )
        )
        return Status.SUCCESS


def _scan_offsets(settings):
    # コース外側から内側まで一定角度で走査し、端数があっても内側端を必ず測定する。
    half = float(settings.scan_half_angle_deg)
    step = float(settings.scan_step_deg)
    if half <= 0 or step <= 0:
        raise ValueError("sumo scan angles must be positive")
    offsets = [half]
    current = half
    while current - step > -half:
        current -= step
        offsets.append(current)
    if offsets[-1] != -half:
        offsets.append(-half)
    return offsets


def _spin(
    name,
    target,
    settings,
    min_power=None,
    max_power=None,
    target_type=HeadingType.RELATIVE,
):
    # 探索の短い段階旋回だけ出力を上書きできるようにし、大旋回と正対旋回への影響を防ぐ。
    effective_min_power = settings.turn_min_power if min_power is None else min_power
    effective_max_power = settings.turn_max_power if max_power is None else max_power
    return SpinAround(
        name=name,
        target=target,
        max_power=effective_max_power,
        min_power=effective_min_power,
        pid_p=settings.turn_pid_p,
        pid_i=settings.turn_pid_i,
        pid_d=settings.turn_pid_d,
        target_type=target_type,
        tolerance=settings.heading_tolerance_deg,
    )


def _build_scan_pass(name, context, settings, offsets):
    # 1回分の外側端から内側端までの段階探索を、再試行でも再利用できる形で構成する。
    root = Sequence(name=name, memory=True)
    root.add_child(
        _spin("%s turn to course outer edge" % name, offsets[0], settings)
    )
    previous_offset = offsets[0]
    for index, offset in enumerate(offsets):
        if index > 0:
            root.add_child(
                _spin(
                    "%s swing step %d" % (name, index),
                    offset - previous_offset,
                    settings,
                    min_power=settings.scan_step_turn_min_power,
                    max_power=settings.scan_step_turn_max_power,
                )
            )
        # 実行単位：旋回を止めてから距離を複数回取得し、角度と距離を対応付ける。
        root.add_child(StopNow(name="%s stop step %d" % (name, index)))
        root.add_child(
            SampleSonarAtAngle(
                "%s sample step %d" % (name, index),
                context,
                settings,
                offset,
            )
        )
        previous_offset = offset
    return root


def build_locate_sumo_bottle(context, config):
    # No.16：画像を使わず、走行体の左右首振りと距離センサーだけでボトルへ正対する。
    settings = config.sumo
    offsets = _scan_offsets(settings)

    first_alignment = _spin(
        "align first sumo scan result",
        0,
        settings,
        target_type=HeadingType.ABSOLUTE,
    )
    retry_alignment = _spin(
        "align retry sumo scan result",
        0,
        settings,
        target_type=HeadingType.ABSOLUTE,
    )

    retry_advance = Parallel(
        name="advance before sumo sonar retry",
        policy=ParallelPolicy.SuccessOnOne(),
    )
    retry_advance.add_children(
        [
            RunByGyro(
                name="run closer before sumo sonar retry",
                target=0,
                power=settings.retry_advance_power,
                pid_p=settings.drive_pid_p,
                pid_i=settings.drive_pid_i,
                pid_d=settings.drive_pid_d,
                target_type=HeadingType.RELATIVE,
            ),
            IsDistanceEarned(
                name="sumo sonar retry advance distance",
                delta_dist=settings.retry_advance_distance_mm,
            ),
        ]
    )

    retry = Sequence(name="retry sumo sonar after advance", memory=True)
    retry.add_children(
        [
            # 1回目の未検出時は絶対方位で探索中心へ復帰済みなので、低速で規定距離だけ近づく。
            retry_advance,
            StopNow(name="stop at closer sumo search position"),
            PrepareSonarRetry("prepare second sumo sonar scan", context),
            _build_scan_pass("second sumo sonar scan", context, settings, offsets),
            SelectNearestBottleAndConfigureAlignment(
                "select second sumo sonar result",
                context,
                retry_alignment,
                offsets[-1],
            ),
            retry_alignment,
            StopNow(name="stop after second sumo sonar alignment"),
        ]
    )

    finish_or_retry = Selector(name="accept first sumo scan or retry", memory=True)
    finish_or_retry.add_children(
        [
            HasBottleCandidate("accept first sumo sonar candidate", context),
            retry,
        ]
    )

    root = Sequence(name="locate_sumo_bottle", memory=True)
    root.add_children(
        [
            _build_scan_pass("first sumo sonar scan", context, settings, offsets),
            SelectNearestBottleAndConfigureAlignment(
                "select first sumo sonar result",
                context,
                first_alignment,
                offsets[-1],
            ),
            first_alignment,
            StopNow(name="stop after first sumo sonar alignment"),
            finish_or_retry,
        ]
    )
    return root
