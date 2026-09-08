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


class ContinuousSonarSweep(SpinAround):
    # 既存SpinAroundで旋回しながら、各tickの距離と実方位を対応付ける。
    # 精密停止ではなく所定角度の通過で完了し、高い最低PWMによる往復振動を防ぐ。
    def __init__(self, name, context, settings):
        super().__init__(
            name=name,
            target=0,
            min_power=settings.continuous_scan_power,
            max_power=settings.continuous_scan_power,
            pid_p=settings.turn_pid_p,
            pid_i=settings.turn_pid_i,
            pid_d=settings.turn_pid_d,
            target_type=HeadingType.ABSOLUTE,
            tolerance=settings.heading_tolerance_deg,
        )
        self.context = context
        self.settings = settings
        self.started_at = None
        self.next_sample_at = None
        self.last_heading = None
        self.swept_angle_deg = 0.0
        self.sample_count = 0

    @staticmethod
    def _heading_delta(current, previous):
        # 0/360度境界を越えても、1tick分の符号付き角度差を-180～180度で得る。
        return (current - previous + 180.0) % 360.0 - 180.0

    def update(self):
        runtime.require(
            "plotter", "gyro_sensor", "sonar_sensor", "right_motor", "left_motor"
        )
        now = time.monotonic()
        heading = (-runtime.course * runtime.gyro_sensor.get_angle()) % 360.0
        if self.started_at is None:
            self.started_at = now
            self.next_sample_at = now
            self.last_heading = heading
            self.target = (
                self.context.sumo.search_heading_deg
                - self.settings.scan_half_angle_deg
            ) % 360.0
            self.logger.info(
                "continuous scan start center=%.1f outer=%.1f inner=%.1f range=%.1f"
                % (
                    self.context.sumo.search_heading_deg,
                    heading,
                    self.target,
                    2.0 * self.settings.scan_half_angle_deg,
                )
            )
        else:
            delta = self._heading_delta(heading, self.last_heading)
            # 正規化方位では外側端から内側端へ負方向に走査する。
            # 逆方向へ戻った量も差し引くため、往復だけで完了扱いにはならない。
            self.swept_angle_deg -= delta
            self.last_heading = heading

        if now - self.started_at >= self.settings.continuous_scan_timeout_sec:
            self.terminate(Status.FAILURE)
            StopNow(name="stop timed out continuous scan").update()
            raise RuntimeError(
                "Continuous sumo scan timed out; motors stopped "
                "(heading=%.1f swept=%.1f target=%.1f samples=%d candidates=%d)"
                % (
                    heading,
                    self.swept_angle_deg,
                    self.target,
                    self.sample_count,
                    len(self.context.sumo.sonar_samples),
                )
            )

        if now >= self.next_sample_at:
            self.next_sample_at = now + self.settings.continuous_sample_interval_sec
            raw_distance = runtime.sonar_sensor.get_distance()
            self.sample_count += 1
            distance_mm = (
                None
                if raw_distance is None or raw_distance <= 0
                else float(raw_distance) * self.settings.sonar_mm_per_unit
            )
            valid = (
                distance_mm is not None
                and self.settings.sonar_min_distance_mm
                <= distance_mm
                <= self.settings.sonar_max_distance_mm
            )
            if valid:
                offset = (
                    heading - self.context.sumo.search_heading_deg + 180.0
                ) % 360.0 - 180.0
                self.context.sumo.sonar_samples.append(
                    SumoSonarSample(offset, distance_mm)
                )
            log = self.logger.info if valid else self.logger.debug
            log(
                "continuous sonar elapsed_ms=%.1f heading=%.1f swept=%.1f "
                "raw=%s valid=%s"
                % (
                    (now - self.started_at) * 1000.0,
                    heading,
                    self.swept_angle_deg,
                    raw_distance,
                    valid,
                )
            )

        required_sweep = 2.0 * self.settings.scan_half_angle_deg
        # 目標近傍を1tickで通過しても、走査角度で確実に完了する。
        if self.swept_angle_deg >= required_sweep:
            self.logger.info(
                "continuous scan complete reason=angle-crossed heading=%.1f "
                "swept=%.1f samples=%d candidates=%d elapsed_ms=%.1f"
                % (
                    heading,
                    self.swept_angle_deg,
                    self.sample_count,
                    len(self.context.sumo.sonar_samples),
                    (now - self.started_at) * 1000.0,
                )
            )
            return Status.SUCCESS

        status = super().update()
        if status == Status.SUCCESS:
            self.logger.info(
                "continuous scan complete reason=target-tolerance heading=%.1f "
                "swept=%.1f samples=%d candidates=%d elapsed_ms=%.1f"
                % (
                    heading,
                    self.swept_angle_deg,
                    self.sample_count,
                    len(self.context.sumo.sonar_samples),
                    (now - self.started_at) * 1000.0,
                )
            )
        return status


class ConfirmSonarCandidate(SampleSonarAtAngle):
    # 走査中の候補へ正対後、静止した状態で3有効値を再確認する。
    def __init__(self, name, context, settings):
        super().__init__(name, context, settings, 0.0)

    def update(self):
        state = self.context.sumo
        if state.skipped or state.bottle_distance_mm is None:
            return Status.SUCCESS
        if self.started_at is None:
            heading = (-runtime.course * runtime.gyro_sensor.get_angle()) % 360.0
            self.angle_offset_deg = (
                heading - state.search_heading_deg + 180.0
            ) % 360.0 - 180.0
        status = super().update()
        if status == Status.SUCCESS:
            if len(self.valid_distances) >= self.settings.sonar_samples_per_angle:
                state.bottle_distance_mm = statistics.median(self.valid_distances)
                state.bottle_bearing_deg = self.angle_offset_deg
                self.logger.info(
                    "stationary sonar candidate confirmed distance=%.1fmm"
                    % state.bottle_distance_mm
                )
            else:
                state.skipped = True
                state.bottle_distance_mm = None
                state.bottle_bearing_deg = None
                state.failure_reason = "stationary sonar confirmation failed"
                self.logger.warning(state.failure_reason)
        return status


class ConfigureConfirmationReturn(Behaviour):
    # 静止確認に失敗した場合だけ探索中央へ戻し、次の接近方向を揃える。
    def __init__(self, name, context, spin):
        super().__init__(name)
        self.context = context
        self.spin = spin

    def update(self):
        self.spin.target = (
            self.context.sumo.search_heading_deg
            if self.context.sumo.skipped
            else (-runtime.course * runtime.gyro_sensor.get_angle()) % 360.0
        )
        return Status.SUCCESS


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
    # 1回分の外側端から内側端までの探索を、再試行でも再利用できる形で構成する。
    root = Sequence(name=name, memory=True)
    root.add_child(
        _spin("%s turn to course outer edge" % name, offsets[0], settings)
    )
    if settings.continuous_scan_enabled:
        root.add_children(
            [
                StopNow(name="%s stop before continuous sweep" % name),
                ContinuousSonarSweep(
                    "%s continuous sweep" % name, context, settings
                ),
                StopNow(name="%s stop after continuous sweep" % name),
            ]
        )
        return root

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


def _confirmation_nodes(name, context, settings):
    # 従来の停止探索では各角度ですでに3回測定しているため、追加確認しない。
    if not settings.continuous_scan_enabled:
        return []
    recovery = _spin(
        "%s return after confirmation" % name,
        0,
        settings,
        target_type=HeadingType.ABSOLUTE,
    )
    return [
        ConfirmSonarCandidate(
            "%s stationary confirmation" % name, context, settings
        ),
        ConfigureConfirmationReturn(
            "%s configure confirmation return" % name, context, recovery
        ),
        recovery,
        StopNow(name="%s stop after confirmation" % name),
    ]


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
    retry_distances = tuple(settings.retry_advance_distances_mm)
    if not retry_distances or any(distance <= 0 for distance in retry_distances):
        raise ValueError("sumo retry advance distances must be positive")
    retry_names = ("second", "third", "fourth")
    if len(retry_distances) > len(retry_names):
        raise ValueError("at most three sumo sonar retries are supported")

    finish_or_retry = Selector(
        name="accept sumo scan or retry closer", memory=True
    )
    finish_or_retry.add_child(
        HasBottleCandidate("accept first sumo sonar candidate", context)
    )

    for retry_index, retry_distance in enumerate(retry_distances):
        ordinal = retry_names[retry_index]
        retry_number = retry_index + 1
        retry_alignment = _spin(
            "align %s sumo sonar scan result" % ordinal,
            0,
            settings,
            target_type=HeadingType.ABSOLUTE,
        )
        retry_advance = Parallel(
            name="advance before %s sumo sonar scan" % ordinal,
            policy=ParallelPolicy.SuccessOnOne(),
        )
        retry_advance.add_children(
            [
                RunByGyro(
                    name="run closer before sumo sonar retry %d" % retry_number,
                    target=0,
                    power=settings.retry_advance_power,
                    pid_p=settings.drive_pid_p,
                    pid_i=settings.drive_pid_i,
                    pid_d=settings.drive_pid_d,
                    target_type=HeadingType.RELATIVE,
                ),
                IsDistanceEarned(
                    name="sumo sonar retry %d advance distance" % retry_number,
                    delta_dist=retry_distance,
                ),
            ]
        )
        retry = Sequence(
            name="%s sumo sonar scan after advance" % ordinal, memory=True
        )
        retry_children = [
            retry_advance,
            StopNow(
                name="stop at closer sumo search position %d" % retry_number
            ),
            PrepareSonarRetry(
                "prepare %s sumo sonar scan" % ordinal, context
            ),
            _build_scan_pass(
                "%s sumo sonar scan" % ordinal, context, settings, offsets
            ),
            SelectNearestBottleAndConfigureAlignment(
                "select %s sumo sonar result" % ordinal,
                context,
                retry_alignment,
                offsets[-1],
            ),
            retry_alignment,
            StopNow(name="stop after %s sumo sonar alignment" % ordinal),
            *_confirmation_nodes(ordinal, context, settings),
        ]
        # 最終再探索以外は未検出時に次のSelector枝へ進む。
        if retry_index < len(retry_distances) - 1:
            retry_children.append(
                HasBottleCandidate(
                    "accept %s sumo sonar candidate" % ordinal, context
                )
            )
        retry.add_children(retry_children)
        finish_or_retry.add_child(retry)

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
            *_confirmation_nodes("first", context, settings),
            finish_or_retry,
        ]
    )
    return root
