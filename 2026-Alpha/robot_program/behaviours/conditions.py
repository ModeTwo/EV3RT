"""Reusable condition behaviors."""

import time

from py_trees.behaviour import Behaviour
from py_trees.common import Status

from py_etrobo_util import Color, ColorClassifier

from ..runtime import runtime


class IsTimePassed(Behaviour):
    # Behavior開始後に指定時間が経過した場合に成功する。
    def __init__(self, name: str, delta_time: float):
        super().__init__(name)
        self.delta_time = delta_time
        self.running = False
        self.earned = False
        self.started_at = 0.0

    def update(self) -> Status:
        runtime.require("plotter")
        if not self.running:
            self.running = True
            self.started_at = time.monotonic()
            self.logger.info(
                "%+06d %s.accumulation started for delta=%.3f"
                % (
                    runtime.plotter.get_distance(),
                    self.__class__.__name__,
                    self.delta_time,
                )
            )

        if time.monotonic() - self.started_at >= self.delta_time:
            if not self.earned:
                self.earned = True
                self.logger.info(
                    "%+06d %s.delta time passed"
                    % (runtime.plotter.get_distance(), self.__class__.__name__)
                )
            return Status.SUCCESS
        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        # 同じインスタンスが再実行された場合に、新しい開始時刻から計測し直す。
        self.running = False
        self.earned = False


class IsDistanceEarned(Behaviour):
    # Behavior開始時から指定距離以上移動した場合に成功する。
    def __init__(self, name: str, delta_dist: int):
        super().__init__(name)
        self.delta_dist = delta_dist
        self.running = False
        self.earned = False
        self.orig_dist = 0

    def update(self) -> Status:
        runtime.require("plotter")
        if not self.running:
            self.running = True
            self.orig_dist = runtime.plotter.get_distance()
            self.logger.info(
                "%+06d %s.accumulation started for delta=%d"
                % (self.orig_dist, self.__class__.__name__, self.delta_dist)
            )

        cur_dist = runtime.plotter.get_distance()
        earned_dist = cur_dist - self.orig_dist
        if abs(earned_dist) >= self.delta_dist:
            if not self.earned:
                self.earned = True
                self.logger.info(
                    "%+06d %s.delta distance earned"
                    % (cur_dist, self.__class__.__name__)
                )
            return Status.SUCCESS
        return Status.RUNNING


class IsColorDetected(Behaviour):
    # カラーセンサーのHSV値を分類し、指定色を検知した場合に成功する。
    def __init__(self, name: str, color: Color):
        super().__init__(name)
        self.color = color
        self.prev_color = Color.UNKNOWN
        self.classifier = ColorClassifier()
        self.running = False
        self.detected = False

    def update(self) -> Status:
        runtime.require("plotter", "color_sensor")
        cur_dist = runtime.plotter.get_distance()
        if not self.running:
            self.running = True
            self.logger.info(
                "%+06d %s.detection started for color=%s"
                % (cur_dist, self.__class__.__name__, self.color.value)
            )

        h, s, v = runtime.color_sensor.get_raw_color_hsv()
        detected_color = self.classifier.classify(h, s, v)
        if detected_color == self.color:
            if not self.detected:
                self.detected = True
                self.logger.info(
                    "%+06d %s.color=%s detected"
                    % (cur_dist, self.__class__.__name__, self.color.value)
                )
            return Status.SUCCESS

        if detected_color != self.prev_color:
            # UNKNOWNが継続する場合はログを抑え、色変化だけを記録する。
            if detected_color != Color.UNKNOWN or self.prev_color != Color.UNKNOWN:
                self.logger.info(
                    "%+06d %s.color changed from %s to %s"
                    % (
                        cur_dist,
                        self.__class__.__name__,
                        self.prev_color.value,
                        detected_color.value,
                    )
                )
                self.prev_color = detected_color
        return Status.RUNNING

class IsBlackDetected(Behaviour):
    def __init__(
        self,
        name: str,
        black_threshold: float,
        required_frames: int = 5
    ):
        super(IsBlackDetected, self).__init__(name)

        self.black_threshold = black_threshold
        self.required_frames = required_frames

        self.detect_count = 0
        self.running = False

    def initialise(self):
        self.detect_count = 0
        self.running = False

    def update(self) -> Status:

        if not self.running:
            self.running = True
            self.logger.info(
                "%s.detection started threshold=%.1f frames=%d"
                % (
                    self.__class__.__name__,
                    self.black_threshold,
                    self.required_frames
                )
            )

        _, _, v = runtime.color_sensor.get_raw_color_hsv()

        if v <= self.black_threshold:
            self.detect_count += 1
        else:
            self.detect_count = 0

        if self.detect_count >= self.required_frames:

            self.logger.info(
                "%s BLACK detected v=%.1f count=%d"
                % (
                    self.__class__.__name__,
                    v,
                    self.detect_count
                )
            )

            return Status.SUCCESS

        return Status.RUNNING

    def terminate(self, new_status):
        self.detect_count = 0

class IsColorPassed(Behaviour):
    # 指定色に入り、その後その色から抜けた(通り抜けた)時点で成功する。
    def __init__(self, name: str, color: Color):
        super().__init__(name)
        self.color = color
        self.prev_color = Color.UNKNOWN
        self.classifier = ColorClassifier()
        self.running = False
        self.entered = False

    def update(self) -> Status:
        runtime.require("plotter", "color_sensor")
        cur_dist = runtime.plotter.get_distance()
        if not self.running:
            self.running = True
            self.entered = False
            self.logger.info(
                "%+06d %s.waiting to pass through color=%s"
                % (cur_dist, self.__class__.__name__, self.color.value)
            )

        h, s, v = runtime.color_sensor.get_raw_color_hsv()
        detected_color = self.classifier.classify(h, s, v)
        if detected_color != self.prev_color:
            self.logger.info(
                "%+06d %s.color changed from %s to %s"
                % (cur_dist, self.__class__.__name__, self.prev_color.value, detected_color.value)
            )
            self.prev_color = detected_color

        if not self.entered:
            if detected_color == self.color:
                self.entered = True
                self.logger.info(
                    "%+06d %s.entered color=%s"
                    % (cur_dist, self.__class__.__name__, self.color.value)
                )
            return Status.RUNNING

        if detected_color != self.color:
            self.logger.info(
                "%+06d %s.passed through color=%s"
                % (cur_dist, self.__class__.__name__, self.color.value)
            )
            return Status.SUCCESS
        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        # enteredはここでリセットしない。他のBehaviourがこのインスタンスの
        # enteredを安全上限の無効化条件として参照するため、通過成功直後の
        # terminateでFalseへ戻すと「まだ入っていない」と誤認させてしまう。
        # 同じインスタンスが本当に再実行される場合は、update()側のrunning
        # チェックで開始時に改めてenteredをFalseへ初期化する。
        self.running = False
        self.prev_color = Color.UNKNOWN


class IsDistanceEarnedUntilColorEntered(Behaviour):
    # guard(IsColorPassed等)がまだ対象色に入っていない間だけ距離を計測して
    # 安全上限として成功する。guardが色に入った(entered=True)後は、この
    # Behaviour自体は成功しなくなり(RUNNINGを返し続ける)、安全上限が無効化される。
    def __init__(self, name: str, delta_dist: int, guard: IsColorPassed):
        super().__init__(name)
        self.delta_dist = delta_dist
        self.guard = guard
        self.running = False
        self.orig_dist = 0

    def update(self) -> Status:
        if self.guard.entered:
            # 対象色に入った後は安全上限を無効化し、以後は本来の成功条件(guard側)に委ねる。
            return Status.RUNNING
        runtime.require("plotter")
        if not self.running:
            self.running = True
            self.orig_dist = runtime.plotter.get_distance()
            self.logger.info(
                "%+06d %s.accumulation started for delta=%d"
                % (self.orig_dist, self.__class__.__name__, self.delta_dist)
            )

        cur_dist = runtime.plotter.get_distance()
        if abs(cur_dist - self.orig_dist) >= self.delta_dist:
            self.logger.info(
                "%+06d %s.delta distance earned"
                % (cur_dist, self.__class__.__name__)
            )
            return Status.SUCCESS
        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        self.running = False


class IsColorDetectedThenHeadingStable(Behaviour):
    """色を一度検知した後、指定方位が安定してから成功する。"""

    def __init__(
        self,
        name: str,
        color: Color,
        target_heading_deg: float,
        tolerance_deg: float,
        stable_samples: int,
    ) -> None:
        super().__init__(name)
        self.detector = IsColorDetected(name + ' color', color)
        self.target_heading_deg = target_heading_deg
        self.tolerance_deg = tolerance_deg
        self.stable_samples = stable_samples
        self.color_latched = False
        self.heading_count = 0

    def update(self) -> Status:
        runtime.require('plotter', 'gyro_sensor')
        if not self.color_latched:
            self.detector.tick_once()
            self.color_latched = self.detector.status == Status.SUCCESS
            if not self.color_latched:
                return Status.RUNNING
            self.logger.info(
                '%+06d %s.color latched; waiting for heading'
                % (runtime.plotter.get_distance(), self.__class__.__name__)
            )

        heading = -runtime.course * float(runtime.gyro_sensor.get_angle())
        error = (self.target_heading_deg - heading + 180.0) % 360.0 - 180.0
        self.heading_count = self.heading_count + 1 if abs(error) <= self.tolerance_deg else 0
        if self.heading_count >= self.stable_samples:
            self.logger.info(
                '%+06d %s.heading stable heading=%.1f error=%.1f'
                % (runtime.plotter.get_distance(), self.__class__.__name__, heading, error)
            )
            return Status.SUCCESS
        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        if self.detector.status != Status.INVALID:
            self.detector.stop(Status.INVALID)
        self.color_latched = False
        self.heading_count = 0


class IsColorTransitionDetected(Behaviour):
    # 指定した開始色を確認した後、終了色へ変化したことを連続検出時間で判定する。
    # 開始色の時間を0にすると、開始色は1回の確定判定だけで通過扱いになる。
    # 開始地点が白地や色付き領域でも、開始色を一度通過するまでは成功しない。
    def __init__(
        self,
        name: str,
        from_color: Color,
        to_color: Color,
        from_duration_sec: float,
        to_duration_sec: float,
    ):
        super().__init__(name)
        if from_duration_sec < 0 or to_duration_sec <= 0:
            raise ValueError("from duration must be non-negative and to duration must be positive")
        self.from_color = from_color
        self.to_color = to_color
        self.from_duration_sec = from_duration_sec
        self.to_duration_sec = to_duration_sec
        self.classifier = ColorClassifier()
        self.from_started_at = None
        self.to_started_at = None
        self.from_confirmed = False

    def update(self) -> Status:
        runtime.require("plotter", "color_sensor")
        h, s, v = runtime.color_sensor.get_raw_color_hsv()
        detected_color = self.classifier.classify(h, s, v)
        now = time.monotonic()

        # まず黒ラインなどの開始色へ確実に入ったことを確認する。
        if not self.from_confirmed:
            if detected_color == self.from_color:
                if self.from_started_at is None:
                    self.from_started_at = now
                if now - self.from_started_at >= self.from_duration_sec:
                    self.from_confirmed = True
                    self.logger.info(
                        "%+06d %s.from color=%s confirmed for %.3fs"
                        % (
                            runtime.plotter.get_distance(),
                            self.__class__.__name__,
                            self.from_color.value,
                            self.from_duration_sec,
                        )
                    )
            else:
                self.from_started_at = None
            return Status.RUNNING

        # 開始色確認後に白地などの終了色が規定時間続いた場合だけ、境界を抜けたと判断する。
        if detected_color == self.to_color:
            if self.to_started_at is None:
                self.to_started_at = now
            if now - self.to_started_at >= self.to_duration_sec:
                self.logger.info(
                    "%+06d %s.transition %s to %s detected for %.3fs"
                    % (
                        runtime.plotter.get_distance(),
                        self.__class__.__name__,
                        self.from_color.value,
                        self.to_color.value,
                        self.to_duration_sec,
                    )
                )
                return Status.SUCCESS
        else:
            self.to_started_at = None
        return Status.RUNNING
