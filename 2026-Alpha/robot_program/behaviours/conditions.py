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
