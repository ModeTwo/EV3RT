"""ET sumo bearing adapters for the unchanged shared gyro behaviors."""

from py_trees.behaviour import Behaviour
from py_trees.common import Status

from ..behaviours.gyro_drive import RunByGyro, SpinAround
from ..runtime import runtime
from ..types import HeadingType
from .sumo_bearing import choose_search_bearing, normalize_bearing


def current_bearing(context):
    runtime.require("gyro_sensor")
    return context.sumo.bearing_reference.bearing(runtime.gyro_sensor.get_angle())


def search_bearing(context, settings):
    return choose_search_bearing(
        current_bearing(context), settings.garage_search_offset_deg,
        settings.garage_bearing_deg, prefer_clockwise=runtime.course < 0,
    )


class RegisterSumoBearing(Behaviour):
    # 全体のResetDevice完了直後に一度だけ登録する。相撲開始時には登録し直さない。
    def __init__(self, name, context, initial_bearing):
        super().__init__(name)
        self.context = context
        self.initial_bearing = normalize_bearing(initial_bearing)

    def update(self):
        runtime.require("gyro_sensor")
        raw = runtime.gyro_sensor.get_angle()
        self.context.sumo.bearing_reference.register(self.initial_bearing, raw)
        self.logger.info("sumo bearing reference: bearing=%.1f gyro=%.1f" % (self.initial_bearing, raw))
        return Status.SUCCESS


class _BearingTarget:
    def _configure_bearing(self):
        # 動的な方位はBehavior開始時に一度だけ評価する。走行中に目標を追従更新しない。
        requested = self.bearing() if callable(self.bearing) else self.bearing
        self.target_bearing = normalize_bearing(requested)
        raw = runtime.gyro_sensor.get_angle()
        self.target = self.context.sumo.bearing_reference.legacy_target(self.target_bearing, raw, runtime.course)
        self.logger.info("bearing target=%.1f current=%.1f legacy_target=%.1f gyro=%.1f" % (
            self.target_bearing, current_bearing(self.context), self.target, raw))


class SpinToBearing(_BearingTarget, SpinAround):
    # 実際の旋回・停止処理は既存SpinAroundをそのまま使う。
    def __init__(self, name, context, bearing, **kwargs):
        self.context = context
        self.bearing = bearing
        super().__init__(name=name, target=0, target_type=HeadingType.ABSOLUTE, **kwargs)

    def update(self):
        if not self.running:
            self._configure_bearing()
        result = super().update()
        if result == Status.SUCCESS:
            self.logger.info("sumo turn complete bearing=%.1f" % current_bearing(self.context))
        return result


class RunAtBearing(_BearingTarget, RunByGyro):
    # 実際のPID走行は既存RunByGyroへ委譲し、他工程のABSOLUTEの意味を変えない。
    def __init__(self, name, context, bearing, **kwargs):
        self.context = context
        self.bearing = bearing
        super().__init__(name=name, target=0, target_type=HeadingType.ABSOLUTE, **kwargs)

    def update(self):
        if not self.running:
            self._configure_bearing()
        return super().update()
