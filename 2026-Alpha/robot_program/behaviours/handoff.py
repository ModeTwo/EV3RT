"""Capture a section origin without resetting encoders, gyro or Plotter."""

from py_trees.behaviour import Behaviour
from py_trees.common import Status

from ..runtime import runtime


class CaptureAtToHandoff(Behaviour):
    # 移植時にATの完全停止直後・TOの最初の旋回直前へ挿入する。
    def __init__(self, name, context):
        super().__init__(name)
        self.context = context

    def update(self):
        runtime.require('plotter', 'gyro_sensor')
        if runtime.course not in (-1, 1):
            raise RuntimeError('Course must be configured before AT_TO capture')
        state = self.context.at_to
        state.distance_mm = runtime.plotter.get_distance()
        state.heading_deg = -runtime.course * runtime.gyro_sensor.get_angle()
        self.logger.info(
            'AT_TO handoff distance_mm=%.1f heading_deg=%.1f bottle_color=%s'
            % (state.distance_mm, state.heading_deg, self.context.bottle_color)
        )
        return Status.SUCCESS
