"""Stationary, fresh-frame red/blue/yellow recognition for AT."""
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_etrobo_util import BottleColor, TargetInterested
from ..runtime import runtime


class DetectBottleColor(Behaviour):
    def __init__(self, name, context, settings=None, min_area=150, min_frames=3):
        super().__init__(name)
        self.context, self.settings = context, settings
        self.min_area, self.min_frames = min_area, min_frames

    def initialise(self):
        self.context.bottle_color = None
        self.session = runtime.video.begin_bottle_read()
        self.last_frame, self.candidate, self.hits = -1, None, 0

    def update(self):
        for motor in (runtime.left_motor, runtime.right_motor):
            motor.set_power(0)
            motor.set_brake(True)
        session, frame_id, observation = runtime.video.get_bottle_observation()
        if session != self.session or frame_id <= self.last_frame:
            return Status.RUNNING
        self.last_frame = frame_id
        insight, color, _, _, _, area, _ = observation
        if not insight or area < self.min_area or color not in (BottleColor.RED, BottleColor.BLUE, BottleColor.YELLOW):
            self.candidate, self.hits = None, 0
            return Status.RUNNING
        self.hits = self.hits + 1 if color == self.candidate else 1
        self.candidate = color
        if self.hits < self.min_frames:
            return Status.RUNNING
        self.context.bottle_color = color.value
        self.logger.info('AT bottle_color=%s frame=%d' % (color.value, frame_id))
        return Status.SUCCESS

    def terminate(self, new_status):
        runtime.video.set_target_interested(TargetInterested.LINE)
