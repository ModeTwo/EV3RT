"""Read a fresh QR session and preserve raw, undeciphered hint text."""
import time
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_etrobo_util import TargetInterested
from ..runtime import runtime


class ReadHintCard(Behaviour):
    def __init__(self, name, hint_number, context, timeout_sec=20.0):
        super().__init__(name)
        if hint_number not in (1, 2):
            raise ValueError('hint_number must be 1 or 2')
        self.hint_number, self.context = hint_number, context
        self.timeout_sec, self.running = timeout_sec, False

    def update(self):
        runtime.require('plotter', 'video')
        if not self.running:
            self.session = runtime.video.begin_qr_read()
            self.started = time.monotonic()
            self.running = True
            setattr(self.context, f'hint{self.hint_number}', None)
            self.logger.info('TO hint%d reading session=%d' % (self.hint_number, self.session))
        if time.monotonic() - self.started >= self.timeout_sec:
            self.logger.error('TO hint%d timed out' % self.hint_number)
            return Status.FAILURE
        session, frame_id, raw_text = runtime.video.get_qr_observation()
        if session != self.session or frame_id < 0 or not raw_text:
            return Status.RUNNING
        if self.hint_number == 2 and raw_text == self.context.hint1:
            return Status.RUNNING
        setattr(self.context, f'hint{self.hint_number}', raw_text)
        runtime.video.set_target_interested(TargetInterested.LINE)
        self.logger.info('TO hint%d captured session=%d frame=%d' % (self.hint_number, session, frame_id))
        return Status.SUCCESS

    def terminate(self, new_status):
        if runtime.video is not None:
            runtime.video.set_target_interested(TargetInterested.LINE)
        self.running = False
