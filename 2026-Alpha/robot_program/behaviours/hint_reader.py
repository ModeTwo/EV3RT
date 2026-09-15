"""Read a fresh QR session and preserve raw, undeciphered hint text."""
import time
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_etrobo_util import TargetInterested
from ..runtime import runtime


class ReadHintCard(Behaviour):
    WAIT_LOG_INTERVAL_SEC = 2.0  # 同じ待機理由の出力間隔。工程の時間制限ではない。
    def __init__(self, name, hint_number, context):
        super().__init__(name)
        if hint_number not in (1, 2):
            raise ValueError('hint_number must be 1 or 2')
        self.hint_number, self.context = hint_number, context
        self.running = False

    def update(self):
        runtime.require('plotter', 'video')
        if not self.running:
            self.started_at = time.monotonic()
            self.last_wait_reason = None
            self.last_wait_log = self.started_at
            self.session = runtime.video.begin_qr_read()
            self.running = True
            setattr(self.context, f'hint{self.hint_number}', None)
            self.logger.info('TO hint%d reading session=%d' % (self.hint_number, self.session))
        try:
            session, frame_id, raw_text = runtime.video.get_qr_observation()
        except Exception as error:
            self.logger.error('QR_ERROR hint=%d session=%d elapsed=%.2fs error=%r cause=%r' % (
                self.hint_number, self.session, time.monotonic()-self.started_at,
                error, error.__cause__))
            raise
        if session != self.session or frame_id < 0 or not raw_text:
            reason = 'session_mismatch' if session != self.session else 'no_decoded_text'
            self._log_wait(reason, session, frame_id, raw_text)
            return Status.RUNNING
        if self.hint_number == 2 and raw_text == self.context.hint1:
            self._log_wait('same_as_hint1', session, frame_id, raw_text)
            return Status.RUNNING
        setattr(self.context, f'hint{self.hint_number}', raw_text)
        runtime.video.set_target_interested(TargetInterested.LINE)
        self.logger.info('QR_SUCCESS hint=%d session=%d frame=%d elapsed=%.2fs text=%r' % (
            self.hint_number, session, frame_id, time.monotonic()-self.started_at, raw_text))
        return Status.SUCCESS

    def _log_wait(self, reason, session, frame_id, raw_text):
        now = time.monotonic()
        if reason != self.last_wait_reason or now-self.last_wait_log >= self.WAIT_LOG_INTERVAL_SEC:
            self.logger.info('QR_WAIT hint=%d reason=%s elapsed=%.2fs session=%d expected=%d frame=%d text=%r' % (
                self.hint_number, reason, now-self.started_at, session, self.session, frame_id, raw_text))
            self.last_wait_reason, self.last_wait_log = reason, now

    def terminate(self, new_status):
        if self.running:
            self.logger.info('QR_END hint=%d status=%s elapsed=%.2fs camera=LINE' % (
                self.hint_number, new_status, time.monotonic()-self.started_at))
        if runtime.video is not None:
            runtime.video.set_target_interested(TargetInterested.LINE)
        self.running = False
