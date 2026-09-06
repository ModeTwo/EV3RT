"""Thread-safe camera observations, keyed by read session and capture frame."""
import threading


class VisionSessions:
    def __init__(self):
        self.lock = threading.Lock()
        self.generation = 0
        self.mode = 'line'
        self.qr = (0, -1, '')
        self.bottle = (0, -1, None)
        self.error = None

    def start(self, mode):
        with self.lock:
            self.generation += 1
            self.mode = mode
            self.qr = (self.generation, -1, '')
            self.bottle = (self.generation, -1, None)
            self.error = None
            return self.generation

    def capture_token(self):
        with self.lock:
            return self.generation, self.mode

    def publish_qr(self, session, frame_id, text, error=None):
        with self.lock:
            if self.mode != 'qr' or session != self.generation:
                return False
            if error is not None:
                self.error = error
            if text and frame_id > self.qr[1]:
                self.qr = (session, frame_id, text)
            return True

    def publish_bottle(self, session, frame_id, observation):
        with self.lock:
            if self.mode != 'bottle' or session != self.generation:
                return False
            if frame_id > self.bottle[1]:
                self.bottle = (session, frame_id, observation)
            return True

    def get_qr(self):
        with self.lock:
            if self.error is not None:
                raise RuntimeError('QR decoder failed') from self.error
            return self.qr

    def get_bottle(self):
        with self.lock:
            return self.bottle
