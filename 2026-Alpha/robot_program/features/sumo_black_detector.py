"""Full-image acquisition and predictive ROI tracking, independent of robot IO.

Coordinates refer to the existing central 16:9 crop resized to 320x180.
This implements the full_hybrid policy evaluated in the rescue study.
"""
import math

import cv2
import numpy as np


def pixel_bearing(cx):
    # Existing video.py geometry; camera/arm alignment still needs field calibration.
    return math.degrees(math.atan((cx - 160.0) * 270.0 / 320.0 / 230.0))


def prepare(frame):
    h, w = frame.shape[:2]
    ch = min(h, int(w * 9 / 16))
    top = (h - ch) // 2
    return cv2.resize(frame[top:top + ch], (320, 180), interpolation=cv2.INTER_AREA)


class SumoBlackDetector:
    def __init__(self):
        self.kernel = np.ones((7, 7), np.uint8)
        self.history = []
        self.locked = False
        self.misses = 0
        self.top = 0
        self.last_frame_id = -1

    @staticmethod
    def compatible(a, b):
        return bool(a and b and b['frame_id'] > a['frame_id']
                    and 0 < b['t'] - a['t'] <= 0.5 and 0 <= b['d'] - a['d'] <= 180
                    and abs(b['cx'] - a['cx']) <= 40 and abs(b['cy'] - a['cy']) <= 25)

    def _detect(self, mask, roi, d, t, frame_id):
        mask = mask.copy()
        mask[:roi[0]] = 0
        mask[roi[1]:] = 0
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        # Preserve the study's order: select the largest original-valid contour,
        # then apply the additional height/aspect guard to that selected contour.
        for cnt in contours:
            area = float(cv2.contourArea(cnt))
            if area < 150:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            if h <= 0 or w > 176 or w / h > 4 or area / (w * h) < 0.45:
                continue
            if best is None or area > best['area']:
                best = dict(x=x, y=y, w=w, h=h, cx=x + w // 2, cy=y + h / 2,
                            area=area, d=d, t=t, frame_id=frame_id)
        if best and (best['h'] < 12 or best['w'] / best['h'] > 1.5):
            return None
        return best

    def step(self, image, d, t, frame_id):
        if frame_id <= self.last_frame_id:
            return None  # Re-reading a frame must not advance confirmation/misses.
        if not all(math.isfinite(v) for v in (d, t)):
            raise ValueError('capture time and odometry must be finite')
        self.last_frame_id = frame_id
        if self.history and not (0 <= t - self.history[-1]['t'] <= .5
                                 and 0 <= d - self.history[-1]['d'] <= 180):
            self.history = []
            self.locked = False
            self.misses = 0
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array((0, 0, 0), np.uint8), np.array((179, 120, 60), np.uint8))
        roi = (0, 180)
        candidate = None
        mode = 'full-acquire'
        if self.locked:
            last = self.history[-1]
            slope = 0.0
            if len(self.history) > 1:
                a, b = self.history[-2:]
                dd = b['d'] - a['d']
                if dd > 0:
                    slope = min(.2, max(0, (b['cy'] - a['cy']) / dd))
            center = last['cy'] + slope * max(0, d - last['d'])
            top = min(84, max(self.top, int(round(center - 48))))
            roi = (top, top + 96)
            point = self._detect(mask, roi, d, t, frame_id)
            if self.compatible(last, point):
                candidate = point
                mode = 'track'
            else:
                roi = (0, 180)
                point = self._detect(mask, roi, d, t, frame_id)
                if self.compatible(last, point):
                    candidate = point
                mode = 'full-recover'
        else:
            candidate = self._detect(mask, roi, d, t, frame_id)
        confirmed = False
        if candidate is None:
            self.misses += 1
            if not self.locked or self.misses >= 2:
                self.history = []
                self.locked = False
        else:
            confirmed = bool(self.history and self.compatible(self.history[-1], candidate))
            if not confirmed:
                self.history = []
            self.history = (self.history + [candidate])[-2:]
            self.locked = confirmed
            self.misses = 0
            self.top = roi[0]
        return dict(candidate=candidate, confirmed=confirmed, roi=roi, mode=mode)
