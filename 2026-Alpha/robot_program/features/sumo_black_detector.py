"""Black bottle candidates and bounded identity association, without robot IO.

カメラフレームから黒候補を作り、同一対象を関連付ける。走行判断やモーター出力は行わない。"""

import math
import cv2
import numpy as np


def pixel_bearing(cx):
    # Retained geometry: changing the angle/turn controller is a later step.
    return math.degrees(math.atan((cx - 160.0) * 270.0 / 320.0 / 230.0))


def prepare(frame):
    h, w = frame.shape[:2]
    ch = min(h, int(w * 9 / 16))
    top = (h - ch) // 2
    return cv2.resize(frame[top : top + ch], (320, 180), interpolation=cv2.INTER_AREA)


class SumoBlackDetector:
    """No predicted-only output. New read sessions require two new frames."""

    def __init__(self, mask_policy="dual"):
        if mask_policy not in ("hsv", "dual"):
            raise ValueError("mask_policy must be hsv or dual")
        self.mask_policy = mask_policy
        self.kernel = np.ones((7, 7), np.uint8)
        self.history = []
        self.locked = False
        self.misses = 0
        self.top = 0
        self.last_frame_id = -1
        self.track_id = 0
        self.streak = 0

    def new_observation_session(self):
        # Camera-thread-only: history survives, confirmation does not.
        self.streak = 0

    @staticmethod
    def _prediction(a, b):
        if a.get("gyro_deg") is None or b.get("gyro_deg") is None:
            return a["cx"]
        yaw = (b["gyro_deg"] - a["gyro_deg"] + 180) % 360 - 180
        angle = math.radians(pixel_bearing(a["cx"]) - yaw)
        if abs(angle) >= math.radians(85):
            return None
        return 160 + (320 * 230 / 270) * math.tan(angle)

    @staticmethod
    def _association(a, b):
        if not a or not b or b["frame_id"] <= a["frame_id"]:
            return None
        if not (0 < b["t"] - a["t"] <= 0.5 and 0 <= b["d"] - a["d"] <= 180):
            return None
        predicted = SumoBlackDetector._prediction(a, b)
        if predicted is None:
            return None
        dx, dy = abs(b["cx"] - predicted), abs(b["cy"] - a["cy"])
        ratio = b["area"] / a["area"]
        promote = (
            a["source"] == "hsv"
            and b["source"] == "value"
            and ratio > 1.15
            and b["x"] <= predicted <= b["x"] + b["w"]
            and b["y"] <= a["cy"] <= b["y"] + b["h"]
        )
        if (
            dx > max(40, a["w"] * 1.4)
            or dy > max(30, a["h"] * 0.6)
            or not 0.25 <= ratio <= (12 if promote else 4)
        ):
            return None
        score = dx / max(20, a["w"]) + dy / max(20, a["h"]) + 0.4 * abs(math.log(ratio))
        if promote:
            score -= 2
        return score, dx, dy

    @staticmethod
    def compatible(a, b):
        return SumoBlackDetector._association(a, b) is not None

    def _candidates(self, mask, source, d, t, frame_id, gyro_deg):
        # Apply ALL shape conditions before choosing any candidate.
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        points = []
        for cnt in contours:
            area = float(cv2.contourArea(cnt))
            x, y, w, h = cv2.boundingRect(cnt)
            if area < 150 or h < 12 or w > 176 or w / h > 1.5 or area / (w * h) < 0.45:
                continue
            points.append(
                dict(
                    x=x,
                    y=y,
                    w=w,
                    h=h,
                    cx=x + w // 2,
                    cy=y + h / 2,
                    area=area,
                    d=d,
                    t=t,
                    frame_id=frame_id,
                    gyro_deg=gyro_deg,
                    source=source,
                )
            )
        return sorted(points, key=lambda point: point["area"], reverse=True)

    def step(self, image, d, t, frame_id, gyro_deg=None):
        if frame_id <= self.last_frame_id:
            return None
        if not all(math.isfinite(v) for v in (d, t)) or (
            gyro_deg is not None and not math.isfinite(gyro_deg)
        ):
            raise ValueError("capture pose must be finite")
        self.last_frame_id = frame_id
        expired = False
        if self.history and not (
            0 < t - self.history[-1]["t"] <= 0.5
            and 0 <= d - self.history[-1]["d"] <= 180
        ):
            self.history = []
            self.locked = False
            self.streak = 0
            self.top = 0
            expired = True
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        points = self._candidates(
            cv2.inRange(hsv, (0, 0, 0), (179, 120, 60)), "hsv", d, t, frame_id, gyro_deg
        )
        hsv_count = len(points)
        value_count = 0
        if self.mask_policy == "dual":
            values = self._candidates(
                cv2.inRange(hsv, (0, 0, 0), (179, 255, 60)),
                "value",
                d,
                t,
                frame_id,
                gyro_deg,
            )
            value_count = len(values)
            points += values
        roi, candidate, association = (0, 180), None, None
        mode = "full-acquire"
        if self.history:
            previous = self.history[-1]
            top = max(0, min(84, int(previous["cy"] - 48)))
            roi = (top, top + 96)
            scored = [(self._association(previous, point), point) for point in points]
            scored = [(score, point) for score, point in scored if score is not None]
            # Prefer the ROI but never truncate contours at its border.
            near = [
                (score, point)
                for score, point in scored
                if roi[0] <= point["cy"] < roi[1]
            ]
            choices = near or scored
            if near:
                mode = "track"
            else:
                mode = "full-recover"
            if choices:
                association, candidate = min(choices, key=lambda pair: pair[0][0])
            else:
                mode = "identity-miss"
                roi = (0, 180)
        elif points:
            # Prefer distant HSV acquisition; V may merge with dark background.
            # A horizontally clipped fragment has an unknown real center. It
            # cannot seed identity; an existing track may still follow it.
            seeds = [
                point
                for point in points
                if point["x"] > 0 and point["x"] + point["w"] < 320
            ]
            if seeds:
                candidate = seeds[0]
                self.track_id += 1
                self.streak = 0
        confirmed = False
        if candidate is None:
            self.misses += 1
            self.streak = 0
            if self.misses >= 2:
                self.locked = False
        else:
            self.streak += 1
            confirmed = self.streak >= 2 and bool(self.history)
            self.history = (self.history + [candidate])[-12:]
            self.locked = confirmed
            self.misses = 0
            self.top = roi[0]
        return dict(
            candidate=candidate,
            confirmed=confirmed,
            roi=roi,
            mode=mode,
            track_id=self.track_id or None,
            track_status=(
                "confirmed" if confirmed else "tentative" if candidate else "missing"
            ),
            mask_policy=self.mask_policy,
            mask_source=candidate["source"] if candidate else "",
            hsv_candidate_count=hsv_count,
            value_candidate_count=value_count,
            history_length=len(self.history),
            association_dx=association[1] if association else None,
            association_dy=association[2] if association else None,
            history_expired=expired,
        )
