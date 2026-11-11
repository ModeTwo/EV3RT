"""ETラリー開始位置の青い丸を検出する。"""

import math

import cv2
import numpy as np


class SumoBlueDetector:
    # OpenCVのHSV値。実機カメラ画像から最終調整する。
    HSV_LOWER = (85, 45, 35)
    HSV_UPPER = (140, 255, 255)

    MIN_AREA = 80
    MIN_CIRCULARITY = 0.40
    MAX_CENTER_MOVE_PX = 35
    STABLE_FRAMES = 3

    def __init__(self):
        self.stable_frames = 0
        self.previous_center = None

    def step(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower = np.array(self.HSV_LOWER, dtype=np.uint8)
        upper = np.array(self.HSV_UPPER, dtype=np.uint8)

        mask = cv2.inRange(hsv, lower, upper)

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_OPEN, kernel
        )
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_CLOSE, kernel
        )

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        candidates = []

        for contour in contours:
            area = cv2.contourArea(contour)

            if area < self.MIN_AREA:
                continue

            perimeter = cv2.arcLength(contour, True)

            if perimeter <= 0:
                continue

            circularity = (
                4.0 * math.pi * area / (perimeter * perimeter)
            )

            if circularity < self.MIN_CIRCULARITY:
                continue

            x, y, w, h = cv2.boundingRect(contour)

            if w <= 0 or h <= 0:
                continue

            aspect_ratio = w / float(h)

            # 床面の青い丸は斜めから見ると横長になる。
            if not 0.35 <= aspect_ratio <= 3.2:
                continue

            moments = cv2.moments(contour)

            if moments["m00"] == 0:
                continue

            cx = moments["m10"] / moments["m00"]
            cy = moments["m01"] / moments["m00"]

            candidates.append({
                "x": x,
                "y": y,
                "w": w,
                "h": h,
                "cx": cx,
                "cy": cy,
                "area": area,
                "circularity": circularity,
            })

        if not candidates:
            self.stable_frames = 0
            self.previous_center = None

            return {
                "confirmed": False,
                "candidate": None,
                "candidate_count": 0,
            }

        # 前フレームで追跡していた対象に近い候補を優先。
        if self.previous_center is not None:
            px, py = self.previous_center

            nearby = [
                candidate
                for candidate in candidates
                if math.hypot(
                    candidate["cx"] - px,
                    candidate["cy"] - py,
                ) <= self.MAX_CENTER_MOVE_PX
            ]
        else:
            nearby = []

        if nearby:
            candidate = min(
                nearby,
                key=lambda item: math.hypot(
                    item["cx"] - px,
                    item["cy"] - py,
                ),
            )
            self.stable_frames += 1
        else:
            candidate = max(
                candidates,
                key=lambda item: (
                    item["circularity"],
                    item["area"],
                ),
            )
            self.stable_frames = 1

        self.previous_center = (
            candidate["cx"],
            candidate["cy"],
        )

        return {
            "confirmed": self.stable_frames >= self.STABLE_FRAMES,
            "candidate": candidate,
            "candidate_count": len(candidates),
        }
