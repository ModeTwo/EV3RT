"""床面の青い楕円を検出し、位置と面積で連続確認する。"""
import math
import cv2
import numpy as np


class SumoBlueDetector:
    HSV_LOWER = (85, 45, 35)
    HSV_UPPER = (140, 255, 255)
    MIN_AREA = 80
    MIN_ASPECT = 1.4
    MAX_ASPECT = 5.0
    MAX_TILT_DEG = 25.0
    MIN_ELLIPSE_IOU = 0.82
    MAX_CENTER_MOVE_PX = 35
    STABLE_FRAMES = 3

    def __init__(self):
        self.stable_frames = 0
        self.previous_center = None
        self.previous_area = None
        self.bottom_previous = None
        self.bottom_frames = 0

    def _candidate(self, contour, shape):
        area = float(cv2.contourArea(contour))
        if area < self.MIN_AREA:
            return None
        x, y, w, h = cv2.boundingRect(contour)
        # 欠けた領域の中心を、丸全体の中心として使わない。
        if x <= 0 or y <= 0 or x + w >= shape[1] or y + h >= shape[0]:
            return None
        if not self.MIN_ASPECT <= w / float(h) <= self.MAX_ASPECT:
            return None
        if len(contour) < 5:
            return None

        try:
            ellipse = cv2.fitEllipse(contour)
        except cv2.error:
            return None
        (ex, ey), (aw, ah), angle = ellipse
        if not all(math.isfinite(v) for v in (ex, ey, aw, ah, angle)):
            return None
        if min(aw, ah) <= 0:
            return None
        if max(aw, ah) / min(aw, ah) > self.MAX_ASPECT:
            return None

        long_axis_angle = angle if aw >= ah else angle + 90.0
        tilt = abs((long_axis_angle + 90.0) % 180.0 - 90.0)
        if tilt > self.MAX_TILT_DEG:
            return None

        # 楕円が外接矩形からはみ出す部分も含めて一致度を計算。
        pad = max(w, h) + 4
        actual = np.zeros((h + 2 * pad, w + 2 * pad), np.uint8)
        fitted = np.zeros_like(actual)
        local = contour.copy()
        local[:, :, 0] += pad - x
        local[:, :, 1] += pad - y
        cv2.drawContours(actual, [local], -1, 255, -1)
        cv2.ellipse(
            fitted,
            ((ex - x + pad, ey - y + pad), (aw, ah), angle),
            255, -1,
        )
        union = int(np.count_nonzero(actual | fitted))
        if union == 0:
            return None
        iou = float(np.count_nonzero(actual & fitted)) / union
        if iou < self.MIN_ELLIPSE_IOU:
            return None

        moments = cv2.moments(contour)
        if moments["m00"] <= 0:
            return None
        perimeter = cv2.arcLength(contour, True)
        circularity = (
            4.0 * math.pi * area / (perimeter * perimeter)
            if perimeter > 0 else 0.0
        )
        return {
            "x": x, "y": y, "w": w, "h": h,
            "cx": moments["m10"] / moments["m00"],
            "cy": moments["m01"] / moments["m00"],
            "area": area,
            "circularity": circularity,
            "ellipse_iou": iou,
            "ellipse_tilt_deg": tilt,
        }

    def _bottom_hint(self, contours, shape):
        # 見切れ領域は後退判断専用。接近対象として確定しない。
        hints = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            x, y, w, h = cv2.boundingRect(contour)
            if (area < self.MIN_AREA or w < 20 or h < 4
                    or y < shape[0] * 0.5
                    or y + h < shape[0] - 1
                    or not 1.4 <= w / float(h) <= 10.0
                    or area / (w * h) < 0.5):
                continue
            hull_area = float(
                cv2.contourArea(cv2.convexHull(contour))
            )
            if hull_area <= 0 or area / hull_area < 0.85:
                continue
            hints.append(dict(
                cx=x + w / 2.0, cy=y + h / 2.0,
                area=area, x=x, y=y, w=w, h=h,
            ))

        if not hints:
            self.bottom_previous = None
            self.bottom_frames = 0
            return None

        hint = max(hints, key=lambda item: item["area"])
        previous = self.bottom_previous
        same = (
            previous is not None
            and math.hypot(
                hint["cx"] - previous["cx"],
                hint["cy"] - previous["cy"],
            ) <= 35.0
            and 0.5 <= hint["area"] / previous["area"] <= 2.0
        )
        self.bottom_frames = self.bottom_frames + 1 if same else 1
        self.bottom_previous = hint
        return hint if self.bottom_frames >= 3 else None

    def _split_noisy_contour(self, contour, shape):
        # 通常の楕円判定に失敗した大きな輪郭だけ再分離する。
        area = float(cv2.contourArea(contour))
        if area < 600.0:
            return []
        isolated = np.zeros(shape, np.uint8)
        cv2.drawContours(isolated, [contour], -1, 255, -1)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        isolated = cv2.morphologyEx(
            isolated, cv2.MORPH_OPEN, kernel
        )
        pieces, _ = cv2.findContours(
            isolated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        result = []
        for piece in pieces:
            candidate = self._candidate(piece, shape)
            if (
                candidate is not None
                and candidate["area"] >= area * 0.25
            ):
                result.append(candidate)
        return result

    def step(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            np.array(self.HSV_LOWER, dtype=np.uint8),
            np.array(self.HSV_UPPER, dtype=np.uint8),
        )
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        bottom_hint = self._bottom_hint(contours, mask.shape)
        candidates = []
        for contour in contours:
            candidate = self._candidate(contour, mask.shape)
            if candidate is not None:
                candidates.append(candidate)
            else:
                candidates.extend(
                    self._split_noisy_contour(contour, mask.shape)
                )

        if not candidates:
            self.stable_frames = 0
            self.previous_center = None
            self.previous_area = None
            return {
                "confirmed": False,
                "candidate": None,
                "candidate_count": 0,
                "bottom_hint": bottom_hint,
            }

        nearby = []
        if self.previous_center is not None and self.previous_area is not None:
            px, py = self.previous_center
            nearby = [
                c for c in candidates
                if math.hypot(c["cx"] - px, c["cy"] - py)
                <= self.MAX_CENTER_MOVE_PX
                and 0.5 <= c["area"] / self.previous_area <= 2.0
            ]

        candidate = max(
            nearby or candidates,
            key=lambda c: (c["area"], c["ellipse_iou"]),
        )
        self.stable_frames = self.stable_frames + 1 if nearby else 1
        self.previous_center = (candidate["cx"], candidate["cy"])
        self.previous_area = candidate["area"]

        return {
            "confirmed": self.stable_frames >= self.STABLE_FRAMES,
            "candidate": candidate,
            "candidate_count": len(candidates),
            "bottom_hint": bottom_hint,
        }
