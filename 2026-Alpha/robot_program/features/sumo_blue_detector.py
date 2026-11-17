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

    # 2026-11-17(devSS): 画面の左右の端で切れている青い丸(候補からは外れる)を知らせる。
    # 相撲のあと青い丸へ向けて旋回したとき、丸が左下の端にかかって切れ、見つからずに失敗した(ロボットA)。
    # 下半分にあって、上の端に触れず(ゲートの青い脚を外す)、左右どちらかの端に触れている、一定以上の大きさの青。
    EDGE_MIN_AREA = 300
    EDGE_MIN_CY_RATIO = 0.5

    def _edge_blob(self, contours, shape):
        height, width = shape[0], shape[1]
        best = None
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.EDGE_MIN_AREA:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if y <= 0:
                continue
            left, right = x <= 0, x + w >= width
            if left == right:
                continue
            moments = cv2.moments(contour)
            if moments["m00"] <= 0:
                continue
            cx = moments["m10"] / moments["m00"]
            cy = moments["m01"] / moments["m00"]
            if cy < height * self.EDGE_MIN_CY_RATIO:
                continue
            if best is None or area > best["area"]:
                best = {"side": "left" if left else "right", "cx": cx, "cy": cy, "area": area}
        return best

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
        candidates = []
        for contour in contours:
            candidate = self._candidate(contour, mask.shape)
            if candidate is not None:
                candidates.append(candidate)

        if not candidates:
            self.stable_frames = 0
            self.previous_center = None
            self.previous_area = None
            return {
                "confirmed": False,
                "candidate": None,
                "candidate_count": 0,
                "edge": self._edge_blob(contours, mask.shape),
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
        }
