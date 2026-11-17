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
    # 2026-11-17(devSS): 候補の中の画素の彩度(S)の中央値の下限。床の灰色の点(ゲートの置き場の印)は色相が青の範囲に入り
    # S が下限45ぎりぎり(46〜54)で、青い丸と誤って確定することがあった(面積 最大561)。本物の青い丸は 226〜230
    # (11/17 の A・C の129枚、本物54個・灰色の点33個で重なりなし)。マスク自体は変えず、候補ごとに中央値で見る。
    MIN_MEDIAN_SATURATION = 120
    # 2026-11-17(devSS): 青い丸が写っていないときの手がかり。黒線の上で青い丸の隣にある赤い丸(床の横長の楕円)を探し、
    # 青い丸の横の位置を推定する。11/17 の画像で両方写っていたとき、青の cx − 赤の cx は
    # 右コース(course=-1)で +60〜+107px、左コース(+1)で −66〜−165px → 青 ≈ 赤 − course × 90px(±50px くらい)。
    RED_HSV_RANGES = (((0, 120, 60), (10, 255, 255)), ((165, 120, 60), (180, 255, 255)))
    RED_MIN_AREA = 150
    RED_TO_BLUE_PX = 90.0
    SIDE_DEADBAND_PX = 40.0

    def __init__(self):
        self.stable_frames = 0
        self.previous_center = None
        self.previous_area = None
        self.bottom_previous = None
        self.bottom_frames = 0

    def _median_saturation(self, contour, saturation, shape):
        x, y, w, h = cv2.boundingRect(contour)
        local = np.zeros((h, w), np.uint8)
        shifted = contour.copy()
        shifted[:, :, 0] -= x
        shifted[:, :, 1] -= y
        cv2.drawContours(local, [shifted], -1, 255, -1)
        values = saturation[y:y + h, x:x + w][local > 0]
        return float(np.median(values)) if values.size else 0.0

    def _candidate(self, contour, shape, saturation=None):
        area = float(cv2.contourArea(contour))
        if area < self.MIN_AREA:
            return None
        if saturation is not None:
            sat = self._median_saturation(contour, saturation, shape)
            if sat < self.MIN_MEDIAN_SATURATION:
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

    def _bottom_hint(self, contours, shape, saturation=None):
        # 見切れ領域は後退判断専用。接近対象として確定しない。
        hints = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            x, y, w, h = cv2.boundingRect(contour)
            if (saturation is not None and area >= self.MIN_AREA
                    and self._median_saturation(contour, saturation, shape) < self.MIN_MEDIAN_SATURATION):
                continue
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

    def _red_hint(self, hsv):
        """床の赤い丸(横長の楕円、画面の上30%は除く。ゲートの赤い脚は縦長なので除く)。一番大きいもの。"""
        height = hsv.shape[0]
        red = None
        for lower, upper in self.RED_HSV_RANGES:
            part = cv2.inRange(hsv, np.array(lower, np.uint8), np.array(upper, np.uint8))
            red = part if red is None else (red | part)
        red = cv2.morphologyEx(red, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        for contour in contours:
            area = float(cv2.contourArea(contour))
            x, y, w, h = cv2.boundingRect(contour)
            if area < self.RED_MIN_AREA or y < height * 0.3 or w < 1.3 * h:
                continue
            moments = cv2.moments(contour)
            if moments["m00"] <= 0:
                continue
            if best is None or area > best["area"]:
                best = dict(cx=moments["m10"] / moments["m00"], cy=moments["m01"] / moments["m00"],
                            area=area, x=x, y=y, w=w, h=h)
        return best

    def blue_side_from_red(self, red_hint, course, image_width=320):
        """赤い丸の位置から、青い丸の推定 cx と向き('left'/'right'/'center')。分からなければ None。"""
        if red_hint is None or course not in (-1, 1):
            return None
        est = float(red_hint["cx"]) - course * self.RED_TO_BLUE_PX
        center = image_width / 2.0
        if est < center - self.SIDE_DEADBAND_PX:
            side = "left"
        elif est > center + self.SIDE_DEADBAND_PX:
            side = "right"
        else:
            side = "center"
        return dict(blue_cx_estimate=est, side=side)

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
        saturation = hsv[:, :, 1]
        bottom_hint = self._bottom_hint(contours, mask.shape, saturation)
        candidates = []
        for contour in contours:
            candidate = self._candidate(contour, mask.shape, saturation)
            if candidate is not None:
                candidates.append(candidate)

        if not candidates:
            self.stable_frames = 0
            self.previous_center = None
            self.previous_area = None
            red_hint = self._red_hint(hsv)
            course = None
            try:
                from robot_program.runtime import runtime as _runtime
                course = _runtime.course
            except Exception:
                pass
            return {
                "confirmed": False,
                "candidate": None,
                "candidate_count": 0,
                "bottom_hint": bottom_hint,
                # 2026-11-17(devSS): 青い丸が見えないときの手がかり(探す向きを決めるのに使える)。
                # red_hint: 床の赤い丸、blue_from_red: {"blue_cx_estimate": 推定の青の cx, "side": left/right/center}
                "red_hint": red_hint,
                "blue_from_red": self.blue_side_from_red(red_hint, course, mask.shape[1]),
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
