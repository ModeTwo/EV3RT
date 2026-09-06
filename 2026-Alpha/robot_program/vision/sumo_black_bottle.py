"""Black-label detector dedicated to the ET sumo bottle."""

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np


@dataclass(frozen=True)
class SumoBlackBottleConfig:
    # 3色ボトル用BOTTLE_HSVとは共有せず、力士ボトルの黒テープだけを調整する。
    frame_width: int = 320
    frame_height: int = 180
    black_max_saturation: int = 120
    black_max_value: int = 60
    min_area_px: float = 150.0
    min_extent: float = 0.45
    max_aspect_ratio: float = 4.0
    max_width_ratio: float = 0.55
    morphology_kernel_size: int = 7


@dataclass(frozen=True)
class SumoBlackBottleDetection:
    # プレビューと将来の走行制御が同じ検出結果を利用できるよう、形状情報を保持する。
    x: int
    y: int
    width: int
    height: int
    center_x: int
    bottom_y: int
    area_px: float
    extent: float
    aspect_ratio: float


class SumoBlackBottleDetector:
    def __init__(self, config: Optional[SumoBlackBottleConfig] = None) -> None:
        self.config = config or SumoBlackBottleConfig()
        size = max(1, int(self.config.morphology_kernel_size))
        if size % 2 == 0:
            size += 1
        self.kernel = np.ones((size, size), dtype=np.uint8)

    def prepare_frame(self, frame: np.ndarray) -> np.ndarray:
        # 既存カメラ処理と同じく4:3画像の中央を16:9で切り出し、320x180へ縮小する。
        if frame is None or frame.size == 0:
            raise ValueError("frame must not be empty")
        frame_height, frame_width = frame.shape[:2]
        crop_height = min(frame_height, int(frame_width * 9 / 16))
        crop_top = max(0, (frame_height - crop_height) // 2)
        cropped = frame[crop_top : crop_top + crop_height, :]
        return cv2.resize(
            cropped,
            (self.config.frame_width, self.config.frame_height),
            interpolation=cv2.INTER_AREA,
        )

    def detect(
        self, frame: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, Optional[SumoBlackBottleDetection]]:
        prepared = self.prepare_frame(frame)
        hsv = cv2.cvtColor(prepared, cv2.COLOR_BGR2HSV)

        # 色相には依存せず、低彩度かつ低明度の領域を黒として抽出する。
        lower = np.array((0, 0, 0), dtype=np.uint8)
        upper = np.array(
            (
                179,
                self.config.black_max_saturation,
                self.config.black_max_value,
            ),
            dtype=np.uint8,
        )
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        best = None
        maximum_width = self.config.frame_width * self.config.max_width_ratio

        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.config.min_area_px:
                continue
            x, y, width, height = cv2.boundingRect(contour)
            if height <= 0 or width > maximum_width:
                continue
            extent = area / float(width * height)
            aspect_ratio = width / float(height)

            # コースの黒ラインは横長になりやすいため、幅・縦横比・充填率で除外する。
            if aspect_ratio > self.config.max_aspect_ratio:
                continue
            if extent < self.config.min_extent:
                continue

            detection = SumoBlackBottleDetection(
                x=x,
                y=y,
                width=width,
                height=height,
                center_x=x + width // 2,
                bottom_y=y + height,
                area_px=area,
                extent=extent,
                aspect_ratio=aspect_ratio,
            )
            if best is None or detection.area_px > best.area_px:
                best = detection

        return prepared, mask, best
