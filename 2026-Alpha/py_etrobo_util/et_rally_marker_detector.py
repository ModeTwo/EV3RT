"""ETラリー区画内の、座標プリント(4隅がグレーの円に囲まれた四角い模様)を、
QRコードとしてデコードせず、単なる物体として検出するための専用モジュール。

別チームから入手した参考プログラム(marker_detector.py)のアルゴリズムを土台に、
このプロジェクトへ組み込むための調整を行った:
  - デバッグ用のcv2.imshow呼び出しは、常時起動するとVideoThread(video.py)の
    ループを不要に遅くするため削除した。デバッグ画像が欲しい場合は、
    detect(frame, want_debug_image=True)で取得できるようにした
    (Video.preview_enabledと同じ、必要な時だけ有効化する設計に合わせた)。
  - 赤枠除去(_remove_red_overlay)は、参考プログラム自身のデバッグ描画を
    次フレームで誤検出しないための対策だった。このモジュールは検出専用で、
    入力フレームに事前の描画を行わないため、不要と判断し削除した。
  - それ以外の検出ロジック(CLAHE→二値化→内部模様の塊化→輪郭→
    角の多さでの判定)は、参考プログラムをほぼそのまま踏襲している。

【未確認・要調整】MIN_CORNER_COUNT等の閾値は、参考プログラム側のコメントに
「現時点では仮値。実測結果を見て後で調整する」とあった通りの仮値のまま。
実機のマーカーで撮影したテスト画像が無いため、本モジュールでも未調整。
"""
import sys
import platform

if platform.python_implementation() == 'CPython':
    sys.path.append('/usr/lib/python3/dist-packages')
elif platform.python_implementation() == 'PyPy':
    sys.path.append('/usr/local/lib/pypy3/dist-packages')

import cv2
import numpy as np


# 検出用に縮小する上限(これを超える幅の画像は、この幅に縮小してから処理する)
RESIZE_MAX_WIDTH = 960

# 候補領域(輪郭)の面積フィルタ
MIN_AREA_PX = 80
MAX_AREA_RATIO = 0.06  # 画像全体の面積に対する上限

# 候補領域の一辺の最小画素数
MIN_RECT_SIDE_PX = 8

# 縦横比の上限(床置きマーカーを斜めから見るため、ある程度の横長は許容する)
MAX_ASPECT_RATIO = 4.0

# 画像全体に対して大きすぎる候補は除外(近すぎる誤検出対策)
MAX_RECT_SIDE_RATIO = 0.40

# 内部の角の数による判定。QR風の模様は角が多く、丸や単純な塊は角が少ない。
# 【仮値】実測結果を見て後で調整すること(marker_detector.py由来のコメントを踏襲)。
MIN_CORNER_COUNT = 4


def _order_corners(points):
    """4隅を 左上・右上・右下・左下 の順に並べる。"""
    pts = np.array(points, dtype=np.float32)
    ordered = np.zeros((4, 2), dtype=np.float32)
    s = pts.sum(axis=1)
    diff = np.diff(pts, axis=1).reshape(-1)
    ordered[0] = pts[np.argmin(s)]      # 左上: x+yが最小
    ordered[1] = pts[np.argmin(diff)]   # 右上: y-xが最小
    ordered[2] = pts[np.argmax(s)]      # 右下: x+yが最大
    ordered[3] = pts[np.argmax(diff)]   # 左下: y-xが最大
    return ordered


def _count_corners(gray_roi):
    """候補領域の内部模様にある角の数を、Shi-Tomasi法で数える。

    QR風マーカー: 内部に細かい白黒模様があり、角が多い。
    丸・単純な塊: 角が比較的少ない。
    """
    if gray_roi is None:
        return 0
    h, w = gray_roi.shape[:2]
    if h < 8 or w < 8:
        return 0

    # 候補そのものの外形ではなく、中身の模様を見たいので外周を少し除外する
    margin_x = max(1, int(w * 0.08))
    margin_y = max(1, int(h * 0.08))
    x1, x2 = margin_x, w - margin_x
    y1, y2 = margin_y, h - margin_y
    if x2 <= x1 or y2 <= y1:
        return 0

    core = cv2.equalizeHist(gray_roi[y1:y2, x1:x2])
    corners = cv2.goodFeaturesToTrack(
        core, maxCorners=50, qualityLevel=0.02, minDistance=2,
        blockSize=3, useHarrisDetector=False)
    if corners is None:
        return 0
    return len(corners)


def detect(frame, *, want_debug_image=False):
    """フレームから、QR風の四角マーカー候補を検出する(内容はデコードしない)。

    戻り値: マーカー候補のlist(面積の大きい順)。各要素は
        {"corners": [(x,y)x4 左上→右上→右下→左下], "center": (x,y),
         "area": float, "corner_count": int}
    want_debug_image=Trueなら、(markers, debug_image)のtupleを返す
    (debug_imageは元フレームに候補の矩形を描き込んだコピー)。
    """
    if frame is None:
        return ([], None) if want_debug_image else []

    original_height, original_width = frame.shape[:2]
    scale = 1.0
    if original_width > RESIZE_MAX_WIDTH:
        scale = RESIZE_MAX_WIDTH / float(original_width)
        work = cv2.resize(frame, None, fx=scale, fy=scale)
    else:
        work = frame.copy()
    height, width = work.shape[:2]

    gray_original = cv2.cvtColor(work, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    gray = clahe.apply(gray_original)
    gray_blur = cv2.GaussianBlur(gray, (3, 3), 0)
    binary = cv2.adaptiveThreshold(
        gray_blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV, 31, 7)

    close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    cleaned = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, close_kernel, iterations=1)

    # マーカー内部の細かい黒模様を、1つの塊にまとめる
    group_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 7))
    merged = cv2.dilate(cleaned, group_kernel, iterations=1)

    contours, _ = cv2.findContours(merged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    markers = []
    debug_image = work.copy() if want_debug_image else None
    max_area = width * height * MAX_AREA_RATIO

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < MIN_AREA_PX or area > max_area:
            continue

        rect = cv2.minAreaRect(contour)
        rect_width, rect_height = rect[1]
        if rect_width <= 0 or rect_height <= 0:
            continue
        if rect_width < MIN_RECT_SIDE_PX or rect_height < MIN_RECT_SIDE_PX:
            continue

        ratio = max(rect_width, rect_height) / min(rect_width, rect_height)
        if ratio > MAX_ASPECT_RATIO:
            continue
        if rect_width > width * MAX_RECT_SIDE_RATIO or rect_height > height * MAX_RECT_SIDE_RATIO:
            continue

        x, y, w, h = cv2.boundingRect(contour)
        if w <= 0 or h <= 0:
            continue
        roi_gray = gray_original[y:y + h, x:x + w]
        corner_count = _count_corners(roi_gray)

        if debug_image is not None:
            color = (0, 255, 0) if corner_count >= MIN_CORNER_COUNT else (0, 255, 255)
            cv2.rectangle(debug_image, (x, y), (x + w, y + h), color, 1)

        if corner_count < MIN_CORNER_COUNT:
            continue

        box = cv2.boxPoints(rect)
        ordered = _order_corners(box) / scale
        center_x = float(np.mean(ordered[:, 0]))
        center_y = float(np.mean(ordered[:, 1]))
        corners = [(float(p[0]), float(p[1])) for p in ordered]

        markers.append({
            "corners": corners,
            "center": (center_x, center_y),
            "area": area / (scale * scale),
            "corner_count": corner_count,
        })

    markers.sort(key=lambda m: m["area"], reverse=True)
    return (markers, debug_image) if want_debug_image else markers
