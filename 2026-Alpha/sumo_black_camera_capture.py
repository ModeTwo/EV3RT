#!/usr/bin/env python3
"""Preview the ET-sumo black-bottle detector and save frames with S."""

import argparse
from datetime import datetime
from pathlib import Path
import platform
import sys
import time

# Match py_etrobo_util/video.py: Raspberry Pi system packages may live here.
if platform.python_implementation() == 'CPython':
    sys.path.append('/usr/lib/python3/dist-packages')
elif platform.python_implementation() == 'PyPy':
    sys.path.append('/usr/local/lib/pypy3/dist-packages')

import cv2
import numpy as np

# Keep these values synchronized with py_etrobo_util/video.py.
FRAME_WIDTH = 320
FRAME_HEIGHT = 180
BOTTLE_MIN_AREA = 150
BOTTLE_MIN_EXTENT = 0.45
BOTTLE_BLACK_MAX_W = int(FRAME_WIDTH * 0.55)
BOTTLE_BLACK_MAX_ASPECT = 4.0
BOTTLE_HSV_BLACK_LOWER = np.array((0, 0, 0), dtype=np.uint8)
BOTTLE_HSV_BLACK_UPPER = np.array((179, 120, 60), dtype=np.uint8)
MORPH_KERNEL_SIZE = 7


def detect_black(frame, kernel):
    """Return prepared frame, raw HSV threshold mask, and best qualified contour."""
    fh, fw = frame.shape[:2]
    crop_h = int(fw * 9 / 16)
    y0 = max(0, (fh - crop_h) // 2)
    frame_169 = frame[y0:y0 + min(crop_h, fh), :]
    prepared = cv2.resize(frame_169, (FRAME_WIDTH, FRAME_HEIGHT))
    hsv = cv2.cvtColor(prepared, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, BOTTLE_HSV_BLACK_LOWER, BOTTLE_HSV_BLACK_UPPER)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best = None
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < BOTTLE_MIN_AREA:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        extent = area / float(w * h)
        aspect = w / float(h) if h > 0 else 999.0
        if w > BOTTLE_BLACK_MAX_W or aspect > BOTTLE_BLACK_MAX_ASPECT or extent < BOTTLE_MIN_EXTENT:
            continue
        if best is None or area > best[0]:
            best = (area, x, y, w, h, extent, aspect)
    return prepared, mask, best


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera', type=int, default=0, help='camera index (default: 0)')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--output-dir', type=Path, default=Path(__file__).resolve().parent / 'sumo_black_captures')
    args = parser.parse_args()

    backend = cv2.CAP_V4L2 if hasattr(cv2, 'CAP_V4L2') else cv2.CAP_ANY
    camera = cv2.VideoCapture(args.camera, backend)
    if not camera.isOpened():
        parser.error('could not open camera index %d' % args.camera)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    kernel = np.ones((MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE), dtype=np.uint8)
    preview_name = 'ET-sumo black detector (S=save, Q/Esc=quit)'
    cv2.namedWindow(preview_name, cv2.WINDOW_NORMAL)
    print('S: save raw frame, detector preview, and mask; Q/Esc: quit')
    print('Output directory:', args.output_dir.resolve())

    try:
        while True:
            ok, frame = camera.read()
            if not ok or frame is None:
                print('Camera read failed; retrying')
                time.sleep(0.1)
                key = cv2.waitKey(1) & 0xff
                if key in (ord('q'), 27):
                    break
                continue

            prepared, mask, best = detect_black(frame, kernel)
            view = prepared.copy()
            if best is None:
                label = 'No qualified black candidate'
                color = (30, 30, 230)
            else:
                area, x, y, w, h, extent, aspect = best
                cv2.rectangle(view, (x, y), (x + w, y + h), (0, 220, 255), 2)
                cv2.circle(view, (x + w // 2, y + h // 2), 4, (0, 0, 255), -1)
                label = 'BLACK area=%d extent=%.2f aspect=%.2f' % (area, extent, aspect)
                color = (0, 220, 255)
            cv2.putText(view, label, (5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)
            cv2.putText(view, 'S: save  Q/Esc: quit', (5, FRAME_HEIGHT - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.imshow(preview_name, view)
            cv2.imshow('Black threshold mask (white=pixel passes HSV)', mask)

            key = cv2.waitKey(1) & 0xff
            if key in (ord('q'), 27):
                break
            if key in (ord('s'), ord('S')):
                stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
                outputs = (
                    (args.output_dir / (stamp + '_raw.png'), frame),
                    (args.output_dir / (stamp + '_detector.png'), view),
                    (args.output_dir / (stamp + '_mask.png'), mask),
                )
                if all(cv2.imwrite(str(path), image) for path, image in outputs):
                    print('Saved:', ', '.join(path.name for path, _ in outputs))
                else:
                    print('One or more images failed to save', file=sys.stderr)
    except KeyboardInterrupt:
        print('\nInterrupted')
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

