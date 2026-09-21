import sys
import platform

# video.pyと同じOpenCVのパス設定
if platform.python_implementation() == 'CPython':
    sys.path.append('/usr/lib/python3/dist-packages')
elif platform.python_implementation() == 'PyPy':
    sys.path.append('/usr/local/lib/pypy3/dist-packages')

import cv2
import numpy as np
import time


CAMERA_ID = 0

# 実際のBottle認識と同じ入力サイズ
FRAME_WIDTH = 320
FRAME_HEIGHT = 180

# 中央の測定範囲
ROI_WIDTH = 40
ROI_HEIGHT = 80

MEASURE_SECONDS = 10


def main():

    # video.pyと同じカメラ設定
    cap = cv2.VideoCapture(CAMERA_ID, cv2.CAP_V4L2)

    cap.set(
        cv2.CAP_PROP_FOURCC,
        cv2.VideoWriter_fourcc(*"MJPG")
    )

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)

    cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)

    if not cap.isOpened():
        print("ERROR: camera could not be opened")
        return

    # カメラを安定させる
    for _ in range(5):
        cap.read()

    print("========================================")
    print(" BLACK BOTTLE HSV CALIBRATION")
    print("========================================")
    print("カメラ映像を見ながら黒ボトルを緑枠の中に置いてください。")
    print("SPACE : 測定開始")
    print("Q     : 終了")
    print("")

    # ========================================
    # ① ボトル位置を合わせる
    # ========================================

    while True:

        ret, frame = cap.read()

        if not ret or frame is None:
            continue

        # ========================================
        # 実際のBottle認識と同じ画像変換
        # 640x480
        # ↓
        # 中央16:9クロップ
        # ↓
        # 320x180
        # ========================================

        fh, fw = frame.shape[:2]

        crop_h = int(fw * 9 / 16)
        y0 = (fh - crop_h) // 2

        frame_169 = frame[
            y0:y0 + crop_h,
            :
        ]

        img_orig = cv2.resize(
            frame_169,
            (FRAME_WIDTH, FRAME_HEIGHT)
        )

        # ========================================
        # ROI位置
        # ========================================

        center_x = FRAME_WIDTH // 2
        center_y = FRAME_HEIGHT // 2

        x1 = center_x - ROI_WIDTH // 2
        x2 = center_x + ROI_WIDTH // 2

        y1 = center_y - ROI_HEIGHT // 2
        y2 = center_y + ROI_HEIGHT // 2

        # ========================================
        # カメラ映像にROIを表示
        # ========================================

        preview = img_orig.copy()

        # 緑色のROI枠
        cv2.rectangle(
            preview,
            (x1, y1),
            (x2, y2),
            (0, 255, 0),
            2
        )

        # 画像中心を赤い点で表示
        cv2.circle(
            preview,
            (center_x, center_y),
            4,
            (0, 0, 255),
            -1
        )

        # 説明表示
        cv2.putText(
            preview,
            "BLACK BOTTLE HSV ROI",
            (5, 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (0, 255, 0),
            1
        )

        cv2.putText(
            preview,
            "SPACE: START   Q: QUIT",
            (5, FRAME_HEIGHT - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (255, 255, 255),
            1
        )

        cv2.imshow(
            "Bottle HSV Calibration",
            preview
        )

        key = cv2.waitKey(1) & 0xFF

        # SPACEキー
        if key == ord(' '):
            print("")
            print("========================================")
            print("START")
            print("========================================")
            break

        # qキー
        if key == ord('q'):
            print("終了します。")
            cap.release()
            cv2.destroyAllWindows()
            return

    # ========================================
    # ② HSV測定開始
    # ========================================

    start = time.time()

    all_h = []
    all_s = []
    all_v = []

    frame_count = 0

    while time.time() - start < MEASURE_SECONDS:

        ret, frame = cap.read()

        if not ret or frame is None:
            continue

        # ========================================
        # 実際のBottle認識と同じ画像変換
        # ========================================

        fh, fw = frame.shape[:2]

        crop_h = int(fw * 9 / 16)
        y0 = (fh - crop_h) // 2

        frame_169 = frame[
            y0:y0 + crop_h,
            :
        ]

        img_orig = cv2.resize(
            frame_169,
            (FRAME_WIDTH, FRAME_HEIGHT)
        )

        # HSV変換
        img_hsv = cv2.cvtColor(
            img_orig,
            cv2.COLOR_BGR2HSV
        )

        # ========================================
        # ROI位置
        # ========================================

        center_x = FRAME_WIDTH // 2
        center_y = FRAME_HEIGHT // 2

        x1 = center_x - ROI_WIDTH // 2
        x2 = center_x + ROI_WIDTH // 2

        y1 = center_y - ROI_HEIGHT // 2
        y2 = center_y + ROI_HEIGHT // 2

        # ========================================
        # ROI取得
        # ========================================

        roi = img_hsv[
            y1:y2,
            x1:x2
        ]

        h = roi[:, :, 0]
        s = roi[:, :, 1]
        v = roi[:, :, 2]

        # ========================================
        # 測定中もROIを表示
        # ========================================

        preview = img_orig.copy()

        cv2.rectangle(
            preview,
            (x1, y1),
            (x2, y2),
            (0, 255, 0),
            2
        )

        cv2.circle(
            preview,
            (center_x, center_y),
            4,
            (0, 0, 255),
            -1
        )

        cv2.putText(
            preview,
            "MEASURING...",
            (5, 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (0, 0, 255),
            1
        )

        cv2.imshow(
            "Bottle HSV Calibration",
            preview
        )

        cv2.waitKey(1)

        # ========================================
        # フレームごとのHSV
        # ========================================

        print(
            "HSV "
            "H(mean=%5.1f med=%5.1f) "
            "S(mean=%5.1f med=%5.1f) "
            "V(mean=%5.1f med=%5.1f)"
            % (
                np.mean(h),
                np.median(h),
                np.mean(s),
                np.median(s),
                np.mean(v),
                np.median(v)
            )
        )

        all_h.extend(h.flatten().tolist())
        all_s.extend(s.flatten().tolist())
        all_v.extend(v.flatten().tolist())

        frame_count += 1

        # 約5回/秒
        time.sleep(0.2)

    # ========================================
    # カメラ終了
    # ========================================

    cap.release()
    cv2.destroyAllWindows()

    # ========================================
    # 測定結果
    # ========================================

    print("")
    print("========================================")
    print(" BLACK BOTTLE HSV RESULT")
    print("========================================")

    if len(all_h) == 0:
        print("HSVデータを取得できませんでした。")
        return

    print("frames =", frame_count)

    print(
        "H: mean=%.1f median=%.1f min=%d "
        "p05=%.1f p25=%.1f p75=%.1f p95=%.1f max=%d"
        % (
            np.mean(all_h),
            np.median(all_h),
            np.min(all_h),
            np.percentile(all_h, 5),
            np.percentile(all_h, 25),
            np.percentile(all_h, 75),
            np.percentile(all_h, 95),
            np.max(all_h)
        )
    )

    print(
        "S: mean=%.1f median=%.1f min=%d "
        "p05=%.1f p25=%.1f p75=%.1f p95=%.1f max=%d"
        % (
            np.mean(all_s),
            np.median(all_s),
            np.min(all_s),
            np.percentile(all_s, 5),
            np.percentile(all_s, 25),
            np.percentile(all_s, 75),
            np.percentile(all_s, 95),
            np.max(all_s)
        )
    )

    print(
        "V: mean=%.1f median=%.1f min=%d "
        "p05=%.1f p25=%.1f p75=%.1f p95=%.1f max=%d"
        % (
            np.mean(all_v),
            np.median(all_v),
            np.min(all_v),
            np.percentile(all_v, 5),
            np.percentile(all_v, 25),
            np.percentile(all_v, 75),
            np.percentile(all_v, 95),
            np.max(all_v)
        )
    )

    print("========================================")
    print("OpenCV HSV: H=0-179, S=0-255, V=0-255")


if __name__ == "__main__":
    main()