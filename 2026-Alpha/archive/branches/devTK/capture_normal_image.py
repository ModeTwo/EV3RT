"""Preview the normal camera stream and save images on demand."""

import argparse
from datetime import datetime
from pathlib import Path

# Raspberry Pi用のOpenCV検索パスは、この既存モジュールの読込時に設定される。
# このファイルではcv2を直接importせず、既存Videoモジュールが読み込んだものを使う。
import py_etrobo_util.video as video_module


cv2 = video_module.cv2

DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480
DEFAULT_FPS = 30
DEFAULT_FOURCC = "MJPG"
WINDOW_NAME = "normal camera preview"


def build_output_path(output_dir: Path) -> Path:
    # 連続して撮影しても上書きしないよう、ファイル名へマイクロ秒まで含める。
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    return output_dir / ("normal_mode_%s.jpg" % timestamp)


def open_camera(camera_index: int, width: int, height: int, fps: int):
    # 既存Videoの通常LINEモードと同じMJPG形式でカメラを開く。
    capture = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*DEFAULT_FOURCC))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    capture.set(cv2.CAP_PROP_FPS, fps)
    capture.set(cv2.CAP_PROP_AUTOFOCUS, 1)
    capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)

    if not capture.isOpened():
        capture.release()
        raise RuntimeError("Failed to open camera index %d" % camera_index)

    return capture


def show_capture_preview(
    output_dir: Path,
    camera_index: int,
    width: int,
    height: int,
    fps: int,
    warmup_frames: int,
) -> int:
    # 画像は撮影操作があるまで作成せず、保存先ディレクトリだけ準備する。
    output_dir.mkdir(parents=True, exist_ok=True)
    capture = open_camera(camera_index, width, height, fps)
    saved_count = 0

    try:
        # 自動露出とフォーカスが安定するまで、指定枚数を読み捨てる。
        for _ in range(warmup_frames):
            capture.read()

        actual_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = capture.get(cv2.CAP_PROP_FPS)
        print(
            "Camera opened: requested=%dx%d@%d actual=%dx%d@%.1f"
            % (width, height, fps, actual_width, actual_height, actual_fps)
        )
        if (actual_width, actual_height) != (width, height):
            print("Warning: the camera did not apply the requested resolution")
        print("Press SPACE or S to save an image. Press Q or ESC to exit.")

        consecutive_read_failures = 0
        while True:
            # プレビュー用の最新フレームを継続取得する。
            captured, frame = capture.read()
            if not captured or frame is None:
                consecutive_read_failures += 1
                if consecutive_read_failures >= 30:
                    raise RuntimeError("Camera frame capture failed repeatedly")
                continue
            consecutive_read_failures = 0

            # 保存対象の生画像は変更せず、操作説明はプレビューの複製にだけ描画する。
            preview = frame.copy()
            cv2.putText(
                preview,
                "SPACE/S: save  Q/ESC: exit  saved=%d" % saved_count,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow(WINDOW_NAME, preview)

            # キー入力を1フレームごとに確認し、保存後もプレビューを継続する。
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            if key in (ord("s"), ord("S"), 32):
                output_path = build_output_path(output_dir)
                if not cv2.imwrite(str(output_path), frame):
                    raise RuntimeError("Failed to save image: %s" % output_path)
                saved_count += 1
                print("Image saved: %s" % output_path)

        return saved_count
    finally:
        # Ctrl+Cや例外の場合もカメラとプレビュー画面を確実に解放する。
        capture.release()
        cv2.destroyAllWindows()


def positive_int(value: str) -> int:
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("value must be greater than zero")
    return number


def non_negative_int(value: str) -> int:
    number = int(value)
    if number < 0:
        raise argparse.ArgumentTypeError("value must be zero or greater")
    return number


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Preview the normal camera stream and save multiple images."
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parent / "captures",
        help="Directory for captured JPEG files. Default: ./captures",
    )
    parser.add_argument("--width", type=positive_int, default=DEFAULT_WIDTH)
    parser.add_argument("--height", type=positive_int, default=DEFAULT_HEIGHT)
    parser.add_argument("--fps", type=positive_int, default=DEFAULT_FPS)
    parser.add_argument("--camera-index", type=non_negative_int, default=0)
    parser.add_argument("--warmup-frames", type=non_negative_int, default=5)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    saved_count = show_capture_preview(
        output_dir=args.output_dir.expanduser().resolve(),
        camera_index=args.camera_index,
        width=args.width,
        height=args.height,
        fps=args.fps,
        warmup_frames=args.warmup_frames,
    )
    print("Capture finished: %d image(s) saved" % saved_count)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
