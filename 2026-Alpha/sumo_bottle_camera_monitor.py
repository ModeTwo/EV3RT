"""Preview-only camera monitor for the black-labelled ET sumo bottle."""

import argparse
from dataclasses import dataclass
import os
import platform
import signal
import sys
import threading
import time

# Raspberry Pi上でapt版OpenCVを利用できるよう、既存プログラムと同じ探索先を追加する。
if platform.python_implementation() == "CPython":
    sys.path.append("/usr/lib/python3/dist-packages")
elif platform.python_implementation() == "PyPy":
    sys.path.append("/usr/local/lib/pypy3/dist-packages")

import cv2

from robot_program.vision import SumoBlackBottleConfig, SumoBlackBottleDetector


CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30
WINDOW_NAME = "ET Sumo Black Bottle Monitor"
DISPATCH_INTERVAL_SEC = 0.02
SONAR_MM_PER_UNIT = 1.0


@dataclass(frozen=True)
class SonarSnapshot:
    raw: object = None
    distance_mm: object = None
    updated_at: object = None
    error: object = None


class SonarState:
    # ETRobo制御周期とカメラスレッドの間で、最新の距離値だけを安全に共有する。
    def __init__(self):
        self.lock = threading.Lock()
        self.value = SonarSnapshot()

    def update(self, raw, error=None):
        distance_mm = (
            None
            if raw is None or not isinstance(raw, (int, float)) or raw <= 0
            else float(raw) * SONAR_MM_PER_UNIT
        )
        with self.lock:
            self.value = SonarSnapshot(
                raw=raw,
                distance_mm=distance_mm,
                updated_at=time.monotonic(),
                error=error,
            )

    def snapshot(self):
        with self.lock:
            return self.value


class SonarReader:
    # 読取りだけを行い、モーター出力・リセット・デバイス設定変更は行わない。
    def __init__(self, state):
        self.state = state

    def __call__(self, hub, sonar_sensor):
        try:
            self.state.update(sonar_sensor.get_distance())
        except Exception as error:
            self.state.update(None, "%s: %s" % (error.__class__.__name__, error))


def open_camera(camera_index: int):
    # ロボットの通常画像処理と同じMJPG・640x480・30fpsでカメラを開く。
    capture = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    capture.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
    capture.set(cv2.CAP_PROP_AUTOFOCUS, 1)
    capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
    if not capture.isOpened():
        capture.release()
        raise RuntimeError("Failed to open camera index %d" % camera_index)
    return capture


def format_sonar(snapshot, now=None):
    if snapshot.error is not None:
        return "SONAR ERROR: %s" % snapshot.error, False
    if snapshot.updated_at is None:
        return "SONAR raw=N/A mm=N/A", False
    now = time.monotonic() if now is None else now
    age_ms = max(0.0, (now - snapshot.updated_at) * 1000.0)
    if snapshot.distance_mm is None:
        return "SONAR raw=%s mm=N/A INVALID age=%.0fms" % (
            snapshot.raw,
            age_ms,
        ), False
    accepted = 50.0 <= snapshot.distance_mm <= 800.0
    return "SONAR raw=%s mm=%.1f %s age=%.0fms" % (
        snapshot.raw,
        snapshot.distance_mm,
        "VALID" if accepted else "OUT-OF-RANGE",
        age_ms,
    ), accepted


def draw_preview(
    frame,
    mask,
    detection,
    hits,
    required_hits,
    measured_fps,
    sonar_snapshot,
):
    # 黒領域候補を黄色、連続検出が成立した力士ボトルを緑で囲む。
    confirmed = detection is not None and hits >= required_hits
    if detection is None:
        status = "SEARCHING"
        status_color = (0, 165, 255)
        detail = "No accepted black-label contour"
    else:
        status = "DETECTED" if confirmed else "CANDIDATE"
        status_color = (0, 255, 0) if confirmed else (0, 255, 255)
        detail = "area=%.0f extent=%.2f aspect=%.2f center=%d" % (
            detection.area_px,
            detection.extent,
            detection.aspect_ratio,
            detection.center_x,
        )
        cv2.rectangle(
            frame,
            (detection.x, detection.y),
            (detection.x + detection.width, detection.y + detection.height),
            status_color,
            2,
        )
        cv2.circle(
            frame,
            (detection.center_x, detection.bottom_y),
            4,
            status_color,
            -1,
        )

    cv2.putText(
        frame,
        "%s  frames=%d/%d" % (status, hits, required_hits),
        (8, 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        status_color,
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        detail,
        (8, 44),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.38,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    sonar_text, sonar_valid = format_sonar(sonar_snapshot)
    cv2.putText(
        frame,
        sonar_text,
        (8, 66),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.38,
        (0, 255, 0) if sonar_valid else (0, 165, 255),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        "FPS=%.1f  Q/ESC: exit" % measured_fps,
        (8, frame.shape[0] - 8),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.4,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )

    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    cv2.putText(
        mask_bgr,
        "SUMO BLACK MASK",
        (8, 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return cv2.hconcat((frame, mask_bgr)), confirmed


def run_monitor(args, sonar_state, stop_event) -> int:
    # カメラと距離センサーだけを使用し、モーターやアームへ命令を送らない。
    config = SumoBlackBottleConfig(
        black_max_saturation=args.black_max_saturation,
        black_max_value=args.black_max_value,
        min_area_px=args.min_area,
        min_extent=args.min_extent,
        max_aspect_ratio=args.max_aspect,
    )
    detector = SumoBlackBottleDetector(config)
    capture = open_camera(args.camera_index)
    hits = 0
    previous_state = None
    previous_frame_at = time.monotonic()
    measured_fps = 0.0

    try:
        # 自動露出とフォーカスが安定するまで、判定に使用しない画像を読み捨てる。
        for _ in range(args.warmup_frames):
            capture.read()

        print(
            "Camera opened: actual=%dx%d@%.1f"
            % (
                int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
                int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                capture.get(cv2.CAP_PROP_FPS),
            )
        )
        print("This monitor does not control any motor. Press Q or ESC to exit.")

        consecutive_capture_failures = 0
        while not stop_event.is_set():
            captured, source_frame = capture.read()
            if not captured or source_frame is None:
                consecutive_capture_failures += 1
                if consecutive_capture_failures >= 30:
                    raise RuntimeError("Camera frame capture failed repeatedly")
                continue
            consecutive_capture_failures = 0

            now = time.monotonic()
            elapsed = now - previous_frame_at
            previous_frame_at = now
            if elapsed > 0:
                instant_fps = 1.0 / elapsed
                measured_fps = (
                    instant_fps
                    if measured_fps == 0.0
                    else measured_fps * 0.9 + instant_fps * 0.1
                )

            prepared, mask, detection = detector.detect(source_frame)
            hits = hits + 1 if detection is not None else 0
            preview, confirmed = draw_preview(
                prepared,
                mask,
                detection,
                hits,
                args.confirm_frames,
                measured_fps,
                sonar_state.snapshot(),
            )
            state = "DETECTED" if confirmed else "SEARCHING"
            if state != previous_state:
                print("SUMO_BLACK_BOTTLE=%s" % state)
                previous_state = state

            cv2.imshow(WINDOW_NAME, preview)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
        return 0
    finally:
        # Ctrl+Cやカメラ例外でもプレビューとデバイスを確実に解放する。
        capture.release()
        cv2.destroyAllWindows()


class CameraMonitorThread(threading.Thread):
    def __init__(self, args, sonar_state, stop_event):
        super().__init__(name="sumo-camera-monitor", daemon=True)
        self.args = args
        self.sonar_state = sonar_state
        self.stop_event = stop_event
        self.error = None

    def run(self):
        try:
            run_monitor(self.args, self.sonar_state, self.stop_event)
        except Exception as error:
            self.error = error
        finally:
            # Q終了またはカメラ異常を、メインスレッドのETRobo dispatchへ通知する。
            # 端末側Ctrl+Cですでに終了要求済みなら、SIGINTを重ねて送らない。
            if not self.stop_event.is_set():
                self.stop_event.set()
                os.kill(os.getpid(), signal.SIGINT)


def initialize_etrobo():
    # 距離センサーだけを登録し、モーターは登録も操作もしない。
    from etrobo_python import ETRobo, SonarSensor

    return (
        ETRobo(backend="raspike_art")
        .add_hub("hub")
        .add_device("sonar_sensor", device_type=SonarSensor, port="F")
    )


def bounded_int(minimum, maximum):
    def parse(value):
        parsed = int(value)
        if not minimum <= parsed <= maximum:
            raise argparse.ArgumentTypeError(
                "value must be between %d and %d" % (minimum, maximum)
            )
        return parsed

    return parse


def positive_int(value):
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("value must be greater than zero")
    return parsed


def ratio(value):
    parsed = float(value)
    if not 0.0 < parsed <= 1.0:
        raise argparse.ArgumentTypeError("value must be greater than zero and at most one")
    return parsed


def parse_args():
    parser = argparse.ArgumentParser(
        description="Preview black-label detection for the ET sumo bottle without motion."
    )
    parser.add_argument("--camera-index", type=bounded_int(0, 16), default=0)
    parser.add_argument("--warmup-frames", type=bounded_int(0, 300), default=5)
    parser.add_argument("--confirm-frames", type=positive_int, default=3)
    parser.add_argument("--black-max-saturation", type=bounded_int(0, 255), default=120)
    parser.add_argument("--black-max-value", type=bounded_int(0, 255), default=60)
    parser.add_argument("--min-area", type=float, default=150.0)
    parser.add_argument("--min-extent", type=ratio, default=0.45)
    parser.add_argument("--max-aspect", type=float, default=4.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    sonar_state = SonarState()
    stop_event = threading.Event()
    camera_thread = CameraMonitorThread(args, sonar_state, stop_event)
    camera_thread.start()

    try:
        etrobo = initialize_etrobo()
        etrobo.add_handler(SonarReader(sonar_state))
        etrobo.dispatch(interval=DISPATCH_INTERVAL_SEC)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        camera_thread.join(timeout=3.0)

    if camera_thread.error is not None:
        raise RuntimeError("Camera monitor failed") from camera_thread.error
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
