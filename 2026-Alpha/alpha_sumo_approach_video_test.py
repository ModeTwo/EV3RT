#!/usr/bin/env python3
"""ET sumo: record detector video while driving a fixed gyro-held distance.

This is a standalone test entry point modeled on 2026-Alpha's device setup,
touch start, Plotter odometry and 20 ms handler loop. It does not push a bottle.
"""

import argparse
import csv
from datetime import datetime
import math
from pathlib import Path
import platform
import signal
import sys
import threading
import time

if platform.python_implementation() == "CPython":
    sys.path.append("/usr/lib/python3/dist-packages")
elif platform.python_implementation() == "PyPy":
    sys.path.append("/usr/local/lib/pypy3/dist-packages")

import cv2
import numpy as np
from etrobo_python import ETRobo, GyroSensor, Hub, Motor, TouchSensor
from simple_pid import PID

from py_etrobo_util.plotter import Plotter


CONTROL_INTERVAL_SEC = 0.02
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30.0
FRAME_WIDTH = 320
FRAME_HEIGHT = 180

# Keep these detector settings aligned with py_etrobo_util/video.py's
# BottleColor.BLACK branch. Update both when the production detector changes.
BLACK_MAX_SATURATION = 120
BLACK_MAX_VALUE = 60
SEARCH_TOP_RATIO = 0.30
MIN_AREA_PX = 150.0
MIN_EXTENT = 0.45
MAX_ASPECT_RATIO = 4.0
MAX_WIDTH_RATIO = 0.55
HORIZON_DISTANCE_MM = 270.0
AXLE_TO_HORIZON_DISTANCE_MM = 230.0
BLIND_ROW = FRAME_HEIGHT - 4
DRIVE_PID_P = 1.1
DRIVE_PID_I = 0.1
DRIVE_PID_D = 0.03


def prepare_and_detect(frame, kernel):
    """Return 320x180 detector image, mask, and best black contour."""
    frame_h, frame_w = frame.shape[:2]
    crop_h = min(frame_h, int(frame_w * 9 / 16))
    crop_top = max(0, (frame_h - crop_h) // 2)
    crop = frame[crop_top:crop_top + crop_h, :]
    prepared = cv2.resize(
        crop, (FRAME_WIDTH, FRAME_HEIGHT), interpolation=cv2.INTER_AREA
    )
    hsv = cv2.cvtColor(prepared, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.array((0, 0, 0), dtype=np.uint8),
        np.array((179, BLACK_MAX_SATURATION, BLACK_MAX_VALUE), dtype=np.uint8),
    )
    mask[:int(FRAME_HEIGHT * SEARCH_TOP_RATIO), :] = 0
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    best = None
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < MIN_AREA_PX:
            continue
        x, y, width, height = cv2.boundingRect(contour)
        if height <= 0 or width > FRAME_WIDTH * MAX_WIDTH_RATIO:
            continue
        extent = area / float(width * height)
        aspect = width / float(height)
        if aspect > MAX_ASPECT_RATIO or extent < MIN_EXTENT:
            continue
        if best is None or area > best["area"]:
            cx = x + width // 2
            bottom = y + height
            vxp = cx - FRAME_WIDTH // 2
            vxm = vxp * HORIZON_DISTANCE_MM / FRAME_WIDTH
            theta = math.degrees(math.atan(vxm / AXLE_TO_HORIZON_DISTANCE_MM))
            best = {
                "x": x, "y": y, "width": width, "height": height,
                "cx": cx, "bottom": bottom, "theta": theta,
                "area": area, "extent": extent, "aspect": aspect,
                "blind": bottom >= BLIND_ROW,
            }
    return prepared, mask, best, crop_top, crop_h


class ApproachCaptureWorker(threading.Thread):
    """Own the camera, annotate detections, write AVI and per-frame CSV."""

    def __init__(self, camera, output_dir, run_stamp):
        super().__init__(name="sumo-approach-camera", daemon=True)
        self.camera = camera
        self.output_dir = output_dir
        self.run_stamp = run_stamp
        self.stop_event = threading.Event()
        self.record_requested = threading.Event()
        self.record_ready = threading.Event()
        self.record_closed = threading.Event()
        self.state_lock = threading.Lock()
        self.state = {
            "phase": "WAIT_TOUCH", "distance_mm": 0.0,
            "gyro_deg": 0.0, "target_heading_deg": 0.0,
            "left_power": 0, "right_power": 0,
        }
        self.error = None
        self.frames_written = 0
        self.video_path = output_dir / (run_stamp + "_approach.avi")
        self.csv_path = output_dir / (run_stamp + "_frames.csv")

    def update_state(self, **values):
        with self.state_lock:
            self.state.update(values)

    def start_recording(self):
        self.record_closed.clear()
        self.record_ready.clear()
        self.record_started_at = None
        self.record_requested.set()

    def stop_recording(self):
        self.record_requested.clear()

    def stop(self):
        self.stop_recording()
        self.stop_event.set()

    def run(self):
        writer = None
        csv_file = None
        csv_writer = None
        kernel = np.ones((7, 7), dtype=np.uint8)
        frame_id = 0
        failed_reads = 0
        try:
            while not self.stop_event.is_set():
                ok, frame = self.camera.read()
                captured_at = time.monotonic()
                if not ok or frame is None:
                    failed_reads += 1
                    if failed_reads >= 10:
                        raise RuntimeError("camera read failed 10 consecutive times")
                    time.sleep(0.02)
                    continue
                failed_reads = 0
                frame_id += 1
                if not self.record_requested.is_set():
                    if writer is not None:
                        writer.release()
                        writer = None
                        if csv_file is not None:
                            csv_file.flush()
                            csv_file.close()
                            csv_file = None
                        self.record_closed.set()
                    continue

                prepared, _mask, detection, crop_top, crop_h = prepare_and_detect(
                    frame, kernel
                )

                if self.record_requested.is_set() and writer is None:
                    fourcc = cv2.VideoWriter_fourcc(*"MJPG")
                    writer = cv2.VideoWriter(
                        str(self.video_path), fourcc, CAMERA_FPS,
                        (frame.shape[1], frame.shape[0]),
                    )
                    if not writer.isOpened():
                        writer.release()
                        writer = None
                        raise RuntimeError("could not create MJPG AVI writer")
                    self.record_started_at = captured_at
                    csv_file = self.csv_path.open(
                        "w", newline="", encoding="utf-8"
                    )
                    csv_writer = csv.writer(csv_file)
                    csv_writer.writerow([
                        "frame_id", "elapsed_sec", "odo_mm", "gyro_deg",
                        "target_heading_deg", "left_power", "right_power",
                        "detected", "cx_320", "theta_deg", "bottom_row_180",
                        "area_px", "extent", "aspect", "in_blind_region",
                    ])
                    csv_file.flush()
                    self.record_ready.set()

                if writer is not None and self.record_requested.is_set():
                    with self.state_lock:
                        state = dict(self.state)
                    elapsed = captured_at - getattr(self, "record_started_at", captured_at)
                    view = frame.copy()
                    label = "NO BLACK CANDIDATE"
                    if detection is not None:
                        sx = frame.shape[1] / float(FRAME_WIDTH)
                        sy = crop_h / float(FRAME_HEIGHT)
                        x1 = int(detection["x"] * sx)
                        x2 = int((detection["x"] + detection["width"]) * sx)
                        y1 = crop_top + int(detection["y"] * sy)
                        y2 = crop_top + int(
                            (detection["y"] + detection["height"]) * sy
                        )
                        cv2.rectangle(view, (x1, y1), (x2, y2), (0, 220, 255), 3)
                        label = "BLACK theta=%+.1f area=%.0f" % (
                            detection["theta"], detection["area"]
                        )
                        csv_writer.writerow([
                            frame_id, "%.4f" % elapsed,
                            "%.1f" % state["distance_mm"],
                            "%.2f" % state["gyro_deg"],
                            "%.2f" % state["target_heading_deg"],
                            state["left_power"], state["right_power"],
                            1, detection["cx"], "%.2f" % detection["theta"],
                            detection["bottom"], "%.1f" % detection["area"],
                            "%.3f" % detection["extent"],
                            "%.3f" % detection["aspect"],
                            int(detection["blind"]),
                        ])
                    else:
                        csv_writer.writerow([
                            frame_id, "%.4f" % elapsed,
                            "%.1f" % state["distance_mm"],
                            "%.2f" % state["gyro_deg"],
                            "%.2f" % state["target_heading_deg"],
                            state["left_power"], state["right_power"],
                            0, "", "", "", "", "", "", 0,
                        ])
                    cv2.putText(
                        view,
                        "FID=%d odo=%.0fmm gyro=%.1f L/R=%d/%d"
                        % (frame_id, state["distance_mm"], state["gyro_deg"],
                           state["left_power"], state["right_power"]),
                        (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (255, 255, 255), 2, cv2.LINE_AA,
                    )
                    cv2.putText(
                        view, label, (8, 48), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (0, 220, 255), 2, cv2.LINE_AA,
                    )
                    writer.write(view)
                    self.frames_written += 1
                    if self.frames_written % 10 == 0:
                        csv_file.flush()

        except Exception as error:
            self.error = error
        finally:
            if writer is not None:
                writer.release()
            if csv_file is not None:
                csv_file.flush()
                csv_file.close()
            self.record_closed.set()


class StraightApproachHandler:
    """Touch-start, gyro-held straight drive, stop at encoder distance."""

    WAIT_STATIONARY = "WAIT_STATIONARY"
    WAIT_TOUCH = "WAIT_TOUCH"
    RECORDER_STARTING = "RECORDER_STARTING"
    RUNNING = "RUNNING"
    FINALIZING = "FINALIZING"
    COMPLETE = "COMPLETE"

    def __init__(self, args, worker):
        self.args = args
        self.worker = worker
        self.initialized = False
        self.stationary_samples = 0
        self.plotter = None
        self.phase = self.WAIT_STATIONARY
        self.origin_mm = None
        self.target_heading = None
        self.pid = None
        self.left_power = 0
        self.right_power = 0
        self.completed_reported = False
        self.right_motor = None
        self.left_motor = None

    @staticmethod
    def stop_motors(right_motor, left_motor):
        for motor in (right_motor, left_motor):
            motor.set_power(0)
            motor.set_brake(True)

    def __call__(self, hub: Hub, right_motor: Motor, left_motor: Motor,
                 touch_sensor: TouchSensor, gyro_sensor: GyroSensor):
        self.right_motor = right_motor
        self.left_motor = left_motor
        if not self.initialized:
            self.plotter = Plotter()
            self.initialized = True
            self.worker.update_state(phase=self.phase)
            self.stop_motors(right_motor, left_motor)
            return

        if self.phase == self.WAIT_STATIONARY:
            self.stop_motors(right_motor, left_motor)
            if hub.hub_imu_is_stationary():
                self.stationary_samples += 1
            else:
                self.stationary_samples = 0
            if self.stationary_samples >= 4:
                # Match alpha.py's ResetDevice: wait for stationarity, then
                # let Plotter's first update reset the encoders and gyro.
                self.plotter.plot(
                    hub, None, right_motor, left_motor, touch_sensor,
                    None, None, gyro_sensor,
                )
                self.phase = self.WAIT_TOUCH
                print("-- IMU stable; press touch sensor to start recording/run",
                      flush=True)
            self.worker.update_state(phase=self.phase,
                                     gyro_deg=float(gyro_sensor.get_angle()))
            return

        self.plotter.plot(
            hub, None, right_motor, left_motor, touch_sensor,
            None, None, gyro_sensor,
        )
        if self.worker.error is not None:
            self.stop_motors(right_motor, left_motor)
            raise RuntimeError("camera/recording failed; motors stopped") from self.worker.error

        raw_gyro = float(gyro_sensor.get_angle())
        heading = -self.args.course_sign * raw_gyro
        distance = float(self.plotter.get_distance())

        if self.phase == self.WAIT_TOUCH:
            self.stop_motors(right_motor, left_motor)
            self.worker.update_state(
                phase=self.phase, distance_mm=distance, gyro_deg=raw_gyro,
                left_power=0, right_power=0,
            )
            if touch_sensor.is_pressed():
                self.target_heading = heading
                self.origin_mm = distance
                self.worker.update_state(
                    target_heading_deg=heading,
                    distance_mm=0.0,
                )
                self.worker.start_recording()
                self.phase = self.RECORDER_STARTING
                self.worker.update_state(phase=self.phase)
                print("-- touch start; waiting for video writer", flush=True)
            return

        if self.phase == self.RECORDER_STARTING:
            self.stop_motors(right_motor, left_motor)
            self.worker.update_state(
                phase=self.phase, distance_mm=distance, gyro_deg=raw_gyro,
                left_power=0, right_power=0,
            )
            if self.worker.record_ready.is_set():
                self.origin_mm = distance
                self.target_heading = heading
                turn_limit = min(self.args.power, 100 - self.args.power)
                self.pid = PID(
                    DRIVE_PID_P, DRIVE_PID_I, DRIVE_PID_D,
                    setpoint=self.target_heading,
                    sample_time=CONTROL_INTERVAL_SEC,
                    output_limits=(-turn_limit, turn_limit),
                )
                self.phase = self.RUNNING
                print("-- run start distance=%.1fmm power=%d heading=%.2f"
                      % (self.args.distance_mm, self.args.power,
                         self.target_heading), flush=True)
            return

        if self.phase == self.RUNNING:
            progress = max(0.0, distance - self.origin_mm)
            if progress >= self.args.distance_mm:
                self.stop_motors(right_motor, left_motor)
                self.left_power = self.right_power = 0
                self.phase = self.FINALIZING
                self.worker.stop_recording()
                self.worker.update_state(
                    phase=self.phase, distance_mm=progress,
                    gyro_deg=raw_gyro, left_power=0, right_power=0,
                )
                print("-- target distance reached: %.1fmm; stopping and finalizing video"
                      % progress, flush=True)
                return

            turn = int(self.pid(heading))
            self.right_power = max(0, min(100,
                self.args.power + self.args.course_sign * turn))
            self.left_power = max(0, min(100,
                self.args.power - self.args.course_sign * turn))
            right_motor.set_brake(False)
            left_motor.set_brake(False)
            right_motor.set_power(self.right_power)
            left_motor.set_power(self.left_power)
            self.worker.update_state(
                phase=self.phase, distance_mm=progress,
                gyro_deg=raw_gyro, target_heading_deg=self.target_heading,
                left_power=self.left_power, right_power=self.right_power,
            )
            return

        if self.phase == self.FINALIZING:
            self.stop_motors(right_motor, left_motor)
            if self.worker.record_closed.is_set():
                self.phase = self.COMPLETE
                self.worker.update_state(phase=self.phase)
                if not self.completed_reported:
                    self.completed_reported = True
                    print("-- capture complete frames=%d\n-- video=%s\n-- csv=%s\n"
                          "-- press Ctrl+C to exit safely"
                          % (self.worker.frames_written, self.worker.video_path,
                             self.worker.csv_path), flush=True)
            return

        if self.phase == self.COMPLETE:
            self.stop_motors(right_motor, left_motor)


def open_camera():
    backend = cv2.CAP_V4L2 if hasattr(cv2, "CAP_V4L2") else cv2.CAP_ANY
    camera = cv2.VideoCapture(0, backend)
    if not camera.isOpened():
        raise RuntimeError("could not open camera index 0")
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    camera.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
    camera.set(cv2.CAP_PROP_AUTOFOCUS, 1)
    ok, frame = camera.read()
    if not ok or frame is None:
        camera.release()
        raise RuntimeError("camera opened but returned no initial frame")
    return camera


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("course", choices=("right", "left"),
                        help="same course-side sign convention as alpha.py")
    parser.add_argument("--distance-mm", type=float, default=500.0,
                        help="encoder odometry distance (default: 500 mm)")
    parser.add_argument("--power", type=int, default=80,
                        help="base motor power 1..100 (default: 80)")
    parser.add_argument("--output-dir", type=Path,
                        default=Path(__file__).resolve().parent / "sumo_approach_video_runs",
                        help="directory for AVI and per-frame CSV")
    args = parser.parse_args(argv)
    if not math.isfinite(args.distance_mm) or args.distance_mm <= 0:
        parser.error("--distance-mm must be a positive finite number")
    if not 1 <= args.power <= 100:
        parser.error("--power must be an integer from 1 to 100")
    args.course_sign = -1 if args.course == "right" else 1

    args.output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    camera = open_camera()
    worker = ApproachCaptureWorker(camera, args.output_dir, stamp)
    handler = StraightApproachHandler(args, worker)
    etrobo = (
        ETRobo(backend="raspike_art")
        .add_hub("hub")
        .add_device("right_motor", device_type=Motor, port="A")
        .add_device("left_motor", device_type=Motor, port="B")
        .add_device("touch_sensor", device_type=TouchSensor, port="D")
        .add_device("gyro_sensor", device_type=GyroSensor, port="")
        .add_handler(handler)
    )
    print("-- straight video test: course=%s distance=%.1fmm power=%d"
          % (args.course, args.distance_mm, args.power), flush=True)
    worker_started = False
    try:
        worker.start()
        worker_started = True
        etrobo.dispatch(interval=CONTROL_INTERVAL_SEC)
    finally:
        if handler.right_motor is not None and handler.left_motor is not None:
            try:
                handler.stop_motors(handler.right_motor, handler.left_motor)
            except Exception as error:
                print("-- motor stop during cleanup failed: %s" % error,
                      file=sys.stderr, flush=True)
        worker.stop()
        if worker_started:
            worker.join(timeout=3.0)
        camera.release()


if __name__ == "__main__":
    main()
