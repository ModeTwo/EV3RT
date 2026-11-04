"""Bounded asynchronous raw/detection video and CSV for sumo alignment.

カメラ・制御から受け取る記録を、専用スレッドで動画・CSV・summaryへ保存する。走行判断はしない。"""

import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import queue
import threading
import time
from uuid import uuid4

import cv2
from .sumo_sensor_data import (
    SAMPLE_FIELDS as SENSOR_SAMPLE_FIELDS,
    FRAME_FIELDS as SENSOR_FRAME_FIELDS,
)


VISUAL_FIELDS = (
    "visual_time",
    "visual_stage",
    "visual_reason",
    "visual_power",
    "visual_error_px",
    "visual_predicted_error_px",
    "visual_velocity_deg_s",
    "visual_image_age_sec",
    "visual_reversals",
    "visual_lower_deg",
    "visual_upper_deg",
    "visual_pulses",
    "visual_frame_id",
    "visual_capture_time",
    "visual_capture_gyro_deg",
    "visual_control_gyro_deg",
)

CATCH_FIELDS = (
    "catch_progress_mm",
    "catch_camera_error_px",
    "catch_image_age_sec",
    "catch_track_id",
    "catch_base_power",
    "catch_outcome",
)

ROUTE_FIELDS = (
    "route_x_mm",
    "route_y_mm",
    "route_bearing_deg",
    "route_error_mm",
    "route_capture_assumed",
    "route_sonar_raw",
    "route_kind",
    "route_alignment_delta_deg",
    "route_angle_decision",
    "route_policy_bearing_deg",
    "route_push_start_bearing_deg",
)

FIELDS = (
    ROUTE_FIELDS
    + SENSOR_FRAME_FIELDS
    + CATCH_FIELDS
    + VISUAL_FIELDS
    + (
        "frame_id",
        "capture_session",
        "requested_session",
        "capture_mode",
        "capture_monotonic_sec",
        "elapsed_sec",
        "phase",
        "turns",
        "decision",
        "control_sample_monotonic_sec",
        "odo_mm",
        "gyro_deg",
        "pose_source",
        "observation_available",
        "observation_session",
        "confirmed",
        "candidate",
        "x",
        "y",
        "w",
        "h",
        "cx",
        "cy",
        "area",
        "roi_top",
        "roi_bottom",
        "detector_mode",
        "target_cx",
        "queue_dropped_total",
        "track_id",
        "track_status",
        "mask_policy",
        "mask_source",
        "hsv_candidate_count",
        "value_candidate_count",
        "history_length",
        "association_dx",
        "association_dy",
        "history_expired",
        "detector_started_monotonic_sec",
        "observation_ready_monotonic_sec",
        "detector_duration_ms",
        "capture_to_ready_ms",
        "sampled_angular_velocity_deg_s",
        "phase_entered_monotonic_sec",
        "brake_command_monotonic_sec",
        "selected_frame_id",
        "selected_capture_monotonic_sec",
        "observation_selected_monotonic_sec",
        "selected_observation_age_ms",
        "requested_turn_bearing_deg",
        "requested_turn_step_deg",
        "measured_heading_error_deg",
        "camera_error_px",
        "turn_stage",
        "turn_main_power_setting",
        "test_control_gyro_deg",
        "test_left_count",
        "test_right_count",
        "test_left_command",
        "test_right_command",
    )
)


class SumoAlignRecording:
    """Only this worker owns writers; camera/control never wait for disk IO."""

    def __init__(self, *, output_dir=None, fps=30.0, queue_size=8):
        if output_dir is not None:
            self.output_dir = Path(output_dir)
        else:
            self.output_dir = Path(__file__).resolve().parents[2] / "sumo_align_runs"
        stamp = (
            datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
            + "_"
            + uuid4().hex[:8]
        )
        self.paths = {
            key: self.output_dir / (stamp + suffix)
            for key, suffix in (
                ("raw", "_raw.avi"),
                ("detected", "_detected.avi"),
                ("csv", "_frames.csv"),
                ("summary", "_summary.json"),
            )
        }
        self.fps = fps
        self.packets = queue.Queue(maxsize=queue_size)
        self.ready = threading.Event()
        self.finished = threading.Event()
        self.stopping = threading.Event()
        self.lock = threading.Lock()
        self.error = None
        self.result = "INTERRUPTED"
        self.reason = "recording stopped without an explicit result"
        self.details = None
        self.written = 0
        self.dropped = 0
        self.last_frame_id = -1
        self.thread = threading.Thread(
            target=self._run, name="sumo-align-recording", daemon=True
        )
        self.thread.start()

    def status(self):
        with self.lock:
            return dict(
                ready=self.ready.is_set(),
                finished=self.finished.is_set(),
                error=str(self.error) if self.error is not None else None,
                written=self.written,
                dropped=self.dropped,
                paths={k: str(v) for k, v in self.paths.items()},
            )

    def submit(self, raw, detected, metadata):
        with self.lock:
            if self.stopping.is_set() or self.error is not None:
                return False
            if metadata["frame_id"] <= self.last_frame_id:
                return False
            self.last_frame_id = metadata["frame_id"]
            if self.packets.full():
                self.dropped += 1
                return False
            row = dict(metadata, queue_dropped_total=self.dropped)
            # Copies are made while holding the short queue lock, never disk IO.
            try:
                self.packets.put_nowait((raw.copy(), detected.copy(), row))
            except queue.Full:
                self.dropped += 1
                return False
        return True

    def request_stop(self, result="INTERRUPTED", reason="stopped", *, details=None):
        with self.lock:
            if not self.stopping.is_set():
                self.result, self.reason = result, reason
                self.details = details
                self.stopping.set()

    def close(self, timeout=10.0):
        if not self.stopping.is_set():
            self.request_stop()
        self.thread.join(timeout)
        if self.thread.is_alive():
            raise RuntimeError("recording writer did not finish within %.1fs" % timeout)

    def _run(self):
        raw_writer = detected_writer = stream = None
        first_t = last_t = None
        phases = {}
        raw_size = detected_size = None
        observed = confirmed = 0
        try:
            self.output_dir.mkdir(parents=True, exist_ok=True)
            stream = self.paths["csv"].open("x", newline="", encoding="utf-8")
            writer = csv.DictWriter(stream, fieldnames=FIELDS, extrasaction="ignore")
            writer.writeheader()
            while not self.stopping.is_set() or not self.packets.empty():
                try:
                    raw, detected, row = self.packets.get(timeout=0.1)
                except queue.Empty:
                    continue
                if raw_writer is None:
                    raw_size = (raw.shape[1], raw.shape[0])
                    detected_size = (detected.shape[1], detected.shape[0])
                    fourcc = cv2.VideoWriter_fourcc(*"MJPG")
                    raw_writer = cv2.VideoWriter(
                        str(self.paths["raw"]), fourcc, self.fps, raw_size
                    )
                    detected_writer = cv2.VideoWriter(
                        str(self.paths["detected"]), fourcc, self.fps, detected_size
                    )
                    if not raw_writer.isOpened() or not detected_writer.isOpened():
                        raise RuntimeError("Cannot open MJPG video writers")
                    self.ready.set()
                if (raw.shape[1], raw.shape[0]) != raw_size or (
                    detected.shape[1],
                    detected.shape[0],
                ) != detected_size:
                    raise RuntimeError(
                        "Camera frame dimensions changed during alignment recording"
                    )
                t = row["capture_monotonic_sec"]
                if first_t is None:
                    first_t = t
                last_t = t
                row["elapsed_sec"] = t - first_t
                label = "%s F=%s C=%s %.2fs" % (
                    row.get("phase", "?"),
                    row["frame_id"],
                    row.get("confirmed", 0),
                    row["elapsed_sec"],
                )
                cv2.putText(
                    detected,
                    label,
                    (3, 12),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.33,
                    (0, 255, 255),
                    1,
                )
                if row.get("probe_label") is not None:
                    sonar_label = "SONAR #%s raw=%s mm=%s" % (
                        row.get("probe_sample_id", "-"),
                        str(row.get("probe_raw_value"))[:12],
                        str(row.get("probe_numeric_mm"))[:12],
                    )
                    cv2.putText(
                        detected,
                        sonar_label,
                        (3, 25),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.33,
                        (0, 255, 255),
                        1,
                    )
                    placement = str(row["probe_label"])
                    cv2.putText(
                        detected,
                        placement.encode("ascii", "replace").decode("ascii")[:40],
                        (3, 38),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.33,
                        (0, 255, 255),
                        1,
                    )
                raw_writer.write(raw)
                detected_writer.write(detected)
                writer.writerow(row)
                with self.lock:
                    self.written += 1
                phases[row.get("phase", "?")] = phases.get(row.get("phase", "?"), 0) + 1
                observed += int(bool(row.get("observation_available")))
                confirmed += int(bool(row.get("confirmed")))
                if self.written % 10 == 0:
                    stream.flush()
        except Exception as error:
            with self.lock:
                self.error = error
                if not self.stopping.is_set():
                    self.result, self.reason = "RECORDING_ERROR", str(error)
                    self.stopping.set()
        finally:
            for writer in (raw_writer, detected_writer):
                if writer is not None:
                    try:
                        writer.release()
                    except Exception as error:
                        with self.lock:
                            self.error = self.error or error
            if stream is not None:
                try:
                    stream.close()
                except Exception as error:
                    with self.lock:
                        self.error = self.error or error
            with self.lock:
                summary = dict(
                    result=self.result,
                    reason=self.reason,
                    writer_error=str(self.error) if self.error else None,
                    frames_written=self.written,
                    queue_dropped=self.dropped,
                    nominal_video_fps=self.fps,
                    raw_size=raw_size,
                    detected_size=detected_size,
                    first_capture_monotonic_sec=first_t,
                    last_capture_monotonic_sec=last_t,
                    phase_frames=phases,
                    observation_frames=observed,
                    confirmed_frames=confirmed,
                    paths={k: str(v) for k, v in self.paths.items()},
                )
            if self.details is not None:
                summary["details"] = self.details
            if self.details and self.details.get("kind") == "sumo-sensor-test-v1":
                sensor_path = self.paths["summary"].with_name(
                    self.paths["summary"].name.replace(
                        "_summary.json", "_sensor_samples.csv"
                    )
                )
                try:
                    with sensor_path.open(
                        "x", newline="", encoding="utf-8"
                    ) as sensor_stream:
                        sensor_writer = csv.DictWriter(
                            sensor_stream,
                            fieldnames=SENSOR_SAMPLE_FIELDS,
                            extrasaction="ignore",
                        )
                        sensor_writer.writeheader()
                        sensor_writer.writerows(self.details["sensor_samples"])
                    summary["paths"]["sensor_samples"] = str(sensor_path)
                except Exception as error:
                    with self.lock:
                        self.error = self.error or error
                    summary["writer_error"] = str(self.error)
            if last_t is not None and first_t is not None:
                span = last_t - first_t
            else:
                span = 0
            if span > 0:
                summary["observed_saved_fps"] = (self.written - 1) / span
            else:
                summary["observed_saved_fps"] = None
            try:
                self.paths["summary"].write_text(
                    json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
                )
            except Exception as error:
                with self.lock:
                    self.error = self.error or error
            self.finished.set()
