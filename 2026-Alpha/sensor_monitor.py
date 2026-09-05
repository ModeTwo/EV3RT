"""Display raw robot sensor values continuously for hardware adjustment."""

import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path

from etrobo_python import (
    ColorSensor,
    ETRobo,
    GyroSensor,
    Hub,
    Motor,
    SonarSensor,
    TouchSensor,
)

from py_etrobo_util import ColorClassifier


DEFAULT_DISPATCH_INTERVAL_SEC = 0.02
DEFAULT_DISPLAY_INTERVAL_SEC = 0.10
SONAR_MM_PER_UNIT = 1.0


class SensorMonitor:
    # ETRoboの周期呼出しを受け、指定した表示周期ごとに全センサーを読み取る。
    # モーター出力、エンコーダーリセット、ジャイロリセットは一切実施しない。
    def __init__(self, display_interval_sec, csv_path=None, clear_screen=True):
        self.display_interval_sec = float(display_interval_sec)
        self.csv_path = Path(csv_path) if csv_path else None
        self.clear_screen = clear_screen
        self.started_at = None
        self.last_display_at = None
        self.sample_number = 0
        self.color_classifier = ColorClassifier()
        self.csv_file = None
        self.csv_writer = None

    def _open_csv(self):
        # CSV指定時だけ保存先を作成し、画面と同じ計測値を記録する。
        if self.csv_path is None or self.csv_file is not None:
            return
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = self.csv_path.open("w", encoding="utf-8", newline="")
        self.csv_writer = csv.DictWriter(
            self.csv_file,
            fieldnames=[
                "timestamp",
                "elapsed_sec",
                "sample_number",
                "touch_pressed",
                "color_h",
                "color_s",
                "color_v",
                "classified_color",
                "sonar_raw",
                "sonar_mm",
                "gyro_deg",
                "arm_motor_deg",
                "right_motor_deg",
                "left_motor_deg",
                "imu_stationary",
            ],
        )
        self.csv_writer.writeheader()
        self.csv_file.flush()

    @staticmethod
    def _read_value(reader):
        # 一つのデバイス読取り失敗で、他センサーの表示まで停止させない。
        try:
            return reader(), None
        except Exception as exc:
            return None, "%s: %s" % (exc.__class__.__name__, exc)

    def _collect(
        self,
        hub,
        arm_motor,
        right_motor,
        left_motor,
        touch_sensor,
        color_sensor,
        sonar_sensor,
        gyro_sensor,
        now,
    ):
        errors = []

        touch_pressed, error = self._read_value(touch_sensor.is_pressed)
        if error:
            errors.append("touch=" + error)

        color_hsv, error = self._read_value(color_sensor.get_raw_color_hsv)
        if error:
            errors.append("color=" + error)
            color_h, color_s, color_v = None, None, None
            classified_color = None
        else:
            color_h, color_s, color_v = color_hsv
            classified_color, classify_error = self._read_value(
                lambda: self.color_classifier.classify(color_h, color_s, color_v)
            )
            if classify_error:
                errors.append("classification=" + classify_error)
                classified_color = None

        sonar_raw, error = self._read_value(sonar_sensor.get_distance)
        if error:
            errors.append("sonar=" + error)
        sonar_mm = None if sonar_raw is None else float(sonar_raw) * SONAR_MM_PER_UNIT

        gyro_deg, error = self._read_value(gyro_sensor.get_angle)
        if error:
            errors.append("gyro=" + error)

        arm_motor_deg, error = self._read_value(arm_motor.get_count)
        if error:
            errors.append("arm_motor=" + error)

        right_motor_deg, error = self._read_value(right_motor.get_count)
        if error:
            errors.append("right_motor=" + error)

        left_motor_deg, error = self._read_value(left_motor.get_count)
        if error:
            errors.append("left_motor=" + error)

        imu_stationary, error = self._read_value(hub.hub_imu_is_stationary)
        if error:
            errors.append("imu=" + error)

        color_name = None
        if classified_color is not None:
            color_name = getattr(classified_color, "value", str(classified_color))

        return {
            "timestamp": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            "elapsed_sec": now - self.started_at,
            "sample_number": self.sample_number,
            "touch_pressed": touch_pressed,
            "color_h": color_h,
            "color_s": color_s,
            "color_v": color_v,
            "classified_color": color_name,
            "sonar_raw": sonar_raw,
            "sonar_mm": sonar_mm,
            "gyro_deg": gyro_deg,
            "arm_motor_deg": arm_motor_deg,
            "right_motor_deg": right_motor_deg,
            "left_motor_deg": left_motor_deg,
            "imu_stationary": imu_stationary,
            "errors": errors,
        }

    @staticmethod
    def _format_value(value, decimals=None):
        if value is None:
            return "N/A"
        if decimals is not None:
            return ("%%.%df" % decimals) % value
        return str(value)

    def _render(self, sample):
        # ANSIエスケープで表示位置を先頭へ戻し、値だけが更新される画面にする。
        lines = [
            "ETRC 2026 Sensor Monitor",
            "Press Ctrl+C to stop.",
            "",
            "Elapsed             : %s sec" % self._format_value(sample["elapsed_sec"], 2),
            "Sample              : %s" % sample["sample_number"],
            "Touch pressed       : %s" % self._format_value(sample["touch_pressed"]),
            "Color HSV           : H=%s  S=%s  V=%s"
            % (
                self._format_value(sample["color_h"]),
                self._format_value(sample["color_s"]),
                self._format_value(sample["color_v"]),
            ),
            "Classified color    : %s" % self._format_value(sample["classified_color"]),
            "Sonar raw           : %s" % self._format_value(sample["sonar_raw"]),
            "Sonar converted     : %s mm" % self._format_value(sample["sonar_mm"], 1),
            "Gyro                : %s degree" % self._format_value(sample["gyro_deg"]),
            "Arm motor encoder   : %s degree" % self._format_value(sample["arm_motor_deg"]),
            "Right motor encoder : %s degree" % self._format_value(sample["right_motor_deg"]),
            "Left motor encoder  : %s degree" % self._format_value(sample["left_motor_deg"]),
            "IMU stationary      : %s" % self._format_value(sample["imu_stationary"]),
        ]
        if self.csv_path is not None:
            lines.append("CSV output          : %s" % self.csv_path)
        if sample["errors"]:
            lines.extend(["", "Read errors:", *["- " + item for item in sample["errors"]]])

        prefix = "\033[2J\033[H" if self.clear_screen else ""
        sys.stdout.write(prefix + "\n".join(lines) + "\n")
        sys.stdout.flush()

    def __call__(
        self,
        hub: Hub,
        arm_motor: Motor,
        right_motor: Motor,
        left_motor: Motor,
        touch_sensor: TouchSensor,
        color_sensor: ColorSensor,
        sonar_sensor: SonarSensor,
        gyro_sensor: GyroSensor,
    ):
        now = time.monotonic()
        if self.started_at is None:
            self.started_at = now
            self._open_csv()

        # ETRobo自体は20ms周期で動かし、端末更新とCSV保存は指定周期へ間引く。
        if (
            self.last_display_at is not None
            and now - self.last_display_at < self.display_interval_sec
        ):
            return

        self.last_display_at = now
        self.sample_number += 1
        sample = self._collect(
            hub,
            arm_motor,
            right_motor,
            left_motor,
            touch_sensor,
            color_sensor,
            sonar_sensor,
            gyro_sensor,
            now,
        )
        self._render(sample)

        if self.csv_writer is not None:
            csv_sample = {key: value for key, value in sample.items() if key != "errors"}
            self.csv_writer.writerow(csv_sample)
            self.csv_file.flush()

    def close(self):
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None


def initialize_etrobo(backend):
    # alpha.pyと同じポート割当てを使用し、計測結果と競技プログラムの条件を揃える。
    return (
        ETRobo(backend=backend)
        .add_hub("hub")
        .add_device("arm_motor", device_type=Motor, port="C")
        .add_device("right_motor", device_type=Motor, port="A")
        .add_device("left_motor", device_type=Motor, port="B")
        .add_device("touch_sensor", device_type=TouchSensor, port="D")
        .add_device("color_sensor", device_type=ColorSensor, port="E")
        .add_device("sonar_sensor", device_type=SonarSensor, port="F")
        .add_device("gyro_sensor", device_type=GyroSensor, port="")
    )


def parse_args():
    parser = argparse.ArgumentParser(description="Continuously display robot sensor values.")
    parser.add_argument(
        "--interval",
        type=float,
        default=DEFAULT_DISPLAY_INTERVAL_SEC,
        help="Display and CSV sampling interval in seconds (default: 0.10)",
    )
    parser.add_argument(
        "--csv",
        type=str,
        default=None,
        help="Optional CSV output path",
    )
    parser.add_argument(
        "--no-clear",
        action="store_true",
        help="Print every sample without clearing the terminal",
    )
    args = parser.parse_args()
    if args.interval <= 0:
        parser.error("--interval must be greater than zero")
    return args


def main():
    args = parse_args()
    monitor = SensorMonitor(
        display_interval_sec=args.interval,
        csv_path=args.csv,
        clear_screen=not args.no_clear,
    )
    try:
        etrobo = initialize_etrobo(backend="raspike_art")
        etrobo.add_handler(monitor)
        etrobo.dispatch(interval=DEFAULT_DISPATCH_INTERVAL_SEC)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.close()
        print("Sensor monitor stopped.")


if __name__ == "__main__":
    main()
