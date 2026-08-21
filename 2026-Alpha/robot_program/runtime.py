"""Runtime references used by reusable robot behaviors."""

from typing import Any


class RobotRuntime:
    # 実機デバイスはETRoboのhandler起動時に設定し、各Behaviorから同じ参照を利用する。
    def __init__(self) -> None:
        self.hub: Any = None
        self.arm_motor: Any = None
        self.right_motor: Any = None
        self.left_motor: Any = None
        self.touch_sensor: Any = None
        self.color_sensor: Any = None
        self.sonar_sensor: Any = None
        self.gyro_sensor: Any = None
        self.plotter: Any = None
        self.video: Any = None
        self.course: int = 0

    def configure(
        self,
        hub,
        arm_motor,
        right_motor,
        left_motor,
        touch_sensor,
        color_sensor,
        sonar_sensor,
        gyro_sensor,
        plotter,
        video,
        course,
    ) -> None:
        # 実機側から渡された参照を一度だけ保存する。
        self.hub = hub
        self.arm_motor = arm_motor
        self.right_motor = right_motor
        self.left_motor = left_motor
        self.touch_sensor = touch_sensor
        self.color_sensor = color_sensor
        self.sonar_sensor = sonar_sensor
        self.gyro_sensor = gyro_sensor
        self.plotter = plotter
        self.video = video
        self.course = course

    def require(self, *names: str) -> None:
        # 初期化漏れを不明瞭なAttributeErrorではなく、明示的なエラーとして検出する。
        missing = [name for name in names if getattr(self, name) is None]
        if missing:
            raise RuntimeError("Robot runtime is not configured: " + ", ".join(missing))


runtime = RobotRuntime()
