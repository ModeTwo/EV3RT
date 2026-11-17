import math

from ..types import HeadingType
from .gyro_drive import RunByGyro


class ArcTurn(RunByGyro):
    """指定した半径で指定角度だけ円弧旋回する。"""

    def __init__(
        self,
        name,
        angle,
        radius,
        power,
        pid_p,
        pid_i,
        pid_d,
        wheel_tread_mm=95.0,
        feedforward_gain=1.0,
    ):
        if not math.isfinite(angle) or angle == 0:
            raise ValueError("angle must be finite and non-zero")
        if not math.isfinite(radius) or radius <= 0:
            raise ValueError("radius must be positive and finite")

        self.angle = float(angle)
        self.radius = float(radius)

        # 円弧の走行距離
        self.arc_distance = abs(
            math.radians(self.angle) * self.radius
        )

        # 開始時の絶対方位を基準に目標角度を算出
        def heading_at(distance_mm):
            progress = max(
                0.0,
                min(float(distance_mm), self.arc_distance)
            )

            return self.heading_origin + math.copysign(
                math.degrees(progress / self.radius),
                self.angle,
            )

        super().__init__(
            name=name,
            target=heading_at,
            power=power,
            pid_p=pid_p,
            pid_i=pid_i,
            pid_d=pid_d,
            target_type=HeadingType.ABSOLUTE,
            distance_limit_mm=self.arc_distance,
            feedforward_gain=feedforward_gain,
            wheel_tread_mm=wheel_tread_mm,
        )