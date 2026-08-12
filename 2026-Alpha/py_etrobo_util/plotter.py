import math
from etrobo_python import ETRobo, Hub, Motor, TouchSensor, ColorSensor, SonarSensor, GyroSensor

TIRE_DIAMETER: float = 55.0
WHEEL_TREAD: float = 110.0
IMU_HEADING_SIGN: float = 1.0

class Plotter(object):
    def __init__(self) -> None:
        self.running = False
        self.distance = 0.0
        self.loc_x = 0.0
        self.loc_y = 0.0
        self.prev_azimuth = 0.0

    def plot(
        self,
        hub: Hub,
        arm_motor: Motor,
        right_motor: Motor,
        left_motor: Motor,
        touch_sensor: TouchSensor,
        color_sensor: ColorSensor,
        sonar_sensor: SonarSensor,
        gyro_sensor: GyroSensor,
    ) -> None:
        if not self.running:
            self.running = True
            right_motor.reset_count()
            left_motor.reset_count()
            self.prev_ang_r = right_motor.get_count()
            self.prev_ang_l = left_motor.get_count()
            gyro_sensor.reset()
            return

        # --- distance: taken from the wheel encoders ------------------
        # (IMU acceleration is too noisy/biased for double-integrated
        # distance -- a few mm/s^2 of bias becomes meters of error within a
        # second. Wheel encoders remain the right source for this.)
        cur_ang_r = right_motor.get_count()
        cur_ang_l = left_motor.get_count()
        delta_dist_r = math.pi * TIRE_DIAMETER * (cur_ang_r - self.prev_ang_r) / 360.0
        delta_dist_l = math.pi * TIRE_DIAMETER * (cur_ang_l - self.prev_ang_l) / 360.0
        delta_dist = (delta_dist_r + delta_dist_l) / 2.0
        if (delta_dist >= 0.0):
            self.distance += delta_dist
        else:
            self.distance -= delta_dist
        self.prev_ang_r = cur_ang_r
        self.prev_ang_l = cur_ang_l


        # --- azimuth: taken from the IMU heading -------------------------
        # The previous encoder-based azimuth, (delta_dist_l - delta_dist_r) /
        # WHEEL_TREAD, has no absolute reference and drifts whenever a wheel
        # slips. The IMU heading is an absolute measurement and avoids that.
        cur_azimuth = IMU_HEADING_SIGN * math.radians(gyro_sensor.get_angle())
        cur_azimuth %= (2.0 * math.pi)
 
        # Shortest-path delta for the mid-point azimuth, so a wrap-around
        # (e.g. 359deg -> 1deg) doesn't create a spurious large jump.
        delta_azi = cur_azimuth - self.prev_azimuth
        if delta_azi > math.pi:
            delta_azi -= 2.0 * math.pi
        elif delta_azi < -math.pi:
            delta_azi += 2.0 * math.pi
        azi_mid = self.prev_azimuth + delta_azi / 2.0

        self.prev_azimuth = cur_azimuth

        # --- location --------------------------------------------------------
        self.loc_x += delta_dist * math.sin(azi_mid)
        self.loc_y += delta_dist * math.cos(azi_mid)
        return

    def get_distance(self) -> int:
        return int(self.distance)

    def get_azimuth(self) -> int:
        return int(IMU_HEADING_SIGN * math.radians(gyro_sensor.get_angle()) % (2.0 * math.pi))

    def get_degree(self) -> int:
        degree = int(IMU_HEADING_SIGN * gyro_sensor.get_angle()) % 360
        return degree

    def get_loc_x(self) -> int:
        return int(self.loc_x)

    def get_loc_y(self) -> int:
        return int(self.loc_y)
    
