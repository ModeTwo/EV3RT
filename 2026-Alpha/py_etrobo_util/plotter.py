import math
from etrobo_python import ETRobo, Hub, Motor, TouchSensor, ColorSensor, SonarSensor, GyroSensor

# 2026-09-12: 直進100cm/102.9cmの実測テスト(計4回、指令距離に対する誤差は
# +3.5cm/+3.8cm/+4.0cm/+3.9cm。誤差率に直すと+3.500%/+3.693%/+3.887%/+3.790%、
# 平均+3.7175%)から、想定していたタイヤ径が実効径より約3.7%小さいと判明した
# ため55.0→57.05に較正(1段階目)。元の値は55.0(参考用に残す)。
# TIRE_DIAMETER: float = 55.0  # 較正前
# TIRE_DIAMETER: float = 57.05  # 1段階目の較正値(55.0 * 1.037175)
#
# 57.05較正後に102.9cmで再実測(3回、+8mm/+5mm/+5mm、誤差率+0.78%/+0.49%/+0.49%)
# したところ、まだ僅かに過走行が残っていたため、多数派だった+0.49%を採用して
# 追加補正(2段階目、57.05 * 1.0049)。
TIRE_DIAMETER: float = 56.87
# WHEEL_TREAD: 2026-09-12に実測(左右タイヤの接地面中心間の距離、11.7cm)。
# et_rally_planner側でのその場旋回の位置ズレ調査(SpinAroundByEncoder、
# sample_comment.py参照)に使う。
WHEEL_TREAD: float = 117.0
IMU_HEADING_SIGN: float = 1.0

# 2026-09-14: 同方向に90度を28回連続で回すテストで、フェーズ1・フェーズ2とも
# 毎回ジャイロの目標値(28回目は2520度=0度相当)にぴったり収束しているにも
# 関わらず(ログの"encoder-spin ended"が全28回とも目標と完全一致、
# sample_comment.pyのSpinAroundByEncoder参照)、実際の物理的な角度は約16度
# ズレていた。つまり制御ロジックではなく、ジャイロセンサー自体が実際の回転量を
# 過少に報告している(較正されていない)ことが原因と判明。
# 2520度(ジャイロの申告値)に対し、実際は約2520+16=2536度回っていた計算になる
# ので、GYRO_SCALE_FACTOR = 2536/2520 ≈ 1.00635。get_angle()の生値に
# この係数を掛けたものを「真の回転角度の推定値」として使う。
# タイヤ径較正と同じく、実測1回ぶんの暫定値なので、テストを重ねて精緻化すること。
GYRO_SCALE_FACTOR: float = 1.00635

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
        cur_azimuth = IMU_HEADING_SIGN * math.radians(gyro_sensor.get_angle() * GYRO_SCALE_FACTOR)
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
    
