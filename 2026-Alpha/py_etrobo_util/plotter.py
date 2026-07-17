# 【日本語解説】 Raspberry PiからSPIKEとWebカメラを連携させ、ETロボコン2026の走行・画像認識を制御する。
# 【日本語解説】 行動木の各update()は短時間で1周期だけ処理し、完了まではRUNNINGを返す。
import math
from etrobo_python import ETRobo, Hub, Motor, TouchSensor, ColorSensor, SonarSensor, GyroSensor

TIRE_DIAMETER: float = 55.0
WHEEL_TREAD: float = 110.0
IMU_HEADING_SIGN: float = 1.0

# 【日本語解説】 左右車輪角とジャイロ値から走行距離、方位、二次元位置を推定するクラス。
class Plotter(object):
    # 【日本語解説】 Plotterの設定値と実行中に保持する状態を初期化する。
    def __init__(self) -> None:
        self.running = False
        self.distance = 0.0
        self.loc_x = 0.0
        self.loc_y = 0.0
        self.prev_azimuth = 0.0

    # 【日本語解説】 左右車輪の回転量とジャイロ角から、距離・方位・座標の推定値を更新する。
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
        # 【引数】 hub: SPIKEハブを操作・参照するデバイスオブジェクト。
        # 【引数】 arm_motor: アーム駆動用モーター。
        # 【引数】 right_motor: 右車輪駆動用モーター。
        # 【引数】 left_motor: 左車輪駆動用モーター。
        # 【引数】 touch_sensor: 走行開始などの入力に使うタッチセンサー。
        # 【引数】 color_sensor: 路面のHSV値・反射光値を読むカラーセンサー。
        # 【引数】 sonar_sensor: 前方障害物までの距離を読む超音波センサー。
        # 【引数】 gyro_sensor: 機体の旋回角・角速度を読むジャイロセンサー。
        if not self.running:
            self.running = True
            right_motor.reset_count()
            left_motor.reset_count()
            self.prev_ang_r = right_motor.get_count()
            self.prev_ang_l = left_motor.get_count()
            gyro_sensor.reset()
            return

        # cur_ang_rへ後続処理で使用する計算結果を保存する。
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


        # cur_azimuthへ後続処理で使用する計算結果を保存する。
        cur_azimuth = IMU_HEADING_SIGN * math.radians(gyro_sensor.get_angle())
        cur_azimuth %= (2.0 * math.pi)
 
        # delta_aziへ後続処理で使用する計算結果を保存する。
        delta_azi = cur_azimuth - self.prev_azimuth
        if delta_azi > math.pi:
            delta_azi -= 2.0 * math.pi
        elif delta_azi < -math.pi:
            delta_azi += 2.0 * math.pi
        azi_mid = self.prev_azimuth + delta_azi / 2.0

        self.prev_azimuth = cur_azimuth

        # self.loc_x +へ後続処理で使用する計算結果を保存する。
        self.loc_x += delta_dist * math.sin(azi_mid)
        self.loc_y += delta_dist * math.cos(azi_mid)
        return

    # 【日本語解説】 累積走行距離の推定値を返す。
    def get_distance(self) -> int:
        return int(self.distance)

    # 【日本語解説】 現在の機体方位の推定値を返す。
    def get_azimuth(self) -> int:
        return int(IMU_HEADING_SIGN * math.radians(gyro_sensor.get_angle()) % (2.0 * math.pi))

    # 【日本語解説】 現在の旋回角度の推定値を返す。
    def get_degree(self) -> int:
        degree = int(IMU_HEADING_SIGN * gyro_sensor.get_angle()) % 360
        return degree

    # 【日本語解説】 推定した現在位置のX座標を返す。
    def get_loc_x(self) -> int:
        return int(self.loc_x)

    # 【日本語解説】 推定した現在位置のY座標を返す。
    def get_loc_y(self) -> int:
        return int(self.loc_y)
    
