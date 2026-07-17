# 【日本語解説】 Raspberry PiからSPIKEとWebカメラを連携させ、ETロボコン2026の走行・画像認識を制御する。
# 【日本語解説】 行動木の各update()は短時間で1周期だけ処理し、完了まではRUNNINGを返す。
import sys
import argparse
import time
import threading
import signal
import math
from enum import IntEnum, Enum, auto
from etrobo_python import ETRobo, Hub, Motor, TouchSensor, ColorSensor, SonarSensor, GyroSensor
from simple_pid import PID
from py_trees.trees import BehaviourTree
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees.composites import Selector
from py_trees.composites import Parallel
from py_trees.common import ParallelPolicy
from py_trees import (
    display as display_tree,
    logging as log_tree
)
from py_etrobo_util import Video, TraceSide, TargetInterested, Plotter, SymmetricClamper, Color, ColorClassifier, LowPassFilter, BottleColor, Hint, HintType

# 実行周期を定義する定数
EXEC_INTERVAL: float  = 0.02
VIDEO_INTERVAL: float = 0.02

# 行動木の定義で使用する定数
SPIN_MAX_POWER     = 57
SPIN_MIN_POWER     = 47
TRACELINE_TARGET_V = 75

# 各行動クラス固有の定数
GS_MIN_DEFAULT     = 0
GS_MAX_DEFAULT     = 55
ARM_SHIFT_PWM      = 35  # アームを上下端へ動かす際のモーター出力。
JUNCT_UPPER_THRESH = 50  # ライン合流開始と判定する輪郭幅の上側しきい値。
JUNCT_LOWER_THRESH = 40  # ライン分岐完了と判定する輪郭幅の下側しきい値。
ROE_DEGEN          = 90  # ラインが画像の接線方向に近いとみなす両端幅のしきい値。
CURV_MIN_ROWS_SEP  = 15  # 曲率推定を有効とする近側・遠側走査行の最小間隔。

# 【日本語解説】 アームを上下どちらへ動かすかを、モーター回転方向の符号で表す列挙型。
class ArmDirection(IntEnum):
    UP = -1
    DOWN = 1

# 【日本語解説】 カメラ画像から判定する合流・分岐の進行状態を表す列挙型。
class JState(Enum):
    INITIAL = auto()
    JOINING = auto()
    JOINED = auto()
    FORKING = auto()
    FORKED = auto()

# 【日本語解説】 旋回目標角を絶対角と相対角のどちらで解釈するかを表す列挙型。
class HeadingType(Enum):
    ABSOLUTE = "absolute"
    RELATIVE = "relative"

g_plotter: Plotter = None
g_hub: Hub = None
g_arm_motor: Motor = None
g_right_motor: Motor = None
g_left_motor: Motor = None
g_touch_sensor: TouchSensor = None
g_color_sensor: ColorSensor = None
g_sonar_sensor: SonarSensor = None
g_gyro_sensor: GyroSensor = None
g_course: int = 0
g_key: str = None  # ReadKey.update()が書き込むQRヒント復号キー。
g_bottle_color = BottleColor.NONE  # CatchBottle.update()が書き込む回収対象ボトルの色。
g_hint1: str = None  # IsQRDecoded.update()が書き込む第1ヒント。
g_hint2: str = None  # IsQRDecoded.update()が書き込む第2ヒント。

# 【日本語解説】 行動木の全工程終了後もプロセスを維持し、安全な手動終了を待つ終端ノード。
class TheEnd(Behaviour):
    # 【日本語解説】 TheEndの設定値と実行中に保持する状態を初期化する。
    def __init__(self, name: str):
        # 【引数】 name: 行動木のログや可視化で識別するノード名。
        super(TheEnd, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.running = False

    # 【日本語解説】 TheEndの処理を1制御周期分実行し、行動木へRUNNING・SUCCESS・FAILUREの状態を返す。
    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.behavior tree exhausted. ctrl+C shall terminate the program" % (g_plotter.get_distance(), self.__class__.__name__))
        return Status.RUNNING


# 【日本語解説】 走行開始前にモーター角度、ジャイロ、カメラ判定条件を初期化する行動ノード。
class ResetDevice(Behaviour):
    # 【日本語解説】 ResetDeviceの設定値と実行中に保持する状態を初期化する。
    def __init__(self, name: str):
        # 【引数】 name: 行動木のログや可視化で識別するノード名。
        super(ResetDevice, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.count = 0

    # 【日本語解説】 ResetDeviceの処理を1制御周期分実行し、行動木へRUNNING・SUCCESS・FAILUREの状態を返す。
    def update(self) -> Status:
        if self.count == 0:
            g_arm_motor.reset_count()
            g_right_motor.reset_count()
            g_left_motor.reset_count()
            g_gyro_sensor.reset()
            g_video.set_thresholds(GS_MIN_DEFAULT, GS_MAX_DEFAULT)
            g_video.set_target_interested(TargetInterested.LINE)
            self.logger.info("%+06d %s.resetting..." % (g_plotter.get_distance(), self.__class__.__name__))
            self.logger.info("%+06d %s.waiting for IMU to be stationary..." % (g_plotter.get_distance(), self.__class__.__name__))
        elif self.count > 3:
            self.logger.info("%+06d %s.complete" % (g_plotter.get_distance(), self.__class__.__name__))
            return Status.SUCCESS
        if g_hub.hub_imu_is_stationary():
            self.count += 1
        return Status.RUNNING


# 【日本語解説】 アームを機械端まで動かし、角度変化の停止から到達を判定する行動ノード。
class ArmUpDownFull(Behaviour):
    # 【日本語解説】 ArmUpDownFullの設定値と実行中に保持する状態を初期化する。
    def __init__(self, name: str, direction: ArmDirection):
        # 【引数】 name: 行動木のログや可視化で識別するノード名。
        # 【引数】 direction: アームを動かす方向（上または下）。
        super(ArmUpDownFull, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.direction = direction
        self.running = False

    # 【日本語解説】 ArmUpDownFullの処理を1制御周期分実行し、行動木へRUNNING・SUCCESS・FAILUREの状態を返す。
    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.prev_degree = g_arm_motor.get_count()
            self.logger.info("%+06d %s.start position is %d" % (g_plotter.get_distance(), self.__class__.__name__, self.prev_degree))
            self.count = 0
            g_arm_motor.set_power(ARM_SHIFT_PWM * self.direction)
        else:
            cur_degree = g_arm_motor.get_count()
            if abs(cur_degree - self.prev_degree) < 5:
                if self.count > 20:
                    g_arm_motor.set_power(0)
                    g_arm_motor.set_brake(True)
                    self.logger.info("%+06d %s.position set to %d" % (g_plotter.get_distance(), self.__class__.__name__, cur_degree))
                    return Status.SUCCESS
                else:
                    self.count += 1
            self.prev_degree = cur_degree
        return Status.RUNNING


# 【日本語解説】 暗号化ヒントを復号する4文字キーを対話入力し、確認後に共有状態へ保存する行動ノード。
class ReadKey(Behaviour):
    # 【日本語解説】 ReadKeyの設定値と実行中に保持する状態を初期化する。
    def __init__(self, name: str):
        # 【引数】 name: 行動木のログや可視化で識別するノード名。
        super(ReadKey, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.running = False

    # 【日本語解説】 ReadKeyの処理を1制御周期分実行し、行動木へRUNNING・SUCCESS・FAILUREの状態を返す。
    def update(self) -> Status:
        if not self.running:
            self.running = True
            return Status.RUNNING
        else:
            global g_key
            g_key = input("Enter the given key for decryption: ")
            # 入力キーが4文字であることを確認する
            if len(g_key) != 4:
                self.logger.warning("%+06d %s.invalid key length: %d. key should be 4 characters long." % (g_plotter.get_distance(), self.__class__.__name__, len(g_key)))
                return Status.RUNNING
            else:
                # 入力されたキーを表示し、利用者へ確認を求める
                self.logger.info("%+06d %s.entered key: %s" % (g_plotter.get_distance(), self.__class__.__name__, g_key))
                confirmation = input("Is the entered key correct? (y/n): ")
                if confirmation.lower() == 'y':
                    self.logger.info("%+06d %s.key confirmed" % (g_plotter.get_distance(), self.__class__.__name__))
                    return Status.SUCCESS
                else:
                    self.logger.info("%+06d %s.key rejected, please enter again" % (g_plotter.get_distance(), self.__class__.__name__))
                    return Status.RUNNING


# 【日本語解説】 初回実行時から指定秒数が経過したかを判定する条件ノード。
class IsTimePassed(Behaviour):
    # 【日本語解説】 IsTimePassedの設定値と実行中に保持する状態を初期化する。
    def __init__(self, name: str, delta_time: int):
        # 【引数】 name: 行動木のログや可視化で識別するノード名。
        # 【引数】 delta_time: 条件成立まで待つ時間（秒）。
        super(IsTimePassed, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.delta_time = delta_time
        self.running = False
        self.earned = False

    # 【日本語解説】 IsTimePassedの処理を1制御周期分実行し、行動木へRUNNING・SUCCESS・FAILUREの状態を返す。
    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.orig_time = time.time()
            self.logger.info("%+06d %s.accumulation started for delta=%d" % (self.orig_time, self.__class__.__name__, self.delta_time))
        cur_time = time.time()
        earned_time = cur_time - self.orig_time
        if earned_time >= self.delta_time:
            if not self.earned:
                self.earned = True
                self.logger.info("%+06d %s.delta time passed" % (g_plotter.get_distance(), self.__class__.__name__))
            return Status.SUCCESS
        else:
            return Status.RUNNING


# 【日本語解説】 オドメトリ上の基準位置から指定距離を走行したかを判定する条件ノード。
class IsDistanceEarned(Behaviour):
    # 【日本語解説】 IsDistanceEarnedの設定値と実行中に保持する状態を初期化する。
    def __init__(self, name: str, delta_dist: int):
        # 【引数】 name: 行動木のログや可視化で識別するノード名。
        # 【引数】 delta_dist: 条件成立とみなす基準位置からの走行距離（mm）。
        super(IsDistanceEarned, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.delta_dist = delta_dist
        self.running = False
        self.earned = False

    # 【日本語解説】 IsDistanceEarnedの処理を1制御周期分実行し、行動木へRUNNING・SUCCESS・FAILUREの状態を返す。
    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.orig_dist = g_plotter.get_distance()
            self.logger.info("%+06d %s.accumulation started for delta=%d" % (self.orig_dist, self.__class__.__name__, self.delta_dist))
        cur_dist = g_plotter.get_distance()
        earned_dist = cur_dist - self.orig_dist
        if (earned_dist >= self.delta_dist or -earned_dist <= -self.delta_dist):
            if not self.earned:
                self.earned = True
                self.logger.info("%+06d %s.delta distance earned" % (cur_dist, self.__class__.__name__))
            return Status.SUCCESS
        else:
            return Status.RUNNING


# 【日本語解説】 SPIKEのカラーセンサー値を平滑化して指定色の検出を判定する条件ノード。
class IsColorDetected(Behaviour):
    # 【日本語解説】 IsColorDetectedの設定値と実行中に保持する状態を初期化する。
    def __init__(self, name: str, color: Color):
        # 【引数】 name: 行動木のログや可視化で識別するノード名。
        # 【引数】 color: 検出・照合の対象とする色。
        super(IsColorDetected, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.color = color
        self.prevColor = Color.UNKNOWN
        self.classifier = ColorClassifier()
        self.running = False
        self.detected = False

    # 【日本語解説】 IsColorDetectedの処理を1制御周期分実行し、行動木へRUNNING・SUCCESS・FAILUREの状態を返す。
    def update(self) -> Status:
        cur_dist = g_plotter.get_distance()
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.detection started for color=%s" % (cur_dist, self.__class__.__name__, self.color.value))
        h, s, v = g_color_sensor.get_raw_color_hsv()

        detected_color = self.classifier.classify(h, s, v)
        if detected_color == self.color:
            if not self.detected:
                self.detected = True
                self.logger.info("%+06d %s.color=%s detected" % (cur_dist, self.__class__.__name__, self.color.value))
            return Status.SUCCESS
        else:
            if detected_color != self.prevColor:
                # ログが煩雑にならないよう、UNKNOWNは出力しない
                if detected_color != Color.UNKNOWN or self.prevColor != Color.UNKNOWN:
                    self.logger.info("%+06d %s.color changed from %s to %s" % (cur_dist, self.__class__.__name__, self.prevColor.value, detected_color.value))
                    self.prevColor = detected_color
            return Status.RUNNING


# 【日本語解説】 Webカメラで得たQR文字列を分類・復号し、競技用ヒントとして保存する条件ノード。
class IsQRDecoded(Behaviour):
    # 【日本語解説】 IsQRDecodedの設定値と実行中に保持する状態を初期化する。
    def __init__(self, name: str):
        # 【引数】 name: 行動木のログや可視化で識別するノード名。
        super(IsQRDecoded, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.running = False
        self.detected = False

    # 【日本語解説】 IsQRDecodedの処理を1制御周期分実行し、行動木へRUNNING・SUCCESS・FAILUREの状態を返す。
    def update(self) -> Status:
        global g_key, g_hint1, g_hint2
        if not self.running:
            self.running = True
            g_video.set_target_interested(TargetInterested.QRCODE)
            self.logger.info("%+06d %s.detection started for QR code" % (g_plotter.get_distance(), self.__class__.__name__))
        text = g_video.get_QR_text()
        if text != "":
            if not self.detected:
                self.detected = True
                hint_type, hint_text = Hint(text).resolve(password=g_key)
                if hint_type == HintType.HINT1:
                    g_hint1 = hint_text
                elif hint_type == HintType.HINT2:
                    g_hint2 = hint_text
                self.logger.info("%+06d %s.QR code decoded: %s" % (g_plotter.get_distance(), self.__class__.__name__, hint_text))
                g_video.set_target_interested(TargetInterested.LINE)  # 後続処理で使用する状態または設定値を更新する
            return Status.SUCCESS
        else:
            return Status.RUNNING


# 【日本語解説】 超音波センサーの距離が警戒距離以内かを判定する条件ノード。
class IsSonarOn(Behaviour):
    # 【日本語解説】 IsSonarOnの設定値と実行中に保持する状態を初期化する。
    def __init__(self, name: str, alert_dist: int):
        # 【引数】 name: 行動木のログや可視化で識別するノード名。
        # 【引数】 alert_dist: 障害物ありと判定する超音波センサー距離の上限（mm）。
        super(IsSonarOn, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.alert_dist = alert_dist
        self.running = False

    # 【日本語解説】 IsSonarOnの処理を1制御周期分実行し、行動木へRUNNING・SUCCESS・FAILUREの状態を返す。
    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.detection started for dist=%d" % (g_plotter.get_distance(), self.__class__.__name__, self.alert_dist))
        
        dist = g_sonar_sensor.get_distance()
        if (dist <= self.alert_dist and dist > 0):
            self.logger.info("%+06d %s.alerted at dist=%d" % (g_plotter.get_distance(), self.__class__.__name__, dist))
            return Status.SUCCESS
        else:
            return Status.RUNNING


# 【日本語解説】 タッチセンサーが押されたかを判定する条件ノード。
class IsTouchOn(Behaviour):
    # 【日本語解説】 IsTouchOnの設定値と実行中に保持する状態を初期化する。
    def __init__(self, name: str):
        # 【引数】 name: 行動木のログや可視化で識別するノード名。
        super(IsTouchOn, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.running = False

    # 【日本語解説】 IsTouchOnの処理を1制御周期分実行し、行動木へRUNNING・SUCCESS・FAILUREの状態を返す。
    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.waiting for touch..." % (g_plotter.get_distance(), self.__class__.__name__))
        if g_touch_sensor.is_pressed():
            self.logger.info("%+06d %s.pressed" % (g_plotter.get_distance(), self.__class__.__name__))
            return Status.SUCCESS
        else:
            return Status.RUNNING


# 【日本語解説】 左右モーターへの出力を直ちにゼロにして機体を停止させる行動ノード。
class StopNow(Behaviour):
    # 【日本語解説】 StopNowの設定値と実行中に保持する状態を初期化する。
    def __init__(self, name: str):
        # 【引数】 name: 行動木のログや可視化で識別するノード名。
        super(StopNow, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    # 【日本語解説】 StopNowの処理を1制御周期分実行し、行動木へRUNNING・SUCCESS・FAILUREの状態を返す。
    def update(self) -> Status:
        g_right_motor.set_power(0)
        g_right_motor.set_brake(True)
        g_left_motor.set_power(0)
        g_left_motor.set_brake(True)
        self.logger.info("%+06d %s.motors stopped" % (g_plotter.get_distance(), self.__class__.__name__))
        return Status.SUCCESS


# 【日本語解説】 左右モーターへ個別の固定PWMを与えて走行する行動ノード。
class RunAsInstructed(Behaviour):
    # 【日本語解説】 RunAsInstructedの設定値と実行中に保持する状態を初期化する。
    def __init__(self, name: str, pwm_l: int, pwm_r: int) -> None:
        # 【引数】 name: 行動木のログや可視化で識別するノード名。
        # 【引数】 pwm_l: 左モーターへ与えるPWM値。正負で回転方向を表す。
        # 【引数】 pwm_r: 右モーターへ与えるPWM値。正負で回転方向を表す。
        super(RunAsInstructed, self).__init__(name)
        self.pwm_l = g_course * pwm_l
        self.pwm_r = g_course * pwm_r
        self.running = False


    # 【日本語解説】 RunAsInstructedの処理を1制御周期分実行し、行動木へRUNNING・SUCCESS・FAILUREの状態を返す。
    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.started with pwm=(%s, %s)" % (g_plotter.get_distance(), self.__class__.__name__, self.pwm_l, self.pwm_r))
        g_right_motor.set_power(self.pwm_r)
        g_left_motor.set_power(self.pwm_l)
        return Status.RUNNING


# 【日本語解説】 カラーセンサーの反射光をPID制御し、指定した側のライン端を追従する行動ノード。
class TraceLine(Behaviour):
    # 【日本語解説】 TraceLineの設定値と実行中に保持する状態を初期化する。
    def __init__(self, name: str, target: int, power: int, pid_p: float, pid_i: float, pid_d: float,
                 trace_side: TraceSide,
                 # ローパスフィルターの設定値
                 cutoff_hz: float = 12.0, median_window: int = 0,
                 # 走行状態に応じて速度を変えるための設定値
                 power_min: int = None,  # カーブ走行時に許可する基準出力の下限。
                 err_lo: float = 6.0,  # この誤差以下を直線寄りとみなして高速側へ制御する。
                 err_hi: float = 22.0,  # この誤差以上を急カーブとみなして低速側へ制御する。
                 accel_per_s: float = 60.0,  # 基準出力を1秒間に増加できる最大量。
                 decel_per_s: float = 180.0,  # 基準出力を1秒間に減少できる最大量。
                 metric_hz: float = 2.0,
                 # 現在速度に合わせ、低速用と高速用のPIDゲインを線形補間する
                 # 直線では速度を上げ、カーブでは操舵余裕を確保するため速度を下げる
                 # 低速・カーブ側で使用するKpとKd。
                 gains_slow: tuple = None,  # 低速・カーブ側で使用するKpとKd。
                 gains_fast: tuple = None,  # 高速・直線側で使用するKpとKd。
                 # ライン消失が継続した場合、最後に見えていた側へ旋回して再検出を試みる
                 recover_v: int = None,  # ライン消失と判断する明度のしきい値。
                 recover_after: int = 3,  # 復帰旋回を始めるまでに必要な連続ライン消失回数。
                 recover_turn: int = None         # カーブで基準速度を落としても操舵可能量が狭まらないよう、PID出力上限には最大速度を使う
                ) -> None:
        # 【引数】 name: 行動木のログや可視化で識別するノード名。
        # 【引数】 target: 追従する反射光値、または目標方位角。用途はクラスごとに異なる。
        # 【引数】 power: 走行時の基準モーター出力。
        # 【引数】 pid_p: PID制御の比例ゲイン。現在の誤差に対する補正量を決める。
        # 【引数】 pid_i: PID制御の積分ゲイン。継続する偏差を補正する。
        # 【引数】 pid_d: PID制御の微分ゲイン。誤差の急変や振動を抑える。
        # 【引数】 trace_side: ラインの通常側・反対側のどちらの端を追従するか。
        # 【引数】 cutoff_hz: ローパスフィルターの遮断周波数（Hz）。
        # 【引数】 median_window: 単発ノイズ除去に使う中央値フィルターのサンプル数。0で無効。
        # 【引数】 power_min: カーブ減速時にも維持する最小モーター出力。
        # 【引数】 err_lo: 直線相当と判断して加速を始める追従誤差の下限。
        # 【引数】 err_hi: 強いカーブ相当と判断して最小速度にする追従誤差の上限。
        # 【引数】 accel_per_s: 1秒当たりに許可する基準出力の増加量。
        # 【引数】 decel_per_s: 1秒当たりに許可する基準出力の減少量。
        # 【引数】 metric_hz: metric_hzとして処理へ渡す値。
        # 【引数】 gains_slow: 低速・カーブ走行時に使用する比例ゲインと微分ゲインの組。
        # 【引数】 gains_fast: 高速・直線走行時に使用する比例ゲインと微分ゲインの組。
        # 【引数】 recover_v: ライン消失と判断する反射光値のしきい値。
        # 【引数】 recover_after: ライン消失後、復帰旋回を開始するまでの連続検出回数。
        # 【引数】 recover_turn: ライン再検出のために与える旋回出力。
        super(TraceLine, self).__init__(name)
        # PID出力を左右モーターの差動操舵量として使用する
        # カーブで基準速度を落としても操舵可能量が狭まらないよう、PID出力上限には最大速度を使う
        # 走行状態に応じたモーター出力を計算する。
        self.power_max = power
        self.power_min = power if power_min is None else power_min
        self.power = power
        self.adapt = power_min is not None
        self.target = target
        self.pid = PID(pid_p, pid_i, pid_d, setpoint=target, sample_time=EXEC_INTERVAL, output_limits=(-self.power_max, self.power_max))
        self.trace_side = trace_side
        self.lpf = (LowPassFilter(cutoff_hz, EXEC_INTERVAL, median_window) if cutoff_hz else None) # PID出力を左右モーターの差動操舵量として使用する
        # 追従誤差の絶対値を平滑化し、走行の不安定さとカーブの強さを推定する
        self.err_lo, self.err_hi = err_lo, err_hi
        self.metric_lpf = LowPassFilter(metric_hz, EXEC_INTERVAL)
        self.err_metric = 0.0
        # 急減速と緩やかな加速になるよう、制御周期ごとの速度変化量を制限する
        self.accel_step = accel_per_s * EXEC_INTERVAL
        self.decel_step = decel_per_s * EXEC_INTERVAL
        # 現在速度に合わせ、低速用と高速用のPIDゲインを線形補間する
        # 低速・カーブ側で使用するKpとKd。
        self.gains_slow = gains_slow
        self.gains_fast = gains_fast
        self.schedule = (gains_slow is not None and gains_fast is not None
                         and self.power_max > self.power_min)
        # ラインを見失った場合の復帰処理
        self.recover_v = recover_v
        self.recover_after = recover_after
        self.recover_turn = recover_turn
        self._lost_count = 0

        self.running = False

    # 【日本語解説】 TraceLineの処理を1制御周期分実行し、行動木へRUNNING・SUCCESS・FAILUREの状態を返す。
    def update(self) -> Status:
        if not self.running:
            if self.lpf:
                self.lpf.reset()
            self.metric_lpf.reset()
            self.running = True
            self.logger.info("%+06d %s.trace started with TS=%s" % (g_plotter.get_distance(), self.__class__.__name__, self.trace_side.name))

        h, s, v_raw = g_color_sensor.get_raw_color_hsv()
        v = self.lpf(v_raw) if self.lpf else v_raw

        # ライン追従誤差に応じて基準速度を適応的に調整する
        # 追従誤差の絶対値を平滑化し、走行の不安定さとカーブの強さを推定する
        # self.err_metricへ後続処理で使用する計算結果を保存する。
        self.err_metric = self.metric_lpf(abs(self.target - v_raw))
        if self.adapt:
            # この誤差以下を直線寄りとみなして高速側へ制御する。
            frac = (self.err_metric - self.err_lo) / (self.err_hi - self.err_lo)
            frac = 0.0 if frac < 0.0 else (1.0 if frac > 1.0 else frac)
            target_power = self.power_max - frac * (self.power_max - self.power_min)
            # dpへ後続処理で使用する計算結果を保存する。
            dp = target_power - self.power
            if dp > self.accel_step:
                dp = self.accel_step
            elif dp < -self.decel_step:
                dp = -self.decel_step
            self.power += dp
        # PID制御器から現在の誤差に対応する補正量を求める。
        kp_now, kd_now = self.pid.Kp, self.pid.Kd
        if self.schedule:
            f = (self.power - self.power_min) / (self.power_max - self.power_min)
            f = 0.0 if f < 0.0 else (1.0 if f > 1.0 else f)
            kp_now = self.gains_slow[0] + f * (self.gains_fast[0] - self.gains_slow[0])
            kd_now = self.gains_slow[1] + f * (self.gains_fast[1] - self.gains_slow[1])
            self.pid.tunings = (kp_now, self.pid.Ki, kd_now)
        # カーブで基準速度を落としても操舵可能量が狭まらないよう、PID出力上限には最大速度を使う
        if self.trace_side == TraceSide.NORMAL:
            turn = (-1) * g_course * int(self.pid(v))
        else:  # 上記条件に当てはまらない場合の処理。
            turn = g_course * int(self.pid(v))

        # ライン消失が継続した場合、最後に見えていた側へ旋回して再検出を試みる
        # ライン消失と判断する明度のしきい値。
        if self.recover_v is not None:
            if v_raw >= self.recover_v:
                self._lost_count += 1
            else:
                self._lost_count = 0
            if self._lost_count >= self.recover_after and turn != 0:
                mag = self.power_max if self.recover_turn is None else self.recover_turn
                turn = int(math.copysign(mag, turn))

        # pへ後続処理で使用する計算結果を保存する。
        p = int(round(self.power))
        left  = max(-100, min(100, p + turn))  # 左モーター出力を-100～100へ制限する。
        right = max(-100, min(100, p - turn))
        g_right_motor.set_power(right)
        g_left_motor.set_power(left)

        # 計算または判定した結果を呼び出し元へ返す。
        #self.logger.info("%+06d %s.color sensor HSV=(%d, %d, %d) vf=%d, em=%d, pwr=%d, kp=%.3f, kd=%.3f, turn=%d" % (
        #    g_plotter.get_distance(), self.__class__.__name__,
        # 計算または判定した結果を呼び出し元へ返す。

        return Status.RUNNING


# 【日本語解説】 その場旋回しながら反射光の変化を調べ、ライン位置で停止する行動ノード。
class SpinAndLocateLine(Behaviour):
    # 【日本語解説】 SpinAndLocateLineの設定値と実行中に保持する状態を初期化する。
    def __init__(self, name: str, target: int, max_power: int, min_power: int,
                 pid_p: float, pid_i: float, pid_d: float, trace_side: TraceSide) -> None:
        # 【引数】 name: 行動木のログや可視化で識別するノード名。
        # 【引数】 target: 追従する反射光値、または目標方位角。用途はクラスごとに異なる。
        # 【引数】 max_power: 旋回制御で許可する最大モーター出力。
        # 【引数】 min_power: 旋回を止めないために維持する最小モーター出力。
        # 【引数】 pid_p: PID制御の比例ゲイン。現在の誤差に対する補正量を決める。
        # 【引数】 pid_i: PID制御の積分ゲイン。継続する偏差を補正する。
        # 【引数】 pid_d: PID制御の微分ゲイン。誤差の急変や振動を抑える。
        # 【引数】 trace_side: ラインの通常側・反対側のどちらの端を追従するか。
        super(SpinAndLocateLine, self).__init__(name)
        self.target = target
        self.spin_direction = 1 if trace_side == TraceSide.NORMAL else -1
        self.pid_p = pid_p
        self.pid_i = pid_i
        self.pid_d = pid_d
        self.clamper = SymmetricClamper(min_power, max_power)
        self.move_away = True
        self.running = False

    # 【日本語解説】 SpinAndLocateLineの処理を1制御周期分実行し、行動木へRUNNING・SUCCESS・FAILUREの状態を返す。
    def update(self) -> Status:
        # current_headingへ後続処理で使用する計算結果を保存する。
        current_heading = (-1) * g_course * g_gyro_sensor.get_angle()
        if not self.running:
            # self.target_headingへ後続処理で使用する計算結果を保存する。
            self.target_heading = current_heading + self.spin_direction * 30
            self.pid = PID(self.pid_p, self.pid_i, self.pid_d, setpoint=self.target_heading, sample_time=EXEC_INTERVAL)
            self.running = True
            self.logger.info("%+06d %s.spin started at heading=%d for %d" % (g_plotter.get_distance(),
                                                                             self.__class__.__name__, current_heading, self.target_heading))
        if self.move_away:
            # 目標値と現在値の差から制御誤差を計算する。
            error = float(self.target_heading) - current_heading
            # 現在の状態と判定条件に応じて後続処理を分岐する。
            if error > 180.0:
                error -= 360.0
            if error < -180.0:
                error += 360.0
            if abs(error) < 2.0:
                self.logger.info("%+06d %s.move away spin ended at heading=%d" % (g_plotter.get_distance(),
                                                                    self.__class__.__name__, current_heading))
                self.pid = PID(self.pid_p, self.pid_i, self.pid_d, setpoint=self.target, sample_time=EXEC_INTERVAL)
                self.spin_direction *= -1
                self.move_away = False
                return Status.RUNNING
            power = int(self.clamper.clamp(self.pid(current_heading)))
        else:  # 上記条件に当てはまらない場合の処理。
            h, s, v = g_color_sensor.get_raw_color_hsv()
            error = float(self.target) - v
            if abs(error) < 5.0:
                self.logger.info("%+06d %s.line located at heading=%d" % (g_plotter.get_distance(),
                                                                    self.__class__.__name__, current_heading))        
                return Status.SUCCESS
            power = int(self.clamper.clamp(self.pid(v))) * self.spin_direction * (-1)
        g_right_motor.set_power(g_course * power)
        g_left_motor.set_power((-1) * g_course * power)
        return Status.RUNNING    


# 【日本語解説】 ジャイロ角を監視しながら目標方位までその場旋回する行動ノード。
class SpinAround(Behaviour):
    # 【日本語解説】 SpinAroundの設定値と実行中に保持する状態を初期化する。
    def __init__(self, name: str, target: int, max_power: int, min_power: int,
                 pid_p: float, pid_i: float, pid_d: float, target_type: HeadingType) -> None:
        # 【引数】 name: 行動木のログや可視化で識別するノード名。
        # 【引数】 target: 追従する反射光値、または目標方位角。用途はクラスごとに異なる。
        # 【引数】 max_power: 旋回制御で許可する最大モーター出力。
        # 【引数】 min_power: 旋回を止めないために維持する最小モーター出力。
        # 【引数】 pid_p: PID制御の比例ゲイン。現在の誤差に対する補正量を決める。
        # 【引数】 pid_i: PID制御の積分ゲイン。継続する偏差を補正する。
        # 【引数】 pid_d: PID制御の微分ゲイン。誤差の急変や振動を抑える。
        # 【引数】 target_type: 目標方位を絶対角または現在角からの相対角として解釈する指定。
        super(SpinAround, self).__init__(name)
        self.target = target
        self.target_type = target_type
        self.pid_p = pid_p
        self.pid_i = pid_i
        self.pid_d = pid_d
        self.clamper = SymmetricClamper(min_power, max_power)
        self.running = False

    # 【日本語解説】 SpinAroundの処理を1制御周期分実行し、行動木へRUNNING・SUCCESS・FAILUREの状態を返す。
    def update(self) -> Status:
        current_heading = (-1) * g_course * g_gyro_sensor.get_angle()
        if not self.running:
            if self.target_type == HeadingType.RELATIVE:
                self.target_heading = current_heading + self.target
            else:
                self.target_heading = self.target
            self.pid = PID(self.pid_p, self.pid_i, self.pid_d, setpoint=self.target_heading, sample_time=EXEC_INTERVAL)
            self.running = True
            self.logger.info("%+06d %s.spin started at heading=%d for %d" % (g_plotter.get_distance(),
                                                                             self.__class__.__name__, current_heading, self.target_heading))
        error = float(self.target_heading) - current_heading
        # 現在の状態と判定条件に応じて後続処理を分岐する。
        if error > 180.0:
            error -= 360.0
        if error < -180.0:
            error += 360.0
        if abs(error) < 2.0:
            self.logger.info("%+06d %s.spin ended at heading=%d" % (g_plotter.get_distance(),
                                                                    self.__class__.__name__, current_heading))
            return Status.SUCCESS
        power = int(self.clamper.clamp(self.pid(current_heading)))
        g_right_motor.set_power(g_course * power)
        g_left_motor.set_power((-1) * g_course * power)
        return Status.RUNNING    


# 【日本語解説】 ジャイロ方位のずれを補正し、指定方向を保ちながら直進する行動ノード。
class RunByGyro(Behaviour):
    # 【日本語解説】 RunByGyroの設定値と実行中に保持する状態を初期化する。
    def __init__(self, name: str, target: int, power: int,
                 pid_p: float, pid_i: float, pid_d: float, target_type: HeadingType) -> None:
        # 【引数】 name: 行動木のログや可視化で識別するノード名。
        # 【引数】 target: 追従する反射光値、または目標方位角。用途はクラスごとに異なる。
        # 【引数】 power: 走行時の基準モーター出力。
        # 【引数】 pid_p: PID制御の比例ゲイン。現在の誤差に対する補正量を決める。
        # 【引数】 pid_i: PID制御の積分ゲイン。継続する偏差を補正する。
        # 【引数】 pid_d: PID制御の微分ゲイン。誤差の急変や振動を抑える。
        # 【引数】 target_type: 目標方位を絶対角または現在角からの相対角として解釈する指定。
        super(RunByGyro, self).__init__(name)
        self.target = target
        self.target_type = target_type
        self.power = power
        self.pid_p = pid_p
        self.pid_i = pid_i
        self.pid_d = pid_d
        self.last_log_time = None
        self.running = False

    # 【日本語解説】 RunByGyroの処理を1制御周期分実行し、行動木へRUNNING・SUCCESS・FAILUREの状態を返す。
    def update(self) -> Status:
        current_heading = (-1) * g_course * g_gyro_sensor.get_angle()
        # ログ量を抑えるため、一定時間が経過した場合だけ状態を出力する。
        if self.last_log_time == None or time.time() - self.last_log_time >= 1.0:
            self.logger.info("%+06d %s.current heading=%d" % (g_plotter.get_distance(), self.__class__.__name__, current_heading))
            self.last_log_time = time.time()
        if not self.running:
            if self.target_type == HeadingType.RELATIVE:
                self.target_heading = current_heading + self.target
            else:
                self.target_heading = self.target
            self.pid = PID(self.pid_p, self.pid_i, self.pid_d, setpoint=self.target_heading, sample_time=EXEC_INTERVAL, output_limits=(-self.power, self.power))
            self.logger.info("%+06d %s.gyro run started toward heading=%d" % (g_plotter.get_distance(),
                                                                              self.__class__.__name__, self.target_heading))
            self.running = True
        turn = int(self.pid(current_heading))
        g_right_motor.set_power(self.power + g_course * turn)
        g_left_motor.set_power(self.power - g_course * turn)
        return Status.RUNNING


# 【日本語解説】 Webカメラから得たライン角度をPID制御へ入力して追従走行する行動ノード。
class TraceLineCam(Behaviour):
    # 【日本語解説】 TraceLineCamの設定値と実行中に保持する状態を初期化する。
    def __init__(self, name: str, power: int, pid_p: float, pid_i: float, pid_d: float,
                 gs_min: int, gs_max: int, trace_side: TraceSide,
                 tilt_ff_gain: float = 0.0,  # ライン傾きを操舵量へ先行加算するフィードフォワードゲイン。
                 ff_cap: float = 8.0,  # ff_cap: floatへ、この処理で使用する設定値または計算結果を保存する。
                 blind_hold_frames: int = 3,  # blind_hold_frames: intへ、この処理で使用する設定値または計算結果を保存する。
                 blind_turn_frac: float = 0.55,  # blind_turn_frac: floatへ、この処理で使用する設定値または計算結果を保存する。
                 ) -> None:
        # 【引数】 name: 行動木のログや可視化で識別するノード名。
        # 【引数】 power: 走行時の基準モーター出力。
        # 【引数】 pid_p: PID制御の比例ゲイン。現在の誤差に対する補正量を決める。
        # 【引数】 pid_i: PID制御の積分ゲイン。継続する偏差を補正する。
        # 【引数】 pid_d: PID制御の微分ゲイン。誤差の急変や振動を抑える。
        # 【引数】 gs_min: ラインとして抽出するグレースケール値の下限。
        # 【引数】 gs_max: ラインとして抽出するグレースケール値の上限。
        # 【引数】 trace_side: ラインの通常側・反対側のどちらの端を追従するか。
        # 【引数】 tilt_ff_gain: tilt_ff_gainとして処理へ渡す値。
        # 【引数】 ff_cap: ff_capとして処理へ渡す値。
        # 【引数】 blind_hold_frames: blind_hold_framesとして処理へ渡す値。
        # 【引数】 blind_turn_frac: blind_turn_fracとして処理へ渡す値。
        super(TraceLineCam, self).__init__(name)
        self.power = power
        self.pid = PID(pid_p, pid_i, pid_d, setpoint=0, sample_time=EXEC_INTERVAL, output_limits=(-power, power))
        self.gs_min = gs_min
        self.gs_max = gs_max
        self._tilt_ff_gain = tilt_ff_gain
        self._ff_cap = ff_cap
        self._blind_hold_frames = blind_hold_frames
        self._blind_turn_frac = blind_turn_frac
        self._blind = 0
        self.trace_side = trace_side
        self.running = False

    # 【日本語解説】 TraceLineCamの処理を1制御周期分実行し、行動木へRUNNING・SUCCESS・FAILUREの状態を返す。
    def update(self) -> Status:
        if not self.running:
            self.running = True
            g_video.set_thresholds(self.gs_min, self.gs_max)
            g_video.set_target_interested(TargetInterested.LINE)
            if self.trace_side == TraceSide.NORMAL:
                if g_course == -1:  # if g_courseへ、この処理で使用する設定値または計算結果を保存する。
                    g_video.set_trace_side(TraceSide.RIGHT)
                else:
                    g_video.set_trace_side(TraceSide.LEFT)
            elif self.trace_side == TraceSide.OPPOSITE: 
                if g_course == -1:  # if g_courseへ、この処理で使用する設定値または計算結果を保存する。
                    g_video.set_trace_side(TraceSide.LEFT)
                else:
                    g_video.set_trace_side(TraceSide.RIGHT)
            else:  # 上記条件に当てはまらない場合の処理。
                g_video.set_trace_side(TraceSide.CENTER)
            self.logger.info("%+06d %s.trace started with TS=%s" % (g_plotter.get_distance(), self.__class__.__name__, self.trace_side.name))

        theta, fid, cap_t, odo_cap = g_video.get_theta_stamped()
        odo_now = g_plotter.get_distance()

        # 認識状況を確認できるよう画像へ線を描画する。
        tilt = g_video.get_line_tilt()
        roe  = g_video.get_range_of_edges()
        tilt_ff = 0.0
        ff_gated = (roe == 0
                    or roe > ROE_DEGEN
                    or g_video.get_band_sep() < CURV_MIN_ROWS_SEP)
        if not ff_gated:
            tilt_ff = self._tilt_ff_gain * tilt
            tilt_ff = max(-self._ff_cap, min(self._ff_cap, tilt_ff))  # tilt_ffへ、この処理で使用する設定値または計算結果を保存する。

        # PID出力を左右モーターの差動操舵量として使用する
        turn_pid = self.pid(theta)
        turn = turn_pid + tilt_ff

        # 現在の状態と判定条件に応じて後続処理を分岐する。
        if not g_video.is_target_insight():
            self._blind += 1
        else:
            self._blind = 0
        blind_capped = False
        if self._blind > self._blind_hold_frames:
            hold = self.power * self._blind_turn_frac
            if turn > hold:
                turn = hold; blind_capped = True
            elif turn < -hold:
                turn = -hold; blind_capped = True

        g_right_motor.set_power(self.power + int(turn))
        g_left_motor.set_power(self.power - int(turn))

        # 計算または判定した結果を呼び出し元へ返す。
        #self.logger.info(
        # 計算または判定した結果を呼び出し元へ返す。
        #        int(turn), self.power + int(turn), self.power - int(turn),
        # 計算または判定した結果を呼び出し元へ返す。
        return Status.RUNNING


# 【日本語解説】 画像中のライン幅と傾きから合流・分岐の状態遷移を判定する条件ノード。
class IsJunction(Behaviour):
    # 【日本語解説】 IsJunctionの設定値と実行中に保持する状態を初期化する。
    def __init__(self, name: str, target_state: JState) -> None:
        # 【引数】 name: 行動木のログや可視化で識別するノード名。
        # 【引数】 target_state: 条件成立とみなす合流・分岐の目標状態。
        super(IsJunction, self).__init__(name)
        self.target_state = target_state
        self.reached = False
        self.prev_roe = 0
        self.state:JState = JState.INITIAL
        self.running = False

    # 【日本語解説】 IsJunctionの処理を1制御周期分実行し、行動木へRUNNING・SUCCESS・FAILUREの状態を返す。
    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.scan started" % (g_plotter.get_distance(), self.__class__.__name__))
        roe = g_video.get_range_of_edges()
        if roe != 0:
            if self.state == JState.INITIAL:
                if (self.target_state == JState.JOINING or self.target_state == JState.JOINED) and roe >= JUNCT_UPPER_THRESH and self.prev_roe <= JUNCT_LOWER_THRESH:
                    self.logger.info("%+06d %s.lines are joining" % (g_plotter.get_distance(), self.__class__.__name__))
                    self.state = JState.JOINING
                elif (self.target_state == JState.FORKING or self.target_state == JState.FORKED) and roe >= JUNCT_LOWER_THRESH and self.prev_roe <= JUNCT_LOWER_THRESH:
                    self.logger.info("%+06d %s.lines are forking" % (g_plotter.get_distance(), self.__class__.__name__))
                    self.state = JState.FORKING
            elif self.state == JState.JOINING:
                if roe <= JUNCT_LOWER_THRESH:
                    self.logger.info("%+06d %s.the join completed" % (g_plotter.get_distance(), self.__class__.__name__))
                    self.state = JState.JOINED
                    
            elif self.state == JState.FORKING:
                if roe <= JUNCT_LOWER_THRESH and self.prev_roe >= JUNCT_UPPER_THRESH:
                    self.logger.info("%+06d %s.the fork completed" % (g_plotter.get_distance(), self.__class__.__name__))
                    self.state = JState.FORKED
            else:
                pass
        self.prev_roe = roe

        if not self.reached and self.state == self.target_state:
            self.reached = True
            self.logger.info("%+06d %s.target state reached" % (g_plotter.get_distance(), self.__class__.__name__))
            return Status.SUCCESS
        else:
            return Status.RUNNING


# 【日本語解説】 カメラでボトルを捕捉し、方向補正しながら接近してアームで回収する行動ノード。
class CatchBottle(Behaviour):
    """テープを巻いたボトルへ接近し、左右のフロントアーム間へ取り込む。
    
        IDENTIFY: 色帯を検出して色を共有状態へ保存し、追跡対象色を固定する。
        APPROACH: 色帯の方位をPID制御し、ボトルがカメラ死角へ入るまで接近する。
        CATCH: 最後に確定した方位をジャイロで維持し、指定距離だけ直進して回収する。
    """
    IDENTIFY, APPROACH, CATCH = range(3)

    # 【日本語解説】 CatchBottleの設定値と実行中に保持する状態を初期化する。
    def __init__(self, name: str, power: int,
                 pid_p: float, pid_i: float, pid_d: float,
                 catch_run_mm: int = 150,
                 lock_color: 'BottleColor' = None,  # 指定時は画像判定を待たず、このボトル色へ追跡を固定する。
                 identify_area: int = 400,  # ボトル色の確定に必要な色帯輪郭の最小面積。
                 identify_frames: int = 3,  # ボトル色を確定するために必要な連続検出数。
                 heading_avg_frames: int = 5,  # ボトル消失前の進行方位を平均するフレーム数。
                 ) -> None:
        # 【引数】 name: 行動木のログや可視化で識別するノード名。
        # 【引数】 power: 走行時の基準モーター出力。
        # 【引数】 pid_p: PID制御の比例ゲイン。現在の誤差に対する補正量を決める。
        # 【引数】 pid_i: PID制御の積分ゲイン。継続する偏差を補正する。
        # 【引数】 pid_d: PID制御の微分ゲイン。誤差の急変や振動を抑える。
        # 【引数】 catch_run_mm: ボトルが死角へ入った後、回収のために直進する距離（mm）。
        # 【引数】 lock_color: 探索開始時から固定するボトル色。Noneなら画像から識別する。
        # 【引数】 identify_area: ボトル色を確定するために必要な色帯輪郭の最小面積。
        # 【引数】 identify_frames: ボトル色確定に必要な連続検出フレーム数。
        # 【引数】 heading_avg_frames: 接近方位の平滑化に使用する直近フレーム数。
        super(CatchBottle, self).__init__(name)
        self.power = power
        self.pid_p, self.pid_i, self.pid_d = pid_p, pid_i, pid_d
        self.catch_run_mm = catch_run_mm
        self.lock_color = lock_color
        self.identify_area = identify_area
        self.identify_frames = identify_frames
        self.heading_avg_frames = heading_avg_frames
        self._state = self.IDENTIFY
        self._solid = 0
        self._heading_hist = []
        self._catch_start_odo = None
        self._target_heading = None
        self._blind_steer = 0
        self.pid = None
        self.running = False

    # 【日本語解説】 CatchBottleで_cur_headingに対応する処理を実行する。
    def _cur_heading(self) -> int:
        return (-1) * g_course * g_gyro_sensor.get_angle()

    # 【日本語解説】 CatchBottleで_steer_visionに対応する処理を実行する。
    def _steer_vision(self, theta: float) -> None:
        # PID制御器から現在の誤差に対応する補正量を求める。
        # 【引数】 theta: カメラが求めた対象物への方位角。
        turn = int(self.pid(theta))
        g_right_motor.set_power(self.power + turn)
        g_left_motor.set_power(self.power - turn)

    # 【日本語解説】 CatchBottleの処理を1制御周期分実行し、行動木へRUNNING・SUCCESS・FAILUREの状態を返す。
    def update(self) -> Status:
        global g_bottle_color

        if not self.running:
            self.running = True
            g_video.set_target_interested(TargetInterested.BOTTLE)
            if self.lock_color is not None:
                g_video.set_bottle_color(self.lock_color)
            self.pid = PID(self.pid_p, self.pid_i, self.pid_d, setpoint=0,
                           sample_time=EXEC_INTERVAL,
                           output_limits=(-self.power, self.power))
            self.logger.info("%+06d %s.started" % (g_plotter.get_distance(), self.__class__.__name__))

        insight, color, bcx, btheta, bbottom, barea, in_blind = g_video.get_bottle_stamped()

        # 現在の状態と判定条件に応じて後続処理を分岐する。
        if self._state == self.IDENTIFY:
            self._solid = self._solid + 1 if (insight and barea >= self.identify_area) else 0
            self._steer_vision(btheta if insight else 0.0)  # この行で指定する値の用途を示す。
            if self._solid >= self.identify_frames:
                g_bottle_color = color
                g_video.set_bottle_color(color)  # この行で指定する値の用途を示す。
                self.logger.info("%+06d %s.color=%s area=%d -> APPROACH" % (
                    g_plotter.get_distance(), self.__class__.__name__, color.name, barea))
                self._state = self.APPROACH
            return Status.RUNNING

        # 現在の状態と判定条件に応じて後続処理を分岐する。
        if self._state == self.APPROACH:
            if insight:
                self._blind_steer = 0
                self._steer_vision(btheta)
                self._heading_hist.append(self._cur_heading())  # この行で指定する値の用途を示す。
                if len(self._heading_hist) > self.heading_avg_frames:
                    self._heading_hist.pop(0)
            else:
                self._blind_steer += 1                            # 短時間の見失いでは直前の関心領域を段階的に広げ、実際の移動先で再捕捉する
                g_right_motor.set_power(self.power)
                g_left_motor.set_power(self.power)

            # 現在の状態と判定条件に応じて後続処理を分岐する。
            if in_blind or (not insight and self._blind_steer > 8):
                hist = self._heading_hist or [self._cur_heading()]
                self._target_heading = sum(hist) / len(hist)
                self._catch_start_odo = g_plotter.get_distance()
                self.pid = PID(self.pid_p, self.pid_i, self.pid_d,
                               setpoint=self._target_heading,
                               sample_time=EXEC_INTERVAL,
                               output_limits=(-self.power, self.power))
                self.logger.info("%+06d %s.blind edge, heading=%.1f -> CATCH(+%dmm)" % (
                    g_plotter.get_distance(), self.__class__.__name__,
                    self._target_heading, self.catch_run_mm))
                self._state = self.CATCH
            return Status.RUNNING

        # 現在の状態と判定条件に応じて後続処理を分岐する。
        if self._state == self.CATCH:
            travelled = g_plotter.get_distance() - self._catch_start_odo
            if travelled >= self.catch_run_mm:
                g_right_motor.set_power(0)
                g_left_motor.set_power(0)
                self.logger.info("%+06d %s.caught (ran %dmm, color=%s)" % (
                    g_plotter.get_distance(), self.__class__.__name__,
                    travelled, g_bottle_color.name))
                return Status.SUCCESS
            turn = int(self.pid(self._cur_heading()))
            g_right_motor.set_power(self.power + g_course * turn)
            g_left_motor.set_power(self.power - g_course * turn)
            return Status.RUNNING

        return Status.RUNNING


# 【日本語解説】 指定色のボトルがカメラ視野内へ安定して入ったかを判定する条件ノード。
class IsBottleInsight(Behaviour):
    """指定色のボトルが視野内にある間はSUCCESS、それ以外はFAILUREを返す。
    
        BottleColor.NONEなら検出した任意色を許可する。色指定時は、十分な面積の色帯が一致した場合だけ成功する。
        単発の色揺れを抑えるため、指定フレーム数の連続検出を要求する。
    """
    # 【日本語解説】 IsBottleInsightの設定値と実行中に保持する状態を初期化する。
    def __init__(self, name: str, color: 'BottleColor',
                 min_area: int = 150,  # ボトル検出を有効とみなす輪郭面積の下限。
                 min_frames: int = 2,  # 一時的な色揺れを除外するために必要な連続検出数。
                 set_target: bool = True,  # 有効時は判定開始と同時にカメラをボトル認識へ切り替える。
                 ) -> None:
        # 【引数】 name: 行動木のログや可視化で識別するノード名。
        # 【引数】 color: 検出・照合の対象とする色。
        # 【引数】 min_area: 有効なボトル検出と判断する輪郭の最小面積。
        # 【引数】 min_frames: 一時的な誤検出を除外するために必要な連続検出数。
        # 【引数】 set_target: Trueの場合、判定開始時に画像処理をボトル認識モードへ切り替える。
        super(IsBottleInsight, self).__init__(name)
        self.color = color
        self.min_area = min_area
        self.min_frames = min_frames
        self.set_target = set_target
        self._hits = 0
        self.running = False

    # 【日本語解説】 IsBottleInsightの処理を1制御周期分実行し、行動木へRUNNING・SUCCESS・FAILUREの状態を返す。
    def update(self) -> Status:
        if not self.running:
            self.running = True
            if self.set_target:
                g_video.set_target_interested(TargetInterested.BOTTLE)
            self.logger.info("%+06d %s.watching for color=%s" % (
                g_plotter.get_distance(), self.__class__.__name__, self.color.name))

        insight, color, bcx, btheta, bbottom, barea, in_blind = g_video.get_bottle_stamped()

        match = (insight
                 and barea >= self.min_area
                 and (self.color == BottleColor.NONE or color == self.color))

        self._hits = self._hits + 1 if match else 0

        if self._hits >= self.min_frames:
            return Status.SUCCESS
        return Status.FAILURE


# 【日本語解説】 回収済みボトルの色が指定条件と一致するかを判定する条件ノード。
class HasCaughtBottle(Behaviour):
    """CatchBottleが保存した回収済みボトル色と指定色を比較し、一致時にSUCCESSを返す。"""
    # 【日本語解説】 HasCaughtBottleの設定値と実行中に保持する状態を初期化する。
    def __init__(self, name: str, color: 'BottleColor') -> None:
        # 【引数】 name: 行動木のログや可視化で識別するノード名。
        # 【引数】 color: 検出・照合の対象とする色。
        super(HasCaughtBottle, self).__init__(name)
        self.color = color

    # 【日本語解説】 HasCaughtBottleの処理を1制御周期分実行し、行動木へRUNNING・SUCCESS・FAILUREの状態を返す。
    def update(self) -> Status:
        if self.color == BottleColor.NONE:
            caught = (g_bottle_color != BottleColor.NONE)
        else:
            caught = (g_bottle_color == self.color)

        self.logger.info("%+06d %s.want=%s caught=%s -> %s" % (
            g_plotter.get_distance(), self.__class__.__name__,
            self.color.name, g_bottle_color.name,
            "SUCCESS" if caught else "FAILURE"))

        return Status.SUCCESS if caught else Status.FAILURE


# 【日本語解説】 各制御周期で自己位置を更新した後、行動木を1回だけ進める呼び出し可能オブジェクト。
class TraverseBehaviourTree(object):
    # 【日本語解説】 TraverseBehaviourTreeの設定値と実行中に保持する状態を初期化する。
    def __init__(self, tree: BehaviourTree) -> None:
        # 【引数】 tree: 制御周期ごとに実行するpy_treesの行動木。
        self.tree = tree
        self.last_log_time = None
        self.running = False
    # 【日本語解説】 TraverseBehaviourTreeを関数のように呼び出し、入力値に対する処理結果を返す。
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
    ) -> None:
        # 【引数】 hub: SPIKEハブを操作・参照するデバイスオブジェクト。
        # 【引数】 arm_motor: アーム駆動用モーター。
        # 【引数】 right_motor: 右車輪駆動用モーター。
        # 【引数】 left_motor: 左車輪駆動用モーター。
        # 【引数】 touch_sensor: 走行開始などの入力に使うタッチセンサー。
        # 【引数】 color_sensor: 路面のHSV値・反射光値を読むカラーセンサー。
        # 【引数】 sonar_sensor: 前方障害物までの距離を読む超音波センサー。
        # 【引数】 gyro_sensor: 機体の旋回角・角速度を読むジャイロセンサー。
        global g_hub, g_arm_motor, g_right_motor, g_left_motor, g_touch_sensor, g_color_sensor, g_sonar_sensor, g_gyro_sensor, g_plotter
        if not self.running:
            g_hub = hub
            g_arm_motor = arm_motor
            g_right_motor = right_motor
            g_left_motor = left_motor
            g_touch_sensor = touch_sensor
            g_color_sensor = color_sensor
            g_sonar_sensor = sonar_sensor
            g_gyro_sensor = gyro_sensor
            g_plotter = Plotter()
            print(" -- TraverseBehaviorTree initialization complete")
            self.running = True
        else:
            self.tree.tick_once()
            g_plotter.plot(hub, arm_motor, right_motor, left_motor, touch_sensor, color_sensor, sonar_sensor, gyro_sensor)
            # 現在の制御状態に必要な値を更新し、次の処理へ進む。
            #if self.last_log_time == None or time.time() - self.last_log_time >= 1.0:
            #    print(" --  estimated position X=%d, Y=%d" % (g_plotter.get_loc_x(), g_plotter.get_loc_y()))
            #    self.last_log_time = time.time()


# 【日本語解説】 Webカメラ画像処理を走行制御とは別周期で継続実行するスレッド。
class VideoThread(threading.Thread):
    # 【日本語解説】 VideoThreadの設定値と実行中に保持する状態を初期化する。
    def __init__(self):
        super().__init__()
        self._stop_event = threading.Event()
        self.prev_time = time.time()

    # 【日本語解説】 VideoThreadへ停止要求を通知する。
    def stop(self):
        self._stop_event.set()

    # 【日本語解説】 VideoThreadのスレッド本体として、停止要求まで周期処理を繰り返す。
    def run(self):
        while not self._stop_event.is_set():
            g_video.process(g_plotter, g_hub, g_arm_motor, g_right_motor, g_left_motor, g_color_sensor, g_sonar_sensor, g_gyro_sensor)
            current_time = time.time()
            elapsed_time = current_time - self.prev_time
            self.prev_time = current_time
            if elapsed_time < VIDEO_INTERVAL:
                time.sleep(VIDEO_INTERVAL - elapsed_time)


# 【日本語解説】 競技コースの走行手順をpy_treesのノードとして組み立て、実行用の行動木を返す。
def build_behaviour_tree() -> BehaviourTree:
    root = Sequence(name="2026 base", memory=True)
    calibration = Sequence(name="calibration", memory=True)
    start = Parallel(name="start", policy=ParallelPolicy.SuccessOnOne())
    lap2 = Parallel(name="lap2", policy=ParallelPolicy.SuccessOnOne())
    lap3 = Parallel(name="lap3", policy=ParallelPolicy.SuccessOnOne())
    carry1 = Parallel(name="carry1", policy=ParallelPolicy.SuccessOnOne())
    carry2 = Parallel(name="carry2", policy=ParallelPolicy.SuccessOnOne())
    carry3 = Parallel(name="carry3", policy=ParallelPolicy.SuccessOnOne())
    carry4 = Parallel(name="carry4", policy=ParallelPolicy.SuccessOnOne())
    qr2 = Parallel(name="qr2", policy=ParallelPolicy.SuccessOnOne())
    qr3 = Parallel(name="qr3", policy=ParallelPolicy.SuccessOnOne())
    qr4 = Parallel(name="qr4", policy=ParallelPolicy.SuccessOnOne())
    qr5 = Parallel(name="qr4", policy=ParallelPolicy.SuccessOnOne())
    qr_read = Parallel(name="qr_read", policy=ParallelPolicy.SuccessOnOne())
    qr_scan_shake = Sequence(name="qr_scan_shake", memory=True)
    qr_scan_move_back = Parallel(name="qr_scan_move_back2", policy=ParallelPolicy.SuccessOnOne())
    calibration.add_children(
        [
            # 【子ノード】アームを指定方向の機械端まで動かす。
            ArmUpDownFull(name="arm up", direction=ArmDirection.UP),
            # 【子ノード】アームを指定方向の機械端まで動かす。
            ArmUpDownFull(name="arm down", direction=ArmDirection.DOWN),
            # 【子ノード】モーター、ジャイロ、カメラ設定を初期化する。
            ResetDevice(name="device reset"),
            #ReadKey(name="read key"),
        ]
    )
    start.add_children(
        [
            # 【子ノード】タッチセンサーが押されるまで開始を待つ。
            IsTouchOn(name="touch start"),
        ]
    )
    lap2.add_children(
        [
            # 【子ノード】指定条件でカラーセンサーによるライン追従を行う。
            TraceLine(name="sensor trace normal edge", target=TRACELINE_TARGET_V,
                power=70, power_min=33,
                pid_p=0.65, pid_i=0.000001, pid_d=0.045,
                err_lo=6, err_hi=16, decel_per_s=350, gains_slow=(0.65, 0.045), gains_fast=(0.55, 0.065),
                recover_v=97, recover_after=3, recover_turn=35,
                trace_side=TraceSide.NORMAL),
            # 【子ノード】カラーセンサーが指定色を検出したかを判定する。
            IsColorDetected(name="check color", color=Color.BLUE),
        ]
    )
    lap3.add_children(
        [
            # 【子ノード】ジャイロで方位を補正しながら指定方向へ直進する。
            RunByGyro(name="run straight to catch the bottle", target=3, power=33,
                pid_p=1.1, pid_i=0.1, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
            # 【子ノード】指定した走行距離へ到達したかを判定する。
            IsDistanceEarned(name="check distance", delta_dist = 370),
        ]
    )
    carry1.add_children(
        [
            # 【子ノード】指定条件でカラーセンサーによるライン追従を行う。
            TraceLine(name="sensor trace normal edge", target=TRACELINE_TARGET_V,
                power=70, power_min=33,
                pid_p=0.65, pid_i=0.000001, pid_d=0.045,
                err_lo=6, err_hi=16, decel_per_s=350, gains_slow=(0.65, 0.045), gains_fast=(0.55, 0.065),
                recover_v=97, recover_after=3, recover_turn=35,
                trace_side=TraceSide.NORMAL),
            # 【子ノード】カラーセンサーが指定色を検出したかを判定する。
            IsColorDetected(name="check color", color=Color.BLUE),
        ]
    )
    carry2.add_children(
        [
            # 【子ノード】ジャイロで方位を補正しながら指定方向へ直進する。
            RunByGyro(name="run straight to pass the blue line", target=90, power=33,
                pid_p=1.1, pid_i=0.1, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
            # 【子ノード】指定した走行距離へ到達したかを判定する。
            IsDistanceEarned(name="check distance", delta_dist = 120),
        ]
    )
    carry3.add_children(
        [
            # 【子ノード】指定条件でカラーセンサーによるライン追従を行う。
            TraceLine(name="sensor trace normal edge", target=TRACELINE_TARGET_V,
                power=70, power_min=33,
                pid_p=0.65, pid_i=0.000001, pid_d=0.045,
                err_lo=6, err_hi=16, decel_per_s=350, gains_slow=(0.65, 0.045), gains_fast=(0.55, 0.065),
                recover_v=97, recover_after=3, recover_turn=35,
                trace_side=TraceSide.NORMAL),
            # 【子ノード】指定した走行距離へ到達したかを判定する。
            IsDistanceEarned(name="check distance", delta_dist = 1100),
        ]
    )
    carry4.add_children(
        [
            # 【子ノード】指定条件でカラーセンサーによるライン追従を行う。
            TraceLine(name="sensor trace normal edge", target=TRACELINE_TARGET_V,
                power=33,
                pid_p=0.65, pid_i=0.000001, pid_d=0.045,
                err_lo=6, err_hi=16, decel_per_s=350, gains_slow=(0.65, 0.045), gains_fast=(0.55, 0.065),
                recover_v=97, recover_after=3, recover_turn=35,
                trace_side=TraceSide.NORMAL),
            # 【子ノード】カラーセンサーが指定色を検出したかを判定する。
            IsColorDetected(name="check color", color=Color.BLUE),
        ]
    )
    qr2.add_children(
        [
            # 【子ノード】ジャイロで方位を補正しながら指定方向へ直進する。
            RunByGyro(name="run straight to correct heading", target=0, power=33,
                pid_p=1.1, pid_i=0.1, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
            # 【子ノード】指定した走行距離へ到達したかを判定する。
            IsDistanceEarned(name="check distance", delta_dist = 50),
        ]
    )
    qr3.add_children(
        [
            # 【子ノード】指定条件でカラーセンサーによるライン追従を行う。
            TraceLine(name="sensor trace opposite edge", target=TRACELINE_TARGET_V,
                power=33,
                pid_p=0.65, pid_i=0.000001, pid_d=0.045,
                err_lo=6, err_hi=16, decel_per_s=350, gains_slow=(0.65, 0.045), gains_fast=(0.55, 0.065),
                recover_v=97, recover_after=3, recover_turn=35,
                trace_side=TraceSide.OPPOSITE),
            # 【子ノード】指定した走行距離へ到達したかを判定する。
            IsDistanceEarned(name="check distance", delta_dist = 500),
        ]
    )
    qr4.add_children(
        [
            # 【子ノード】指定条件でカラーセンサーによるライン追従を行う。
            TraceLine(name="sensor trace opposite edge", target=TRACELINE_TARGET_V,
                power=70, power_min=33,
                pid_p=0.65, pid_i=0.000001, pid_d=0.045,
                err_lo=6, err_hi=16, decel_per_s=350, gains_slow=(0.65, 0.045), gains_fast=(0.55, 0.065),
                recover_v=97, recover_after=3, recover_turn=35,
                trace_side=TraceSide.OPPOSITE),
            # 【子ノード】カラーセンサーが指定色を検出したかを判定する。
            IsColorDetected(name="check color", color=Color.BLUE),
        ]
    )
    qr5.add_children(
        [
            # 【子ノード】ジャイロで方位を補正しながら指定方向へ直進する。
            RunByGyro(name="run straight to pass half the blue line", target=-90, power=33,
                pid_p=1.1, pid_i=0.1, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
            # 【子ノード】指定した走行距離へ到達したかを判定する。
            IsDistanceEarned(name="check distance", delta_dist = 100),
        ]
    )
    qr_scan_move_back.add_children(
        [
            # 【子ノード】左右モーターへ指定PWMを与えて走行する。
            RunAsInstructed(name="move back a little", pwm_l=-SPIN_MIN_POWER, pwm_r=-SPIN_MIN_POWER),
            # 【子ノード】指定した走行距離へ到達したかを判定する。
            IsDistanceEarned(name="check distance", delta_dist = 50),
        ]
    )
    qr_scan_shake.add_children(
        [
            # 【子ノード】指定時間が経過するまで待つ。
            IsTimePassed(name="wait for a moment", delta_time=3.0),
            # 【子ノード】QRコードとの撮影距離を確保するため、左右輪を後転させて50mm後退する。
            qr_scan_move_back,
            # 【子ノード】左右モーターを停止する。
            StopNow(name="stop"),
            # 【子ノード】指定時間が経過するまで待つ。
            IsTimePassed(name="wait for a moment", delta_time=3.0),
            # 【子ノード】ジャイロ角を監視しながら指定角度まで旋回する。
            SpinAround(name="scan for QR code", target=3, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                pid_p=0.4, pid_i=0.001, pid_d=0.03, target_type=HeadingType.RELATIVE),
            # 【子ノード】左右モーターを停止する。
            StopNow(name="stop"),
            # 【子ノード】指定時間が経過するまで待つ。
            IsTimePassed(name="wait for a moment", delta_time=2.0),
            # 【子ノード】ジャイロ角を監視しながら指定角度まで旋回する。
            SpinAround(name="scan for QR code", target=-6, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                pid_p=0.4, pid_i=0.001, pid_d=0.03, target_type=HeadingType.RELATIVE),
            # 【子ノード】左右モーターを停止する。
            StopNow(name="stop"),
            # 【子ノード】指定時間が経過するまで待つ。
            IsTimePassed(name="wait for a moment", delta_time=2.0),
            # 【子ノード】ジャイロ角を監視しながら指定角度まで旋回する。
            SpinAround(name="scan for QR code", target=3, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                pid_p=0.4, pid_i=0.001, pid_d=0.03, target_type=HeadingType.RELATIVE),
            # 【子ノード】左右モーターを停止する。
            StopNow(name="stop"),
            # 【子ノード】指定時間が経過するまで待つ。
            IsTimePassed(name="wait for a moment", delta_time=3.0),
        ]
    )
    qr_read.add_children(
        [
            # 【子ノード】カメラ画像からQRコードを認識し、ヒントを取得する。
            IsQRDecoded(name="check QR code"),
            # 【子ノード】QRが読めない場合に、後退と左右の小旋回を行ってカメラの撮影位置・角度を変える。
            qr_scan_shake,
        ]
    )
    root.add_children(
        [
            # 【子ノード】アームを上端・下端まで往復させた後、モーター角度・ジャイロ・カメラ設定を初期化する。
            calibration,
            # 【子ノード】タッチセンサーが押されるまで待機し、押されたら競技走行を開始する。
            start,
            # 【子ノード】カラーセンサーで通常側のライン端を高速追従し、青線を検出するまで進む。
            lap2,
            # 【子ノード】ボトル回収地点へ向けて絶対方位3度を維持し、370mm直進する。
            lap3,
            # 【子ノード】ボトル運搬区間で通常側のライン端を高速追従し、青線まで進む。
            carry1,
            # 【子ノード】青線を通過するため、絶対方位90度を維持して120mm直進する。
            carry2,
            # 【子ノード】ボトルを運びながら通常側のライン端を追従し、1100mm走行する。
            carry3,
            # 【子ノード】速度を落として通常側のライン端を追従し、次の青線まで進む。
            carry4,
            # 【子ノード】ジャイロ角を監視しながら指定角度まで旋回する。
            SpinAround(name="about the face", target=10, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                pid_p=0.4, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
            # 【子ノード】QRコード読取区間へ入る前に、絶対方位0度へ補正しながら50mm直進する。
            qr2,
            # 【子ノード】左右モーターを停止する。
            StopNow(name="stop"),
            # 【子ノード】旋回しながらラインを探索し、検出位置へ合わせる。
            SpinAndLocateLine(name="spin and locate line", target=TRACELINE_TARGET_V, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                pid_p=0.4, pid_i=0.001, pid_d=0.03, trace_side=TraceSide.OPPOSITE),
            # 【子ノード】左右モーターを停止する。
            StopNow(name="stop"),
            # 【子ノード】ラインの反対側の端へ持ち替えて追従し、500mm先のQR接近区間まで進む。
            qr3,
            # 【子ノード】反対側のライン端を高速追従し、QR読取位置の目印となる青線まで進む。
            qr4,
            # 【子ノード】青線の中央付近へ機体を合わせるため、絶対方位-90度で100mm直進する。
            qr5,
            # 【子ノード】左右モーターを停止する。
            StopNow(name="stop"),
            # 【子ノード】アームを指定方向の機械端まで動かす。
            ArmUpDownFull(name="arm up", direction=ArmDirection.UP),
            # 【子ノード】ジャイロ角を監視しながら指定角度まで旋回する。
            SpinAround(name="align for QR code scanning", target=0, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                pid_p=0.4, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
            # 【子ノード】左右モーターを停止する。
            StopNow(name="stop"),
            # 【子ノード】QR認識を試し、未認識の間は機体を小刻みに動かして読取可能な画角を探索する。
            qr_read,
            # 【子ノード】アームを指定方向の機械端まで動かす。
            ArmUpDownFull(name="arm down", direction=ArmDirection.DOWN),
            # 【子ノード】左右モーターを停止する。
            StopNow(name="stop"),
            # 【子ノード】全工程完了後の終端状態へ移行する。
            TheEnd(name="end"),
        ]
    )
    return root

# 【日本語解説】 指定バックエンドでETRoboを生成し、SPIKEのハブ、モーター、各センサーをポートへ割り当てる。
def initialize_etrobo(backend: str) -> ETRobo:
    # 【引数】 backend: ETRoboがSPIKEと通信するために使用するバックエンド名。
    return (ETRobo(backend=backend)
            .add_hub('hub')
            .add_device('arm_motor', device_type=Motor, port='C')
            .add_device('right_motor', device_type=Motor, port='A')
            .add_device('left_motor', device_type=Motor, port='B')
            .add_device('touch_sensor', device_type=TouchSensor, port='D')
            .add_device('color_sensor', device_type=ColorSensor, port='E')
            .add_device('sonar_sensor', device_type=SonarSensor, port='F')
            .add_device('gyro_sensor', device_type=GyroSensor, port='')
    )

# 【日本語解説】 カメラ画像処理スレッドを生成して開始する。
def setup_thread():
    global g_video, g_video_thread
    g_video = Video()

    print(" -- starting VideoThread...")
    g_video_thread = VideoThread()
    g_video_thread.start()

# 【日本語解説】 画像処理スレッドへ停止を通知し、終了を待って資源を回収する。
def cleanup_thread():
    global g_video, g_video_thread
    print(" -- stopping VideoThread...")
    g_video_thread.stop()
    g_video_thread.join()

    del g_video

# 【日本語解説】 終了シグナル受信時にモーターと画像処理を安全に停止してプロセスを終了する。
def sig_handler(signum, frame) -> None:
    # 【引数】 signum: 受信した終了シグナルの番号。
    # 【引数】 frame: シグナル受信時点のスタックフレーム。
    sys.exit(1)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('course', choices=['right', 'left'], help='Course to run')
    parser.add_argument('--logfile', type=str, default=None, help='Path to log file')
    args = parser.parse_args()

    if args.course == 'right':
        g_course = -1
    else:
        g_course = 1

    setup_thread()

    #log_tree.level = log_tree.Level.DEBUG
    tree = build_behaviour_tree()
    #display_tree.render_dot_tree(tree)

    signal.signal(signal.SIGTERM, sig_handler)

    try:
        etrobo = initialize_etrobo(backend='raspike_art')
        etrobo.add_handler(TraverseBehaviourTree(tree))
        etrobo.dispatch(interval=EXEC_INTERVAL, logfile=args.logfile)
    finally:
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        cleanup_thread()
        signal.signal(signal.SIGTERM, signal.SIG_DFL)
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        print(" -- exiting...")
