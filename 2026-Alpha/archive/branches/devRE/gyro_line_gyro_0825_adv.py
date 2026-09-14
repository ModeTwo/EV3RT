#8/26更新バージョン
#ライントレース部分を高度バージョンにした
#
#sample_gyro_line_0820からの変更は、ジャイロ⇒ラインから、ジャイロ⇒ライン⇒ジャイロにした
#以下08/20バージョン
#ジャイロ走行からSpinAroundを消してみる
#追加! ライントレース、最初はゆっくりで、途中から早くしてみる
#rootがsequenceなのでsquareの構成はなくてもいいのでは？まあいいか

import sys
import argparse
import time
import threading
import signal
from enum import IntEnum, Enum, auto
from etrobo_python import ETRobo, Hub, Motor, TouchSensor, ColorSensor, SonarSensor, GyroSensor
from simple_pid import PID
from py_trees.trees import BehaviourTree
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees.composites import Parallel
from py_trees.common import ParallelPolicy
from py_trees import (
    display as display_tree,
    logging as log_tree
)
from py_etrobo_util import Video, TraceSide, TargetInterested, Plotter, SymmetricClamper, Color, ColorClassifier

# --- 制御の周期（インターバル）設定 ---
#floatを明示するためにあえて書いている
EXEC_INTERVAL: float = 0.03   # メインの制御ツリーを動かす周期（0.03秒ごと）
VIDEO_INTERVAL: float = 0.02  # バックグラウンドの動画・画像処理を動かす周期（0.02秒ごと）

# --- ロボットの動作に関する定数設定 ---
ARM_SHIFT_PWM = 30          # アームを上下に動かすときのモーター出力（パワー）
JUNCT_UPPER_THREAH = 50     # 交差点（ラインの合流・分岐）を判定するための、エッジ幅の上限しきい値
JUNCT_LOWER_THREAH = 30     # 交差点（ラインの合流・分岐）を判定するための、エッジ幅の下限しきい値
#↑交差点ではラインの幅が太くなるので、幅が30～50になったら交差点かもと判断
SPIN_MAX_POWER = 57         # その場回旋（スピン）するときの最大モーター出力
SPIN_MIN_POWER = 47         # その場回旋（スピン）するときの最低モーター出力
TRACELINE_TARGET_V = 65     # ライントレース時の目標とする輝度値（センサーの真下の明るさ目標）
#↑黒と白の境界走行をしている（輝度を見ている。65だと境界だろうということ）

MAX_POWER = 100
MIN_POWER = 50

# アームの移動方向を定義する列挙型（UPならマイナス方向、DOWNならプラス方向にモーターを回す）
class ArmDirection(IntEnum):
    UP = -1
    DOWN = 1

# 交差点（Junction）の状態を管理するステートマシン用の列挙型
#段階的に状態を把握することで、「合流」「分岐」を判断している
class JState(Enum):
    INITIAL = auto()  # 初期状態
    JOINING = auto()  # ラインが合流しつつある状態
    JOINED = auto()   # 合流が完了した状態
    FORKING = auto()  # ラインが分岐しつつある状態
    FORKED = auto()   # 分岐が完了した状態

# ジャイロセンサー等で向き（Heading）を指定する際の種類
class HeadingType(Enum):
    ABSOLUTE = "absolute"  # 絶対方位（リセット時を基準とした固定の角度）
    RELATIVE = "relative"  # 相対方位（今の向きからプラスマイナス何度動くか）

# --- グローバル変数の定義（プログラム全体、別スレッドからも共有して使う機材や設定） ---
g_plotter: Plotter = None          # 自己位置推定・走行軌跡を計算するオブジェクト
g_hub: Hub = None                  # ロボットのコントロールハブ（基盤）
g_arm_motor: Motor = None          # アーム駆動用モーター
g_right_motor: Motor = None        # 右車輪駆動用モーター
g_left_motor: Motor = None         # 左車輪駆動用モーター
g_touch_sensor: TouchSensor = None  # タッチセンサー（スタートボタン用など）
g_color_sensor: ColorSensor = None  # カラーセンサー（地面の識別用）
g_sonar_sensor: SonarSensor = None  # 超音波センサー（障害物の距離測定用）
g_gyro_sensor: GyroSensor = None    # ジャイロセンサー（車体の向き測定用）
g_course: int = 0                  # 走行コース（右なら -1、左なら 1 を掛けて左右のモーター出力を反転させる）
g_key: str = None                  # 暗号解読等で使用する、入力されたキー文字列


# =============================================================================
# 【ビヘイビアツリー（Behavior Tree）用の各ノード（動作部品）の定義】
# 全てのクラスは py_trees.behaviour.Behaviour クラスの能力を「継承」しています。
# =============================================================================

#__init__を書かない場合、super(TheEnd, self).__init__(name)は勝手に呼ばれる。
#loggeとrunninng=Falseのために__init__()を明示的に書く。
#組んだビヘイビアツリーの最後に配置することで、プログラムが終了せずにctrl+Cを待つ状態を維持することができます。
class TheEnd(Behaviour):
    """プログラムの終了状態を維持し、ctrl+Cを待つためのノード"""
    def __init__(self, name: str):
        super(TheEnd, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.running = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            # ビヘイビアツリーが最後まで到達したことをログに出力
            self.logger.info("%+06d %s.behavior tree exhausted. ctrl+C shall terminate the program" % (g_plotter.get_distance(), self.__class__.__name__))
        return Status.RUNNING  # 終了せず、ずっとRUNNING状態を返し続けて待機する


class ResetDevice(Behaviour):
    """各種モーターの回転角やジャイロセンサーを初期化（リセット）するノード"""
    def __init__(self, name: str):
        super(ResetDevice, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.count = 0

    def update(self) -> Status:
        if self.count == 0:
            # すべてのモーターのカウンターとジャイロの角度を0にリセットする
            g_arm_motor.reset_count()
            g_right_motor.reset_count()
            g_left_motor.reset_count()
            g_gyro_sensor.reset()
            self.logger.info("%+06d %s.resetting..." % (g_plotter.get_distance(), self.__class__.__name__))
            self.logger.info("%+06d %s.waiting for IMU to be stationary..." % (g_plotter.get_distance(), self.__class__.__name__))
        elif self.count > 3:
            # 完全に静止し、リセット状態が数フレーム維持できたら「成功（SUCCESS）」を返して次の動作へ進む
            self.logger.info("%+06d %s.complete" % (g_plotter.get_distance(), self.__class__.__name__))
            return Status.SUCCESS
            
        # 慣性計測装置（IMU / ジャイロ）が完全に静止しているかチェックし、静止していればカウントを進める
        if g_hub.hub_imu_is_stationary():
            self.count += 1
        return Status.RUNNING


class ArmUpDownFull(Behaviour):
    """アームを上限、または下限まで限界まで動かすノード（突っかかって動かなくなるまで回す）"""
    def __init__(self, name: str, direction: ArmDirection):
        super(ArmUpDownFull, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.direction = direction  # 動かす方向（UPかDOWN）
        self.running = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.prev_degree = g_arm_motor.get_count()
            self.logger.info("%+06d %s.start position is %d" % (g_plotter.get_distance(), self.__class__.__name__, self.prev_degree))
            self.count = 0
            # 指定された方向に向けてモーターを回し始める
            g_arm_motor.set_power(ARM_SHIFT_PWM * self.direction)
        else:
            cur_degree = g_arm_motor.get_count()
            # 「前回の角度と今回の角度の差が5度未満」＝アームが限界まで達してロック（停止）したかチェック
            if abs(cur_degree - self.prev_degree) < 5:
                if self.count > 10:  # 10フレーム連続で動いていなければ完全に限界位置に到達したとみなす
                    g_arm_motor.set_power(0)      # モーターを停止
                    g_arm_motor.set_brake(True)   # その位置をがっちりキープ（ブレーキ）
                    self.logger.info("%+06d %s.position set to %d" % (g_plotter.get_distance(), self.__class__.__name__, cur_degree))
                    return Status.SUCCESS        # 成功して次の動作へ
                else:
                    self.count += 1
            self.prev_degree = cur_degree
        return Status.RUNNING


class ReadKey(Behaviour):
    """コンソール（キーボード）から解読用キー入力を求めるデバッグ・ミッション用ノード"""
    def __init__(self, name: str):
        super(ReadKey, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.running = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            return Status.RUNNING
        else:
            global g_key
            g_key = input("Enter the given key for decryption: ")
            # バリデーション：4文字でなければ再入力を促す
            if len(g_key) != 4:
                self.logger.warning("%+06d %s.invalid key length: %d. key should be 4 characters long." % (g_plotter.get_distance(), self.__class__.__name__, len(g_key)))
                return Status.RUNNING
            else:
                self.logger.info("%+06d %s.entered key: %s" % (g_plotter.get_distance(), self.__class__.__name__, g_key))
                confirmation = input("Is the entered key correct? (y/n): ")
                if confirmation.lower() == 'y':
                    self.logger.info("%+06d %s.key confirmed" % (g_plotter.get_distance(), self.__class__.__name__))
                    return Status.SUCCESS
                else:
                    self.logger.info("%+06d %s.key rejected, please enter again" % (g_plotter.get_distance(), self.__class__.__name__))
                    return Status.RUNNING


class IsTimePassed(Behaviour):
    """指定された時間（秒）が経過したかどうかを判定するタイマー（条件付き待機）ノード"""
    def __init__(self, name: str, delta_time: int):
        super(IsTimePassed, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.delta_time = delta_time  # 待ちたい秒数
        self.running = False
        self.earned = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.orig_time = time.time()  # 開始時刻を記録
            self.logger.info("%+06d %s.accumulation started for delta=%d" % (self.orig_time, self.__class__.__name__, self.delta_time))
        cur_time = time.time()
        earned_time = cur_time - self.orig_time  # 経過時間を計算
        if earned_time >= self.delta_time:
            if not self.earned:
                self.earned = True
                self.logger.info("%+06d %s.delta time passed" % (g_plotter.get_distance(), self.__class__.__name__))
            return Status.SUCCESS  # 指定時間経ったら SUCCESS
        else:
            return Status.RUNNING  # 経ってなければ RUNNING でループを維持


class IsDistanceEarned(Behaviour):
    """ロボットが指定された距離（ミリメートル）を進んだかどうかを判定するノード"""
    def __init__(self, name: str, delta_dist: int):
        super(IsDistanceEarned, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.delta_dist = delta_dist  # 進みたい距離（mm）
        self.running = False
        self.earned = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.orig_dist = g_plotter.get_distance()  # 開始時点の走行距離（オドメーター）を取得
            self.logger.info("%+06d %s.accumulation started for delta=%d" % (self.orig_dist, self.__class__.__name__, self.delta_dist))
        cur_dist = g_plotter.get_distance()
        earned_dist = cur_dist - self.orig_dist  # 進んだ距離を算出（前進・後退どちらにも対応）
        if (earned_dist >= self.delta_dist or -earned_dist <= -self.delta_dist):
            if not self.earned:
                self.earned = True
                self.logger.info("%+06d %s.delta distance earned" % (cur_dist, self.__class__.__name__))
            return Status.SUCCESS  # 目標距離に達したら SUCCESS
        else:
            return Status.RUNNING


class IsColorDetected(Behaviour):
    """カラーセンサーで地面の特定の特定色（青など）を検知したかを判定するノード"""
    def __init__(self, name: str, color: Color):
        super(IsColorDetected, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.color = color              # 待ち構えるターゲットの色
        self.prevColor = Color.UNKNOWN
        self.classifier = ColorClassifier()  # HSVから色を判定する識別器
        self.running = False
        self.detected = False

    def update(self) -> Status:
        cur_dist = g_plotter.get_distance()
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.detection started for color=%s" % (cur_dist, self.__class__.__name__, self.color.value))
        
        # カラーセンサーから生のHSV（色相・彩度・明度）を取得
        h, s, v = g_color_sensor.get_raw_color_hsv()

        # HSV値をもとに、現在の色を「BLUE」「BLACK」などに変換
        detected_color = self.classifier.classify(h, s, v)
        if detected_color == self.color:
            if not self.detected:
                self.detected = True
                self.logger.info("%+06d %s.color=%s detected" % (cur_dist, self.__class__.__name__, self.color.value))
            return Status.SUCCESS  # 狙った色を検知できたら SUCCESS
        else:
            # ログが埋め尽くされないよう、色が変わった瞬間だけ変化ログを吐く工夫
            if detected_color != self.prevColor:
                if detected_color != Color.UNKNOWN or self.prevColor != Color.UNKNOWN:
                    self.logger.info("%+06d %s.color changed from %s to %s" % (cur_dist, self.__class__.__name__, self.prevColor.value, detected_color.value))
                    self.prevColor = detected_color
            return Status.RUNNING


class IsQRDecoded(Behaviour):
    """カメラを使って、正面のQRコードの読み取りに成功したかを判定するノード"""
    def __init__(self, name: str):
        super(IsQRDecoded, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.running = False
        self.detected = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            # グローバル変数のビデオオブジェクトに対して「QRコード認識モード」へ切り替えるよう指示
            g_video.set_target_interested(TargetInterested.QRCODE)
            self.logger.info("%+06d %s.detection started for QR code" % (g_plotter.get_distance(), self.__class__.__name__))
        
        # バックグラウンドの別スレッドが解読した最新のQRテキストを取得
        text = g_video.get_QR_text()
        if text != "":  # 空文字でなければ解読成功！
            if not self.detected:
                self.detected = True
                self.logger.info("%+06d %s.QR code decoded: %s" % (g_plotter.get_distance(), self.__class__.__name__, text))
            return Status.SUCCESS  # 次のステップへ
        else:
            return Status.RUNNING


class IsSonarOn(Behaviour):
    """超音波センサーで、前方に障害物（ペットボトル等）が指定距離以内に近づいたか判定するノード"""
    def __init__(self, name: str, alert_dist: int):
        super(IsSonarOn, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.alert_dist = alert_dist  # 警戒距離（cm）
        self.running = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.detection started for dist=%d" % (g_plotter.get_distance(), self.__class__.__name__, self.alert_dist))
        
        # 超音波センサーから前方物体までの距離を取得
        dist = g_sonar_sensor.get_distance()
        if (dist <= self.alert_dist and dist > 0):  # 0より大きく、指定距離以下なら捕捉とみなす
            self.logger.info("%+06d %s.alerted at dist=%d" % (g_plotter.get_distance(), self.__class__.__name__, dist))
            return Status.SUCCESS
        else:
            return Status.RUNNING


class IsTouchOn(Behaviour):
    """タッチセンサーのボタンが物理的にカチッと押されたかを判定するノード（主にスタート合図用）"""
    def __init__(self, name: str):
        super(IsTouchOn, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.running = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.waiting for touch..." % (g_plotter.get_distance(), self.__class__.__name__))
        
        # ボタンが押し込まれていれば SUCCESS を返して即座に発進させる
        if g_touch_sensor.is_pressed():
            self.logger.info("%+06d %s.pressed" % (g_plotter.get_distance(), self.__class__.__name__))
            return Status.SUCCESS
        else:
            return Status.RUNNING


class StopNow(Behaviour):
    """左右の車輪モーターのパワーを0にし、その場で緊急停止・ブレーキをかけるノード"""
    def __init__(self, name: str):
        super(StopNow, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def update(self) -> Status:
        g_right_motor.set_power(0)
        g_right_motor.set_brake(True)
        g_left_motor.set_power(0)
        g_left_motor.set_brake(True)
        self.logger.info("%+06d %s.motors stopped" % (g_plotter.get_distance(), self.__class__.__name__))
        return Status.SUCCESS


class RunAsInstructed(Behaviour):
    """指定された左右の固定パワー（PWM値）で、ひたすら車輪を回し駆動させる単純移動ノード"""
    def __init__(self, name: str, pwm_l: int, pwm_r: int) -> None:
        super(RunAsInstructed, self).__init__(name)
        # コース設定（右コース=-1、左コース=1）を掛けることで、左右の旋回や進行方向を自動でコースにアジャストする
        self.pwm_l = g_course * pwm_l
        self.pwm_r = g_course * pwm_r
        self.running = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.started with pwm=(%s, %s)" % (g_plotter.get_distance(), self.__class__.__name__, self.pwm_l, self.pwm_r))
        g_right_motor.set_power(self.pwm_r)
        g_left_motor.set_power(self.pwm_l)
        return Status.RUNNING


class TraceLine(Behaviour):
    def __init__(self, name: str, target: int, power: int, pid_p: float, pid_i: float, pid_d: float,
                 trace_side: TraceSide,
                 # low-pass filter parameters
                 cutoff_hz: float = 12.0, median_window: int = 0,
                 # adaptive speed parameters
                 power_min: int = None,          # floor speed; None = constant speed (old behaviour)
                 err_lo: float = 6.0,            # rolling |err| at/below which we run full speed
                 err_hi: float = 22.0,           # rolling |err| at/above which we run power_min
                 accel_per_s: float = 60.0,      # how fast we may speed up   (gentle)
                 decel_per_s: float = 180.0,     # how fast we may slow down  (quick = pseudo lookahead)
                 metric_hz: float = 2.0,
                 # ---- gain scheduling: interpolate gains on current speed ----
                 # give (Kp, Kd) at the slow (curve) end and the fast (straight) end;
                 # None -> no scheduling, the fixed pid_p/pid_d above are used everywhere.
                 gains_slow: tuple = None,       # (Kp, Kd) at power_min
                 gains_fast: tuple = None,       # (Kp, Kd) at power_max
                 # ---- line-lost recovery (outer-edge curve rescue) ----
                 recover_v: int = None,           # bright-rail v that means "line lost to floor"; None = off
                 recover_after: int = 3,          # consecutive lost samples before hard recovery
                 recover_turn: int = None         # recovery steering magnitude; None = power_max
                ) -> None:
        super(TraceLine, self).__init__(name)
        # power is treated as the MAX/nominal speed. The PID output limit is
        # pinned to +/-power_max so steering authority does NOT shrink when the
        # base speed drops on a curve (that was the trap in the original code).
        self.power_max = power
        self.power_min = power if power_min is None else power_min
        self.power = power
        self.adapt = power_min is not None
        self.target = target
        self.pid = PID(pid_p, pid_i, pid_d, setpoint=target, sample_time=EXEC_INTERVAL, output_limits=(-self.power_max, self.power_max))
        self.trace_side = trace_side
        self.lpf = (LowPassFilter(cutoff_hz, EXEC_INTERVAL, median_window) if cutoff_hz else None) # when cutoff_hz = None, no low-pass filter is applied and the raw PID output is used
        # instability/curviness estimate = smoothed |tracking error|
        self.err_lo, self.err_hi = err_lo, err_hi
        self.metric_lpf = LowPassFilter(metric_hz, EXEC_INTERVAL)
        self.err_metric = 0.0
        # per-step power slew limits (asymmetric: brake fast, accelerate slow)
        self.accel_step = accel_per_s * EXEC_INTERVAL
        self.decel_step = decel_per_s * EXEC_INTERVAL
        # gain schedule: linearly interpolate (Kp, Kd) between the slow and fast
        # anchors as a function of self.power. Ki is left fixed at pid_i.
        self.gains_slow = gains_slow
        self.gains_fast = gains_fast
        self.schedule = (gains_slow is not None and gains_fast is not None
                         and self.power_max > self.power_min)
        # line-lost recovery
        self.recover_v = recover_v
        self.recover_after = recover_after
        self.recover_turn = recover_turn
        self._lost_count = 0

        self.running = False

    def update(self) -> Status:
        if not self.running:
            if self.lpf:
                self.lpf.reset()
            self.metric_lpf.reset()
            self.running = True
            self.logger.info("%+06d %s.trace started with TS=%s" % (g_plotter.get_distance(), self.__class__.__name__, self.trace_side.name))

        h, s, v_raw = g_color_sensor.get_raw_color_hsv()
        v = self.lpf(v_raw) if self.lpf else v_raw

        # ---- adaptive base speed -------------------------------------------
        # Use the TRUE tracking error (target - raw v) as the instability metric,
        # smoothed so the speed reacts to course shape, not to every wobble.
        self.err_metric = self.metric_lpf(abs(self.target - v_raw))
        if self.adapt:
            # map smoothed |error| in [err_lo, err_hi] -> power in [max, min]
            frac = (self.err_metric - self.err_lo) / (self.err_hi - self.err_lo)
            frac = 0.0 if frac < 0.0 else (1.0 if frac > 1.0 else frac)
            target_power = self.power_max - frac * (self.power_max - self.power_min)
            # rate-limit the change (slow down quickly, speed up gently)
            dp = target_power - self.power
            if dp > self.accel_step:
                dp = self.accel_step
            elif dp < -self.decel_step:
                dp = -self.decel_step
            self.power += dp
        # ---- gain scheduling: gains track the current speed -----------------
        kp_now, kd_now = self.pid.Kp, self.pid.Kd
        if self.schedule:
            f = (self.power - self.power_min) / (self.power_max - self.power_min)
            f = 0.0 if f < 0.0 else (1.0 if f > 1.0 else f)
            kp_now = self.gains_slow[0] + f * (self.gains_fast[0] - self.gains_slow[0])
            kd_now = self.gains_slow[1] + f * (self.gains_fast[1] - self.gains_slow[1])
            self.pid.tunings = (kp_now, self.pid.Ki, kd_now)
        # ---- steering (PID already clamped to +/-power_max) ----------------
        if self.trace_side == TraceSide.NORMAL:
            turn = (-1) * g_course * int(self.pid(v))
        else: # TraceSide.OPPOSITE
            turn = g_course * int(self.pid(v))

        # ---- line-lost recovery --------------------------------------------
        # When the sensor pins at the bright rail, target=75 only yields a weak
        # clamped-P turn (Kp*(75-100) ~= -16), too gentle to curl back to a line
        # that curved away on an OUTER edge -> the robot drives off. Detect a
        # SUSTAINED bright-rail pin (not a 1-2 sample weave touch) and steer at
        # full authority in the direction P already (correctly) chose, until the
        # edge is reacquired. The dark rail already recovers on its own.
        if self.recover_v is not None:
            if v_raw >= self.recover_v:
                self._lost_count += 1
            else:
                self._lost_count = 0
            if self._lost_count >= self.recover_after and turn != 0:
                mag = self.power_max if self.recover_turn is None else self.recover_turn
                turn = int(math.copysign(mag, turn))

        # On a sharp slow curve, |turn| may exceed the reduced base speed, so the
        # inner wheel can go to zero/negative -> a tight pivot. That's desired.
        p = int(round(self.power))
        left  = max(-100, min(100, p + turn))    # motors cap at +-100
        right = max(-100, min(100, p - turn))
        g_right_motor.set_power(right)
        g_left_motor.set_power(left)

        # log raw v, filtered vf, error-metric, commanded power, gains, and turn
        #self.logger.info("%+06d %s.color sensor HSV=(%d, %d, %d) vf=%d, em=%d, pwr=%d, kp=%.3f, kd=%.3f, turn=%d" % (
        #    g_plotter.get_distance(), self.__class__.__name__,
        #    h, s, v_raw, int(v), int(self.err_metric), p, kp_now, kd_now, turn))

        return Status.RUNNING


class SpinAndLocateLine(Behaviour):
    def __init__(self, name: str, target: int, max_power: int, min_power: int,
                 pid_p: float, pid_i: float, pid_d: float, trace_side: TraceSide) -> None:
        super(SpinAndLocateLine, self).__init__(name)
        self.target = target
        self.spin_direction = 1 if trace_side == TraceSide.NORMAL else -1
        self.pid_p = pid_p
        self.pid_i = pid_i
        self.pid_d = pid_d
        self.clamper = SymmetricClamper(min_power, max_power)
        self.move_away = True
        self.running = False

    def update(self) -> Status:
        # first to spin to move away from the line, then to locate the line by spinning in the opposite direction
        current_heading = (-1) * g_course * g_gyro_sensor.get_angle()
        if not self.running:
            # spin for 30 degrees to move away from the line
            self.target_heading = current_heading + self.spin_direction * 30
            self.pid = PID(self.pid_p, self.pid_i, self.pid_d, setpoint=self.target_heading, sample_time=EXEC_INTERVAL)
            self.running = True
            self.logger.info("%+06d %s.spin started at heading=%d for %d" % (g_plotter.get_distance(),
                                                                             self.__class__.__name__, current_heading, self.target_heading))
        if self.move_away:
            # spin in the normal direction to move away from the line
            error = float(self.target_heading) - current_heading
            # normalize error to [-180, 180]
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
        else:             # spin in the opposite direction to locate the line
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


class SpinAround(Behaviour):
    def __init__(self, name: str, target: int, max_power: int, min_power: int,
                 pid_p: float, pid_i: float, pid_d: float, target_type: HeadingType) -> None:
        super(SpinAround, self).__init__(name)
        self.target = target
        self.target_type = target_type
        self.pid_p = pid_p
        self.pid_i = pid_i
        self.pid_d = pid_d
        self.clamper = SymmetricClamper(min_power, max_power)
        self.running = False

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
        # normalize error to [-180, 180]
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

class SpinAndLocateLine(Behaviour):
    """一度ラインから外れる方向に旋回（スピン）したのち、逆旋回して再度黒線を「視覚的に捕捉」する高度なノード"""
    def __init__(self, name: str, target: int, max_power: int, min_power: int,
                 pid_p: float, pid_i: float, pid_d: float, trace_side: TraceSide) -> None:
        super(SpinAndLocateLine, self).__init__(name)
        self.target = target
        self.spin_direction = 1 if trace_side == TraceSide.NORMAL else -1
        self.pid_p = pid_p
        self.pid_i = pid_i
        self.pid_d = pid_d
        self.clamper = SymmetricClamper(min_power, max_power) # パワーが弱すぎて不感帯に入ったり、強すぎたりするのを防ぐリミッター
        self.move_away = True  # 最初は「ラインから離れるフェーズ」
        self.running = False

    def update(self) -> Status:
        current_heading = (-1) * g_course * g_gyro_sensor.get_angle()
        if not self.running:
            # フェーズ1: 確実に元のラインを見失う（外に出る）ために、今の向きから30度強制的にスピンさせる目標を設定
            self.target_heading = current_heading + self.spin_direction * 30
            self.pid = PID(self.pid_p, self.pid_i, self.pid_d, setpoint=self.target_heading, sample_time=EXEC_INTERVAL)
            self.running = True
            self.logger.info("%+06d %s.spin started at heading=%d for %d" % (g_plotter.get_distance(),
                                                                             self.__class__.__name__, current_heading, self.target_heading))
        if self.move_away:
            # --- フェーズ1: ラインから外側へ離れる動作 ---
            error = float(self.target_heading) - current_heading
            # 角度の差を -180 ? 180 度の範囲に正規化（補正）
            if error > 180.0: error -= 360.0
            if error < -180.0: error += 360.0
            
            if abs(error) < 2.0:  # 30度しっかり離れ終わったら
                self.logger.info("%+06d %s.move away spin ended at heading=%d" % (g_plotter.get_distance(), self.__class__.__name__, current_heading))
                # フェーズ2へ移行：目標値をジャイロ角度ではなく、探し求める「黒線の明暗度（target）」にスイッチ！
                self.pid = PID(self.pid_p, self.pid_i, self.pid_d, setpoint=self.target, sample_time=EXEC_INTERVAL)
                self.spin_direction *= -1  # 旋回方向を真逆に反転
                self.move_away = False     # 離れるフェーズを終了
                return Status.RUNNING
            power = int(self.clamper.clamp(self.pid(current_heading)))
        else:
            # --- フェーズ2: 逆旋回して床の黒線を探し当てる動作 ---
            h, s, v = g_color_sensor.get_raw_color_hsv()
            error = float(self.target) - v
            if abs(error) < 5.0:  # センサー値が目標の黒線の明暗に重なった瞬間！「線を発見」とみなす
                self.logger.info("%+06d %s.line located at heading=%d" % (g_plotter.get_distance(), self.__class__.__name__, current_heading))        
                return Status.SUCCESS  # 任務完了で SUCCESS
            power = int(self.clamper.clamp(self.pid(v))) * self.spin_direction * (-1)
            
        g_right_motor.set_power(g_course * power)
        g_left_motor.set_power((-1) * g_course * power)
        return Status.RUNNING    


class SpinAround(Behaviour):
    """ジャイロセンサーをフィードバックに用いて、その場で指定角度だけ超精密にスピン（回旋）するノード"""
    def __init__(self, name: str, target: int, max_power: int, min_power: int,
                 pid_p: float, pid_i: float, pid_d: float, target_type: HeadingType) -> None:
        super(SpinAround, self).__init__(name)
        self.target = target
        self.target_type = target_type  # ABSOLUTE(絶対値指定) か RELATIVE(現在の向きからの相対) か
        self.pid_p = pid_p
        self.pid_i = pid_i
        self.pid_d = pid_d
        self.clamper = SymmetricClamper(min_power, max_power)
        self.running = False

    def update(self) -> Status:
        current_heading = (-1) * g_course * g_gyro_sensor.get_angle()
        if not self.running:
            if self.target_type == HeadingType.RELATIVE:
                self.target_heading = current_heading + self.target  # 相対角度で目標決定
            else:
                self.target_heading = self.target                  # 絶対角度で目標決定
            self.pid = PID(self.pid_p, self.pid_i, self.pid_d, setpoint=self.target_heading, sample_time=EXEC_INTERVAL)
            self.running = True
            self.logger.info("%+06d %s.spin started at heading=%d for %d" % (g_plotter.get_distance(),
                                                                             self.__class__.__name__, current_heading, self.target_heading))
        error = float(self.target_heading) - current_heading
        if error > 180.0: error -= 360.0
        if error < -180.0: error += 360.0
        
        if abs(error) < 2.0:  # 目標角度との誤差が2度以内に収まったら旋回完了
            self.logger.info("%+06d %s.spin ended at heading=%d" % (g_plotter.get_distance(), self.__class__.__name__, current_heading))
            return Status.SUCCESS
        power = int(self.clamper.clamp(self.pid(current_heading)))
        g_right_motor.set_power(g_course * power)
        g_left_motor.set_power((-1) * g_course * power)
        return Status.RUNNING    


class RunByGyro(Behaviour):
    """床の線を見ず、ジャイロセンサーの角度だけを頼りに指定の方角へ向かって真っ直ぐ直進するノード"""
    def __init__(self, name: str, target: int, power: int,
                 pid_p: float, pid_i: float, pid_d: float, target_type: HeadingType) -> None:
        super(RunByGyro, self).__init__(name)
        self.target = target
        self.target_type = target_type
        self.power = power  # 直進の前進パワー
        self.pid_p = pid_p
        self.pid_i = pid_i
        self.pid_d = pid_d
        self.last_log_time = None
        self.running = False

    def update(self) -> Status:
        current_heading = (-1) * g_course * g_gyro_sensor.get_angle()
        # 1秒ごとに現在の方角をログに吐き出してデバッグしやすくする
        if self.last_log_time == None or time.time() - self.last_log_time >= 1.0:
            self.logger.info("%+06d %s.current heading=%d" % (g_plotter.get_distance(), self.__class__.__name__, current_heading))
            self.last_log_time = time.time()
        if not self.running:
            if self.target_type == HeadingType.RELATIVE:
                self.target_heading = current_heading + self.target
            else:
                self.target_heading = self.target
            # 算出した旋回量が直進パワーを超えて暴走しないよう、output_limitsでしっかりキャップをかける
            self.pid = PID(self.pid_p, self.pid_i, self.pid_d, setpoint=self.target_heading, sample_time=EXEC_INTERVAL, output_limits=(-self.power, self.power))
            self.logger.info("%+06d %s.gyro run started toward heading=%d" % (g_plotter.get_distance(), self.__class__.__name__, self.target_heading))
            self.running = True
        
        turn = int(self.pid(current_heading))  # まっすぐ走るために必要な微修正の旋回量
        g_right_motor.set_power(self.power + g_course * turn)
        g_left_motor.set_power(self.power - g_course * turn)
        return Status.RUNNING


class TraceLineCam(Behaviour):
    """【前上方のカメラ画像を使用】Video.pyが算出した偏差角（theta）をターゲットにした先進的なライントレース"""
    def __init__(self, name: str, power: int, pid_p: float, pid_i: float, pid_d: float,
                 gs_min: int, gs_max: int, trace_side: TraceSide) -> None:
        super(TraceLineCam, self).__init__(name)
        self.power = power
        # 画面中央に対する線の角度偏差を「0（ズレなし）」に近づけるためのPID
        self.pid = PID(pid_p, pid_i, pid_d, setpoint=0, sample_time=EXEC_INTERVAL, output_limits=(-power, power))
        self.gs_min = gs_min  # 二値化の輝度最小値
        self.gs_max = gs_max  # 二値化の輝度最大値
        self.trace_side = trace_side
        self.running = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            # ビデオオブジェクトに二値化の閾値、モード（LINE検出）、およびトレース側（右・左・中央）を設定
            g_video.set_thresholds(self.gs_min, self.gs_max)
            g_video.set_target_interested(TargetInterested.LINE)
            
            # コース（右・左）に応じて、画像内のエッジ（TraceSide.RIGHT / LEFT）を自動で最適化
            if self.trace_side == TraceSide.NORMAL:
                if g_course == -1: g_video.set_trace_side(TraceSide.RIGHT)
                else: g_video.set_trace_side(TraceSide.LEFT)
            elif self.trace_side == TraceSide.OPPOSITE: 
                if g_course == -1: g_video.set_trace_side(TraceSide.LEFT)
                else: g_video.set_trace_side(TraceSide.RIGHT)
            else:
                g_video.set_trace_side(TraceSide.CENTER)
            self.logger.info("%+06d %s.trace started with TS=%s" % (g_plotter.get_distance(), self.__class__.__name__, self.trace_side.name))
        
        # video.pyが裏で計算してくれている最新の「theta（線のズレ角度）」を取得してPIDに放り込む
        turn = (-1) * int(self.pid(g_video.get_theta()))
        g_right_motor.set_power(self.power - turn)
        g_left_motor.set_power(self.power + turn)
        return Status.RUNNING


class IsJunction(Behaviour):
    """カメラ画像処理で割り出した線の幅（Range of edges）を用いて、交差点や分岐点への到達を検知するノード"""
    def __init__(self, name: str, target_state: JState) -> None:
        super(IsJunction, self).__init__(name)
        self.target_state = target_state  # 待ち構える状態（JOINEDやFORKEDなど）
        self.reached = False
        self.prev_roe = 0
        self.state:JState = JState.INITIAL
        self.running = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.scan started" % (g_plotter.get_distance(), self.__class__.__name__))
        
        # video.pyから最新の「線の横幅ピクセル数（roe）」を取得
        roe = g_video.get_range_of_edges()
        if roe != 0:
            if self.state == JState.INITIAL:
                # 合流待ちの時に、線の幅が急に太くなったら（JUNCT_UPPER_THRESHを超えたら）合流開始と判定
                if (self.target_state == JState.JOINING or self.target_state == JState.JOINED) and roe >= JUNCT_UPPER_THREAH and self.prev_roe <= JUNCT_LOWER_THREAH:
                    self.logger.info("%+06d %s.lines are joining" % (g_plotter.get_distance(), self.__class__.__name__))
                    self.state = JState.JOINING
                # 分岐待ちの時に、線の幅が急に変動し始めたら分岐開始と判定
                elif (self.target_state == JState.FORKING or self.target_state == JState.FORKED) and roe >= JUNCT_LOWER_THREAH and self.prev_roe <= JUNCT_LOWER_THREAH:
                    self.logger.info("%+06d %s.lines are forking" % (g_plotter.get_distance(), self.__class__.__name__))
                    self.state = JState.FORKING
            elif self.state == JState.JOINING:
                # 太かった線がまた元の細さに戻ったら、完全に1本のラインに合流完了（JOINED）したと判定
                if roe <= JUNCT_LOWER_THREAH:
                    self.logger.info("%+06d %s.the join completed" % (g_plotter.get_distance(), self.__class__.__name__))
                    self.state = JState.JOINED
            elif self.state == JState.FORKING:
                # 分岐を通り過ぎてまた線が通常の細さに戻ったら、分岐完了（FORKED）したと判定
                if roe <= JUNCT_LOWER_THREAH and self.prev_roe >= JUNCT_UPPER_THREAH:
                    self.logger.info("%+06d %s.the fork completed" % (g_plotter.get_distance(), self.__class__.__name__))
                    self.state = JState.FORKED
            else:
                pass
        self.prev_roe = roe

        # 自分が目指していた交差点の状態（target_state）に完全に一致したら SUCCESS を返して次のタスクへバトンタッチ
        if not self.reached and self.state == self.target_state:
            self.reached = True
            self.logger.info("%+06d %s.target state reached" % (g_plotter.get_distance(), self.__class__.__name__))
            return Status.SUCCESS
        else:
            return Status.RUNNING


class TraverseBehaviourTree(object):
    """【全体の司令塔】毎周期ロボットから呼ばれ、ビヘイビアツリーを1回進め（tick）、自己位置をプロットするクラス"""
    def __init__(self, tree: BehaviourTree) -> None:
        self.tree = tree
        self.last_log_time = None
        self.running = False
        
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
        global g_hub, g_arm_motor, g_right_motor, g_left_motor, g_touch_sensor, g_color_sensor, g_sonar_sensor, g_gyro_sensor, g_plotter
        if not self.running:
            # 最初の1回目だけ呼ばれる初期化処理。ハードウェア群をすべてグローバル変数へ登録し、Plotter（自己位置推定）を生成
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
            # 2回目以降（毎周期 0.03秒ごと）：
            self.tree.tick_once() # ビヘイビアツリーの条件を1ステップ動かす
            # モーターの回転数やセンサー値から、ロボットが今マップ上のどこにいるか（位置）を毎ステップ計算・更新する（オドメトリ）
            g_plotter.plot(hub, arm_motor, right_motor, left_motor, touch_sensor, color_sensor, sonar_sensor, gyro_sensor)


class VideoThread(threading.Thread):
    """メイン制御とは『別部屋』で動き、カメラ画像処理（video.py）を高速並行実行するためのスレッドクラス"""
    def __init__(self):
        super().__init__()
        # スレッドを安全に外部から終了させるためのシグナルイベントオブジェクトを自前で用意
        self._stop_event = threading.Event()
        self.prev_time = time.time()

    def stop(self):
        # 外部（メイン側）からこのstop()を呼ぶと、内部フラグ（_flag）がTrueになり、裏のループが止まる仕掛け
        self._stop_event.set()

    def run(self):
        # start()が呼ばれたらこのrunが裏で回り出す。stop_eventがONにならない限り、無限にカメラ画像を更新（process）し続ける
        while not self._stop_event.is_set():
            g_video.process(g_plotter, g_hub, g_arm_motor, g_right_motor, g_left_motor, g_color_sensor, g_sonar_sensor, g_gyro_sensor)
            
            # --- CPUパワーを使い果たさないためのウェイト（お昼寝）処理 ---
            current_time = time.time()
            elapsed_time = current_time - self.prev_time
            self.prev_time = current_time
            # 画像処理がVIDEO_INTERVAL（0.02秒＝秒間50フレームペース）より早く終わったら、余った時間分だけsleepしてCPUを休ませる
            if elapsed_time < VIDEO_INTERVAL:
                time.sleep(VIDEO_INTERVAL - elapsed_time)


def build_behaviour_tree() -> BehaviourTree:
    """【ロボットの作戦マップ】どういう順番・条件でミッションをクリアしていくかを構築する巨大な設計図関数"""
    root = Sequence(name="2026 base", memory=True)              # すべての根本となる親シーケンス（順番に実行、クリアしたものは記憶（memory））
    calibration = Sequence(name="calibration", memory=True)    # 機材リセット・調整用の子シーケンス
    start = Parallel(name="start", policy=ParallelPolicy.SuccessOnOne()) # 並行処理ノード（どれか1つがSUCCESSになればクリア）
    edge_01 = Parallel(name="edge_01", policy=ParallelPolicy.SuccessOnOne())#parallelでmemory=trueは持っていないが、sequenceに組み込むことでparallelの結果も覚えられる。
    edge_02 = Parallel(name="edge_02", policy=ParallelPolicy.SuccessOnOne())
    edge_03 = Parallel(name="edge_03", policy=ParallelPolicy.SuccessOnOne())
    edge_04 = Parallel(name="edge_04", policy=ParallelPolicy.SuccessOnOne())
    edge_05 = Parallel(name="edge_05", policy=ParallelPolicy.SuccessOnOne())
    # ジャイロ走行全体
    square = Sequence(name="square", memory=True)
    
    lap2_1 = Parallel(name="lap2_1", policy=ParallelPolicy.SuccessOnOne())#ライントレース
    lap2_3 = Parallel(name="lap2_3", policy=ParallelPolicy.SuccessOnOne())#カーブ終了後のジャイロ走行。
    lap3 = Parallel(name="lap3", policy=ParallelPolicy.SuccessOnOne())
    carry1 = Parallel(name="carry1", policy=ParallelPolicy.SuccessOnOne())
    carry2 = Parallel(name="carry2", policy=ParallelPolicy.SuccessOnOne())
    carry3 = Parallel(name="carry3", policy=ParallelPolicy.SuccessOnOne())
    qr1 = Parallel(name="qr1", policy=ParallelPolicy.SuccessOnOne())
    qr2 = Parallel(name="qr2", policy=ParallelPolicy.SuccessOnOne())
    qr3 = Parallel(name="qr3", policy=ParallelPolicy.SuccessOnOne())
    qr4 = Parallel(name="qr4", policy=ParallelPolicy.SuccessOnOne())
    qr_read = Parallel(name="qr_read", policy=ParallelPolicy.SuccessOnOne())
    qr_scan_shake = Sequence(name="qr_scan_shake", memory=True)
    qr_scan_move_back = Parallel(name="qr_scan_move_back2", policy=ParallelPolicy.SuccessOnOne())
    
    # 1. 機材キャリブレーション（アームを上下に限界まで振り、デバイスをリセット）
    # name を渡してるのはログとかわかりやすくするための物
    calibration.add_children(
        [
            ArmUpDownFull(name="arm up", direction=ArmDirection.UP),
            ArmUpDownFull(name="arm down", direction=ArmDirection.DOWN),
            ResetDevice(name="device reset"),
        ]
    )
    # 2. スタート（タッチセンサーがカチッと押されるまでその場で待機）
    start.add_children(
        [
            IsTouchOn(name="touch start"),
        ]
    )


     # edge_01：角度0°で直進 → 距離500で成功
    edge_01.add_children(
        [
            RunByGyro(
                name="run straight",
                target=0,
                power=70,
                pid_p=1.1,
                pid_i=0.1,
                pid_d=0.03,
                target_type=HeadingType.ABSOLUTE
            ),
            IsDistanceEarned(name="check distance", delta_dist=500),
        ]
    )

     # edge_02：角度45°で直進 → 距離200で成功
    edge_02.add_children(
        [
            RunByGyro(
                name="run straight",
                target=-45,
                power=70,
                pid_p=1.1,
                pid_i=0.1,
                pid_d=0.03,
                target_type=HeadingType.ABSOLUTE
            ),
            IsDistanceEarned(name="check distance", delta_dist=200),
        ]
    )

     # edge_03：角度90°で直進 → 距離550で成功
    edge_03.add_children(
        [
            RunByGyro(
                name="run straight",
                target=-90,
                power=70,
                pid_p=1.1,
                pid_i=0.1,
                pid_d=0.03,
                target_type=HeadingType.ABSOLUTE
            ),
            IsDistanceEarned(name="check distance", delta_dist=550),
        ]
    )

    
    # edge_04：角度135°で直進 → 距離200で成功
    edge_04.add_children(
        [
            RunByGyro(
                name="run straight",
                target=-135,
                power=70,
                pid_p=1.1,
                pid_i=0.1,
                pid_d=0.03,
                target_type=HeadingType.ABSOLUTE
            ),
            IsDistanceEarned(name="check distance", delta_dist=200),
        ]
    )

     
    # edge_05：角度180°で直進 → 距離200で成功
    edge_05.add_children(
         [
             RunByGyro(
                 name="run straight",
                 target=-180,
                 power=60, #ここだけ60
                 pid_p=1.1,
                 pid_i=0.1,
                 pid_d=0.03,
                 target_type=HeadingType.ABSOLUTE
                ),
                IsDistanceEarned(name="check distance", delta_dist=200),
        ]
    )

    
     # ジャイロ走行区間の構成（直進→回転→直進→回転…）
    square.add_children(
        [
            edge_01,
            edge_02,
            edge_03,
            edge_04,
            edge_05
           
        ]
    )

    #lap2_1（3つ目の黄色△までライントレース）power変える?2600mmで合っている？
    lap2_1.add_children(
        [
             TraceLine(name="sensor trace normal edge", target=TRACELINE_TARGET_V,
                power=50, power_min=33,
                pid_p=0.65, pid_i=0.000001, pid_d=0.045,
                err_lo=6, err_hi=16, decel_per_s=350, gains_slow=(0.65, 0.045), gains_fast=(0.55, 0.065),
                recover_v=97, recover_after=3, recover_turn=35,
                trace_side=TraceSide.NORMAL),
            IsDistanceEarned(name="check distance", delta_dist=2600),
        ]
    )

    #lap2_3 (ジャイロ走行。青いマーカーを超えるくらいまで）
    lap2_3.add_children(
        [
            RunByGyro(
                name="run straight",
                target=0,
                power=70,
                pid_p=1.1,
                pid_i=0.1,
                pid_d=0.03,
                target_type=HeadingType.ABSOLUTE
            ),
            IsDistanceEarned(name="check distance", delta_dist=830),
        ]
    )


    """
    power:前進のパワー
    pid_p:ズレに対する反応の強さ（大きい P → せっかちで反応が速い。小さい P → 落ち着いていて慎重）
    pid_i:誤差が長時間続いたときに、少しずつ補正を積み上げていく仕組み（大きい I → 我慢強くズレを直そうとする（でも過剰補正しやすい）。
          小さい I → ほぼ無視する（でも安定する）
    pid_d:誤差の変化の速さ（変化量）に反応して、ロボットの暴れを抑える役割(大きい D → 落ち着いている（でも反応が鈍くなる）
          小さい D → 反応が速い（でも暴れやすい）)
    target:角度
    """

    # 4. 第3区間（ジャイロ直進で線を無視して370mm先のペットボトルの位置まで真っ直ぐ進む）
    lap3.add_children(
        [
            RunByGyro(name="run straight to catch the bottle", target=5, power=33,
                pid_p=1.1, pid_i=0.00075, pid_d=0.04, target_type=HeadingType.ABSOLUTE),
            IsDistanceEarned(name="check distance", delta_dist = 370),
        ]
    )
    # 5. ボトル運搬区間1（ラインに復帰してトレース開始、また次の青線が見えるまで前進）
    carry1.add_children(
        [
            TraceLine(name="sensor trace normal edge", target=TRACELINE_TARGET_V, power=33,
                pid_p=0.55, pid_i=0.0000009, pid_d=0.015, trace_side=TraceSide.NORMAL),
            IsColorDetected(name="check color", color=Color.BLUE),
        ]
    )
    # 6. ボトル運搬区間2（ジャイロを使い、絶対方位90度を向いて青線をまたぎ越すように120mm直進）
    carry2.add_children(
        [
            RunByGyro(name="run straight to pass the blue line", target=90, power=33,
                pid_p=1.1, pid_i=0.00075, pid_d=0.04, target_type=HeadingType.ABSOLUTE),
            IsDistanceEarned(name="check distance", delta_dist = 120),
        ]
    )
    # 7. ボトル運搬区間3（少しスピードを上げてライントレースを再開、再度青線が見えるまで前進）
    carry3.add_children(
        [
            TraceLine(name="sensor trace normal edge", target=TRACELINE_TARGET_V+10, power=33,
                pid_p=0.65, pid_i=0.000001, pid_d=0.011, trace_side=TraceSide.NORMAL),
            IsColorDetected(name="check color", color=Color.BLUE),
        ]
    )
    # 8. QRコード接近区間1（ジャイロ直進を用いて、対向車線側のエッジと平行になるよう50mm進む）
    qr1.add_children(
        [
            RunByGyro(name="run straight to align with opposite edge", target=5, power=33,
                pid_p=1.1, pid_i=0.00075, pid_d=0.04, target_type=HeadingType.ABSOLUTE),
            IsDistanceEarned(name="check distance", delta_dist = 50),
        ]
    )
    # 9. QRコード接近区間2（絶対方位0度に向けて車体の歪みをきれいに補正しながら50mm直進）
    qr2.add_children(
        [
            RunByGyro(name="run straight to correct heading", target=0, power=33,
                pid_p=1.1, pid_i=0.00075, pid_d=0.04, target_type=HeadingType.ABSOLUTE),
            IsDistanceEarned(name="check distance", delta_dist = 50),
        ]
    )
    # 10. QRコード接近区間3（今度は『逆側（Opposite）』のエッジを這いながら、青線を見つけるまでトレース）
    qr3.add_children(
        [
            TraceLine(name="sensor trace opposite edge", target=TRACELINE_TARGET_V+10, power=33,
                pid_p=0.655, pid_i=0.0000011, pid_d=0.012, trace_side=TraceSide.OPPOSITE),
            IsColorDetected(name="check color", color=Color.BLUE),
        ]
    )
    # 11. QRコード接近区間4（絶対方位-90度を向いて、青線の半分くらいの位置を超えるように100mm直進）
    qr4.add_children(
        [
            RunByGyro(name="run straight to pass half the blue line", target=-90, power=33,
                pid_p=1.1, pid_i=0.00075, pid_d=0.04, target_type=HeadingType.ABSOLUTE),
            IsDistanceEarned(name="check distance", delta_dist = 100),
        ]
    )
    # QRコード読み取り用：少し後ろに下がってカメラとの適切なピント・画角の距離を作る動作
    qr_scan_move_back.add_children(
        [
            RunAsInstructed(name="move back a little", pwm_l=-SPIN_MIN_POWER, pwm_r=-SPIN_MIN_POWER),
            IsDistanceEarned(name="check distance", delta_dist = 50),
        ]
    )
    # QRコード読み取り用：カメラがQRを捉えやすくするために、車体を「左右に細かくイヤイヤとシェイク」させる一連の連続シーケンス
    qr_scan_shake.add_children(
        [
            SpinAround(name="scan for QR code", target=4, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                pid_p=0.2, pid_i=0.00075, pid_d=0.03, target_type=HeadingType.RELATIVE),
            StopNow(name="stop"),
            IsTimePassed(name="wait for a moment", delta_time=0.8), # 止まってカメラのブレが収まるのを待つ
            SpinAround(name="scan for QR code", target=-8, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                pid_p=0.2, pid_i=0.00075, pid_d=0.03, target_type=HeadingType.RELATIVE),
            StopNow(name="stop"),
            IsTimePassed(name="wait for a moment", delta_time=0.8),
            SpinAround(name="scan for QR code", target=4, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                pid_p=0.2, pid_i=0.00075, pid_d=0.03, target_type=HeadingType.RELATIVE),
            StopNow(name="stop"),
            IsTimePassed(name="wait for a moment", delta_time=2.0),
            qr_scan_move_back, # 1回シェイクしても読めなければ、50mmバックして仕切り直す
            SpinAround(name="scan for QR code", target=3, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                pid_p=0.2, pid_i=0.00075, pid_d=0.03, target_type=HeadingType.RELATIVE),
            StopNow(name="stop"),
            IsTimePassed(name="wait for a moment", delta_time=0.8),
            SpinAround(name="scan for QR code", target=-6, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                pid_p=0.2, pid_i=0.00075, pid_d=0.03, target_type=HeadingType.RELATIVE),
            StopNow(name="stop"),
            IsTimePassed(name="wait for a moment", delta_time=0.8),
            SpinAround(name="scan for QR code", target=3, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                pid_p=0.2, pid_i=0.00075, pid_d=0.03, target_type=HeadingType.RELATIVE),
            StopNow(name="stop"),
            IsTimePassed(name="wait for a moment", delta_time=2.0),
        ]
    )
    # パラレルノード：QRコードのデコード（IsQRDecoded）を見張りつつ、読めるまで下の「イヤイヤシェイク動作」を並行して回し続ける
    qr_read.add_children(
        [
            IsQRDecoded(name="check QR code"),
            qr_scan_shake,
        ]
    )
    
    # -------------------------------------------------------------------------
    # 各子要素を、大元の根本ルート（Sequence）へ上から順番に一本道として組み立てる
    # -------------------------------------------------------------------------
    root.add_children(
        [
            calibration,      # ① 起動時のアーム・ジャイロ初期化
            start,               # ② タッチセンサー押し下げでのスタート待ち
            square,           #ジャイロ走行
            lap2_1,           #ライントレース 
            lap3,               # ④ 青いマーカーからジャイロ直進でボトルをキャッチしに行く
            carry1,            # ⑤ ボトルを乗せたまま青線までトレース
            carry2,            # ⑥ 直角にジャイロ直進して青線を越える
            carry3,            # ⑦ 再びライントレースで最後の青線まで運搬
            # ⑧ その場でぐるっと回れ右（絶対方位10度を向くようにスピン反転）
            SpinAround(name="about the face", target=10, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                pid_p=0.2, pid_i=0.00075, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
            StopNow(name="stop"),
            IsTimePassed(name="wait for a moment", delta_time=1.0),
            qr2,              # ⑨ 方角を0度に綺麗に整えて少し直進
            StopNow(name="stop"),
            IsTimePassed(name="wait for a moment", delta_time=1.0),
            # ⑩ 逆側エッジ方向へスピンし、床の黒線をセンサーで見つけ出すまで回る
            SpinAndLocateLine(name="spin and locate line", target=TRACELINE_TARGET_V-20, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                pid_p=0.2, pid_i=0.00075, pid_d=0.03, trace_side=TraceSide.OPPOSITE),
            StopNow(name="stop"),
            IsTimePassed(name="wait for a moment", delta_time=1.0),
            qr3,              # ⑪ 見つけたラインを逆側エッジでトレースして青線まで進む
            qr4,              # ⑫ -90度を向いて青線をまたぎ越す
            StopNow(name="stop"),
            IsTimePassed(name="wait for a moment", delta_time=1.0),
            # ⑬ カメラの視界を確保するため、アームを完全に上に跳ね上げる
            ArmUpDownFull(name="arm up", direction=ArmDirection.UP),
            # ⑭ 正面のQRコードボードと正対するように絶対方位0度へ車体を向ける
            SpinAround(name="align for QR code scanning", target=0, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                pid_p=0.2, pid_i=0.00075, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
            qr_read,          # ⑮ 体をシェイクさせながら正面のQRコードをカメラで読み取る（読めるまで次のタスクに進まない）
            # ⑯ 読み終わったらアームを元の位置にガシャ降ろす
            ArmUpDownFull(name="arm down", direction=ArmDirection.DOWN),
            StopNow(name="stop"), # ⑰ モーター完全停止
            TheEnd(name="end"),   # ⑱ プログラムの終了待機状態へ
        ]
    )
    return root

def initialize_etrobo(backend: str) -> ETRobo:
    """ロボットの各ポート（A?F）にどのモーターやセンサーが繋がっているかを設定して起動する関数"""
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

def setup_thread():
    """【起動スイッチ】グローバル変数としてVideoクラスのインスタンス化を行い、別スレッドを立ち上げて並行処理を開始する関数"""
    global g_video, g_video_thread
    g_video = Video()

    print(" -- starting VideoThread...")
    g_video_thread = VideoThread()
    g_video_thread.start() # これにより自動的に裏で VideoThread の run() メソッドが回り出す

def cleanup_thread():
    """【後片付け】プログラム終了時に呼び出され、別スレッドに停止命令を送り、メモリから安全に削除する関数"""
    global g_video, g_video_thread
    print(" -- stopping VideoThread...")
    g_video_thread.stop() # stop_event のフラグをTrueにする（裏のwhileループが終了する）
    g_video_thread.join() # 裏のスレッドが完全に「お部屋を片付けて退去」するのをメイン側で待つ

    del g_video # メモリから削除

def sig_handler(signum, frame) -> None:
    """Linuxシステム等からプログラムの強制終了シグナル（SIGTERM）を受け取った際の割り込みハンドラ"""
    sys.exit(1)

# =============================================================================
# 【メインプログラムのエントリーポイント】
# コマンドラインからこのファイルが直接実行された場合のみ、ここから処理がスタートします。
# =============================================================================
if __name__ == '__main__':
    # 引数パーサーの窓口を作成
    parser = argparse.ArgumentParser()
    # 必須の引数として、走行するコース（rightかleftのいずれか）の入力を要求
    parser.add_argument('course', choices=['right', 'left'], help='Course to run')
    # オプション引数として、ログを保存するファイルパス（省略時はNone）を指定可能にする
    parser.add_argument('--logfile', type=str, default=None, help='Path to log file')
    args = parser.parse_args() # 入力された文字を解析し、args.course / args.logfile として利用可能にする

    # コースが「右」なら、旋回方向などの極性を反転させるため、全体の倍率係数（g_course）を -1 に設定
    if args.course == 'right':
        g_course = -1
    else:
        g_course = 1

    # 1. 裏方仕事（カメラ・画像処理専用のマルチスレッド）をバックグラウンドで発進させる
    setup_thread()

    # 2. 本日走る予定のミッションマップ（ビヘイビアツリー）を一階層ずつ綺麗にビルドする
    tree = build_behaviour_tree()

    # システム終了シグナル（SIGTERM）が飛んできたときに、安全に終了させるための関数（sig_handler）をOSに登録
    signal.signal(signal.SIGTERM, sig_handler)

    try:
        # 3. ハードウェア（RasPikeなどの実機またはシミュレータの通信）を初期化
        etrobo = initialize_etrobo(backend='raspike_art')
        
        # 4. add_handler = ETRoboのシステム（エンジン）に対して、『周期ごとに実行してほしい仕事を登録する窓口
        etrobo.add_handler(TraverseBehaviourTree(tree))
        
        # 5. ロボットシステムを本番稼働！0.03秒に1回のテンポでツリーを評価し、ロボットを自律走行させる（無限ループ突入）
        etrobo.dispatch(interval=EXEC_INTERVAL, logfile=args.logfile)
        
    finally:
        # 【セーフティネット】例外クラッシュや強制終了（Ctrl+C）が入った場合でも、必ずここのfinallyを通す
        # 車輪が回りっぱなしで暴走するのを防ぐため、OSの終了シグナルを一度無視（IGN）させ、安全に停止シーケンスを実行する
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        
        # 6. カメラのマルチスレッドに「終了ボタン」を押し、安全に停止させてからメモリを解放する
        cleanup_thread()
        
        # シグナル設定をデフォルト（DFL）に戻してプログラムを美しく終了させる
        signal.signal(signal.SIGTERM, signal.SIG_DFL)
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        print(" -- exiting...")