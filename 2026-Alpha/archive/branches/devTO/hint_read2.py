# ==========================================
# EV3RT Robot Control Program - Hint Reading System
# ==========================================
# このプログラムはEV3ロボットの行動制御システムを実装しています。
# BehaviorTreeを使用して複雑なロボット動作を階層的に管理し、
# QRコード読込、色検出、ライントレース等の機能を統合しています。
# ==========================================

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

# ==========================================
# 実行間隔の定数定義
# ==========================================
# メインループの実行周期（秒）。全体的なタイミングを管理する基準値
EXEC_INTERVAL: float  = 0.02
# ビデオ処理スレッドの実行周期（秒）。カメラ画像処理の間隔
VIDEO_INTERVAL: float = 0.02

# ==========================================
# ロボット動作の動力定数
# ==========================================
# 回転アクションの最大出力パワー（0-100）
SPIN_MAX_POWER     = 57
# 回転アクションの最小出力パワー（0-100）
SPIN_MIN_POWER     = 47
# ラインしる際のターゲット輝度値（グレースケール）
# この値に色センサーの読値が近づくようにPID制御で調整
TRACELINE_TARGET_V = 75

# ==========================================
# カメラセンサーの閾値定数
# ==========================================
# グレースケール最小値（ライン検出のための下限）
GS_MIN_DEFAULT     = 0
# グレースケール最大値（ライン検出のための上限）
GS_MAX_DEFAULT     = 55

# ==========================================
# 各種アクションの動作パラメータ
# ==========================================
# アーム上下動作の出力パワー
ARM_SHIFT_PWM      = 35   # ArmUpDownFull - アーム上下動時の出力値

# 交差点検出の閾値（高い方の値）
# ROE（Range Of Edges：エッジの範囲）がこの値以上でラインが結合していると判定
JUNCT_UPPER_THRESH = 50   # IsJunction 

# 交差点検出の閾値（低い方の値）
# ROEがこの値以下でラインが分離していると判定
JUNCT_LOWER_THRESH = 40   # IsJunction

# TraceLineCamでのROE退化閾値
# ROEがこの値を超えるとライン接線状態と判定（曲率推定不可）
ROE_DEGEN          = 90   # TraceLineCam: span above this = line ~tangent

# 曲率計算時の最小行間隔
# 近距離と遠距離の検出位置の間隔がこの値以上必要
CURV_MIN_ROWS_SEP  = 15   # TraceLineCam: need this many rows between near/far to trust the slope


# ==========================================
# 列挙型定義 - ロボットの状態・制御値を管理
# ==========================================

class ArmDirection(IntEnum):
    """アーム上下動作の方向を定義する列挙型
    
    UP (-1): アーム上昇
    DOWN (1): アーム下降
    """
    UP = -1
    DOWN = 1

class JState(Enum):
    """ラインの交差点状態を追跡する列挙型
    
    INITIAL: 初期状態。まだ交差点を検出していない状態
    JOINING: ラインが結合中（2本のラインが近づいてくる途中）
    JOINED: ラインが完全に結合した状態（1本のラインになった）
    FORKING: ラインが分岐中（1本のラインが2本に分かれていく途中）
    FORKED: ラインが完全に分岐した状態（2本のラインに分かれた）
    """
    INITIAL = auto()
    JOINING = auto()
    JOINED = auto()
    FORKING = auto()
    FORKED = auto()

class HeadingType(Enum):
    """ロボットの向き指定方式を定義する列挙型
    
    ABSOLUTE: 絶対座標系での方位角指定（常に同じ向きを維持）
    RELATIVE: 相対座標系での角度指定（現在の向きからの相対変化）
    """
    ABSOLUTE = "absolute"
    RELATIVE = "relative"

# ==========================================
# グローバル変数 - ロボット制御の各種情報を保管
# ==========================================
# これらの変数は各Behaviourクラスから共有可能な状態情報を保持しています

# プロッター：ロボットの移動距離、位置情報などを追跡管理するオブジェクト
g_plotter: Plotter = None
# EV3 Hubへのインタフェース（モータ・センサ制御の中核）
g_hub: Hub = None
# アーム（C ポート）モータのコントローラー
g_arm_motor: Motor = None
# 右モータ（A ポート）のコントローラー
g_right_motor: Motor = None
# 左モータ（B ポート）のコントローラー
g_left_motor: Motor = None
# タッチセンサー（D ポート）
g_touch_sensor: TouchSensor = None
# カラーセンサー（E ポート）- ライン検出に使用
g_color_sensor: ColorSensor = None
# ソナーセンサー（F ポート）- 距離検測に使用
g_sonar_sensor: SonarSensor = None
# ジャイロセンサー - ロボットの向き角度を検測
g_gyro_sensor: GyroSensor = None

# コース選択フラグ（右コース=-1、左コース=1）
# モータの出力方向を反転させる際に使用
g_course: int = 0
# 復号用キー（ReadKey.update()で値を設定）
# QRコードの複号化に必要な4文字キー
g_key: str = None                   # written by ReadKey.update()
# キャッチしたボトルの色（CatchBottle.update()で値を設定）
g_bottle_color = BottleColor.NONE   # written by CatchBottle.update()
# QRコードで取得したヒント1（IsQRDecoded.update()で値を設定）
g_hint1: str = None                 # written by IsQRDecoded
# QRコードで取得したヒント2（IsQRDecoded.update()で値を設定）
g_hint2: str = None                 # written by IsQRDecoded


# ==========================================
# Behaviour基底クラスの実装群
# ==========================================
# BehaviorTreeの各ノードは以下の処理フローに従います：
# 1. __init__(): ノード初期化（一度だけ実行）
# 2. update(): 毎フレーム実行される処理
#    - 初回呼出時に初期化処理を実行
#    - 以後は継続的に状態を確認
#    - Status.SUCCESS / RUNNING / FAILURE を返す

class TheEnd(Behaviour):
    """BehaviorTreeの終了を処理するノード
    
    処理概要:
        - ツリーがすべての処理を終了した際に呼出される
        - プログラム終了時のクリーンアップが必要な場合は
          このノードを修正してそこに処理を追加する
    
    戻り値:
        - Status.RUNNING: 常にRUNNINGを返す（ツリー終了を表現）
    """
    def __init__(self, name: str):
        super(TheEnd, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.running = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.behavior tree exhausted. ctrl+C shall terminate the program" % (g_plotter.get_distance(), self.__class__.__name__))
        return Status.RUNNING


class ResetDevice(Behaviour):
    """ロボット全体の初期化処理を実行するノード
    
    処理概要:
        1. 全モータのエンコーダーをリセット
        2. ジャイロセンサーのキャリブレーション
        3. ビデオシステムの初期設定（グレースケール範囲、追跡対象）
        4. IMU（加速度計）が静止状態になるまで待機
    
    戻り値:
        - Status.SUCCESS: IMUが4フレーム以上静止（初期化完了）
        - Status.RUNNING: 初期化処理中
    """
    def __init__(self, name: str):
        super(ResetDevice, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.count = 0

    def update(self) -> Status:
        if self.count == 0:
            # すべてのモータのエンコーダーをリセット（距離計測を0から開始）
            g_arm_motor.reset_count()
            g_right_motor.reset_count()
            g_left_motor.reset_count()
            # ジャイロセンサーをリセット（方位角を0から開始）
            g_gyro_sensor.reset()
            # ビデオシステムの初期設定
            g_video.set_thresholds(GS_MIN_DEFAULT, GS_MAX_DEFAULT)
            g_video.set_target_interested(TargetInterested.LINE)
            self.logger.info("%+06d %s.resetting..." % (g_plotter.get_distance(), self.__class__.__name__))
            self.logger.info("%+06d %s.waiting for IMU to be stationary..." % (g_plotter.get_distance(), self.__class__.__name__))
        elif self.count > 3:
            # IMUが十分な時間静止状態にあったと判定 → 初期化成功
            self.logger.info("%+06d %s.complete" % (g_plotter.get_distance(), self.__class__.__name__))
            return Status.SUCCESS
        # IMUの静止状態を確認し、静止していればカウント増加
        if g_hub.hub_imu_is_stationary():
            self.count += 1
        return Status.RUNNING


class ArmUpDownFull(Behaviour):
    """アーム上下動作を実行し、可動範囲の限界位置まで移動させるノード
    
    処理概要:
        1. 指定方向（UP/DOWN）にアームを最大出力で駆動
        2. エンコーダー値の変化が5度未満になったら（限界位置に達した）
           その状態を20フレーム以上継続して確認
        3. 確認完了したらブレーキをかけて停止
    
    パラメータ:
        - direction: アーム上下方向（ArmDirection.UP or DOWN）
    
    戻り値:
        - Status.SUCCESS: 限界位置への到達確認完了
        - Status.RUNNING: 駆動中
    """
    def __init__(self, name: str, direction: ArmDirection):
        super(ArmUpDownFull, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.direction = direction
        self.running = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.prev_degree = g_arm_motor.get_count()
            self.logger.info("%+06d %s.start position is %d" % (g_plotter.get_distance(), self.__class__.__name__, self.prev_degree))
            self.count = 0
            # 指定方向にアームを駆動開始
            g_arm_motor.set_power(ARM_SHIFT_PWM * self.direction)
        else:
            cur_degree = g_arm_motor.get_count()
            # エンコーダー値の変化が5度未満 = 回転が止まった = 限界位置に到達
            if abs(cur_degree - self.prev_degree) < 5:
                if self.count > 20:
                    # 限界位置であることを20フレーム以上確認後、停止
                    g_arm_motor.set_power(0)
                    g_arm_motor.set_brake(True)
                    self.logger.info("%+06d %s.position set to %d" % (g_plotter.get_distance(), self.__class__.__name__, cur_degree))
                    return Status.SUCCESS
                else:
                    self.count += 1
            self.prev_degree = cur_degree
        return Status.RUNNING


class ReadKey(Behaviour):
    """ユーザーから復号キーを入力受け付けるノード
    
    処理概要:
        1. キーボードから4文字のキーを入力要求
        2. 長さが4文字でない場合はエラーログ出力して再入力要求
        3. 入力されたキーの確認を取得
        4. 確認がyの場合は成功、それ以外は再入力
    
    戻り値:
        - Status.SUCCESS: キーの確認完了
        - Status.RUNNING: 入力待機中または再入力中
    """
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
            # キーボードから入力を受け取り
            g_key = input("Enter the given key for decryption: ")
            # キーの長さが4文字かどうかをチェック
            if len(g_key) != 4:
                self.logger.warning("%+06d %s.invalid key length: %d. key should be 4 characters long." % (g_plotter.get_distance(), self.__class__.__name__, len(g_key)))
                self.running = False  # 再入力のためにフラグをリセット
                return Status.RUNNING
            else:
                # 入力されたキーを表示して確認を取得
                self.logger.info("%+06d %s.entered key: %s" % (g_plotter.get_distance(), self.__class__.__name__, g_key))
                confirmation = input("Is the entered key correct? (y/n): ")
                if confirmation.lower() == 'y':
                    self.logger.info("%+06d %s.key confirmed" % (g_plotter.get_distance(), self.__class__.__name__))
                    return Status.SUCCESS
                else:
                    self.logger.info("%+06d %s.key rejected, please enter again" % (g_plotter.get_distance(), self.__class__.__name__))
                    self.running = False  # 再入力のためにフラグをリセット
                    return Status.RUNNING


class IsTimePassed(Behaviour):
    def __init__(self, name: str, delta_time: int):
        super(IsTimePassed, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.delta_time = delta_time
        self.running = False
        self.earned = False

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


class IsDistanceEarned(Behaviour):
    def __init__(self, name: str, delta_dist: int):
        super(IsDistanceEarned, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.delta_dist = delta_dist
        self.running = False
        self.earned = False

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


class IsColorDetected(Behaviour):
    def __init__(self, name: str, color: Color):
        super(IsColorDetected, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.color = color
        self.prevColor = Color.UNKNOWN
        self.classifier = ColorClassifier()
        self.running = False
        self.detected = False

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
                # do not log UNKNOWN color to reduce log clutter
                if detected_color != Color.UNKNOWN or self.prevColor != Color.UNKNOWN:
                    self.logger.info("%+06d %s.color changed from %s to %s" % (cur_dist, self.__class__.__name__, self.prevColor.value, detected_color.value))
                    self.prevColor = detected_color
            return Status.RUNNING


class IsQRDecoded(Behaviour):
    """QRコード検出・複号を行うノード
    
    処理概要:
        1. ビデオシステムをQRコード検出モードに切り替え
        2. フレームごとにQRコードのテキスト情報を取得
        3. QRコードが検出されたら：
           - 取得したテキストをHintクラスで複号化
           - 複号化時にg_keyを使用してパスワード検証
           - ヒント種類に応じて g_hint1 または g_hint2 に保存
           - ビデオモードを自動的にLINE追跡に戻す
    
    グローバル変数の更新:
        - g_hint1: HintType.HINT1が取得された場合に設定
        - g_hint2: HintType.HINT2が取得された場合に設定
    
    戻り値:
        - Status.SUCCESS: QRコードが正常に複号化された
        - Status.RUNNING: QRコード検出待機中
    """
    def __init__(self, name: str):
        super(IsQRDecoded, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.running = False
        self.detected = False

    def update(self) -> Status:
        global g_key, g_hint1, g_hint2
        if not self.running:
            self.running = True
            # ビデオシステムをQRコード検出モードに設定
            g_video.set_target_interested(TargetInterested.QRCODE)
            self.logger.info("%+06d %s.detection started for QR code" % (g_plotter.get_distance(), self.__class__.__name__))
        # ビデオシステムからQRコードのテキスト情報を取得
        text = g_video.get_QR_text()
        if text != "":
            if not self.detected:
                self.detected = True
                # 複号化処理：テキストとパスワードから情報を取得
                hint_type, hint_text = Hint(text).resolve(password=g_key)
                # ヒント種類に応じて適切なグローバル変数に保存
                if hint_type == HintType.HINT1:
                    g_hint1 = hint_text
                elif hint_type == HintType.HINT2:
                    g_hint2 = hint_text
                self.logger.info("%+06d %s.QR code decoded: %s" % (g_plotter.get_distance(), self.__class__.__name__, hint_text))
                # カメラがLINE追跡に戻るまで時間がかかるため、ここでビデオモードを切り替え
                g_video.set_target_interested(TargetInterested.LINE)
            return Status.SUCCESS
        else:
            return Status.RUNNING


class IsSonarOn(Behaviour):
    def __init__(self, name: str, alert_dist: int):
        super(IsSonarOn, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.alert_dist = alert_dist
        self.running = False

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


class IsTouchOn(Behaviour):
    def __init__(self, name: str):
        super(IsTouchOn, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.running = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.waiting for touch..." % (g_plotter.get_distance(), self.__class__.__name__))
        if g_touch_sensor.is_pressed():
            self.logger.info("%+06d %s.pressed" % (g_plotter.get_distance(), self.__class__.__name__))
            return Status.SUCCESS
        else:
            return Status.RUNNING


class StopNow(Behaviour):
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
    def __init__(self, name: str, pwm_l: int, pwm_r: int) -> None:
        super(RunAsInstructed, self).__init__(name)
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
    """色センサーを使用したラインしる行動ノード
    
    処理概要:
        このノードは複雑なPID制御アルゴリズムを使用して色センサー値を
        目標値に保つことで、ロボットをラインに沿わせるように制御します。
        
        主な機能：
        1. 基本PID制御：色センサーの値がターゲット値になるよう操舵量を計算
        2. 適応型速度制御：追跡誤差に応じて基本速度を動的に変更
        3. ゲイン調整：速度に応じてPIDゲインを補間（曲線で低ゲイン、直線で高ゲイン）
        4. ラインロスト回復：センサー値が飽和したら強制的にステアリング
        
    パラメータ説明:
        - target: 色センサーのターゲット輝度値
        - power: 基本駆動出力パワー
        - pid_p/i/d: PIDゲイン（比例/積分/微分）
        - power_min: 最小出力（適応制御の下限値）
        - err_lo/hi: 誤差の低/高閾値（速度適応の範囲）
        - gains_slow/fast: 低速/高速時のゲイン（Kp, Kd）
        - recover_v: ラインロスト判定の輝度値
    """
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
        # 基本出力値は最大速度として機能
        # PID出力は±power_maxに制限され、速度低下時でも操舵権限が低下しない
        self.power_max = power
        self.power_min = power if power_min is None else power_min
        self.power = power
        # 適応型速度制御の有効フラグ
        self.adapt = power_min is not None
        self.target = target
        # PIDコントローラーの初期化
        self.pid = PID(pid_p, pid_i, pid_d, setpoint=target, sample_time=EXEC_INTERVAL, output_limits=(-self.power_max, self.power_max))
        self.trace_side = trace_side
        # ローパスフィルター：ノイズ低減（cutoff_hz=Noneの場合はフィルターなし）
        self.lpf = (LowPassFilter(cutoff_hz, EXEC_INTERVAL, median_window) if cutoff_hz else None)
        # 追跡誤差の平滑化メトリック（曲率推定に使用）
        self.err_lo, self.err_hi = err_lo, err_hi
        self.metric_lpf = LowPassFilter(metric_hz, EXEC_INTERVAL)
        self.err_metric = 0.0
        # 速度スルーレート制限（非対称：ブレーキ速い、加速ゆっくり）
        self.accel_step = accel_per_s * EXEC_INTERVAL
        self.decel_step = decel_per_s * EXEC_INTERVAL
        # ゲイン調整：速度に応じてKp/Kdを補間
        self.gains_slow = gains_slow
        self.gains_fast = gains_fast
        self.schedule = (gains_slow is not None and gains_fast is not None
                         and self.power_max > self.power_min)
        # ラインロスト回復機構
        self.recover_v = recover_v
        self.recover_after = recover_after
        self.recover_turn = recover_turn
        self._lost_count = 0

        self.running = False

    def update(self) -> Status:
        if not self.running:
            # 初期化：フィルターのリセット、トレース開始ログ
            if self.lpf:
                self.lpf.reset()
            self.metric_lpf.reset()
            self.running = True
            self.logger.info("%+06d %s.trace started with TS=%s" % (g_plotter.get_distance(), self.__class__.__name__, self.trace_side.name))

        # カラーセンサー値（HSV）を取得
        h, s, v_raw = g_color_sensor.get_raw_color_hsv()
        # ローパスフィルター適用（またはフィルターなし）
        v = self.lpf(v_raw) if self.lpf else v_raw

        # ==================== 適応型基本速度制御 ====================
        # 真の追跡誤差（target - raw v）を不安定性メトリックとして使用
        # 平滑化することで、コース形状に応じた速度反応が可能になる
        self.err_metric = self.metric_lpf(abs(self.target - v_raw))
        if self.adapt:
            # 平滑化された|誤差|を[err_lo, err_hi]から[power_max, power_min]にマッピング
            frac = (self.err_metric - self.err_lo) / (self.err_hi - self.err_lo)
            frac = 0.0 if frac < 0.0 else (1.0 if frac > 1.0 else frac)
            target_power = self.power_max - frac * (self.power_max - self.power_min)
            # 速度変化のレート制限（急ブレーキ、ゆっくり加速）
            dp = target_power - self.power
            if dp > self.accel_step:
                dp = self.accel_step
            elif dp < -self.decel_step:
                dp = -self.decel_step
            self.power += dp
        
        # ==================== ゲイン調整：速度による動的ゲイン補間 ====================
        kp_now, kd_now = self.pid.Kp, self.pid.Kd
        if self.schedule:
            # 現在の速度に基づいてゲインを補間
            f = (self.power - self.power_min) / (self.power_max - self.power_min)
            f = 0.0 if f < 0.0 else (1.0 if f > 1.0 else f)
            kp_now = self.gains_slow[0] + f * (self.gains_fast[0] - self.gains_slow[0])
            kd_now = self.gains_slow[1] + f * (self.gains_fast[1] - self.gains_slow[1])
            self.pid.tunings = (kp_now, self.pid.Ki, kd_now)
        
        # ==================== PID操舵制御（コース向き考慮） ====================
        # PIDは既に±power_maxに制限されているため、直接使用可能
        if self.trace_side == TraceSide.NORMAL:
            turn = (-1) * g_course * int(self.pid(v))
        else: # TraceSide.OPPOSITE
            turn = g_course * int(self.pid(v))

        # ==================== ラインロスト回復機構 ====================
        # センサーが明るいレール端に張り付くと、target=75では弱いP制御のみ
        # → カーブの外側に落ちやすい。持続的な張り付きを検出して強制ステアリング
        if self.recover_v is not None:
            if v_raw >= self.recover_v:
                self._lost_count += 1
            else:
                self._lost_count = 0
            if self._lost_count >= self.recover_after and turn != 0:
                mag = self.power_max if self.recover_turn is None else self.recover_turn
                turn = int(math.copysign(mag, turn))

        # シャープなカーブでは|turn| > power_min → 内輪ゼロ/負値 → タイトピボット
        # これは望ましい動作
        p = int(round(self.power))
        left  = max(-100, min(100, p + turn))    # モータは±100でキャップ
        right = max(-100, min(100, p - turn))
        g_right_motor.set_power(right)
        g_left_motor.set_power(left)

        # ログ出力（デバッグ用・通常はコメントアウト）
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


class RunByGyro(Behaviour):
    def __init__(self, name: str, target: int, power: int,
                 pid_p: float, pid_i: float, pid_d: float, target_type: HeadingType) -> None:
        super(RunByGyro, self).__init__(name)
        self.target = target
        self.target_type = target_type
        self.power = power
        self.pid_p = pid_p
        self.pid_i = pid_i
        self.pid_d = pid_d
        self.last_log_time = None
        self.running = False

    def update(self) -> Status:
        current_heading = (-1) * g_course * g_gyro_sensor.get_angle()
        # log every 1 second
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


class TraceLineCam(Behaviour):
    def __init__(self, name: str, power: int, pid_p: float, pid_i: float, pid_d: float,
                 gs_min: int, gs_max: int, trace_side: TraceSide,
                 tilt_ff_gain: float = 0.0,     # feed-forward turn per unit tilt
                 ff_cap: float = 8.0,           # hard clamp on |tilt_ff|
                 blind_hold_frames: int = 3,    # blind frames before easing the pivot
                 blind_turn_frac: float = 0.55, # fraction of power for the blind hold
                 ) -> None:
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

    def update(self) -> Status:
        if not self.running:
            self.running = True
            g_video.set_thresholds(self.gs_min, self.gs_max)
            g_video.set_target_interested(TargetInterested.LINE)
            if self.trace_side == TraceSide.NORMAL:
                if g_course == -1: # right course
                    g_video.set_trace_side(TraceSide.RIGHT)
                else:
                    g_video.set_trace_side(TraceSide.LEFT)
            elif self.trace_side == TraceSide.OPPOSITE: 
                if g_course == -1: # right course
                    g_video.set_trace_side(TraceSide.LEFT)
                else:
                    g_video.set_trace_side(TraceSide.RIGHT)
            else: # TraceSide.CENTER
                g_video.set_trace_side(TraceSide.CENTER)
            self.logger.info("%+06d %s.trace started with TS=%s" % (g_plotter.get_distance(), self.__class__.__name__, self.trace_side.name))

        theta, fid, cap_t, odo_cap = g_video.get_theta_stamped()
        odo_now = g_plotter.get_distance()

        # ----- live tilt feed-forward (anti-cut) -----
        # Driven by the CURRENT frame's band tilt, not a buffered past value,
        # so it tracks the curve as it tightens and can't invert at the exit
        # the way the fixed delay did. Gated OFF when the band is degenerate
        # (line tangent / wall-clipped), exactly where tilt stops being a
        # trustworthy curvature signal (FID128: roe=60, n=5).
        tilt = g_video.get_line_tilt()
        roe  = g_video.get_range_of_edges()
        tilt_ff = 0.0
        ff_gated = (roe == 0
                    or roe > ROE_DEGEN
                    or g_video.get_band_sep() < CURV_MIN_ROWS_SEP)
        if not ff_gated:
            tilt_ff = self._tilt_ff_gain * tilt
            tilt_ff = max(-self._ff_cap, min(self._ff_cap, tilt_ff))   # don't let FF override the PID's sign

        # PID runs on theta; feed-forward is ADDED to the turn output.
        turn_pid = self.pid(theta)
        turn = turn_pid + tilt_ff

        # ----- blind-pivot cap -----
        # When the band is blind (no usable target) the PID is running on a
        # stale saturated theta -> full-power open-loop pivot. Keep rotating the
        # SAME direction but ease the magnitude so it doesn't spin past the line.
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

        # ----- single per-tick line: FF, PID split, motors, frame state -----
        #p, i, d = self.pid.components
        #self.logger.info(
        #    "%+06d CAM fid=%06d theta=%+06.1f P=%+.1f I=%+.1f D=%+.1f "
        #    "tilt=%+05.2f ff=%+06.1f g=%d turn=%+d L=%d R=%d roe=%03d insight=%d bc=%d age=%.1f" % (
        #        odo_now, fid, theta, p, i, d,
        #        tilt, tilt_ff, int(ff_gated),
        #        int(turn), self.power + int(turn), self.power - int(turn),
        #        roe, int(g_video.is_target_insight()), int(blind_capped),
        #        (time.time() - cap_t) * 1000))
        return Status.RUNNING


class IsJunction(Behaviour):
    def __init__(self, name: str, target_state: JState) -> None:
        super(IsJunction, self).__init__(name)
        self.target_state = target_state
        self.reached = False
        self.prev_roe = 0
        self.state:JState = JState.INITIAL
        self.running = False

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


class CatchBottle(Behaviour):
    """
    Drive up to a tape-wrapped bottle and trap it between the two front arms.

    IDENTIFY  - find the coloured band, latch its colour into g_bottle_color and
                lock the video tracker onto that colour (kills colour flicker).
    APPROACH  - PID-steer on the band bearing (bottle_theta) at `power` until the
                band's lower edge reaches the bottom of the frame, i.e. it starts
                dropping into the camera blind spot (~220 mm ahead).
    CATCH     - the band is now blind. Hold the heading we held while still locked
                on the bottle and run straight by gyro for `catch_run_mm` (150 mm
                default) so the two front arms close over it, then stop.
    """
    IDENTIFY, APPROACH, CATCH = range(3)

    def __init__(self, name: str, power: int,
                 pid_p: float, pid_i: float, pid_d: float,
                 catch_run_mm: int = 150,
                 lock_color: 'BottleColor' = None,   # force a colour, else auto
                 identify_area: int = 400,           # min area to trust the colour
                 identify_frames: int = 3,           # consecutive solid frames
                 heading_avg_frames: int = 5,        # smooth heading before the run-in
                 ) -> None:
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

    def _cur_heading(self) -> int:
        return (-1) * g_course * g_gyro_sensor.get_angle()

    def _steer_vision(self, theta: float) -> None:
        # vision-based: theta already encodes direction, no g_course factor
        turn = int(self.pid(theta))
        g_right_motor.set_power(self.power + turn)
        g_left_motor.set_power(self.power - turn)

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

        # ---------- IDENTIFY ----------
        if self._state == self.IDENTIFY:
            self._solid = self._solid + 1 if (insight and barea >= self.identify_area) else 0
            self._steer_vision(btheta if insight else 0.0)   # creep while identifying
            if self._solid >= self.identify_frames:
                g_bottle_color = color
                g_video.set_bottle_color(color)              # lock -> stable bearing
                self.logger.info("%+06d %s.color=%s area=%d -> APPROACH" % (
                    g_plotter.get_distance(), self.__class__.__name__, color.name, barea))
                self._state = self.APPROACH
            return Status.RUNNING

        # ---------- APPROACH ----------
        if self._state == self.APPROACH:
            if insight:
                self._blind_steer = 0
                self._steer_vision(btheta)
                self._heading_hist.append(self._cur_heading())   # log heading while locked
                if len(self._heading_hist) > self.heading_avg_frames:
                    self._heading_hist.pop(0)
            else:
                self._blind_steer += 1                            # brief dropout: hold straight
                g_right_motor.set_power(self.power)
                g_left_motor.set_power(self.power)

            # band reached / passed the blind edge -> commit to the run-in
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

        # ---------- CATCH (gyro run-in) ----------
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


class IsBottleInsight(Behaviour):
    """
    Condition node: SUCCESS while a tape-wrapped bottle of `color` is in sight,
    FAILURE otherwise.

    - color == BottleColor.NONE  -> match any detected bottle colour.
    - color == a specific colour -> SUCCESS only when the detected band is that
      colour (and big enough to trust).

    Reads the same snapshot CatchBottle consumes (g_video.get_bottle_stamped()),
    so it stays consistent with whatever the BOTTLE detector decided this frame.
    A short debounce (`min_frames`) suppresses single-frame colour flicker; set
    it to 1 for an instantaneous check.
    """
    def __init__(self, name: str, color: 'BottleColor',
                 min_area: int = 150,     # ignore specks below this contour area
                 min_frames: int = 2,     # consecutive matching frames to assert SUCCESS
                 set_target: bool = True,  # put the camera in BOTTLE mode on first tick
                 ) -> None:
        super(IsBottleInsight, self).__init__(name)
        self.color = color
        self.min_area = min_area
        self.min_frames = min_frames
        self.set_target = set_target
        self._hits = 0
        self.running = False

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


class HasCaughtBottle(Behaviour):
    """
    Condition node: SUCCESS if the colour latched by CatchBottle (g_bottle_color)
    matches `color`, FAILURE otherwise.

    - color == BottleColor.NONE -> SUCCESS if ANY bottle has been caught
      (i.e. g_bottle_color is no longer NONE).
    - color == a specific colour -> SUCCESS only when that exact colour was caught.

    Pure read of g_bottle_color; no camera or motor side effects.
    """
    def __init__(self, name: str, color: 'BottleColor') -> None:
        super(HasCaughtBottle, self).__init__(name)
        self.color = color

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


class TraverseBehaviourTree(object):
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
            # log estimated position every 1 second
            #if self.last_log_time == None or time.time() - self.last_log_time >= 1.0:
            #    print(" --  estimated position X=%d, Y=%d" % (g_plotter.get_loc_x(), g_plotter.get_loc_y()))
            #    self.last_log_time = time.time()


class VideoThread(threading.Thread):
    def __init__(self):
        super().__init__()
        self._stop_event = threading.Event()
        self.prev_time = time.time()

    def stop(self):
        self._stop_event.set()

    def run(self):
        while not self._stop_event.is_set():
            g_video.process(g_plotter, g_hub, g_arm_motor, g_right_motor, g_left_motor, g_color_sensor, g_sonar_sensor, g_gyro_sensor)
            current_time = time.time()
            elapsed_time = current_time - self.prev_time
            self.prev_time = current_time
            if elapsed_time < VIDEO_INTERVAL:
                time.sleep(VIDEO_INTERVAL - elapsed_time)


def build_behaviour_tree() -> BehaviourTree:
    root = Sequence(name="2026 base", memory=True)
    calibration = Sequence(name="calibration", memory=True)
    start = Parallel(name="start", policy=ParallelPolicy.SuccessOnOne())
    hint1_1 = Parallel(name="approach hint1 part1", policy=ParallelPolicy.SuccessOnOne())
    hint1_2 = Parallel(name="approach hint1 part2", policy=ParallelPolicy.SuccessOnOne())
    hint1_3 = Parallel(name="approach hint1 part3", policy=ParallelPolicy.SuccessOnOne())
    hint1 = Sequence(name="approach hint1", memory=True)
    hint1_scan_move_forward = Parallel(name="move back", policy=ParallelPolicy.SuccessOnOne())
    hint1_scan_shake = Sequence(name="back off and shake heading", memory=True)
    hint1_read = Parallel(name="read hint1 card", policy=ParallelPolicy.SuccessOnOne())
    hint2_1 = Parallel(name="approach hint2 part1", policy=ParallelPolicy.SuccessOnOne())
    hint2_2 = Parallel(name="approach hint2 part2", policy=ParallelPolicy.SuccessOnOne())
    hint2_3 = Parallel(name="approach hint2 part3", policy=ParallelPolicy.SuccessOnOne())
    hint2 = Sequence(name="approach hint2", memory=True)
    hint2_scan_move_back = Parallel(name="move back", policy=ParallelPolicy.SuccessOnOne())
    hint2_scan_shake = Sequence(name="back off and shake heading", memory=True)
    hint2_read = Parallel(name="read hint2 card", policy=ParallelPolicy.SuccessOnOne())
    calibration.add_children(
        [
            ArmUpDownFull(name="arm up", direction=ArmDirection.UP),
            ArmUpDownFull(name="arm down", direction=ArmDirection.DOWN),
            ResetDevice(name="device reset"),
            ReadKey(name="read key"),
        ]
    )
    start.add_children(
        [
            IsTouchOn(name="touch start"),
        ]
    )
    hint1_1.add_children(
        [
            TraceLine(name="sensor trace opposite edge", target=TRACELINE_TARGET_V,
                power=70, power_min=33,
                pid_p=0.65, pid_i=0.000001, pid_d=0.045,
                err_lo=6, err_hi=16, decel_per_s=350, gains_slow=(0.65, 0.045), gains_fast=(0.55, 0.065),
                recover_v=97, recover_after=3, recover_turn=35,
                trace_side=TraceSide.OPPOSITE),
            IsColorDetected(name="check color", color=Color.BLUE),
        ]
    )
    hint1_2.add_children(
        [
            RunByGyro(name="run straight to pass blue marker 3", target=0, power=33,
                pid_p=1.1, pid_i=0.1, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
            IsDistanceEarned(name="check distance", delta_dist = 120),
        ]
    )
    hint1_3.add_children(
        [
            TraceLine(name="sensor trace opposite edge", target=TRACELINE_TARGET_V,
                power=70, power_min=33,
                pid_p=0.65, pid_i=0.000001, pid_d=0.045,
                err_lo=6, err_hi=16, decel_per_s=350, gains_slow=(0.65, 0.045), gains_fast=(0.55, 0.065),
                recover_v=97, recover_after=3, recover_turn=35,
                trace_side=TraceSide.OPPOSITE),
            IsDistanceEarned(name="check distance", delta_dist = 330),
        ]
    )
    hint1.add_children(
        [
            hint1_1,
            hint1_2,
            hint1_3,
        ]
    )
    hint1_scan_move_forward.add_children(
        [
            RunAsInstructed(name="move forward a little", pwm_l=-SPIN_MIN_POWER, pwm_r=-SPIN_MIN_POWER),
            IsDistanceEarned(name="check distance", delta_dist = 50),
        ]
    )
    hint1_scan_shake.add_children(
        [
            StopNow(name="stop"),
            SpinAround(name="align for QR code scanning", target=90, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                pid_p=0.4, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
            StopNow(name="stop"),
            IsTimePassed(name="wait for a moment", delta_time=3.0),
            hint1_scan_move_forward,
            StopNow(name="stop"),
            IsTimePassed(name="wait for a moment", delta_time=3.0),
            SpinAround(name="scan for QR code", target=3, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                pid_p=0.4, pid_i=0.001, pid_d=0.03, target_type=HeadingType.RELATIVE),
            StopNow(name="stop"),
            IsTimePassed(name="wait for a moment", delta_time=2.0),
            SpinAround(name="scan for QR code", target=-6, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                pid_p=0.4, pid_i=0.001, pid_d=0.03, target_type=HeadingType.RELATIVE),
            StopNow(name="stop"),
            IsTimePassed(name="wait for a moment", delta_time=2.0),
            SpinAround(name="scan for QR code", target=3, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                pid_p=0.4, pid_i=0.001, pid_d=0.03, target_type=HeadingType.RELATIVE),
            StopNow(name="stop"),
            IsTimePassed(name="wait for a moment", delta_time=3.0),
        ]
    )
    hint1_read.add_children(
        [
            IsQRDecoded(name="check QR code"),
            hint1_scan_shake,
        ]
    )
    hint2_1.add_children(
        [
            TraceLineCam(name="camera trace opposite", power=40,
                pid_p=2.0, pid_i=0.0, pid_d=0.06, tilt_ff_gain=8.0, ff_cap=8.0,
                gs_min=GS_MIN_DEFAULT, gs_max=GS_MAX_DEFAULT, trace_side=TraceSide.OPPOSITE),
            IsDistanceEarned(name="check distance", delta_dist = 1100),
        ]
    )
    hint2_2.add_children(
        [
            TraceLine(name="sensor trace opposite edge", target=TRACELINE_TARGET_V,
                power=70, power_min=33,
                pid_p=0.65, pid_i=0.000001, pid_d=0.045,
                err_lo=6, err_hi=16, decel_per_s=350, gains_slow=(0.65, 0.045), gains_fast=(0.55, 0.065),
                recover_v=97, recover_after=3, recover_turn=35,
                trace_side=TraceSide.OPPOSITE),
            IsColorDetected(name="check color", color=Color.BLUE),
        ]
    )
    hint2_3.add_children(
        [
            RunByGyro(name="run straight to pass half the blue line", target=-90, power=33,
                pid_p=1.1, pid_i=0.1, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
            IsDistanceEarned(name="check distance", delta_dist = 100),
        ]
    )
    hint2.add_children(
        [
            SpinAround(name="about the face", target=-80, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                pid_p=0.4, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
            StopNow(name="stop"),
            hint2_1,
            hint2_2,
            hint2_3,
        ]
    )
    hint2_scan_move_back.add_children(
        [
            RunAsInstructed(name="move back a little", pwm_l=-SPIN_MIN_POWER, pwm_r=-SPIN_MIN_POWER),
            IsDistanceEarned(name="check distance", delta_dist = 50),
        ]
    )
    hint2_scan_shake.add_children(
        [
            StopNow(name="stop"),
            ArmUpDownFull(name="arm up", direction=ArmDirection.UP),
            SpinAround(name="align for QR code scanning", target=0, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                pid_p=0.4, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
            StopNow(name="stop"),
            IsTimePassed(name="wait for a moment", delta_time=3.0),
            hint2_scan_move_back,
            StopNow(name="stop"),
            IsTimePassed(name="wait for a moment", delta_time=3.0),
            SpinAround(name="scan for QR code", target=3, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                pid_p=0.4, pid_i=0.001, pid_d=0.03, target_type=HeadingType.RELATIVE),
            StopNow(name="stop"),
            IsTimePassed(name="wait for a moment", delta_time=2.0),
            SpinAround(name="scan for QR code", target=-6, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                pid_p=0.4, pid_i=0.001, pid_d=0.03, target_type=HeadingType.RELATIVE),
            StopNow(name="stop"),
            IsTimePassed(name="wait for a moment", delta_time=2.0),
            SpinAround(name="scan for QR code", target=3, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                pid_p=0.4, pid_i=0.001, pid_d=0.03, target_type=HeadingType.RELATIVE),
            StopNow(name="stop"),
            IsTimePassed(name="wait for a moment", delta_time=3.0),
        ]
    )
    hint2_read.add_children(
        [
            IsQRDecoded(name="check QR code"),
            hint2_scan_shake,
        ]
    )
    root.add_children(
        [
            calibration,
            start,
            hint1,
            hint1_read,
            hint2,
            hint2_read,
            ArmUpDownFull(name="arm down", direction=ArmDirection.DOWN),
            StopNow(name="stop"),
            TheEnd(name="end"),
        ]
    )
    return root

def initialize_etrobo(backend: str) -> ETRobo:
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
    global g_video, g_video_thread
    g_video = Video()

    print(" -- starting VideoThread...")
    g_video_thread = VideoThread()
    g_video_thread.start()

def cleanup_thread():
    global g_video, g_video_thread
    print(" -- stopping VideoThread...")
    g_video_thread.stop()
    g_video_thread.join()

    del g_video

def sig_handler(signum, frame) -> None:
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
