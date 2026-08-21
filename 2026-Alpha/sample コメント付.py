import sys  # sys モジュールを読み込む
import argparse  # argparse モジュールを読み込む
import time  # time モジュールを読み込む
import threading  # threading モジュールを読み込む
import signal  # signal モジュールを読み込む
import math  # math モジュールを読み込む
from enum import IntEnum, Enum, auto  # enum から必要なクラス・関数を読み込む
from etrobo_python import ETRobo, Hub, Motor, TouchSensor, ColorSensor, SonarSensor, GyroSensor  # etrobo_python から必要なクラス・関数を読み込む
from simple_pid import PID  # simple_pid から必要なクラス・関数を読み込む
from py_trees.trees import BehaviourTree  # py_trees.trees から必要なクラス・関数を読み込む
from py_trees.behaviour import Behaviour  # py_trees.behaviour から必要なクラス・関数を読み込む
from py_trees.common import Status  # py_trees.common から必要なクラス・関数を読み込む
from py_trees.composites import Sequence  # py_trees.composites から必要なクラス・関数を読み込む
from py_trees.composites import Selector  # py_trees.composites から必要なクラス・関数を読み込む
from py_trees.composites import Parallel  # py_trees.composites から必要なクラス・関数を読み込む
from py_trees.common import ParallelPolicy  # py_trees.common から必要なクラス・関数を読み込む
from py_trees import (  # py_trees から必要なクラス・関数を読み込む
    display as display_tree,  # 直前の定義・関数呼び出しに渡す値を指定する
    logging as log_tree  # この処理を実行する
)  # 直前から続く定義・引数・リストを閉じる
from py_etrobo_util import Video, TraceSide, TargetInterested, Plotter, SymmetricClamper, Color, ColorClassifier, LowPassFilter, BottleColor, Hint, HintType  # py_etrobo_util から必要なクラス・関数を読み込む

# constants for defining execution intervals
EXEC_INTERVAL: float  = 0.02  # EXEC_INTERVAL: float に処理で使用する値を設定する
VIDEO_INTERVAL: float = 0.02  # VIDEO_INTERVAL: float に処理で使用する値を設定する

# constants useful for behavior tree definition
SPIN_MAX_POWER     = 57  # SPIN_MAX_POWER に処理で使用する値を設定する
SPIN_MIN_POWER     = 47  # SPIN_MIN_POWER に処理で使用する値を設定する
TRACELINE_TARGET_V = 75  # TRACELINE_TARGET_V に処理で使用する値を設定する

# constants for specific action classes
GS_MIN_DEFAULT     = 0  # GS_MIN_DEFAULT に処理で使用する値を設定する
GS_MAX_DEFAULT     = 55  # GS_MAX_DEFAULT に処理で使用する値を設定する
ARM_SHIFT_PWM      = 35   # ArmUpDownFull  # ARM_SHIFT_PWM に処理で使用する値を設定する
JUNCT_UPPER_THRESH = 50   # IsJunction   # JUNCT_UPPER_THRESH に処理で使用する値を設定する
JUNCT_LOWER_THRESH = 40   # IsJunction  # JUNCT_LOWER_THRESH に処理で使用する値を設定する
ROE_DEGEN          = 90   # TraceLineCam: span above this = line ~tangent  # ROE_DEGEN に処理で使用する値を設定する
CURV_MIN_ROWS_SEP  = 15   # TraceLineCam: need this many rows between near/far to trust the slope  # CURV_MIN_ROWS_SEP に処理で使用する値を設定する

class ArmDirection(IntEnum):  # ArmDirection クラスを定義する
    UP = -1  # UP に処理で使用する値を設定する
    DOWN = 1  # DOWN に処理で使用する値を設定する

class JState(Enum):  # JState クラスを定義する
    INITIAL = auto()  # INITIAL に処理で使用する値を設定する
    JOINING = auto()  # JOINING に処理で使用する値を設定する
    JOINED = auto()  # JOINED に処理で使用する値を設定する
    FORKING = auto()  # FORKING に処理で使用する値を設定する
    FORKED = auto()  # FORKED に処理で使用する値を設定する

class HeadingType(Enum):  # HeadingType クラスを定義する
    ABSOLUTE = "absolute"  # ABSOLUTE に処理で使用する値を設定する
    RELATIVE = "relative"  # RELATIVE に処理で使用する値を設定する

g_plotter: Plotter = None  # g_plotter: Plotter に、プログラム全体で共有する値を設定する
g_hub: Hub = None  # g_hub: Hub に、プログラム全体で共有する値を設定する
g_arm_motor: Motor = None  # g_arm_motor: Motor に、プログラム全体で共有する値を設定する
g_right_motor: Motor = None  # g_right_motor: Motor に、プログラム全体で共有する値を設定する
g_left_motor: Motor = None  # g_left_motor: Motor に、プログラム全体で共有する値を設定する
g_touch_sensor: TouchSensor = None  # g_touch_sensor: TouchSensor に、プログラム全体で共有する値を設定する
g_color_sensor: ColorSensor = None  # g_color_sensor: ColorSensor に、プログラム全体で共有する値を設定する
g_sonar_sensor: SonarSensor = None  # g_sonar_sensor: SonarSensor に、プログラム全体で共有する値を設定する
g_gyro_sensor: GyroSensor = None  # g_gyro_sensor: GyroSensor に、プログラム全体で共有する値を設定する
g_course: int = 0  # g_course: int に、プログラム全体で共有する値を設定する
g_key: str = None                   # written by ReadKey.update()  # g_key: str に、プログラム全体で共有する値を設定する
g_bottle_color = BottleColor.NONE   # written by CatchBottle.update()  # g_bottle_color に、プログラム全体で共有する値を設定する
g_hint1: str = None                 # written by IsQRDecoded  # g_hint1: str に、プログラム全体で共有する値を設定する
g_hint2: str = None                 # written by IsQRDecoded  # g_hint2: str に、プログラム全体で共有する値を設定する

class TheEnd(Behaviour):  # TheEnd クラスを定義する
    def __init__(self, name: str):  # __init__ メソッド／関数を定義する
        super(TheEnd, self).__init__(name)  # 親クラスの初期化処理を呼び出す
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))  # デバッグ用のログを出力する
        self.running = False  # self.running に、このオブジェクトで使用する値を設定する

    def update(self) -> Status:  # update メソッド／関数を定義する
        if not self.running:  # 条件を判定し、成立した場合の処理を行う
            self.running = True  # self.running に、このオブジェクトで使用する値を設定する
            self.logger.info("%+06d %s.behavior tree exhausted. ctrl+C shall terminate the program" % (g_plotter.get_distance(), self.__class__.__name__))  # 動作状況を情報ログとして出力する
        return Status.RUNNING  # 処理結果を呼び出し元へ返す


class ResetDevice(Behaviour):  # ResetDevice クラスを定義する
    def __init__(self, name: str):  # __init__ メソッド／関数を定義する
        super(ResetDevice, self).__init__(name)  # 親クラスの初期化処理を呼び出す
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))  # デバッグ用のログを出力する
        self.count = 0  # self.count に、このオブジェクトで使用する値を設定する

    def update(self) -> Status:  # update メソッド／関数を定義する
        if self.count == 0:  # 条件を判定し、成立した場合の処理を行う
            g_arm_motor.reset_count()  # モータの回転角カウントをリセットする
            g_right_motor.reset_count()  # モータの回転角カウントをリセットする
            g_left_motor.reset_count()  # モータの回転角カウントをリセットする
            g_gyro_sensor.reset()  # 対象の状態を初期化する
            g_video.set_thresholds(GS_MIN_DEFAULT, GS_MAX_DEFAULT)  # この処理を実行する
            g_video.set_target_interested(TargetInterested.LINE)  # この処理を実行する
            self.logger.info("%+06d %s.resetting..." % (g_plotter.get_distance(), self.__class__.__name__))  # 動作状況を情報ログとして出力する
            self.logger.info("%+06d %s.waiting for IMU to be stationary..." % (g_plotter.get_distance(), self.__class__.__name__))  # 動作状況を情報ログとして出力する
        elif self.count > 3:  # 前の条件が不成立の場合に追加条件を判定する
            self.logger.info("%+06d %s.complete" % (g_plotter.get_distance(), self.__class__.__name__))  # 動作状況を情報ログとして出力する
            return Status.SUCCESS  # 処理結果を呼び出し元へ返す
        if g_hub.hub_imu_is_stationary():  # 条件を判定し、成立した場合の処理を行う
            self.count += 1  # self.count + に、このオブジェクトで使用する値を設定する
        return Status.RUNNING  # 処理結果を呼び出し元へ返す


class ArmUpDownFull(Behaviour):  # ArmUpDownFull クラスを定義する
    def __init__(self, name: str, direction: ArmDirection):  # __init__ メソッド／関数を定義する
        super(ArmUpDownFull, self).__init__(name)  # 親クラスの初期化処理を呼び出す
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))  # デバッグ用のログを出力する
        self.direction = direction  # self.direction に、このオブジェクトで使用する値を設定する
        self.running = False  # self.running に、このオブジェクトで使用する値を設定する

    def update(self) -> Status:  # update メソッド／関数を定義する
        if not self.running:  # 条件を判定し、成立した場合の処理を行う
            self.running = True  # self.running に、このオブジェクトで使用する値を設定する
            self.prev_degree = g_arm_motor.get_count()  # self.prev_degree に、このオブジェクトで使用する値を設定する
            self.logger.info("%+06d %s.start position is %d" % (g_plotter.get_distance(), self.__class__.__name__, self.prev_degree))  # 動作状況を情報ログとして出力する
            self.count = 0  # self.count に、このオブジェクトで使用する値を設定する
            g_arm_motor.set_power(ARM_SHIFT_PWM * self.direction)  # モータの出力値を設定する
        else:  # 上記の条件に当てはまらない場合の処理を行う
            cur_degree = g_arm_motor.get_count()  # cur_degree に処理で使用する値を設定する
            if abs(cur_degree - self.prev_degree) < 5:  # 条件を判定し、成立した場合の処理を行う
                if self.count > 20:  # 条件を判定し、成立した場合の処理を行う
                    g_arm_motor.set_power(0)  # モータの出力値を設定する
                    g_arm_motor.set_brake(True)  # モータのブレーキ状態を設定する
                    self.logger.info("%+06d %s.position set to %d" % (g_plotter.get_distance(), self.__class__.__name__, cur_degree))  # 動作状況を情報ログとして出力する
                    return Status.SUCCESS  # 処理結果を呼び出し元へ返す
                else:  # 上記の条件に当てはまらない場合の処理を行う
                    self.count += 1  # self.count + に、このオブジェクトで使用する値を設定する
            self.prev_degree = cur_degree  # self.prev_degree に、このオブジェクトで使用する値を設定する
        return Status.RUNNING  # 処理結果を呼び出し元へ返す


class ReadKey(Behaviour):  # ReadKey クラスを定義する
    def __init__(self, name: str):
        super(ReadKey, self).__init__(name)  # 親クラスBehaviourの初期化処理を呼び出す
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))  # ReadKeyが初期化されたことをデバッグログに出力
        self.running = False  # まだ入力処理を開始していないことを表す

        global g_key  # この関数内で使用するグローバル変数を宣言する
        # 初回呼び出し時は処理開始状態にする
        if not self.running:
            self.running = True
            # ユーザーからの入力を受け取る
            g_key = input("Enter the given key for decryption: ") 
            # check the length of the key, it should be 4 characters long
            if len(g_key) != 4:  # 入力されたキーの長さが4文字のとき
                self.logger.warning("%+06d %s.invalid key length: %d. key should be 4 characters long."
                                    % (g_plotter.get_distance(), 
                                       self.__class__.__name__,
                                        len(g_key))
                                    )  # 警告ログを出力する
                return Status.RUNNING  # 再びキー入力を行う
            # 入力したキーを画面に表示
            print("Entered password:", g_key)
            # 入力した内容が正しか確認
            confirmation = input("Is this key correct? (y/n):")
            if confirmation.lower() == 'y': # 正しい場合
                self.logger.info("%+06d %s.key confirmed" % (g_plotter.get_distance(), self.__class__.__name__))  # 動作状況を情報ログとして出力する
                return Status.SUCCESS # 処理終了
            else:  # 正しくない場合
                self.logger.info("%+06d %s.key rejected, please enter again" % (g_plotter.get_distance(), self.__class__.__name__))  # 動作状況を情報ログとして出力する
                return Status.RUNNING # 再びキー入力を行う

    # def update(self) -> Status:
    #     global g_key  # この関数内で使用するグローバル変数を宣言する
    #     # 初回呼び出し時は処理開始状態にする
    #     if not self.running:
    #         self.running = True
    #         # ユーザーからの入力を受け取る
    #         g_key = input("Enter the given key for decryption: ") 
    #         # check the length of the key, it should be 4 characters long
    #         if len(g_key) != 4:  # 入力されたキーの長さが4文字のとき
    #             self.logger.warning("%+06d %s.invalid key length: %d. key should be 4 characters long."
    #                                 % (g_plotter.get_distance(), 
    #                                    self.__class__.__name__,
    #                                     len(g_key))
    #                                 )  # 警告ログを出力する
    #             return Status.RUNNING  # 再びキー入力を行う
    #         # 入力したキーを画面に表示
    #         print("Entered password:", g_key)
    #         # 入力した内容が正しか確認
    #         confirmation = input("Is this key correct? (y/n):")
    #         if confirmation.lower() == 'y': # 正しい場合
    #             self.logger.info("%+06d %s.key confirmed" % (g_plotter.get_distance(), self.__class__.__name__))  # 動作状況を情報ログとして出力する
    #             return Status.SUCCESS # 処理終了
    #         else:  # 正しくない場合
    #             self.logger.info("%+06d %s.key rejected, please enter again" % (g_plotter.get_distance(), self.__class__.__name__))  # 動作状況を情報ログとして出力する
    #             return Status.RUNNING # 再びキー入力を行う


class IsTimePassed(Behaviour):  # IsTimePassed クラスを定義する
    def __init__(self, name: str, delta_time: int):  # __init__ メソッド／関数を定義する
        super(IsTimePassed, self).__init__(name)  # 親クラスの初期化処理を呼び出す
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))  # デバッグ用のログを出力する
        self.delta_time = delta_time  # self.delta_time に、このオブジェクトで使用する値を設定する
        self.running = False  # self.running に、このオブジェクトで使用する値を設定する
        self.earned = False  # self.earned に、このオブジェクトで使用する値を設定する

    def update(self) -> Status:  # update メソッド／関数を定義する
        if not self.running:  # 条件を判定し、成立した場合の処理を行う
            self.running = True  # self.running に、このオブジェクトで使用する値を設定する
            self.orig_time = time.time()  # 現在時刻を取得して時間計測に使用する
            self.logger.info("%+06d %s.accumulation started for delta=%d" % (self.orig_time, self.__class__.__name__, self.delta_time))  # 動作状況を情報ログとして出力する
        cur_time = time.time()  # 現在時刻を取得して時間計測に使用する
        earned_time = cur_time - self.orig_time  # earned_time に処理で使用する値を設定する
        if earned_time >= self.delta_time:  # 条件を判定し、成立した場合の処理を行う
            if not self.earned:  # 条件を判定し、成立した場合の処理を行う
                self.earned = True  # self.earned に、このオブジェクトで使用する値を設定する
                self.logger.info("%+06d %s.delta time passed" % (g_plotter.get_distance(), self.__class__.__name__))  # 動作状況を情報ログとして出力する
            return Status.SUCCESS  # 処理結果を呼び出し元へ返す
        else:  # 上記の条件に当てはまらない場合の処理を行う
            return Status.RUNNING  # 処理結果を呼び出し元へ返す


class IsDistanceEarned(Behaviour):  # IsDistanceEarned クラスを定義する
    def __init__(self, name: str, delta_dist: int):  # __init__ メソッド／関数を定義する
        super(IsDistanceEarned, self).__init__(name)  # 親クラスの初期化処理を呼び出す
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))  # デバッグ用のログを出力する
        self.delta_dist = delta_dist  # self.delta_dist に、このオブジェクトで使用する値を設定する
        self.running = False  # self.running に、このオブジェクトで使用する値を設定する
        self.earned = False  # self.earned に、このオブジェクトで使用する値を設定する

    def update(self) -> Status:  # update メソッド／関数を定義する
        if not self.running:  # 条件を判定し、成立した場合の処理を行う
            self.running = True  # self.running に、このオブジェクトで使用する値を設定する
            self.orig_dist = g_plotter.get_distance()  # 現在の走行距離またはセンサ距離を取得して変数に保存する
            self.logger.info("%+06d %s.accumulation started for delta=%d" % (self.orig_dist, self.__class__.__name__, self.delta_dist))  # 動作状況を情報ログとして出力する
        cur_dist = g_plotter.get_distance()  # 現在の走行距離またはセンサ距離を取得して変数に保存する
        earned_dist = cur_dist - self.orig_dist  # earned_dist に処理で使用する値を設定する
        if (earned_dist >= self.delta_dist or -earned_dist <= -self.delta_dist):  # 条件を判定し、成立した場合の処理を行う
            if not self.earned:  # 条件を判定し、成立した場合の処理を行う
                self.earned = True  # self.earned に、このオブジェクトで使用する値を設定する
                self.logger.info("%+06d %s.delta distance earned" % (cur_dist, self.__class__.__name__))  # 動作状況を情報ログとして出力する
            return Status.SUCCESS  # 処理結果を呼び出し元へ返す
        else:  # 上記の条件に当てはまらない場合の処理を行う
            return Status.RUNNING  # 処理結果を呼び出し元へ返す


class IsColorDetected(Behaviour):  # IsColorDetected クラスを定義する
    def __init__(self, name: str, color: Color):  # __init__ メソッド／関数を定義する
        super(IsColorDetected, self).__init__(name)  # 親クラスの初期化処理を呼び出す
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))  # デバッグ用のログを出力する
        self.color = color  # self.color に、このオブジェクトで使用する値を設定する
        self.prevColor = Color.UNKNOWN  # self.prevColor に、このオブジェクトで使用する値を設定する
        self.classifier = ColorClassifier()  # self.classifier に、このオブジェクトで使用する値を設定する
        self.running = False  # self.running に、このオブジェクトで使用する値を設定する
        self.detected = False  # self.detected に、このオブジェクトで使用する値を設定する

    def update(self) -> Status:  # update メソッド／関数を定義する
        cur_dist = g_plotter.get_distance()  # 現在の走行距離またはセンサ距離を取得して変数に保存する
        if not self.running:  # 条件を判定し、成立した場合の処理を行う
            self.running = True  # self.running に、このオブジェクトで使用する値を設定する
            self.logger.info("%+06d %s.detection started for color=%s" % (cur_dist, self.__class__.__name__, self.color.value))  # 動作状況を情報ログとして出力する
        h, s, v = g_color_sensor.get_raw_color_hsv()  # カラーセンサからHSV形式の色情報を取得する

        detected_color = self.classifier.classify(h, s, v)  # detected_color に処理で使用する値を設定する
        if detected_color == self.color:  # 条件を判定し、成立した場合の処理を行う
            if not self.detected:  # 条件を判定し、成立した場合の処理を行う
                self.detected = True  # self.detected に、このオブジェクトで使用する値を設定する
                self.logger.info("%+06d %s.color=%s detected" % (cur_dist, self.__class__.__name__, self.color.value))  # 動作状況を情報ログとして出力する
            return Status.SUCCESS  # 処理結果を呼び出し元へ返す
        else:  # 上記の条件に当てはまらない場合の処理を行う
            if detected_color != self.prevColor:  # 条件を判定し、成立した場合の処理を行う
                # do not log UNKNOWN color to reduce log clutter
                if detected_color != Color.UNKNOWN or self.prevColor != Color.UNKNOWN:  # 条件を判定し、成立した場合の処理を行う
                    self.logger.info("%+06d %s.color changed from %s to %s" % (cur_dist, self.__class__.__name__, self.prevColor.value, detected_color.value))  # 動作状況を情報ログとして出力する
                    self.prevColor = detected_color  # self.prevColor に、このオブジェクトで使用する値を設定する
            return Status.RUNNING  # 処理結果を呼び出し元へ返す


class IsQRDecoded(Behaviour):  # IsQRDecoded クラスを定義する
    def __init__(self, name: str):  # __init__ メソッド／関数を定義する
        super(IsQRDecoded, self).__init__(name)  # 親クラスの初期化処理を呼び出す
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))  # デバッグ用のログを出力する
        self.running = False  # self.running に、このオブジェクトで使用する値を設定する
        self.detected = False  # self.detected に、このオブジェクトで使用する値を設定する

    def update(self) -> Status:  # update メソッド／関数を定義する
        global g_key, g_hint1, g_hint2  # この関数内で使用するグローバル変数を宣言する
        if not self.running:  # 条件を判定し、成立した場合の処理を行う
            self.running = True  # self.running に、このオブジェクトで使用する値を設定する
            g_video.set_target_interested(TargetInterested.QRCODE)  # この処理を実行する
            self.logger.info("%+06d %s.detection started for QR code" % (g_plotter.get_distance(), self.__class__.__name__))  # 動作状況を情報ログとして出力する
        text = g_video.get_QR_text()  # text に処理で使用する値を設定する
        if text != "":  # 条件を判定し、成立した場合の処理を行う
            if not self.detected:  # 条件を判定し、成立した場合の処理を行う
                self.detected = True  # self.detected に、このオブジェクトで使用する値を設定する
                hint_type, hint_text = Hint(text).resolve(password=g_key)  # hint_type, hint_text に処理で使用する値を設定する
                if hint_type == HintType.HINT1:  # 条件を判定し、成立した場合の処理を行う
                    g_hint1 = hint_text  # g_hint1 に、プログラム全体で共有する値を設定する
                elif hint_type == HintType.HINT2:  # 前の条件が不成立の場合に追加条件を判定する
                    g_hint2 = hint_text  # g_hint2 に、プログラム全体で共有する値を設定する
                self.logger.info("%+06d %s.QR code decoded: %s" % (g_plotter.get_distance(), self.__class__.__name__, hint_text))  # 動作状況を情報ログとして出力する
                g_video.set_target_interested(TargetInterested.LINE)  # set target back to default as it takes for camera to settle   # この処理を実行する
            return Status.SUCCESS  # 処理結果を呼び出し元へ返す
        else:  # 上記の条件に当てはまらない場合の処理を行う
            return Status.RUNNING  # 処理結果を呼び出し元へ返す


class IsSonarOn(Behaviour):  # IsSonarOn クラスを定義する
    def __init__(self, name: str, alert_dist: int):  # __init__ メソッド／関数を定義する
        super(IsSonarOn, self).__init__(name)  # 親クラスの初期化処理を呼び出す
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))  # デバッグ用のログを出力する
        self.alert_dist = alert_dist  # self.alert_dist に、このオブジェクトで使用する値を設定する
        self.running = False  # self.running に、このオブジェクトで使用する値を設定する

    def update(self) -> Status:  # update メソッド／関数を定義する
        if not self.running:  # 条件を判定し、成立した場合の処理を行う
            self.running = True  # self.running に、このオブジェクトで使用する値を設定する
            self.logger.info("%+06d %s.detection started for dist=%d" % (g_plotter.get_distance(), self.__class__.__name__, self.alert_dist))  # 現在の走行距離またはセンサ距離を取得して変数に保存する
        
        dist = g_sonar_sensor.get_distance()  # 現在の走行距離またはセンサ距離を取得して変数に保存する
        if (dist <= self.alert_dist and dist > 0):  # 条件を判定し、成立した場合の処理を行う
            self.logger.info("%+06d %s.alerted at dist=%d" % (g_plotter.get_distance(), self.__class__.__name__, dist))  # 現在の走行距離またはセンサ距離を取得して変数に保存する
            return Status.SUCCESS  # 処理結果を呼び出し元へ返す
        else:  # 上記の条件に当てはまらない場合の処理を行う
            return Status.RUNNING  # 処理結果を呼び出し元へ返す


class IsTouchOn(Behaviour):  # IsTouchOn クラスを定義する
    def __init__(self, name: str):  # __init__ メソッド／関数を定義する
        super(IsTouchOn, self).__init__(name)  # 親クラスの初期化処理を呼び出す
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))  # デバッグ用のログを出力する
        self.running = False  # self.running に、このオブジェクトで使用する値を設定する

    def update(self) -> Status:  # update メソッド／関数を定義する
        if not self.running:  # 条件を判定し、成立した場合の処理を行う
            self.running = True  # self.running に、このオブジェクトで使用する値を設定する
            self.logger.info("%+06d %s.waiting for touch..." % (g_plotter.get_distance(), self.__class__.__name__))  # 動作状況を情報ログとして出力する
        if g_touch_sensor.is_pressed():  # 条件を判定し、成立した場合の処理を行う
            self.logger.info("%+06d %s.pressed" % (g_plotter.get_distance(), self.__class__.__name__))  # 動作状況を情報ログとして出力する
            return Status.SUCCESS  # 処理結果を呼び出し元へ返す
        else:  # 上記の条件に当てはまらない場合の処理を行う
            return Status.RUNNING  # 処理結果を呼び出し元へ返す


class StopNow(Behaviour):  # StopNow クラスを定義する
    def __init__(self, name: str):  # __init__ メソッド／関数を定義する
        super(StopNow, self).__init__(name)  # 親クラスの初期化処理を呼び出す
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))  # デバッグ用のログを出力する

    def update(self) -> Status:  # update メソッド／関数を定義する
        g_right_motor.set_power(0)  # モータの出力値を設定する
        g_right_motor.set_brake(True)  # モータのブレーキ状態を設定する
        g_left_motor.set_power(0)  # モータの出力値を設定する
        g_left_motor.set_brake(True)  # モータのブレーキ状態を設定する
        self.logger.info("%+06d %s.motors stopped" % (g_plotter.get_distance(), self.__class__.__name__))  # 動作状況を情報ログとして出力する
        return Status.SUCCESS  # 処理結果を呼び出し元へ返す


class RunAsInstructed(Behaviour):  # RunAsInstructed クラスを定義する
    def __init__(self, name: str, pwm_l: int, pwm_r: int) -> None:  # __init__ メソッド／関数を定義する
        super(RunAsInstructed, self).__init__(name)  # 親クラスの初期化処理を呼び出す
        self.pwm_l = g_course * pwm_l  # self.pwm_l に、このオブジェクトで使用する値を設定する
        self.pwm_r = g_course * pwm_r  # self.pwm_r に、このオブジェクトで使用する値を設定する
        self.running = False  # self.running に、このオブジェクトで使用する値を設定する


    def update(self) -> Status:  # update メソッド／関数を定義する
        if not self.running:  # 条件を判定し、成立した場合の処理を行う
            self.running = True  # self.running に、このオブジェクトで使用する値を設定する
            self.logger.info("%+06d %s.started with pwm=(%s, %s)" % (g_plotter.get_distance(), self.__class__.__name__, self.pwm_l, self.pwm_r))  # 現在の走行距離またはセンサ距離を取得して変数に保存する
        g_right_motor.set_power(self.pwm_r)  # モータの出力値を設定する
        g_left_motor.set_power(self.pwm_l)  # モータの出力値を設定する
        return Status.RUNNING  # 処理結果を呼び出し元へ返す


class TraceLine(Behaviour):  # TraceLine クラスを定義する
    def __init__(self, name: str, target: int, power: int, pid_p: float, pid_i: float, pid_d: float,  # __init__ メソッド／関数を定義する
                 trace_side: TraceSide,  # 直前の定義・関数呼び出しに渡す値を指定する
                 # low-pass filter parameters
                 cutoff_hz: float = 12.0, median_window: int = 0,  # cutoff_hz: float に処理で使用する値を設定する
                 # adaptive speed parameters
                 power_min: int = None,          # floor speed; None = constant speed (old behaviour)  # power_min: int に処理で使用する値を設定する
                 err_lo: float = 6.0,            # rolling |err| at/below which we run full speed  # err_lo: float に処理で使用する値を設定する
                 err_hi: float = 22.0,           # rolling |err| at/above which we run power_min  # err_hi: float に処理で使用する値を設定する
                 accel_per_s: float = 60.0,      # how fast we may speed up   (gentle)  # accel_per_s: float に処理で使用する値を設定する
                 decel_per_s: float = 180.0,     # how fast we may slow down  (quick = pseudo lookahead)  # decel_per_s: float に処理で使用する値を設定する
                 metric_hz: float = 2.0,  # metric_hz: float に処理で使用する値を設定する
                 # ---- gain scheduling: interpolate gains on current speed ----
                 # give (Kp, Kd) at the slow (curve) end and the fast (straight) end;
                 # None -> no scheduling, the fixed pid_p/pid_d above are used everywhere.
                 gains_slow: tuple = None,       # (Kp, Kd) at power_min  # gains_slow: tuple に処理で使用する値を設定する
                 gains_fast: tuple = None,       # (Kp, Kd) at power_max  # gains_fast: tuple に処理で使用する値を設定する
                 # ---- line-lost recovery (outer-edge curve rescue) ----
                 recover_v: int = None,           # bright-rail v that means "line lost to floor"; None = off  # recover_v: int に処理で使用する値を設定する
                 recover_after: int = 3,          # consecutive lost samples before hard recovery  # recover_after: int に処理で使用する値を設定する
                 recover_turn: int = None         # recovery steering magnitude; None = power_max  # recover_turn: int に処理で使用する値を設定する
                ) -> None:  # 直前から続く関数呼び出しまたは定義を閉じる
        super(TraceLine, self).__init__(name)  # 親クラスの初期化処理を呼び出す
        # power is treated as the MAX/nominal speed. The PID output limit is
        # pinned to +/-power_max so steering authority does NOT shrink when the
        # base speed drops on a curve (that was the trap in the original code).
        self.power_max = power  # self.power_max に、このオブジェクトで使用する値を設定する
        self.power_min = power if power_min is None else power_min  # self.power_min に、このオブジェクトで使用する値を設定する
        self.power = power  # self.power に、このオブジェクトで使用する値を設定する
        self.adapt = power_min is not None  # self.adapt に、このオブジェクトで使用する値を設定する
        self.target = target  # self.target に、このオブジェクトで使用する値を設定する
        self.pid = PID(pid_p, pid_i, pid_d, setpoint=target, sample_time=EXEC_INTERVAL, output_limits=(-self.power_max, self.power_max))  # 指定したゲインと目標値でPID制御器を生成する
        self.trace_side = trace_side  # self.trace_side に、このオブジェクトで使用する値を設定する
        self.lpf = (LowPassFilter(cutoff_hz, EXEC_INTERVAL, median_window) if cutoff_hz else None) # when cutoff_hz = None, no low-pass filter is applied and the raw PID output is used  # self.lpf に、このオブジェクトで使用する値を設定する
        # instability/curviness estimate = smoothed |tracking error|
        self.err_lo, self.err_hi = err_lo, err_hi  # self.err_lo, self.err_hi に、このオブジェクトで使用する値を設定する
        self.metric_lpf = LowPassFilter(metric_hz, EXEC_INTERVAL)  # self.metric_lpf に、このオブジェクトで使用する値を設定する
        self.err_metric = 0.0  # self.err_metric に、このオブジェクトで使用する値を設定する
        # per-step power slew limits (asymmetric: brake fast, accelerate slow)
        self.accel_step = accel_per_s * EXEC_INTERVAL  # self.accel_step に、このオブジェクトで使用する値を設定する
        self.decel_step = decel_per_s * EXEC_INTERVAL  # self.decel_step に、このオブジェクトで使用する値を設定する
        # gain schedule: linearly interpolate (Kp, Kd) between the slow and fast
        # anchors as a function of self.power. Ki is left fixed at pid_i.
        self.gains_slow = gains_slow  # self.gains_slow に、このオブジェクトで使用する値を設定する
        self.gains_fast = gains_fast  # self.gains_fast に、このオブジェクトで使用する値を設定する
        self.schedule = (gains_slow is not None and gains_fast is not None  # self.schedule に、このオブジェクトで使用する値を設定する
                         and self.power_max > self.power_min)  # この処理を実行する
        # line-lost recovery
        self.recover_v = recover_v  # self.recover_v に、このオブジェクトで使用する値を設定する
        self.recover_after = recover_after  # self.recover_after に、このオブジェクトで使用する値を設定する
        self.recover_turn = recover_turn  # self.recover_turn に、このオブジェクトで使用する値を設定する
        self._lost_count = 0  # self._lost_count に、このオブジェクトで使用する値を設定する

        self.running = False  # self.running に、このオブジェクトで使用する値を設定する

    def update(self) -> Status:  # update メソッド／関数を定義する
        if not self.running:  # 条件を判定し、成立した場合の処理を行う
            if self.lpf:  # 条件を判定し、成立した場合の処理を行う
                self.lpf.reset()  # 対象の状態を初期化する
            self.metric_lpf.reset()  # 対象の状態を初期化する
            self.running = True  # self.running に、このオブジェクトで使用する値を設定する
            self.logger.info("%+06d %s.trace started with TS=%s" % (g_plotter.get_distance(), self.__class__.__name__, self.trace_side.name))  # 現在の走行距離またはセンサ距離を取得して変数に保存する

        h, s, v_raw = g_color_sensor.get_raw_color_hsv()  # カラーセンサからHSV形式の色情報を取得する
        v = self.lpf(v_raw) if self.lpf else v_raw  # v に処理で使用する値を設定する

        # ---- adaptive base speed -------------------------------------------
        # Use the TRUE tracking error (target - raw v) as the instability metric,
        # smoothed so the speed reacts to course shape, not to every wobble.
        self.err_metric = self.metric_lpf(abs(self.target - v_raw))  # self.err_metric に、このオブジェクトで使用する値を設定する
        if self.adapt:  # 条件を判定し、成立した場合の処理を行う
            # map smoothed |error| in [err_lo, err_hi] -> power in [max, min]
            frac = (self.err_metric - self.err_lo) / (self.err_hi - self.err_lo)  # frac に処理で使用する値を設定する
            frac = 0.0 if frac < 0.0 else (1.0 if frac > 1.0 else frac)  # frac に処理で使用する値を設定する
            target_power = self.power_max - frac * (self.power_max - self.power_min)  # target_power に処理で使用する値を設定する
            # rate-limit the change (slow down quickly, speed up gently)
            dp = target_power - self.power  # dp に処理で使用する値を設定する
            if dp > self.accel_step:  # 条件を判定し、成立した場合の処理を行う
                dp = self.accel_step  # dp に処理で使用する値を設定する
            elif dp < -self.decel_step:  # 前の条件が不成立の場合に追加条件を判定する
                dp = -self.decel_step  # dp に処理で使用する値を設定する
            self.power += dp  # self.power + に、このオブジェクトで使用する値を設定する
        # ---- gain scheduling: gains track the current speed -----------------
        kp_now, kd_now = self.pid.Kp, self.pid.Kd  # kp_now, kd_now に処理で使用する値を設定する
        if self.schedule:  # 条件を判定し、成立した場合の処理を行う
            f = (self.power - self.power_min) / (self.power_max - self.power_min)  # f に処理で使用する値を設定する
            f = 0.0 if f < 0.0 else (1.0 if f > 1.0 else f)  # f に処理で使用する値を設定する
            kp_now = self.gains_slow[0] + f * (self.gains_fast[0] - self.gains_slow[0])  # kp_now に処理で使用する値を設定する
            kd_now = self.gains_slow[1] + f * (self.gains_fast[1] - self.gains_slow[1])  # kd_now に処理で使用する値を設定する
            self.pid.tunings = (kp_now, self.pid.Ki, kd_now)  # self.pid.tunings に、このオブジェクトで使用する値を設定する
        # ---- steering (PID already clamped to +/-power_max) ----------------
        if self.trace_side == TraceSide.NORMAL:  # 条件を判定し、成立した場合の処理を行う
            turn = (-1) * g_course * int(self.pid(v))  # turn に処理で使用する値を設定する
        else: # TraceSide.OPPOSITE  # この処理を実行する
            turn = g_course * int(self.pid(v))  # turn に処理で使用する値を設定する

        # ---- line-lost recovery --------------------------------------------
        # When the sensor pins at the bright rail, target=75 only yields a weak
        # clamped-P turn (Kp*(75-100) ~= -16), too gentle to curl back to a line
        # that curved away on an OUTER edge -> the robot drives off. Detect a
        # SUSTAINED bright-rail pin (not a 1-2 sample weave touch) and steer at
        # full authority in the direction P already (correctly) chose, until the
        # edge is reacquired. The dark rail already recovers on its own.
        if self.recover_v is not None:  # 条件を判定し、成立した場合の処理を行う
            if v_raw >= self.recover_v:  # 条件を判定し、成立した場合の処理を行う
                self._lost_count += 1  # self._lost_count + に、このオブジェクトで使用する値を設定する
            else:  # 上記の条件に当てはまらない場合の処理を行う
                self._lost_count = 0  # self._lost_count に、このオブジェクトで使用する値を設定する
            if self._lost_count >= self.recover_after and turn != 0:  # 条件を判定し、成立した場合の処理を行う
                mag = self.power_max if self.recover_turn is None else self.recover_turn  # mag に処理で使用する値を設定する
                turn = int(math.copysign(mag, turn))  # turn に処理で使用する値を設定する

        # On a sharp slow curve, |turn| may exceed the reduced base speed, so the
        # inner wheel can go to zero/negative -> a tight pivot. That's desired.
        p = int(round(self.power))  # p に処理で使用する値を設定する
        left  = max(-100, min(100, p + turn))    # motors cap at +-100  # left に処理で使用する値を設定する
        right = max(-100, min(100, p - turn))  # right に処理で使用する値を設定する
        g_right_motor.set_power(right)  # モータの出力値を設定する
        g_left_motor.set_power(left)  # モータの出力値を設定する

        # log raw v, filtered vf, error-metric, commanded power, gains, and turn
        #self.logger.info("%+06d %s.color sensor HSV=(%d, %d, %d) vf=%d, em=%d, pwr=%d, kp=%.3f, kd=%.3f, turn=%d" % (
        #    g_plotter.get_distance(), self.__class__.__name__,
        #    h, s, v_raw, int(v), int(self.err_metric), p, kp_now, kd_now, turn))

        return Status.RUNNING  # 処理結果を呼び出し元へ返す


class SpinAndLocateLine(Behaviour):  # SpinAndLocateLine クラスを定義する
    def __init__(self, name: str, target: int, max_power: int, min_power: int,  # __init__ メソッド／関数を定義する
                 pid_p: float, pid_i: float, pid_d: float, trace_side: TraceSide) -> None:  # この処理を実行する
        super(SpinAndLocateLine, self).__init__(name)  # 親クラスの初期化処理を呼び出す
        self.target = target  # self.target に、このオブジェクトで使用する値を設定する
        self.spin_direction = 1 if trace_side == TraceSide.NORMAL else -1  # この処理を実行する
        self.pid_p = pid_p  # self.pid_p に、このオブジェクトで使用する値を設定する
        self.pid_i = pid_i  # self.pid_i に、このオブジェクトで使用する値を設定する
        self.pid_d = pid_d  # self.pid_d に、このオブジェクトで使用する値を設定する
        self.clamper = SymmetricClamper(min_power, max_power)  # self.clamper に、このオブジェクトで使用する値を設定する
        self.move_away = True  # self.move_away に、このオブジェクトで使用する値を設定する
        self.running = False  # self.running に、このオブジェクトで使用する値を設定する

    def update(self) -> Status:  # update メソッド／関数を定義する
        # first to spin to move away from the line, then to locate the line by spinning in the opposite direction
        current_heading = (-1) * g_course * g_gyro_sensor.get_angle()  # ジャイロセンサから現在の角度を取得する
        if not self.running:  # 条件を判定し、成立した場合の処理を行う
            # spin for 30 degrees to move away from the line
            self.target_heading = current_heading + self.spin_direction * 30  # self.target_heading に、このオブジェクトで使用する値を設定する
            self.pid = PID(self.pid_p, self.pid_i, self.pid_d, setpoint=self.target_heading, sample_time=EXEC_INTERVAL)  # 指定したゲインと目標値でPID制御器を生成する
            self.running = True  # self.running に、このオブジェクトで使用する値を設定する
            self.logger.info("%+06d %s.spin started at heading=%d for %d" % (g_plotter.get_distance(),  # 現在の走行距離またはセンサ距離を取得して変数に保存する
                                                                             self.__class__.__name__, current_heading, self.target_heading))  # この処理を実行する
        if self.move_away:  # 条件を判定し、成立した場合の処理を行う
            # spin in the normal direction to move away from the line
            error = float(self.target_heading) - current_heading  # error に処理で使用する値を設定する
            # normalize error to [-180, 180]
            if error > 180.0:  # 条件を判定し、成立した場合の処理を行う
                error -= 360.0  # error - に処理で使用する値を設定する
            if error < -180.0:  # 条件を判定し、成立した場合の処理を行う
                error += 360.0  # error + に処理で使用する値を設定する
            if abs(error) < 2.0:  # 条件を判定し、成立した場合の処理を行う
                self.logger.info("%+06d %s.move away spin ended at heading=%d" % (g_plotter.get_distance(),  # 現在の走行距離またはセンサ距離を取得して変数に保存する
                                                                    self.__class__.__name__, current_heading))  # この処理を実行する
                self.pid = PID(self.pid_p, self.pid_i, self.pid_d, setpoint=self.target, sample_time=EXEC_INTERVAL)  # 指定したゲインと目標値でPID制御器を生成する
                self.spin_direction *= -1  # self.spin_direction * に、このオブジェクトで使用する値を設定する
                self.move_away = False  # self.move_away に、このオブジェクトで使用する値を設定する
                return Status.RUNNING  # 処理結果を呼び出し元へ返す
            power = int(self.clamper.clamp(self.pid(current_heading)))  # power に処理で使用する値を設定する
        else:             # spin in the opposite direction to locate the line  # この処理を実行する
            h, s, v = g_color_sensor.get_raw_color_hsv()  # カラーセンサからHSV形式の色情報を取得する
            error = float(self.target) - v  # error に処理で使用する値を設定する
            if abs(error) < 5.0:  # 条件を判定し、成立した場合の処理を行う
                self.logger.info("%+06d %s.line located at heading=%d" % (g_plotter.get_distance(),  # 現在の走行距離またはセンサ距離を取得して変数に保存する
                                                                    self.__class__.__name__, current_heading))          # この処理を実行する
                return Status.SUCCESS  # 処理結果を呼び出し元へ返す
            power = int(self.clamper.clamp(self.pid(v))) * self.spin_direction * (-1)  # power に処理で使用する値を設定する
        g_right_motor.set_power(g_course * power)  # モータの出力値を設定する
        g_left_motor.set_power((-1) * g_course * power)  # モータの出力値を設定する
        return Status.RUNNING      # 処理結果を呼び出し元へ返す


class SpinAround(Behaviour):  # SpinAround クラスを定義する
    def __init__(self, name: str, target: int, max_power: int, min_power: int,  # __init__ メソッド／関数を定義する
                 pid_p: float, pid_i: float, pid_d: float, target_type: HeadingType) -> None:  # この処理を実行する
        super(SpinAround, self).__init__(name)  # 親クラスの初期化処理を呼び出す
        self.target = target  # self.target に、このオブジェクトで使用する値を設定する
        self.target_type = target_type  # self.target_type に、このオブジェクトで使用する値を設定する
        self.pid_p = pid_p  # self.pid_p に、このオブジェクトで使用する値を設定する
        self.pid_i = pid_i  # self.pid_i に、このオブジェクトで使用する値を設定する
        self.pid_d = pid_d  # self.pid_d に、このオブジェクトで使用する値を設定する
        self.clamper = SymmetricClamper(min_power, max_power)  # self.clamper に、このオブジェクトで使用する値を設定する
        self.running = False  # self.running に、このオブジェクトで使用する値を設定する

    def update(self) -> Status:  # update メソッド／関数を定義する
        current_heading = (-1) * g_course * g_gyro_sensor.get_angle()  # ジャイロセンサから現在の角度を取得する
        if not self.running:  # 条件を判定し、成立した場合の処理を行う
            if self.target_type == HeadingType.RELATIVE:  # 条件を判定し、成立した場合の処理を行う
                self.target_heading = current_heading + self.target  # self.target_heading に、このオブジェクトで使用する値を設定する
            else:  # 上記の条件に当てはまらない場合の処理を行う
                self.target_heading = self.target  # self.target_heading に、このオブジェクトで使用する値を設定する
            self.pid = PID(self.pid_p, self.pid_i, self.pid_d, setpoint=self.target_heading, sample_time=EXEC_INTERVAL)  # 指定したゲインと目標値でPID制御器を生成する
            self.running = True  # self.running に、このオブジェクトで使用する値を設定する
            self.logger.info("%+06d %s.spin started at heading=%d for %d" % (g_plotter.get_distance(),  # 現在の走行距離またはセンサ距離を取得して変数に保存する
                                                                             self.__class__.__name__, current_heading, self.target_heading))  # この処理を実行する
        error = float(self.target_heading) - current_heading  # error に処理で使用する値を設定する
        # normalize error to [-180, 180]
        if error > 180.0:  # 条件を判定し、成立した場合の処理を行う
            error -= 360.0  # error - に処理で使用する値を設定する
        if error < -180.0:  # 条件を判定し、成立した場合の処理を行う
            error += 360.0  # error + に処理で使用する値を設定する
        if abs(error) < 2.0:  # 条件を判定し、成立した場合の処理を行う
            self.logger.info("%+06d %s.spin ended at heading=%d" % (g_plotter.get_distance(),  # 現在の走行距離またはセンサ距離を取得して変数に保存する
                                                                    self.__class__.__name__, current_heading))  # この処理を実行する
            return Status.SUCCESS  # 処理結果を呼び出し元へ返す
        power = int(self.clamper.clamp(self.pid(current_heading)))  # power に処理で使用する値を設定する
        g_right_motor.set_power(g_course * power)  # モータの出力値を設定する
        g_left_motor.set_power((-1) * g_course * power)  # モータの出力値を設定する
        return Status.RUNNING      # 処理結果を呼び出し元へ返す


class RunByGyro(Behaviour):  # RunByGyro クラスを定義する
    def __init__(self, name: str, target: int, power: int,  # __init__ メソッド／関数を定義する
                 pid_p: float, pid_i: float, pid_d: float, target_type: HeadingType) -> None:  # この処理を実行する
        super(RunByGyro, self).__init__(name)  # 親クラスの初期化処理を呼び出す
        self.target = target  # self.target に、このオブジェクトで使用する値を設定する
        self.target_type = target_type  # self.target_type に、このオブジェクトで使用する値を設定する
        self.power = power  # self.power に、このオブジェクトで使用する値を設定する
        self.pid_p = pid_p  # self.pid_p に、このオブジェクトで使用する値を設定する
        self.pid_i = pid_i  # self.pid_i に、このオブジェクトで使用する値を設定する
        self.pid_d = pid_d  # self.pid_d に、このオブジェクトで使用する値を設定する
        self.last_log_time = None  # self.last_log_time に、このオブジェクトで使用する値を設定する
        self.running = False  # self.running に、このオブジェクトで使用する値を設定する

    def update(self) -> Status:  # update メソッド／関数を定義する
        current_heading = (-1) * g_course * g_gyro_sensor.get_angle()  # ジャイロセンサから現在の角度を取得する
        # log every 1 second
        if self.last_log_time == None or time.time() - self.last_log_time >= 1.0:  # 条件を判定し、成立した場合の処理を行う
            self.logger.info("%+06d %s.current heading=%d" % (g_plotter.get_distance(), self.__class__.__name__, current_heading))  # 現在の走行距離またはセンサ距離を取得して変数に保存する
            self.last_log_time = time.time()  # 現在時刻を取得して時間計測に使用する
        if not self.running:  # 条件を判定し、成立した場合の処理を行う
            if self.target_type == HeadingType.RELATIVE:  # 条件を判定し、成立した場合の処理を行う
                self.target_heading = current_heading + self.target  # self.target_heading に、このオブジェクトで使用する値を設定する
            else:  # 上記の条件に当てはまらない場合の処理を行う
                self.target_heading = self.target  # self.target_heading に、このオブジェクトで使用する値を設定する
            self.pid = PID(self.pid_p, self.pid_i, self.pid_d, setpoint=self.target_heading, sample_time=EXEC_INTERVAL, output_limits=(-self.power, self.power))  # 指定したゲインと目標値でPID制御器を生成する
            self.logger.info("%+06d %s.gyro run started toward heading=%d" % (g_plotter.get_distance(),  # 現在の走行距離またはセンサ距離を取得して変数に保存する
                                                                              self.__class__.__name__, self.target_heading))  # この処理を実行する
            self.running = True  # self.running に、このオブジェクトで使用する値を設定する
        turn = int(self.pid(current_heading))  # turn に処理で使用する値を設定する
        g_right_motor.set_power(self.power + g_course * turn)  # モータの出力値を設定する
        g_left_motor.set_power(self.power - g_course * turn)  # モータの出力値を設定する
        return Status.RUNNING  # 処理結果を呼び出し元へ返す


class TraceLineCam(Behaviour):  # TraceLineCam クラスを定義する
    def __init__(self, name: str, power: int, pid_p: float, pid_i: float, pid_d: float,  # __init__ メソッド／関数を定義する
                 gs_min: int, gs_max: int, trace_side: TraceSide,  # 直前の定義・関数呼び出しに渡す値を指定する
                 tilt_ff_gain: float = 0.0,     # feed-forward turn per unit tilt  # tilt_ff_gain: float に処理で使用する値を設定する
                 ff_cap: float = 8.0,           # hard clamp on |tilt_ff|  # ff_cap: float に処理で使用する値を設定する
                 blind_hold_frames: int = 3,    # blind frames before easing the pivot  # blind_hold_frames: int に処理で使用する値を設定する
                 blind_turn_frac: float = 0.55, # fraction of power for the blind hold  # blind_turn_frac: float に処理で使用する値を設定する
                 ) -> None:  # 直前から続く関数呼び出しまたは定義を閉じる
        super(TraceLineCam, self).__init__(name)  # 親クラスの初期化処理を呼び出す
        self.power = power  # self.power に、このオブジェクトで使用する値を設定する
        self.pid = PID(pid_p, pid_i, pid_d, setpoint=0, sample_time=EXEC_INTERVAL, output_limits=(-power, power))  # 指定したゲインと目標値でPID制御器を生成する
        self.gs_min = gs_min  # self.gs_min に、このオブジェクトで使用する値を設定する
        self.gs_max = gs_max  # self.gs_max に、このオブジェクトで使用する値を設定する
        self._tilt_ff_gain = tilt_ff_gain  # self._tilt_ff_gain に、このオブジェクトで使用する値を設定する
        self._ff_cap = ff_cap  # self._ff_cap に、このオブジェクトで使用する値を設定する
        self._blind_hold_frames = blind_hold_frames  # self._blind_hold_frames に、このオブジェクトで使用する値を設定する
        self._blind_turn_frac = blind_turn_frac  # self._blind_turn_frac に、このオブジェクトで使用する値を設定する
        self._blind = 0  # self._blind に、このオブジェクトで使用する値を設定する
        self.trace_side = trace_side  # self.trace_side に、このオブジェクトで使用する値を設定する
        self.running = False  # self.running に、このオブジェクトで使用する値を設定する

    def update(self) -> Status:  # update メソッド／関数を定義する
        if not self.running:  # 条件を判定し、成立した場合の処理を行う
            self.running = True  # self.running に、このオブジェクトで使用する値を設定する
            g_video.set_thresholds(self.gs_min, self.gs_max)  # この処理を実行する
            g_video.set_target_interested(TargetInterested.LINE)  # この処理を実行する
            if self.trace_side == TraceSide.NORMAL:  # 条件を判定し、成立した場合の処理を行う
                if g_course == -1: # right course  # 条件を判定し、成立した場合の処理を行う
                    g_video.set_trace_side(TraceSide.RIGHT)  # この処理を実行する
                else:  # 上記の条件に当てはまらない場合の処理を行う
                    g_video.set_trace_side(TraceSide.LEFT)  # この処理を実行する
            elif self.trace_side == TraceSide.OPPOSITE:   # 前の条件が不成立の場合に追加条件を判定する
                if g_course == -1: # right course  # 条件を判定し、成立した場合の処理を行う
                    g_video.set_trace_side(TraceSide.LEFT)  # この処理を実行する
                else:  # 上記の条件に当てはまらない場合の処理を行う
                    g_video.set_trace_side(TraceSide.RIGHT)  # この処理を実行する
            else: # TraceSide.CENTER  # この処理を実行する
                g_video.set_trace_side(TraceSide.CENTER)  # この処理を実行する
            self.logger.info("%+06d %s.trace started with TS=%s" % (g_plotter.get_distance(), self.__class__.__name__, self.trace_side.name))  # 現在の走行距離またはセンサ距離を取得して変数に保存する

        theta, fid, cap_t, odo_cap = g_video.get_theta_stamped()  # theta, fid, cap_t, odo_cap に処理で使用する値を設定する
        odo_now = g_plotter.get_distance()  # 現在の走行距離またはセンサ距離を取得して変数に保存する

        # ----- live tilt feed-forward (anti-cut) -----
        # Driven by the CURRENT frame's band tilt, not a buffered past value,
        # so it tracks the curve as it tightens and can't invert at the exit
        # the way the fixed delay did. Gated OFF when the band is degenerate
        # (line tangent / wall-clipped), exactly where tilt stops being a
        # trustworthy curvature signal (FID128: roe=60, n=5).
        tilt = g_video.get_line_tilt()  # tilt に処理で使用する値を設定する
        roe  = g_video.get_range_of_edges()  # roe に処理で使用する値を設定する
        tilt_ff = 0.0  # tilt_ff に処理で使用する値を設定する
        ff_gated = (roe == 0  # この処理を実行する
                    or roe > ROE_DEGEN  # この処理を実行する
                    or g_video.get_band_sep() < CURV_MIN_ROWS_SEP)  # この処理を実行する
        if not ff_gated:  # 条件を判定し、成立した場合の処理を行う
            tilt_ff = self._tilt_ff_gain * tilt  # tilt_ff に処理で使用する値を設定する
            tilt_ff = max(-self._ff_cap, min(self._ff_cap, tilt_ff))   # don't let FF override the PID's sign  # tilt_ff に処理で使用する値を設定する

        # PID runs on theta; feed-forward is ADDED to the turn output.
        turn_pid = self.pid(theta)  # turn_pid に処理で使用する値を設定する
        turn = turn_pid + tilt_ff  # turn に処理で使用する値を設定する

        # ----- blind-pivot cap -----
        # When the band is blind (no usable target) the PID is running on a
        # stale saturated theta -> full-power open-loop pivot. Keep rotating the
        # SAME direction but ease the magnitude so it doesn't spin past the line.
        if not g_video.is_target_insight():  # 条件を判定し、成立した場合の処理を行う
            self._blind += 1  # self._blind + に、このオブジェクトで使用する値を設定する
        else:  # 上記の条件に当てはまらない場合の処理を行う
            self._blind = 0  # self._blind に、このオブジェクトで使用する値を設定する
        blind_capped = False  # blind_capped に処理で使用する値を設定する
        if self._blind > self._blind_hold_frames:  # 条件を判定し、成立した場合の処理を行う
            hold = self.power * self._blind_turn_frac  # hold に処理で使用する値を設定する
            if turn > hold:  # 条件を判定し、成立した場合の処理を行う
                turn = hold; blind_capped = True  # turn に処理で使用する値を設定する
            elif turn < -hold:  # 前の条件が不成立の場合に追加条件を判定する
                turn = -hold; blind_capped = True  # turn に処理で使用する値を設定する

        g_right_motor.set_power(self.power + int(turn))  # モータの出力値を設定する
        g_left_motor.set_power(self.power - int(turn))  # モータの出力値を設定する

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
        return Status.RUNNING  # 処理結果を呼び出し元へ返す


class IsJunction(Behaviour):  # IsJunction クラスを定義する
    def __init__(self, name: str, target_state: JState) -> None:  # __init__ メソッド／関数を定義する
        super(IsJunction, self).__init__(name)  # 親クラスの初期化処理を呼び出す
        self.target_state = target_state  # self.target_state に、このオブジェクトで使用する値を設定する
        self.reached = False  # self.reached に、このオブジェクトで使用する値を設定する
        self.prev_roe = 0  # self.prev_roe に、このオブジェクトで使用する値を設定する
        self.state:JState = JState.INITIAL  # self.state:JState に、このオブジェクトで使用する値を設定する
        self.running = False  # self.running に、このオブジェクトで使用する値を設定する

    def update(self) -> Status:  # update メソッド／関数を定義する
        if not self.running:  # 条件を判定し、成立した場合の処理を行う
            self.running = True  # self.running に、このオブジェクトで使用する値を設定する
            self.logger.info("%+06d %s.scan started" % (g_plotter.get_distance(), self.__class__.__name__))  # 動作状況を情報ログとして出力する
        roe = g_video.get_range_of_edges()  # roe に処理で使用する値を設定する
        if roe != 0:  # 条件を判定し、成立した場合の処理を行う
            if self.state == JState.INITIAL:  # 条件を判定し、成立した場合の処理を行う
                if (self.target_state == JState.JOINING or self.target_state == JState.JOINED) and roe >= JUNCT_UPPER_THRESH and self.prev_roe <= JUNCT_LOWER_THRESH:  # 条件を判定し、成立した場合の処理を行う
                    self.logger.info("%+06d %s.lines are joining" % (g_plotter.get_distance(), self.__class__.__name__))  # 動作状況を情報ログとして出力する
                    self.state = JState.JOINING  # self.state に、このオブジェクトで使用する値を設定する
                elif (self.target_state == JState.FORKING or self.target_state == JState.FORKED) and roe >= JUNCT_LOWER_THRESH and self.prev_roe <= JUNCT_LOWER_THRESH:  # 前の条件が不成立の場合に追加条件を判定する
                    self.logger.info("%+06d %s.lines are forking" % (g_plotter.get_distance(), self.__class__.__name__))  # 動作状況を情報ログとして出力する
                    self.state = JState.FORKING  # self.state に、このオブジェクトで使用する値を設定する
            elif self.state == JState.JOINING:  # 前の条件が不成立の場合に追加条件を判定する
                if roe <= JUNCT_LOWER_THRESH:  # 条件を判定し、成立した場合の処理を行う
                    self.logger.info("%+06d %s.the join completed" % (g_plotter.get_distance(), self.__class__.__name__))  # 動作状況を情報ログとして出力する
                    self.state = JState.JOINED  # self.state に、このオブジェクトで使用する値を設定する
                    
            elif self.state == JState.FORKING:  # 前の条件が不成立の場合に追加条件を判定する
                if roe <= JUNCT_LOWER_THRESH and self.prev_roe >= JUNCT_UPPER_THRESH:  # 条件を判定し、成立した場合の処理を行う
                    self.logger.info("%+06d %s.the fork completed" % (g_plotter.get_distance(), self.__class__.__name__))  # 動作状況を情報ログとして出力する
                    self.state = JState.FORKED  # self.state に、このオブジェクトで使用する値を設定する
            else:  # 上記の条件に当てはまらない場合の処理を行う
                pass  # この条件では特別な処理を行わない
        self.prev_roe = roe  # self.prev_roe に、このオブジェクトで使用する値を設定する

        if not self.reached and self.state == self.target_state:  # 条件を判定し、成立した場合の処理を行う
            self.reached = True  # self.reached に、このオブジェクトで使用する値を設定する
            self.logger.info("%+06d %s.target state reached" % (g_plotter.get_distance(), self.__class__.__name__))  # 動作状況を情報ログとして出力する
            return Status.SUCCESS  # 処理結果を呼び出し元へ返す
        else:  # 上記の条件に当てはまらない場合の処理を行う
            return Status.RUNNING  # 処理結果を呼び出し元へ返す


class CatchBottle(Behaviour):  # CatchBottle クラスを定義する
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
    IDENTIFY, APPROACH, CATCH = range(3)  # IDENTIFY, APPROACH, CATCH に処理で使用する値を設定する

    def __init__(self, name: str, power: int,  # __init__ メソッド／関数を定義する
                 pid_p: float, pid_i: float, pid_d: float,  # 直前の定義・関数呼び出しに渡す値を指定する
                 catch_run_mm: int = 150,  # catch_run_mm: int に処理で使用する値を設定する
                 lock_color: 'BottleColor' = None,   # force a colour, else auto  # lock_color: 'BottleColor' に処理で使用する値を設定する
                 identify_area: int = 400,           # min area to trust the colour  # identify_area: int に処理で使用する値を設定する
                 identify_frames: int = 3,           # consecutive solid frames  # identify_frames: int に処理で使用する値を設定する
                 heading_avg_frames: int = 5,        # smooth heading before the run-in  # heading_avg_frames: int に処理で使用する値を設定する
                 ) -> None:  # 直前から続く関数呼び出しまたは定義を閉じる
        super(CatchBottle, self).__init__(name)  # 親クラスの初期化処理を呼び出す
        self.power = power  # self.power に、このオブジェクトで使用する値を設定する
        self.pid_p, self.pid_i, self.pid_d = pid_p, pid_i, pid_d  # self.pid_p, self.pid_i, self.pid_d に、このオブジェクトで使用する値を設定する
        self.catch_run_mm = catch_run_mm  # self.catch_run_mm に、このオブジェクトで使用する値を設定する
        self.lock_color = lock_color  # self.lock_color に、このオブジェクトで使用する値を設定する
        self.identify_area = identify_area  # self.identify_area に、このオブジェクトで使用する値を設定する
        self.identify_frames = identify_frames  # self.identify_frames に、このオブジェクトで使用する値を設定する
        self.heading_avg_frames = heading_avg_frames  # self.heading_avg_frames に、このオブジェクトで使用する値を設定する
        self._state = self.IDENTIFY  # self._state に、このオブジェクトで使用する値を設定する
        self._solid = 0  # self._solid に、このオブジェクトで使用する値を設定する
        self._heading_hist = []  # self._heading_hist に、このオブジェクトで使用する値を設定する
        self._catch_start_odo = None  # self._catch_start_odo に、このオブジェクトで使用する値を設定する
        self._target_heading = None  # self._target_heading に、このオブジェクトで使用する値を設定する
        self._blind_steer = 0  # self._blind_steer に、このオブジェクトで使用する値を設定する
        self.pid = None  # self.pid に、このオブジェクトで使用する値を設定する
        self.running = False  # self.running に、このオブジェクトで使用する値を設定する

    def _cur_heading(self) -> int:  # _cur_heading メソッド／関数を定義する
        return (-1) * g_course * g_gyro_sensor.get_angle()  # 処理結果を呼び出し元へ返す

    def _steer_vision(self, theta: float) -> None:  # _steer_vision メソッド／関数を定義する
        # vision-based: theta already encodes direction, no g_course factor
        turn = int(self.pid(theta))  # turn に処理で使用する値を設定する
        g_right_motor.set_power(self.power + turn)  # モータの出力値を設定する
        g_left_motor.set_power(self.power - turn)  # モータの出力値を設定する

    def update(self) -> Status:  # update メソッド／関数を定義する
        global g_bottle_color  # この関数内で使用するグローバル変数を宣言する

        if not self.running:  # 条件を判定し、成立した場合の処理を行う
            self.running = True  # self.running に、このオブジェクトで使用する値を設定する
            g_video.set_target_interested(TargetInterested.BOTTLE)  # この処理を実行する
            if self.lock_color is not None:  # 条件を判定し、成立した場合の処理を行う
                g_video.set_bottle_color(self.lock_color)  # この処理を実行する
            self.pid = PID(self.pid_p, self.pid_i, self.pid_d, setpoint=0,  # 指定したゲインと目標値でPID制御器を生成する
                           sample_time=EXEC_INTERVAL,  # sample_time に処理で使用する値を設定する
                           output_limits=(-self.power, self.power))  # output_limits に処理で使用する値を設定する
            self.logger.info("%+06d %s.started" % (g_plotter.get_distance(), self.__class__.__name__))  # 動作状況を情報ログとして出力する

        insight, color, bcx, btheta, bbottom, barea, in_blind = g_video.get_bottle_stamped()  # insight, color, bcx, btheta, bbottom, barea, in_blind に処理で使用する値を設定する

        # ---------- IDENTIFY ----------
        if self._state == self.IDENTIFY:  # 条件を判定し、成立した場合の処理を行う
            self._solid = self._solid + 1 if (insight and barea >= self.identify_area) else 0  # この処理を実行する
            self._steer_vision(btheta if insight else 0.0)   # creep while identifying  # この処理を実行する
            if self._solid >= self.identify_frames:  # 条件を判定し、成立した場合の処理を行う
                g_bottle_color = color  # g_bottle_color に、プログラム全体で共有する値を設定する
                g_video.set_bottle_color(color)              # lock -> stable bearing  # この処理を実行する
                self.logger.info("%+06d %s.color=%s area=%d -> APPROACH" % (  # 動作状況を情報ログとして出力する
                    g_plotter.get_distance(), self.__class__.__name__, color.name, barea))  # この処理を実行する
                self._state = self.APPROACH  # self._state に、このオブジェクトで使用する値を設定する
            return Status.RUNNING  # 処理結果を呼び出し元へ返す

        # ---------- APPROACH ----------
        if self._state == self.APPROACH:  # 条件を判定し、成立した場合の処理を行う
            if insight:  # 条件を判定し、成立した場合の処理を行う
                self._blind_steer = 0  # self._blind_steer に、このオブジェクトで使用する値を設定する
                self._steer_vision(btheta)  # この処理を実行する
                self._heading_hist.append(self._cur_heading())   # log heading while locked  # この処理を実行する
                if len(self._heading_hist) > self.heading_avg_frames:  # 条件を判定し、成立した場合の処理を行う
                    self._heading_hist.pop(0)  # この処理を実行する
            else:  # 上記の条件に当てはまらない場合の処理を行う
                self._blind_steer += 1                            # brief dropout: hold straight  # self._blind_steer + に、このオブジェクトで使用する値を設定する
                g_right_motor.set_power(self.power)  # モータの出力値を設定する
                g_left_motor.set_power(self.power)  # モータの出力値を設定する

            # band reached / passed the blind edge -> commit to the run-in
            if in_blind or (not insight and self._blind_steer > 8):  # 条件を判定し、成立した場合の処理を行う
                hist = self._heading_hist or [self._cur_heading()]  # hist に処理で使用する値を設定する
                self._target_heading = sum(hist) / len(hist)  # self._target_heading に、このオブジェクトで使用する値を設定する
                self._catch_start_odo = g_plotter.get_distance()  # 現在の走行距離またはセンサ距離を取得して変数に保存する
                self.pid = PID(self.pid_p, self.pid_i, self.pid_d,  # 指定したゲインと目標値でPID制御器を生成する
                               setpoint=self._target_heading,  # setpoint に処理で使用する値を設定する
                               sample_time=EXEC_INTERVAL,  # sample_time に処理で使用する値を設定する
                               output_limits=(-self.power, self.power))  # output_limits に処理で使用する値を設定する
                self.logger.info("%+06d %s.blind edge, heading=%.1f -> CATCH(+%dmm)" % (  # 動作状況を情報ログとして出力する
                    g_plotter.get_distance(), self.__class__.__name__,  # 直前の定義・関数呼び出しに渡す値を指定する
                    self._target_heading, self.catch_run_mm))  # この処理を実行する
                self._state = self.CATCH  # self._state に、このオブジェクトで使用する値を設定する
            return Status.RUNNING  # 処理結果を呼び出し元へ返す

        # ---------- CATCH (gyro run-in) ----------
        if self._state == self.CATCH:  # 条件を判定し、成立した場合の処理を行う
            travelled = g_plotter.get_distance() - self._catch_start_odo  # 現在の走行距離またはセンサ距離を取得して変数に保存する
            if travelled >= self.catch_run_mm:  # 条件を判定し、成立した場合の処理を行う
                g_right_motor.set_power(0)  # モータの出力値を設定する
                g_left_motor.set_power(0)  # モータの出力値を設定する
                self.logger.info("%+06d %s.caught (ran %dmm, color=%s)" % (  # 動作状況を情報ログとして出力する
                    g_plotter.get_distance(), self.__class__.__name__,  # 直前の定義・関数呼び出しに渡す値を指定する
                    travelled, g_bottle_color.name))  # この処理を実行する
                return Status.SUCCESS  # 処理結果を呼び出し元へ返す
            turn = int(self.pid(self._cur_heading()))  # turn に処理で使用する値を設定する
            g_right_motor.set_power(self.power + g_course * turn)  # モータの出力値を設定する
            g_left_motor.set_power(self.power - g_course * turn)  # モータの出力値を設定する
            return Status.RUNNING  # 処理結果を呼び出し元へ返す

        return Status.RUNNING  # 処理結果を呼び出し元へ返す


class IsBottleInsight(Behaviour):  # IsBottleInsight クラスを定義する
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
    def __init__(self, name: str, color: 'BottleColor',  # __init__ メソッド／関数を定義する
                 min_area: int = 150,     # ignore specks below this contour area  # min_area: int に処理で使用する値を設定する
                 min_frames: int = 2,     # consecutive matching frames to assert SUCCESS  # min_frames: int に処理で使用する値を設定する
                 set_target: bool = True,  # put the camera in BOTTLE mode on first tick  # set_target: bool に処理で使用する値を設定する
                 ) -> None:  # 直前から続く関数呼び出しまたは定義を閉じる
        super(IsBottleInsight, self).__init__(name)  # 親クラスの初期化処理を呼び出す
        self.color = color  # self.color に、このオブジェクトで使用する値を設定する
        self.min_area = min_area  # self.min_area に、このオブジェクトで使用する値を設定する
        self.min_frames = min_frames  # self.min_frames に、このオブジェクトで使用する値を設定する
        self.set_target = set_target  # self.set_target に、このオブジェクトで使用する値を設定する
        self._hits = 0  # self._hits に、このオブジェクトで使用する値を設定する
        self.running = False  # self.running に、このオブジェクトで使用する値を設定する

    def update(self) -> Status:  # update メソッド／関数を定義する
        if not self.running:  # 条件を判定し、成立した場合の処理を行う
            self.running = True  # self.running に、このオブジェクトで使用する値を設定する
            if self.set_target:  # 条件を判定し、成立した場合の処理を行う
                g_video.set_target_interested(TargetInterested.BOTTLE)  # この処理を実行する
            self.logger.info("%+06d %s.watching for color=%s" % (  # 動作状況を情報ログとして出力する
                g_plotter.get_distance(), self.__class__.__name__, self.color.name))  # この処理を実行する

        insight, color, bcx, btheta, bbottom, barea, in_blind = g_video.get_bottle_stamped()  # insight, color, bcx, btheta, bbottom, barea, in_blind に処理で使用する値を設定する

        match = (insight  # match に処理で使用する値を設定する
                 and barea >= self.min_area  # この処理を実行する
                 and (self.color == BottleColor.NONE or color == self.color))  # この処理を実行する

        self._hits = self._hits + 1 if match else 0  # self._hits に、このオブジェクトで使用する値を設定する

        if self._hits >= self.min_frames:  # 条件を判定し、成立した場合の処理を行う
            return Status.SUCCESS  # 処理結果を呼び出し元へ返す
        return Status.FAILURE  # 処理結果を呼び出し元へ返す


class HasCaughtBottle(Behaviour):  # HasCaughtBottle クラスを定義する
    """
    Condition node: SUCCESS if the colour latched by CatchBottle (g_bottle_color)
    matches `color`, FAILURE otherwise.

    - color == BottleColor.NONE -> SUCCESS if ANY bottle has been caught
      (i.e. g_bottle_color is no longer NONE).
    - color == a specific colour -> SUCCESS only when that exact colour was caught.

    Pure read of g_bottle_color; no camera or motor side effects.
    """
    def __init__(self, name: str, color: 'BottleColor') -> None:  # __init__ メソッド／関数を定義する
        super(HasCaughtBottle, self).__init__(name)  # 親クラスの初期化処理を呼び出す
        self.color = color  # self.color に、このオブジェクトで使用する値を設定する

    def update(self) -> Status:  # update メソッド／関数を定義する
        if self.color == BottleColor.NONE:  # 条件を判定し、成立した場合の処理を行う
            caught = (g_bottle_color != BottleColor.NONE)  # この処理を実行する
        else:  # 上記の条件に当てはまらない場合の処理を行う
            caught = (g_bottle_color == self.color)  # この処理を実行する

        self.logger.info("%+06d %s.want=%s caught=%s -> %s" % (  # 動作状況を情報ログとして出力する
            g_plotter.get_distance(), self.__class__.__name__,  # 直前の定義・関数呼び出しに渡す値を指定する
            self.color.name, g_bottle_color.name,  # 直前の定義・関数呼び出しに渡す値を指定する
            "SUCCESS" if caught else "FAILURE"))  # この処理を実行する

        return Status.SUCCESS if caught else Status.FAILURE  # 処理結果を呼び出し元へ返す


class TraverseBehaviourTree(object):  # TraverseBehaviourTree クラスを定義する
    def __init__(self, tree: BehaviourTree) -> None:  # __init__ メソッド／関数を定義する
        self.tree = tree  # self.tree に、このオブジェクトで使用する値を設定する
        self.last_log_time = None  # self.last_log_time に、このオブジェクトで使用する値を設定する
        self.running = False  # self.running に、このオブジェクトで使用する値を設定する
    def __call__(  # __call__ メソッド／関数を定義する
        self,  # 直前の定義・関数呼び出しに渡す値を指定する
        hub: Hub,  # 直前の定義・関数呼び出しに渡す値を指定する
        arm_motor: Motor,  # 直前の定義・関数呼び出しに渡す値を指定する
        right_motor: Motor,  # 直前の定義・関数呼び出しに渡す値を指定する
        left_motor: Motor,  # 直前の定義・関数呼び出しに渡す値を指定する
        touch_sensor: TouchSensor,  # 直前の定義・関数呼び出しに渡す値を指定する
        color_sensor: ColorSensor,  # 直前の定義・関数呼び出しに渡す値を指定する
        sonar_sensor: SonarSensor,  # 直前の定義・関数呼び出しに渡す値を指定する
        gyro_sensor: GyroSensor,  # 直前の定義・関数呼び出しに渡す値を指定する
    ) -> None:  # 直前から続く関数呼び出しまたは定義を閉じる
        global g_hub, g_arm_motor, g_right_motor, g_left_motor, g_touch_sensor, g_color_sensor, g_sonar_sensor, g_gyro_sensor, g_plotter  # この関数内で使用するグローバル変数を宣言する
        if not self.running:  # 条件を判定し、成立した場合の処理を行う
            g_hub = hub  # g_hub に、プログラム全体で共有する値を設定する
            g_arm_motor = arm_motor  # g_arm_motor に、プログラム全体で共有する値を設定する
            g_right_motor = right_motor  # g_right_motor に、プログラム全体で共有する値を設定する
            g_left_motor = left_motor  # g_left_motor に、プログラム全体で共有する値を設定する
            g_touch_sensor = touch_sensor  # g_touch_sensor に、プログラム全体で共有する値を設定する
            g_color_sensor = color_sensor  # g_color_sensor に、プログラム全体で共有する値を設定する
            g_sonar_sensor = sonar_sensor  # g_sonar_sensor に、プログラム全体で共有する値を設定する
            g_gyro_sensor = gyro_sensor  # g_gyro_sensor に、プログラム全体で共有する値を設定する
            g_plotter = Plotter()  # g_plotter に、プログラム全体で共有する値を設定する
            print(" -- TraverseBehaviorTree initialization complete")  # メッセージをコンソールに出力する
            self.running = True  # self.running に、このオブジェクトで使用する値を設定する
        else:  # 上記の条件に当てはまらない場合の処理を行う
            self.tree.tick_once()  # この処理を実行する
            g_plotter.plot(hub, arm_motor, right_motor, left_motor, touch_sensor, color_sensor, sonar_sensor, gyro_sensor)  # この処理を実行する
            # log estimated position every 1 second
            #if self.last_log_time == None or time.time() - self.last_log_time >= 1.0:
            #    print(" --  estimated position X=%d, Y=%d" % (g_plotter.get_loc_x(), g_plotter.get_loc_y()))
            #    self.last_log_time = time.time()


class VideoThread(threading.Thread):  # VideoThread クラスを定義する
    def __init__(self):  # __init__ メソッド／関数を定義する
        super().__init__()  # 親クラスの初期化処理を呼び出す
        self._stop_event = threading.Event()  # self._stop_event に、このオブジェクトで使用する値を設定する
        self.prev_time = time.time()  # 現在時刻を取得して時間計測に使用する

    def stop(self):  # stop メソッド／関数を定義する
        self._stop_event.set()  # この処理を実行する

    def run(self):  # run メソッド／関数を定義する
        while not self._stop_event.is_set():  # 条件が成立している間、処理を繰り返す
            g_video.process(g_plotter, g_hub, g_arm_motor, g_right_motor, g_left_motor, g_color_sensor, g_sonar_sensor, g_gyro_sensor)  # この処理を実行する
            current_time = time.time()  # 現在時刻を取得して時間計測に使用する
            elapsed_time = current_time - self.prev_time  # elapsed_time に処理で使用する値を設定する
            self.prev_time = current_time  # self.prev_time に、このオブジェクトで使用する値を設定する
            if elapsed_time < VIDEO_INTERVAL:  # 条件を判定し、成立した場合の処理を行う
                time.sleep(VIDEO_INTERVAL - elapsed_time)  # 指定時間だけ処理を待機する


def build_behaviour_tree() -> BehaviourTree:  # build_behaviour_tree メソッド／関数を定義する
    root = Sequence(name="2026 base", memory=True)  # root に処理で使用する値を設定する
    calibration = Sequence(name="calibration", memory=True)  # calibration に処理で使用する値を設定する
    start = Parallel(name="start", policy=ParallelPolicy.SuccessOnOne())  # start に処理で使用する値を設定する
    lap2 = Parallel(name="lap2", policy=ParallelPolicy.SuccessOnOne())  # lap2 に処理で使用する値を設定する
    lap3 = Parallel(name="lap3", policy=ParallelPolicy.SuccessOnOne())  # lap3 に処理で使用する値を設定する
    carry1 = Parallel(name="carry1", policy=ParallelPolicy.SuccessOnOne())  # carry1 に処理で使用する値を設定する
    carry2 = Parallel(name="carry2", policy=ParallelPolicy.SuccessOnOne())  # carry2 に処理で使用する値を設定する
    carry3 = Parallel(name="carry3", policy=ParallelPolicy.SuccessOnOne())  # carry3 に処理で使用する値を設定する
    carry4 = Parallel(name="carry4", policy=ParallelPolicy.SuccessOnOne())  # carry4 に処理で使用する値を設定する
    qr2 = Parallel(name="qr2", policy=ParallelPolicy.SuccessOnOne())  # qr2 に処理で使用する値を設定する
    qr3 = Parallel(name="qr3", policy=ParallelPolicy.SuccessOnOne())  # qr3 に処理で使用する値を設定する
    qr4 = Parallel(name="qr4", policy=ParallelPolicy.SuccessOnOne())  # qr4 に処理で使用する値を設定する
    qr5 = Parallel(name="qr4", policy=ParallelPolicy.SuccessOnOne())  # qr5 に処理で使用する値を設定する
    qr_read = Parallel(name="qr_read", policy=ParallelPolicy.SuccessOnOne())  # qr_read に処理で使用する値を設定する
    qr_scan_shake = Sequence(name="qr_scan_shake", memory=True)  # qr_scan_shake に処理で使用する値を設定する
    qr_scan_move_back = Parallel(name="qr_scan_move_back2", policy=ParallelPolicy.SuccessOnOne())  # qr_scan_move_back に処理で使用する値を設定する
    calibration.add_children(  # ビヘイビアツリーに子ノードを追加する
        [  # 直前から続く定義・引数・リストを閉じる
            ArmUpDownFull(name="arm up", direction=ArmDirection.UP),  # ArmUpDownFull(name に処理で使用する値を設定する
            ArmUpDownFull(name="arm down", direction=ArmDirection.DOWN),  # ArmUpDownFull(name に処理で使用する値を設定する
            ResetDevice(name="device reset"),  # ResetDevice(name に処理で使用する値を設定する
            #ReadKey(name="read key"),
        ]  # 直前から続く定義・引数・リストを閉じる
    )  # 直前から続く定義・引数・リストを閉じる
    start.add_children(  # ビヘイビアツリーに子ノードを追加する
        [  # 直前から続く定義・引数・リストを閉じる
            IsTouchOn(name="touch start"),  # IsTouchOn(name に処理で使用する値を設定する
            ReadKey
        ]  # 直前から続く定義・引数・リストを閉じる
    )  # 直前から続く定義・引数・リストを閉じる
    lap2.add_children(  # ビヘイビアツリーに子ノードを追加する
        [  # 直前から続く定義・引数・リストを閉じる
            TraceLine(name="sensor trace normal edge", target=TRACELINE_TARGET_V,  # TraceLine(name に処理で使用する値を設定する
                power=70, power_min=33,  # power に処理で使用する値を設定する
                pid_p=0.65, pid_i=0.000001, pid_d=0.045,  # pid_p に処理で使用する値を設定する
                err_lo=6, err_hi=16, decel_per_s=350, gains_slow=(0.65, 0.045), gains_fast=(0.55, 0.065),  # err_lo に処理で使用する値を設定する
                recover_v=97, recover_after=3, recover_turn=35,  # recover_v に処理で使用する値を設定する
                trace_side=TraceSide.NORMAL),  # trace_side に処理で使用する値を設定する
            IsColorDetected(name="check color", color=Color.BLUE),  # IsColorDetected(name に処理で使用する値を設定する
        ]  # 直前から続く定義・引数・リストを閉じる
    )  # 直前から続く定義・引数・リストを閉じる
    lap3.add_children(  # ビヘイビアツリーに子ノードを追加する
        [  # 直前から続く定義・引数・リストを閉じる
            RunByGyro(name="run straight to catch the bottle", target=3, power=33,  # RunByGyro(name に処理で使用する値を設定する
                pid_p=1.1, pid_i=0.1, pid_d=0.03, target_type=HeadingType.ABSOLUTE),  # pid_p に処理で使用する値を設定する
            IsDistanceEarned(name="check distance", delta_dist = 370),  # IsDistanceEarned(name に処理で使用する値を設定する
        ]  # 直前から続く定義・引数・リストを閉じる
    )  # 直前から続く定義・引数・リストを閉じる
    carry1.add_children(  # ビヘイビアツリーに子ノードを追加する
        [  # 直前から続く定義・引数・リストを閉じる
            TraceLine(name="sensor trace normal edge", target=TRACELINE_TARGET_V,  # TraceLine(name に処理で使用する値を設定する
                power=70, power_min=33,  # power に処理で使用する値を設定する
                pid_p=0.65, pid_i=0.000001, pid_d=0.045,  # pid_p に処理で使用する値を設定する
                err_lo=6, err_hi=16, decel_per_s=350, gains_slow=(0.65, 0.045), gains_fast=(0.55, 0.065),  # err_lo に処理で使用する値を設定する
                recover_v=97, recover_after=3, recover_turn=35,  # recover_v に処理で使用する値を設定する
                trace_side=TraceSide.NORMAL),  # trace_side に処理で使用する値を設定する
            IsColorDetected(name="check color", color=Color.BLUE),  # IsColorDetected(name に処理で使用する値を設定する
        ]  # 直前から続く定義・引数・リストを閉じる
    )  # 直前から続く定義・引数・リストを閉じる
    carry2.add_children(  # ビヘイビアツリーに子ノードを追加する
        [  # 直前から続く定義・引数・リストを閉じる
            RunByGyro(name="run straight to pass the blue line", target=90, power=33,  # RunByGyro(name に処理で使用する値を設定する
                pid_p=1.1, pid_i=0.1, pid_d=0.03, target_type=HeadingType.ABSOLUTE),  # pid_p に処理で使用する値を設定する
            IsDistanceEarned(name="check distance", delta_dist = 120),  # IsDistanceEarned(name に処理で使用する値を設定する
        ]  # 直前から続く定義・引数・リストを閉じる
    )  # 直前から続く定義・引数・リストを閉じる
    carry3.add_children(  # ビヘイビアツリーに子ノードを追加する
        [  # 直前から続く定義・引数・リストを閉じる
            TraceLine(name="sensor trace normal edge", target=TRACELINE_TARGET_V,  # TraceLine(name に処理で使用する値を設定する
                power=70, power_min=33,  # power に処理で使用する値を設定する
                pid_p=0.65, pid_i=0.000001, pid_d=0.045,  # pid_p に処理で使用する値を設定する
                err_lo=6, err_hi=16, decel_per_s=350, gains_slow=(0.65, 0.045), gains_fast=(0.55, 0.065),  # err_lo に処理で使用する値を設定する
                recover_v=97, recover_after=3, recover_turn=35,  # recover_v に処理で使用する値を設定する
                trace_side=TraceSide.NORMAL),  # trace_side に処理で使用する値を設定する
            IsDistanceEarned(name="check distance", delta_dist = 1100),  # IsDistanceEarned(name に処理で使用する値を設定する
        ]  # 直前から続く定義・引数・リストを閉じる
    )  # 直前から続く定義・引数・リストを閉じる
    carry4.add_children(  # ビヘイビアツリーに子ノードを追加する
        [  # 直前から続く定義・引数・リストを閉じる
            TraceLine(name="sensor trace normal edge", target=TRACELINE_TARGET_V,  # TraceLine(name に処理で使用する値を設定する
                power=33,  # power に処理で使用する値を設定する
                pid_p=0.65, pid_i=0.000001, pid_d=0.045,  # pid_p に処理で使用する値を設定する
                err_lo=6, err_hi=16, decel_per_s=350, gains_slow=(0.65, 0.045), gains_fast=(0.55, 0.065),  # err_lo に処理で使用する値を設定する
                recover_v=97, recover_after=3, recover_turn=35,  # recover_v に処理で使用する値を設定する
                trace_side=TraceSide.NORMAL),  # trace_side に処理で使用する値を設定する
            IsColorDetected(name="check color", color=Color.BLUE),  # IsColorDetected(name に処理で使用する値を設定する
        ]  # 直前から続く定義・引数・リストを閉じる
    )  # 直前から続く定義・引数・リストを閉じる
    qr2.add_children(  # ビヘイビアツリーに子ノードを追加する
        [  # 直前から続く定義・引数・リストを閉じる
            RunByGyro(name="run straight to correct heading", target=0, power=33,  # RunByGyro(name に処理で使用する値を設定する
                pid_p=1.1, pid_i=0.1, pid_d=0.03, target_type=HeadingType.ABSOLUTE),  # pid_p に処理で使用する値を設定する
            IsDistanceEarned(name="check distance", delta_dist = 50),  # IsDistanceEarned(name に処理で使用する値を設定する
        ]  # 直前から続く定義・引数・リストを閉じる
    )  # 直前から続く定義・引数・リストを閉じる
    qr3.add_children(  # ビヘイビアツリーに子ノードを追加する
        [  # 直前から続く定義・引数・リストを閉じる
            TraceLine(name="sensor trace opposite edge", target=TRACELINE_TARGET_V,  # TraceLine(name に処理で使用する値を設定する
                power=33,  # power に処理で使用する値を設定する
                pid_p=0.65, pid_i=0.000001, pid_d=0.045,  # pid_p に処理で使用する値を設定する
                err_lo=6, err_hi=16, decel_per_s=350, gains_slow=(0.65, 0.045), gains_fast=(0.55, 0.065),  # err_lo に処理で使用する値を設定する
                recover_v=97, recover_after=3, recover_turn=35,  # recover_v に処理で使用する値を設定する
                trace_side=TraceSide.OPPOSITE),  # trace_side に処理で使用する値を設定する
            IsDistanceEarned(name="check distance", delta_dist = 500),  # IsDistanceEarned(name に処理で使用する値を設定する
        ]  # 直前から続く定義・引数・リストを閉じる
    )  # 直前から続く定義・引数・リストを閉じる
    qr4.add_children(  # ビヘイビアツリーに子ノードを追加する
        [  # 直前から続く定義・引数・リストを閉じる
            TraceLine(name="sensor trace opposite edge", target=TRACELINE_TARGET_V,  # TraceLine(name に処理で使用する値を設定する
                power=70, power_min=33,  # power に処理で使用する値を設定する
                pid_p=0.65, pid_i=0.000001, pid_d=0.045,  # pid_p に処理で使用する値を設定する
                err_lo=6, err_hi=16, decel_per_s=350, gains_slow=(0.65, 0.045), gains_fast=(0.55, 0.065),  # err_lo に処理で使用する値を設定する
                recover_v=97, recover_after=3, recover_turn=35,  # recover_v に処理で使用する値を設定する
                trace_side=TraceSide.OPPOSITE),  # trace_side に処理で使用する値を設定する
            IsColorDetected(name="check color", color=Color.BLUE),  # IsColorDetected(name に処理で使用する値を設定する
        ]  # 直前から続く定義・引数・リストを閉じる
    )  # 直前から続く定義・引数・リストを閉じる
    qr5.add_children(  # ビヘイビアツリーに子ノードを追加する
        [  # 直前から続く定義・引数・リストを閉じる
            RunByGyro(name="run straight to pass half the blue line", target=-90, power=33,  # RunByGyro(name に処理で使用する値を設定する
                pid_p=1.1, pid_i=0.1, pid_d=0.03, target_type=HeadingType.ABSOLUTE),  # pid_p に処理で使用する値を設定する
            IsDistanceEarned(name="check distance", delta_dist = 100),  # IsDistanceEarned(name に処理で使用する値を設定する
        ]  # 直前から続く定義・引数・リストを閉じる
    )  # 直前から続く定義・引数・リストを閉じる
    qr_scan_move_back.add_children(  # ビヘイビアツリーに子ノードを追加する
        [  # 直前から続く定義・引数・リストを閉じる
            RunAsInstructed(name="move back a little", pwm_l=-SPIN_MIN_POWER, pwm_r=-SPIN_MIN_POWER),  # RunAsInstructed(name に処理で使用する値を設定する
            IsDistanceEarned(name="check distance", delta_dist = 50),  # IsDistanceEarned(name に処理で使用する値を設定する
        ]  # 直前から続く定義・引数・リストを閉じる
    )  # 直前から続く定義・引数・リストを閉じる
    qr_scan_shake.add_children(  # ビヘイビアツリーに子ノードを追加する
        [  # 直前から続く定義・引数・リストを閉じる
            IsTimePassed(name="wait for a moment", delta_time=3.0),  # IsTimePassed(name に処理で使用する値を設定する
            qr_scan_move_back,  # 直前の定義・関数呼び出しに渡す値を指定する
            StopNow(name="stop"),  # StopNow(name に処理で使用する値を設定する
            IsTimePassed(name="wait for a moment", delta_time=3.0),  # IsTimePassed(name に処理で使用する値を設定する
            SpinAround(name="scan for QR code", target=3, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,  # SpinAround(name に処理で使用する値を設定する
                pid_p=0.4, pid_i=0.001, pid_d=0.03, target_type=HeadingType.RELATIVE),  # pid_p に処理で使用する値を設定する
            StopNow(name="stop"),  # StopNow(name に処理で使用する値を設定する
            IsTimePassed(name="wait for a moment", delta_time=2.0),  # IsTimePassed(name に処理で使用する値を設定する
            SpinAround(name="scan for QR code", target=-6, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,  # SpinAround(name に処理で使用する値を設定する
                pid_p=0.4, pid_i=0.001, pid_d=0.03, target_type=HeadingType.RELATIVE),  # pid_p に処理で使用する値を設定する
            StopNow(name="stop"),  # StopNow(name に処理で使用する値を設定する
            IsTimePassed(name="wait for a moment", delta_time=2.0),  # IsTimePassed(name に処理で使用する値を設定する
            SpinAround(name="scan for QR code", target=3, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,  # SpinAround(name に処理で使用する値を設定する
                pid_p=0.4, pid_i=0.001, pid_d=0.03, target_type=HeadingType.RELATIVE),  # pid_p に処理で使用する値を設定する
            StopNow(name="stop"),  # StopNow(name に処理で使用する値を設定する
            IsTimePassed(name="wait for a moment", delta_time=3.0),  # IsTimePassed(name に処理で使用する値を設定する
        ]  # 直前から続く定義・引数・リストを閉じる
    )  # 直前から続く定義・引数・リストを閉じる
    qr_read.add_children(  # ビヘイビアツリーに子ノードを追加する
        [  # 直前から続く定義・引数・リストを閉じる
            IsQRDecoded(name="check QR code"),  # IsQRDecoded(name に処理で使用する値を設定する
            qr_scan_shake,  # 直前の定義・関数呼び出しに渡す値を指定する
        ]  # 直前から続く定義・引数・リストを閉じる
    )  # 直前から続く定義・引数・リストを閉じる
    root.add_children(  # ビヘイビアツリーに子ノードを追加する
        [  # 直前から続く定義・引数・リストを閉じる
            calibration,  # 直前の定義・関数呼び出しに渡す値を指定する
            start,  # 直前の定義・関数呼び出しに渡す値を指定する
            lap2,  # 直前の定義・関数呼び出しに渡す値を指定する
            lap3,  # 直前の定義・関数呼び出しに渡す値を指定する
            carry1,  # 直前の定義・関数呼び出しに渡す値を指定する
            carry2,  # 直前の定義・関数呼び出しに渡す値を指定する
            carry3,  # 直前の定義・関数呼び出しに渡す値を指定する
            carry4,  # 直前の定義・関数呼び出しに渡す値を指定する
            SpinAround(name="about the face", target=10, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,  # SpinAround(name に処理で使用する値を設定する
                pid_p=0.4, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),  # pid_p に処理で使用する値を設定する
            qr2,  # 直前の定義・関数呼び出しに渡す値を指定する
            StopNow(name="stop"),  # StopNow(name に処理で使用する値を設定する
            SpinAndLocateLine(name="spin and locate line", target=TRACELINE_TARGET_V, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,  # SpinAndLocateLine(name に処理で使用する値を設定する
                pid_p=0.4, pid_i=0.001, pid_d=0.03, trace_side=TraceSide.OPPOSITE),  # pid_p に処理で使用する値を設定する
            StopNow(name="stop"),  # StopNow(name に処理で使用する値を設定する
            qr3,  # 直前の定義・関数呼び出しに渡す値を指定する
            qr4,  # 直前の定義・関数呼び出しに渡す値を指定する
            qr5,  # 直前の定義・関数呼び出しに渡す値を指定する
            StopNow(name="stop"),  # StopNow(name に処理で使用する値を設定する
            ArmUpDownFull(name="arm up", direction=ArmDirection.UP),  # ArmUpDownFull(name に処理で使用する値を設定する
            SpinAround(name="align for QR code scanning", target=0, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,  # SpinAround(name に処理で使用する値を設定する
                pid_p=0.4, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),  # pid_p に処理で使用する値を設定する
            StopNow(name="stop"),  # StopNow(name に処理で使用する値を設定する
            qr_read,  # 直前の定義・関数呼び出しに渡す値を指定する
            ArmUpDownFull(name="arm down", direction=ArmDirection.DOWN),  # ArmUpDownFull(name に処理で使用する値を設定する
            StopNow(name="stop"),  # StopNow(name に処理で使用する値を設定する
            TheEnd(name="end"),  # TheEnd(name に処理で使用する値を設定する
        ]  # 直前から続く定義・引数・リストを閉じる
    )  # 直前から続く定義・引数・リストを閉じる
    return root  # 処理結果を呼び出し元へ返す

def initialize_etrobo(backend: str) -> ETRobo:  # initialize_etrobo メソッド／関数を定義する
    return (ETRobo(backend=backend)  # 処理結果を呼び出し元へ返す
            .add_hub('hub')  # この処理を実行する
            .add_device('arm_motor', device_type=Motor, port='C')  # .add_device('arm_motor', device_type に処理で使用する値を設定する
            .add_device('right_motor', device_type=Motor, port='A')  # .add_device('right_motor', device_type に処理で使用する値を設定する
            .add_device('left_motor', device_type=Motor, port='B')  # .add_device('left_motor', device_type に処理で使用する値を設定する
            .add_device('touch_sensor', device_type=TouchSensor, port='D')  # .add_device('touch_sensor', device_type に処理で使用する値を設定する
            .add_device('color_sensor', device_type=ColorSensor, port='E')  # .add_device('color_sensor', device_type に処理で使用する値を設定する
            .add_device('sonar_sensor', device_type=SonarSensor, port='F')  # .add_device('sonar_sensor', device_type に処理で使用する値を設定する
            .add_device('gyro_sensor', device_type=GyroSensor, port='')  # .add_device('gyro_sensor', device_type に処理で使用する値を設定する
    )  # 直前から続く定義・引数・リストを閉じる

def setup_thread():  # setup_thread メソッド／関数を定義する
    global g_video, g_video_thread  # この関数内で使用するグローバル変数を宣言する
    g_video = Video()  # g_video に、プログラム全体で共有する値を設定する

    print(" -- starting VideoThread...")  # メッセージをコンソールに出力する
    g_video_thread = VideoThread()  # g_video_thread に、プログラム全体で共有する値を設定する
    g_video_thread.start()  # この処理を実行する

def cleanup_thread():  # cleanup_thread メソッド／関数を定義する
    global g_video, g_video_thread  # この関数内で使用するグローバル変数を宣言する
    print(" -- stopping VideoThread...")  # メッセージをコンソールに出力する
    g_video_thread.stop()  # この処理を実行する
    g_video_thread.join()  # この処理を実行する

    del g_video  # この処理を実行する

def sig_handler(signum, frame) -> None:  # sig_handler メソッド／関数を定義する
    sys.exit(1)  # この処理を実行する

if __name__ == '__main__':  # このファイルが直接実行された場合のメイン処理を開始する
    parser = argparse.ArgumentParser()  # コマンドライン引数を解析するためのパーサを生成する
    parser.add_argument('course', choices=['right', 'left'], help='Course to run')  # 受け付けるコマンドライン引数を定義する
    parser.add_argument('--logfile', type=str, default=None, help='Path to log file')  # 受け付けるコマンドライン引数を定義する
    args = parser.parse_args()  # コマンドライン引数を解析して取得する

    if args.course == 'right':  # 条件を判定し、成立した場合の処理を行う
        g_course = -1  # g_course に、プログラム全体で共有する値を設定する
    else:  # 上記の条件に当てはまらない場合の処理を行う
        g_course = 1  # g_course に、プログラム全体で共有する値を設定する

    setup_thread()  # この処理を実行する

    #log_tree.level = log_tree.Level.DEBUG
    tree = build_behaviour_tree()  # tree に処理で使用する値を設定する
    #display_tree.render_dot_tree(tree)

    signal.signal(signal.SIGTERM, sig_handler)  # 終了シグナルを受け取ったときの処理を設定する

    try:  # 例外が発生する可能性のある処理を開始する
        etrobo = initialize_etrobo(backend='raspike_art')  # etrobo に処理で使用する値を設定する
        etrobo.add_handler(TraverseBehaviourTree(tree))  # ETロボの周期処理としてハンドラを登録する
        etrobo.dispatch(interval=EXEC_INTERVAL, logfile=args.logfile)  # 指定した周期でETロボの制御処理を開始する
    finally:  # 例外の有無にかかわらず最後に実行する処理を開始する
        signal.signal(signal.SIGTERM, signal.SIG_IGN)  # 終了シグナルを受け取ったときの処理を設定する
        signal.signal(signal.SIGINT, signal.SIG_IGN)  # 終了シグナルを受け取ったときの処理を設定する
        cleanup_thread()  # この処理を実行する
        signal.signal(signal.SIGTERM, signal.SIG_DFL)  # 終了シグナルを受け取ったときの処理を設定する
        signal.signal(signal.SIGINT, signal.SIG_DFL)  # 終了シグナルを受け取ったときの処理を設定する
        print(" -- exiting...")  # メッセージをコンソールに出力する


if __name__ == '__main__':  # このファイルが直接実行された場合のメイン処理を開始する
    ReadKey(name="readkey")