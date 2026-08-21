# 必要なモジュールのインポート
import sys  # システム機能
import argparse  # コマンドライン引数パーサー
import time  # 時間機能
import threading  # スレッド機能
import signal  # シグナルハンドリング
from enum import IntEnum, Enum, auto  # 列挙型定義
from etrobo_python import ETRobo, Hub, Motor, TouchSensor, ColorSensor, SonarSensor, GyroSensor  # EV3RT関連クラス
from simple_pid import PID  # PID制御クラス
from py_trees.trees import BehaviourTree  # ビヘイビアツリー
from py_trees.behaviour import Behaviour  # ビヘイビアベースクラス
from py_trees.common import Status  # ビヘイビアのステータス
from py_trees.composites import Sequence  # シーケンス合成ノード
from py_trees.composites import Parallel  # パラレル合成ノード
from py_trees.common import ParallelPolicy  # パラレルノードのポリシー
from py_trees import (
    display as display_tree,  # ツリー表示用
    logging as log_tree  # ツリーロギング用
)
from py_etrobo_util import Video, TraceSide, TargetInterested, Plotter, SymmetricClamper, Color, ColorClassifier  # ETRobo補助ユーティリティ

# 実行周期定義（秒）
EXEC_INTERVAL: float = 0.05  # メイン実行周期 30ミリ秒
VIDEO_INTERVAL: float = 0.02  # ビデオ処理周期 20ミリ秒
# モーター制御用PWM値定義
ARM_SHIFT_PWM = 30  # アームの移動時のPWM値
JUNCT_UPPER_THREAH = 50  # 交差点判定の上限閾値
JUNCT_LOWER_THREAH = 30  # 交差点判定の下限閾値
SPIN_MAX_POWER = 65  # スピン時の最大パワー
SPIN_MIN_POWER = 60  # スピン時の最小パワー
TRACELINE_TARGET_V = 65  # ラインをトレースする際の目標カラーセンサー値

# アーム移動方向の列挙型
class ArmDirection(IntEnum):
    UP = -1  # アームを上げる
    DOWN = 1  # アームを下げる

# 交差点状態の列挙型
class JState(Enum):
    INITIAL = auto()  # 初期状態
    JOINING = auto()  # ラインが結合中
    JOINED = auto()  # ラインが結合完了
    FORKING = auto()  # ラインが分岐中
    FORKED = auto()  # ラインが分岐完了

# 目標方向タイプの列挙型
class HeadingType(Enum):
    ABSOLUTE = "absolute"  # 絶対的な向き
    RELATIVE = "relative"  # 相対的な向き

# グローバル変数（ビヘイビア内で使用）
g_plotter: Plotter = None  # プロッターオブジェクト
g_hub: Hub = None  # ハブオブジェクト
g_arm_motor: Motor = None  # アームモーター
g_right_motor: Motor = None  # 右モーター
g_left_motor: Motor = None  # 左モーター
g_touch_sensor: TouchSensor = None  # タッチセンサー
g_color_sensor: ColorSensor = None  # カラーセンサー
g_sonar_sensor: SonarSensor = None  # ソナーセンサー
g_gyro_sensor: GyroSensor = None  # ジャイロセンサー
g_course: int = 0  # コース情報（右：-1、左：1）
g_key: str = None  # 暗号化キー
g_video: "Video" = None  # ビデオオブジェクト（グローバル）
g_video_thread: threading.Thread = None  # ビデオ処理用スレッド（グローバル）


# プログラム終了時のビヘイビア（ビヘイビアツリーが終了したときの処理）
class TheEnd(Behaviour):
    def __init__(self, name: str):
        # ビヘイビアの初期化
        super(TheEnd, self).__init__(name)
        # デバッグログ出力
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        # 実行フラグ
        self.running = False

    def update(self) -> Status:
        # 初回実行時の処理
        if not self.running:
            self.running = True
            # ログメッセージ出力
            self.logger.info("%+06d %s.behavior tree exhausted. ctrl+C shall terminate the program" % (g_plotter.get_distance(), self.__class__.__name__))
        # 常にRUNNING状態を返す（プログラム終了まで待機）
        return Status.RUNNING


# デバイスをリセットするビヘイビア
class ResetDevice(Behaviour):
    def __init__(self, name: str):
        super(ResetDevice, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        # IMUが静止になるまで待つためのカウンター
        self.count = 0

    def update(self) -> Status:
        # カウンターが0のときに初期リセット処理を実行
        if self.count == 0:
            # 各モーターのエンコーダーをリセット
            g_arm_motor.reset_count()
            g_right_motor.reset_count()
            g_left_motor.reset_count()
            # ジャイロセンサーをリセット
            g_gyro_sensor.reset()
            self.logger.info("%+06d %s.resetting..." % (g_plotter.get_distance(), self.__class__.__name__))
            self.logger.info("%+06d %s.waiting for IMU to be stationary..." % (g_plotter.get_distance(), self.__class__.__name__))
        # カウンターが3より大きいときにリセット完了
        elif self.count > 3:
            self.logger.info("%+06d %s.complete" % (g_plotter.get_distance(), self.__class__.__name__))
            # このビヘイビアは成功
            return Status.SUCCESS
        # ハブのIMUが静止状態になったかチェック
        if g_hub.hub_imu_is_stationary():
            # カウンターをインクリメント
            self.count += 1
        # このビヘイビアは処理中
        return Status.RUNNING


# アームを上下に動かすビヘイビア
class ArmUpDownFull(Behaviour):
    def __init__(self, name: str, direction: ArmDirection):
        super(ArmUpDownFull, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        # アーム移動方向（UP or DOWN）
        self.direction = direction
        # 実行フラグ
        self.running = False

    def update(self) -> Status:
        # 初回実行時の処理
        if not self.running:
            self.running = True
            # 現在のアームの位置を記録
            self.prev_degree = g_arm_motor.get_count()
            self.logger.info("%+06d %s.start position is %d" % (g_plotter.get_distance(), self.__class__.__name__, self.prev_degree))
            # 位置変化がないかをチェックするためのカウンター
            self.count = 0
            # アームをPWM値で回転開始
            g_arm_motor.set_power(ARM_SHIFT_PWM * self.direction)
        else:
            # 現在のアーム位置取得
            cur_degree = g_arm_motor.get_count()
            # 前回の位置との差が5未満（ほぼ動いていない）かチェック
            if abs(cur_degree - self.prev_degree) < 5:
                # 動いていない状態が10回以上続いたか
                if self.count > 10:
                    # モーター停止
                    g_arm_motor.set_power(0)
                    # ブレーキをON
                    g_arm_motor.set_brake(True)
                    self.logger.info("%+06d %s.position set to %d" % (g_plotter.get_distance(), self.__class__.__name__, cur_degree))
                    # このビヘイビアは成功
                    return Status.SUCCESS
                else:
                    # カウンターをインクリメント
                    self.count += 1
            # 現在の位置を更新
            self.prev_degree = cur_degree
        # このビヘイビアは処理中
        return Status.RUNNING


# キーを入力して暗号化キーを読み込むビヘイビア（コメント化されて使用されていない）
class ReadKey(Behaviour):
    def __init__(self, name: str):
        super(ReadKey, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        # 実行フラグ
        self.running = False

    def update(self) -> Status:
        # 初回実行時の処理
        if not self.running:
            self.running = True
            # 初回は待機状態を返す
            return Status.RUNNING
        else:
            # グローバル変数にキーを格納
            global g_key
            # ユーザーに入力を促す
            g_key = input("Enter the given key for decryption: ")
            # キーの長さをチェック（4文字のはず）
            if len(g_key) != 4:
                self.logger.warning("%+06d %s.invalid key length: %d. key should be 4 characters long." % (g_plotter.get_distance(), self.__class__.__name__, len(g_key)))
                # 無効なキーなので再入力を促す
                return Status.RUNNING
            else:
                # 入力されたキーを表示
                self.logger.info("%+06d %s.entered key: %s" % (g_plotter.get_distance(), self.__class__.__name__, g_key))
                # キーが正しいか確認を取る
                confirmation = input("Is the entered key correct? (y/n): ")
                # ユーザーが確認した場合
                if confirmation.lower() == 'y':
                    self.logger.info("%+06d %s.key confirmed" % (g_plotter.get_distance(), self.__class__.__name__))
                    # このビヘイビアは成功
                    return Status.SUCCESS
                else:
                    self.logger.info("%+06d %s.key rejected, please enter again" % (g_plotter.get_distance(), self.__class__.__name__))
                    # ユーザーが拒否したので再入力を促す
                    return Status.RUNNING


# 指定時間が経過したかをチェックするビヘイビア
class IsTimePassed(Behaviour):
    def __init__(self, name: str, delta_time: int):
        super(IsTimePassed, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        # 目標時間差分（秒）
        self.delta_time = delta_time
        # 実行フラグ
        self.running = False
        # 時間経過済みフラグ
        self.earned = False

    def update(self) -> Status:
        # 初回実行時の処理
        if not self.running:
            self.running = True
            # 開始時刻を記録
            self.orig_time = time.time()
            self.logger.info("%+06d %s.accumulation started for delta=%d" % (self.orig_time, self.__class__.__name__, self.delta_time))
        # 現在の時刻を取得
        cur_time = time.time()
        # 経過時間を計算
        earned_time = cur_time - self.orig_time
        # 目標時間に達したか
        if earned_time >= self.delta_time:
            # 最初に目標時間に達したときのみログ出力
            if not self.earned:
                self.earned = True
                self.logger.info("%+06d %s.delta time passed" % (g_plotter.get_distance(), self.__class__.__name__))
            # このビヘイビアは成功
            return Status.SUCCESS
        else:
            # このビヘイビアは処理中
            return Status.RUNNING


# 指定距離が移動したかをチェックするビヘイビア
class IsDistanceEarned(Behaviour):
    def __init__(self, name: str, delta_dist: int):
        super(IsDistanceEarned, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        # 目標距離差分（ミリメートル）
        self.delta_dist = delta_dist
        # 実行フラグ
        self.running = False
        # 距離移動済みフラグ
        self.earned = False

    def update(self) -> Status:
        # 初回実行時の処理
        if not self.running:
            self.running = True
            # 開始時の距離を記録
            self.orig_dist = g_plotter.get_distance()
            self.logger.info("%+06d %s.accumulation started for delta=%d" % (self.orig_dist, self.__class__.__name__, self.delta_dist))
        # 現在の移動距離を取得
        cur_dist = g_plotter.get_distance()
        # 移動した距離を計算
        earned_dist = cur_dist - self.orig_dist
        # 目標距離に達したか（前後の移動に対応）
        if (earned_dist >= self.delta_dist or -earned_dist <= -self.delta_dist):
            # 最初に目標距離に達したときのみログ出力
            if not self.earned:
                self.earned = True
                self.logger.info("%+06d %s.delta distance earned" % (cur_dist, self.__class__.__name__))
            # このビヘイビアは成功
            return Status.SUCCESS
        else:
            # このビヘイビアは処理中
            return Status.RUNNING


# 指定色が検出されたかをチェックするビヘイビア
class IsColorDetected(Behaviour):
    def __init__(self, name: str, color: Color):
        super(IsColorDetected, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        # 検出対象の色
        self.color = color
        # 前回の検出色
        self.prevColor = Color.UNKNOWN
        # 色分類器のインスタンス化
        self.classifier = ColorClassifier()
        # 実行フラグ
        self.running = False
        # 色検出済みフラグ
        self.detected = False

    def update(self) -> Status:
        # 現在の移動距離を取得
        cur_dist = g_plotter.get_distance()
        # 初回実行時の処理
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.detection started for color=%s" % (cur_dist, self.__class__.__name__, self.color.value))
        # カラーセンサーのHSV値を取得
        h, s, v = g_color_sensor.get_raw_color_hsv()

        # 色分類器を使用して色を分類
        detected_color = self.classifier.classify(h, s, v)
        # 検出色が目標色と一致したか
        if detected_color == self.color:
            # 最初に目標色を検出したときのみログ出力
            if not self.detected:
                self.detected = True
                self.logger.info("%+06d %s.color=%s detected" % (cur_dist, self.__class__.__name__, self.color.value))
            # このビヘイビアは成功
            return Status.SUCCESS
        else:
            # 検出色が前回の検出色と異なるか
            if detected_color != self.prevColor:
                # UNKNOWN色以外の変化をログ（ノイズを減らすため）
                if detected_color != Color.UNKNOWN or self.prevColor != Color.UNKNOWN:
                    self.logger.info("%+06d %s.color changed from %s to %s" % (cur_dist, self.__class__.__name__, self.prevColor.value, detected_color.value))
                    # 前回検出色を更新
                    self.prevColor = detected_color
            # このビヘイビアは処理中
            return Status.RUNNING


# QRコードがデコードされたかをチェックするビヘイビア
class IsQRDecoded(Behaviour):
    def __init__(self, name: str):
        super(IsQRDecoded, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        # 実行フラグ
        self.running = False
        # QRコード検出済みフラグ
        self.detected = False

    def update(self) -> Status:
        # 初回実行時の処理
        if not self.running:
            self.running = True
            # ビデオの目標をQRコードに設定
            g_video.set_target_interested(TargetInterested.QRCODE)
            self.logger.info("%+06d %s.detection started for QR code" % (g_plotter.get_distance(), self.__class__.__name__))
        # QRコードのテキストを取得
        text = g_video.get_QR_text()
        # QRコードが読み込まれたか
        if text != "":
            # 最初にQRコードを読み込んだときのみログ出力
            if not self.detected:
                self.detected = True
                self.logger.info("%+06d %s.QR code decoded: %s" % (g_plotter.get_distance(), self.__class__.__name__, text))
            # このビヘイビアは成功
            return Status.SUCCESS
        else:
            # このビヘイビアは処理中
            return Status.RUNNING


# ソナーセンサーで障害物が検出されたかをチェックするビヘイビア
class IsSonarOn(Behaviour):
    def __init__(self, name: str, alert_dist: int):
        super(IsSonarOn, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        # アラート距離（ミリメートル）
        self.alert_dist = alert_dist
        # 実行フラグ
        self.running = False

    def update(self) -> Status:
        # 初回実行時の処理
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.detection started for dist=%d" % (g_plotter.get_distance(), self.__class__.__name__, self.alert_dist))
        
        # ソナーセンサーからの距離を取得
        dist = g_sonar_sensor.get_distance()
        # アラート距離以内で、かつ有効な距離（> 0）か
        if (dist <= self.alert_dist and dist > 0):
            self.logger.info("%+06d %s.alerted at dist=%d" % (g_plotter.get_distance(), self.__class__.__name__, dist))
            # このビヘイビアは成功
            return Status.SUCCESS
        else:
            # このビヘイビアは処理中
            return Status.RUNNING


# タッチセンサーが押されたかをチェックするビヘイビア
class IsTouchOn(Behaviour):
    def __init__(self, name: str):
        super(IsTouchOn, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        # 実行フラグ
        self.running = False

    def update(self) -> Status:
        # 初回実行時の処理
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.waiting for touch..." % (g_plotter.get_distance(), self.__class__.__name__))
        # タッチセンサーが押されているか
        if g_touch_sensor.is_pressed():
            self.logger.info("%+06d %s.pressed" % (g_plotter.get_distance(), self.__class__.__name__))
            # このビヘイビアは成功
            return Status.SUCCESS
        else:
            # このビヘイビアは処理中
            return Status.RUNNING


# モーターを停止するビヘイビア
class StopNow(Behaviour):
    def __init__(self, name: str):
        super(StopNow, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def update(self) -> Status:
        # 右モーターの出力を0に設定
        g_right_motor.set_power(0)
        # 右モーターのブレーキをON
        g_right_motor.set_brake(True)
        # 左モーターの出力を0に設定
        g_left_motor.set_power(0)
        # 左モーターのブレーキをON
        g_left_motor.set_brake(True)
        self.logger.info("%+06d %s.motors stopped" % (g_plotter.get_distance(), self.__class__.__name__))
        # このビヘイビアは成功
        return Status.SUCCESS


# 指定されたPWMで直進するビヘイビア
class RunAsInstructed(Behaviour):
    def __init__(self, name: str, pwm_l: int, pwm_r: int) -> None:
        super(RunAsInstructed, self).__init__(name)
        # 左モーターのPWM（コース係数を掛ける）
        self.pwm_l = g_course * pwm_l
        # 右モーターのPWM（コース係数を掛ける）
        self.pwm_r = g_course * pwm_r
        # 実行フラグ
        self.running = False


    def update(self) -> Status:
        # 初回実行時の処理
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.started with pwm=(%s, %s)" % (g_plotter.get_distance(), self.__class__.__name__, self.pwm_l, self.pwm_r))
        # 右モーターに指定のPWMを設定
        g_right_motor.set_power(self.pwm_r)
        # 左モーターに指定のPWMを設定
        g_left_motor.set_power(self.pwm_l)
        # このビヘイビアは処理中
        return Status.RUNNING


# ラインに沿って直進しながらトレースするビヘイビア
class TraceLine(Behaviour):
    def __init__(self, name: str, target: int, power: int, pid_p: float, pid_i: float, pid_d: float,
                 trace_side: TraceSide) -> None:
        super(TraceLine, self).__init__(name)
        # 直進のベースパワー
        self.power = power
        # PID制御器を初期化
        self.pid = PID(pid_p, pid_i, pid_d, setpoint=target, sample_time=EXEC_INTERVAL, output_limits=(-power, power))
        # トレース側（NORMAL or OPPOSITE）
        self.trace_side = trace_side
        # 実行フラグ
        self.running = False

    def update(self) -> Status:
        # 初回実行時の処理
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.trace started with TS=%s" % (g_plotter.get_distance(), self.__class__.__name__, self.trace_side.name))
        # カラーセンサーのHSV値を取得
        h, s, v = g_color_sensor.get_raw_color_hsv()
        # トレース側に応じて旋回量を計算
        if self.trace_side == TraceSide.NORMAL:
            # 通常トレースの場合
            turn = (-1) * g_course * int(self.pid(v))
        else:  # TraceSide.OPPOSITE
            # 反対トレースの場合
            turn = g_course * int(self.pid(v))
        # 右モーターのパワー設定
        g_right_motor.set_power(self.power - turn)
        # 左モーターのパワー設定
        g_left_motor.set_power(self.power + turn)
        # このビヘイビアは処理中
        return Status.RUNNING


# ラインから離れた後、ラインを探して位置するビヘイビア
class SpinAndLocateLine(Behaviour):
    def __init__(self, name: str, target: int, max_power: int, min_power: int,
                 pid_p: float, pid_i: float, pid_d: float, trace_side: TraceSide) -> None:
        super(SpinAndLocateLine, self).__init__(name)
        # 目標のカラー値
        self.target = target
        # スピン方向（NORMAL: 1, OPPOSITE: -1）
        self.spin_direction = 1 if trace_side == TraceSide.NORMAL else -1
        # PID制御のP係数
        self.pid_p = pid_p
        # PID制御のI係数
        self.pid_i = pid_i
        # PID制御のD係数
        self.pid_d = pid_d
        # 出力を制限する対称クランパー
        self.clamper = SymmetricClamper(min_power, max_power)
        # ラインから離れるフェーズ
        self.move_away = True
        # 実行フラグ
        self.running = False

    def update(self) -> Status:
        # 最初にラインから離れるスピン、その後ラインを探すスピンを行う
        # ジャイロセンサーから現在の向きを取得
        current_heading = (-1) * g_course * g_gyro_sensor.get_angle()

        #if current_heading > 45 and current_heading < 135:

            #current_heading = 0
        # 初回実行時の処理
        if not self.running:
            # ラインから30度離れるための目標向きを設定
            self.target_heading = current_heading + self.spin_direction * 30
            # PID制御器を初期化
            self.pid = PID(self.pid_p, self.pid_i, self.pid_d, setpoint=self.target_heading, sample_time=EXEC_INTERVAL)
            self.running = True
            self.logger.info("%+06d %s.spin started at heading=%d for %d" % (g_plotter.get_distance(),
                                                                             self.__class__.__name__, current_heading, self.target_heading))
        # ラインから離れるフェーズ
        if self.move_away:
            # 目標向きとの誤差を計算
            error = float(self.target_heading) - current_heading
            # 誤差を[-180, 180]の範囲に正規化
            if error > 180.0:
                error -= 360.0
            if error < -180.0:
                error += 360.0
            # 誤差が2度以下か（ほぼ目標に達した）
            if abs(error) < 2.0:
                self.logger.info("%+06d %s.move away spin ended at heading=%d" % (g_plotter.get_distance(),
                                                                    self.__class__.__name__, current_heading))
                # ラインを探すフェーズのPID制御器を初期化
                self.pid = PID(self.pid_p, self.pid_i, self.pid_d, setpoint=self.target, sample_time=EXEC_INTERVAL)
                # スピン方向を反転
                self.spin_direction *= -1
                # ラインを探すフェーズに移行
                self.move_away = False
                # このビヘイビアは処理中
                return Status.RUNNING
            # PID制御の出力をクランプして出力を得る
            power = int(self.clamper.clamp(self.pid(current_heading)))
        else:
            # ラインを探すフェーズ（反対方向にスピン）
            # カラーセンサーのHSV値を取得
            h, s, v = g_color_sensor.get_raw_color_hsv()
            # 目標値との誤差を計算
            error = float(self.target) - v
            # 誤差が5以下か（ラインが見つかった）
            if abs(error) < 5.0:
                self.logger.info("%+06d %s.line located at heading=%d" % (g_plotter.get_distance(),
                                                                    self.__class__.__name__, current_heading))
                # このビヘイビアは成功
                return Status.SUCCESS
            # PID制御の出力をクランプして出力を得る
            power = int(self.clamper.clamp(self.pid(v))) * self.spin_direction * (-1)
        # 右モーターに出力を設定
        g_right_motor.set_power(g_course * power)
        # 左モーターに出力を設定
        g_left_motor.set_power((-1) * g_course * power)
        # このビヘイビアは処理中
        return Status.RUNNING


# ロボットを指定角度回転させるビヘイビア
class SpinAround(Behaviour):
    def __init__(self, name: str, target: int, max_power: int, min_power: int,
                 pid_p: float, pid_i: float, pid_d: float, target_type: HeadingType) -> None:
        super(SpinAround, self).__init__(name)
        # 目標の向き（度）
        self.target = target
        # 目標タイプ（ABSOLUTE or RELATIVE）
        self.target_type = target_type
        # PID制御のP係数
        self.pid_p = pid_p
        # PID制御のI係数
        self.pid_i = pid_i
        # PID制御のD係数
        self.pid_d = pid_d
        # 出力を制限する対称クランパー
        self.clamper = SymmetricClamper(min_power, max_power)
        # 実行フラグ
        self.running = False

    def update(self) -> Status:
        # ジャイロセンサーから現在の向きを取得
        current_heading = (-1) * g_course * g_gyro_sensor.get_angle()
        # 初回実行時の処理
        if not self.running:
            # 目標タイプに応じて目標向きを決定
            if self.target_type == HeadingType.RELATIVE:
                # 相対的な向き
                self.target_heading = current_heading + self.target
            else:
                # 絶対的な向き
                self.target_heading = self.target
            # PID制御器を初期化
            self.pid = PID(self.pid_p, self.pid_i, self.pid_d, setpoint=self.target_heading, sample_time=EXEC_INTERVAL)
            self.running = True
            self.logger.info("%+06d %s.spin started at heading=%d for %d" % (g_plotter.get_distance(),
                                                                             self.__class__.__name__, current_heading, self.target_heading))
        # 目標向きとの誤差を計算
        error = float(self.target_heading) - current_heading
        # 誤差を[-180, 180]の範囲に正規化
        if error > 180.0:
            error -= 360.0
        if error < -180.0:
            error += 360.0
        # 誤差が2度以下か（ほぼ目標に達した）
        if abs(error) < 2.0:
            self.logger.info("%+06d %s.spin ended at heading=%d" % (g_plotter.get_distance(),
                                                                    self.__class__.__name__, current_heading))
            #g_gyro_sensor.reset()
            # このビヘイビアは成功
            return Status.SUCCESS
        # PID制御の出力をクランプして出力を得る
        power = int(self.clamper.clamp(self.pid(current_heading)))

        #print(
        #"### SPIN:",
        #"gyro=", current_heading,
        #"target=", self.target_heading,
        #"power=", power
        #)


        # 右モーターに出力を設定
        g_right_motor.set_power(g_course * power)
        # 左モーターに出力を設定
        g_left_motor.set_power((-1) * g_course * power)
        # このビヘイビアは処理中
        return Status.RUNNING


# ジャイロセンサーを使用して指定向きに直進するビヘイビア
class RunByGyro(Behaviour):
    def __init__(self, name: str, target: int, power: int,
                 pid_p: float, pid_i: float, pid_d: float, target_type: HeadingType) -> None:
        super(RunByGyro, self).__init__(name)
        # 目標の向き（度）
        self.target = target
        # 目標タイプ（ABSOLUTE or RELATIVE）
        self.target_type = target_type
        # 直進のベースパワー
        self.power = power
        # PID制御のP係数
        self.pid_p = pid_p
        # PID制御のI係数
        self.pid_i = pid_i
        # PID制御のD係数
        self.pid_d = pid_d
        # ログ出力用のカウンター
        self.count = None
        # 実行フラグ
        self.running = False

    def update(self) -> Status:
        # ジャイロセンサーから現在の向きを取得
        current_heading = (-1) * g_course * g_gyro_sensor.get_angle()
        # 1秒ごとにログを出力
        if not self.count == None and self.count % (1000.0 / EXEC_INTERVAL) == 0:
            self.logger.info("%+06d %s.current heading=%d" % (g_plotter.get_distance(), self.__class__.__name__, current_heading))
            # カウンターをリセット
            self.count = 0
        # 初回実行時の処理
        if not self.running:
            # 目標タイプに応じて目標向きを決定
            if self.target_type == HeadingType.RELATIVE:
                # 相対的な向き
                self.target_heading = current_heading + self.target
                #self.target_heading = self.target
            else:
                # 絶対的な向き
                self.target_heading = self.target
            # PID制御器を初期化
            self.pid = PID(self.pid_p, self.pid_i, self.pid_d, setpoint=self.target_heading, sample_time=EXEC_INTERVAL, output_limits=(-self.power, self.power))
            # ログ用カウンターを初期化
            self.count = 0
            self.logger.info("%+06d %s.gyro run started toward heading=%d" % (g_plotter.get_distance(),
                                                                              self.__class__.__name__, self.target_heading))
            self.running = True
        # PID制御から旋回量を取得
        turn = int(self.pid(current_heading))
        print(
            "### RUN:",
            "gyro=", current_heading,
            "target=", self.target_heading,
            "turn=", turn,
            "right=", self.power - g_course * turn,
            "left=", self.power + g_course * turn
        )
        # 右モーターのパワー設定
        g_right_motor.set_power(self.power - g_course * turn)
        # 左モーターのパワー設定
        g_left_motor.set_power(self.power + g_course * turn)
        # カウンターをインクリメント
        self.count += 1
        # このビヘイビアは処理中
        return Status.RUNNING


# カメラを使用してラインをトレースするビヘイビア
class TraceLineCam(Behaviour):
    def __init__(self, name: str, power: int, pid_p: float, pid_i: float, pid_d: float,
                 gs_min: int, gs_max: int, trace_side: TraceSide) -> None:
        super(TraceLineCam, self).__init__(name)
        # 直進のベースパワー
        self.power = power
        # PID制御器を初期化
        self.pid = PID(pid_p, pid_i, pid_d, setpoint=0, sample_time=EXEC_INTERVAL, output_limits=(-power, power))
        # グレースケール最小値
        self.gs_min = gs_min
        # グレースケール最大値
        self.gs_max = gs_max
        # トレース側（NORMAL, OPPOSITE, CENTER）
        self.trace_side = trace_side
        # 実行フラグ
        self.running = False

    def update(self) -> Status:
        # 初回実行時の処理
        if not self.running:
            self.running = True
            # ビデオの閾値を設定
            g_video.set_thresholds(self.gs_min, self.gs_max)
            # ビデオの目標をラインに設定
            g_video.set_target_interested(TargetInterested.LINE)
            # トレース側に応じてビデオのトレース側を設定
            if self.trace_side == TraceSide.NORMAL:
                if g_course == -1:  # 右コース
                    g_video.set_trace_side(TraceSide.RIGHT)
                else:
                    g_video.set_trace_side(TraceSide.LEFT)
            elif self.trace_side == TraceSide.OPPOSITE:
                if g_course == -1:  # 右コース
                    g_video.set_trace_side(TraceSide.LEFT)
                else:
                    g_video.set_trace_side(TraceSide.RIGHT)
            else:  # TraceSide.CENTER
                g_video.set_trace_side(TraceSide.CENTER)
            self.logger.info("%+06d %s.trace started with TS=%s" % (g_plotter.get_distance(), self.__class__.__name__, self.trace_side.name))
        # ビデオから角度（theta）を取得して旋回量を計算
        turn = (-1) * int(self.pid(g_video.get_theta()))
        # 右モーターのパワー設定
        g_right_motor.set_power(self.power - turn)
        # 左モーターのパワー設定
        g_left_motor.set_power(self.power + turn)
        # このビヘイビアは処理中
        return Status.RUNNING


# ラインの交差点を判定するビヘイビア
class IsJunction(Behaviour):
    def __init__(self, name: str, target_state: JState) -> None:
        super(IsJunction, self).__init__(name)
        # 目標の交差点状態
        self.target_state = target_state
        # 目標状態に到達したかのフラグ
        self.reached = False
        # 前回の辺の範囲（roe）
        self.prev_roe = 0
        # 現在の交差点状態
        self.state: JState = JState.INITIAL
        # 実行フラグ
        self.running = False

    def update(self) -> Status:
        # 初回実行時の処理
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.scan started" % (g_plotter.get_distance(), self.__class__.__name__))
        # ビデオから辺の範囲を取得
        roe = g_video.get_range_of_edges()
        # roe が0でない場合
        if roe != 0:
            # 初期状態の場合
            if self.state == JState.INITIAL:
                # ラインが結合しようとしているか
                if (self.target_state == JState.JOINING or self.target_state == JState.JOINED) and roe >= JUNCT_UPPER_THRESH and self.prev_roe <= JUNCT_LOWER_THRESH:
                    self.logger.info("%+06d %s.lines are joining" % (g_plotter.get_distance(), self.__class__.__name__))
                    # 状態を結合中に変更
                    self.state = JState.JOINING
                # ラインが分岐しようとしているか
                elif (self.target_state == JState.FORKING or self.target_state == JState.FORKED) and roe >= JUNCT_LOWER_THRESH and self.prev_roe <= JUNCT_LOWER_THRESH:
                    self.logger.info("%+06d %s.lines are forking" % (g_plotter.get_distance(), self.__class__.__name__))
                    # 状態を分岐中に変更
                    self.state = JState.FORKING
            # 結合中の場合
            elif self.state == JState.JOINING:
                # 結合が完了したか
                if roe <= JUNCT_LOWER_THRESH:
                    self.logger.info("%+06d %s.the join completed" % (g_plotter.get_distance(), self.__class__.__name__))
                    # 状態を結合完了に変更
                    self.state = JState.JOINED

            # 分岐中の場合
            elif self.state == JState.FORKING:
                # 分岐が完了したか
                if roe <= JUNCT_LOWER_THRESH and self.prev_roe >= JUNCT_UPPER_THRESH:
                    self.logger.info("%+06d %s.the fork completed" % (g_plotter.get_distance(), self.__class__.__name__))
                    # 状態を分岐完了に変更
                    self.state = JState.FORKED
            else:
                # その他の状態
                pass
        # 前回のroeを更新
        self.prev_roe = roe

        # 目標状態に到達したか
        if not self.reached and self.state == self.target_state:
            self.reached = True
            self.logger.info("%+06d %s.target state reached" % (g_plotter.get_distance(), self.__class__.__name__))
            # このビヘイビアは成功
            return Status.SUCCESS
        else:
            # このビヘイビアは処理中
            return Status.RUNNING


# ビヘイビアツリーを走査・実行するクラス
class TraverseBehaviourTree(object):
    def __init__(self, tree: BehaviourTree) -> None:
        # ビヘイビアツリーのインスタンス
        self.tree = tree
        # 実行フラグ
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
        # グローバル変数をこのメソッドで使用することを宣言
        global g_hub, g_arm_motor, g_right_motor, g_left_motor, g_touch_sensor, g_color_sensor, g_sonar_sensor, g_gyro_sensor, g_plotter
        # 初回実行時の処理
        if not self.running:
            # グローバル変数にハードウェアオブジェクトを割り当て
            g_hub = hub
            g_arm_motor = arm_motor
            g_right_motor = right_motor
            g_left_motor = left_motor
            g_touch_sensor = touch_sensor
            g_color_sensor = color_sensor
            g_sonar_sensor = sonar_sensor
            g_gyro_sensor = gyro_sensor
            # プロッターを初期化
            g_plotter = Plotter()
            print(" -- TraverseBehaviorTree initialization complete")
            self.running = True
        else:
            # ビヘイビアツリーを1ステップ実行
            self.tree.tick()
            #self.tree.tick()
            # プロッターに現在の状態を記録
            g_plotter.plot(hub, arm_motor, right_motor, left_motor, touch_sensor, color_sensor, sonar_sensor, gyro_sensor)


# ビデオ処理をスレッドで実行するクラス
class VideoThread(threading.Thread):
    def __init__(self):
        # スレッドの初期化
        super().__init__()
        # スレッド停止イベント
        self._stop_event = threading.Event()
        # 前回の実行時刻
        self.prev_time = time.time()

    def stop(self):
        # スレッド停止イベントを設定
        self._stop_event.set()

    def run(self):
        # スレッド停止イベントが設定されるまで実行
        while not self._stop_event.is_set():
            # ビデオ処理を実行
            g_video.process(g_plotter, g_hub, g_arm_motor, g_right_motor, g_left_motor, g_color_sensor, g_sonar_sensor, g_gyro_sensor)
            # 現在の時刻を取得
            current_time = time.time()
            # 経過時間を計算
            elapsed_time = current_time - self.prev_time
            # 前回の実行時刻を更新
            self.prev_time = current_time
            # ビデオ処理周期に調整
            if elapsed_time < VIDEO_INTERVAL:
                time.sleep(VIDEO_INTERVAL - elapsed_time)


# ビヘイビアツリーを構築する関数
def build_behaviour_tree() -> BehaviourTree:
    # ルートノード（シーケンス：全て順序通り実行）
    root = Sequence(name="2026 base", memory=True)
    # キャリブレーションシーケンス
    calibration = Sequence(name="calibration", memory=True)
    # スタート並列ノード
    start = Parallel(name="start", policy=ParallelPolicy.SuccessOnOne())
    # ラップ2並列ノード
    lap2 = Parallel(name="lap2", policy=ParallelPolicy.SuccessOnOne())
    # ラップ3並列ノード
    lap3 = Parallel(name="lap3", policy=ParallelPolicy.SuccessOnOne())
    # キャリー1並列ノード
    carry1 = Parallel(name="carry1", policy=ParallelPolicy.SuccessOnOne())
    # キャリー2並列ノード
    carry2 = Parallel(name="carry2", policy=ParallelPolicy.SuccessOnOne())
    # キャリー3並列ノード
    carry3 = Parallel(name="carry3", policy=ParallelPolicy.SuccessOnOne())
    # QR1並列ノード
    qr1 = Parallel(name="qr1", policy=ParallelPolicy.SuccessOnOne())
    # QR2並列ノード
    qr2 = Parallel(name="qr2", policy=ParallelPolicy.SuccessOnOne())
    # QR3並列ノード
    qr3 = Parallel(name="qr3", policy=ParallelPolicy.SuccessOnOne())
    # QR4並列ノード
    qr4 = Parallel(name="qr4", policy=ParallelPolicy.SuccessOnOne())
    # QR読み込み並列ノード
    qr_read = Parallel(name="qr_read", policy=ParallelPolicy.SuccessOnOne())
    # QRスキャン揺れシーケンス
    qr_scan_shake = Sequence(name="qr_scan_shake", memory=True)
    # QRスキャン移動戻り並列ノード
    qr_scan_move_back = Parallel(name="qr_scan_move_back2", policy=ParallelPolicy.SuccessOnOne())
    
    # キャリブレーション処理の設定
    calibration.add_children(
        [
            ArmUpDownFull(name="arm up", direction=ArmDirection.UP),  # アームを上げる
            ArmUpDownFull(name="arm down", direction=ArmDirection.DOWN),  # アームを下げる
            ResetDevice(name="device reset"),  # デバイスをリセット
            #ReadKey(name="read key"),  # キーの読み込み（コメント化）
        ]
    )
    # スタート処理の設定
    start.add_children(
        [
            IsTouchOn(name="touch start"),  # タッチセンサーの入力待機
        ]
    )
    # ラップ2処理の設定
    lap2.add_children(
        [
            TraceLine(name="sensor trace normal edge", target=TRACELINE_TARGET_V, power=33,
                pid_p=0.55, pid_i=0.0000009, pid_d=0.015, trace_side=TraceSide.NORMAL),  # ラインをトレース
            IsColorDetected(name="check color", color=Color.BLUE),  # 青色検出
        ]
    )
    # ラップ3処理の設定
    lap3.add_children(
        [
            RunByGyro(name="run straight to catch the bottle", target=5, power=33,
                pid_p=1.1, pid_i=0.00075, pid_d=0.04, target_type=HeadingType.ABSOLUTE),  # ボトルをキャッチするために直進
            IsDistanceEarned(name="check distance", delta_dist = 370),  # 距離チェック
        ]
    )
    # キャリー1処理の設定
    carry1.add_children(
        [
            TraceLine(name="sensor trace normal edge", target=TRACELINE_TARGET_V, power=33,
                pid_p=0.55, pid_i=0.0000009, pid_d=0.015, trace_side=TraceSide.NORMAL),  # ラインをトレース
            IsColorDetected(name="check color", color=Color.BLUE),  # 青色検出
        ]
    )
    # キャリー2処理の設定
    carry2.add_children(
        [
            RunByGyro(name="run straight to pass the blue line", target=90, power=33,
                pid_p=1.1, pid_i=0.00075, pid_d=0.04, target_type=HeadingType.ABSOLUTE),  # 青ラインを通過
            IsDistanceEarned(name="check distance", delta_dist = 120),  # 距離チェック
        ]
    )
    # キャリー3処理の設定
    carry3.add_children(
        [
            TraceLine(name="sensor trace normal edge", target=TRACELINE_TARGET_V+10, power=33,
                pid_p=0.65, pid_i=0.000001, pid_d=0.011, trace_side=TraceSide.NORMAL),  # ラインをトレース
            IsColorDetected(name="check color", color=Color.BLUE),  # 青色検出
        ]
    )
    # QR1処理の設定
    qr1.add_children(
        [
            RunByGyro(name="run straight to align with opposite edge", target=5, power=33,
                pid_p=1.1, pid_i=0.00075, pid_d=0.04, target_type=HeadingType.ABSOLUTE),  # 反対端に揃える
            IsDistanceEarned(name="check distance", delta_dist = 50),  # 距離チェック
        ]
    )
    # QR2処理の設定
    qr2.add_children(
        [
            RunByGyro(name="run straight to correct heading", target=0, power=33,
                pid_p=1.1, pid_i=0.00075, pid_d=0.04, target_type=HeadingType.ABSOLUTE),  # 向きを正す
            IsDistanceEarned(name="check distance", delta_dist = 50),  # 距離チェック
        ]
    )
    # QR3処理の設定
    qr3.add_children(
        [
            TraceLine(name="sensor trace opposite edge", target=TRACELINE_TARGET_V+10, power=33,
                pid_p=0.655, pid_i=0.0000011, pid_d=0.012, trace_side=TraceSide.OPPOSITE),  # 反対側ラインをトレース
            IsColorDetected(name="check color", color=Color.BLUE),  # 青色検出
        ]
    )
    # QR4処理の設定
    qr4.add_children(
        [
            RunByGyro(name="run straight to pass half the blue line", target=-90, power=33,
                pid_p=1.1, pid_i=0.00075, pid_d=0.04, target_type=HeadingType.ABSOLUTE),  # 青ラインの半分を通過
            IsDistanceEarned(name="check distance", delta_dist = 100),  # 距離チェック
        ]
    )
    # QRスキャン移動戻り処理の設定
    qr_scan_move_back.add_children(
        [
            RunAsInstructed(name="move back a little", pwm_l=-SPIN_MIN_POWER, pwm_r=-SPIN_MIN_POWER),  # 後ろに少し動く
            IsDistanceEarned(name="check distance", delta_dist = 50),  # 距離チェック
        ]
    )
    # QRスキャン揺れ処理の設定
    qr_scan_shake.add_children(
        [
            SpinAround(name="scan for QR code", target=4, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                pid_p=0.2, pid_i=0.00075, pid_d=0.03, target_type=HeadingType.RELATIVE),  # QRコードスキャン
            StopNow(name="stop"),  # 停止
            IsTimePassed(name="wait for a moment", delta_time=0.8),  # 0.8秒待機
            SpinAround(name="scan for QR code", target=-8, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                pid_p=0.2, pid_i=0.00075, pid_d=0.03, target_type=HeadingType.RELATIVE),  # QRコードスキャン
            StopNow(name="stop"),  # 停止
            IsTimePassed(name="wait for a moment", delta_time=0.8),  # 0.8秒待機
            SpinAround(name="scan for QR code", target=4, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                pid_p=0.2, pid_i=0.00075, pid_d=0.03, target_type=HeadingType.RELATIVE),  # QRコードスキャン
            StopNow(name="stop"),  # 停止
            IsTimePassed(name="wait for a moment", delta_time=2.0),  # 2秒待機
            qr_scan_move_back,  # 後ろに移動
            SpinAround(name="scan for QR code", target=3, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                pid_p=0.2, pid_i=0.00075, pid_d=0.03, target_type=HeadingType.RELATIVE),  # QRコードスキャン
            StopNow(name="stop"),  # 停止
            IsTimePassed(name="wait for a moment", delta_time=0.8),  # 0.8秒待機
            SpinAround(name="scan for QR code", target=-6, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                pid_p=0.2, pid_i=0.00075, pid_d=0.03, target_type=HeadingType.RELATIVE),  # QRコードスキャン
            StopNow(name="stop"),  # 停止
            IsTimePassed(name="wait for a moment", delta_time=0.8),  # 0.8秒待機
            SpinAround(name="scan for QR code", target=3, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                pid_p=0.2, pid_i=0.00075, pid_d=0.03, target_type=HeadingType.RELATIVE),  # QRコードスキャン
            StopNow(name="stop"),  # 停止
            IsTimePassed(name="wait for a moment", delta_time=2.0),  # 2秒待機
        ]
    )
    # QR読み込み処理の設定
    qr_read.add_children(
        [
            IsQRDecoded(name="check QR code"),  # QRコード読み込み
            qr_scan_shake,  # スキャン揺れ処理
        ]
    )
    
    # ルートノード（全体の流れ）の設定
    root.add_children(
        [
            calibration,  # キャリブレーション
            start,  # スタート
            lap2,  # ラップ2
            lap3,  # ラップ3
            carry1,  # キャリー1
            carry2,  # キャリー2
            carry3,  # キャリー3
            SpinAround(name="about the face", target=10, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                pid_p=0.2, pid_i=0.00075, pid_d=0.03, target_type=HeadingType.ABSOLUTE),  # 方向転換
            StopNow(name="stop"),  # 停止
            IsTimePassed(name="wait for a moment", delta_time=1.0),  # 1秒待機
            #qr1,  # QR1（コメント化）
            qr2,  # QR2
            StopNow(name="stop"),  # 停止
            IsTimePassed(name="wait for a moment", delta_time=1.0),  # 1秒待機
            SpinAndLocateLine(name="spin and locate line", target=TRACELINE_TARGET_V-20, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                pid_p=0.2, pid_i=0.00075, pid_d=0.03, trace_side=TraceSide.OPPOSITE),  # スピンしてラインを探す
            StopNow(name="stop"),  # 停止
            IsTimePassed(name="wait for a moment", delta_time=1.0),  # 1秒待機
            qr3,  # QR3
            qr4,  # QR4
            StopNow(name="stop"),  # 停止
            IsTimePassed(name="wait for a moment", delta_time=1.0),  # 1秒待機
            ArmUpDownFull(name="arm up", direction=ArmDirection.UP),  # アームを上げる
            SpinAround(name="align for QR code scanning", target=0, max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                pid_p=0.2, pid_i=0.00075, pid_d=0.03, target_type=HeadingType.ABSOLUTE),  # QRコードスキャンのために揃える
            qr_read,  # QR読み込み
            ArmUpDownFull(name="arm down", direction=ArmDirection.DOWN),  # アームを下げる
            StopNow(name="stop"),  # 停止
            TheEnd(name="end"),  # 終了
        ]
    )
    # ツリーを返す
    return root


# ETRoboを初期化する関数
def initialize_etrobo(backend: str) -> ETRobo:
    # ETRoboインスタンスの作成とハードウェアの追加
    return (ETRobo(backend=backend)
            .add_hub('hub')  # ハブの追加
            .add_device('arm_motor', device_type=Motor, port='C')  # アームモーター（ポートC）
            .add_device('right_motor', device_type=Motor, port='A')  # 右モーター（ポートA）
            .add_device('left_motor', device_type=Motor, port='B')  # 左モーター（ポートB）
            .add_device('touch_sensor', device_type=TouchSensor, port='D')  # タッチセンサー（ポートD）
            .add_device('color_sensor', device_type=ColorSensor, port='E')  # カラーセンサー（ポートE）
            .add_device('sonar_sensor', device_type=SonarSensor, port='F')  # ソナーセンサー（ポートF）
            .add_device('gyro_sensor', device_type=GyroSensor, port='')  # ジャイロセンサー
    )


# ビデオスレッドをセットアップする関数
def setup_thread():
    global g_video, g_video_thread
    # ビデオオブジェクトを作成
    g_video = Video()

    print(" -- starting VideoThread...")
    # ビデオスレッドを作成
    g_video_thread = VideoThread()
    # スレッドを開始
    g_video_thread.start()


# ビデオスレッドをクリーンアップする関数
def cleanup_thread():
    global g_video, g_video_thread
    print(" -- stopping VideoThread...")
    # スレッド停止イベントを設定
    g_video_thread.stop()
    # スレッドの終了を待つ
    g_video_thread.join()

    # ビデオオブジェクトを削除
    del g_video


# シグナルハンドラ関数
def sig_handler(signum, frame) -> None:
    # プログラムを終了
    sys.exit(1)


# メイン処理
if __name__ == '__main__':
    # コマンドライン引数パーサーを作成
    parser = argparse.ArgumentParser()
    # コース引数の追加（必須）
    parser.add_argument('course', choices=['right', 'left'], help='Course to run')
    # ログファイル引数の追加（オプション）
    parser.add_argument('--logfile', type=str, default=None, help='Path to log file')
    # 引数をパース
    args = parser.parse_args()

    # コース設定に基づいてg_courseを設定
    if args.course == 'right':
        g_course = -1  # 右コース
    else:
        g_course = 1  # 左コース

    # ビデオスレッドをセットアップ
    setup_thread()

    # ビヘイビアツリーをビルド
    #py_trees.logging.level = py_trees.logging.Level.DEBUG  # デバッグログを有効化（コメント化）
    tree = build_behaviour_tree()
    #display_tree.render_dot_tree(tree)  # ツリーを表示（コメント化）

    # SIGTERMシグナルのハンドラを設定
    signal.signal(signal.SIGTERM, sig_handler)

    # メイン処理
    try:
        # ETRoboを初期化
        etrobo = initialize_etrobo(backend='raspike_art')
        # ビヘイビアツリーハンドラを追加
        etrobo.add_handler(TraverseBehaviourTree(tree))
        # ETRoboをディスパッチ（実行開始）
        etrobo.dispatch(interval=EXEC_INTERVAL, logfile=args.logfile)
    finally:
        # シグナルハンドラを無視に設定
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        # ビデオスレッドをクリーンアップ
        cleanup_thread()
        # シグナルハンドラをデフォルトに設定
        signal.signal(signal.SIGTERM, signal.SIG_DFL)
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        print(" -- exiting...")
