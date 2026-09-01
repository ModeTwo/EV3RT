import math

from etrobo_python import (
    Motor,
    TouchSensor,
    ColorSensor
)

from simple_pid import PID

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.trees import BehaviourTree
from py_trees.composites import Sequence
from py_trees.composites import Parallel
from py_trees.common import ParallelPolicy

from py_etrobo_util import (
    TraceSide,
    Color,
    ColorClassifier,
    LowPassFilter
)

# ==========================================
# 定数
# ==========================================

# Behaviour Treeの実行間隔
EXEC_INTERVAL: float = 0.02

# ライントレース時の目標V値
TRACELINE_TARGET_V = 75


# ==========================================
# グローバル変数
# ==========================================

# 走行距離取得用
g_plotter = None

# 左右モーター
g_right_motor: Motor = None
g_left_motor: Motor = None

# タッチセンサー
g_touch_sensor: TouchSensor = None

# カラーセンサー
g_color_sensor: ColorSensor = None

# コース方向
g_course: int = 0

class IsColorDetected(Behaviour):
    # 色を検出するためのクラス
    def __init__(self, name: str, color: Color):
        # 親クラス Behaviour の初期化処理を呼び出す
        super(IsColorDetected, self).__init__(name)
        # IsColorDetected が初期化されたことをデバッグログに出力する
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        # 検出したい色を保存する
        # 例：Color.RED、Color.BLUE など
        self.color = color
        # 直前に検出した色を保存する
        # 最初はまだ色を検出していないため UNKNOWN とする
        self.prevColor = Color.UNKNOWN
        # HSV値から色を判定するための ColorClassifier を生成する
        self.classifier = ColorClassifier()
        # 色検出処理を開始したかどうかを管理する
        self.running = False
        # 指定した色を検出したかどうかを管理する
        self.detected = False
    def update(self) -> Status:
        # 現在の走行距離を取得する
        # 主にログへ「どの位置で色を検出したか」を出力するために使用する
        cur_dist = g_plotter.get_distance()
        # 初回の update() 呼び出し時のみ実行する
        if not self.running:
            # 色検出処理を開始した状態にする
            self.running = True
            # 「色の検出を開始した」という情報をログに出力する
            self.logger.info(
                "%+06d %s.detection started for color=%s"
                % (cur_dist,
                    self.__class__.__name__,
                    self.color.value)
            )
        # カラーセンサーから現在の色をHSV形式で取得する
        # h：色相（Hue）
        # s：彩度（Saturation）
        # v：明度（Value）
        h, s, v = g_color_sensor.get_raw_color_hsv()

        # 取得したHSV値をログに出力
        self.logger.info(
            "%+06d %s.HSV=(H=%d, S=%d, V=%d)"
            % (cur_dist, self.__class__.__name__, h, s, v)
        )

        # 取得したHSV値から、現在の色を判定する
        detected_color = self.classifier.classify(h, s, v)
        # 判定した色が「検出したい色」と一致した場合
        if detected_color == self.color:
            # まだ検出済みになっていない場合
            if not self.detected:
                # 指定した色を検出済みにする
                self.detected = True
                # 「指定した色を検出した」という情報をログに出力する
                self.logger.info(
                    "%+06d %s.color=%s detected"
                    % (cur_dist,
                        self.__class__.__name__,
                        self.color.value)
                )
            # 指定した色を検出できたため SUCCESS を返す
            return Status.SUCCESS
        # 判定した色が「検出したい色」と一致しなかった場合
        else:
            # 今回検出した色が、前回検出した色と異なる場合
            if detected_color != self.prevColor:
                # UNKNOWN のログが大量に出ないようにする
                if detected_color != Color.UNKNOWN or self.prevColor != Color.UNKNOWN:
                    # 色が変化したことをログに出力する
                    self.logger.info(
                        "%+06d %s.color changed from %s to %s"
                        % (cur_dist,
                            self.__class__.__name__,
                            self.prevColor.value,
                            detected_color.value)
                    )
                    # 今回検出した色を「前回の色」として保存する
                    self.prevColor = detected_color
            # まだ目的の色を検出していないため RUNNING を返す
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

class TraceLine(Behaviour):
    # カラーセンサーの明度(V値)を使ってラインを追従するクラス
    # PID制御に加えて、
    # ・センサー値のノイズ除去
    # ・カーブ時の自動減速
    # ・速度に応じたPIDゲイン調整
    # ・ラインを見失った場合の復帰処理
    # を行う

    def __init__(
        self,
        name: str,
        target: int,
        power: int,
        pid_p: float,
        pid_i: float,
        pid_d: float,
        trace_side: TraceSide,

        # ローパスフィルタ関連の設定
        cutoff_hz: float = 12.0,
        median_window: int = 0,

        # 自動速度調整関連の設定
        power_min: int = None,          # 最低速度。Noneの場合は速度を固定する
        err_lo: float = 6.0,            # 誤差がこの値以下なら最高速度で走る
        err_hi: float = 22.0,           # 誤差がこの値以上なら最低速度で走る
        accel_per_s: float = 60.0,      # 1秒あたりの最大加速度
        decel_per_s: float = 180.0,     # 1秒あたりの最大減速度
        metric_hz: float = 2.0,

        # 速度に応じてPIDゲインを変更するための設定
        gains_slow: tuple = None,       # 低速時の(Kp, Kd)
        gains_fast: tuple = None,       # 高速時の(Kp, Kd)

        # ラインを見失った場合の復帰処理の設定
        recover_v: int = None,          # このV値以上を「ラインを見失った」と判断する
        recover_after: int = 3,         # 何回連続したらラインロストと判断するか
        recover_turn: int = None        # 復帰時の旋回量。Noneの場合は最大値を使用する
    ) -> None:

        # 親クラス Behaviour の初期化処理を呼び出す
        super(TraceLine, self).__init__(name)

        # ------------------------------------------------------------
        # 走行速度の設定
        # ------------------------------------------------------------

        # powerを通常走行時の最大速度として保存する
        self.power_max = power

        # power_minが指定されていない場合は、
        # 最低速度もpowerと同じにして速度固定で走行する
        self.power_min = power if power_min is None else power_min

        # 現在の走行速度
        # 初期値は最大速度powerとする
        self.power = power

        # power_minが指定されている場合は、自動速度調整を有効にする
        self.adapt = power_min is not None

        # ------------------------------------------------------------
        # PID制御の設定
        # ------------------------------------------------------------

        # ライントレース時に目標とするカラーセンサーのV値
        self.target = target

        # PID制御器を生成する
        # setpointには目標V値を設定する
        # PIDの出力は -power_max ～ +power_max の範囲に制限する
        self.pid = PID(
            pid_p,
            pid_i,
            pid_d,
            setpoint=target,
            sample_time=EXEC_INTERVAL,
            output_limits=(-self.power_max, self.power_max)
        )

        # ラインのどちら側をトレースするかを保存する
        # NORMAL / OPPOSITE によって旋回方向を切り替える
        self.trace_side = trace_side

        # ------------------------------------------------------------
        # カラーセンサー値のノイズ除去
        # ------------------------------------------------------------

        # cutoff_hzが指定されている場合は、
        # ローパスフィルタを使用してV値の細かな揺れを抑える
        # cutoff_hzがNoneまたは0の場合はフィルタを使用しない
        self.lpf = (
            LowPassFilter(
                cutoff_hz,
                EXEC_INTERVAL,
                median_window
            )
            if cutoff_hz else None
        )

        # ------------------------------------------------------------
        # カーブ判定・自動速度調整の設定
        # ------------------------------------------------------------

        # targetと実際のV値との差を使って、
        # ラインを安定して追えているか、カーブしているかを判断する
        self.err_lo = err_lo
        self.err_hi = err_hi

        # 誤差もそのままでは変動が大きいため、
        # ローパスフィルタで滑らかにする
        self.metric_lpf = LowPassFilter(
            metric_hz,
            EXEC_INTERVAL
        )

        # 平滑化した誤差を保存する変数
        self.err_metric = 0.0

        # ------------------------------------------------------------
        # 加速・減速量の設定
        # ------------------------------------------------------------

        # update()1回あたりに増加できる速度量
        # 加速は比較的ゆっくり行う
        self.accel_step = accel_per_s * EXEC_INTERVAL

        # update()1回あたりに減少できる速度量
        # カーブ突入時などは素早く減速する
        self.decel_step = decel_per_s * EXEC_INTERVAL

        # ------------------------------------------------------------
        # 速度に応じたPIDゲイン変更の設定
        # ------------------------------------------------------------

        # 低速時のKp、Kd
        self.gains_slow = gains_slow

        # 高速時のKp、Kd
        self.gains_fast = gains_fast

        # 低速用・高速用ゲインが両方設定されていて、
        # 最大速度と最低速度が異なる場合のみ
        # ゲインスケジューリングを有効にする
        self.schedule = (
            gains_slow is not None
            and gains_fast is not None
            and self.power_max > self.power_min
        )

        # ------------------------------------------------------------
        # ラインロスト復帰処理の設定
        # ------------------------------------------------------------

        # このV値以上になった場合、
        # 明るい床に出てラインを見失った可能性があると判断する
        self.recover_v = recover_v

        # 何回連続でラインロスト状態になったら復帰動作を行うか
        self.recover_after = recover_after

        # 復帰時の旋回量
        self.recover_turn = recover_turn

        # ラインロスト状態が何回連続したかを数える
        self._lost_count = 0

        # TraceLineが開始済みかどうかを管理する
        self.running = False


    def update(self) -> Status:
        # ------------------------------------------------------------
        # 初回実行時の処理
        # ------------------------------------------------------------

        # 初めてupdate()が呼ばれた場合のみ実行する
        if not self.running:

            # カラーセンサー用ローパスフィルタを初期化する
            if self.lpf:
                self.lpf.reset()

            # 誤差計算用のローパスフィルタを初期化する
            self.metric_lpf.reset()

            # ライントレース開始済みにする
            self.running = True

            # ライントレースを開始したことをログへ出力する
            self.logger.info(
                "%+06d %s.trace started with TS=%s"
                % (
                    g_plotter.get_distance(),
                    self.__class__.__name__,
                    self.trace_side.name
                )
            )

        # ------------------------------------------------------------
        # カラーセンサーの値を取得
        # ------------------------------------------------------------

        # カラーセンサーからHSV値を取得する
        # TraceLineでは主にV値（明度）を使用する
        h, s, v_raw = g_color_sensor.get_raw_color_hsv()

        # ローパスフィルタが有効な場合は、
        # 生のV値を滑らかにした値を使用する
        # 無効な場合は生のV値をそのまま使用する
        v = self.lpf(v_raw) if self.lpf else v_raw

        # ------------------------------------------------------------
        # カーブの強さを判定
        # ------------------------------------------------------------

        # 目標V値と現在の生V値との差を求める
        # 誤差が小さいほど安定してラインを追えている
        # 誤差が大きいほどカーブやラインからのズレが大きいと考える
        self.err_metric = self.metric_lpf(
            abs(self.target - v_raw)
        )

        # ------------------------------------------------------------
        # カーブに応じて走行速度を自動調整
        # ------------------------------------------------------------

        # power_minが指定されている場合のみ実行する
        if self.adapt:

            # 現在の誤差を0.0～1.0の値に変換する
            #
            # err_metric <= err_lo
            #   → frac = 0
            #   → 最大速度
            #
            # err_metric >= err_hi
            #   → frac = 1
            #   → 最低速度
            frac = (
                self.err_metric - self.err_lo
            ) / (
                self.err_hi - self.err_lo
            )

            # fracを0.0～1.0の範囲に制限する
            frac = (
                0.0 if frac < 0.0
                else (1.0 if frac > 1.0 else frac)
            )

            # 誤差に応じた目標速度を計算する
            # 誤差が小さい → power_max
            # 誤差が大きい → power_min
            target_power = (
                self.power_max
                - frac * (self.power_max - self.power_min)
            )

            # 現在速度と目標速度との差を求める
            dp = target_power - self.power

            # 急加速を防ぐ
            if dp > self.accel_step:
                dp = self.accel_step

            # カーブでは素早く減速する
            elif dp < -self.decel_step:
                dp = -self.decel_step

            # 現在速度を更新する
            self.power += dp

        # ------------------------------------------------------------
        # 現在速度に応じてPIDゲインを変更
        # ------------------------------------------------------------

        # 現在設定されているKp、Kdを取得する
        kp_now = self.pid.Kp
        kd_now = self.pid.Kd

        # ゲインスケジューリングが有効な場合
        if self.schedule:

            # 現在の速度を0.0～1.0に変換する
            #
            # power_min → f = 0
            # power_max → f = 1
            f = (
                self.power - self.power_min
            ) / (
                self.power_max - self.power_min
            )

            # fを0.0～1.0の範囲に制限する
            f = (
                0.0 if f < 0.0
                else (1.0 if f > 1.0 else f)
            )

            # 現在速度に合わせてKpを計算する
            kp_now = (
                self.gains_slow[0]
                + f * (
                    self.gains_fast[0]
                    - self.gains_slow[0]
                )
            )

            # 現在速度に合わせてKdを計算する
            kd_now = (
                self.gains_slow[1]
                + f * (
                    self.gains_fast[1]
                    - self.gains_slow[1]
                )
            )

            # PIDゲインを更新する
            # Kiについては初期設定値をそのまま使用する
            self.pid.tunings = (
                kp_now,
                self.pid.Ki,
                kd_now
            )

        # ------------------------------------------------------------
        # PID制御による旋回量の計算
        # ------------------------------------------------------------

        # フィルタ後のV値をPIDへ渡し、
        # ラインからのズレに応じた旋回量turnを計算する
        #
        # trace_sideによって符号を変えることで、
        # ラインのどちら側を走るかを切り替える
        if self.trace_side == TraceSide.NORMAL:

            turn = (
                (-1)
                * g_course
                * int(self.pid(v))
            )

        else:
            # TraceSide.OPPOSITEの場合
            turn = (
                g_course
                * int(self.pid(v))
            )

        # ------------------------------------------------------------
        # ラインを見失った場合の復帰処理
        # ------------------------------------------------------------

        # recover_vが指定されている場合のみ復帰処理を使用する
        if self.recover_v is not None:

            # V値がrecover_v以上の場合、
            # 明るい床へ出てラインを見失った可能性がある
            if v_raw >= self.recover_v:

                # ラインロスト回数を1増やす
                self._lost_count += 1

            else:
                # ラインを再び検出した場合はカウントをリセットする
                self._lost_count = 0

            # recover_after回以上連続でラインを見失い、
            # かつ旋回方向が決まっている場合
            if (
                self._lost_count >= self.recover_after
                and turn != 0
            ):

                # recover_turnが指定されていない場合は
                # 最大速度と同じ値を復帰時の旋回量として使用する
                mag = (
                    self.power_max
                    if self.recover_turn is None
                    else self.recover_turn
                )

                # PIDが判断した旋回方向は維持したまま、
                # 強い旋回量を設定してラインへ戻る
                turn = int(
                    math.copysign(mag, turn)
                )

        # ------------------------------------------------------------
        # 左右モーターの出力を計算
        # ------------------------------------------------------------

        # 現在の基本速度を整数へ変換する
        p = int(round(self.power))

        # 基本速度に旋回量を加減して左右モーターの速度を決める
        #
        # turnが正の場合
        #   左モーターを速く
        #   右モーターを遅く
        #
        # turnが負の場合
        #   左モーターを遅く
        #   右モーターを速く
        #
        # モーター出力は-100～100の範囲に制限する
        left = max(
            -100,
            min(100, p + turn)
        )

        right = max(
            -100,
            min(100, p - turn)
        )

        # ------------------------------------------------------------
        # モーターを実際に動かす
        # ------------------------------------------------------------

        # 右モーターへ計算した出力を設定する
        g_right_motor.set_power(right)

        # 左モーターへ計算した出力を設定する
        g_left_motor.set_power(left)

        # ------------------------------------------------------------
        # デバッグ用ログ
        # ------------------------------------------------------------

        # 必要に応じて以下のコメントアウトを外すことで、
        # センサー値や速度、PIDゲイン、旋回量などを確認できる
        #
        # self.logger.info(
        #     "%+06d %s.color sensor HSV=(%d, %d, %d) "
        #     "vf=%d, em=%d, pwr=%d, kp=%.3f, kd=%.3f, turn=%d"
        #     % (
        #         g_plotter.get_distance(),
        #         self.__class__.__name__,
        #         h,
        #         s,
        #         v_raw,
        #         int(v),
        #         int(self.err_metric),
        #         p,
        #         kp_now,
        #         kd_now,
        #         turn
        #     )
        # )

        # TraceLine自身は終了条件を持たないため、
        # ライントレース中は常にRUNNINGを返す
        #
        # Parallel内でIsColorDetectedなどと組み合わせる場合は、
        # 別のBehaviourがSUCCESSになるまで走行を続ける
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
        self.logger.info("%+06d %s.motors stopped" % (g_plotter.get_distance(), self.__class__.__name__))  # 動作状況を情報ログとして出力する
        return Status.SUCCESS  




    def build_behaviour_tree() -> BehaviourTree:

        # 全体を順番に実行するSequence
        root = Sequence(name="blue detection test", memory=True)

        # ライントレースと青色検知を同時に行う
        trace_until_blue = Parallel(
            name="trace until blue",
            policy=ParallelPolicy.SuccessOnOne()
        )

        # ライントレースしながら青色を探す
        trace_until_blue.add_children(
            [
                # ライントレース
                TraceLine(
                    name="sensor trace normal edge",
                    target=TRACELINE_TARGET_V,
                    power=33,
                    pid_p=0.65,
                    pid_i=0.000001,
                    pid_d=0.045,
                    trace_side=TraceSide.NORMAL
                ),

                # 青色を検知
                IsColorDetected(
                    name="check blue",
                    color=Color.BLUE
                ),
            ]
        )

        # 実行する順番を設定
        root.add_children(
            [
                # ① タッチセンサーが押されるまで待つ
                IsTouchOn(name="touch start"),

                # ② ライントレースしながら青色を探す
                trace_until_blue,

                # ③ 青色を検知したら停止
                StopNow(name="stop"),
            ]
        )

        return root