import sys
import time
import csv
import statistics
from datetime import datetime

from etrobo_python import ETRobo, ColorSensor, TouchSensor, Motor
from py_trees.common import Status

from robot_program.behaviours.line_trace import TraceLine
from robot_program.runtime import runtime
from py_etrobo_util import TraceSide


# ============================================================
# 設定
# ============================================================

EXEC_INTERVAL = 0.02       # 制御周期 [秒]
SAMPLE_SECONDS = 10.0      # 1条件あたりの測定時間 [秒]

COLOR_SENSOR_PORT = 'E'
TOUCH_SENSOR_PORT = 'D'

# ============================================================
# ライントレース設定
#
# ★普段使用しているTraceLineの設定値に合わせてください
# ============================================================

TRACE_TARGET = 75
TRACE_POWER = 33

PID_P = 0.65
PID_I = 0.000001
PID_D = 0.045

# 通常のライントレース
TRACE_SIDE = TraceSide.NORMAL

# ============================================================
# モーターポート
# ============================================================

RIGHT_MOTOR_PORT = 'A'
LEFT_MOTOR_PORT = 'B'

# ============================================================
# コース設定
#
# 実行例:
#   pypy3 color_calibration.py right
#   pypy3 color_calibration.py left
# ============================================================

if len(sys.argv) < 2:

    print("Usage:")
    print("  pypy3 color_calibration.py right")
    print("  pypy3 color_calibration.py left")

    sys.exit(1)


COURSE_NAME = sys.argv[1].lower()


if COURSE_NAME == "right":

    # ★既存プログラムの設定に応じて
    #   1 / -1 を変更してください
    COURSE = -1


elif COURSE_NAME == "left":

    COURSE = 1


else:

    print("right または left を指定してください。")

    sys.exit(1)


# ============================================================
# 測定条件
#
# mode:
#   "static" = 停止状態で測定
#   "trace"  = ライントレースしながら測定
# ============================================================

CONDITIONS = [

    # --------------------------------------------------------
    # 停止状態・ボトルなし
    # --------------------------------------------------------

    (
        "static_no_bottle_white",
        "停止・ボトルなし・白",
        "static"
    ),

    (
        "static_no_bottle_black",
        "停止・ボトルなし・黒ライン",
        "static"
    ),

    (
        "static_no_bottle_blue",
        "停止・ボトルなし・青ライン",
        "static"
    ),

    # --------------------------------------------------------
    # 停止状態・ボトルあり
    # --------------------------------------------------------

    (
        "static_with_bottle_white",
        "停止・ボトルあり・白",
        "static"
    ),

    (
        "static_with_bottle_black",
        "停止・ボトルあり・黒ライン",
        "static"
    ),

    (
        "static_with_bottle_blue",
        "停止・ボトルあり・青ライン",
        "static"
    ),

    # --------------------------------------------------------
    # ライントレース状態
    # --------------------------------------------------------

    (
        "trace_no_bottle",
        "ライントレース・ボトルなし",
        "trace"
    ),

    (
        "trace_with_bottle",
        "ライントレース・ボトルあり",
        "trace"
    ),
]

# ============================================================
# DummyPlotter
#
# TraceLineではログ出力時に
#
#   runtime.plotter.get_distance()
#
# を使用するため、簡易的なPlotterを用意する。
#
# 今回は距離そのものは使用しないため、
# 常に0を返す。
# ============================================================

class DummyPlotter:

    def get_distance(self):

        return 0


# ============================================================
# カラーキャリブレーション
# ============================================================

class ColorCalibration:

    def __init__(self):

        # ----------------------------------------------------
        # 現在の測定条件
        # ----------------------------------------------------

        self.index = 0


        # ----------------------------------------------------
        # HSVサンプル
        # ----------------------------------------------------

        self.samples = []


        # ----------------------------------------------------
        # 条件ごとの集計結果
        # ----------------------------------------------------

        self.results = {}


        # ----------------------------------------------------
        # 状態管理
        # ----------------------------------------------------

        self.measuring = False

        self.finished = False

        self.initialized = False

        self.runtime_initialized = False


        # ----------------------------------------------------
        # 測定開始時間
        # ----------------------------------------------------

        self.start_time = None


        # ----------------------------------------------------
        # タッチセンサー前回状態
        #
        # 押しっぱなしによる連続開始を防ぐ
        # ----------------------------------------------------

        self.prev_touch = False


        # ----------------------------------------------------
        # TraceLine用 DummyPlotter
        # ----------------------------------------------------

        self.plotter = DummyPlotter()


        # ----------------------------------------------------
        # TraceLine生成
        # ----------------------------------------------------

        self.trace_line = TraceLine(

            name="ColorCalibrationTrace",

            target=TRACE_TARGET,

            power=TRACE_POWER,

            pid_p=PID_P,

            pid_i=PID_I,

            pid_d=PID_D,

            trace_side=TRACE_SIDE
        )


    # ========================================================
    # 測定条件の案内
    # ========================================================

    def print_condition(self):

        key, label, mode = CONDITIONS[self.index]

        print()
        print("=" * 60)
    
        print(
            "測定 %d / %d"
            % (
                self.index + 1,
                len(CONDITIONS)
            )
        )

        print(label)

        print("=" * 60)

        print(
            "走行体を測定開始位置に置いてください。"
        )

        print(
            "ボトルの有無を確認してください。"
        )

        print()

        if mode == "static":

            print(
                "カラーセンサーを対象の色の真上に置いてください。"
            )

            print(
                "この測定では走行体は動きません。"
            )

        else:

            print(
                "ライントレースできる位置に走行体を置いてください。"
            )

            print(
                "タッチ後、ライントレースしながら測定します。"
            )

        print()

        print(
            "準備できたらタッチセンサーを押してください。"
        )

        print()

        print(
            "%.1f秒間HSVを測定します。"
            % SAMPLE_SECONDS
        )


    # ========================================================
    # 測定結果集計
    # ========================================================

    def summarize(
        self,
        key,
        label
    ):

        print()

        print(
            "【測定結果】%s"
            % label
        )

        print(
            "サンプル数: %d"
            % len(self.samples)
        )


        # ----------------------------------------------------
        # サンプルが無い場合
        # ----------------------------------------------------

        if len(self.samples) == 0:

            print(
                "HSVデータが取得できませんでした。"
            )

            return


        result = {}


        # ----------------------------------------------------
        # H / S / V それぞれ集計
        # ----------------------------------------------------

        for name, i in [

            ("H", 0),

            ("S", 1),

            ("V", 2),

        ]:

            values = [

                sample[i]

                for sample in self.samples
            ]


            result[name] = {

                "mean":
                    statistics.mean(values),

                "median":
                    statistics.median(values),

                "min":
                    min(values),

                "max":
                    max(values),
            }


            print(

                "%s: "
                "平均=%.1f "
                "中央値=%.1f "
                "最小=%d "
                "最大=%d"

                % (

                    name,

                    result[name]["mean"],

                    result[name]["median"],

                    result[name]["min"],

                    result[name]["max"],
                )
            )


        self.results[key] = result


    # ========================================================
    # CSV保存
    # ========================================================

    def save_csv(self):

        filename = (

            "color_calibration_%s.csv"

            % datetime.now().strftime(
                "%Y%m%d_%H%M%S"
            )
        )


        with open(
            filename,
            "w",
            newline=""
        ) as f:

            writer = csv.writer(f)


            # ------------------------------------------------
            # ヘッダー
            # ------------------------------------------------

            writer.writerow([

                "condition",

                "color",

                "mean",

                "median",

                "min",

                "max",
            ])


            # ------------------------------------------------
            # 条件ごとの結果
            # ------------------------------------------------

            for key, result in self.results.items():

                for color in [

                    "H",

                    "S",

                    "V",

                ]:

                    r = result[color]


                    writer.writerow([

                        key,

                        color,

                        "%.2f"
                        % r["mean"],

                        "%.2f"
                        % r["median"],

                        r["min"],

                        r["max"],
                    ])


        print()

        print(
            "CSV保存完了: %s"
            % filename
        )


    # ========================================================
    # メイン処理
    # ========================================================

    def __call__(
    self,
    color_sensor: ColorSensor,
    touch_sensor: TouchSensor,
    left_motor: Motor,
    right_motor: Motor
    ):

        # ====================================================
        # runtime初期化
        # ====================================================

        if not self.runtime_initialized:

            runtime.configure(
                hub=None,
                arm_motor=None,
                right_motor=right_motor,
                left_motor=left_motor,
                touch_sensor=touch_sensor,
                color_sensor=color_sensor,
                sonar_sensor=None,
                gyro_sensor=None,
                plotter=self.plotter,
                video=None,
                course=COURSE,
            )

            self.runtime_initialized = True

            print()
            print("=" * 60)
            print("runtime 初期化完了")
            print("course = %s" % COURSE_NAME)
            print("=" * 60)

        # ====================================================
        # 全条件測定終了
        # ====================================================

        if self.finished:

            left_motor.set_power(0)
            right_motor.set_power(0)

            return

        # ====================================================
        # 初回案内
        # ====================================================

        if not self.initialized:

            self.initialized = True
            self.print_condition()

        # ====================================================
        # タッチセンサー状態取得
        # ====================================================

        touch = touch_sensor.is_pressed()

        # False → True の瞬間だけ検知
        touch_pressed = (
            touch
            and not self.prev_touch
        )

        self.prev_touch = touch

        # ====================================================
        # 測定開始待ち
        # ====================================================

        if not self.measuring:

            # 待機中は停止
            left_motor.set_power(0)
            right_motor.set_power(0)

            # --------------------------------------------
            # タッチされたら測定開始
            # --------------------------------------------

            if touch_pressed:

                key, label, mode = CONDITIONS[self.index]

                print()
                print("=" * 40)

                if mode == "static":
                    print("停止状態・測定開始")
                else:
                    print("ライントレース・測定開始")

                print("=" * 40)

                # HSVデータ初期化
                self.samples = []

                # 測定開始時刻
                self.start_time = time.monotonic()

                # 測定状態へ
                self.measuring = True

            return

        # ====================================================
        # 現在の測定条件
        # ====================================================

        key, label, mode = CONDITIONS[self.index]

        # ====================================================
        # 測定モードに応じた走行制御
        # ====================================================

        if mode == "trace":

            # ライントレースしながら測定
            self.trace_line.tick_once()

        else:

            # 停止状態で測定
            left_motor.set_power(0)
            right_motor.set_power(0)

        # ====================================================
        # HSV値取得
        # ====================================================

        h, s, v = color_sensor.get_raw_color_hsv()

        # ====================================================
        # HSV値保存
        # ====================================================

        self.samples.append(
            (
                h,
                s,
                v
            )
        )

        # ====================================================
        # HSVログ
        #
        # 約0.5秒に1回表示
        # 20ms × 25 = 約0.5秒
        # ====================================================

        if len(self.samples) % 25 == 0:

            print(
                "HSV: H=%d S=%d V=%d"
                % (
                    h,
                    s,
                    v
                )
            )

        # ====================================================
        # 経過時間
        # ====================================================

        elapsed = (
            time.monotonic()
            - self.start_time
        )

        # ====================================================
        # 測定終了判定
        # ====================================================

        if elapsed >= SAMPLE_SECONDS:

            # --------------------------------------------
            # TraceLine終了
            # --------------------------------------------

            if mode == "trace":

                self.trace_line.stop(
                    Status.INVALID
                )

            # --------------------------------------------
            # モーター停止
            # --------------------------------------------

            left_motor.set_power(0)
            right_motor.set_power(0)

            print()
            print("=" * 40)
            print("測定終了")
            print("=" * 40)

            # --------------------------------------------
            # 測定結果集計
            # --------------------------------------------

            self.summarize(
                key,
                label
            )

            # --------------------------------------------
            # 次の条件へ
            # --------------------------------------------

            self.index += 1

            self.samples = []
            self.measuring = False
            self.start_time = None

            # =================================================
            # 全条件終了
            # =================================================

            if self.index >= len(CONDITIONS):

                self.finished = True

                self.save_csv()

                print()
                print("=" * 60)
                print("すべての測定が完了しました。")
                print()
                print("Ctrl+C でプログラムを終了してください。")
                print("=" * 60)

            # =================================================
            # 次の測定条件
            # =================================================

            else:

                self.print_condition()
# ============================================================
# インスタンス作成
# ============================================================

calibration = ColorCalibration()

# ============================================================
# ETRobo実行
# ============================================================

(
    ETRobo(
        backend='raspike_art'
    )


    # ========================================================
    # カラーセンサー
    # ========================================================

    .add_device(

        'color_sensor',

        device_type=ColorSensor,

        port=COLOR_SENSOR_PORT
    )


    # ========================================================
    # タッチセンサー
    # ========================================================

    .add_device(

        'touch_sensor',

        device_type=TouchSensor,

        port=TOUCH_SENSOR_PORT
    )


    # ========================================================
    # 左モーター
    # ========================================================

    .add_device(

        'left_motor',

        device_type=Motor,

        port=LEFT_MOTOR_PORT
    )


    # ========================================================
    # 右モーター
    # ========================================================

    .add_device(

        'right_motor',

        device_type=Motor,

        port=RIGHT_MOTOR_PORT
    )


    # ========================================================
    # Handler
    # ========================================================

    .add_handler(
        calibration
    )


    # ========================================================
    # 実行開始
    # ========================================================

    .dispatch(
        interval=EXEC_INTERVAL
    )
)