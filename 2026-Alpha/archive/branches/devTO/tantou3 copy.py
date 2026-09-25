from py_trees.trees import BehaviourTree
from py_trees.composites import Sequence, Parallel
from py_trees.common import ParallelPolicy
import sample2

# sample2の部品を使う
RunByGyro = sample2.RunByGyro
SpinAround = sample2.SpinAround
TraceLine = sample2.TraceLine
IsQRDecoded = sample2.IsQRDecoded
IsColorDetected = sample2.IsColorDetected
IsDistanceEarned = sample2.IsDistanceEarned
StopNow = sample2.StopNow
IsTimePassed = sample2.IsTimePassed
TheEnd = sample2.TheEnd
TraverseBehaviourTree = sample2.TraverseBehaviourTree
ResetDevice = sample2.ResetDevice

HeadingType = sample2.HeadingType
Color = sample2.Color
TraceSide = sample2.TraceSide

TRACELINE_TARGET_V = sample2.TRACELINE_TARGET_V
SPIN_MAX_POWER = sample2.SPIN_MAX_POWER
SPIN_MIN_POWER = sample2.SPIN_MIN_POWER
EXEC_INTERVAL = sample2.EXEC_INTERVAL

initialize_etrobo = sample2.initialize_etrobo
setup_thread = sample2.setup_thread
cleanup_thread = sample2.cleanup_thread

# ============================================================
# build_tantou_tree
#
# コース全体：
#
# START
#   ↓
# 左90°
#   ↓
# 黒線まで直進
#   ↓
# 黒線で停止
#   ↓
# 右90°
#   ↓
# ライントレース
#   ↓
# QR1読み取り
#   ↓
# 青線まで直進
#   ↓
# 青線で停止
#   ↓
# 左90°
#   ↓
# 1500mmライントレース
#   ↓
# 停止
#   ↓
# 右90°
#   ↓
# QR2読み取り
#   ↓
# 左90°
#   ↓
# 青線まで直進
#   ↓
# 停止
#   ↓
# END
#
# sample2.py の以下のクラス等を利用する前提：
#
# RunByGyro
# SpinAround
# TraceLine
# IsQRDecoded
# IsColorDetected
# IsDistanceEarned
# StopNow
# IsTimePassed
# TheEnd
#
# また、
# HeadingType
# Color
# TraceSide
# TRACELINE_TARGET_V
# SPIN_MAX_POWER
# SPIN_MIN_POWER
# EXEC_INTERVAL
# setup_thread()
# initialize_etrobo()
# cleanup_thread()
# TraverseBehaviourTree
#
# なども sample2.py 側から利用できる前提。
# ============================================================


def build_tantou_tree() -> BehaviourTree:

    root = Sequence(
        name="Tantou Section",
        memory=True
    )

    # ========================================================
    # 1. START → 左55°
    # ========================================================

    turn_left_90 = Sequence(
        name="turn_left_90",
        memory=True
    )

    turn_left_90.add_children([
        ResetDevice(name="device_reset"), 
        SpinAround(
            name="left 90",
            target=55,
            max_power=SPIN_MAX_POWER,
            min_power=SPIN_MIN_POWER,
            pid_p=0.2,
            pid_i=0.00075,
            pid_d=0.03,
            target_type=HeadingType.RELATIVE
        ),

        #IsTimePassed(name="wait_after_spin", delta_time=0.05), 

        StopNow(
            name="stop_after_left"
        ),

        IsTimePassed(
            name="wait_left",
            delta_time=0.5
        ),
    ])


    # ========================================================
    # 2. 黒線まで直進
    #
    # 走行と黒線検出を同時に実行する。
    #
    # ・黒線を検出
    # または
    # ・600mm到達
    #
    # のどちらかで終了。
    # ========================================================

    go_to_black = Parallel(
        name="go_to_black",
        policy=ParallelPolicy.SuccessOnOne()
    )

    go_to_black.add_children([
        RunByGyro(
            name="run_to_black",
            target=55,
            power=40,
            pid_p=0.02,
            pid_i=0.000075,
            pid_d=0.04,
            target_type=HeadingType.ABSOLUTE
        ),

        #IsColorDetected(
         #   name="detect_black",
         #   color=Color.BLACK
        #),

        IsDistanceEarned(
            name="black_distance_limit",
            delta_dist=700
        ),       
    ])


    # 黒線地点で停止

    #stop_at_black = StopNow(
    #    name="stop_black"
    #)

    # ========================================================
    # 3.左55°
    # ========================================================

    turn_left_90 = Sequence(
        name="turn_left_90",
        memory=True
    )


    # ========================================================
    # 3. 黒線 → QR1
    #
    # ライントレースしながらQRコードを監視する。
    #
    # QRコードがデコードできたら終了。
    # ========================================================

    trace_to_qr1 = Parallel(
        name="trace_to_qr1",
        policy=ParallelPolicy.SuccessOnOne()
    )

    trace_to_qr1.add_children([
        TraceLine(
            name="trace_to_qr1_line",
            target=TRACELINE_TARGET_V,
            power=50,
            pid_p=0.55,
            pid_i=0.0000009,
            pid_d=0.015,
            trace_side=TraceSide.NORMAL
        ),

        IsQRDecoded(
            name="read_qr1"
        ),
    ])


    # QR1で停止

    stop_at_qr1 = StopNow(
        name="stop_at_qr1"
    )


    # ========================================================
    # 4. QR1 → 青線
    #
    # QR1読み取り後、
    # ・直進
    # ・青線検出
    # ・150mm到達
    #
    # を同時に監視する。
    # ========================================================

    go_to_blue_after_qr1 = Parallel(
        name="go_to_blue_after_qr1",
        policy=ParallelPolicy.SuccessOnOne()
    )

    go_to_blue_after_qr1.add_children([
        RunByGyro(
            name="run_after_qr1",
            target=0,
            power=50,
            pid_p=1.1,
            pid_i=0.00075,
            pid_d=0.04,
            target_type=HeadingType.RELATIVE
        ),

        IsColorDetected(
            name="detect_blue_after_qr1",
            color=Color.BLUE
        ),

        IsDistanceEarned(
            name="distance_after_qr1",
            delta_dist=150
        ),
    ])


    # 青線地点で停止

    stop_at_blue = StopNow(
        name="stop_at_blue"
    )


    # ========================================================
    # 5. 青線 → 左90°
    # ========================================================

    turn_left_90_b = Sequence(
        name="turn_left_90_b",
        memory=True
    )

    turn_left_90_b.add_children([
        SpinAround(
            name="left 90 again",
            target=90,
            max_power=SPIN_MAX_POWER,
            min_power=SPIN_MIN_POWER,
            pid_p=0.2,
            pid_i=0.00075,
            pid_d=0.03,
            target_type=HeadingType.RELATIVE
        ),

        StopNow(
            name="stop_after_left_b"
        ),

        IsTimePassed(
            name="wait_left_b",
            delta_time=0.5
        ),
    ])


    # ========================================================
    # 6. 1500mmライントレース
    #
    # ライントレースしながら1500mm走行する。
    #
    # 1500mm到達で終了。
    # ========================================================

    line_trace_150 = Parallel(
        name="line_trace_150",
        policy=ParallelPolicy.SuccessOnOne()
    )

    line_trace_150.add_children([
        TraceLine(
            name="trace_150",
            target=TRACELINE_TARGET_V,
            power=50,
            pid_p=0.55,
            pid_i=0.0000009,
            pid_d=0.015,
            trace_side=TraceSide.NORMAL
        ),

        IsDistanceEarned(
            name="dist_1500",
            delta_dist=1500
        ),
    ])


    # 1500mm地点で停止

    stop_at_1500 = StopNow(
        name="stop_trace_150"
    )


    # ========================================================
    # 7. 1500mm地点 → 右90°
    # ========================================================

    turn_right_qr2 = Sequence(
        name="turn_right_qr2",
        memory=True
    )

    turn_right_qr2.add_children([
        StopNow(
            name="stop_before_qr2"
        ),

        SpinAround(
            name="right 90 for qr2",
            target=-90,
            max_power=SPIN_MAX_POWER,
            min_power=SPIN_MIN_POWER,
            pid_p=0.2,
            pid_i=0.00075,
            pid_d=0.03,
            target_type=HeadingType.RELATIVE
        ),

        StopNow(
            name="stop_after_qr2_turn"
        ),

        IsTimePassed(
            name="wait_after_qr2_turn",
            delta_time=0.5
        ),
    ])


    # ========================================================
    # 8. QR2読み取り
    #
    # 絶対方位0°を向いてQR2を読む。
    # ========================================================

    qr2_read = Sequence(
        name="qr2_read",
        memory=True
    )

    qr2_read.add_children([
        SpinAround(
            name="face_qr2",
            target=0,
            max_power=SPIN_MAX_POWER,
            min_power=SPIN_MIN_POWER,
            pid_p=0.2,
            pid_i=0.00075,
            pid_d=0.03,
            target_type=HeadingType.ABSOLUTE
        ),

        StopNow(
            name="stop_face_qr2"
        ),

        IsQRDecoded(
            name="read_qr2"
        ),

        StopNow(
            name="stop_after_qr2"
        ),
    ])


    # ========================================================
    # 9. QR2 → 元の向き
    #
    # 左90°回転。
    # ========================================================

    return_heading = Sequence(
        name="return_heading",
        memory=True
    )

    return_heading.add_children([
        SpinAround(
            name="left 90 return",
            target=90,
            max_power=SPIN_MAX_POWER,
            min_power=SPIN_MIN_POWER,
            pid_p=0.2,
            pid_i=0.00075,
            pid_d=0.03,
            target_type=HeadingType.RELATIVE
        ),

        StopNow(
            name="stop_after_return_heading"
        ),

        IsTimePassed(
            name="wait_return_heading",
            delta_time=0.5
        ),
    ])


    # ========================================================
    # 10. 最終地点まで直進
    #
    # 青線を検出するまで走行。
    #
    # 安全のため600mmの距離上限も設定。
    # ========================================================

    go_to_goal = Parallel(
        name="go_to_goal",
        policy=ParallelPolicy.SuccessOnOne()
    )

    go_to_goal.add_children([
        RunByGyro(
            name="run_to_goal",
            target=0,
            power=50,
            pid_p=1.1,
            pid_i=0.00075,
            pid_d=0.04,
            target_type=HeadingType.RELATIVE
        ),

        IsColorDetected(
            name="detect_goal_blue",
            color=Color.BLUE
        ),

        IsDistanceEarned(
            name="goal_distance_limit",
            delta_dist=600
        ),
    ])


    # ゴールで停止

    goal_stop = StopNow(
        name="stop_final"
    )


    # ========================================================
    # 全体の流れ
    # ========================================================

    root.add_children([

        # START → 左90°
        turn_left_90,

        # 左90° → 黒線
        go_to_black,
        stop_at_black,

        # # 黒線 → QR1
        # trace_to_qr1,
        # stop_at_qr1,

        # # QR1 → 青線
        # go_to_blue_after_qr1,
        # stop_at_blue,

        # # 青線 → 左90°
        # turn_left_90_b,

        # # 左90° → 1500mm
        # line_trace_150,
        # stop_at_1500,

        # # 1500mm → 右90° → QR2
        # turn_right_qr2,
        # qr2_read,

        # # QR2 → 元の向き
        # return_heading,

        # # GOAL
        # go_to_goal,
        # goal_stop,

        # # 終了
        TheEnd(
            name="end"
        ),
    ])

    return BehaviourTree(root)


# ============================================================
# 実行部
# ============================================================

if __name__ == '__main__':

    import argparse

    parser = argparse.ArgumentParser()

    parser.add_argument(
        'course',
        choices=['right', 'left']
    )

    args = parser.parse_args()


    # コース方向設定

    #if args.course == 'right':
    #    g_course = -1
    #else:
    #    g_course = 1
    
    if args.course == 'right':
        sample2.g_course = -1
    else:
        sample2.g_course = 1
    
    print("### g_course =", sample2.g_course)

    # センサ・デバイス初期化

    setup_thread()


    try:

        etrobo = initialize_etrobo(
            backend='raspike_art'
        )

        # Behaviour Tree作成

        tree = build_tantou_tree()

        # 実行登録

        etrobo.add_handler(
            TraverseBehaviourTree(tree)
        )

        # 実行開始

        etrobo.dispatch(
            interval=EXEC_INTERVAL
        )


    finally:

        cleanup_thread()

        print(" -- exiting...")
