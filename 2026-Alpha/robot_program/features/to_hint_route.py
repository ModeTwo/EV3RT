"""TO担当の編集箇所。非B版tantou3.pyの変数名・コメント・ツリー定義を保持。

【統合差分】sample2の単体機器を作らず共有部品を使用。
SpinAround/RunByGyroはAT終了方位基準、IsQRDecodedは共有Context保存。
各工程のパラメータは下の元コードの位置で編集する。
"""
from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time
from ..behaviours.section_motion import LocalSpin as SpinAround, LocalDrive as RunByGyro
from ..behaviours.line_trace import TraceLine
from ..behaviours.conditions import IsDistanceEarned, IsTimePassed
from ..behaviours.motor_control import StopNow
from ..behaviours.hint_reader import ReadHintCard as IsQRDecoded


def build_tantou_tree(context, config, include_exit=True):
    settings = config.integration
    SPIN_MAX_POWER = settings.to_spin_max_power  # 元: 60
    SPIN_MIN_POWER = settings.to_spin_min_power  # 元: 55
    TRACELINE_TARGET_V = 65

    root = Sequence(
        name="Tantou Section",
        memory=True
    )

    # ========================================================
    # 1. START → 左55°
    # ========================================================

    turn_left_55 = Sequence(
        name="turn_left_55",
        memory=True
    )

    #turn_left_55.add_children([
        #RunByGyro(
            #name="left 55",
            #target=0,
            #power=50,
            #pid_p=1.1,
            #pid_i=0.00075,
            #pid_d=0.04,
            #target_type=HeadingType.RELATIVE
        #),
#
        #IsDistanceEarned(
            #name="distance_turn_left_55",
            #delta_dist=100
        #),
#
        #RunByGyro(
            #name="turn_left_55",
            #target=55,               # 左に55°
            #power=50,
            #pid_p=1.1,
            #pid_i=0.00075,
            #pid_d=0.04,
            #target_type=HeadingType.RELATIVE
        #),
#
        #IsTimePassed(
            #name="wait_after_turn",
            #delta_time=0.3           # 姿勢安定のため少し待つ
         #),
#
        ## ③ 70cm（700mm）進む
        #RunByGyro(
            #name="go_700mm",
            #target=55,               # 左55°方向へ直進
            #power=50,
            #pid_p=1.1,
            #pid_i=0.00075,
            #pid_d=0.04,
            #target_type=HeadingType.RELATIVE
        #),
        #IsDistanceEarned(
            #name="dist_700mm",
            #delta_dist=700
        #),

    turn_left_55.add_children([
        # 【統合差分】ジャイロをリセットせずAT終了方位を局所0度とする。
        SpinAround(
            context=context,  # 【統合差分】AT終了方位を基準に変換
            name="left 55",
            target=90,
            max_power=SPIN_MAX_POWER,
            min_power=SPIN_MIN_POWER,
            pid_p=0.2,
            pid_i=0.00075,
            pid_d=0.03,
            target_type=HeadingType.ABSOLUTE
        ),

        #IsTimePassed(name="wait_after_spin", delta_time=0.05),

        StopNow(
            name="stop_after_left_e"
        ),

        IsTimePassed(
            name="wait_left_e",
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
            context=context,  # 【統合差分】AT終了方位を基準に変換
            name="run_to_black",
            target=0,
            power=60,
            pid_p=0.0001,
            pid_i=0.00001,
            pid_d=0.04,
            target_type=HeadingType.ABSOLUTE
        ),

        #IsColorDetected(
         #   name="detect_black",
         #   color=Color.BLACK
        #),

        IsDistanceEarned(
            name="black_distance_limit",
            delta_dist=settings.to_first_black_limit_mm  # 元: 565
        ),
       # ResetDevice(name="device_reset"),
    ])


    # 黒線地点で停止

    #stop_at_black = StopNow(
    #    name="stop_black"
    #)

    # ========================================================
    # 3.右125°
    # ========================================================

    turn_right_125 = Sequence(
        name="turn_right_125",
        memory=True
    )

    turn_right_125.add_children([
        #ResetDevice(name="device_reset"),
        SpinAround(
            context=context,  # 【統合差分】AT終了方位を基準に変換
            name="right 125",
            target=0,
            max_power=SPIN_MAX_POWER,
            min_power=SPIN_MIN_POWER,
            pid_p=0.2,
            pid_i=0.00075,
            pid_d=0.03,
            target_type=HeadingType.ABSOLUTE
        ),

        #IsTimePassed(name="wait_after_spin", delta_time=0.05),

        StopNow(
            name="stop_after_right"
        ),

        IsTimePassed(
            name="wait_right",
            delta_time=0.5
        ),
    ])


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
        # TraceLine(
         #   name="trace_to_qr1_line",
          #  target=TRACELINE_TARGET_V,
           # power=50,
            #pid_p=0.55,
            #pid_i=0.0000009,
            #pid_d=0.015,
            #trace_side=TraceSide.NORMAL
        #),

        IsQRDecoded(
            name="read_qr1", context=context, hint_number=1
        ),
    ])

    #ここに読み取れない場合の挙動追加

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
        #memory=True
    )

    go_to_blue_after_qr1.add_children([
        RunByGyro(
            context=context,  # 【統合差分】AT終了方位を基準に変換
            name="run_after_qr1",
            target=0,
            power=60,
            pid_p=0.0001,
            pid_i=0.00001,
            pid_d=0.04,
            target_type=HeadingType.ABSOLUTE
        ),

        #IsColorDetected(
        #    name="detect_blue_after_qr1",
        #    color=Color.BLUE
        #),

        IsDistanceEarned(
            name="distance_after_qr1",
            delta_dist=settings.to_after_hint1_mm  # 元: 385
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
        #ResetDevice(name="device_reset"),
        SpinAround(
            context=context,  # 【統合差分】AT終了方位を基準に変換
            name="left 90 again",
            target=90,
            max_power=SPIN_MAX_POWER,
            min_power=SPIN_MIN_POWER,
            pid_p=0.2,
            pid_i=0.00075,
            pid_d=0.03,
            target_type=HeadingType.ABSOLUTE
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
    # 6. 1200mmライントレース
    #
    # ライントレースしながら1200mm走行する。
    #
    # 1200mm到達で終了。
    # ========================================================

    line_trace_120 = Parallel(
        name="line_trace_120",
        policy=ParallelPolicy.SuccessOnOne()
    )

    line_trace_120.add_children([
        TraceLine(
            name="trace_120",
            target=TRACELINE_TARGET_V,
            power=60,
            pid_p=0.055,
            pid_i=0.005,
            pid_d=0.5,
            trace_side=TraceSide.NORMAL,
            cutoff_hz=None  # 【統合差分】元sample2と同じ平滑化なし
        ),

        IsDistanceEarned(
            name="dist_1200",
            delta_dist=settings.to_hint2_trace_mm  # 元: 1000
        ),
    ])


    # 1200mm地点で停止

    stop_at_1200 = StopNow(
        name="stop_trace_120"
    )


    # ========================================================
    # 7. 1200mm地点 → 右30°
    # ========================================================

    turn_right_qr2 = Sequence(
        name="turn_right_qr2",
        memory=True
    )

    turn_right_qr2.add_children([
        StopNow(
            name="stop_before_qr2"
        ),

        #ResetDevice(name="device_reset"),
        SpinAround(
            context=context,  # 【統合差分】AT終了方位を基準に変換
            name="right 25 for qr2",
            target=25,
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
        # SpinAround(
        #     name="face_qr2",
        #     target=0,
        #     max_power=SPIN_MAX_POWER,
        #     min_power=SPIN_MIN_POWER,
        #     pid_p=0.2,
        #    pid_i=0.00075,
        #    pid_d=0.03,
        #    target_type=HeadingType.ABSOLUTE """
        #),

       #  StopNow(
        #    name="stop_face_qr2"
        #),

        IsQRDecoded(
            name="read_qr2", context=context, hint_number=2
        ),

        StopNow(
            name="stop_after_qr2"
        ),
    ])


    # ========================================================
    # 9. QR2 → 元の向き
    #
    # 右30°回転。
    # ========================================================

    return_heading = Sequence(
        name="return_heading",
        memory=True
    )

    return_heading.add_children([
        SpinAround(
            context=context,  # 【統合差分】AT終了方位を基準に変換
            name="right 25 return",
            target=-25,
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
        TraceLine(
            name="trace_150",
            target=TRACELINE_TARGET_V,
            power=50,
            pid_p=0.55,
            pid_i=0.0000009,
            pid_d=0.015,
            trace_side=TraceSide.NORMAL,
            cutoff_hz=None  # 【統合差分】元sample2と同じ平滑化なし
        ),


        #IsColorDetected(
        #    name="detect_goal_blue",
        #    color=Color.BLUE
        #),

        IsDistanceEarned(
            name="goal_distance_limit",
            delta_dist=settings.to_exit_trace_mm  # 元: 600
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
        turn_left_55,
        go_to_black,
        turn_right_125,
        trace_to_qr1,
        stop_at_qr1,
        go_to_blue_after_qr1,
        stop_at_blue,
        turn_left_90_b,
        line_trace_120,
        stop_at_1200,
        turn_right_qr2,
        qr2_read,

        # 【統合差分】TheEndはalpha.pyの末尾が担当。
    ])

    # 【統合差分】hint2単体モードのみ読取後で終了。通常は元の出口走行まで続ける。
    if include_exit:
        root.add_children([
            return_heading,
            go_to_goal,
            goal_stop,
        ])

    # 【統合差分】alphaが所有するツリーへ接続するためrootを返す。
    return root
