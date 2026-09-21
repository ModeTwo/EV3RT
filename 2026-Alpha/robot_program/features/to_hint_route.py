"""TO担当の編集箇所。受領tantou4.pyの工程順・変数名・ツリー定義を保持。

【統合差分】sample2の単体機器を作らず共有部品を使用。
SpinAround/RunByGyroは起動時からの共通方位基準、IsQRDecodedは共有Context保存。
各工程のパラメータは下の元コードの位置で編集する。
【統合差分】ResetDeviceは入れず共通方位を維持。QR準備/読取とHint2Exitは現行版。
添付の角度-90/25/-25は基準を確認できないため採用せず、既存0/115/90を維持。
"""
from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time
from ..behaviours.corrected_run import LocalEtRun as RunByGyro
# 旋回はエンコーダ旋回+ジャイロ仕上げ(behaviours/encoder_spin.py)。直進はET用(behaviours/corrected_run.py)
from ..behaviours.encoder_spin import LocalEncoderSpin as SpinAround
from ..behaviours.line_trace import TraceLine
from ..behaviours.camera_line_trace import RecoverLineByCamera
from ..behaviours.hint2_exit import Hint2Exit
from ..behaviours.projected_distance import IsProjectedDistanceEarned
from ..behaviours.conditions import IsDistanceEarned, IsTimePassed, IsColorDetected, IsBlackDetected, IsColorPassed, IsDistanceEarnedUntilColorEntered
from ..behaviours.motor_control import StopNow
from ..behaviours.hint_reader import ReadHintCard as IsQRDecoded, PrepareHintCamera


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
    
    go_to_qr = Parallel(
        name="go_to_qr",
        policy=ParallelPolicy.SuccessOnOne()
    )
    
    go_to_qr_drive = RunByGyro(
        context=context,
        name="go_to_qr_drive",
        target=0,
        power=60,
        pid_p=1.1,
        pid_i=0.00075,
        pid_d=0.04,
        target_type=HeadingType.ABSOLUTE
    )
    
    limit_qr = IsDistanceEarned(
        name="limit_qr",
        delta_dist=50
    )
    
    go_to_qr.add_children([
        go_to_qr_drive,
        limit_qr,
    ])


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

        #IsColorDetected(
         #   name="detect_black",
         #   color=Color.BLACK
        #),

    turn_left_55.add_children([
        # 【統合差分】ジャイロをリセットせず、起動時からの共通方位基準を維持する。
        SpinAround(
            context=context,  # 【統合差分】AT終了方位を加算せず共通方位を使用
            name="left 55",
            target=90,
            max_power=SPIN_MAX_POWER,
            min_power=SPIN_MIN_POWER,
            pid_p=0.2,
            pid_i=0.005,
            pid_d=0.03,
            target_type=HeadingType.ABSOLUTE
        ),

        #IsTimePassed(name="wait_after_spin", delta_time=0.05),

        StopNow(
            name="stop_after_left_e"
        ),

        IsTimePassed(
            name="wait_left_e",
            delta_time=0.2
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
            context=context,  # 【統合差分】AT終了方位を加算せず共通方位を使用
            name="run_to_black",
            target=90,
            power=55,
            pid_p=1.1,
            pid_i=0.00075,
            pid_d=0.04,
            target_type=HeadingType.ABSOLUTE
        ),

        #IsColorDetected(
         #   name="detect_black",
         #   color=Color.BLACK
        #),

        IsDistanceEarned(
            name="black_distance_limit",
            delta_dist=settings.to_first_black_limit_mm  # tantou4: 550
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
            context=context,  # 【統合差分】AT終了方位を加算せず共通方位を使用
            name="right 125",
            target=0,  # 【統合差分】添付-90。共通方位の既存値を維持
            max_power=SPIN_MAX_POWER,
            min_power=SPIN_MIN_POWER,
            pid_p=0.2,
            pid_i=0.005,
            pid_d=0.03,
            target_type=HeadingType.ABSOLUTE
        ),

        #IsTimePassed(name="wait_after_spin", delta_time=0.05),

        StopNow(
            name="stop_after_right"
        ),

        IsTimePassed(
            name="wait_right",
            delta_time=0.2
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
            # 【統合差分】QR1成功直後にカメラをLINEモードへ戻し、直進・旋回中に
            # カメラ切替(約3.4秒)を隠す。QR2向けの再準備は後段のカメラ復帰後に行う。
            name="read_qr1", context=context, hint_number=1, keep_qr_ready=False
        ),
    ])

    #ここに読み取れない場合の挙動追加

    # QR1で停止

    stop_at_qr1 = StopNow(
        name="stop_at_qr1"
    )


    # ========================================================
    # 4. QR1 → 緑通過後320mm
    #
    # QR1読み取り後、
    # ・直進
    # ・緑以外の色から緑に入り、緑を通り抜けたら、そこから320mm直進
    # ・(保険)緑にまだ入っていない間だけ有効な安全上限距離。
    #   緑に入った後は無効化し、緑を見続けても打ち切られない。
    #
    # を同時に監視する。
    # ========================================================

    pass_through_green_after_qr1 = IsColorPassed(
        name="pass_through_green_after_qr1",
        color=Color.GREEN
    )

    green_pass_then_320mm = Sequence(
        name="green_pass_then_320mm",
        memory=True
    )
    green_pass_then_320mm.add_children([
        pass_through_green_after_qr1,

        IsDistanceEarned(
            name="distance_after_green",
            #delta_dist=settings.to_after_hint1_green_pass_mm  # 緑通過後320mm
            delta_dist=165
        ),
    ])

    go_to_blue_after_qr1 = Parallel(
        name="go_to_blue_after_qr1",
        policy=ParallelPolicy.SuccessOnOne()
        #memory=True
    )

    go_to_blue_after_qr1.add_children([
        RunByGyro(
            context=context,  # 【統合差分】AT終了方位を加算せず共通方位を使用
            name="run_after_qr1",
            target=0,
            power=60,
            pid_p=1.1,
            pid_i=0.00075,
            pid_d=0.04,
            target_type=HeadingType.ABSOLUTE
        ),

        green_pass_then_320mm,

        IsDistanceEarnedUntilColorEntered(
            # 緑を見つけるまでの安全上限。緑に入った後は無効化される(実機未校正)。
            name="distance_after_qr1_safety_limit",
            delta_dist=settings.to_after_hint1_safety_limit_mm,
            guard=pass_through_green_after_qr1
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
            context=context,  # 【統合差分】AT終了方位を加算せず共通方位を使用
            name="left 90 again",
            #target=90,
            target=45,
            max_power=SPIN_MAX_POWER,
            min_power=SPIN_MIN_POWER,
            pid_p=0.2,
            pid_i=0.005,
            pid_d=0.03,
            target_type=HeadingType.ABSOLUTE
        ),

        StopNow(
            name="stop_after_left_b"
        ),

        IsTimePassed(
            name="wait_left_b",
            delta_time=0.2
        ),
    ])


    # ========================================================
    # 6. カメラでライン中央へ寄せてから1200mmライントレース
    #
    # 【統合差分】90度旋回直後は色センサーtrace_120だけでは追従開始距離が
    # 足りず脱線するため、LAP前と同じRecoverLineByCamera(camera_recovery)で
    # カメラ操舵→方位安定→色センサーへの低速引渡しを行ってから、
    # 従来のtrace_120(色センサー)へ接続する。
    # カメラはQR1直後にLINEへ戻し済み(read_qr1のkeep_qr_ready=False)。
    # camera_recovery成功直後にQR2向け撮像を再準備し、残りのtrace_120走行
    # (数秒〜十数秒)へ約3.4秒のカメラ切替を重ねる。
    #
    # 1200mm(カメラ復帰区間を含む投影距離)到達で終了。
    # ========================================================

    #camera_recovery_after_hint1 = RecoverLineByCamera(
        #name="recover line by camera after hint1",
        #power=settings.to_after_hint1_camera_power,
        #pid_p=settings.to_after_hint1_camera_pid_p,
        #pid_i=settings.to_after_hint1_camera_pid_i,
        #pid_d=settings.to_after_hint1_camera_pid_d,
        #max_camera_turn=settings.to_after_hint1_camera_max_turn,
        #align_power=settings.to_after_hint1_camera_align_power,
        #handoff_power=settings.to_after_hint1_camera_handoff_power,
        #handoff_target_v=TRACELINE_TARGET_V,
        #handoff_pid_p=settings.to_after_hint1_camera_handoff_pid_p,
        #handoff_turn_cap=settings.to_after_hint1_camera_handoff_turn_cap,
        #handoff_v_tolerance=settings.to_after_hint1_camera_handoff_v_tolerance,
        #handoff_stable_samples=settings.to_after_hint1_camera_handoff_stable_samples,
        #line_v=settings.to_after_hint1_camera_rejoin_v,
        #line_samples=settings.to_after_hint1_camera_rejoin_samples,
        #trace_side=TraceSide.NORMAL,
        #tilt_ff_gain=settings.to_after_hint1_camera_tilt_ff_gain,
        #ff_cap=settings.to_after_hint1_camera_ff_cap,
        #heading_tolerance_deg=settings.to_after_hint1_camera_heading_tolerance_deg,
        #stable_samples=settings.to_after_hint1_camera_stable_samples,
        ## 【統合差分】LAP前の流用元は0度だが、この区間は"left 90 again"で絶対方位90度へ
        ## 旋回済みで、trace_120の前進方向もdist_1200と同じ90度。0度のままだと
        ## ALIGN/HANDOFFがtrace_120の進行方向と直角の向きへ引き込んでしまう。
        #gyro_heading_deg=90.0,
        #gyro_kp=settings.to_after_hint1_camera_gyro_kp,
        #gyro_turn_cap=settings.to_after_hint1_camera_gyro_turn_cap,
    #)
#
    #camera_recovery_then_trace_120 = Sequence(
        #name="camera_recovery_then_trace_120",
        #memory=True
    #)
    #camera_recovery_then_trace_120.add_children([
        #camera_recovery_after_hint1,
#
        #PrepareHintCamera(name="prepare hint2 camera"),
#
        #TraceLine(
            #name="trace_120",
            #target=TRACELINE_TARGET_V,
            #power=60,
            #pid_p=0.65,
            #pid_i=0.000001,
            #pid_d=0.045,
            #trace_side=TraceSide.NORMAL,
        #),
    #])
#
    #line_trace_120 = Parallel(
        #name="line_trace_120",
        #policy=ParallelPolicy.SuccessOnOne()
    #)
#
    #line_trace_120.add_children([
        #camera_recovery_then_trace_120,
#
        #IsProjectedDistanceEarned(
            #name="dist_1200", context=context, local_heading_deg=90.0,
            ##delta_dist=settings.to_hint2_trace_mm  # 【統合差分】投影距離を維持、tantou4の1200mmを採用
            #delta_dist=650
        #),
    #])
    
    # --- camera recovery を使わない簡易版の line_trace_120 定義 ---
    # ==========================================
    # ① 黒線検知 OR 550mm走行
    # ==========================================
    
    black_or_550 = Parallel(
        name="black_or_550",
        policy=ParallelPolicy.SuccessOnOne()
    )
    
    run_to_black = RunByGyro(
        context=context,
        name="run_to_black",
        target=45,
        power=50,
        pid_p=1.1,
        pid_i=0.00075,
        pid_d=0.04,
        target_type=HeadingType.ABSOLUTE
    )
    
    detect_black = IsBlackDetected(
        name="detect_black",
        black_threshold=settings.to_exit_black_v,
        required_frames=25
    )
    
    black_distance_limit = IsDistanceEarned(
        name="black_distance_limit",
        delta_dist=200
    )
    
    black_or_550.add_children([
        run_to_black,
        detect_black,
        black_distance_limit,
    ])

    # ==========================================
    # ①終了後、黒検知の有無に関わらず絶対方位90度へ向き直してから
    # 650mmライントレースへ進む(ET相撲のガレージ復帰と同じ「検知→既知方位へ
    # 旋回」構造。見つからなかった場合も失敗にはせず、90度で試行を続ける)。
    # ==========================================

    turn_to_90_before_trace = SpinAround(
        context=context,
        name="turn_to_90_before_trace",
        target=90,
        max_power=SPIN_MAX_POWER,
        min_power=SPIN_MIN_POWER,
        pid_p=0.2,
        pid_i=0.005,
        pid_d=0.03,
        target_type=HeadingType.ABSOLUTE
    )

    stop_after_turn_to_90 = StopNow(
        name="stop_after_turn_to_90"
    )

    wait_after_turn_to_90 = IsTimePassed(
        name="wait_after_turn_to_90",
        delta_time=0.2
    )

    # ==========================================
    # ② ①終了後 → ライントレース
    # 【統合差分】独自の650mm上限は削除。終了は外側のdist_1200_from_75deg_start
    # (75度直進の開始位置を基準にした90度成分1200mm)だけに一本化する。
    # ==========================================

    trace_650 = TraceLine(
        name="trace_650",
        target=TRACELINE_TARGET_V,
        power=50,
        pid_p=0.65,
        pid_i=0.000001,
        pid_d=0.045,
        trace_side=TraceSide.NORMAL,
    )


    # ==========================================
    # ③ 順番に実行
    # ==========================================

    line_trace_120 = Sequence(
        name="line_trace_120",
        memory=True
    )

    line_trace_120.add_children([
        black_or_550,
        turn_to_90_before_trace,
        stop_after_turn_to_90,
        wait_after_turn_to_90,
        # 【統合差分】QR2用のカメラ切替(~3.4秒)をtrace_650の走行時間に重ねて隠す。
        # camera_recovery_then_trace_120(コメントアウト済み)にあった
        # PrepareHintCamera("prepare hint2 camera")と同じ配置・目的。
        # これが無いと、read_qr2開始時に初めて切替が始まり、切替コストが
        # そのままQR2読み取りのelapsed時間に乗ってしまう。
        PrepareHintCamera(name="prepare hint2 camera"),
        trace_650,
    ])

    # 75度直進の開始位置(black_or_550が最初にtickされる瞬間)を基準に、
    # 90度成分で1200mm進んだら全体を打ち切る(旧プログラムの1200mm相当)。
    # trace_650自体には独自の終了条件がないため、これが唯一の終了条件になる。
    dist_1200_from_75deg_start = IsProjectedDistanceEarned(
        name="dist_1200_from_75deg_start",
        context=context,
        local_heading_deg=90.0,
        delta_dist=settings.to_hint2_trace_mm,
    )

    line_trace_120_with_cap = Parallel(
        name="line_trace_120_with_cap",
        policy=ParallelPolicy.SuccessOnOne()
    )
    line_trace_120_with_cap.add_children([
        line_trace_120,
        dist_1200_from_75deg_start,
    ])

    # 650mm到達後に停止
    stop_at_650 = StopNow(
        name="stop_after_650"
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
            context=context,  # 【統合差分】AT終了方位を加算せず共通方位を使用
            name="right 25 for qr2",
            target=115,  # 【統合差分】添付25。共通方位の既存値を維持
            max_power=SPIN_MAX_POWER,
            min_power=SPIN_MIN_POWER,
            pid_p=0.2,
            pid_i=0.005,
            pid_d=0.03,
            target_type=HeadingType.ABSOLUTE
        ),

        StopNow(
            name="stop_after_qr2_turn"
        ),

        IsTimePassed(
            name="wait_after_qr2_turn",
            delta_time=0.2
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

    #return_heading = Sequence(
        #name="return_heading",
        #memory=True
    #)
#
    #return_heading.add_children([
        #SpinAround(
            #context=context,  # 【統合差分】AT終了方位を加算せず共通方位を使用
            #name="right 25 return",
            #target=90,  # 【統合差分】添付-25。共通方位の既存値を維持
            #max_power=SPIN_MAX_POWER,
            #min_power=SPIN_MIN_POWER,
            #pid_p=0.2,
            #pid_i=0.005,
            #pid_d=0.03,
            #target_type=HeadingType.ABSOLUTE
        #),
#
        #StopNow(
            #name="stop_after_return_heading"
        #),
#
        #IsTimePassed(
            #name="wait_return_heading",
            #delta_time=0.2
        #),
    #])
#
#
    ## 【統合差分】添付の600mm単純追従に置換せず、現行Hint2Exitを維持。
    ## 10. 90度保持→連続白→追加前進→TO基準190度へのその場旋回→黒線再取得。
    ## 旋回途中でも黒検出で追従へ移り、距離/時間上限ではFAILURE停止する。
    #go_to_goal = Hint2Exit('hint2 white exit', context, settings)

    # ==========================================
    # ① 黒線検知 OR 200mm走行
    # ==========================================
    
    black_or_200 = Parallel(
        name="black_or_200",
        policy=ParallelPolicy.SuccessOnOne()
    )
    
    run_to_black_2 = RunByGyro(
        context=context,
        name="run_to_black_2",
        target=115,
        power=50,
        pid_p=1.1,
        pid_i=0.00075,
        pid_d=0.04,
        target_type=HeadingType.ABSOLUTE
    )
    
    detect_black_2 = IsBlackDetected(
        name="detect_black_2",
        black_threshold=settings.to_exit_black_v,
        required_frames=25
    )
    
    black_distance_limit_2 = IsDistanceEarned(
        name="black_distance_limit_2",
        delta_dist=140
    )
    
    black_or_200.add_children([
        run_to_black_2,
        detect_black_2,
        black_distance_limit_2,
    ])

    # ==========================================
    # ①終了後、黒検知の有無に関わらず絶対方位180度へ向き直してから
    # ボトルデリバリーへ進む(ET相撲のガレージ復帰と同じ「検知→既知方位へ
    # 旋回」構造。見つからなかった場合も失敗にはせず、90度で試行を続ける)。
    # ==========================================

    turn_to_180_before_trace = SpinAround(
        context=context,
        name="turn_to_180_before_trace",
        target=180,
        max_power=SPIN_MAX_POWER,
        min_power=SPIN_MIN_POWER,
        pid_p=0.2,
        pid_i=0.005,
        pid_d=0.03,
        target_type=HeadingType.ABSOLUTE
    )

    stop_after_turn_to_180 = StopNow(
        name="stop_after_turn_to_180"
    )

    wait_after_turn_to_180 = IsTimePassed(
        name="wait_after_turn_to_180",
        delta_time=0.2
    )

    # ==========================================
    # ② ①終了後 → ライントレース
    # 【統合差分】独自の650mm上限は削除。終了は外側のdist_1200_from_75deg_start
    # (75度直進の開始位置を基準にした90度成分1200mm)だけに一本化する。
    # ==========================================
    
    l_trace_last = Parallel(
        name="l_trace_last",
        policy=ParallelPolicy.SuccessOnOne()
    )
    
    trace_last = TraceLine(
        name="trace_last",
        target=TRACELINE_TARGET_V,
        power=60,
        pid_p=0.65,
        pid_i=0.000001,
        pid_d=0.045,
        trace_side=TraceSide.NORMAL,
    )

    trace_last_limit = IsDistanceEarned(
        name="trace_last_limit",
        delta_dist=100
    )  
    
    l_trace_last.add_children([
        trace_last,
        trace_last_limit,
    ])

    # ==========================================
    # ③ 順番に実行
    # ==========================================

    line_trace_last = Sequence(
        name="line_trace_last",
        memory=True
    )

    line_trace_last.add_children([
        black_or_200,
        turn_to_180_before_trace,
        stop_after_turn_to_180,
        wait_after_turn_to_180,
        # 【統合差分】QR2用のカメラ切替(~3.4秒)をtrace_lastの走行時間に重ねて隠す。
        # camera_recovery_then_trace_120(コメントアウト済み)にあった
        # PrepareHintCamera("prepare hint2 camera")と同じ配置・目的。
        # これが無いと、read_qr2開始時に初めて切替が始まり、切替コストが
        # そのままQR2読み取りのelapsed時間に乗ってしまう。
        #PrepareHintCamera(name="prepare hint2 camera"),
        l_trace_last,
    ])

    # 75度直進の開始位置(black_or_550が最初にtickされる瞬間)を基準に、
    # 90度成分で1200mm進んだら全体を打ち切る(旧プログラムの1200mm相当)。
    # trace_650自体には独自の終了条件がないため、これが唯一の終了条件になる。
    #dist_1200_from_75deg_start = IsProjectedDistanceEarned(
        #name="dist_1200_from_75deg_start",
        #context=context,
        #local_heading_deg=90.0,
        #delta_dist=settings.to_hint2_trace_mm,
    #)
#
    #line_trace_120_with_cap = Parallel(
        #name="line_trace_120_with_cap",
        #policy=ParallelPolicy.SuccessOnOne()
    #)
    #line_trace_120_with_cap.add_children([
        #line_trace_120,
        #dist_1200_from_75deg_start,
    #])
#
    ## 650mm到達後に停止
    #stop_at_650 = StopNow(
        #name="stop_after_650"
    #)


    # ゴールで停止

    goal_stop = StopNow(
        name="stop_final"
    )

# ========================================================
# 全体の流れ
# ========================================================

    root.add_children([
        # AT colour recognition is finished; TO uses gyro/colour-sensor driving.
        # Prepare QR capture during the approach, then start a fresh read at rest.
        PrepareHintCamera(name="prepare hint1 camera"),
        go_to_qr,
        turn_left_55,
        go_to_black,
        turn_right_125,
        trace_to_qr1,
        stop_at_qr1,
        go_to_blue_after_qr1,
        stop_at_blue,
        turn_left_90_b,
        line_trace_120_with_cap,
        stop_at_650,
        turn_right_qr2,
        qr2_read,
        line_trace_last,

        # 【統合差分】TheEndはalpha.pyの末尾が担当。
    ])

    # 【統合差分】hint2単体モードのみ読取後で終了。通常は元の出口走行まで続ける。
    if include_exit:
        root.add_children([
            goal_stop,
        ])

    # 【統合差分】alphaが所有するツリーへ接続するためrootを返す。
    return root
