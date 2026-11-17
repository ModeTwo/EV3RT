#11/13時点の断面です。
#tO_hint_route.py、nonstop_recover.pyが内容同じで現在採用しているコードです。
#いずれもQRコードを走りながら読み取り、読み取れない場合のリカバリまで実装しています。
#走行体ｃに合わせて値を修正しており、日によって都度調整は必要になるかもしれないです。

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
from ..behaviours.conditions import IsDistanceEarned, IsTimePassed, IsColorDetected, IsBlackDetected, IsColorPassed, IsDistanceEarnedUntilColorEntered, IsQRDistanceLimitReached
from ..behaviours.motor_control import StopNow
from ..behaviours.hint_reader import ReadHintCard as IsQRDecoded, PrepareHintCamera

# ========================================================
# QR読み取り開始から4秒経過したかを判定
# ========================================================
class IsQRTimeoutReached(Behaviour):

    def __init__(self, name, qr_reader, timeout_sec=4.0):
        super(IsQRTimeoutReached, self).__init__(name)

        self.qr_reader = qr_reader
        self.timeout_sec = timeout_sec
        self.started_at = None

    def update(self):

        # QR読み取り成功済みならタイムアウトしない
        if self.qr_reader.status == Status.SUCCESS:
            return Status.FAILURE

        # 初回実行時刻を記録
        if self.started_at is None:
            self.started_at = time.monotonic()

        elapsed = time.monotonic() - self.started_at

        # 4秒経過
        if elapsed >= self.timeout_sec:
            return Status.SUCCESS

        return Status.FAILURE


# ========================================================
# QR読み取り成功まで待機
# ========================================================
class WaitForQRDecode(Behaviour):
    """止まってQRが読めるまで待つ。

    2026-11-17(devSS、ユーザー: ヒント2が読めないことは絶対に避けたい):
    カメラはピントが固定(ピントの値を0〜250に変えても画像が同じ、実機で確認)。11/17 15:04 のヒント2は、止まった位置で
    QRがぼけたまま30回以上読めなかった(幅約280画素。読めた回の画像も幅約277画素でぼけ方も近く、距離だけでは決まらない。
    走っている間は毎回読めず、止まってから読めている)。同じ位置で待ち続けても変わらないので、読めないまま待つだけにせず、
    WAIT_SEC 待っても読めなければ、STEPS の順に少しずつ前後に動いて(下がる→もっと下がる→…→元の位置→少し前→元の位置)、
    動くたびに HOLD_SEC 止まって読む。一巡しても読めなければ繰り返す。
    読めたら、元の位置へ戻ってから先へ進む(このあとの「黒線まで前進」は元の位置が前提)。
    (11/17 の前の版はピントを振っていたが効かず、下がる出力30では車輪が回らなかった)
    """

    WAIT_SEC = 1.5
    HOLD_SEC = 1.5
    # 元の位置からの位置(mm、前が+)。この順に動く
    STEPS = (-50.0, -100.0, -150.0, 0.0, 40.0, 0.0)
    MOVE_POWER = 55
    MOVE_TIMEOUT_SEC = 2.5

    def __init__(self, name, qr_reader):
        super(WaitForQRDecode, self).__init__(name)

        self.qr_reader = qr_reader

    def initialise(self):
        self.started_at = None
        self.stage = "wait"
        self.stage_at = None
        self.step = 0
        self.cycle = 0
        self.offset_mm = 0.0          # 元の位置からの今の位置(前が+)
        self.returning = False

    def _stop(self):
        for motor in (runtime.left_motor, runtime.right_motor):
            if motor is not None:
                motor.set_power(0)
                motor.set_brake(True)

    def _counts(self):
        return float(runtime.left_motor.get_count()), float(runtime.right_motor.get_count())

    def _moved_mm(self):
        left, right = self._counts()
        return (abs(left - self.move_origin[0]) + abs(right - self.move_origin[1])) / 2.0 * self.move_mm_per_deg

    def _start_move(self, target_offset, now):
        import math
        dist = target_offset - self.offset_mm
        if abs(dist) < 1.0:
            return False
        self.move_dir = 1.0 if dist > 0 else -1.0
        self.move_target_mm = abs(dist)
        self.move_goal_offset = target_offset
        self.move_origin = self._counts()
        self.move_heading = float(runtime.gyro_sensor.get_angle())
        self.move_mm_per_deg = math.pi * float(runtime.plotter.tire_diameter) / 360.0
        self.stage = "move"
        self.stage_at = now
        self.logger.warning("QR_RECOVER move %s %.0fmm to %+.0fmm from the stop (cycle %d)" % (
            "back" if dist < 0 else "forward", abs(dist), target_offset, self.cycle))
        return True

    def _update_move(self, now):
        moved = self._moved_mm()
        if moved >= self.move_target_mm or now - self.stage_at > self.MOVE_TIMEOUT_SEC:
            self._stop()
            self.offset_mm += self.move_dir * moved
            if moved < self.move_target_mm:
                self.logger.warning("QR_RECOVER move stopped short: %.0f of %.0fmm" % (moved, self.move_target_mm))
            self.stage = "hold"
            self.stage_at = now
            return
        error = (self.move_heading - float(runtime.gyro_sensor.get_angle()) + 180.0) % 360.0 - 180.0
        correction = max(-10.0, min(10.0, 2.0 * error))
        power = self.move_dir * self.MOVE_POWER
        runtime.left_motor.set_brake(False)
        runtime.right_motor.set_brake(False)
        runtime.left_motor.set_power(int(round(power + correction)))
        runtime.right_motor.set_power(int(round(power - correction)))

    def update(self):
        now = time.monotonic()
        if self.started_at is None:
            self.started_at = now
            self.stage_at = now

        if self.returning:
            # 読めたあと、元の位置へ戻っている途中
            if self.stage == "move":
                self._update_move(now)
                return Status.RUNNING
            self.logger.warning("QR_RECOVER back at the stop position (offset %+.0fmm)" % self.offset_mm)
            return Status.SUCCESS

        # QR読み取り成功
        if self.qr_reader.status == Status.SUCCESS:
            if self.stage == "move":
                # 動いている途中で読めた: ここまでの分を数えてから戻る
                moved = self._moved_mm()
                self._stop()
                self.offset_mm += self.move_dir * moved
            if self.stage != "wait" or self.cycle:
                self.logger.warning("QR_RECOVER decoded at %+.0fmm from the stop after %.1fs (cycle %d)" % (
                    self.offset_mm, now - self.started_at, self.cycle))
            if abs(self.offset_mm) > 1.0:
                self.returning = True
                self._start_move(0.0, now)
                return Status.RUNNING
            return Status.SUCCESS

        if self.stage == "wait":
            if now - self.stage_at >= self.WAIT_SEC:
                self.logger.warning("QR_RECOVER not decoded for %.1fs after stopping, moving to read from other distances" % (
                    now - self.started_at))
                self.step = 0
                if not self._start_move(self.STEPS[0], now):
                    self.stage, self.stage_at = "hold", now
            return Status.RUNNING
        if self.stage == "move":
            self._update_move(now)
            return Status.RUNNING
        if self.stage == "hold":
            if now - self.stage_at >= self.HOLD_SEC:
                self.step += 1
                if self.step >= len(self.STEPS):
                    self.step = 0
                    self.cycle += 1
                if not self._start_move(self.STEPS[self.step], now):
                    self.stage_at = now
            return Status.RUNNING

        # 未読み取りなら待機
        return Status.RUNNING

    def terminate(self, new_status):
        if getattr(self, "stage", None) == "move":
            self._stop()


def build_tantou_tree(context, config, include_exit=True):
    settings = config.integration
    # 2026-11-17(devSS、ユーザー): この区間はLAPでつかんだボトルをアームに挟んで押しながら走るので、旋回の出力を下げる
    # (65 では勢いでボトルがアームの間から出ていきそうだった)。止まったときに上げる出力の上限も下げる。
    SPIN_MAX_POWER = settings.delivery_carry_turn_power
    SPIN_MIN_POWER = settings.delivery_carry_turn_power
    STALL_CAP = settings.delivery_carry_turn_stall_cap
    TRACELINE_TARGET_V = 65

    class QRSpeedUpRun(RunByGyro):
        """QR読み取り成功後に走行パワーを上げる"""
        def __init__(self, name, qr_reader,
                        slow_power=55, fast_power=80, **kwargs):
            self.qr_reader = qr_reader
            self.slow_power = slow_power
            self.fast_power = fast_power
            # 初回起動時は55で走行
            kwargs["power"] = slow_power
            super(QRSpeedUpRun, self).__init__(
                name=name,
                **kwargs
            )
        def update(self):
            # QR読み取り状況に応じて速度を切り替える
            if self.qr_reader.status == Status.SUCCESS:
                target_power = self.fast_power
            else:
                target_power = self.slow_power
            # 基本出力を変更
            self.power = target_power
            # PID初期化後なら補正量の上限も変更
            if self.running and hasattr(self, "pid"):
                self.pid.output_limits = (
                    -abs(target_power),
                    abs(target_power)
                )
            return super().update()

    # ここから下は、元のツリー構築処理を続ける
        # TO担当区間の親ノード
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
        power=70,
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


    turn_left_55.add_children([

        SpinAround(
            context=context,  # 【統合差分】AT終了方位を加算せず共通方位を使用
            name="left 55",
            target=40,
            max_power=SPIN_MAX_POWER,
            min_power=SPIN_MIN_POWER,
            stall_power_cap=STALL_CAP,
            pid_p=0.2,
            pid_i=0.005,
            pid_d=0.03,
            target_type=HeadingType.ABSOLUTE
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
    
    # ========================================================
    # 1. 黒線方向への通常走行
    # ========================================================
    
    #go_to_black_a = Parallel(
        #name="go_to_black_a",
        #policy=ParallelPolicy.SuccessOnOne()
    #)
#    
    #go_to_black_a.add_children([
#    
        #RunByGyro(
            #context=context,
            #name="run_to_black_a",
            #target=45,
            #power=55,
            #pid_p=1.1,
            #pid_i=0.00075,
            #pid_d=0.04,
            #target_type=HeadingType.ABSOLUTE
        #),
#    
        ## IsColorDetected(
        ##     name="detect_black",
        ##     color=Color.BLACK
        ## ),
#    
        #IsDistanceEarned(
            #name="black_distance_limit_a",
            #delta_dist=640  # tantou4: 550
        #),
#    
    #])

#    go_to_black_a = Parallel(
        #name="go_to_black_a",
        #policy=ParallelPolicy.SuccessOnOne()
    #)
#    
    #go_to_black_a.add_children([
#    
        #QRSpeedUpRun(
            #context=context,
            #name="run_to_black_a",
            #qr_reader=read_qr1,
            #target=45,
            #slow_power=55,
            #fast_power=80,
            #pid_p=1.1,
            #pid_i=0.00075,
            #pid_d=0.04,
            #target_type=HeadingType.ABSOLUTE
        #),
#    
        #IsDistanceEarned(
            #name="black_distance_limit_a",
            #delta_dist=640
        #),
#    
    #])

    #go_to_black_a2 = Parallel(
        #name="go_to_black_a2",
        #policy=ParallelPolicy.SuccessOnOne()
    #)
#    
    #go_to_black_a2.add_children([
#    
        #RunByGyro(
            #context=context,
            #name="run_to_black_a2",
            #target=40,
            #power=50,
            #pid_p=1.1,
            #pid_i=0.00075,
            #pid_d=0.04,
            #target_type=HeadingType.ABSOLUTE
        #),
#    
        ## IsColorDetected(
        ##     name="detect_black",
        ##     color=Color.BLACK
        ## ),
#    
        #IsDistanceEarned(
            #name="black_distance_limit_a2",
            #delta_dist=460  # tantou4: 550
        #),
#    
    #])

    #go_to_black_aa = Sequence(
        #name="go_to_black_aa",
        #memory=True
    #)
#    
    #go_to_black_aa.add_children([
#    
        #go_to_black_a,
        ##go_to_black_a2,
#    
    #])
    
    
    # ========================================================
    # 2. QR1読み取り
    #
    # 通常走行と並行してQRコードを監視する
    # ========================================================
    
    read_qr1 = IsQRDecoded(
        name="read_qr1",
        context=context,
        hint_number=1,
        keep_qr_ready=True
    )
    
    trace_to_qr1 = Parallel(
        name="trace_to_qr1",
        policy=ParallelPolicy.SuccessOnOne()
    )
    
    trace_to_qr1.add_children([
        read_qr1,
    ])

    go_to_black_a = Parallel(
        name="go_to_black_a",
        policy=ParallelPolicy.SuccessOnOne()
    )
    
    go_to_black_a.add_children([
    
        QRSpeedUpRun(
            context=context,
            name="run_to_black_a",
            qr_reader=read_qr1,
            target=40,
            slow_power=50,
            fast_power=70,
            pid_p=1.1,
            pid_i=0.00075,
            pid_d=0.04,
            target_type=HeadingType.ABSOLUTE,
            distance_mm=760,
            decel_mm=50,
            decel_min_power=20
        ),
    
        IsDistanceEarned(
            name="black_distance_limit_a",
            delta_dist=760
        ),
    
    ])

    go_to_black_aa = Sequence(
        name="go_to_black_aa",
        memory=True
    )
    
    go_to_black_aa.add_children([
    
        go_to_black_a,
        #go_to_black_a2,
    
    ])
    
    
    # ========================================================
    # 3. QR読み取り成功後に200mm前進
    #
    # 方位45度を維持して200mm走行する
    # ========================================================
    
    go_forward_200mm = Parallel(
        name="go_forward_200mm",
        policy=ParallelPolicy.SuccessOnOne()
    )
    
    go_forward_200mm.add_children([
    
        RunByGyro(
            context=context,
            name="run_after_qr_200mm",
            target=40,
            power=70,
            pid_p=1.1,
            pid_i=0.00075,
            pid_d=0.04,
            target_type=HeadingType.ABSOLUTE,
            distance_mm=250,
            decel_mm=50,
            decel_min_power=20
        ),
    
        IsDistanceEarned(
            name="distance_after_qr_200mm",
            delta_dist=250
        ),

    ])
    
    
    # ========================================================
    # 4. 4秒経過後、QR未読み取りなら一時停止
    #
    # QR読み取り成功後は200mm前進する
    # ========================================================
    
    pause_after_qr_timeout = Sequence(
        name="pause_after_qr_timeout",
        memory=True
    )
    
    pause_after_qr_timeout.add_children([
    
        # 4秒経過かつQR未読み取りの場合にSUCCESS
        #IsQRTimeoutReached(
            #name="qr_timeout_4sec",
            #qr_reader=read_qr1,
            #timeout_sec=4.0
        #),

        IsQRDistanceLimitReached(
            name="qr_distance_500mm",
            qr_reader=read_qr1,
            delta_dist=500
        ),
    
    
        # 一時停止
        StopNow(
            name="stop_waiting_for_qr"
        ),
    
        # QR読み取り成功まで待機
        WaitForQRDecode(
            name="wait_until_qr_decoded",
            qr_reader=read_qr1
        ),
    
        # QR読み取り成功後、200mm前進
        go_forward_200mm,
    
    ])
    
    
    # ========================================================
    # 5. Selectorによる走行切り替え
    #
    # 優先順位
    # ① 4秒経過後、QR未読み取りなら一時停止処理
    # ② それ以外は通常走行
    # ========================================================
    
    drive_selector = Selector(
        name="drive_selector",
        memory=False
    )
    
    drive_selector.add_children([
    
        pause_after_qr_timeout,
        go_to_black_aa,
    
    ])
    
    
    # ========================================================
    # 6. QR読み取りと走行を並行実行
    # ========================================================
    
    go_to_black_a_with_qr = Parallel(
        name="go_to_black_a_with_qr",
        policy=ParallelPolicy.SuccessOnAll()
    )
    
    go_to_black_a_with_qr.add_children([
    
        # QR読み取り監視
        trace_to_qr1,
    
        # 通常走行・一時停止・200mm前進の切り替え
        drive_selector,
    
    ])
        
    #go_to_black_a = Parallel(
        #name="go_to_black_a",
        #policy=ParallelPolicy.SuccessOnOne()
    #)    
    #go_to_black_a.add_children([
        #RunByGyro(
            #context=context,  # 【統合差分】AT終了方位を加算せず共通方位を使用
            #name="run_to_black_a",
            #target=45,
            #power=55,
            #pid_p=1.1,
            #pid_i=0.00075,
            #pid_d=0.04,
            #target_type=HeadingType.ABSOLUTE
        #),
#
        ##IsColorDetected(
         ##   name="detect_black",
         ##   color=Color.BLACK
        ##),
#
        #IsDistanceEarned(
            #name="black_distance_limit_a",
            #delta_dist=760# tantou4: 550
        #),
       ## ResetDevice(name="device_reset"),
    #])
#
    ## ========================================================
    ## 3. 黒線 → QR1
    ##
    ## ライントレースしながらQRコードを監視する。
    ##
    ## QRコードがデコードできたら終了。
    ## ========================================================
#
    #trace_to_qr1 = Parallel(
        #name="trace_to_qr1",
        #policy=ParallelPolicy.SuccessOnOne()
    #)
#
    #trace_to_qr1.add_children([
#
        #IsQRDecoded(
            ## 【統合差分】QR1成功直後にカメラをLINEモードへ戻し、直進・旋回中に
            ## カメラ切替(約3.4秒)を隠す。QR2向けの再準備は後段のカメラ復帰後に行う。
            #name="read_qr1", context=context, hint_number=1, keep_qr_ready=False
        #),
    #])
#
    #go_to_black_a_with_qr = Parallel(
        #name="go_to_black_a_with_qr",
        #policy=ParallelPolicy.SuccessOnAll()
    #)
#
    #go_to_black_a_with_qr.add_children([
#
        #go_to_black_a,
        #trace_to_qr1,
#
    #])

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
        stall_power_cap=STALL_CAP,
        pid_p=0.2,
        pid_i=0.005,
        pid_d=0.03,
        target_type=HeadingType.ABSOLUTE
    )
    
    #go_to_black_90_9 = Parallel(
        #name="go_to_black_90_9",
        #policy=ParallelPolicy.SuccessOnOne()
    #)    
    #go_to_black_90_9.add_children([
        #TraceLine(
            #name="run_to_black_90_9",
            #target=TRACELINE_TARGET_V,
            #power=80,
            #pid_p=0.65,
            #pid_i=0.000001,
            #pid_d=0.045,
            #trace_side=TraceSide.NORMAL,
        #),
#
        ##IsColorDetected(
         ##   name="detect_black",
         ##   color=Color.BLACK
        ##),
#
        #IsDistanceEarned(
            #name="black_distance_limit_4",
            #delta_dist=100  # tantou4: 550
        #),
       ## ResetDevice(name="device_reset"),
    #])

    # ==========================================
    # ② ①終了後 → ライントレース
    # 【統合差分】独自の650mm上限は削除。終了は外側のdist_1200_from_75deg_start
    # (75度直進の開始位置を基準にした90度成分1200mm)だけに一本化する。
    # ==========================================

    #trace_650 = TraceLine(
        #name="trace_650",
        #target=TRACELINE_TARGET_V,
        #power=100,
        #pid_p=0.65,
        #pid_i=0.000001,
        #pid_d=0.045,
        #trace_side=TraceSide.NORMAL,
    #)

    trace_650 = RunByGyro(
        context=context,
        name="trace_650",
        target=90,
        power=100,
        pid_p=1.1,
        pid_i=0.00075,
        pid_d=0.04,
        target_type=HeadingType.ABSOLUTE,
        distance_mm=950,
        decel_mm=100,
        decel_min_power=40
    )


    # ==========================================
    # ③ 順番に実行
    # ==========================================

    line_trace_120 = Sequence(
        name="line_trace_120",
        memory=True
    )

    line_trace_120.add_children([
        #non_black_run,
        #black_or_550,
        turn_to_90_before_trace,
        #go_to_black_90_9,
        #stop_after_turn_to_90,
        #wait_after_turn_to_90,
        #to_line,
        #go_to_black_90_5,
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
    #stop_at_650 = StopNow(
        #name="stop_after_650"
    #)

    #go_to_black_90_4 = Parallel(
        #name="go_to_black_90_4",
        #policy=ParallelPolicy.SuccessOnOne()
    #)    
    #go_to_black_90_4.add_children([
        #TraceLine(
            #name="run_to_black_90_4",
            #target=TRACELINE_TARGET_V,
            #power=70,
            #pid_p=0.65,
            #pid_i=0.000001,
            #pid_d=0.045,
            #trace_side=TraceSide.NORMAL,
        #),
#
        ##IsColorDetected(
         ##   name="detect_black",
         ##   color=Color.BLACK
        ##),
#
        #IsDistanceEarned(
            #name="black_distance_limit_4",
            #delta_dist=100  # tantou4: 550
        #),
       ## ResetDevice(name="device_reset"),
    #])

    # ========================================================
    # 7. 1200mm地点 → 右30°
    # ========================================================

    turn_right_qr2 = Sequence(
        name="turn_right_qr2",
        memory=True
    )

    turn_right_qr2.add_children([
        #StopNow(
            #name="stop_before_qr2"
        #),

        #ResetDevice(name="device_reset"),
        SpinAround(
            context=context,  # 【統合差分】AT終了方位を加算せず共通方位を使用
            name="right 25 for qr2",
            target=105,  # 【統合差分】添付25。共通方位の既存値を維持
            max_power=SPIN_MAX_POWER,
            min_power=SPIN_MIN_POWER,
            stall_power_cap=STALL_CAP,
            pid_p=0.2,
            pid_i=0.005,
            pid_d=0.03,
            target_type=HeadingType.ABSOLUTE
        ),

        #StopNow(
            #name="stop_after_qr2_turn"
        #),
#
        #IsTimePassed(
            #name="wait_after_qr2_turn",
            #delta_time=0.2
        #),
    ])


    # ========================================================
    # 8. QR2読み取り
    #
    # 絶対方位0°を向いてQR2を読む。
    # ========================================================

    #qr2_read = Sequence(
        #name="qr2_read",
        #memory=True
    #)
#
    #qr2_read.add_children([
#
        #IsQRDecoded(
            #name="read_qr2", context=context, hint_number=2
        #),
#
        ##StopNow(
            ##name="stop_after_qr2"
        ##),
    #])

    read_qr2 = IsQRDecoded(
        name="read_qr2",
        context=context,
        hint_number=2
    )

    qr2_read = Sequence(
        name="qr2_read",
        memory=True
    )

    qr2_read.add_children([
        read_qr2,
    ])


    black_or_200 = Sequence(
        name="black_or_200",
        memory=True
    )
    
    run_to_black_2_01 = Parallel(
        name="run_to_black_2_01",
        policy=ParallelPolicy.SuccessOnOne()
    )

    run_to_black_2_01.add_children([

    #RunByGyro(
        #context=context,
        #name="run_to_black_2_01",
        #target=105,
        #power=55,
        #pid_p=1.1,
        #pid_i=0.00075,
        #pid_d=0.04,
        #target_type=HeadingType.ABSOLUTE
    #),

    QRSpeedUpRun(
        context=context,
        name="run_to_black_2_01",
        qr_reader=read_qr2,
        target=105,
        slow_power=50,
        fast_power=75,
        pid_p=1.1,
        pid_i=0.00075,
        pid_d=0.04,
        target_type=HeadingType.ABSOLUTE
    ),
    
    IsDistanceEarned(
        name="run_to_black_limit",
        # 2026-11-17(devSS、実機): 400→200。黒線を見るのはこの距離のあとからなので、ロボットの位置が回によって
        # ずれて黒線が400mmより手前に来ると、見ずに通り過ぎた(ユーザー: 黒線に対して行き過ぎ)。
        # 黒線までは走れた回で約515mm、遠かった回で746mm(位置の振れ ±200mm くらい)。始めの200mmは見ない
        # (旋回した場所の近くの線を拾わないため)
        delta_dist=200  # tantou4: 550
    ),
    ])

    run_to_black_2 = Parallel(
        name="run_to_black_2",
        policy=ParallelPolicy.SuccessOnOne()
    )

    run_to_black_2.add_children([

    #RunByGyro(
        #context=context,
        #name="run_to_black_2",
        #target=105,
        #power=55,
        #pid_p=1.1,
        #pid_i=0.00075,
        #pid_d=0.04,
        #target_type=HeadingType.ABSOLUTE
    #),

    QRSpeedUpRun(
        context=context,
        name="run_to_black_2",
        qr_reader=read_qr2,
        target=105,
        slow_power=50,
        # 2026-11-17(devSS): 黒線を見ながら走る区間を 200→400mm に延ばした(上の run_to_black_limit を 400→200 にした分)。
        # 出力 80→65: 黒線の上を通る間にセンサーを読む回数を増やす(2回続けて黒で判定)
        fast_power=65,
        pid_p=1.1,
        pid_i=0.00075,
        pid_d=0.04,
        target_type=HeadingType.ABSOLUTE,
        distance_mm=400,
        decel_mm=50,
        # 2026-11-17(devSS、実機): 20 だと200mmを過ぎたあと車輪が回らず(出力20で止まる)、黒線に届かないまま
        # 動かなくなった(ヒント2のQRは読めたあと、黒線が想定より約230mm先だった回)。200mmを過ぎても黒線まで
        # ゆっくり進み続けられる出力にする(出力30でも車輪が回らなかったことがある)
        decel_min_power=40
    ),
    
    IsBlackDetected(
        name="detect_black_2",
        black_threshold=settings.to_exit_black_v,
        required_frames=2
    ),

    # 2026-11-17(devSS): それでも黒線が見つからないときに止まったままにならないよう、上限を置いて先へ進む
    # (ふだんはこの区間の約110mmで黒線。11/17 の外れた回は約340mm)
    IsDistanceEarned(
        name="black_2_give_up",
        delta_dist=600
    ),
    ])
    

    # 黒線検知後に必ず停止する
    black_detect_and_stop_2 = Sequence(
        name="black_detect_and_stop_2",
        memory=True
    )
    
    black_detect_and_stop_2.add_children([
        run_to_black_2,
        StopNow(
            name="stop_after_black_detected_2"
        ),
    ])


    black_or_200.add_children([
        run_to_black_2_01,
        black_detect_and_stop_2,
        # black_distance_limit_2,
    ])


    go_forward_100mm = Parallel(
        name="go_forward_100mm",
        policy=ParallelPolicy.SuccessOnOne()
    )
    
    go_forward_100mm.add_children([
    
        RunByGyro(
            context=context,
            name="run_after_qr_100mm",
            target=105,
            power=80,
            pid_p=1.1,
            pid_i=0.00075,
            pid_d=0.04,
            target_type=HeadingType.ABSOLUTE,
            distance_mm=200,
            decel_mm=50,
            # 2026-11-17(devSS): 20 だと200mmを過ぎて黒線がないとき車輪が回らず止まったままになる(run_to_black_2 と同じ)
            decel_min_power=40
        ),
    
        IsBlackDetected(
            name="detect_black_3",
            black_threshold=settings.to_exit_black_v,
            required_frames=2
        ),

        # 2026-11-17(devSS): 黒線が見つからなくても止まったままにしない上限
        IsDistanceEarned(
            name="black_3_give_up",
            delta_dist=600
        ),

        #IsDistanceEarned(
            #name="detect_black_3",
            #delta_dist=30  # tantou4: 550
        #),
    
    ])

    pause_after_qr_timeout_2 = Sequence(
        name="pause_after_qr_timeout_2",
        memory=True
    )
    
    pause_after_qr_timeout_2.add_children([
    
        # 4秒経過かつQR未読み取りの場合にSUCCESS
        #IsQRTimeoutReached(
            #name="qr_timeout_4sec_2",
            #qr_reader=read_qr2,
            #timeout_sec=4.0
        #),

        IsQRDistanceLimitReached(
            name="qr_distance_500mm_2",
            qr_reader=read_qr2,
            delta_dist=400
        ),
    
        # 一時停止
        StopNow(
            name="stop_waiting_for_qr_2"
        ),
    
        # QR読み取り成功まで待機
        WaitForQRDecode(
            name="wait_until_qr_decoded_2",
            qr_reader=read_qr2
        ),
    
        # QR読み取り成功後、200mm前進
        go_forward_100mm,
    
    ])

    drive_selector_2 = Selector(
        name="drive_selector_2",
        memory=False
    )
    
    drive_selector_2.add_children([
    
        pause_after_qr_timeout_2,
        black_or_200,
    
    ])
    
    # black_distance_limit_2 = IsDistanceEarned(
    #     name="black_distance_limit_2",
    #     delta_dist=750
    # )
    
    #black_or_200.add_children([
        #run_to_black_2,
        #detect_black_2,
        ## black_distance_limit_2,
    #])

    go_to_black_a_with_qr_2 = Parallel(
        name="go_to_black_a_with_qr_2",
        policy=ParallelPolicy.SuccessOnAll()
    )

    # 2026-11-17(devSS): 黒線を早めに(200mmから)見るようにしたので、QRがまだ読めないうちに黒線で止まることがある。
    # そのときは止まったまま待つだけにせず、WaitForQRDecode(少しずつ前後に動いて読み直し、読めたら止まった位置へ戻る)に入る。
    # すでに読めていれば WaitForQRDecode はすぐ SUCCESS を返す。
    drive_then_wait_qr_2 = Sequence(
        name="drive_then_wait_qr_2",
        memory=True
    )
    drive_then_wait_qr_2.add_children([
        drive_selector_2,
        WaitForQRDecode(
            name="wait_qr_after_black_2",
            qr_reader=read_qr2
        ),
    ])

    go_to_black_a_with_qr_2.add_children([

        qr2_read,
        #black_or_50,
        drive_then_wait_qr_2,

    ])

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
        go_to_black_a_with_qr,
        #stop_at_black,
        #non_black_45,
        #go_to_black_45,
        #turn_left_90,
        #go_to_black_90,
        #go_to_black_90_2,
        #turn_right_125,
        #turn_right_125_2,
        #turn_right_125_3,
        #trace_to_qr1,
        #stop_at_qr1,
        #go_to_blue_after_qr1,
        #stop_at_blue,
        #go_to_black_90_3,
        #turn_left_90_b,
        line_trace_120_with_cap,
        #stop_at_650,
        #go_to_black_90_4,
        turn_right_qr2,
        #qr2_read,
        #line_trace_last,
        go_to_black_a_with_qr_2,

        # 【統合差分】TheEndはalpha.pyの末尾が担当。
    ])

    # 【統合差分】hint2単体モードのみ読取後で終了。通常は元の出口走行まで続ける。
    if include_exit:
        root.add_children([
            goal_stop,
        ])

    # 【統合差分】alphaが所有するツリーへ接続するためrootを返す。
    return root
