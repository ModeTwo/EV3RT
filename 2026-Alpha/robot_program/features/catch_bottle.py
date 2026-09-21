"""AT担当の編集箇所。bottle_catch2.pyのコメントとツリー構造を保持。"""
from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time
from ..behaviours.line_trace import TraceLine
from ..behaviours.corrected_run import BrakeReleasingEtRun
from ..behaviours.conditions import IsDistanceEarned, IsColorDetected
from ..behaviours.motor_control import StopNow
from ..behaviours.detect_bottle_color import DetectBottleColor
from ..behaviours.handoff import CaptureAtToHandoff


class MarkerZeroDrive(BrakeReleasingEtRun):
    # 単体開始時や直前工程の停止ブレーキを解除して走行する。
    def update(self):
        runtime.require('left_motor', 'right_motor')
        runtime.left_motor.set_brake(False)
        runtime.right_motor.set_brake(False)
        return super().update()


class RequireBottleColor(Behaviour):
    def __init__(self, context):
        super().__init__('AT require recognised bottle color')
        self.context = context

    def update(self):
        if self.context.bottle_color in (
                BottleColor.RED.value, BottleColor.BLUE.value, BottleColor.YELLOW.value):
            return Status.SUCCESS
        self.logger.error('AT stopped at distance limit without a recognised bottle color')
        return Status.FAILURE


class IsDistanceReached(IsDistanceEarned):
    # 【統合差分】元コードの呼出し名を共有距離計測へ接続する。
    def __init__(self, name, distance_mm):
        super().__init__(name=name, delta_dist=distance_mm)


class DetectBottleColorWhileMoving(DetectBottleColor):
    # 【統合差分】元コードの名前を保持し、新規フレーム判定と共有状態へ接続する。
    def __init__(self, name, context, min_area=150, min_frames=3):
        super().__init__(name=name, context=context, min_area=min_area,
                         min_frames=min_frames, while_moving=True)


def build_catch_bottle(context, config):
    """
    単体実行（mission_mode == "at"）
        青ラインまでライントレース
            ↓
        青ライン検知
            ↓
        ライントレース継続
          + ボトル色認識
          + 400mm走行距離監視
            ↓
        400mm到達
            ↓
        停止
            ↓
        ボトル色確認
            ↓
        TOへ引き渡し

    統合実行
        RE側ですでに青ライン検知済み
            ↓
        ライントレース
          + ボトル色認識
          + 400mm走行距離監視
            ↓
        400mm到達
            ↓
        停止
            ↓
        ボトル色確認
            ↓
        TOへ引き渡し
    """

    # 設定
    settings = config.integration
    TRACELINE_TARGET_V = 75

    # ルートSequence
    root = Sequence(
        name="blue and bottle test",
        memory=True
    )


    # ==========================================
    # 青色を検知するまでライントレース
    # ==========================================

    trace_until_blue = Parallel(
        name="trace until blue",
        policy=ParallelPolicy.SuccessOnOne()
    )

    trace_until_blue.add_children(
        [
            TraceLine(
                name="trace before blue",

                target=TRACELINE_TARGET_V,

                power=75,
                power_min=50,

                err_lo=6.0,
                err_hi=22.0,

                accel_per_s=80.0,
                decel_per_s=180.0,

                pid_p=0.50,
                pid_i=0.000001,
                pid_d=0.045,

                trace_side=TraceSide.OPPOSITE
            ),

            # 青ライン検知
            IsColorDetected(
                name="check blue",
                color=Color.BLUE
            ),
        ]
    )


    # ==========================================
    # 青ライン検知後
    #
    # ・ジャイロで400mm直進
    # ・走行しながらボトル色認識
    #
    # 400mm到達まで並行実行する
    # ==========================================

    trace_detect_bottle_400mm = Parallel(
        name="gyro drive and detect bottle for 400mm",
        policy=ParallelPolicy.SuccessOnOne()
    )

    trace_detect_bottle_400mm.add_children(
        [
            # --------------------------------------
            # 青ライン検知後はライントレースせず、
            # ジャイロを使って絶対0度方向へ直進
            # --------------------------------------
            MarkerZeroDrive(
                name="AT gyro straight after blue",
                target=0,
                power=60,
                pid_p=1.2,
                pid_i=0.0,
                pid_d=0.03,
                target_type=HeadingType.ABSOLUTE
            ),

            # --------------------------------------
            # 走行しながらボトル色認識
            # --------------------------------------
            DetectBottleColorWhileMoving(
                name="detect bottle while moving",
                context=context,
                min_area=150,
                min_frames=3
            ),

            # --------------------------------------
            # 青ライン検知地点から400mm進んだら終了
            # --------------------------------------
            IsDistanceReached(
                name="check 400mm after blue",
                distance_mm=settings.at_to_transfer_trace_mm
            ),
        ]
    )


    # ==========================================
    # Behaviour Tree
    # ==========================================

    # 【統合差分】単体のみ青まで走る。統合ではREが青検知済み。
    if config.mission_mode == "at":
        root.add_child(trace_until_blue)

    root.add_children(
        [

            # ① タッチスタート
            # 【統合差分】IsTouchOnはalpha.pyで実行済み。


            # ② 青ラインまでライントレース
            # 【統合差分】単体時のみ上でtrace_until_blueを接続。


            # ③ 青ライン検知地点から
            #    ボトル認識しながら400mmライントレース
            trace_detect_bottle_400mm,


            # ④ 400mm進んだら停止
            StopNow(
                name="final stop"
            ),

        ]
    )


    # ボトル色が正常に取得できているか確認
    root.add_child(RequireBottleColor(context))
    # ボトル色が正常に取得できているか確認
    root.add_child(CaptureAtToHandoff("AT_TO capture boundary", context))
    return root
