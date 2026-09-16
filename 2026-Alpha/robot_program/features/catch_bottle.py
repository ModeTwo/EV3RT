"""AT担当の編集箇所。bottle_catch2.pyのコメントとツリー構造を保持。"""
from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time
from ..behaviours.line_trace import TraceLine
from ..behaviours.gyro_drive import RunByGyro
from ..behaviours.conditions import IsDistanceEarned, IsColorDetected
from ..behaviours.motor_control import StopNow
from ..behaviours.detect_bottle_color import DetectBottleColor
from ..behaviours.handoff import CaptureAtToHandoff


class MarkerZeroDrive(RunByGyro):
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
    タッチスタート
      ↓
    ライントレース + 青ライン検知
      ↓
    青ラインを検知
      ↓
    ライントレース継続
     + ボトル色認識
     + 400MM走行距離監視
      ↓
    400MM到達
      ↓
    停止
    """

    # 【統合差分】元の有効値・担当者報告に合わせ、450表記は400へ統一。
    settings = config.integration
    TRACELINE_TARGET_V = 75

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

                power=45,

                pid_p=0.65,
                pid_i=0.000001,
                pid_d=0.045,

                trace_side=TraceSide.OPPOSITE
            ),

            IsColorDetected(
                name="check blue",
                color=Color.BLUE
            ),
        ]
    )

    # ==========================================
    # ② 青ライン検知後
    #
    # ・ライントレース
    # ・ボトル色認識
    # ・400mm走行距離監視
    #
    # を同時に実行する
    # ==========================================

    trace_detect_bottle_400mm = Parallel(
        name="trace and detect bottle for 400mm",

        # IsDistanceReached がSUCCESSになったら
        # このParallelを終了する
        policy=ParallelPolicy.SuccessOnOne()
    )


    # 青検知地点を起点に293mmは明度を操舵へ使用しない。
    cross_marker = Parallel(name='AT zero heading across blue and gray',
                            policy=ParallelPolicy.SuccessOnOne())
    cross_marker.add_children([
        MarkerZeroDrive(name='AT marker absolute zero', target=0, power=60,
                  pid_p=1.8, pid_i=0.0, pid_d=0.03,
                  target_type=HeadingType.ABSOLUTE),
        IsDistanceEarned(name='AT marker 293mm', delta_dist=settings.at_marker_straight_mm),
    ])
    marker_then_trace = Sequence(name='AT marker straight then line trace', memory=True)
    marker_then_trace.add_children([cross_marker,
        TraceLine(
                name="trace after blue",

                target=TRACELINE_TARGET_V,

                power=60,

                pid_p=0.65,
                pid_i=0.000001,
                pid_d=0.045,

                trace_side=TraceSide.OPPOSITE
            ),
    ])

    trace_detect_bottle_400mm.add_children(
        [
             # --------------------------------------
            # ライントレース
            # --------------------------------------
            marker_then_trace,


            # --------------------------------------
            # 走行しながらボトル色認識
            # --------------------------------------
            DetectBottleColorWhileMoving(
                name="detect bottle while moving",
                context=context,  # 【統合差分】取得色の共有先

                min_area=150,
                min_frames=3
            ),


            # --------------------------------------
            # 青ライン検知地点から400mm進んだか監視
            # --------------------------------------
            IsDistanceReached(
                name="check 400mm after blue",

                distance_mm=settings.at_to_transfer_trace_mm  # 元: 400
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


    # 【統合差分】色未取得は停止後に失敗とし、取得済みだけTOへ渡す。
    root.add_child(RequireBottleColor(context))
    root.add_child(CaptureAtToHandoff("AT_TO capture boundary", context))
    return root
