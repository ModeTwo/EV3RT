"""AT担当の編集箇所。bottle_catch.pyのbuild_behaviour_treeの見た目を保持。"""
from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time
from ..behaviours.section_motion import DriveDistance
from ..behaviours.line_trace import TraceLine
from ..behaviours.conditions import IsDistanceEarned, IsColorDetected
from ..behaviours.motor_control import StopNow
from ..behaviours.detect_bottle_color import DetectBottleColor
from ..behaviours.handoff import CaptureAtToHandoff


class IsDistanceReached(IsDistanceEarned):
    # AT元コードの呼出し名・引数名を保ち、共有距離計測へ接続する。
    def __init__(self, name, distance_mm):
        super().__init__(name=name, delta_dist=distance_mm)


def build_catch_bottle(context, config):
    settings = config.integration
    TRACELINE_TARGET_V = 75


    """
    タッチ待ち
      ↓
    ライントレース + 青色検知
      ↓
    青色を検知
      ↓
    10cm前進
      ↓
    10cm後退
      ↓
    停止
      ↓
    カメラでボトル色を認識
      ↓
    ボトル色を保存
      ↓
    ライントレース再開
    """

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

                power=60,

                pid_p=0.65,
                pid_i=0.000001,
                pid_d=0.045,

                trace_side=TraceSide.NORMAL
            ),

            IsColorDetected(
                name="check blue",
                color=Color.BLUE
            ),
        ]
    )

    # ==========================================
    # ボトル色認識後、46cmライントレース
    # ==========================================

    trace_after_bottle_46cm = Parallel(
        name="trace 46cm after bottle",
        policy=ParallelPolicy.SuccessOnOne()
    )

    trace_after_bottle_46cm.add_children(
        [
            # ライントレース
            TraceLine(
                name="trace after bottle",

                target=TRACELINE_TARGET_V,

                power=60,

                pid_p=0.65,
                pid_i=0.000001,
                pid_d=0.045,

                trace_side=TraceSide.NORMAL
            ),

            # 46cm進んだか確認
            IsDistanceReached(
                name="check 46cm",
                distance_mm=settings.at_to_transfer_trace_mm  # 元: 460
            ),
        ]
    )

    # ==========================================
    # Behaviour Tree
    # ==========================================

    # 【統合差分】AT単体は青線まで走る。統合走行ではREが検知済み。
    if config.mission_mode == 'at':
        root.add_child(trace_until_blue)

    root.add_children(
        [

            # ① タッチを待つ
            # 【統合差分】IsTouchOnはalpha.pyで実行済み。


            # ② 青色までライントレース
            # 【統合差分】単体時のみ上でtrace_until_blueを先頭へ接続。


            # ③ 青色検知後、10cm前進
            DriveDistance(
                name="forward 10cm",
                distance_mm=settings.at_gate_forward_mm,  # 元: 100
                power=60
            ),


            # ④ 10cm後退
            # 【注記】元の名前・コメントを保持。実距離は元コード同様200mm。
            DriveDistance(
                name="backward 10cm",
                distance_mm=settings.at_recognition_reverse_mm,  # 元: 200
                power=-60
            ),


            # ⑤ 停止
            StopNow(
                name="stop before bottle detection"
            ),


            # ⑥ ボトル色認識
            DetectBottleColor(
                name="detect bottle color",
                context=context,  # 【統合差分】取得色の共有先

                min_area=150,
                min_frames=3
            ),


            # ⑦ ボトル色認識後、
            #    ライントレースしながら46cm走行
            trace_after_bottle_46cm,


            # ⑧ 46cm前進したら停止
            StopNow(
                name="final stop"
            ),

        ]
    )

    # 【統合差分】AT終了位置・方位をTOへ引き継ぐ。
    root.add_child(CaptureAtToHandoff("AT_TO capture boundary", context))
    return root
