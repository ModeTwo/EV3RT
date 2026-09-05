"""Feature 03 subtree factory."""

from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time
from ..placeholder import PendingFeature




def build_catch_bottle(context, config):
    # No.3 LAPゲート通過、ボトル色認識、ボトル取得を担当する。
    root = Sequence(name="catch_bottle", memory=True)
    catch = Parallel(name="catch bottle", policy=ParallelPolicy.SuccessOnOne())

    #ボトルキャッチ
    # catch.add_children(
    #     [
    #         CatchBottle(name="run toward and catch bottle", power=33,
    #                     pid_p=1.1, pid_i=0.1, pid_d=0.03, catch_run_mm = 150),
    #     ]
    # )


    root.add_children(
        [   
            #catch,
            PendingFeature(name="catch_bottle_pending")
        ]
    )
    return root
