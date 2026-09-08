"""Feature 12 subtree factory."""

import json
from pathlib import Path
from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time
from ..placeholder import PendingFeature
from ..behaviours.gyro_drive import RunByGyro, SpinAround
from ..behaviours.conditions import IsDistanceEarned

SPIN_MAX_POWER = 57         # その場回旋（スピン）するときの最大モーター出力
SPIN_MIN_POWER = 47         # その場回旋（スピン）するときの最低モーター出力

# TODO: receive_strategy(feature11)が実装され次第、context.strategyから読み込む形に置き換える。
# それまでの暫定対応として、実行時のカレントディレクトリに依存しないよう
# このファイルの場所を基準にした絶対パスで固定シードのplan.jsonを読む。
DEFAULT_PLAN_PATH = Path(__file__).resolve().parents[1] / "tests" / "plan_seed9392783.json"

def build_execute_strategy(context, config, lap_number):
    # No.12 受信した走行指令に従う走行を担当する。
    root = Sequence(name=f"execute_strategy_lap{lap_number}", memory=True)
    root.add_children(steps_from_plan(DEFAULT_PLAN_PATH))
    return root

def steps_from_plan(plan_path: str, move_power: int = 50, move_pid=(1.1, 0.00075, 0.04),
                     turn_max_power: int = SPIN_MAX_POWER, turn_min_power: int = SPIN_MIN_POWER,
                     turn_pid=(0.2, 0.00075, 0.03)) -> list:
    """ETラリーの経路計画(et_rally_planner/export_plan.pyが書き出すplan.json)を読み込み、
    RunByGyro/IsDistanceEarned/SpinAroundを組み合わせたビヘイビアツリーの部品リストに変換する。

    plan.json側の各stepは絶対方位(target_type=ABSOLUTE)で、経路のスタート時点を0度とした
    シフト済みの値。実機ではジャイロをリセットした瞬間に向いている方角が0度に対応する
    (ResetDeviceの直後にスタートする前提)。旋回の増加方向(右回り/左回り)が実機のジャイロ
    配線と一致しているかは、最初の数ステップだけ動かして必ず確認すること。逆だった場合は
    plan.json側の全target_heading_degの符号を反転すれば直る。

    move_power/turn_max_power等は区間ごとに変えず一律のデフォルト値を使っている。
    区間ごとにパワーやPIDを変えたくなったら、plan.json側にフィールドを足して
    ここで読み取るよう拡張すればよい。
    """
    with open(plan_path, encoding="utf-8") as f:
        plan = json.load(f)

    nodes = []
    for i, step in enumerate(plan["steps"]):
        if step["type"] == "move":
            leg = Parallel(name="et_rally move%d %s" % (i, step["label"]), policy=ParallelPolicy.SuccessOnOne())
            leg.add_children(
                [
                    RunByGyro(name="et_rally run%d" % i, target=step["target_heading_deg"], power=move_power,
                        pid_p=move_pid[0], pid_i=move_pid[1], pid_d=move_pid[2],
                        target_type=HeadingType.ABSOLUTE),
                    IsDistanceEarned(name="et_rally dist%d" % i, delta_dist=int(step["distance_mm"])),
                ]
            )
            nodes.append(leg)
        else:  # "turn"
            nodes.append(
                SpinAround(name="et_rally turn%d" % i, target=step["target_heading_deg"],
                    max_power=turn_max_power, min_power=turn_min_power,
                    pid_p=turn_pid[0], pid_i=turn_pid[1], pid_d=turn_pid[2],
                    target_type=HeadingType.ABSOLUTE)
            )
    return nodes

