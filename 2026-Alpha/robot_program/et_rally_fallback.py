"""ETラリー開始時にエラーが出たときの固定ルート(ゴール=ガレージへ直行する)。

PCからの経路が得られない(受信タイムアウト、PC側のエラー、Hintが揃っていない、受け取った経路を
変換できない)とき、ETラリーを走らずに、ゲートを避けた固定ルートでゴールへ向かう。

    スタート S → A(54,55のグレーポイントの中点) → B → C → ゴール G
    A→B: 行3と行4の間(y=86.1)を、14,15の中点を通って、ゴール側へExit距離だけ先(B)まで直進
    B→C: 11のグレーポイントのY座標(y=0)まで、まっすぐ下がる
    C→G: ゴールへ直進(ゴール側は進入禁止エリアの外で、支柱もない)

座標は、経路計算(wireless_device/et_rally_planner/config.py)と同じ、左コース基準のcm座標。
値を変えたときは、tests/test_et_rally_fallback.py が、planner側の値との食い違いを検出する。

方位の枠: 受信した経路(PCがplanner_to_full_startで変換済み)と同じ「コース正規化した方位」で返す。
右コースは、planner側が幾何を鏡映したうえで方位の符号も反転するため、コース正規化後の方位は、
左右とも同じになる。そのため、常に左コースとして変換すればよい(コースで分岐しない)。
"""

import math

from shared_communication.heading_frame import planner_to_full_start

# --- planner/config.py と同じ式で定義する ---
GRID_PITCH_CM = 24.6
START_OFFSET_CM = 18.0
START_SHIFT_CM = (0.5, -2.0)
GOAL_OFFSET_CM = 14.8
GOAL_SHIFT_CM = (-61.0, -115.0)
GATE_EXIT_OFFSET_CM = 22.0
START_HEADING_DEG = 180.0

START_POS_CM = (4 * GRID_PITCH_CM + START_OFFSET_CM + START_SHIFT_CM[0], 3.5 * GRID_PITCH_CM + START_SHIFT_CM[1])
GOAL_POS_CM = (0 * GRID_PITCH_CM - GOAL_OFFSET_CM + GOAL_SHIFT_CM[0], 3.5 * GRID_PITCH_CM + GOAL_SHIFT_CM[1])
# 経由点(左コース基準)
POINT_A_CM = (4 * GRID_PITCH_CM, 3.5 * GRID_PITCH_CM)                      # 54,55の中点
POINT_B_CM = (0.0 - GATE_EXIT_OFFSET_CM, 3.5 * GRID_PITCH_CM)              # 14,15の中点から、ゴール側へExit距離
POINT_C_CM = (0.0 - GATE_EXIT_OFFSET_CM, 0.0)                               # Bから下へ、11の点のY座標まで

_LABELS = (None, "fallback-A", "fallback-B", "fallback-C", "goal")


def _normalize_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


def fallback_waypoints():
    return [START_POS_CM, POINT_A_CM, POINT_B_CM, POINT_C_CM, GOAL_POS_CM]


def fallback_planner_steps():
    """planner(waypoints_to_plan)と同じ形式のstep列。スタート向きを0度とした絶対方位(左コース基準)。"""
    steps = []
    heading = START_HEADING_DEG
    points = fallback_waypoints()
    for i in range(1, len(points)):
        dx = points[i][0] - points[i - 1][0]
        dy = points[i][1] - points[i - 1][1]
        distance_cm = math.hypot(dx, dy)
        if distance_cm < 1e-6:
            continue
        segment_heading = math.degrees(math.atan2(dy, dx))
        shifted = round(_normalize_deg(segment_heading - START_HEADING_DEG), 2)
        if abs(_normalize_deg(segment_heading - heading)) > 1e-6:
            steps.append({"type": "turn", "target_heading_deg": shifted, "label": None})
        steps.append({
            "type": "move",
            "target_heading_deg": shifted,
            "distance_mm": round(distance_cm * 10.0, 1),
            "label": _LABELS[i],
        })
        heading = segment_heading
    return steps


def fallback_strategy():
    """PCから受信する経路と同じ方位の枠(コース正規化済み)のstep列。"""
    return planner_to_full_start(fallback_planner_steps(), "left")
