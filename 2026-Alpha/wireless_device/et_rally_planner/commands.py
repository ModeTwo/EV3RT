"""折れ線のウェイポイント列を、走行体制御システム向けの move/turn コマンド列に変換する。

move(distance_mm): 前進が正、後退が負
turn(angle_deg): 右旋回(時計回り)が正、左旋回(反時計回り)が負 (暫定規約)
角度は内部的に数学の標準系(反時計回りが正、+x方向が0度)で扱う。
"""

import math

import config
import geometry as geo


def waypoints_to_commands(waypoints, start_heading_deg=config.START_HEADING_DEG,
                           goal_heading_deg=config.GOAL_HEADING_DEG):
    """waypoints(折れ線の頂点座標列)を、隣り合う頂点同士の向きの差分から
    ("turn", 角度deg) / ("move", 距離mm) の相対コマンド列に変換する。

    各区間で「直前の向きから何度曲がるか」だけを見るため、区間をまたいで
    誤差が蓄積しやすい(実機の絶対方位ベース制御を使うならwaypoints_to_plan()
    の方が向く)。move()の直前turn()が省略されるのは旋回不要(ほぼ直進が
    続く)区間のみ。
    """
    commands = []
    heading = start_heading_deg

    for i in range(1, len(waypoints)):
        prev_pt = waypoints[i - 1]
        cur_pt = waypoints[i]
        dist_cm = geo.distance(prev_pt, cur_pt)
        if dist_cm < 1e-6:
            continue  # 同一点はスキップ(旋回のみの中間ノード等)

        segment_heading = math.degrees(math.atan2(cur_pt[1] - prev_pt[1], cur_pt[0] - prev_pt[0]))
        turn_deg = _heading_delta_to_turn_command(heading, segment_heading)
        if abs(turn_deg) > 1e-6:
            commands.append(("turn", round(turn_deg, 2)))
        commands.append(("move", round(dist_cm * 10.0, 1)))  # cm -> mm
        heading = segment_heading

    # 最終姿勢をゴール向きに合わせる
    final_turn = _heading_delta_to_turn_command(heading, goal_heading_deg)
    if abs(final_turn) > 1e-6:
        commands.append(("turn", round(final_turn, 2)))

    return commands


def _heading_delta_to_turn_command(from_heading_deg, to_heading_deg):
    """標準系(反時計回り正)の向きの差から、turn()コマンド(右回り正)の角度を求める。"""
    delta_ccw = geo.normalize_deg(to_heading_deg - from_heading_deg)
    return -delta_ccw


def waypoints_to_plan(waypoints, labels=None, start_heading_deg=config.START_HEADING_DEG,
                       goal_heading_deg=config.GOAL_HEADING_DEG):
    """実機の絶対方位ベースの制御部品(RunByGyro/SpinAround等、target_type=ABSOLUTE)に
    そのまま渡せる形式のステップ列を作る。move/turnコマンド列(waypoints_to_commands)との違いは、
    各区間の「直前からの旋回量」ではなく「その区間の絶対方位」を持つ点(誤差が後続区間に
    蓄積しない)。

    方位は、スタート時点(この経路のstart_heading_deg)を0度とする相対的な基準に
    シフトして返す。実機側は「ジャイロを0にリセットした瞬間に向いている方角」を
    そのまま基準として使えばよく、内部座標系(数学の標準系、反時計回りが正)を
    実機のジャイロの符号規約に合わせる必要はない。ただし、角度が増える向き
    (時計回りか反時計回りか)が実機のジャイロ規約と一致しているかは、
    実際に動かして必ず確認すること(逆だった場合、全区間のtarget_heading_degの
    符号を反転すれば直る)。

    戻り値: [{"type": "move", "target_heading_deg": float, "distance_mm": float, "label": str|None}, ...
             {"type": "turn", "target_heading_deg": float, "label": str|None}, ...]
    """
    steps = []
    heading = start_heading_deg

    def shifted(h):
        return round(geo.normalize_deg(h - start_heading_deg), 2)

    for i in range(1, len(waypoints)):
        prev_pt = waypoints[i - 1]
        cur_pt = waypoints[i]
        dist_cm = geo.distance(prev_pt, cur_pt)
        if dist_cm < 1e-6:
            continue  # 同一点はスキップ(旋回のみの中間ノード等)

        segment_heading = math.degrees(math.atan2(cur_pt[1] - prev_pt[1], cur_pt[0] - prev_pt[0]))
        label = labels[i] if labels is not None else None
        if abs(geo.normalize_deg(segment_heading - heading)) > 1e-6:
            steps.append({"type": "turn", "target_heading_deg": shifted(segment_heading), "label": None})
        steps.append({
            "type": "move",
            "target_heading_deg": shifted(segment_heading),
            "distance_mm": round(dist_cm * 10.0, 1),
            "label": label,
        })
        heading = segment_heading

    if abs(geo.normalize_deg(goal_heading_deg - heading)) > 1e-6:
        steps.append({"type": "turn", "target_heading_deg": shifted(goal_heading_deg), "label": "goal"})

    return steps
