"""Isolated entry point for the unchanged ET rally route planner."""

import json
import sys
from pathlib import Path


CORE_DIR = Path(__file__).resolve().with_name("et_rally_planner")


def _parse_position(value):
    """Convert the official Gx-y two-digit notation to the planner grid."""
    value = value.strip()
    if len(value) != 2 or any(character not in "12345" for character in value):
        raise ValueError("Gate positions must be two digits from 1 to 5")
    x, y = int(value[0]), int(value[1])
    # 公式表記は左上がG1-1、計算ロジックは左下が(0, 0)なので境界で変換する。
    return x - 1, 5 - y


def _parse_gate_text(value):
    parts = [part.strip() for part in value.split(",")]
    if len(parts) != 2:
        raise ValueError("Each gate must use XY,XY format")
    first, second = (_parse_position(part) for part in parts)
    if abs(first[0] - second[0]) + abs(first[1] - second[1]) != 1:
        raise ValueError("Gate endpoints must be adjacent")
    return first, second


def _load_gate_values(payload):
    hint1 = payload.get("hint1")
    hint2 = payload.get("hint2")
    if not isinstance(hint1, str) or not isinstance(hint2, str):
        raise ValueError("hint1 and hint2 must be strings")
    hint2_parts = [part.strip() for part in hint2.split("/")]
    if len(hint2_parts) != 2:
        raise ValueError("hint2 must use XY,XY/XY,XY format")
    values = {
        "red": _parse_gate_text(hint1),
        "blue": _parse_gate_text(hint2_parts[0]),
        "yellow": _parse_gate_text(hint2_parts[1]),
    }
    if values["red"][0][1] != values["red"][1][1]:
        raise ValueError("The red gate must be horizontal")
    if values["blue"][0][0] != values["blue"][1][0]:
        raise ValueError("The blue gate must be vertical")
    if values["yellow"][0][1] != values["yellow"][1][1]:
        raise ValueError("The yellow gate must be horizontal")
    feet = [point for endpoints in values.values() for point in endpoints]
    if len(set(feet)) != len(feet):
        raise ValueError("Gate posts must not overlap")
    return values


def calculate(payload):
    """Adapt communication payload data without changing planner code."""
    course = payload.get("course")
    laps = payload.get("laps")
    if course not in ("left", "right"):
        raise ValueError("course must be left or right")
    if type(laps) is not int or not 1 <= laps <= 3:
        raise ValueError("laps must be an integer from 1 to 3")

    # ZIP内のモジュールはトップレベルimportを前提にしているため、独立した
    # このプロセス内だけで検索パスへ追加する。親の通信プロセスは汚染しない。
    sys.path.insert(0, str(CORE_DIR))
    import config

    config.LAPS = laps
    if course == "right":
        mirror_axis_x_cm = 2 * config.GRID_PITCH_CM

        def mirror_x(value):
            return 2 * mirror_axis_x_cm - value

        def mirror_heading(value):
            return (180.0 - value) % 360.0

        # ZIP付属export_plan_right_course.pyと同じ順序で、他モジュールを
        # importする前に開始・終了位置と方位を左右反転する。
        config.START_POS_CM = (mirror_x(config.START_POS_CM[0]), config.START_POS_CM[1])
        config.GOAL_POS_CM = (mirror_x(config.GOAL_POS_CM[0]), config.GOAL_POS_CM[1])
        config.START_HEADING_DEG = mirror_heading(config.START_HEADING_DEG)
        config.GOAL_HEADING_DEG = mirror_heading(config.GOAL_HEADING_DEG)

    from commands import waypoints_to_plan
    from planner import Gate
    from rule_route import plan_route

    gate_values = _load_gate_values(payload)
    if course == "right":
        # ZIP付属export_plan_right_course.pyと同じグリッド鏡映を適用する。
        gate_values = {
            color: tuple((4 - point[0], point[1]) for point in endpoints)
            for color, endpoints in gate_values.items()
        }
    gates = {
        color: Gate(color, endpoints[0], endpoints[1])
        for color, endpoints in gate_values.items()
    }
    waypoints, _total_distance_cm, labels, _true_points = plan_route(gates)
    return waypoints_to_plan(waypoints, labels)


def main():
    payload = json.load(sys.stdin)
    json.dump(calculate(payload), sys.stdout, ensure_ascii=True, allow_nan=False)


if __name__ == "__main__":
    main()
