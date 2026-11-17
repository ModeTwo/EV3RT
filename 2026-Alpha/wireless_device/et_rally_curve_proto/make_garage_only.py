"""2026-11-14: 予備の計画「スタートからガレージへ行くだけ」を作り、全配置で脚・進入禁止に当たらないかを調べる。

ゲートの配置が分からない・計画が見つからない・経路が作れない配置(942 など)のとき、ガレージで止まる5点だけを取りにいく。
フィールド(灰色の点 x <= 4P)には入らず、スタートの列(x = 4P + 14.9cm)のまま下へ進んでから、ガレージへ向かうので、
ゲートがどこにあっても同じ計画でよい。

  0. 3cm まっすぐ下がる
  1. その場で左へ90°(フィールドの手前の辺と平行に、下を向く)
  2. まっすぐ下へ(y = 3.5P から y = garage_y + r まで)
  3. 左へ90°の円弧(半径 r)で右(ガレージの方)を向く。ここでアームを下げる
  4. まっすぐガレージの中へ進んで止まる

usage: python make_garage_only.py out.json [gates_dir]   (gates_dir の g_*.json 全部で調べる)
"""
import glob
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import check_plan as cpk  # noqa: E402

PARAMS = os.path.join(HERE, 'params_day.json')
P = 24.64
START = (4 * P + 14.9, 3.5 * P)          # cm(経路の道具の start_at='goal')
R = 15.0                                 # 円弧の半径 cm
GOAL = (171.0, -28.8)                    # 止まる車軸の位置 cm(ガレージ x 159〜183.5・y -43.8〜-13.8 の中で、前後・左右とも余裕)


BACK = 3.0                               # 最初に下がる cm(その場で回るとき、前の角が G1 の列のゲートの脚の T字に 0.8cm まで近づいたため)


def make():
    y_turn = GOAL[1] + R                 # 円弧を始める y
    down = START[1] - y_turn
    along = GOAL[0] - (START[0] + BACK + R)
    steps = [
        {"type": "move", "target_heading_deg": 0.0, "distance_mm": BACK * 10, "reverse": True, "stop": True, "label": None},
        {"type": "turn", "target_heading_deg": -90.0, "reverse": False, "stop": False, "label": None},
        {"type": "move", "target_heading_deg": -90.0, "distance_mm": round(down * 10, 1), "reverse": False, "stop": False, "label": None},
        {"type": "arc", "target_heading_deg": -180.0, "sweep_deg": -90.0, "radius_mm": R * 10, "reverse": False, "stop": False, "label": "garage-arm-down"},
        {"type": "move", "target_heading_deg": -180.0, "distance_mm": round(along * 10, 1), "reverse": False, "stop": True, "label": None},
    ]
    return {
        "gates": None,
        "heading_convention": "target_heading_deg is the GYRO target for course=right file-mode execution (same as the other plans).",
        "note_1114": "予備の計画: スタートからガレージへ行くだけ(5点)。フィールドに入らないので、どのゲートの配置でも同じ。make_garage_only.py で作成。",
        "laps": 0, "goal_at": "garage",
        "total_distance_mm": round((down + math.pi / 2 * R + along) * 10, 1),
        "steps": steps,
    }


def check_all(plan, gates_glob):
    import numpy as np
    worst = None
    n = 0
    for f in sorted(glob.glob(gates_glob)):
        g = json.load(open(f))['gates']
        hr = cpk.load_tool(g, PARAMS)
        pts = cpk.sample(plan['steps'], (START[0], START[1], 180.0), pivot_back_cm=0.5)
        Pp = np.array([(q[0], q[1]) for q in pts]); H = np.radians([q[2] for q in pts])
        clr = float(hr._rect_clear(Pp[:, 0], Pp[:, 1], H, -hr.REAR, hr.FRONT).min())
        hr._EXEMPT_ON = True
        ko = int(hr.body_keepout_mask(Pp[:, 0], Pp[:, 1], np.degrees(H)).sum())
        n += 1
        if worst is None or clr < worst[0]:
            worst = (clr, os.path.basename(f), ko)
        if ko:
            print('keepout', f, ko)
    end = pts[-1]
    ov = cpk.garage_overlap(hr, end[0], end[1], end[2])
    print('配置 %d: 一番狭いすき間 %.2fcm(%s)、止まる位置 (%.1f, %.1f, 向き%.1f)、ガレージに食い込む x %.1f・y %.1f cm' % (
        n, worst[0], worst[1], end[0], end[1], end[2], ov[0], ov[1]))


if __name__ == '__main__':
    plan = make()
    json.dump(plan, open(sys.argv[1], 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
    if len(sys.argv) > 2:
        check_all(plan, os.path.join(sys.argv[2], 'g_*.json'))
