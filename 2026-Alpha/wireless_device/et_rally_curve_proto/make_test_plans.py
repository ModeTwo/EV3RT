"""2026-11-02: 円弧と後退の実機テスト用の計画JSONを robot_program/tests/ に作る。
向きは計画JSONの約束どおり(右回りが正、スタートの向きが0)。位置の見込みは、スタートの車軸(駆動輪の接地点の中点)
から見た「前」「右」のmm。止まる直前は40mmの直進(止まって終わる円弧のずれを避けるため)。
usage: python make_test_plans.py <2026-Alpha の robot_program/tests>
"""
import json, math, os, sys

OUT = sys.argv[1]
TAIL = 40.0


def nd(a):
    return (a + 180.0) % 360.0 - 180.0


def arc(end_h, sweep, r, rev=False, stop=False):
    return {'type': 'arc', 'target_heading_deg': round(nd(end_h), 2), 'sweep_deg': sweep, 'radius_mm': r,
            'reverse': rev, 'stop': stop, 'label': None}


def move(h, d, rev=False, stop=False, label=None):
    return {'type': 'move', 'target_heading_deg': round(nd(h), 2), 'distance_mm': d, 'reverse': rev, 'stop': stop,
            'label': label}


def save(name, note, steps, expect):
    plan = {'note_1102': note, 'expected_end': expect, 'steps': steps}
    json.dump(plan, open(os.path.join(OUT, name), 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
    print(name, len(steps), expect)


# A1: 8の字(右に1周→左に1周)を3回、最後に40mm直進して止まる。理想ではスタートの40mm前に、向き0で止まる。
for r in (100, 150, 200):
    steps = []
    h = 0.0
    for _ in range(3):
        for sw in (180, 180, -180, -180):
            h += sw
            steps.append(arc(h, float(sw), float(r)))
    steps.append(move(0.0, TAIL, stop=True))
    save(f'plan_test_arc8_r{r}.json',
         f'円弧のテスト(A1): 半径{r}mmで右に1周→左に1周の8の字を3回、続けて40mm直進して止まる。止まらずに円弧を12本続けて走る。'
         f'終わりの位置をスタートと比べ、ログ(et_loc)とのずれで、ログに出ない滑りを見る。円弧の速さは --rally-path-arc-speed-scale で変える。'
         f'場所は、スタートの左右に{2 * r + 200}mm、前後に{r + 250}mm程度あける。',
         steps, {'front_mm': TAIL, 'right_mm': 0.0, 'heading_deg': 0.0})

# A2: 直進300mm→右に90°/180°の円弧(止まらない)→直進300mmで止まる。円弧の入りと出口のふくらみと、出口のあとの向きの戻り。
for r in (100, 150, 200):
    for sw in (90, 180):
        steps = [move(0.0, 300.0), arc(float(sw), float(sw), float(r)), move(float(sw), 300.0, stop=True)]
        if sw == 90:
            exp = {'front_mm': 300.0 + r, 'right_mm': r + 300.0, 'heading_deg': 90.0}
        else:
            exp = {'front_mm': 0.0, 'right_mm': 2.0 * r, 'heading_deg': 180.0}
        save(f'plan_test_arc_entry_r{r}_{sw}.json',
             f'円弧の入りと出口のテスト(A2): 直進300mm→半径{r}mmで右に{sw}°の円弧→直進300mmで止まる(途中は止まらない)。'
             f'止まった位置と向きを測り、見込みとログを比べる。円弧の途中の向きの遅れ(err_max)はログに出る。',
             steps, exp)

# A4-1: 前進300mm→止まる→後退300mm→止まる を10回。理想ではスタートに戻る。
steps = []
for _ in range(10):
    steps.append(move(0.0, 300.0, stop=True))
    steps.append(move(0.0, 300.0, rev=True, stop=True))
save('plan_test_shuttle10.json',
     '後退のテスト(A4): 前進300mm→止まる→後退300mm→止まる を10回(前後の切り替え20回)。理想ではスタートに戻る。'
     '戻った位置のずれで、前後の切り替え1回あたりのずれ(歯車のガタなど)を見る。',
     steps, {'front_mm': 0.0, 'right_mm': 0.0, 'heading_deg': 0.0})

# A4-2: 後退の8の字(半径150mm)を2回、最後に40mm後退して止まる。理想ではスタートの40mm後ろに、向き0で止まる。
steps = []
h = 0.0
for _ in range(2):
    for sw in (180, 180, -180, -180):
        h += sw
        steps.append(arc(h, float(sw), 150.0, rev=True))
steps.append(move(0.0, TAIL, rev=True, stop=True))
save('plan_test_arc8_reverse_r150.json',
     '後退の円弧のテスト(A4): 半径150mmで、後退のまま8の字を2回、最後に40mm後退して止まる。前進の8の字と比べる。',
     steps, {'front_mm': -TAIL, 'right_mm': 0.0, 'heading_deg': 0.0})
