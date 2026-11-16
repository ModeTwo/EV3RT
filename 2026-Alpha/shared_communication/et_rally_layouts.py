"""2026-11-15: ETラリーのヒント → ゲートの配置(経路の道具の座標) → 計画を探す鍵。全配置(4872通り)の一覧も作る。

ヒント(ロボットが読んで PC へ送る形、wireless_device/et_rally_runner.py と同じ):
    hint1 = 赤 "XY,XY"、hint2 = 青と黄 "XY,XY/XY,XY"。X・Y は公式の座標(1〜5、G<X>-<Y> と同じ)。
道具の格子 (c, r) = (5 - X, 5 - Y)(day_route.py と同じ。11/11 にユーザーと確認)。
左コースも公式の座標のまま同じ計画を使う(走るときに向きの符号だけ逆になる)ので、鍵にコースは入れない。
ルール: 赤・黄は横向き(同じ Y)、青は縦向き(同じ X)、6本の脚は重ならない(et_rally_runner.py の検査と同じ)。

PC(wireless_device の計画の配り役)とロボット(robot_program/et_rally_plan_select.py の予備の選び方)の両方で使う。
使い方(2026-Alpha で):
    python -m shared_communication.et_rally_layouts --hint1 25,35 --hint2 53,54/12,22     # 配置と鍵を表示
    python -m shared_communication.et_rally_layouts --count                               # 全配置の数(4872)
"""
import argparse
import json
import re

COLORS = ('red', 'blue', 'yellow')


def official_to_tool(text):
    """"25" / "G2-5" / "2-5" → 道具の格子 [c, r]。"""
    s = text.strip()
    m = re.fullmatch(r'G?(\d)-?(\d)', s, re.I)
    if not m:
        raise ValueError('座標は XY か G<X>-<Y> の形で(X・Y は1〜5): %r' % text)
    X, Y = int(m.group(1)), int(m.group(2))
    if not (1 <= X <= 5 and 1 <= Y <= 5):
        raise ValueError('座標は1〜5: %r' % text)
    return [5 - X, 5 - Y]


def tool_to_official(p):
    return '%d%d' % (5 - p[0], 5 - p[1])


def _gate(text):
    parts = [p for p in text.split(',') if p.strip()]
    if len(parts) != 2:
        raise ValueError('ゲートは XY,XY の形で: %r' % text)
    a, b = official_to_tool(parts[0]), official_to_tool(parts[1])
    if abs(a[0] - b[0]) + abs(a[1] - b[1]) != 1:
        raise ValueError('ゲートの2つの点はとなり合う点に: %r' % text)
    return {'a': a, 'b': b}


def check(gates):
    """ルールに合っているか。合っていなければ ValueError。"""
    for color in COLORS:
        g = gates[color]
        a, b = g['a'], g['b']
        if abs(a[0] - b[0]) + abs(a[1] - b[1]) != 1:
            raise ValueError('%s: 2つの点がとなり合っていない' % color)
        horizontal = a[1] == b[1]
        if color in ('red', 'yellow') and not horizontal:
            raise ValueError('%s は横向き(同じ Y)のはず' % color)
        if color == 'blue' and horizontal:
            raise ValueError('blue は縦向き(同じ X)のはず')
    feet = [tuple(gates[c][k]) for c in COLORS for k in ('a', 'b')]
    if len(set(feet)) != len(feet):
        raise ValueError('ゲートの脚が重なっている')
    return gates


def gates_from_hints(hint1, hint2):
    """ヒント → 経路の道具の gates(右コースの座標。左コースも同じものを使う)。"""
    parts = [p for p in hint2.split('/')]
    if len(parts) != 2:
        raise ValueError('hint2 は XY,XY/XY,XY の形で: %r' % hint2)
    gates = {'course': 'right', 'red': _gate(hint1), 'blue': _gate(parts[0]), 'yellow': _gate(parts[1])}
    return check(gates)


def layout_key(gates):
    """配置の鍵(ゲートの2つの点の順番によらない)。例 'r1424-b2322-y3141'(道具の格子 c r を並べたもの)。"""
    out = []
    for color in COLORS:
        g = gates[color]
        pts = sorted([tuple(g['a']), tuple(g['b'])])
        out.append(color[0] + ''.join('%d%d' % p for p in pts))
    return '-'.join(out)


def all_layouts():
    """全配置(4872通り)を、決まった順番で。scratchpad の scan.py と同じ数え方(道具の格子で直接数える)。"""
    out = []
    for rr in range(5):
        for rc in range(4):
            red = ([rc, rr], [rc + 1, rr])
            for bc in range(5):
                for br in range(4):
                    blue = ([bc, br], [bc, br + 1])
                    for yr in range(5):
                        for yc in range(4):
                            yel = ([yc, yr], [yc + 1, yr])
                            feet = {tuple(p) for p in (*red, *blue, *yel)}
                            if len(feet) == 6:
                                out.append({'course': 'right', 'red': {'a': red[0], 'b': red[1]},
                                            'blue': {'a': blue[0], 'b': blue[1]}, 'yellow': {'a': yel[0], 'b': yel[1]}})
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--hint1')
    ap.add_argument('--hint2')
    ap.add_argument('--count', action='store_true')
    a = ap.parse_args()
    if a.count:
        L = all_layouts()
        print(len(L), 'layouts,', len({layout_key(g) for g in L}), 'distinct keys')
        return
    g = gates_from_hints(a.hint1, a.hint2)
    print(json.dumps({'key': layout_key(g), 'gates': g}, ensure_ascii=False))


if __name__ == '__main__':
    main()
