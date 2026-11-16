"""2026-11-15: ゲートを抜けて止まった位置を定規で測った値と、ロボットが思っていた位置・止まったあとのカメラの読み(camera tail)を比べる。

測り方(図 https://claude.ai/artifact/2zzLTd2WWwEmXJiL732SMN): 左右のタイヤの印それぞれについて、基準の脚の点から
もう1本の脚の点の向きへ a mm、点どうしを結んだ線からロボットが来た側へ b mm(抜けていく側は −)。

使い方(2026-Alpha で):
    python wireless_device/et_rally_camera_calib/ruler_check.py <run_logs の run_*.log> <計画> <ゲート red|blue|yellow> <基準 a|b> \
        <左 a> <左 b> <右 a> <右 b>
  基準: 計画の gates のそのゲートの 'a' か 'b' の脚(図の「基準」の点。赤は a=G2-1、青は b=G2-2、黄は a=G3-2)。
出力: 定規の車軸の真ん中・向き、ロボットが思っていた位置、カメラとの差(車体の前・左、向き)。
"""
import json
import math
import re
import sys

P = 246.4


def main():
    log, plan_path, gate, base_key = sys.argv[1:5]
    la, lb, ra, rb = (float(v) for v in sys.argv[5:9])
    plan = json.load(open(plan_path, encoding='utf-8'))
    g = plan['gates'][gate]
    base, other = g[base_key], g['b' if base_key == 'a' else 'a']
    bx, by = base[0] * P, base[1] * P
    au = ((other[0] - base[0]), (other[1] - base[1]))
    lines = open(log, encoding='utf-8', errors='replace').read().splitlines()
    last = [l for l in lines if 'EtRallyPathSegment' in l and 'summary' in l][-1]
    m = re.search(r'course=\(([-\d.]+),([-\d.]+),([-+\d.]+)\)', last)
    bel = tuple(float(v) for v in m.groups())
    tail = [l for l in lines if 'camera tail: finished' in l]
    cam = None
    if tail:
        t = re.search(r'median front ([-+\d.]+) right ([-+\d.]+) mm heading ([-+\d.]+)', tail[-1])
        cam = tuple(float(v) for v in t.groups()) if t else None
        if cam is None:
            print('カメラ: 止まったあとに使える画像なし(%s)' % tail[-1].split('camera tail:')[-1].strip())
    # b の向き: ロボットが来た側(思っていた向きの逆)
    d = (math.cos(math.radians(bel[2])), math.sin(math.radians(bel[2])))
    bu = (-au[1], au[0])
    if bu[0] * -d[0] + bu[1] * -d[1] < 0:
        bu = (-bu[0], -bu[1])
    w = lambda a, b: (bx + a * au[0] + b * bu[0], by + a * au[1] + b * bu[1])
    L, R = w(la, lb), w(ra, rb)
    cx, cy = (L[0] + R[0]) / 2, (L[1] + R[1]) / 2
    rx, ry = R[0] - L[0], R[1] - L[1]
    n = math.hypot(rx, ry)
    h = math.degrees(math.atan2(rx / n, -ry / n)) % 360   # 右 = (sin h, -cos h)
    def body(dx, dy, hh):
        a = math.radians(hh)
        return dx * math.cos(a) + dy * math.sin(a), -dx * math.sin(a) + dy * math.cos(a)
    f, l = body(cx - bel[0], cy - bel[1], bel[2])
    dh = ((h - bel[2] + 180) % 360) - 180
    print('定規: 車軸の真ん中 (%.1f, %.1f) 向き %.1f°(タイヤの間 %.0f mm)' % (cx, cy, h, n))
    print('思っていた位置: (%.1f, %.1f) 向き %.1f°' % bel)
    print('定規 − 思っていた位置: 前 %+.1f 左 %+.1f mm 向き %+.1f°' % (f, l, dh))
    if cam:
        print('カメラ − 思っていた位置: 前 %+.1f 左 %+.1f mm 向き %+.2f°' % (cam[0], -cam[1], cam[2]))
        print('カメラの読み違い(カメラ − 定規): 前 %+.1f 左 %+.1f mm 向き %+.1f°' % (cam[0] - f, -cam[1] - l, cam[2] - dh))


if __name__ == '__main__':
    main()
