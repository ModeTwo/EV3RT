"""2026-11-03: 画像で見つけたマーカー(車体から見た床の位置)を、コースの地図と結びつけて、車体のコース上の位置と向きを出す(試作、PCで動かす)。

座標:
- コース: 経路の探索ツール(hybrid_route.py)と同じ右コースの planner 座標を mm にしたもの。x右・y上、向きは左回りが+(度)。
  灰色の点(=ゲートの柱を立てる点)は P=246.4mm の格子の点 (i*P, j*P)、i,j = 0..4。
- マーカーは、灰色の点4つに囲まれたマスの中心 ((i+0.5)P, (j+0.5)P)、i,j = 0..3。辺は格子の向きにそろっている(仮定)。
- 車体: 原点は車軸、X=前・Y=左(mm)。marker_ground.py の出力と同じ。

やること:
1. 走行ログの位置と向き(odom)で、見えたマーカーをコースの上に置き、一番近いマスの中心に結びつける。
   となりのマスの中心まで P(246mm) あるので、odom のずれが約100mm以内なら取り違えない。
2. 結びつけたマーカーから、車体の位置と向きを出す。
   - マーカー1個: 向きは「マーカーの辺が格子にそろっている」ことから(90°ごとに同じに見えるので、odomの向きに一番近いもの)。
     位置は、その向きでマーカーの中心がマスの中心に来るように。
   - マーカー2個以上: 位置と向きを最小二乗でまとめて当てはめる(辺の向きも加える)。
usage: python marker_localize.py --selftest
"""
import argparse
import math

import numpy as np

P_MM = 246.4
CELLS = [((i + 0.5) * P_MM, (j + 0.5) * P_MM) for i in range(4) for j in range(4)]
ASSOC_MAX_MM = 100.0        # odomで置いた位置と、マスの中心がこれより離れていたら結びつけない


def nd(a):
    return (a + 180.0) % 360.0 - 180.0


def rot(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])


def to_world(pose, xy):
    """pose=(x, y, 向き度) の車体から見た点 xy=(前, 左) を、コースの座標へ。"""
    return np.asarray(pose[:2], float) + rot(pose[2]) @ np.asarray(xy, float)


def associate(odom, dets, cells=CELLS, max_mm=ASSOC_MAX_MM):
    """dets: [{'xy': (前, 左)mm, 'rot': マーカーの回転(度、車体から見て左回りが+)}, ...]
    戻り値: [(det, マスの中心, odomで置いた位置とのずれmm)]。同じマスに2つ結びついたら近い方だけ。"""
    C = np.array(cells)
    best = {}
    for d in dets:
        w = to_world(odom, d['xy'])
        dist = np.linalg.norm(C - w, axis=1)
        k = int(np.argmin(dist))
        if dist[k] <= max_mm and (k not in best or dist[k] < best[k][2]):
            best[k] = (d, C[k], float(dist[k]))
    return list(best.values())


def marker_heading(odom_h, rot_deg):
    """マーカーの回転から車体の向きを出す。マーカーの辺がコースの格子にそろっているので、
    車体の向き = -(見えた回転) + 90°×k。odomの向きに一番近い k を選ぶ。"""
    base = -rot_deg
    k = round((odom_h - base) / 90.0)
    return nd(base + 90.0 * k)


def fit_pose(odom, pairs, w_rot=1.0):
    """結びついたマーカーから、車体の位置と向きを出す。
    向き: 各マーカーの辺の向き(重み w_rot)と、2個以上なら中心どうしの並びの向きを、角度の平均でまとめる。
    位置: その向きで、マーカーの中心がマスの中心に来るように(2個以上なら平均)。"""
    if not pairs:
        return None
    hs, ws = [], []
    for d, c, _ in pairs:
        if d.get('rot') is not None:
            hs.append(marker_heading(odom[2], d['rot'])); ws.append(w_rot)
    if len(pairs) >= 2:
        A = np.array([d['xy'] for d, _, _ in pairs], float); B = np.array([c for _, c, _ in pairs], float)
        a0, b0 = A.mean(0), B.mean(0)
        H = (A - a0).T @ (B - b0)
        h2 = math.degrees(math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1]))
        spread = float(np.linalg.norm(A - a0, axis=1).sum())
        hs.append(h2); ws.append(spread / 50.0)     # 離れた2個ほど向きが正確
    if not hs:
        return None
    # 角度の重み付き平均(odomの向きのまわりで)
    h = nd(odom[2] + sum(w * nd(x - odom[2]) for x, w in zip(hs, ws)) / sum(ws))
    R = rot(h)
    pos = np.mean([np.asarray(c, float) - R @ np.asarray(d['xy'], float) for d, c, _ in pairs], axis=0)
    return (float(pos[0]), float(pos[1]), h)


def selftest(n=2000, seed=1):
    """本当の位置と向きをランダムに決め、見えるマーカーを作って(検出のずれ・odomのずれを加えて)、戻せるかを見る。"""
    rng = np.random.default_rng(seed)
    errs, miss, wrong = [], 0, 0
    for _ in range(n):
        true = (rng.uniform(0, 4 * P_MM), rng.uniform(0, 4 * P_MM), rng.uniform(-180, 180))
        dets = []
        for c in CELLS:
            R = rot(true[2]).T
            xy = R @ (np.array(c) - np.array(true[:2]))
            # 写る範囲: 前330〜700mm、横は距離に応じて(較正の結果の近似)
            if 340 <= xy[0] <= 700 and abs(xy[1]) <= 0.36 * xy[0] + 10:
                dets.append({'xy': xy + rng.normal(0, 3.0, 2), 'rot': nd(-true[2] + rng.normal(0, 1.0)) , 'cell': c})
        for d in dets:
            d['rot'] = ((d['rot'] + 45) % 90) - 45      # 見えるのは90°ごとの回転だけ
        if not dets:
            continue
        odom = (true[0] + rng.normal(0, 30), true[1] + rng.normal(0, 30), nd(true[2] + rng.normal(0, 3)))
        pairs = associate(odom, dets)
        if not pairs:
            miss += 1; continue
        if any(tuple(c) != tuple(d['cell']) for d, c, _ in pairs):
            wrong += 1
        est = fit_pose(odom, pairs)
        errs.append((math.hypot(est[0] - true[0], est[1] - true[1]), abs(nd(est[2] - true[2])), len(pairs)))
    e = np.array(errs)
    print('試行 %d: 結びつかない %d、取り違え %d' % (len(e) + miss, miss, wrong))
    for k in (1, 2, 3):
        s = e[e[:, 2] == k] if k < 3 else e[e[:, 2] >= 3]
        if len(s):
            print('  マーカー%s個: %d回  位置のずれ 中央%.1f・95%%%.1f mm  向きのずれ 中央%.2f・95%%%.2f°'
                  % (k if k < 3 else '3以上', len(s), np.median(s[:, 0]), np.percentile(s[:, 0], 95),
                     np.median(s[:, 1]), np.percentile(s[:, 1], 95)))


def check_two_markers():
    """11/03の marker_019〜021(マーカー2個)で、マーカーの辺の向きと、2個の並びの向きが合うかを見る。"""
    A = np.array([[543.6, 88.0], [461.4, -147.1]])
    rot_seen = -18.43
    d = A[0] - A[1]
    pair = math.degrees(math.atan2(d[1], d[0]))
    print('2個の並びの向き %.2f°(格子の軸から %.2f°)、辺から出した回転 %.2f°、2個の間隔 %.1f mm(格子 %.1f)'
          % (pair, nd(pair - 90.0), rot_seen, np.linalg.norm(d), P_MM))


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args()
    check_two_markers()
    selftest()
