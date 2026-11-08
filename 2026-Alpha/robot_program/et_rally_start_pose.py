"""ETラリー専用: スタートで止まっている間に、カメラでコース上の位置と向きを出し、走り出しに取り込む(2026-11-06)。

背景: 本番は前の工程からETラリーの開始位置へ移動してくるので、置かれた位置と向きが保証されない
(ユーザーの見込みで前後2cm・向き5°くらいまで)。机上のシミュレーションでは、知らずに走ると向き5°で
赤ゲートのあたりが3〜7cm外れる。知っていれば、向きはジャイロに足すだけで直り(1mm)、位置のずれは
最初のつなぎを引き直すと4〜11mm(取り込むだけだと13〜18mm)。

流れ:
  1. EtRallyStartPoseCapture: ジャイロの測り直しで止まっている間に、カメラの画像から
     マーカー(マスの中心、1辺約50mm)と灰色の点(格子の点、直径約44mm)を探し、位置と向きを出す。
     何枚か出して真ん中の値(中央値)を使う。
  2. EtRallyApplyStartPose: 計画のスタート(START_MM・向き0)とのずれを、
     - 向き: ジャイロの向きに足す(EtRallyHeading.set_heading_offset)。
     - 位置: ずれが rejoin_min_mm 以上なら、最初のつなぎを引き直す(rejoin_steps)。
             それより小さければ、最初の区間の自分の位置の初期値にする(et_rally_path_segment.set_start_position)。
  3. 計画の読み込み(execute_strategy.DeferredStrategySequence)で apply_pending_rejoin を通す。

座標: 経路を作る道具(右コース)の座標、mm。x はゴール側が+、y はスタートとゴールの列の側が+、向きは左回りが+(度)。
計画の target_heading_deg は「ジャイロ」の向きで、= -(この向き)。左コースは計画が同じで、走るときに向きの
符号だけが逆になる(heading = -course × ジャイロ)。カメラで見た左右も、左コースでは逆にしてから使う。
車体の座標: 原点は車軸の真ん中、前が+、左が+(mm)。カメラの較正(calib_*.json)と同じ。
"""

import json
import math
import time
from pathlib import Path

from py_trees.behaviour import Behaviour
from py_trees.common import Status

from .runtime import runtime

P_MM = 246.4                     # 格子の点の間隔(実測246mm、経路を作る道具は24.64cm)
CELLS = [((i + 0.5) * P_MM, (j + 0.5) * P_MM) for i in range(4) for j in range(4)]   # マーカー(マスの中心)
DOTS = [(i * P_MM, j * P_MM) for i in range(5) for j in range(5)]                      # 灰色の点
# 計画のスタート(hybrid_route.py の START: x = 2*2P - (4P + 17.8cm)、y = 3.5P、向き0)
START_MM = (-178.0, 3.5 * P_MM)
START_HEADING_DEG = 0.0

DEFAULT_CALIB = Path(__file__).resolve().parent.parent / "wireless_device" / "et_rally_camera_calib" / "calib_1108.json"

# 真上から見た画像の範囲(車体の座標、mm)と1画素の大きさ。スタートでは止まっているので、少し広めに細かく見る。
TOP_S = 0.5
TOP_X = (300.0, 820.0)
TOP_Y = (-330.0, 330.0)
QR_SIDE_MM = (44.0, 56.0)        # マーカーの1辺として受け入れる範囲(実物は50〜51mm)
DOT_DIAM_MM = (34.0, 54.0)       # 灰色の点の直径として受け入れる範囲(実物は約44mm)

ASSOC_MARKER_MM = 80.0           # 見えたマーカーと、計画のスタートから見て予想したマスの中心が、これより離れていたら使わない
ASSOC_DOT_MM = 60.0
# 縮尺の当てはめ(fit_pose の fit_scale): 較正のずれで床の距離が全体に k 倍に見えると、位置が (1-k)×距離 だけずれる
# (11/03の較正で約2.5%短く、机上で約14mm)。縮む中心はカメラの真下(車軸の約11〜17mm前)とする。
# 11/08: マットの上では置き場所でロボットの前後の傾き(カメラの下向き)が約0.8°変わり、50〜70cm先で約1.5〜2cmずれる。
# 縮尺の当てはめはその近似(本当は置き場所ごとに下向きを当てはめたい)。
CAM_NADIR_MM = (11.0, 0.0)
SCALE_MIN_POINTS = 3
SCALE_MIN_SPREAD_MM = 80.0       # 点の広がり(重心からの距離の平均)がこれより小さいと縮尺は当てない
SCALE_LIMIT = (0.94, 1.06)


# ---------------- カメラの較正と、真上から見た画像 ----------------
class GroundCamera:
    """較正(calib_*.json の lens と jig[0])から、画像 → 床(車体の座標)の対応表を作っておく。"""

    def __init__(self, calib_path=None, top_x=TOP_X, top_y=TOP_Y, top_s=TOP_S):
        import cv2
        import numpy as np
        self.cv2, self.np = cv2, np
        d = json.load(open(calib_path or DEFAULT_CALIB, encoding="utf-8"))
        K = np.array(d["lens"]["K"]); dist = np.array(d["lens"]["dist"])
        j = d["jig"][0]
        rv = np.array(j["rvec"], dtype=float); t = np.array(j["tvec"], dtype=float).reshape(3)
        self.top_x, self.top_y, self.top_s = top_x, top_y, top_s
        xs = np.arange(top_x[1], top_x[0], -top_s); ys = np.arange(top_y[1], top_y[0], -top_s)
        YY, XX = np.meshgrid(ys, xs)
        pts = np.stack([XX.ravel(), YY.ravel(), np.zeros(XX.size)], 1)
        p, _ = cv2.projectPoints(pts, rv, t, K, dist)
        self.map = p.reshape(XX.shape + (2,)).astype(np.float32)
        # 画像の中に入っている所(端から EDGE_PX 画素より内側)。端で切れた点・マーカーは中心がずれるので使わない(11/08)。
        w, h = d["lens"]["size"]
        e = self.EDGE_PX
        self.valid = ((self.map[..., 0] >= e) & (self.map[..., 0] <= w - 1 - e) &
                      (self.map[..., 1] >= e) & (self.map[..., 1] <= h - 1 - e))

    EDGE_PX = 3

    def top_view(self, img):
        return self.cv2.remap(img, self.map[..., 0], self.map[..., 1], self.cv2.INTER_LINEAR, borderValue=(255, 255, 255))

    def to_mm(self, pts):
        """真上から見た画像の (列, 行) → 車体の座標 (前, 左) mm。"""
        np = self.np
        pts = np.asarray(pts, np.float64).reshape(-1, 2)
        return np.stack([self.top_x[1] - pts[:, 1] * self.top_s, self.top_y[1] - pts[:, 0] * self.top_s], 1)


def _inside(cam, pts):
    """真上から見た画像の点(列, 行)が、すべて元の画像の中(端から少し内側)に入っているか。"""
    hgt, wid = cam.valid.shape
    for x, y in pts:
        xi, yi = int(round(x)), int(round(y))
        if not (0 <= xi < wid and 0 <= yi < hgt) or not cam.valid[yi, xi]:
            return False
    return True


def detect_ground(cam, img):
    """真上から見た画像で、マーカーと灰色の点を探す。
    戻り値: {'markers': [{'xy': (前, 左), 'rot': 辺の向き(度、-45〜45)}], 'dots': [(前, 左)]}
    マーカー: まわりより暗く色の薄い画素を6mmの閉じでつなぎ、1辺44〜56mm・黒の割合0.25〜0.9の正方形
    (wireless_device/et_rally_camera_calib/marker_ground.py の find_qr_ground と同じ考え方。11/03の21枚すべてで検出)。
    灰色の点: まわりより少し暗い丸(直径34〜54mm)で、円の中の暗い画素の割合が高いもの。"""
    cv2, np = cam.cv2, cam.np
    S = cam.top_s
    top = cam.top_view(img)
    hsv = cv2.cvtColor(top, cv2.COLOR_BGR2HSV)
    v = hsv[..., 2].astype(np.int16)
    k_bg = int(round(30.0 / S)) | 1
    bg = cv2.medianBlur(hsv[..., 2], min(k_bg, 255)).astype(np.int16)
    low_sat = hsv[..., 1] < 150
    out = {"markers": [], "dots": []}
    # マーカー
    dark = (((bg - v) > 25) & low_sat).astype(np.uint8) * 255
    k = int(round(6.0 / S))
    blob = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (k, k)))
    n, lab, st, _ = cv2.connectedComponentsWithStats(blob)
    marker_mask = np.zeros(dark.shape, np.uint8)
    for i in range(1, n):
        if not ((35.0 / S) ** 2 <= st[i, 4] <= (62.0 / S) ** 2):
            continue
        ys, xs = np.nonzero((lab == i) & (dark > 0))
        if len(xs) < 10:
            continue
        rect = cv2.minAreaRect(np.stack([xs, ys], 1).astype(np.float32))
        (cx, cy), (rw, rh), _ = rect
        side = (rw + rh) / 2.0
        if not (QR_SIDE_MM[0] <= side * S <= QR_SIDE_MM[1]) or max(rw, rh) / max(1e-6, min(rw, rh)) > 1.2:
            continue
        fill = len(xs) / (rw * rh)
        if not (0.25 < fill < 0.9):
            continue
        c = np.array([cx, cy])
        box = c + (cv2.boxPoints(rect) - c) * ((side + 1.0) / side)
        if not _inside(cam, box):
            continue
        g = cam.to_mm(box)
        dirs = []
        for q in range(4):
            e = g[(q + 1) % 4] - g[q]
            dirs.append(((math.degrees(math.atan2(e[1], e[0])) + 45.0) % 90.0) - 45.0)
        # 辺の向きの平均(-45/45 の境目をまたぐときのために、基準に近い方へそろえる)
        ref = dirs[0]
        dirs = [d - 90.0 * round((d - ref) / 90.0) for d in dirs]
        rot = ((sum(dirs) / 4.0 + 45.0) % 90.0) - 45.0
        out["markers"].append({"xy": tuple(float(x) for x in g.mean(0)), "rot": rot})
        cv2.drawContours(marker_mask, [np.int32(box)], 0, 255, -1)
    # 灰色の点: 点(直径44mm)より広い約100mm四方を明るさの基準にする(30mmだと点の中が基準になり縁しか残らない)。
    # 重いので、4分の1に縮めた画像で中央値をとってから元の大きさに戻す。
    small = cv2.resize(hsv[..., 2], None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)
    k_dot = int(round(100.0 / (S * 4))) | 1
    bg_dot = cv2.resize(cv2.medianBlur(small, min(k_dot, 255)), (v.shape[1], v.shape[0]), interpolation=cv2.INTER_LINEAR).astype(np.int16)
    gray = (((bg_dot - v) > 10) & low_sat & (marker_mask == 0)).astype(np.uint8) * 255
    gray = cv2.morphologyEx(gray, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
    n, lab, st, cen = cv2.connectedComponentsWithStats(gray)
    r_lo, r_hi = DOT_DIAM_MM[0] / 2.0 / S, DOT_DIAM_MM[1] / 2.0 / S
    for i in range(1, n):
        area = st[i, 4]
        if not (math.pi * r_lo ** 2 * 0.6 <= area <= math.pi * r_hi ** 2 * 1.2):
            continue
        ys, xs = np.nonzero(lab == i)
        (ccx, ccy), rr = cv2.minEnclosingCircle(np.stack([xs, ys], 1).astype(np.float32))
        if not (r_lo <= rr <= r_hi) or area / (math.pi * rr * rr) < 0.65:
            continue
        ring = [(ccx + (rr + 3) * math.cos(a), ccy + (rr + 3) * math.sin(a)) for a in np.linspace(0, 2 * math.pi, 16, endpoint=False)]
        if not _inside(cam, ring):
            continue
        out["dots"].append(tuple(float(x) for x in cam.to_mm([(cen[i][0], cen[i][1])])[0]))
    return out


# ---------------- 位置と向きを出す ----------------
def _to_world(pose, xy):
    x, y, h = pose
    a = math.radians(h)
    return (x + xy[0] * math.cos(a) - xy[1] * math.sin(a), y + xy[0] * math.sin(a) + xy[1] * math.cos(a))


def _nd(a):
    return (a + 180.0) % 360.0 - 180.0


def fit_pose(dets, nominal=(START_MM[0], START_MM[1], START_HEADING_DEG), mirror=False, fit_scale=True):
    """見えたもの(detect_ground の戻り値)から、コース上の位置と向き (x, y, 向き度) を出す。
    nominal: 予想の位置(計画のスタート)。見えたものを予想の位置で床に置き、近いマス・点に結びつける。
    mirror: 左コースなら True(カメラで見た左右を逆にする)。
    2点以上結びつけば、点どうしの並びから位置と向きを出す(重みはマーカー2・点1)。
    マーカー1個だけなら、向きはマーカーの辺(90°ごと、予想に近いもの)、位置はマスの中心から逆算する。
    fit_scale: 3点以上で広がりが十分なら、縮尺も一緒に当てはめる(カメラの真下を中心に)。
    戻り値: dict(pose=(x, y, h), n_markers, n_dots, resid_mm, scale) か None。"""
    sgn = -1.0 if mirror else 1.0
    marks = [({"xy": (m["xy"][0], sgn * m["xy"][1]), "rot": sgn * m["rot"]}) for m in dets.get("markers", [])]
    dots = [(d[0], sgn * d[1]) for d in dets.get("dots", [])]
    pose = tuple(nominal)
    result = None
    for _ in range(3):
        pairs = []   # (車体の座標, コースの座標, 重み, 種類)
        for m in marks:
            w = _to_world(pose, m["xy"])
            c = min(CELLS, key=lambda q: math.hypot(q[0] - w[0], q[1] - w[1]))
            if math.hypot(c[0] - w[0], c[1] - w[1]) <= ASSOC_MARKER_MM:
                pairs.append((m["xy"], c, 2.0, m))
        for d in dots:
            w = _to_world(pose, d)
            c = min(DOTS, key=lambda q: math.hypot(q[0] - w[0], q[1] - w[1]))
            if math.hypot(c[0] - w[0], c[1] - w[1]) <= ASSOC_DOT_MM:
                pairs.append((d, c, 1.0, None))
        if not pairs:
            return None
        if len(pairs) >= 2:
            W = sum(p[2] for p in pairs)
            ax = sum(p[2] * p[0][0] for p in pairs) / W; ay = sum(p[2] * p[0][1] for p in pairs) / W
            bx = sum(p[2] * p[1][0] for p in pairs) / W; by = sum(p[2] * p[1][1] for p in pairs) / W
            sxx = sum(p[2] * ((p[0][0] - ax) * (p[1][0] - bx) + (p[0][1] - ay) * (p[1][1] - by)) for p in pairs)
            sxy = sum(p[2] * ((p[0][0] - ax) * (p[1][1] - by) - (p[0][1] - ay) * (p[1][0] - bx)) for p in pairs)
            h = math.degrees(math.atan2(sxy, sxx))
            a = math.radians(h)
            saa = sum(p[2] * ((p[0][0] - ax) ** 2 + (p[0][1] - ay) ** 2) for p in pairs)
            spread = sum(p[2] * math.hypot(p[0][0] - ax, p[0][1] - ay) for p in pairs) / W
            scale = 1.0
            if fit_scale and len(pairs) >= SCALE_MIN_POINTS and spread >= SCALE_MIN_SPREAD_MM and saa > 0:
                scale = min(max(math.hypot(sxx, sxy) / saa, SCALE_LIMIT[0]), SCALE_LIMIT[1])
            # コース = t + scale × R × 車体。t = 本当の位置 + (1 - scale) × R × カメラの真下
            tx = bx - scale * (ax * math.cos(a) - ay * math.sin(a)); ty = by - scale * (ax * math.sin(a) + ay * math.cos(a))
            cx, cy = CAM_NADIR_MM
            x = tx - (1.0 - scale) * (cx * math.cos(a) - cy * math.sin(a))
            y = ty - (1.0 - scale) * (cx * math.sin(a) + cy * math.cos(a))
        else:
            scale = 1.0
            body, c, _, m = pairs[0]
            if m is None:
                return None   # 点1つだけでは向きが決まらない
            base = -m["rot"]
            h = _nd(base + 90.0 * round((pose[2] - base) / 90.0))
            a = math.radians(h)
            x = c[0] - (body[0] * math.cos(a) - body[1] * math.sin(a)); y = c[1] - (body[0] * math.sin(a) + body[1] * math.cos(a))
        pose = (x, y, h)
        cx, cy = CAM_NADIR_MM
        resid = [math.hypot(*(lambda w: (w[0] - p[1][0], w[1] - p[1][1]))(
            _to_world(pose, (cx + (p[0][0] - cx) * scale, cy + (p[0][1] - cy) * scale)))) for p in pairs]
        result = dict(pose=pose, n_markers=sum(1 for p in pairs if p[3] is not None),
                      n_dots=sum(1 for p in pairs if p[3] is None), resid_mm=max(resid), scale=scale)
    return result


def median_pose(results):
    """何枚分かの fit_pose の結果から、それぞれの真ん中の値(向きは予想まわりで)。"""
    if not results:
        return None
    xs = sorted(r["pose"][0] for r in results); ys = sorted(r["pose"][1] for r in results)
    hs = sorted(r["pose"][2] for r in results)
    mid = len(results) // 2
    pick = (lambda v: v[mid]) if len(results) % 2 else (lambda v: (v[mid - 1] + v[mid]) / 2.0)
    return (pick(xs), pick(ys), pick(hs))


# ---------------- 最初のつなぎの引き直し ----------------
def _integrate(steps, k, start=(START_MM[0], START_MM[1], START_HEADING_DEG)):
    """計画の steps[:k] を積算した姿勢 (x, y, 向き度)。円弧・直進だけを想定する。"""
    x, y, h = start
    for s in steps[:k]:
        if s["type"] == "move":
            hh = -float(s["target_heading_deg"]); th = math.radians(hh + (180.0 if s.get("reverse") else 0.0))
            x += float(s["distance_mm"]) * math.cos(th); y += float(s["distance_mm"]) * math.sin(th); h = hh
        elif s["type"] == "arc":
            sw = -float(s["sweep_deg"]); r = float(s["radius_mm"]); sg = 1.0 if sw > 0 else -1.0
            th0 = math.radians(h + (180.0 if s.get("reverse") else 0.0))
            cx, cy = x - sg * r * math.sin(th0), y + sg * r * math.cos(th0)
            th1 = th0 + math.radians(sw)
            x, y = cx + sg * r * math.sin(th1), cy - sg * r * math.cos(th1); h = h + sw
        else:
            raise ValueError("turn before the join point")
    return (x, y, h)


def _step_len(s):
    if s["type"] == "move":
        return float(s["distance_mm"])
    if s["type"] == "arc":
        return float(s["radius_mm"]) * math.radians(abs(float(s["sweep_deg"])))
    return 0.0


def join_index(steps, min_move_mm=150.0):
    """引き直したつなぎで合流する場所(steps[k] の始め)。走り出しから、前進の円弧・直進だけが止まらずに続き、
    ゲートの通過(label)より前にある、最初の min_move_mm 以上の直進の終わり。見つからなければ None。"""
    for i, s in enumerate(steps):
        if s["type"] not in ("move", "arc") or s.get("reverse") or s.get("label") or s.get("stop"):
            return None
        if s["type"] == "move" and float(s["distance_mm"]) >= min_move_mm:
            return i + 1 if i + 1 < len(steps) else None
    return None


def _dubins(A, B, r):
    """A, B = (x, y, 向きrad)。半径 r の 円弧-直進-円弧 のうち一番短いもの。
    戻り値 (全長, 1つめの回る向き(+1=左), 1つめの角度rad, 直進の向きrad, 直進の長さ, 2つめの回る向き, 2つめの角度rad) か None。"""
    tau = 2.0 * math.pi
    best = None
    for t1 in (1, -1):
        for t2 in (1, -1):
            c1 = (A[0] - t1 * r * math.sin(A[2]), A[1] + t1 * r * math.cos(A[2]))
            c2 = (B[0] - t2 * r * math.sin(B[2]), B[1] + t2 * r * math.cos(B[2]))
            vx, vy = c2[0] - c1[0], c2[1] - c1[1]; D = math.hypot(vx, vy)
            if t1 == t2:
                cands = [math.atan2(vy, vx)]
            else:
                if D < 2 * r:
                    continue
                L = math.sqrt(D * D - 4 * r * r)
                cands = [math.atan2(vy, vx) + e * math.atan2(2 * r, L) for e in (1, -1)]
            for phi in cands:
                a1 = (t1 * (phi - A[2])) % tau; a2 = (t2 * (B[2] - phi)) % tau
                p1 = (c1[0] + t1 * r * math.sin(phi), c1[1] - t1 * r * math.cos(phi))
                p2 = (c2[0] + t2 * r * math.sin(phi), c2[1] - t2 * r * math.cos(phi))
                ex, ey = p2[0] - p1[0], p2[1] - p1[1]
                if abs(ex * math.sin(phi) - ey * math.cos(phi)) > 1e-6 or ex * math.cos(phi) + ey * math.sin(phi) < -1e-6:
                    continue
                Ls = math.hypot(ex, ey)
                tot = r * a1 + Ls + r * a2
                if best is None or tot < best[0]:
                    best = (tot, t1, a1, phi, Ls, t2, a2)
    return best


def rejoin_steps(steps, actual, radius_mm=100.0, max_extra_mm=200.0):
    """実際のスタートの姿勢 actual=(x, y, 向き度) から、計画の合流点(join_index)まで、円弧-直進-円弧で
    つなぎ直した steps を返す。合流点がない・つなぎが計画より max_extra_mm 以上長いときは None。"""
    k = join_index(steps)
    if k is None:
        return None, "no join point"
    B = _integrate(steps, k)
    A = (actual[0], actual[1], math.radians(actual[2]))
    best = _dubins(A, (B[0], B[1], math.radians(B[2])), radius_mm)
    if best is None:
        return None, "no dubins path"
    tot, t1, a1, phi, Ls, t2, a2 = best
    old = sum(_step_len(s) for s in steps[:k])
    if tot > old + max_extra_mm:
        return None, "too long (%.0f mm vs %.0f mm)" % (tot, old)
    new = []
    if math.degrees(a1) > 0.05:
        new.append({"type": "arc", "target_heading_deg": round(-(actual[2] + t1 * math.degrees(a1)), 3),
                    "sweep_deg": round(-t1 * math.degrees(a1), 3), "radius_mm": radius_mm,
                    "reverse": False, "stop": False, "label": None})
    new.append({"type": "move", "target_heading_deg": round(-math.degrees(phi), 3), "distance_mm": round(Ls, 1),
                "reverse": False, "stop": False, "label": None})
    if math.degrees(a2) > 0.05:
        new.append({"type": "arc", "target_heading_deg": round(-B[2], 3), "sweep_deg": round(-t2 * math.degrees(a2), 3),
                    "radius_mm": radius_mm, "reverse": False, "stop": False, "label": None})
    return new + list(steps[k:]), "rejoin %d steps -> %d steps, %.0f mm (plan %.0f mm)" % (k, len(new), tot, old)


# ---------------- 走り出しへの取り込み(状態) ----------------
_measured = None          # EtRallyStartPoseCapture の結果 dict(pose=..., frames=..., ...)
_pending_rejoin = None    # EtRallyApplyStartPose が引き直しを決めたときの実際の姿勢 (x, y, 向き度)
_applied = None           # EtRallyApplyStartPose が取り込んだ姿勢(コース上の位置の走り出しに使う)


def measured_start_pose():
    return _measured


def applied_start_pose():
    """取り込んだスタートの姿勢 (x, y, 向き度)。取り込んでいなければ None。
    向きだけを取り込んだとき(計画の座標が分からないとき)は、位置は計画のスタートとする。"""
    return _applied


def apply_pending_rejoin(steps, logger=None):
    """計画を読み込むとき(DeferredStrategySequence)に通す。引き直しの予定があれば、つなぎを引き直した steps を返す。
    予定がない・引き直せないときは steps をそのまま返す(引き直せないときは、最初の区間の自分の位置に入れる)。"""
    global _pending_rejoin
    actual = _pending_rejoin
    _pending_rejoin = None
    if actual is None:
        return steps
    new, why = rejoin_steps(steps, actual)
    if new is None:
        _inject_position(actual, logger)
        if logger is not None:
            logger.warning("start pose: rejoin not possible (%s), injected the position instead" % why)
        return steps
    if logger is not None:
        logger.info("start pose: %s" % why)
    return new


def _inject_position(actual, logger=None):
    """位置のずれを、最初の区間の自分の位置(区間の始めの点が原点、x=進む向き、y=右)の初期値にする。"""
    from .behaviours import et_rally_path_segment
    dx, dy = actual[0] - START_MM[0], actual[1] - START_MM[1]
    a = math.radians(START_HEADING_DEG)
    front = dx * math.cos(a) + dy * math.sin(a)
    right = dx * math.sin(a) - dy * math.cos(a)
    et_rally_path_segment.set_start_position(front, right)
    if logger is not None:
        logger.info("start pose: first segment position set to front %+.1f right %+.1f mm" % (front, right))


def _marker_detector_factory(cam):
    def detect(frame):
        dets = detect_ground(cam, frame)
        # video.py の描画は markers[0]["center"] を使うので、形をそろえる(画像の座標ではないので描画は目安)。
        return [{"center": (0, 0), "ground": dets}]
    return detect


class EtRallyStartPoseCapture(Behaviour):
    """止まっている間に、カメラで位置と向きを出す。duration_sec の間に出せた分の中央値を使う。
    カメラがない・何も見えないときは、結果なし(走り出しは今までどおり計画のスタートにいるとみなす)。"""

    MAX_WAIT_SEC = 15.0   # ready を待つ上限(念のため)

    def __init__(self, name, duration_sec=2.0, calib_path=None, min_frames=3, ready=None):
        super().__init__(name)
        self.duration_sec = float(duration_sec)
        self.calib_path = calib_path
        self.min_frames = int(min_frames)
        # ready: 撮り始めてよいか(アームが動き終わったか)を返す関数。None ならすぐ撮り始める。
        self.ready = ready
        self.started = False
        self.counting_since = None

    def update(self):
        global _measured
        now = time.monotonic()
        if not self.started:
            self.started = True
            self.start_time = now
            self.results = []
            self.results_noscale = []
            self.last_frame = None
            self.frames = 0
            _measured = None
            video = getattr(runtime, "video", None)
            self.prev_mode = getattr(video, "target_interested", None)
            if video is None:
                self.logger.warning("start pose: no camera, skipped")
                return Status.SUCCESS
            try:
                cam = GroundCamera(self.calib_path)
                from py_etrobo_util import et_rally_marker_detector
                et_rally_marker_detector.set_detector(_marker_detector_factory(cam))
                video.begin_et_rally_marker_read()
            except Exception as error:   # 較正ファイルがない・cv2 がないなど
                self.logger.warning("start pose: camera setup failed (%r), skipped" % (error,))
                return Status.SUCCESS
            self.mirror = getattr(runtime, "course", -1) == 1
            return Status.RUNNING
        if self.counting_since is None:
            if self.ready is not None and not self.ready() and now - self.start_time < self.MAX_WAIT_SEC:
                return Status.RUNNING
            self.counting_since = now
        _, frame_id, markers = runtime.video.get_et_rally_marker_observation()
        if frame_id is not None and frame_id != self.last_frame and markers:
            self.last_frame = frame_id
            self.frames += 1
            dets = markers[0].get("ground") if isinstance(markers[0], dict) else None
            if dets:
                r = fit_pose(dets, mirror=self.mirror)
                if r is not None:
                    self.results.append(r)
                    # 縮尺を当てはめない場合も残す(実機で定規と比べて、どちらが合うか決めるため。11/08)
                    r0 = fit_pose(dets, mirror=self.mirror, fit_scale=False)
                    if r0 is not None:
                        self.results_noscale.append(r0)
        if now - self.counting_since < self.duration_sec:
            return Status.RUNNING
        return self._finish()

    def _finish(self):
        global _measured
        try:
            from py_etrobo_util import et_rally_marker_detector
            et_rally_marker_detector.set_detector(None)
        except Exception:
            pass
        from .et_rally_course_pose import restore_video_mode
        restore_video_mode(getattr(self, "prev_mode", None))
        if len(self.results) < self.min_frames:
            self.logger.warning("start pose: not enough frames (%d poses from %d frames), using the plan start" % (
                len(self.results), self.frames))
            return Status.SUCCESS
        pose = median_pose(self.results)
        _measured = dict(pose=pose, frames=self.frames, poses=len(self.results),
                         n_markers=max(r["n_markers"] for r in self.results), n_dots=max(r["n_dots"] for r in self.results),
                         spread_mm=max(math.hypot(r["pose"][0] - pose[0], r["pose"][1] - pose[1]) for r in self.results),
                         spread_deg=max(abs(_nd(r["pose"][2] - pose[2])) for r in self.results),
                         scale=sorted(r.get("scale", 1.0) for r in self.results)[len(self.results) // 2])
        self.logger.info("start pose: x=%.1f y=%.1f h=%+.2f deg (plan start %+.1f,%+.1f,%+.1f) poses=%d/%d frames "
                         "markers=%d dots=%d spread=%.1fmm/%.2fdeg scale=%.4f" % (
                             pose[0], pose[1], pose[2], pose[0] - START_MM[0], pose[1] - START_MM[1],
                             _nd(pose[2] - START_HEADING_DEG), len(self.results), self.frames,
                             _measured["n_markers"], _measured["n_dots"], _measured["spread_mm"], _measured["spread_deg"],
                             _measured["scale"]))
        p0 = median_pose(self.results_noscale)
        if p0 is not None:
            self.logger.info("start pose without scale fit: x=%.1f y=%.1f h=%+.2f deg (plan start %+.1f,%+.1f,%+.1f)" % (
                p0[0], p0[1], p0[2], p0[0] - START_MM[0], p0[1] - START_MM[1], _nd(p0[2] - START_HEADING_DEG)))
        return Status.SUCCESS


class EtRallyApplyStartPose(Behaviour):
    """測った位置と向きを、走り出しに取り込む。向きはジャイロに足し、位置は rejoin_min_mm 以上ずれていれば
    最初のつなぎを引き直す予定にし(計画の読み込み時に apply_pending_rejoin)、小さければ最初の区間の位置に入れる。
    max_offset_mm / max_offset_deg を超えるずれは、測り間違いとみなして取り込まない。"""

    def __init__(self, name, rejoin_min_mm=5.0, max_offset_mm=80.0, max_offset_deg=15.0, plan_frame_known=True):
        super().__init__(name)
        # plan_frame_known: 計画が経路を作る道具の座標(スタート START_MM・向き0、gyro = -向き)か。
        # 受信した計画(古い計画の道具)は座標の決め方が違うので、向きだけを取り込む。
        self.plan_frame_known = bool(plan_frame_known)
        self.rejoin_min_mm = float(rejoin_min_mm)
        self.max_offset_mm = float(max_offset_mm)
        self.max_offset_deg = float(max_offset_deg)

    def update(self):
        global _pending_rejoin, _applied
        _pending_rejoin = None
        _applied = None
        m = _measured
        if m is None:
            self.logger.info("start pose: not measured, start as planned")
            return Status.SUCCESS
        x, y, h = m["pose"]
        dx, dy, dth = x - START_MM[0], y - START_MM[1], _nd(h - START_HEADING_DEG)
        if math.hypot(dx, dy) > self.max_offset_mm or abs(dth) > self.max_offset_deg:
            self.logger.warning("start pose: offset too large (%+.1f,%+.1f mm %+.2f deg), ignored" % (dx, dy, dth))
            return Status.SUCCESS
        gyro = runtime.gyro_sensor
        if hasattr(gyro, "set_heading_offset"):
            # 計画の向き(heading = -course × ジャイロ)で -dth になるように、ジャイロの向きに course × dth を足す。
            gyro.set_heading_offset(getattr(runtime, "course", -1) * dth)
        else:
            self.logger.warning("start pose: gyro has no heading offset (EtRallyHeading not installed), heading not applied")
        _applied = (x, y, h)
        if not self.plan_frame_known:
            _applied = (START_MM[0], START_MM[1], h)
            how = "heading only (plan frame unknown)"
        elif math.hypot(dx, dy) >= self.rejoin_min_mm:
            _pending_rejoin = (x, y, h)
            how = "rejoin planned"
        else:
            _inject_position((x, y, h), self.logger)
            how = "position injected"
        self.logger.info("start pose: applied offset %+.1f,%+.1f mm %+.2f deg (%s)" % (dx, dy, dth, how))
        return Status.SUCCESS
