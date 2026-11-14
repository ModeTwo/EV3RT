"""ETラリー専用: スタートで止まっている間に、カメラでコース上の位置と向きを出し、走り出しに取り込む(2026-11-06)。

背景: 本番は前の工程からETラリーの開始位置へ移動してくるので、置かれた位置と向きが保証されない
(ユーザーの見込みで前後2cm・向き5°くらいまで)。机上のシミュレーションでは、知らずに走ると向き5°で
赤ゲートのあたりが3〜7cm外れる。知っていれば、向きはジャイロに足すだけで直り(1mm)、位置のずれは
最初のつなぎを引き直すと4〜11mm(取り込むだけだと13〜18mm)。

流れ:
  1. EtRallyStartPoseCapture: ジャイロの測り直しで止まっている間に、カメラの画像から
     マーカー(マスの中心、1辺約50mm)と灰色の点(格子の点、直径約44mm)を探し、位置と向きを出す。
     何枚か出して真ん中の値(中央値)を使う。
  2. EtRallyApplyStartPose: 計画のスタート(START_MM・START_HEADING_DEG)とのずれを、
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

try:
    from py_trees.behaviour import Behaviour
    from py_trees.common import Status
except ImportError:   # 2026-11-10: カメラの子プロセス(普通のPython、py_treesなし)では位置の計算だけを使う
    Behaviour = object
    Status = None

from .runtime import runtime

P_MM = 246.4                     # 格子の点の間隔(実測246mm、経路を作る道具は24.64cm)
CELLS = [((i + 0.5) * P_MM, (j + 0.5) * P_MM) for i in range(4) for j in range(4)]   # マーカー(マスの中心)
DOTS = [(i * P_MM, j * P_MM) for i in range(5) for j in range(5)]                      # 灰色の点

# 2026-11-10: ゲートの脚は灰色の点(マス目の交点)の上に立つ。脚の上の点は見えない(一部だけ見えると中心がずれる)ので
# 地図から外し、脚の近くで見つかった点・マーカーは、脚やT字の部品の見間違いのおそれがあるので使わない。
# 計画のファイルの gates(マス目の番号、計画の座標)から set_gate_legs で入れる。
LEG_EXCLUDE_MM = 40.0
_legs = []
_dots_visible = list(DOTS)


def set_gate_legs(gates):
    """gates: 計画のファイルの "gates"({色: {a: [i, j], b: [i, j]}})か None。脚の位置を覚える。"""
    global _legs, _dots_visible
    legs = []
    for color in ("red", "blue", "yellow"):
        g = (gates or {}).get(color)
        if isinstance(g, dict):
            for key in ("a", "b"):
                if key in g:
                    legs.append((float(g[key][0]) * P_MM, float(g[key][1]) * P_MM))
    _legs = legs
    _dots_visible = [d for d in DOTS if all(math.hypot(d[0] - l[0], d[1] - l[1]) > 1.0 for l in legs)]
    try:
        import numpy  # noqa: F401  2026-11-14: 引き直しのすき間の計算で初めて読み込むと、ロボットで約1秒かかった。準備のうちに読み込む
    except Exception:
        pass
    # 2026-11-14: 脚の T字の部品の長方形(経路を作る道具 hybrid_route._make_legs と同じ決まり)。スタートのつなぎの引き直しで、
    # 脚とのすき間を見るのに使う。(中心 x, y, ゲートの線の外向き u, 直角 v, u 方向の半分, v 方向の半分) mm
    global _leg_rects
    rects = []
    off_min = (GATE_INNER_MIN_MM + LEG_DIA_MM - P_MM) / 2.0
    off_max = (GATE_OUTER_MAX_MM - LEG_DIA_MM - P_MM) / 2.0
    s0, s1 = off_min - LEG_DIA_MM / 2.0, off_max + LEG_DIA_MM / 2.0
    for color in ("red", "blue", "yellow"):
        g = (gates or {}).get(color)
        if not (isinstance(g, dict) and "a" in g and "b" in g):
            continue
        pa = (float(g["a"][0]) * P_MM, float(g["a"][1]) * P_MM); pb = (float(g["b"][0]) * P_MM, float(g["b"][1]) * P_MM)
        L = math.hypot(pb[0] - pa[0], pb[1] - pa[1]) or 1.0
        t = ((pb[0] - pa[0]) / L, (pb[1] - pa[1]) / L); n = (-t[1], t[0])
        for p, out in ((pa, (-t[0], -t[1])), (pb, t)):
            c = (p[0] + out[0] * (s0 + s1) / 2.0, p[1] + out[1] * (s0 + s1) / 2.0)
            rects.append((c, out, n, (s1 - s0) / 2.0, LEG_LEN_MM / 2.0))
    _leg_rects = rects
    return legs


# 2026-11-14: 脚とT字の大きさ(経路を作る道具の gate_inner_min・gate_outer_max・leg_dia・leg_len と同じ)と、車体の箱(車軸から前・後ろ・横)
GATE_INNER_MIN_MM = 227.0
GATE_OUTER_MAX_MM = 295.0
LEG_DIA_MM = 25.0
LEG_LEN_MM = 75.0
BODY_FRONT_MM = 75.0
BODY_REAR_MM = 179.0
BODY_HALF_W_MM = 70.0
_leg_rects = []


def _near_leg(w):
    return any(math.hypot(w[0] - l[0], w[1] - l[1]) < LEG_EXCLUDE_MM for l in _legs)
# 計画のスタート(経路を作る道具と同じ座標)
# 2026-11-09: 新スタート = 今のゴール地点(hybrid_route.py の start_at='goal': x = 4P + 14.9cm、y = 3.5P、向き180)
START_MM = (4 * P_MM + 149.0, 3.5 * P_MM)
START_HEADING_DEG = 180.0

DEFAULT_CALIB = Path(__file__).resolve().parent.parent / "wireless_device" / "et_rally_camera_calib" / "calib_11141717.json"   # 2026-11-11 夜: カメラに触れて下向きが約1.3°変わったため較正し直した

# 真上から見た画像の範囲(車体の座標、mm)と1画素の大きさ。スタートでは止まっているので、少し広めに細かく見る。
TOP_S = 0.5
# 2026-11-10: カメラの子プロセス(ロボット)で使う粗さ。0.5mmだと1枚約1秒かかる(ロボットで計測)。1mmにすると
# PCで6.2倍速く、床の画像71枚で灰色の点は取りこぼし1/82・位置の差0.12mm以内、マーカーは取りこぼし15/99・差1.1mm以内。
FAST_TOP_S = 1.0
TOP_X = (300.0, 820.0)
TOP_Y = (-330.0, 330.0)
# 2026-11-12: スタートで、今の範囲に点が見えない(脚が立っている)ときだけ使う広い範囲。配置36はスタートから見える
# 4つの点すべてにゲートの脚が立ち、点0個(マーカー2つだけ)になって位置を取り込めなかった。広げると左右37cm・前87cmの
# 点が入る(スタートの画像で配置36は点0→1〜2個、seed は1→3個)。ただし端の点は左右が数mmずれる癖がある
# (左の遠い点で約-8mm)ので、今の範囲で足りるときは今の範囲だけを使う(choose_start_fit)。
START_WIDE_X = (300.0, 950.0)
START_WIDE_Y = (-430.0, 430.0)
START_MIN_ITEMS_INNER = 3        # 今の範囲で点1つ以上かつ見えたもの3つ以上なら、今の範囲だけを使う
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

    def __init__(self, calib_path=None, top_x=TOP_X, top_y=TOP_Y, top_s=TOP_S, extra_pitch_deg=0.0):
        import cv2
        import numpy as np
        self.cv2, self.np = cv2, np
        self.calib_path = calib_path
        d = json.load(open(calib_path or DEFAULT_CALIB, encoding="utf-8"))
        K = np.array(d["lens"]["K"]); dist = np.array(d["lens"]["dist"])
        j = d["jig"][0]
        rv = np.array(j["rvec"], dtype=float); t = np.array(j["tvec"], dtype=float).reshape(3)
        # 2026-11-10: 較正台での左右の向きのずれの補正(度)。コースの線にぴったり置いた画像で、カメラが出す向きが
        # 約+1.3°ずれていた。車体の座標を車軸の真ん中のまわりにこの角度だけ回してから、カメラへ写す。
        yaw_fix = float(d.get("yaw_fix_deg", 0.0))
        if yaw_fix:
            c_, s_ = math.cos(math.radians(yaw_fix)), math.sin(math.radians(yaw_fix))
            Rz = np.array([[c_, -s_, 0.0], [s_, c_, 0.0], [0.0, 0.0, 1.0]])
            rv = cv2.Rodrigues(cv2.Rodrigues(rv)[0] @ Rz)[0].ravel()
        self.yaw_fix = yaw_fix
        # 2026-11-10: 床での下向きの補正(度)。較正台の上とマットの上でロボットの前後の傾きが違い、床では遠いものが
        # 遠く・近いものが近く見えていた(前後に約6%伸びる)。床で撮った8枚の当てはめで -0.46°(ばらつき0.11°)。
        # カメラの位置は変えずに、車体の左右の軸のまわりに回す(fit_pose_pitch と同じ回し方)。
        # 2026-11-11: extra_pitch_deg = スタート位置の取り込みで当てはめた下向き(走行中の補正に使う)。同じ回し方で足す。
        pitch_fix = float(d.get("pitch_fix_deg", 0.0)) + float(extra_pitch_deg or 0.0)
        if pitch_fix:
            R = cv2.Rodrigues(np.asarray(rv, float).reshape(3, 1))[0]
            C = -R.T @ t
            c_, s_ = math.cos(math.radians(pitch_fix)), math.sin(math.radians(pitch_fix))
            R2 = R @ np.array([[c_, 0.0, s_], [0.0, 1.0, 0.0], [-s_, 0.0, c_]])
            rv = cv2.Rodrigues(R2)[0].ravel()
            t = -R2 @ C
        self.pitch_fix = pitch_fix
        # 2026-11-10: カメラの左右の位置の補正(mm、車体の左が+)。止まって撮った画像でも走行中でも、カメラは
        # ロボットを毎回同じ向き(左)に数mmずれて見ていた(較正台でボードが左右にずれていた分などが入っていない)。
        # カメラの位置だけを車体の左右に動かす(向きは変えない)。
        lat_fix = float(d.get("lat_fix_mm", 0.0))
        # 2026-11-13: カメラの前後の位置の補正(mm、車体の前が+)。calib_11131404 では、止まって撮った8回すべてで
        # ロボットを4〜6mm 前に見ていた(較正台から出したカメラの前後の位置が、床で使うと数mm 後ろすぎる)。
        front_fix = float(d.get("front_fix_mm", 0.0))
        if lat_fix or front_fix:
            R = cv2.Rodrigues(np.asarray(rv, float).reshape(3, 1))[0]
            C = -R.T @ t
            t = -R @ (C + np.array([front_fix, lat_fix, 0.0]))
        self.lat_fix = lat_fix
        self.front_fix = front_fix
        # 2026-11-13: 床での較正(wireless_device/et_rally_camera_calib/floor_calib.py)で求めた補正。カメラの高さ(mm、上が+)と、
        # カメラの向きを車体の軸まわりに回す量(前後の軸まわり roll、上下の軸まわり cam_yaw、度)。カメラの位置は高さ以外変えない。
        h_fix = float(d.get("height_fix_mm", 0.0)); roll = float(d.get("roll_fix_deg", 0.0)); cyaw = float(d.get("cam_yaw_fix_deg", 0.0))
        if h_fix or roll or cyaw:
            R = cv2.Rodrigues(np.asarray(rv, float).reshape(3, 1))[0]
            C = -R.T @ t + np.array([0.0, 0.0, h_fix])
            cr, sr = math.cos(math.radians(roll)), math.sin(math.radians(roll))
            cz, sz = math.cos(math.radians(cyaw)), math.sin(math.radians(cyaw))
            Rb = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]]) @ np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
            R = R @ Rb.T
            rv = cv2.Rodrigues(R)[0].ravel()
            t = -R @ C
        self.floor_fix = (h_fix, roll, cyaw)
        self.top_x, self.top_y, self.top_s = top_x, top_y, top_s
        # 下向きも当てはめる位置の出し方(fit_pose_pitch)で使う
        self.K, self.dist, self.rvec, self.tvec = K, dist, rv, t
        self.R0 = cv2.Rodrigues(rv)[0]
        self.C0 = -self.R0.T @ t
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


def camera_center(calib_path=None):
    """2026-11-12: カメラの位置(車体の座標 mm、前・左・上)を較正ファイルから出す(cv2 を使わない。走行中の補正の親プロセス用)。
    GroundCamera と同じく、左右の向きの補正 yaw_fix で車軸の真ん中のまわりに回し、lat_fix だけ左右に動かす(下向きの補正では位置は変わらない)。"""
    import numpy as np
    d = json.load(open(calib_path or DEFAULT_CALIB, encoding="utf-8"))
    j = d["jig"][0]
    rv = np.array(j["rvec"], dtype=float).reshape(3); t = np.array(j["tvec"], dtype=float).reshape(3)
    th = float(np.linalg.norm(rv))
    if th < 1e-12:
        R = np.eye(3)
    else:
        k = rv / th
        K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        R = np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * (K @ K)
    C = -R.T @ t
    yaw = math.radians(float(d.get("yaw_fix_deg", 0.0)))
    if yaw:
        c_, s_ = math.cos(yaw), math.sin(yaw)
        C = np.array([[c_, s_, 0.0], [-s_, c_, 0.0], [0.0, 0.0, 1.0]]) @ C   # Rz^T C
    C = C + np.array([float(d.get("front_fix_mm", 0.0)), float(d.get("lat_fix_mm", 0.0)), float(d.get("height_fix_mm", 0.0))])
    return tuple(float(v) for v in C)


def repitch(xy, dpitch_deg, C):
    """下向きの補正なしのカメラで見た床の点 (前, 左) を、下向きを dpitch_deg 度足したカメラで見た位置に直す
    (GroundCamera の extra_pitch_deg と同じ回し方。画像から作り直した場合と0.3mm以内で一致、11/12)。"""
    dx, dy, dz = xy[0] - C[0], xy[1] - C[1], -C[2]
    a = math.radians(dpitch_deg)
    c_, s_ = math.cos(a), math.sin(a)
    ex, ez = c_ * dx - s_ * dz, s_ * dx + c_ * dz
    if ez >= -1e-9:
        return None
    k = -C[2] / ez
    return (C[0] + k * ex, C[1] + k * dy)


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
    if S >= 0.75:
        # 粗い画像(子プロセス用)では、30mm四方の中央値も4分の1に縮めた画像でとる(一番重い処理のため)
        sm = cv2.resize(hsv[..., 2], None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)
        k_bg = int(round(30.0 / (S * 4))) | 1
        bg = cv2.resize(cv2.medianBlur(sm, max(3, k_bg)), (v.shape[1], v.shape[0]), interpolation=cv2.INTER_LINEAR).astype(np.int16)
    else:
        k_bg = int(round(30.0 / S)) | 1
        bg = cv2.medianBlur(hsv[..., 2], min(k_bg, 255)).astype(np.int16)
    low_sat = hsv[..., 1] < 150
    out = {"markers": [], "dots": []}
    # マーカー
    dark = (((bg - v) > 25) & low_sat).astype(np.uint8) * 255
    marker_mask = np.zeros(dark.shape, np.uint8)
    # 2026-11-12: 遠い(約75cm)マーカーは粗い画像で暗い所が細かく割れ、6mm の閉じでは1つにならないことがあった
    # (配置36のスタートで前へ2cm置いた回)。6mm で見つからなかった所を、10mm の閉じでもう一度探す。
    for close_mm in (6.0, 10.0):
        _find_markers(cam, dark, close_mm, out, marker_mask)
    # 灰色の点: 点(直径44mm)より広い約100mm四方を明るさの基準にする(30mmだと点の中が基準になり縁しか残らない)。
    # 重いので、4分の1に縮めた画像で中央値をとってから元の大きさに戻す。
    small = cv2.resize(hsv[..., 2], None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)
    k_dot = int(round(100.0 / (S * 4))) | 1
    bg_dot = cv2.resize(cv2.medianBlur(small, min(k_dot, 255)), (v.shape[1], v.shape[0]), interpolation=cv2.INTER_LINEAR).astype(np.int16)
    gray = (((bg_dot - v) > 10) & low_sat & (marker_mask == 0)).astype(np.uint8) * 255
    gray = cv2.morphologyEx(gray, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, ((int(round(2.5 / S)) | 1),) * 2))
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
        ring = [(ccx + (rr + 1.5 / S) * math.cos(a), ccy + (rr + 1.5 / S) * math.sin(a)) for a in np.linspace(0, 2 * math.pi, 16, endpoint=False)]
        if not _inside(cam, ring):
            continue
        out["dots"].append(tuple(float(x) for x in cam.to_mm([(cen[i][0], cen[i][1])])[0]))
    return out


def _find_markers(cam, dark, close_mm, out, marker_mask):
    """detect_ground のマーカー探し。close_mm の閉じで暗い所をつなぎ、まだ marker_mask にない所だけ out に足す。"""
    cv2, np = cam.cv2, cam.np
    S = cam.top_s
    k = max(1, int(round(close_mm / S)))
    blob = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (k, k)))
    if marker_mask.any():
        blob[marker_mask > 0] = 0
    n, lab, st, _ = cv2.connectedComponentsWithStats(blob)
    for i in range(1, n):
        if not ((35.0 / S) ** 2 <= st[i, 4] <= (62.0 / S) ** 2):
            continue
        ys, xs = np.nonzero((lab == i) & (dark > 0))
        if len(xs) < 10 * (0.5 / S) ** 2:
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
            if _near_leg(w):
                continue
            c = min(CELLS, key=lambda q: math.hypot(q[0] - w[0], q[1] - w[1]))
            if math.hypot(c[0] - w[0], c[1] - w[1]) <= ASSOC_MARKER_MM:
                pairs.append((m["xy"], c, 2.0, m))
        for d in dots:
            w = _to_world(pose, d)
            if _near_leg(w) or not _dots_visible:
                continue
            c = min(_dots_visible, key=lambda q: math.hypot(q[0] - w[0], q[1] - w[1]))
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
            marks_used = [p[3] for p in pairs if p[3] is not None]
            if len(pairs) == 2 and marks_used:
                # 2026-11-10: 2つだけ(マーカー1つ+点1つなど)だと、並びの向きはどちらかの少しのずれで大きく動く
                # (実機: 右のタイヤだけ15mm下げた置き方で、並びから右回り4.6°・マーカーの辺から6.2°、置き方は6.8°)。
                # このときは向きをマーカーの辺の傾きから出す(90°ごとのうち、並びの向きに近いもの)。
                hs = []
                for mk in marks_used:
                    base = -mk["rot"]
                    hs.append(base + 90.0 * round((h - base) / 90.0))
                h = math.degrees(math.atan2(sum(math.sin(math.radians(v)) for v in hs), sum(math.cos(math.radians(v)) for v in hs)))
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


def _score(r):
    return (r["n_dots"] + 2 * r["n_markers"], -r["resid_mm"])


def fit_pose_search(dets, nominal=(START_MM[0], START_MM[1], START_HEADING_DEG), mirror=False, max_dh=20.0, max_dxy=40.0):
    """2026-11-10: 予想(計画のスタート)から向きが大きくずれていても結びつくように、予想の向きと位置を少しずつ変えて
    fit_pose(縮尺なし)を試し、一番多く結びつくもの(同じなら残差の小さいもの)を返す。
    実機で向き約+9°・前に25mmずらして置いたら、予想のままでは50〜60cm先のマーカーが約9cm離れて見え、1つも結びつかなかった。
    向きの候補: 予想、マーカーの辺の傾きから出した向き(90°ごとのうち予想に近いもの)、予想±max_dh を4°おき。
    位置の候補: 予想と ±max_dxy。予想から max_dh+5°・100mm より離れた答えは、隣のマスとの取り違えとみなして使わない。"""
    sgn = -1.0 if mirror else 1.0
    x0, y0, h0 = nominal
    heads = [h0]
    for m in dets.get("markers", []):
        base = -sgn * m["rot"]
        h = base + 90.0 * round((h0 - base) / 90.0)
        if abs(_nd(h - h0)) <= max_dh:
            heads.append(h)
    k = int(max_dh // 4)
    heads += [h0 + 4.0 * i for i in range(-k, k + 1) if i]
    best = None
    for h in heads:
        for dx in (0.0, -max_dxy, max_dxy):
            for dy in (0.0, -max_dxy, max_dxy):
                r = fit_pose(dets, nominal=(x0 + dx, y0 + dy, h), mirror=mirror, fit_scale=False)
                if r is None or r["n_dots"] + r["n_markers"] < 2:
                    continue
                p = r["pose"]
                if math.hypot(p[0] - x0, p[1] - y0) > 100.0 or abs(_nd(p[2] - h0)) > max_dh + 5.0:
                    continue
                if best is None or _score(r) > _score(best):
                    best = r
    return best


def fit_pose_pitch(cam, dets, nominal=(START_MM[0], START_MM[1], START_HEADING_DEG), mirror=False, iters=40):
    """2026-11-08: 置き場所ごとに、カメラの下向きも一緒に当てはめて位置と向きを出す。
    マットの上では置き場所でロボットの前後の傾きが変わり、カメラの下向きが約0.2〜0.8°変わる(50〜70cm先で1.5〜2cm)。
    動かした距離が分かっている5か所(前に0/30/60/100mm、右に20mm)で、動きのずれが
    縮尺なし9〜18mm・縮尺あり9〜17mm に対して、この方法は1.8〜4.1mm。
    まず fit_pose(縮尺なし)で、見えたものをマス・点に結びつけ、その画素の位置に合うように
    (下向きの補正, x, y, 向き) を当てはめる(数値微分のガウス・ニュートン、減衰つき)。
    3点以上結びつかないとき・cam がないときは fit_pose(縮尺なし)の結果を返す。
    戻り値: fit_pose と同じ dict に pitch_deg(下向きの補正)と rms_px を足したもの。"""
    r0 = fit_pose_search(dets, nominal=nominal, mirror=mirror)
    if r0 is None or cam is None or not hasattr(cam, "K"):
        return r0
    cv2, np = cam.cv2, cam.np
    sgn = -1.0 if mirror else 1.0
    pose = r0["pose"]
    items = [(m["xy"], CELLS, ASSOC_MARKER_MM) for m in dets.get("markers", [])] + [(d, _dots_visible, ASSOC_DOT_MM) for d in dets.get("dots", [])]
    body_pts, world = [], []
    for xy, ref, lim in items:
        w = _to_world(pose, (xy[0], sgn * xy[1]))
        if _near_leg(w) or not ref:
            continue
        c = min(ref, key=lambda q: math.hypot(q[0] - w[0], q[1] - w[1]))
        if math.hypot(c[0] - w[0], c[1] - w[1]) <= lim:
            body_pts.append([xy[0], xy[1], 0.0]); world.append(c)
    if len(world) < 3:
        return r0
    px0, _ = cv2.projectPoints(np.array(body_pts, float), cam.rvec, cam.tvec, cam.K, cam.dist)
    px0 = px0.reshape(-1, 2)

    def res(p):
        c, s_ = math.cos(p[0]), math.sin(p[0])
        R = cam.R0 @ np.array([[c, 0, s_], [0, 1, 0], [-s_, 0, c]])
        t = -R @ cam.C0
        a = math.radians(p[3])
        body = [[(w[0] - p[1]) * math.cos(a) + (w[1] - p[2]) * math.sin(a),
                 sgn * (-(w[0] - p[1]) * math.sin(a) + (w[1] - p[2]) * math.cos(a)), 0.0] for w in world]
        px, _ = cv2.projectPoints(np.array(body, float), cv2.Rodrigues(R)[0], t, cam.K, cam.dist)
        return (px.reshape(-1, 2) - px0).ravel()

    p = np.array([0.0, pose[0], pose[1], pose[2]])
    r = res(p)
    lam = 1e-3
    steps = (1e-5, 1e-3, 1e-3, 1e-4)
    for _ in range(iters):
        J = np.zeros((len(r), 4))
        for j in range(4):
            q = p.copy(); q[j] += steps[j]
            J[:, j] = (res(q) - r) / steps[j]
        A = J.T @ J
        step = np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-9), -J.T @ r)
        q = p + step
        rq = res(q)
        if (rq ** 2).sum() < (r ** 2).sum():
            p, r = q, rq
            lam = max(lam / 3.0, 1e-7)
            if np.abs(step).max() < 1e-6:
                break
        else:
            lam *= 5.0
    if abs(math.degrees(p[0])) > 3.0:   # 当てはめが外れた(ありえない傾き)ときは使わない
        return r0
    out = dict(r0)
    out.update(pose=(float(p[1]), float(p[2]), _nd(float(p[3]))), pitch_deg=math.degrees(p[0]),
               rms_px=float(math.sqrt((r ** 2).mean())), scale=1.0)
    # 2026-11-12: 食い違い(resid_mm)を、当てはめた下向きで床に写し直して測る。それまでは下向きを直す前の
    # 位置で測っていて、下向きの補正が大きい回(+0.65°)では、実際は3mm以内でも19mm と出ていた。
    try:
        c, s_ = math.cos(p[0]), math.sin(p[0])
        R = cam.R0 @ np.array([[c, 0, s_], [0, 1, 0], [-s_, 0, c]])
        und = cv2.undistortPoints(px0.reshape(-1, 1, 2).astype(np.float64), cam.K, cam.dist).reshape(-1, 2)
        a = math.radians(p[3])
        res_mm = []
        for (u, v), w in zip(und, world):
            d = R.T @ np.array([u, v, 1.0])
            if abs(d[2]) < 1e-9:
                raise ValueError
            k = -cam.C0[2] / d[2]
            bx, by = cam.C0[0] + k * d[0], sgn * (cam.C0[1] + k * d[1])
            wx = p[1] + bx * math.cos(a) - by * math.sin(a); wy = p[2] + bx * math.sin(a) + by * math.cos(a)
            res_mm.append(math.hypot(wx - w[0], wy - w[1]))
        out["resid_mm"] = float(max(res_mm))
    except Exception:
        pass
    return out


def _inner_dets(dets):
    """広い範囲で見つけたもののうち、今の範囲(TOP_X・TOP_Y)に入るものだけ。"""
    def ok(xy):
        return TOP_X[0] <= xy[0] <= TOP_X[1] and TOP_Y[0] <= xy[1] <= TOP_Y[1]
    return {"markers": [m for m in dets.get("markers", []) if ok(m["xy"])],
            "dots": [d for d in dets.get("dots", []) if ok(d)]}


def choose_start_fit(cam, dets, mirror=False):
    """2026-11-12: スタートの位置。dets は広い範囲(START_WIDE_X/Y)で見つけたもの。
    今の範囲だけで点1つ以上・見えたもの START_MIN_ITEMS_INNER 以上なら今の範囲の答え(今までと同じ)、
    足りなければ広い範囲も使った答え(結びついたものが多い方)を返す。戻り値: (fit_pose_pitch の結果, 縮尺なしの結果, 広い範囲を使ったか)。"""
    # 11/12 実機: 当てはめを何度も呼ぶと、ロボットで2秒に4枚しか測れなかった(今までは約12枚)。
    # 縮尺なしの結果(比べる用)は出さず、外側に何もないときは1回だけ当てはめる。
    inner = _inner_dets(dets)
    r_in = fit_pose_pitch(cam, inner, mirror=mirror)
    if r_in is not None and r_in["n_dots"] >= 1 and r_in["n_dots"] + r_in["n_markers"] >= START_MIN_ITEMS_INNER:
        return r_in, None, False
    if len(inner["markers"]) == len(dets.get("markers", [])) and len(inner["dots"]) == len(dets.get("dots", [])):
        r, used, wide = r_in, inner, False
    else:
        r_w = fit_pose_pitch(cam, dets, mirror=mirror)
        if r_w is not None and (r_in is None or (r_w["n_dots"], r_w["n_dots"] + r_w["n_markers"]) > (r_in["n_dots"], r_in["n_dots"] + r_in["n_markers"])):
            r, used, wide = r_w, dets, True
        else:
            r, used, wide = r_in, inner, False
    if r is not None and r["n_dots"] == 0 and r["n_markers"] >= 2 and r["n_markers"] < 3:
        r = _fit_markers_assumed_pitch(cam, used, mirror) or r
    elif r is not None and r["n_dots"] + r["n_markers"] < 3:
        # 2026-11-14: 見えたものが2つ(点1つ+マーカー1つなど)では下向きを当てはめられず、較正のままの下向きで前後を
        # 大きく間違えた(配置36: 前後 -47mm・食い違い33mm と出て、そのまま取り込んでぶつかった。カメラの下向きが較正より
        # 約1°大きくなっていた)。保存した下向きで点もマーカーも写し直して当てはめる。
        r = _fit_markers_assumed_pitch(cam, used, mirror, with_dots=True) or r
    return r, None, wide


START_ASSUMED_PITCH_DEG = 0.45   # 保存した下向きがないときの決め打ち(11/12 のスタートで当てはめた値 +0.12〜+0.65 の真ん中あたり)


def _fit_markers_assumed_pitch(cam, dets, mirror=False, with_dots=False):
    """2026-11-12: 点が見えずマーカー2つだけのとき、前回床で当てはめて保存した下向き(なければ START_ASSUMED_PITCH_DEG)で
    写し直して位置を出す。下向きを当てはめられないと前後を2〜4cm 間違えていたが、+0.45° の決め打ちで、点も見えた8回の画像の
    答えとの差は前後 ±12mm 以内・左右 ±11mm 以内になった。結果に pitch_assumed を付ける(取り込みでは位置も入れる)。"""
    last = load_last_pitch(getattr(cam, "calib_path", None))
    p = last[0] if (last is not None and abs(last[0]) <= 1.5) else START_ASSUMED_PITCH_DEG
    C = (float(cam.C0[0]), float(cam.C0[1]), float(cam.C0[2]))
    mk = []
    for m in dets.get("markers", []):
        q = repitch(m["xy"], p, C)
        if q is None:
            return None
        mk.append(dict(m, xy=q))
    dts = []
    if with_dots:
        for d in dets.get("dots", []):
            q = repitch(d, p, C)
            if q is None:
                return None
            dts.append(tuple(q))
    r = fit_pose_search({"markers": mk, "dots": dts}, mirror=mirror)
    if r is None or (r["n_markers"] + (r["n_dots"] if with_dots else 0)) < 2:
        return None
    return dict(r, pitch_assumed=p)


def median_pose(results):
    """何枚分かの fit_pose の結果から、それぞれの真ん中の値(向きは予想まわりで)。"""
    if not results:
        return None
    xs = sorted(r["pose"][0] for r in results); ys = sorted(r["pose"][1] for r in results)
    # 11/12: スタートの向きは約180°で、+179.9 と -179.9 が混ざると素直に並べた真ん中が0°になっていた
    # (取り込みが「ずれが大きい」で捨てられた)。スタートの向きからの差(-180〜180)で真ん中を取る。
    hs = sorted(_nd(r["pose"][2] - START_HEADING_DEG) for r in results)
    mid = len(results) // 2
    pick = (lambda v: v[mid]) if len(results) % 2 else (lambda v: (v[mid - 1] + v[mid]) / 2.0)
    return (pick(xs), pick(ys), _nd(pick(hs) + START_HEADING_DEG))


# ---------------- 最初のつなぎの引き直し ----------------
def _integrate(steps, k, start=(START_MM[0], START_MM[1], START_HEADING_DEG)):
    """計画の steps[:k] を積算した姿勢 (x, y, 向き度)。円弧・直進だけを想定する。"""
    x, y, h = start
    for s in steps[:k]:
        if s["type"] == "move":
            hh = START_HEADING_DEG - float(s["target_heading_deg"]); th = math.radians(hh + (180.0 if s.get("reverse") else 0.0))
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


def _plan_heading(h):
    """コースの向き(度)→計画の向き(target_heading_deg)。スタートの向きが計画の0で、右回りが+。"""
    return round(_nd(START_HEADING_DEG - h), 3)


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


def _sample_steps(steps, start, step_mm=15.0):
    """steps(前進の直進・円弧)を start=(x, y, 向き度) から積算した、車軸の姿勢の列 [(x, y, 向きrad)]。"""
    x, y, h = start
    out = [(x, y, math.radians(h))]
    for s in steps:
        if s["type"] == "move":
            hh = START_HEADING_DEG - float(s["target_heading_deg"]); th = math.radians(hh + (180.0 if s.get("reverse") else 0.0))
            L = float(s["distance_mm"]); n = max(1, int(L / step_mm))
            for k in range(1, n + 1):
                out.append((x + L * k / n * math.cos(th), y + L * k / n * math.sin(th), math.radians(hh)))
            x, y, h = x + L * math.cos(th), y + L * math.sin(th), hh
        elif s["type"] == "arc":
            sw = -float(s["sweep_deg"]); r = float(s["radius_mm"]); sg = 1.0 if sw > 0 else -1.0
            th0 = math.radians(h + (180.0 if s.get("reverse") else 0.0))
            cx, cy = x - sg * r * math.sin(th0), y + sg * r * math.cos(th0)
            n = max(2, int(r * math.radians(abs(sw)) / step_mm), int(abs(sw) / 3.0))
            for k in range(1, n + 1):
                t1 = th0 + math.radians(sw) * k / n
                out.append((cx + sg * r * math.sin(t1), cy - sg * r * math.cos(t1), math.radians(h + sw * k / n)))
            x, y, h = out[-1][0], out[-1][1], h + sw
        elif s["type"] == "turn":
            # その場旋回: 車軸より PIVOT_BACK_MM 後ろの点を中心に回る(経路を作る道具の pivot_back_cm と同じ)
            hh = START_HEADING_DEG - float(s["target_heading_deg"]); d = _nd(hh - h)
            cx, cy = x - PIVOT_BACK_MM * math.cos(math.radians(h)), y - PIVOT_BACK_MM * math.sin(math.radians(h))
            n = max(1, int(abs(d) / 3.0))
            for k in range(1, n + 1):
                a = math.radians(h + d * k / n)
                out.append((cx + PIVOT_BACK_MM * math.cos(a), cy + PIVOT_BACK_MM * math.sin(a), a))
            x, y, h = out[-1][0], out[-1][1], h + d
        else:
            break
    return out


PIVOT_BACK_MM = 5.0


def body_leg_clearance(poses):
    """車体の箱と脚の T字の長方形の、いちばん小さいすき間(mm)。脚が分からないときは大きな値。"""
    if not _leg_rects or not poses:
        return 1e9
    import numpy as np
    # 2026-11-14: 脚6本をまとめて numpy で計算する(1本ずつだと、ロボットで1回約8ms かかり、引き直しの探索が間に合わなかった)
    C = np.array([r[0] for r in _leg_rects], float); U = np.array([r[1] for r in _leg_rects], float); V = np.array([r[2] for r in _leg_rects], float)
    HU = np.array([r[3] for r in _leg_rects], float); HV = np.array([r[4] for r in _leg_rects], float)
    Q = np.concatenate([C + U * (a * HU)[:, None] + V * (b * HV)[:, None] for a in (-1, 1) for b in (-1, 1)])   # 脚の角 (4n, 2)
    P = np.array(poses, float)
    px, py, ph = P[:, 0:1], P[:, 1:2], P[:, 2:3]
    c, s = np.cos(ph), np.sin(ph)
    # 脚の角 → 車体の箱
    dx = Q[None, :, 0] - px; dy = Q[None, :, 1] - py
    lx = dx * c + dy * s; ly = -dx * s + dy * c
    d1 = np.hypot(np.maximum(np.maximum(-BODY_REAR_MM - lx, lx - BODY_FRONT_MM), 0.0), np.maximum(np.abs(ly) - BODY_HALF_W_MM, 0.0)).min()
    # 車体の角 → 脚の長方形
    bx = np.array([BODY_FRONT_MM, BODY_FRONT_MM, -BODY_REAR_MM, -BODY_REAR_MM]); by = np.array([BODY_HALF_W_MM, -BODY_HALF_W_MM, -BODY_HALF_W_MM, BODY_HALF_W_MM])
    wx = (px + bx * c - by * s)[:, :, None] - C[None, None, :, 0]; wy = (py + bx * s + by * c)[:, :, None] - C[None, None, :, 1]
    du = np.abs(wx * U[:, 0] + wy * U[:, 1]) - HU; dv = np.abs(wx * V[:, 0] + wy * V[:, 1]) - HV
    d2 = np.hypot(np.maximum(du, 0.0), np.maximum(dv, 0.0)).min()
    return float(min(d1, d2))


REJOIN_CLEAR_MM = 30.0   # 2026-11-14: 引き直したつなぎで、脚とのすき間をこれ以上にしたい(計画そのものがこれより狭いときは計画のすき間まで)
REJOIN_PIVOTS_DEG = (10.0, -10.0, 20.0, -20.0, 30.0, -30.0, 45.0, -45.0)   # 円弧だけでは足りないとき、最初にその場で回る角度の候補
REJOIN_PIVOT_COST_MM = 150.0   # その場旋回を入れる候補の、比べるときの上乗せ(止まって回るぶんの時間を道のりに換算した目安)。後退は これ + 後退の2倍
REJOIN_BACK_MM = (40.0, 80.0)  # 円弧だけでは足りないとき、最初に真後ろへ下がる距離の候補(スタートの後ろは広い)
REJOIN_TIME_LIMIT_SEC = 1.5    # 候補のすき間を測るのをこの時間で打ち切り、それまでで一番よいものを使う。
                               # 11/14 実機: 5秒にしていたら、その場旋回まで探した回は毎回5.2秒かかり(打ち切り)、1回はその直後に
                               # 車輪の値が止まったまま回り続けた。合流点から先を1回だけ測るようにして短くし、1.5秒で打ち切る。
                               # 11/14: 0.8秒では、PCが混んでいると、その場旋回・後退の候補にたどり着く前に打ち切られた。大きくずれたときだけの処理なので待つ


def rejoin_steps(steps, actual, radius_mm=100.0, max_extra_mm=200.0):
    """2026-11-14: 脚とのすき間を見て引き直す。実際のスタートの姿勢 actual から、最初のゲートより前の合流点まで、
    円弧-直進-円弧でつなぐ候補を作る(合流点: 区間の終わり、長い直進の途中。半径 80〜200mm。足りなければ最初にその場旋回)。
    各候補で、引き直した道と、その先の最初のゲートを抜けるまでの、車体と脚の T字のすき間を測る。
    目標のすき間 = min(REJOIN_CLEAR_MM, 計画どおりにスタートしたときの同じ範囲のすき間 - 1mm)。目標以上の候補のうち
    一番短いもの(その場旋回は REJOIN_PIVOT_COST_MM を足して比べる)、なければすき間が一番大きいものを使う。
    脚が分からない(計画に gates がない)ときは、従来どおりの1通り。
    11/14: 配置63で、スタートが黄の側に4cm ずれていると、従来の引き直しでは走り出しで黄の脚の T字に当たる計算だった。"""
    if not _leg_rects:
        return _rejoin_simple(steps, actual, radius_mm, max_extra_mm)
    k0 = join_index(steps)
    if k0 is None:
        return None, "no join point"
    # 最初のゲート(label のある区間)の終わりまでを、すき間を見る範囲にする
    lab = next((i for i, s in enumerate(steps) if s.get("label")), None)
    end_check = (lab + 1) if lab is not None else min(len(steps), k0 + 3)
    plan_clr = body_leg_clearance(_sample_steps(steps[:end_check], (START_MM[0], START_MM[1], START_HEADING_DEG)))
    need = min(REJOIN_CLEAR_MM, plan_clr - 1.0)
    # 置いた位置がすでに脚に近いとき: 最初の後退・その場旋回の間は置いた位置より近づかないこと、そのあとの道で need を見る
    start_clr = body_leg_clearance([(actual[0], actual[1], math.radians(actual[2]))])
    # まず従来の引き直し(合流点・半径が決まった1通り)。脚とのすき間が目標以上なら、それをそのまま使う(実機で使ってきた形)
    simple, why_s = _rejoin_simple(steps, actual, radius_mm, max_extra_mm)
    if simple is not None:
        n_new = len(simple) - (len(steps) - k0)
        clr_s = body_leg_clearance(_sample_steps(simple[:n_new + max(0, end_check - k0)], actual)[1:])
        if clr_s >= need:
            return simple, "%s, leg clearance %.1f mm (target %.1f)" % (why_s, clr_s, need)
    run_end = next((i for i, s in enumerate(steps) if s["type"] not in ("move", "arc") or s.get("reverse") or s.get("label") or s.get("stop")), len(steps))
    cands = []   # (合流点の姿勢, 合流点までの元の長さ, 合流点のあとの steps)
    acc = 0.0
    for i in range(run_end):
        s = steps[i]
        L = _step_len(s)
        if s["type"] == "move" and L >= 150.0:
            for d in (80.0, 120.0, 160.0, 220.0, 300.0, 400.0):
                if d < L - 40.0:
                    rest = dict(s, distance_mm=round(L - d, 1))
                    B = _integrate(steps[:i] + [dict(s, distance_mm=d)], i + 1)
                    cands.append((B, acc + d, [rest] + list(steps[i + 1:])))
        acc += L
        if i + 1 < len(steps) and i + 1 <= run_end:
            cands.append((_integrate(steps, i + 1), acc, list(steps[i + 1:])))
    # 2026-11-14: 合流点から先(計画どおりの道)のすき間は、どの引き直しでも同じなので、合流点ごとに1回だけ測る
    # (ロボットで、その場旋回・後退まで探すと5.2秒かかり、その間に走行の制御が止まっていた)
    tail_clr = {}
    for ci, (B, old, tail) in enumerate(cands):
        n_tail = max(0, end_check - (len(steps) - len(tail)))
        tail_clr[ci] = body_leg_clearance(_sample_steps(tail[:n_tail], B)) if n_tail else 1e9

    def search(pres):
        # 道のりの計算は軽いので先に全部出し、短い順にすき間を測って、目標に届いた最初の候補で打ち切る(ロボットで速くするため)
        todo = []
        for back, pv in pres:
            pre = []
            S = tuple(actual)
            n_pre = 1
            if back or pv:
                if back:
                    pre.append({"type": "move", "target_heading_deg": _plan_heading(actual[2]), "distance_mm": back,
                                "reverse": True, "stop": False, "label": None})
                if pv:
                    pre.append({"type": "turn", "target_heading_deg": _plan_heading(actual[2] + pv), "reverse": False, "stop": False, "label": None})
                ps = _sample_steps(pre, actual)
                q = ps[-1]
                S = (q[0], q[1], math.degrees(q[2]))
                n_pre = len(ps)
                if body_leg_clearance(ps) < min(need, start_clr - 0.5):
                    continue
            A = (S[0], S[1], math.radians(S[2]))
            for ci, (B, old, tail) in enumerate(cands):
                if tail_clr[ci] < need and tail_clr[ci] < best_tail - 1e-6:
                    continue   # 合流点から先がすでに目標より狭く、一番よい合流点でもない
                for r in ((radius_mm, 80.0, 150.0, 200.0) if not (back or pv) else (80.0, 150.0)):
                    db = _dubins(A, (B[0], B[1], math.radians(B[2])), r)
                    if db is None:
                        continue
                    tot, t1, a1, phi, Ls, t2, a2 = db
                    if tot > old + max_extra_mm:
                        continue
                    new = list(pre)
                    if math.degrees(a1) > 0.05:
                        new.append({"type": "arc", "target_heading_deg": _plan_heading(S[2] + t1 * math.degrees(a1)),
                                    "sweep_deg": round(-t1 * math.degrees(a1), 3), "radius_mm": r, "reverse": False, "stop": False, "label": None})
                    new.append({"type": "move", "target_heading_deg": _plan_heading(math.degrees(phi)), "distance_mm": round(Ls, 1),
                                "reverse": False, "stop": False, "label": None})
                    if math.degrees(a2) > 0.05:
                        new.append({"type": "arc", "target_heading_deg": _plan_heading(B[2]), "sweep_deg": round(-t2 * math.degrees(a2), 3),
                                    "radius_mm": r, "reverse": False, "stop": False, "label": None})
                    full = new + tail
                    n_check = len(new) + max(0, end_check - (len(steps) - len(tail)))
                    # その場旋回は角度が大きいほど時間がかかり向きもずれやすいので、1°あたり3mm を足す(小さい角度を選ぶ)
                    cost = tot - old + ((REJOIN_PIVOT_COST_MM + 3.0 * abs(pv)) if pv else 0.0) + ((REJOIN_PIVOT_COST_MM + 2.0 * back) if back else 0.0)
                    todo.append((cost, len(todo), full, len(new), n_pre, len(new), tot, old, r, (back, pv), tail_clr[ci]))
        todo.sort(key=lambda t: (t[0], t[1]))
        best = None
        for cost, _, full, n_new, n_pre, nn, tot, old, r, bp, tc in todo:
            if best is not None and min(tc, 1e9) <= best[5] and best[0][0] == 1 and tc < need:
                continue   # 合流点から先だけで、今までの一番よい候補より広くならない
            clr = min(tc, body_leg_clearance(_sample_steps(full[:n_new], actual)[n_pre:]))
            ok = clr >= need
            key = (0 if ok else 1, cost if ok else -clr)
            if best is None or key < best[0]:
                best = (key, full, nn, tot, old, clr, r, bp)
            if ok or (time.time() > deadline and best is not None):
                break
        return best

    deadline = time.time() + REJOIN_TIME_LIMIT_SEC
    best_tail = max(tail_clr.values()) if tail_clr else 1e9

    best = search([(0.0, 0.0)])
    if best is not None and best[0][0] == 1:
        # 円弧だけでは目標に届かない(スタートが脚の近くにずれている): 最初にその場旋回・少し後退を入れる候補も比べる
        pres = [(0.0, pv) for pv in REJOIN_PIVOTS_DEG] + [(b, pv) for b in REJOIN_BACK_MM for pv in (0.0,) + tuple(REJOIN_PIVOTS_DEG)]
        b2 = search(pres)
        if b2 is not None and (b2[0][0] == 0 or b2[5] > best[5] + 1.0):
            best = b2
    if best is None:
        return None, "no dubins path"
    key, full, nn, tot, old, clr, r, (back, pv) = best
    return full, "rejoin (legs) -> %d steps, %.0f mm (plan %.0f mm), radius %.0f, back %.0f, pivot %+.0f, leg clearance %.1f mm (target %.1f)" % (
        nn, tot, old, r, back, pv, clr, need)


def _rejoin_simple(steps, actual, radius_mm=100.0, max_extra_mm=200.0):
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
        new.append({"type": "arc", "target_heading_deg": _plan_heading(actual[2] + t1 * math.degrees(a1)),
                    "sweep_deg": round(-t1 * math.degrees(a1), 3), "radius_mm": radius_mm,
                    "reverse": False, "stop": False, "label": None})
    new.append({"type": "move", "target_heading_deg": _plan_heading(math.degrees(phi)), "distance_mm": round(Ls, 1),
                "reverse": False, "stop": False, "label": None})
    if math.degrees(a2) > 0.05:
        new.append({"type": "arc", "target_heading_deg": _plan_heading(B[2]), "sweep_deg": round(-t2 * math.degrees(a2), 3),
                    "radius_mm": radius_mm, "reverse": False, "stop": False, "label": None})
    return new + list(steps[k:]), "rejoin %d steps -> %d steps, %.0f mm (plan %.0f mm)" % (k, len(new), tot, old)


# ---------------- 走り出しへの取り込み(状態) ----------------
_measured = None          # EtRallyStartPoseCapture の結果 dict(pose=..., frames=..., ...)
def annotate_plan_course(steps, start=None):
    """2026-11-11: 計画の各ステップの始めの、コース上の計画の姿勢 (x, y, コースの向き度) を step["_plan_course"] に入れる
    (計画のスタート START_MM・向き START_HEADING_DEG から、計画どおりに走ったとしてたどる)。
    止まったあとの走り出しで、コース上の今の位置(カメラで直したもの)と計画の位置の差を区間に引き継ぐのに使う
    (et_rally_path_segment の carry_error)。座標・向きは et_rally_course_pose と同じ(左回りが+、コースの向き = 180 − ジャイロの目標)。
    steps をそのまま返す。"""
    x, y, h = (START_MM[0], START_MM[1], START_HEADING_DEG) if start is None else (float(start[0]), float(start[1]), float(start[2]))
    for st in steps:
        t = st.get("type")
        st["_plan_course"] = (x, y, h)
        if t == "turn":
            h = START_HEADING_DEG - float(st["target_heading_deg"])
        elif t == "move":
            hh = START_HEADING_DEG - float(st["target_heading_deg"])
            th = math.radians(hh + (180.0 if st.get("reverse") else 0.0))
            L = float(st["distance_mm"])
            x += L * math.cos(th)
            y += L * math.sin(th)
            h = hh
        elif t == "arc":
            sw = -float(st["sweep_deg"])
            r = float(st["radius_mm"])
            sg = 1.0 if sw > 0 else -1.0
            th0 = math.radians(h + (180.0 if st.get("reverse") else 0.0))
            cx, cy = x - sg * r * math.sin(th0), y + sg * r * math.cos(th0)
            th1 = th0 + math.radians(sw)
            x, y = cx + sg * r * math.sin(th1), cy - sg * r * math.cos(th1)
            h += sw
    return steps


def plan_offset_in_segment(plan_pose, reverse, actual_xy):
    """コース上の今の位置 actual_xy と、計画の区間の始めの姿勢 plan_pose の差を、区間の自分の位置の形 (前, 右) mm にする
    (前 = 計画の始めの進む向き。後退なら車体の後ろ向き。右 = 進む向きの右)。"""
    a = math.radians(plan_pose[2] + (180.0 if reverse else 0.0))
    dx, dy = actual_xy[0] - plan_pose[0], actual_xy[1] - plan_pose[1]
    return dx * math.cos(a) + dy * math.sin(a), dx * math.sin(a) - dy * math.cos(a)


_pending_rejoin = None    # EtRallyApplyStartPose が引き直しを決めたときの実際の姿勢 (x, y, 向き度)
_applied = None           # EtRallyApplyStartPose が取り込んだ姿勢(コース上の位置の走り出しに使う)


def measured_start_pose():
    return _measured


# 2026-11-12: スタートで床の点から当てはめた下向きを覚えておくファイル。点が見えず当てはめられなかった回は、
# ここに残っている前回の値を走行中の補正に使う(較正台での下向きは撮るたびに約1°ずれることがあり、較正ファイルの
# pitch_fix_deg より、直前に床で当てはめた値のほうが確か)。較正ファイルが変わったら使わない。
LAST_PITCH_FILE = Path(__file__).resolve().parents[1] / "run_logs" / "et_rally_last_pitch.json"


def _calib_key(calib_path=None):
    p = Path(calib_path) if calib_path else DEFAULT_CALIB
    try:
        d = json.load(open(p, encoding="utf-8"))
        return "%s pitch_fix=%.3f" % (p.name, float(d.get("pitch_fix_deg", 0.0)))
    except Exception:
        return p.name


def save_last_pitch(pitch_deg, calib_path=None):
    try:
        LAST_PITCH_FILE.parent.mkdir(parents=True, exist_ok=True)
        json.dump({"pitch_deg": float(pitch_deg), "calib": _calib_key(calib_path), "time": time.strftime("%Y-%m-%d %H:%M:%S")},
                  open(LAST_PITCH_FILE, "w", encoding="utf-8"))
        return True
    except Exception:
        return False


def load_last_pitch(calib_path=None):
    """(pitch_deg, 記録した日時) か None(ファイルがない・較正ファイルが違う)。"""
    try:
        d = json.load(open(LAST_PITCH_FILE, encoding="utf-8"))
    except Exception:
        return None
    if d.get("calib") != _calib_key(calib_path):
        return None
    return float(d["pitch_deg"]), d.get("time", "")


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
    t0 = time.time()
    try:
        new, why = rejoin_steps(steps, actual)
    except Exception as e:   # 2026-11-14: 脚を見る引き直しで思わぬ失敗があっても、従来の引き直しで走る
        new, why = _rejoin_simple(steps, actual)
        why = "%s (legs-aware rejoin failed: %r)" % (why, e)
    why = "%s [%.2fs]" % (why, time.time() - t0)
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
    EXTRA_WAIT_SEC = 4.0   # 2026-11-13: 撮れた枚数が足りないとき、開き直しを待つ上限

    def __init__(self, name, duration_sec=2.0, calib_path=None, min_frames=3, ready=None):
        super().__init__(name)
        self.duration_sec = float(duration_sec)
        self.calib_path = calib_path
        self.min_frames = int(min_frames)
        # ready: 撮り始めてよいか(アームが動き終わったか)を返す関数。None ならすぐ撮り始める。
        self.ready = ready
        self.started = False
        self.counting_since = None
        self.cam = None

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
            # 2026-11-10: カメラは子プロセス(et_rally_camera_proc.py)が持ち、位置もそこで出す
            # (同じプロセスでカメラを動かすと、走りの制御が止まるため)。
            from . import et_rally_camera_proc
            self.proc = et_rally_camera_proc.current()
            if self.proc is None:
                self.logger.warning("start pose: camera process not running, skipped")
                return Status.SUCCESS
            return Status.RUNNING
        if self.proc.ready is None and now - self.start_time < self.MAX_WAIT_SEC:
            return Status.RUNNING   # カメラが開くのを待つ
        if self.counting_since is None:
            if self.ready is not None and not self.ready() and now - self.start_time < self.MAX_WAIT_SEC:
                self.proc.poll()
                return Status.RUNNING
            self.counting_since = now
            self.proc.poll()
            self.proc.send("detect 1 %d" % (1 if getattr(runtime, "course", -1) == 1 else 0))
        for ev in self.proc.poll():
            if ev.get("ev") == "error":
                self.logger.warning("start pose: camera process: %s" % ev.get("what"))
            if ev.get("ev") == "reopened":
                self.logger.warning("start pose: camera reopened as video%s (no images for %.1fs)" % (ev.get("device"), ev.get("lost_sec", 0.0)))
            if ev.get("ev") != "pose" or ev.get("t", 0.0) < self.counting_since:
                continue
            self.frames += 1
            # 11/08: 下向きも当てはめる出し方を使う(動かした距離での答え合わせで一番合った)。比べるために当てはめない場合も残す
            if ev.get("r"):
                self.results.append(ev["r"])
            if ev.get("r0"):
                self.results_noscale.append(ev["r0"])
        if now - self.counting_since < self.duration_sec:
            return Status.RUNNING
        # 2026-11-13: カメラのUSBが切れて開き直している間は画像が来ない。足りなければ最大 EXTRA_WAIT_SEC まで待つ
        # (本番は止まると終わりなので、少し待ってでも位置を取り込むほうがよい)。
        if len(self.results) < self.min_frames and now - self.counting_since < self.duration_sec + self.EXTRA_WAIT_SEC:
            return Status.RUNNING
        return self._finish()

    def _finish(self):
        global _measured
        if getattr(self, "proc", None) is not None:
            self.proc.send("detect 0")
        if len(self.results) < self.min_frames:
            self.logger.warning("start pose: not enough frames (%d poses from %d frames), using the plan start" % (
                len(self.results), self.frames))
            return Status.SUCCESS
        pose = median_pose(self.results)
        _measured = dict(pose=pose, frames=self.frames, poses=len(self.results),
                         n_markers=max(r["n_markers"] for r in self.results), n_dots=max(r["n_dots"] for r in self.results),
                         spread_mm=max(math.hypot(r["pose"][0] - pose[0], r["pose"][1] - pose[1]) for r in self.results),
                         spread_deg=max(abs(_nd(r["pose"][2] - pose[2])) for r in self.results),
                         scale=sorted(r.get("scale", 1.0) for r in self.results)[len(self.results) // 2],
                         pitch_deg=sorted(r.get("pitch_deg", 0.0) for r in self.results)[len(self.results) // 2],
                         resid_mm=sorted(r.get("resid_mm", 0.0) for r in self.results)[len(self.results) // 2],
                         items=sorted(int(r["n_markers"] + r["n_dots"]) for r in self.results)[len(self.results) // 2],
                         n_dots_med=sorted(int(r["n_dots"]) for r in self.results)[len(self.results) // 2],
                         wide=sum(1 for r in self.results if r.get("wide")),
                         assumed=sum(1 for r in self.results if r.get("pitch_assumed") is not None))
        if (_measured["n_dots"] >= 1 and _measured["pitch_deg"] and abs(_measured["pitch_deg"]) <= 1.5
                and save_last_pitch(_measured["pitch_deg"], self.calib_path)):
            self.logger.info("start pose: saved pitch %+.2f deg for runs without visible dots" % _measured["pitch_deg"])
        self.logger.info("start pose: x=%.1f y=%.1f h=%+.2f deg (plan start %+.1f,%+.1f,%+.1f) poses=%d/%d frames "
                         "markers=%d dots=%d spread=%.1fmm/%.2fdeg pitch_fix=%+.2fdeg resid=%.1fmm items=%d wide=%d/%d" % (
                             pose[0], pose[1], pose[2], pose[0] - START_MM[0], pose[1] - START_MM[1],
                             _nd(pose[2] - START_HEADING_DEG), len(self.results), self.frames,
                             _measured["n_markers"], _measured["n_dots"], _measured["spread_mm"], _measured["spread_deg"],
                             _measured["pitch_deg"], _measured["resid_mm"], _measured["items"],
                             _measured["wide"], len(self.results)))
        if _measured["assumed"]:
            self.logger.info("start pose: markers only in %d/%d images, position with the saved/assumed pitch" % (
                _measured["assumed"], len(self.results)))
        p0 = median_pose(self.results_noscale)
        if p0 is not None:
            self.logger.info("start pose without pitch fit: x=%.1f y=%.1f h=%+.2f deg (plan start %+.1f,%+.1f,%+.1f)" % (
                p0[0], p0[1], p0[2], p0[0] - START_MM[0], p0[1] - START_MM[1], _nd(p0[2] - START_HEADING_DEG)))
        return Status.SUCCESS


class EtRallyApplyStartPose(Behaviour):
    """測った位置と向きを、走り出しに取り込む。向きはジャイロに足し、位置は rejoin_min_mm 以上ずれていれば
    最初のつなぎを引き直す予定にし(計画の読み込み時に apply_pending_rejoin)、小さければ最初の区間の位置に入れる。
    max_offset_mm / max_offset_deg を超えるずれは、測り間違いとみなして取り込まない。"""

    def __init__(self, name, rejoin_min_mm=5.0, max_offset_mm=80.0, max_offset_deg=15.0, plan_frame_known=True,
                 min_items_for_position=2):
        # min_items_for_position: 11/10 夕に3(2つ以下は向きだけ)としたが、床での下向きの補正(calib_1110b の pitch_fix_deg)を
        # 入れたら、2つだけ(マーカー1+点1)でも前後の測り間違いが約17mm→約4mmになったので2に戻した(1つだけなら向きだけ)。
        super().__init__(name)
        self.min_items_for_position = int(min_items_for_position)
        # plan_frame_known: 計画が経路を作る道具の座標(スタート START_MM・START_HEADING_DEG、gyro = スタートの向き - 向き)か。
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
        elif m.get("items", 99) < self.min_items_for_position:
            # 2026-11-10: 見えたものが1つだけだと、位置は測り間違いのおそれがあるので入れない。
            # 向き(マーカーの辺の傾きから)だけ入れて、計画のスタートにいるとみなす。
            _applied = (START_MM[0], START_MM[1], h)
            how = "heading only (%d items, %d dots)" % (m["items"], m.get("n_dots", -1))
        elif m.get("items", 99) < 3:
            # 2026-11-11: 点が見えずマーカー2つだけのときは、前後を約2〜4cm 間違える(点がないと下向きの当てはめ直しが
            # できないため)。11/12: 同じ画像を広い範囲で点も使って測り直すと、前後は大きく違ったが、左右は数mm で合っていた
            # (配置36の4回)。そこで向きと左右だけを入れ、前後は計画のスタートのままにする。
            # 2026-11-14: 保存した下向きで写し直した場合(pitch_assumed)も同じにする。下向きは見る点の距離で1〜2°変わり
            # (スタート +1.1〜2.2°、フィールドの真ん中 −0.5〜+0.5°)、配置36でマーカー2つから前後 +44mm と出て、
            # そのまま取り込んで1周目が大きく揺れた。見えたものが3つ未満(下向きを当てはめられない)なら、点が見えていても
            # 前後は入れない。
            a = math.radians(START_HEADING_DEG)
            along = dx * math.cos(a) + dy * math.sin(a)
            x, y = x - along * math.cos(a), y - along * math.sin(a)
            _applied = (x, y, h)
            lat = math.hypot(x - START_MM[0], y - START_MM[1])
            if lat >= self.rejoin_min_mm:
                _pending_rejoin = (x, y, h)
            else:
                _inject_position((x, y, h), self.logger)
            how = "heading and sideways only (%d items, %d dots, front/back %+.1f mm not used)" % (
                m["items"], m.get("n_dots", -1), along)
        elif math.hypot(dx, dy) >= self.rejoin_min_mm:
            _pending_rejoin = (x, y, h)
            how = "rejoin planned"
        else:
            _inject_position((x, y, h), self.logger)
            how = "position injected"
        self.logger.info("start pose: applied offset %+.1f,%+.1f mm %+.2f deg (%s)" % (dx, dy, dth, how))
        return Status.SUCCESS
