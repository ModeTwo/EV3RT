"""試作: 前進の円弧・後退・その場の旋回を、つなぎ区間ごとに混ぜて選ぶ経路の探索(4周+ゴール)。

ゲートの通り方は F(前進)/B(後退)/FB(前進→後退の往復)/BF、どちら側から、手前と奥の余り(ext)、
ゲートの中での横の位置(offsets)の組み合わせ。つなぎ区間(前のゲートの終わりの姿勢→次のゲートの始めの姿勢)は、
  1. 1本の走り(前進または後退の Dubins 経路: 円弧-直進-円弧 / 円弧-円弧-円弧)
  2. その場の旋回を使うもの(旋回-直進-旋回、旋回-直進-円弧、円弧-直進-旋回)
  3. 中継を1つ経由するもの(1と2で当たらないときだけ: 旋回-直進-旋回-直進-旋回、走り-走り)
から、当たらずに最短のものを選ぶ。止まる(前後の切り替え・旋回の前・ゴール)直前は直進で終える(tail_cm)。
順番どおりのゲート以外の線は横切らない(同じ色の重複はよいが、色の順番が変わる走行は規則違反)。

探索は「当たり判定を後回しにする」動的計画法: まず当たりを無視した時間で最短経路を出し、使ったつなぎ区間だけ
厳密に判定する。当たっていたら、その区間の次の候補に替えて解き直す(使わない区間は判定しない)。

時間(秒)と精度の重みは設定で変えられる(HR_PARAMS=設定JSONのパス、または HR_<名前>=値)。
usage: hybrid_route.py gates.json out_curved.json    (出力は curve_route.py と同じ形式。to_plan.py に渡せる)
"""
import json, math, sys, os, time
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hr_kernels   # numba があれば重い計算を速くする(なければ numpy 版のまま)

# ---------------- 設定 ----------------
PARAMS = dict(
    laps=4,
    # 車体と当たり判定(cm)
    front=7.5, rear=17.9, half_width=7.0, margin_front=2.0, margin_rear=2.0,   # 11/03: 車体の実測(車軸から前7.5・後ろ17.9cm、幅14cm、左右は同じとした)。11/02: すき間は前後とも2.0cm(以前は1.5cm、その前は後ろ0.4cm)。実機で決め直す
    # 経路の形
    radii=[20.0, 15.0, 12.0, 10.0, 8.0], tail_cm=4.0,   # 2026-11-09: 8cmを追加(50配置で平均-3.7秒、実機は未確認)
    ext=[0.0, 4.0, 8.0, 14.0, 22.0, 35.0], offsets=[0.0], angles=[0.0],   # 11/02: 横のずらしは使わない(反対側の支柱に近づくため)
    # 斜めの通過: 'split' = 真ん中で向きを変える(斜めに入って垂直に抜ける/垂直に入って斜めに抜ける)、
    # 'straight' = 入りから抜けまで同じ斜め、'both' = 両方。split の向きを変える円弧の半径(cm)は split_radii。
    oblique_mode='split', split_radii=[20.0, 15.0, 10.0],
    # 斜めの使い方: 'safest' = そのゲートを(その側から・その通り方で)真ん中を垂直に通れないときだけ、
    # 通過の間の一番近い柱までの距離が最大になる角度と円弧の半径を1つ選ぶ(angle_scan の範囲)。'all' = angles をどこでも使う。
    oblique_policy='safest', angle_scan=[-30.0, 30.0, 1.0],
    # True: 入りも抜けも斜めの通過も候補にする(入りの側にも抜けの側にもとなりのT字があるゲートのため。11/02 seed758743)
    oblique_both=True,
    # スタート: 'default' = 今のスタート位置(向き0)、'goal' = 今のゴール地点(向き180、フィールドの方を向く)、
    # 'goal_mid' = 今のゴール地点のXで、Yはユーザーの呼び方のY4とY3の列の中点(y = 2.5P、向き180)。2026-11-08 検討用
    # 2026-11-09 ユーザー決定: スタートは今のゴール地点で向き180(フィールドを向く)を既定に。ゴールは同じ場所(goal_at='default')、ガレージは goal_at='garage'
    start_at='goal',
    # ゴール: 'default' = 今のゴール地点(向き90)、'garage' = ガレージ(以前の planner config の GOAL_SHIFT_CM=(-61,-115) の位置を
    # 右コースにしたもの、向き goal_heading)。2026-11-08 検討用(ガレージの正確な範囲は未確認)
    goal_at='default', goal_heading=0.0,
    # ゴールで止まるときの向きの候補(度)。空なら goal_heading だけ。2026-11-09: ガレージは向きの決まりがないので複数を比べるため
    goal_headings=[],
    # 2026-11-09 ユーザーの指摘の反映:
    # same_laps: 2周目から最後の1つ前の周までを、同じ通り方・同じ場所にする(最初の周の入りと最後の周の抜けだけ自由)。
    # join_max_cm: 前のゲートの抜けと次のゲートの入りが同じ向きの一直線で、抜けが入りよりこの長さまで先のとき、戻らずにそのまま続ける
    # (平行で隣の列のゲートは間隔24.64cmが車体より短く、抜けきる・手前から入るの条件で数cm戻っていた)。0で無効。
    # start_any_gate: 赤をくぐり始めるまでは、赤以外のゲートをくぐってよい(色の順番は赤から数える)。
    same_laps=True, join_max_cm=8.0, start_any_gate=True,
    # 2段階探索の2段目: 色ごとに使う通り方 [sign, mode, angle, angle_out] だけに絞る(空なら絞らない)
    visit_filter={},
    # 狭い配置の判定(2026-11-09): この順にすき間を試し、3色ともゲートを通る候補がある最初のすき間で探索する。
    # 候補を作るだけなので約1秒で分かる。空なら margin_front/margin_rear のまま。
    margin_list=[],
    # narrow_safest: 狭い配置は垂直に通れないゲートだけ斜めにする(本番の計算用)。事前計算では False にして斜めも全部使う。
    # candidates_only: すき間の判定だけして終わる(全配置から狭い配置を洗い出す用)。
    narrow_safest=True, candidates_only=False,
    # start_prelude: スタートのあと、赤の前に黄を1回通ってから赤へ行く形も候補にする(黄と赤が縦に並ぶ配置で、黄を抜けてそのまま赤を通れる。2026-11-09)
    start_prelude=True,
    # 進入禁止: True = 車体の箱のどこも入らない(スタートで置いた時点でかかっている長方形は、スタートの車軸から start_exempt_cm 以内だけ外す)
    keepout_body=True, start_exempt_cm=20.0,
    # 進入禁止の長方形(右コースのplanner座標cm、[xmin, xmax, ymin, ymax]、±1e9で無限)。None なら plannerの設定(左コース)を左右反転した今の3つ。
    # 'planner' なら以前の暫定の3つ。11/05 ユーザーの説明(規約の追加分)による今の3つ(P=24.64、y=4P がスタートとゴールの列):
    #  ① スタートとゴールの列の点から外へ40cmより先(y > 4P+40)
    #  ② スタートとゴールの列と1つ内側の列の範囲(y >= 3P)で、ゴール側の列の点から外へ40cmより先(x > 4P+40)
    #  ③ スタート側の列の点から外へ34cmより先(x < -34)
    keepout=[[-1e9, 1e9, 138.56, 1e9], [138.56, 1e9, 73.92, 1e9], [-1e9, -34.0, -1e9, 1e9]],
    allow_reverse=True, allow_pivot=True,
    # ゲートの脚の当たり判定(2026-11-05、ユーザーの実測): 'rect' = 床に寝たT字の管(長さ leg_len・直径 leg_dia、
    # ゲートの線と直角)を長方形として扱う。ゲートは組み方で幅が変わる(外側の幅 最大 gate_outer_max、内側の幅 最小
    # gate_inner_min)ので、2つの格子の点の真ん中に左右対称に置いたとき、脚が来うる範囲全体を1つの長方形にする
    # (狭くても広くても当たらないように)。'cap' = 以前の形(格子の点を中心に ±4cm の線 + 半径1cm)。
    gate_model='rect', leg_len=7.5, leg_dia=2.5, gate_inner_min=22.7, gate_outer_max=29.5,
    # アームを下げたままの車体(2026-11-06): 車体の前の端からアームが前に出る長さと、その部分の横幅の半分(cm)。
    # 計画に「アームが上がっていないと脚に当たる最初の点」までの距離と時間を書くのに使う(経路そのものは変えない)。
    # 仮の値(TODO: 実機で測る)。
    arm_front_cm=5.0, arm_half_width_cm=7.0,
    # 時間: 直進 cm/s、円弧は arc_k*v_line/(1+half_tread/r)、後退は rev_speed_scale 倍の速さ
    v_line=36.0, arc_k=0.77, half_tread=5.85, rev_speed_scale=1.0,
    t_stop=0.67, t_turn0=0.7, t_turn_deg=0.012,
    # 精度の重み(秒に換算して時間に足す。0なら時間だけで選ぶ)
    pen_stop=0.0, pen_pivot=0.0, pen_rev_m=0.0, pen_arc_100deg=0.0,
    # 中継
    wp_step=8.0, wp_headings=12, wp_tries=60, via_max_tries=40,
    # True: 遠回りなしで経路が見つかっても、遠回りで速くなる区間がないか探す(最速の取りこぼしを防ぐ)
    via_always=False, via_budget_sec=8.0,
    # その場の旋回の中心が車軸からどれだけ後ろか(cm)。10/31の左右同期の旋回で約0.5cm(測り直し予定)
    pivot_back_cm=0.5,   # 2段目(遠回りで速くなるか)の時間の上限。超えたら1段目の経路を使う
    # 2026-11-11(ユーザーの指示): ゲートを抜けたとみなす距離(cm)。車体の後ろの端(後退なら前の端)の角が、ゲートの2つの点の
    # 中心を結ぶ線を、線に垂直にこの距離だけ越えるまで直進で抜ける。規則は「ロボットが完全にゲートを通過すること」で、
    # ゲートには幅があり置き方にもむらがあるため、灰色の点の端(半径2.2cm、直径約44mm)から2cm = 4.2cm にした(以前は2.0cm)。
    pass_cm=4.2,
    # 2026-11-11(ユーザーの実測): ガレージエリア。今のスタート地点から、フィールドと逆向き(+x)に [x0, x1] cm、
    # Y5の方向(-y)に [y0, y1] cm の長方形。goal_at='garage' のとき、止まる向き(goal_headings)ごとに、車体の箱
    # (車軸から前 front・後ろ rear、幅 half_width×2)がこの長方形の真ん中に来るように車軸の位置を決める。
    # 箱が長方形の内側に garage_margin_cm 以上の余裕で入らない向きは使わない(24.5×30cmなので、車体25.4cmは
    # フィールドと平行な向き ±90 でしか入らない)。空なら以前の1点(スタートから +61, -115)。
    garage_area=[45.5, 70.0, 100.0, 130.0], garage_margin_cm=1.0,
    # 2026-11-11(ユーザー): 規則は「走行体の一部がガレージエリアにかかっていればよい」。'overlap' では、車体の箱が
    # エリアに x・y とも garage_overlap_cm 以上食い込む位置で止める(止まる位置のずれ: 今日の実測で前後約2.5cm・左右約1.6cm)。
    # 向き(goal_headings、空なら 0/90/180/-90)ごとに、フィールドに近い角・フィールド側の辺の真ん中・スタート側の辺の真ん中・
    # エリアの真ん中の4か所を候補にし、一番速いものを選ばせる。'inside' は車体全体を入れる(上の garage_margin_cm)。
    garage_rule='overlap', garage_overlap_cm=6.0,
)
if os.environ.get('HR_PARAMS'):
    PARAMS.update(json.load(open(os.environ['HR_PARAMS'], encoding='utf-8')))
for k in list(PARAMS):
    v = os.environ.get('HR_' + k.upper())
    if v is not None:
        d = PARAMS[k]
        PARAMS[k] = ([float(x) for x in v.split(',')] if isinstance(d, list) else
                     (v == '1' if isinstance(d, bool) else type(d)(v)))
globals().update({k.upper(): v for k, v in PARAMS.items()})

P = 24.64
POST_R, ARM = 1.0, 4.0
PM, CLR = PASS_CM, 0.5   # PM: 抜けの距離(上の pass_cm)。CLR: 入りで先の端の角が線の手前に来る距離
INF = 1e9
TAU = 2 * math.pi
START = np.array([2 * 2 * P - (4 * P + 17.8), 3.5 * P]); START_H = 0.0
GOAL = np.array([2 * 2 * P - (0 * P - 14.9), 3.5 * P]); GOAL_H = 90.0
if START_AT == 'goal':
    START = GOAL.copy(); START_H = 180.0
elif START_AT == 'goal_mid':
    START = np.array([GOAL[0], 2.5 * P]); START_H = 180.0
GARAGE_BOX = None
if GOAL_AT == 'garage':
    _s0 = np.array([2 * 2 * P - (0 * P - 14.9), 3.5 * P])   # 今のスタート地点(新スタート)
    GOAL = np.array([_s0[0] + 61.0, _s0[1] - 115.0]); GOAL_H = float(GOAL_HEADING)
    if GARAGE_AREA:
        _a = [float(v) for v in GARAGE_AREA]
        GARAGE_BOX = (_s0[0] + _a[0], _s0[0] + _a[1], _s0[1] - _a[3], _s0[1] - _a[2])   # (x0, x1, y0, y1)


def garage_overlap_goals(h):
    """'overlap': 向き h で、車体の箱がエリアに x・y とも GARAGE_OVERLAP_CM 以上食い込む車軸の位置の候補(4か所)。
    フィールドとスタートは、エリアから見て x の小さい側・y の大きい側にある。"""
    x0, x1, y0, y1 = GARAGE_BOX
    D = float(GARAGE_OVERLAP_CM)
    c, sn = math.cos(math.radians(h)), math.sin(math.radians(h))
    half_len = (FRONT + REAR) / 2.0
    ex = abs(c) * half_len + abs(sn) * HALF_WIDTH
    ey = abs(sn) * half_len + abs(c) * HALF_WIDTH
    off = (FRONT - REAR) / 2.0
    near_x = x0 + D - ex          # 車体の x の大きい側の端が、エリアのフィールド側の辺から D 入る
    near_y = y1 - D + ey          # 車体の y の小さい側の端が、エリアのスタート側の辺から D 入る
    mid_x, mid_y = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    out = []
    for bx, by in ((near_x, near_y), (near_x, mid_y), (mid_x, near_y), (mid_x, mid_y)):
        out.append(np.array([bx - off * c, by - off * sn]))
    return out


def garage_goal(h):
    """向き h(度)で止まるとき、車体の箱がガレージの真ん中に来る車軸の位置。入らなければ None。"""
    x0, x1, y0, y1 = GARAGE_BOX
    c, sn = abs(math.cos(math.radians(h))), abs(math.sin(math.radians(h)))
    half_len = (FRONT + REAR) / 2.0
    ex, ey = c * half_len + sn * HALF_WIDTH, sn * half_len + c * HALF_WIDTH
    if ex > (x1 - x0) / 2.0 - GARAGE_MARGIN_CM + 1e-9 or ey > (y1 - y0) / 2.0 - GARAGE_MARGIN_CM + 1e-9:
        return None
    off = (FRONT - REAR) / 2.0   # 車軸から見た車体の真ん中(前が+)
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    return np.array([cx - off * math.cos(math.radians(h)), cy - off * math.sin(math.radians(h))])
_mx = lambda x: 4 * P - x
KEEPOUT = [(_mx(INF), _mx(-17.0), -INF, -37.0),
           (_mx(INF), _mx(4 * P + 17.8 + 14.0), -INF, INF),
           (_mx(-28.0), INF, 0.0, INF)]
if PARAMS['keepout'] != 'planner':
    KEEPOUT = [tuple(float(v) for v in r) for r in PARAMS['keepout']]
BOUNDS = (-31.0, 126.0, -36.0, 4 * P + 40)
if GOAL_AT == 'garage':
    BOUNDS = (-31.0, 180.0, -45.0, 4 * P + 40)   # ガレージまでの中継の点も置けるように


def nd(a):
    return (a + 180.0) % 360.0 - 180.0


def v_arc(r):
    return ARC_K * V_LINE / (1 + HALF_TREAD / r)


def spd(m):
    return 1.0 if m > 0 else 1.0 / REV_SPEED_SCALE


def turn_cost(deg):
    return 0.0 if deg < 0.3 else T_TURN0 + T_TURN_DEG * deg


# ---------------- ゲート ----------------
class Gate:
    def __init__(self, color, a, b):
        self.color = color
        self.pa = np.array([a[0] * P, a[1] * P], float); self.pb = np.array([b[0] * P, b[1] * P], float)
        self.c = (self.pa + self.pb) / 2
        self.t = (self.pb - self.pa) / np.linalg.norm(self.pb - self.pa)
        self.n = np.array([-self.t[1], self.t[0]])


GJ = json.load(open(sys.argv[1], encoding='utf-8'))['gates']
GATES = {c: Gate(c, GJ[c]['a'], GJ[c]['b']) for c in ('red', 'blue', 'yellow')}
CAP = np.array([p + g.n * ARM * (2 * t - 1) for g in GATES.values() for p in (g.pa, g.pb) for t in np.linspace(0, 1, 9)])


def _make_legs():
    """脚の長方形: (中心, ゲートの線の向き u, 直角の向き v, u方向の半分, v方向の半分) のリスト。"""
    legs = []
    gap = np.linalg.norm(next(iter(GATES.values())).pb - next(iter(GATES.values())).pa)
    off_min = (GATE_INNER_MIN + LEG_DIA - gap) / 2      # 脚の中心が格子の点から外へずれる量(一番狭いとき)
    off_max = (GATE_OUTER_MAX - LEG_DIA - gap) / 2      # 同(一番広いとき)
    s0, s1 = off_min - LEG_DIA / 2, off_max + LEG_DIA / 2   # 外向きの座標で、脚が来うる範囲
    for g in GATES.values():
        for p, out in ((g.pa, -g.t), (g.pb, g.t)):
            legs.append((p + out * (s0 + s1) / 2, out, g.n, (s1 - s0) / 2, LEG_LEN / 2))
    return legs


LEGS = _make_legs()
LEG_C = np.array([l[0] for l in LEGS]); LEG_U = np.array([l[1] for l in LEGS]); LEG_V = np.array([l[2] for l in LEGS])
LEG_HU = np.array([l[3] for l in LEGS]); LEG_HV = np.array([l[4] for l in LEGS])
# 車軸から車体の一番遠い角 + 脚の中心から角まで + すき間の下限(poses_clear で遠い脚を外すのに使う。少し余裕を足す)
_RECT_REACH = math.hypot(max(FRONT, REAR), HALF_WIDTH) + float(np.hypot(LEG_HU, LEG_HV).max()) + max(MARGIN_FRONT, MARGIN_REAR) + 1.0
LEG_CORNERS = np.array([c + u * a * hu + v * b * hv for c, u, v, hu, hv in LEGS for a in (-1, 1) for b in (-1, 1)])
_LEG_CORNERS4 = LEG_CORNERS.reshape(-1, 4, 2)
if GATE_MODEL == 'rect':
    POST_R = 0.0   # 長方形そのものとの距離なので、点の太さは使わない
GSEG = [(g.pa, g.pb, g.color) for g in GATES.values()]


# ---------------- 当たり判定 ----------------
def _rect_clear(px, py, h_rad, x0, x1, keep=None, hw=None):
    """gate_model='rect': 車体の箱 x∈[x0,x1], |y|<=HW と、脚の長方形の最小のすき間(配列)。
    凸な長方形どうしの距離 = 片方の角からもう片方までの距離の最小(脚の角→車体、車体の角→脚の両方を見る)。
    keep: 見る脚の番号の配列(None なら全部)。hw: 箱の横幅の半分(None なら車体の HALF_WIDTH)。"""
    hw = HALF_WIDTH if hw is None else hw
    if hr_kernels.AVAILABLE and keep is None and np.ndim(x0) == 0 and np.ndim(x1) == 0:
        return hr_kernels.rect_clear(px, py, np.cos(h_rad), np.sin(h_rad), float(x0), float(x1), float(hw),
                                     LEG_C, LEG_U, LEG_V, LEG_HU, LEG_HV, _LEG_CORNERS4)
    n = len(px)
    c = np.cos(h_rad); s = np.sin(h_rad)
    if keep is None:
        corners, lc, lu, lv, lhu, lhv = LEG_CORNERS, LEG_C, LEG_U, LEG_V, LEG_HU, LEG_HV
    else:
        corners = LEG_CORNERS.reshape(-1, 4, 2)[keep].reshape(-1, 2)
        lc, lu, lv, lhu, lhv = LEG_C[keep], LEG_U[keep], LEG_V[keep], LEG_HU[keep], LEG_HV[keep]
    # 脚の角 → 車体の箱
    dx = corners[None, :, 0] - px[:, None]; dy = corners[None, :, 1] - py[:, None]
    lx = dx * c[:, None] + dy * s[:, None]; ly = -dx * s[:, None] + dy * c[:, None]
    x0a = np.broadcast_to(np.asarray(x0, float), (n,))[:, None]; x1a = np.broadcast_to(np.asarray(x1, float), (n,))[:, None]
    d1 = np.hypot(np.maximum(np.maximum(x0a - lx, lx - x1a), 0.0), np.maximum(np.abs(ly) - hw, 0.0)).min(1)
    # 車体の箱の角 → 脚の長方形
    cx = np.stack([x0a[:, 0], x0a[:, 0], x1a[:, 0], x1a[:, 0]], 1); cy = np.array([-hw, hw, -hw, hw])[None, :]
    wx = px[:, None] + cx * c[:, None] - cy * s[:, None]; wy = py[:, None] + cx * s[:, None] + cy * c[:, None]
    rx = wx[:, :, None] - lc[None, None, :, 0]; ry = wy[:, :, None] - lc[None, None, :, 1]
    du = np.abs(rx * lu[None, None, :, 0] + ry * lu[None, None, :, 1]) - lhu[None, None, :]
    dv = np.abs(rx * lv[None, None, :, 0] + ry * lv[None, None, :, 1]) - lhv[None, None, :]
    d2 = np.hypot(np.maximum(du, 0.0), np.maximum(dv, 0.0)).min(axis=(1, 2))
    return np.minimum(d1, d2)


def _rect_clear_front_rear(px, py, h_rad, keep):
    """_rect_clear(…, 0, FRONT, keep) と _rect_clear(…, -REAR, 0, keep) を1回で(脚の角の座標の変換を共有して速くする。値は同じ)。"""
    c = np.cos(h_rad); s = np.sin(h_rad)
    if hr_kernels.AVAILABLE:
        return hr_kernels.rect_front_rear(px, py, c, s, LEG_C, LEG_U, LEG_V, LEG_HU, LEG_HV, _LEG_CORNERS4, keep,
                                          float(FRONT), float(REAR), float(HALF_WIDTH))
    corners = LEG_CORNERS.reshape(-1, 4, 2)[keep].reshape(-1, 2)
    lc, lu, lv, lhu, lhv = LEG_C[keep], LEG_U[keep], LEG_V[keep], LEG_HU[keep], LEG_HV[keep]
    dx = corners[None, :, 0] - px[:, None]; dy = corners[None, :, 1] - py[:, None]
    lx = dx * c[:, None] + dy * s[:, None]; ly = -dx * s[:, None] + dy * c[:, None]
    ey = np.maximum(np.abs(ly) - HALF_WIDTH, 0.0)
    d1f = np.hypot(np.maximum(np.maximum(0.0 - lx, lx - FRONT), 0.0), ey).min(1)
    d1r = np.hypot(np.maximum(np.maximum(-REAR - lx, lx - 0.0), 0.0), ey).min(1)
    # 車体の箱の角: 前の箱 (0,-HW),(0,HW),(FRONT,-HW),(FRONT,HW)、後ろの箱 (-REAR,-HW),(-REAR,HW),(0,-HW),(0,HW)
    cx = np.array([0.0, 0.0, FRONT, FRONT, -REAR, -REAR])[None, :]; cy = np.array([-HALF_WIDTH, HALF_WIDTH] * 3)[None, :]
    wx = px[:, None] + cx * c[:, None] - cy * s[:, None]; wy = py[:, None] + cx * s[:, None] + cy * c[:, None]
    rx = wx[:, :, None] - lc[None, None, :, 0]; ry = wy[:, :, None] - lc[None, None, :, 1]
    du = np.abs(rx * lu[None, None, :, 0] + ry * lu[None, None, :, 1]) - lhu[None, None, :]
    dv = np.abs(rx * lv[None, None, :, 0] + ry * lv[None, None, :, 1]) - lhv[None, None, :]
    dd = np.hypot(np.maximum(du, 0.0), np.maximum(dv, 0.0)).min(2)
    d2f = dd[:, 0:4].min(1); d2r = dd[:, [4, 5, 0, 1]].min(1)
    return np.minimum(d1f, d2f), np.minimum(d1r, d2r)


def box_clear(px, py, h_rad, x0, x1):
    """姿勢(配列)の車体の箱 x∈[x0,x1], |y|<=HW と支柱の最小すき間(配列)。x0,x1 はスカラーか配列。"""
    if GATE_MODEL == 'rect':
        return _rect_clear(np.asarray(px, float), np.asarray(py, float), np.asarray(h_rad, float), x0, x1)
    c = np.cos(h_rad)[:, None]; s = np.sin(h_rad)[:, None]
    dx = CAP[None, :, 0] - px[:, None]; dy = CAP[None, :, 1] - py[:, None]
    lx = dx * c + dy * s; ly = -dx * s + dy * c
    x0 = np.asarray(x0, float).reshape(-1, 1) if np.ndim(x0) else x0
    x1 = np.asarray(x1, float).reshape(-1, 1) if np.ndim(x1) else x1
    ex = np.maximum(np.maximum(x0 - lx, lx - x1), 0.0)
    ey = np.maximum(np.abs(ly) - HALF_WIDTH, 0.0)
    return np.min(np.hypot(ex, ey), axis=1) - POST_R


_BODY_REACH = math.hypot(max(FRONT, REAR), HALF_WIDTH) + POST_R + max(MARGIN_FRONT, MARGIN_REAR) + 0.5


def poses_clear(px, py, h_deg):
    """姿勢ごとに、前の部分と後ろの部分のすき間が足りているか(bool配列)。"""
    if GATE_MODEL == 'rect':
        h = np.radians(np.asarray(h_deg, float)); px = np.asarray(px, float); py = np.asarray(py, float)
        # どの姿勢でも、車軸から車体の一番遠い角 + 脚の半対角線 + すき間の下限 より遠い脚は、足りないすき間の原因になりえないので見ない
        if hr_kernels.AVAILABLE:
            keep, near = hr_kernels.poses_near(px, py, LEG_C, float(_RECT_REACH))
            out = np.ones(len(px), bool)
            if len(near):
                cf, cr = _rect_clear_front_rear(px[near], py[near], h[near], keep)
                out[near] = (cf >= MARGIN_FRONT) & (cr >= MARGIN_REAR)
            return out
        dm = np.hypot(px[:, None] - LEG_C[None, :, 0], py[:, None] - LEG_C[None, :, 1])
        keep = np.flatnonzero(dm.min(0) <= _RECT_REACH)
        out = np.ones(len(px), bool)
        if len(keep) == 0:
            return out
        near = np.flatnonzero(dm[:, keep].min(1) <= _RECT_REACH)   # どの脚からも遠い姿勢は計算しない
        if len(near) == 0:
            return out
        cf, cr = _rect_clear_front_rear(px[near], py[near], h[near], keep)
        out[near] = (cf >= MARGIN_FRONT) & (cr >= MARGIN_REAR)
        return out
    # 車体の近く(車軸から車体の一番遠い角+支柱+すき間)にある支柱の点だけで判定する
    rr = _BODY_REACH
    sel = CAP[(CAP[:, 0] > px.min() - rr) & (CAP[:, 0] < px.max() + rr) & (CAP[:, 1] > py.min() - rr) & (CAP[:, 1] < py.max() + rr)]
    if len(sel) == 0:
        return np.ones(len(px), bool)
    h = np.radians(h_deg)
    c = np.cos(h)[:, None]; s = np.sin(h)[:, None]
    dx = sel[None, :, 0] - px[:, None]; dy = sel[None, :, 1] - py[:, None]
    lx = dx * c + dy * s; ly = -dx * s + dy * c
    ey = np.maximum(np.abs(ly) - HALF_WIDTH, 0.0)
    ef = np.maximum(np.maximum(-lx, lx - FRONT), 0.0)
    er = np.maximum(np.maximum(-REAR - lx, lx), 0.0)
    cf = np.min(np.hypot(ef, ey), axis=1) - POST_R
    cr = np.min(np.hypot(er, ey), axis=1) - POST_R
    return (cf >= MARGIN_FRONT) & (cr >= MARGIN_REAR)


def poses_ok(px, py, h_deg):
    return bool(np.all(poses_clear(np.asarray(px, float), np.asarray(py, float), np.asarray(h_deg, float))))


_piv_int = {}


def _pivot_axle(p, h_from, hs):
    """その場の旋回で、回る中心が車軸の PIVOT_BACK_CM 後ろにあるときの、向き hs での車軸の位置。"""
    hs = np.asarray(hs, float)
    if PIVOT_BACK_CM == 0:
        return np.full(len(hs), p[0]), np.full(len(hs), p[1])
    qx = p[0] - PIVOT_BACK_CM * math.cos(math.radians(h_from)); qy = p[1] - PIVOT_BACK_CM * math.sin(math.radians(h_from))
    return qx + PIVOT_BACK_CM * np.cos(np.radians(hs)), qy + PIVOT_BACK_CM * np.sin(np.radians(hs))


def pivot_interval(p, h):
    """点pで向きhから、その場で回れる最大の角度(左回り, 右回り)。"""
    key = (round(float(p[0]), 3), round(float(p[1]), 3), round(float(h), 3))
    if key not in _piv_int:
        ks = np.arange(1, 182)
        hs = np.concatenate([h + ks, h - ks])
        ax, ay = _pivot_axle(p, h, hs)
        ok = poses_clear(ax, ay, hs)
        res = []
        for part in (ok[:181], ok[181:]):
            bad = np.flatnonzero(~part)
            res.append(181 if len(bad) == 0 else int(bad[0]))
        _piv_int[key] = tuple(res)
    return _piv_int[key]


_KO_CORNERS = None
_START_EXEMPT = None
_EXEMPT_ON = False   # スタートから最初のゲートへのつなぎ区間を判定している間だけ True


def body_keepout(px, py, hdeg):
    """姿勢(配列)のどれかで、車体の箱が進入禁止の長方形にかかるか。"""
    if not KEEPOUT:
        return False
    return bool(np.any(body_keepout_mask(px, py, hdeg)))


_BODY_OUTLINE = None   # 車体の箱の輪郭の点(車体の座標、cm)


def body_keepout_mask(px, py, hdeg):
    """姿勢ごとに、車体の箱が進入禁止の長方形にかかるか(bool配列)。1つずつ呼ぶより速いように、まとめて計算する。"""
    global _KO_CORNERS, _START_EXEMPT, _BODY_OUTLINE
    px = np.asarray(px, float); py = np.asarray(py, float)
    if not KEEPOUT:
        return np.zeros(len(px), bool)
    if not KEEPOUT_BODY:
        return in_keepout(px, py)
    h = np.radians(np.asarray(hdeg, float))
    if _KO_CORNERS is None:
        _KO_CORNERS = [[(x, y) for x in (x0, x1) for y in (y0, y1) if abs(x) < INF / 2 and abs(y) < INF / 2]
                       for x0, x1, y0, y1 in KEEPOUT]
    if _BODY_OUTLINE is None:
        _BODY_OUTLINE = (np.concatenate([np.linspace(-REAR, FRONT, 14), np.full(8, FRONT), np.linspace(FRONT, -REAR, 14), np.full(8, -REAR)]),
                         np.concatenate([np.full(14, HALF_WIDTH), np.linspace(HALF_WIDTH, -HALF_WIDTH, 8), np.full(14, -HALF_WIDTH),
                                         np.linspace(-HALF_WIDTH, HALF_WIDTH, 8)]))
    xs, ys = _BODY_OUTLINE
    ch = np.cos(h); sh = np.sin(h)
    c = ch[:, None]; sn = sh[:, None]
    if not hr_kernels.AVAILABLE:
        bx = px[:, None] + xs * c - ys * sn; by = py[:, None] + xs * sn + ys * c
    if _START_EXEMPT is None:
        sb = [(START[0] + a * math.cos(math.radians(START_H)) - b * math.sin(math.radians(START_H)),
               START[1] + a * math.sin(math.radians(START_H)) + b * math.cos(math.radians(START_H)))
              for a, b in zip(xs, ys)]
        _START_EXEMPT = {k for k, (x0, x1, y0, y1) in enumerate(KEEPOUT) if any(x0 < x < x1 and y0 < y < y1 for x, y in sb)}
    near_start = np.hypot(px - START[0], py - START[1]) <= START_EXEMPT_CM
    out = np.zeros(len(px), bool)
    reach = math.hypot(max(FRONT, REAR), HALF_WIDTH) + 1.0   # 車軸から車体の一番遠い点(少し余裕)
    for k, (x0, x1, y0, y1) in enumerate(KEEPOUT):
        if px.max() + reach < x0 or px.min() - reach > x1 or py.max() + reach < y0 or py.min() - reach > y1:
            continue   # どの姿勢でも車体が届かない
        if hr_kernels.AVAILABLE:
            hit = hr_kernels.keepout_hit(px, py, ch, sh, xs, ys, float(x0), float(x1), float(y0), float(y1),
                                         np.array(_KO_CORNERS[k], float).reshape(-1, 2), float(FRONT), float(REAR), float(HALF_WIDTH))
        else:
            hit = np.any((bx > x0) & (bx < x1) & (by > y0) & (by < y1), axis=1)
            for cx, cy in _KO_CORNERS[k]:   # 長方形の角が車体の中に入る場合
                lx = (cx - px) * ch + (cy - py) * sh; ly = -(cx - px) * sh + (cy - py) * ch
                hit |= (lx > -REAR) & (lx < FRONT) & (np.abs(ly) < HALF_WIDTH)
        if k in _START_EXEMPT and _EXEMPT_ON:
            hit &= ~near_start
        out |= hit
    return out


def in_keepout(px, py):
    bad = np.zeros(len(px), bool)
    for x0, x1, y0, y1 in KEEPOUT:
        bad |= (px > x0) & (px < x1) & (py > y0) & (py < y1)
    return bad


def _orient(ax, ay, bx, by, cx, cy):
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)


def crosses_gate(xs, ys, allow=None):
    """折れ線(xs,ys)がゲートの線を横切るか(allowの色は除く)。"""
    ax, ay, bx, by = xs[:-1], ys[:-1], xs[1:], ys[1:]
    for pa, pb, col in GSEG:
        if col == allow or (_EXEMPT_ON and START_ANY_GATE and col != 'red'):
            continue
        if hr_kernels.AVAILABLE:
            if hr_kernels.seg_cross(xs, ys, float(pa[0]), float(pa[1]), float(pb[0]), float(pb[1])):
                return True
            continue
        o1 = _orient(ax, ay, bx, by, pa[0], pa[1]); o2 = _orient(ax, ay, bx, by, pb[0], pb[1])
        o3 = _orient(pa[0], pa[1], pb[0], pb[1], ax, ay); o4 = _orient(pa[0], pa[1], pb[0], pb[1], bx, by)
        if np.any((o1 * o2 < 0) & (o3 * o4 < 0)):
            return True
    return False


_line_memo = {}   # (始点, 終点, 前後, allow, スタートの除外中か) -> 当たらないか。同じ直線を何度も調べるので覚えて使い回す


def line_ok(a, b, m, allow=None):
    key = (float(a[0]), float(a[1]), float(b[0]), float(b[1]), m, allow, _EXEMPT_ON)
    ok = _line_memo.get(key)
    if ok is None:
        ok = _line_memo[key] = _line_check(a, b, m, allow)
    return ok


def _line_check(a, b, m, allow=None):
    v = b - a; L = float(np.hypot(*v))
    if L < 1e-6:
        return True
    h = math.atan2(v[1], v[0]) + (0 if m > 0 else math.pi)
    if m > 0:
        f0, f1, r0, r1 = 0.0, L + FRONT, -REAR, L
    else:
        f0, f1, r0, r1 = -L, FRONT, -L - REAR, 0.0
    n = max(2, int(L / 2) + 1)
    xs = np.linspace(a[0], b[0], n); ys = np.linspace(a[1], b[1], n)
    if crosses_gate(xs, ys, allow) or np.any(in_keepout(xs, ys)):
        return False
    px, py, hh = np.array([a[0]]), np.array([a[1]]), np.array([h])
    if box_clear(px, py, hh, f0, f1)[0] < MARGIN_FRONT or box_clear(px, py, hh, r0, r1)[0] < MARGIN_REAR:
        return False
    nb = max(2, int(L) + 1)
    return not body_keepout(np.linspace(a[0], b[0], nb), np.linspace(a[1], b[1], nb), np.full(nb, math.degrees(h)))


def nL(phi):
    return np.array([-math.sin(phi), math.cos(phi)])


def tdir(phi):
    return np.array([math.cos(phi), math.sin(phi)])


def ccw(x):
    x = x % TAU
    return 0.0 if x > TAU - 1e-7 else x


def arc_prim(p, theta, s, sweep, r, m):
    """点pで進む向き(動く向き)theta、s=+1左/-1右に sweep(rad) の円弧。m=+1前進/-1後退。"""
    return ('arc', p + s * r * nL(theta), r, theta, s, sweep, m, p)


def arc_end(pr):
    _, c, r, theta, s, sweep, m, p0 = pr
    th = theta + s * sweep
    return c - s * r * nL(th), th


def arc_samples(pr, step=1.0):
    _, c, r, theta, s, sweep, m, p0 = pr
    n = max(2, int(r * sweep / step) + 1, int(math.degrees(sweep)) + 1)   # 向き1°ごとにも(後ろの角は車軸より大きく振れる)
    th = theta + s * sweep * np.arange(n + 1) / n
    xs = c[0] + s * r * np.sin(th); ys = c[1] - s * r * np.cos(th)
    hb = np.degrees(th) + (0 if m > 0 else 180)
    return xs, ys, hb


_arc_memo = {}   # (始点, 向き, 半径, 回る向き, 前後, allow) -> [当たらないと確かめた角度, 当たる最初の角度]


def arc_ok(pr, allow=None):
    """円弧が当たらないか。同じ始点・半径・回る向きの円弧は、どこまで当たらないかを覚えて使い回す。"""
    if pr[5] < 1e-6:
        return True
    _, c, r, theta, sgn, sweep, m, p0 = pr
    key = (round(float(p0[0]), 3), round(float(p0[1]), 3), round(float(theta), 6), r, sgn, m, allow, _EXEMPT_ON)
    memo = _arc_memo.get(key)
    if memo is not None:
        if memo[1] is not None and sweep >= memo[1] - 1e-9:
            return False
        if sweep <= memo[0] + 1e-9:
            return True
    ok, fail_at = _arc_check(pr, allow)
    if memo is None:
        memo = _arc_memo[key] = [0.0, None]
    if ok:
        memo[0] = max(memo[0], sweep)
    else:
        memo[1] = fail_at if memo[1] is None else min(memo[1], fail_at)
    return ok


def _arc_check(pr, allow=None):
    """(当たらないか, 当たる最初の角度rad)"""
    _, c, r, theta, sgn, sweep, m, p0 = pr
    xs, ys, hb = arc_samples(pr)
    n = len(xs) - 1
    okp = poses_clear(xs, ys, hb)
    if KEEPOUT:
        okp &= ~body_keepout_mask(xs, ys, hb)
    bad = np.flatnonzero(~okp)
    if len(bad):
        return False, sweep * max(bad[0] - 1, 0) / n
    if KEEPOUT and np.any(in_keepout(xs, ys)):
        return False, 0.0
    if crosses_gate(xs, ys, allow):
        return False, 0.0
    return True, None


def _arc_ok_old(pr, allow=None):
    if pr[5] < 1e-6:
        return True
    xs, ys, hb = arc_samples(pr)
    if not poses_ok(xs, ys, hb):
        return False
    if np.any(in_keepout(xs, ys)) or body_keepout(xs, ys, hb):
        return False
    return not crosses_gate(xs, ys, allow)


_pivot_cache = {}


def pivot_ok(p, h_from, h_to):
    dd = nd(h_to - h_from)
    if abs(dd) < 0.3:
        return True
    key = (round(float(p[0]), 2), round(float(p[1]), 2), round(h_from, 2), round(h_to, 2), _EXEMPT_ON)
    if key not in _pivot_cache:
        hs = h_from + dd * np.linspace(0, 1, int(abs(dd)) + 2)
        ax, ay = _pivot_axle(p, h_from, hs)
        _pivot_cache[key] = poses_ok(ax, ay, hs) and not body_keepout(ax, ay, hs)
    return _pivot_cache[key]


def prim_ok(pr):
    if pr[0] == 'line':
        return line_ok(pr[1], pr[2], pr[3])
    if pr[0] == 'arc':
        return arc_ok(pr)
    return pivot_ok(pr[1], pr[2], pr[3])


def prim_len(pr):
    if pr[0] == 'line':
        return float(np.hypot(*(pr[2] - pr[1])))
    if pr[0] == 'arc':
        return pr[2] * pr[5]
    return 0.0


def prim_m(pr):
    return pr[3] if pr[0] == 'line' else (pr[6] if pr[0] == 'arc' else None)


def seq_cost(dE, prims, dS):
    """前のゲートの最後の走りの向きdE(0=止まっている)、つなぎのプリミティブ、次のゲートの最初の走りの向きdS(0=ゴール)。
    時間+重み。止まる直前が直進(tail_cm以上)でなければINF。"""
    t = 0.0; stops = 0; pivots = 0; rev = 0.0; arcdeg = 0.0
    cur = dE            # 今の動きの向き(0=止まっている)
    last = 'visit'      # 止まる直前の区間('visit'は通過の直進)
    last_len = INF
    pending_pivot = False

    def stop_here():
        nonlocal stops, cur
        if cur != 0:
            if not (last == 'visit' or (last == 'line' and last_len >= TAIL_CM - 1e-6)):
                return False
            stops += 1
        cur = 0
        return True

    for pr in prims:
        if pr[0] == 'pivot':
            deg = abs(nd(pr[3] - pr[2]))
            if deg < 0.3:
                continue
            if not stop_here():
                return INF
            t += turn_cost(deg); pivots += 1
            continue
        L = prim_len(pr)
        if L < 1e-6:
            continue
        m = prim_m(pr)
        if cur != 0 and cur != m:
            if not stop_here():
                return INF
        t += (L / V_LINE if pr[0] == 'line' else L / v_arc(pr[2])) * spd(m)
        if m < 0:
            rev += L
        if pr[0] == 'arc':
            arcdeg += math.degrees(pr[5])
        cur = m; last = pr[0]; last_len = L
    if dS == 0 or (cur != 0 and cur != dS):
        if not stop_here():
            return INF
    return (t + T_STOP * stops + PEN_STOP * stops + PEN_PIVOT * pivots + PEN_REV_M * rev / 100.0
            + PEN_ARC_100DEG * arcdeg / 100.0)


# ---------------- Dubins(配列) ----------------
WORDS = [('CSC', 1, 1), ('CSC', 1, -1), ('CSC', -1, 1), ('CSC', -1, -1),
         ('CCC', 1, 1), ('CCC', 1, -1), ('CCC', -1, 1), ('CCC', -1, -1)]


def _ccw_v(x):
    x = np.mod(x, TAU)
    return np.where(x > TAU - 1e-7, 0.0, x)


def dubins_vec(x0, y0, t0, x1, y1, t1, r):
    """8つの形の (円弧の角度の合計rad, 直進の長さcm)。だめなものは角度INF。shape (8, N)。"""
    if hr_kernels.AVAILABLE and np.ndim(x0) == 1:
        return hr_kernels.dubins_vec(*[np.asarray(v, float) for v in (x0, y0, t0, x1, y1, t1)], float(r), float(INF))
    A, L = [], []
    for kind, s1, s2 in WORDS:
        if kind == 'CSC':
            c1x = x0 - s1 * r * np.sin(t0); c1y = y0 + s1 * r * np.cos(t0)
            c2x = x1 - s2 * r * np.sin(t1); c2y = y1 + s2 * r * np.cos(t1)
            dx, dy = c2x - c1x, c2y - c1y
            D = np.hypot(dx, dy); k = (s1 - s2) * r
            ok = D > abs(k) + 1e-9
            Ds = np.where(ok, D, 1.0)
            phi = np.arctan2(dy, dx) + np.arcsin(np.clip(k / Ds, -1, 1))
            l = np.sqrt(np.maximum(Ds ** 2 - k ** 2, 0))
            a = _ccw_v(s1 * (phi - t0)) + _ccw_v(s2 * (t1 - phi))
        else:
            s, sg = s1, s2
            c1x = x0 - s * r * np.sin(t0); c1y = y0 + s * r * np.cos(t0)
            c3x = x1 - s * r * np.sin(t1); c3y = y1 + s * r * np.cos(t1)
            dx, dy = c3x - c1x, c3y - c1y
            D = np.hypot(dx, dy)
            ok = (D > 1e-9) & (D < 4 * r)
            al = np.arctan2(dy, dx); be = np.arccos(np.clip(D / (4 * r), -1, 1))
            c2x = c1x + 2 * r * np.cos(al + sg * be); c2y = c1y + 2 * r * np.sin(al + sg * be)
            th1 = np.arctan2(-(c1x - c2x) / (2 * s * r), (c1y - c2y) / (2 * s * r))
            th2 = np.arctan2(-(c3x - c2x) / (2 * s * r), (c3y - c2y) / (2 * s * r))
            a = _ccw_v(s * (th1 - t0)) + _ccw_v(-s * (th2 - th1)) + _ccw_v(s * (t1 - th2))
            l = np.zeros_like(a)
        A.append(np.where(ok, a, INF)); L.append(np.where(ok, l, 0.0))
    return np.array(A), np.array(L)


_arc_limit_memo = {}


def _arc_limit(p, th, s, r, m):
    """点 p から動く向き th で円弧を描いたとき、確実に当たる最初の角度(rad)。当たらなければ INF。
    2026-11-09: 同じ始点・半径・回る向きの円弧は何度も出てくるので1周分を1回だけ調べ、当たる候補は形を作らずに捨てる。
    1°おきの点が2つ続けて当たる所だけを見るので、通れる円弧を捨てることはない(判定は1°以下の間隔のため)。"""
    key = (round(float(p[0]), 3), round(float(p[1]), 3), round(float(th), 6), r, s, m, _EXEMPT_ON)
    v = _arc_limit_memo.get(key)
    if v is None:
        n = 360
        ths = th + s * (TAU * 359.0 / 360.0) * np.arange(n) / (n - 1)
        cx = p[0] - s * r * math.sin(th); cy = p[1] + s * r * math.cos(th)
        xs = cx + s * r * np.sin(ths); ys = cy - s * r * np.cos(ths)
        hb = np.degrees(ths) + (0 if m > 0 else 180)
        bad = ~poses_clear(xs, ys, hb)
        if KEEPOUT:
            bad |= body_keepout_mask(xs, ys, hb)
        both = np.flatnonzero(bad[:-1] & bad[1:])
        v = _arc_limit_memo[key] = float(both[0]) * (TAU * 359.0 / 360.0) / (n - 1) if len(both) else INF
    return v


_LIMIT_MARGIN = math.radians(3.0)


def _dubins_fast(A, thA, B, thB, r, w, m):
    ok, a1, a2, a3, th1, th2, p1x, p1y, p2x, p2y, c1x, c1y, c2x, c2y, c3x, c3y = hr_kernels.dubins_params(
        float(A[0]), float(A[1]), float(thA), float(B[0]), float(B[1]), float(thB), float(r), w)
    if not ok:
        return None
    kind, s1, s2 = WORDS[w]
    last_s = s2 if kind == 'CSC' else s1
    arc1 = ('arc', np.array([c1x, c1y]), r, thA, s1, a1, m, A)
    p1 = np.array([p1x, p1y]); p2 = np.array([p2x, p2y])
    if kind == 'CSC':
        prims = [arc1, ('line', p1, p2, m), ('arc', np.array([c2x, c2y]), r, th2, s2, a3, m, p2)]
    else:
        prims = [arc1, ('arc', np.array([c2x, c2y]), r, th1, -s1, a2, m, p1), ('arc', np.array([c3x, c3y]), r, th2, s1, a3, m, p2)]
    return [q for q in prims if prim_len(q) > 1e-6]


def dubins_one(A, thA, B, thB, r, w, m):
    """形wの Dubins 経路(プリミティブの列)。だめならNone。thA/thB は動く向き。"""
    if hr_kernels.AVAILABLE:
        return _dubins_fast(A, thA, B, thB, r, w, m)
    kind, s1, s2 = WORDS[w]
    if kind == 'CSC':
        c1 = A + s1 * r * nL(thA); c2 = B + s2 * r * nL(thB)
        D = float(np.hypot(*(c2 - c1))); k = (s1 - s2) * r
        if D <= abs(k) + 1e-9:
            return None
        phi = math.atan2(*(c2 - c1)[::-1]) + math.asin(k / D)
        p1 = c1 - s1 * r * nL(phi); p2 = c2 - s2 * r * nL(phi)
        prims = [arc_prim(A, thA, s1, ccw(s1 * (phi - thA)), r, m), ('line', p1, p2, m),
                 arc_prim(p2, phi, s2, ccw(s2 * (thB - phi)), r, m)]
    else:
        s, sg = s1, s2
        c1 = A + s * r * nL(thA); c3 = B + s * r * nL(thB)
        D = float(np.hypot(*(c3 - c1)))
        if not (1e-9 < D < 4 * r):
            return None
        al = math.atan2(*(c3 - c1)[::-1]); be = math.acos(D / (4 * r))
        c2 = c1 + 2 * r * tdir(al + sg * be)
        p1 = (c1 + c2) / 2; p2 = (c2 + c3) / 2
        v1 = (c1 - p1) / (s * r); th1 = math.atan2(-v1[0], v1[1])
        v2 = (c3 - p2) / (s * r); th2 = math.atan2(-v2[0], v2[1])
        prims = [arc_prim(A, thA, s, ccw(s * (th1 - thA)), r, m),
                 arc_prim(p1, th1, -s, ccw(-s * (th2 - th1)), r, m),
                 arc_prim(p2, th2, s, ccw(s * (thB - th2)), r, m)]
    p, th = arc_end(prims[-1])
    if np.hypot(*(p - B)) > 1e-3 or abs(nd(math.degrees(th - thB))) > 1e-3:
        return None
    return [q for q in prims if prim_len(q) > 1e-6]


# ---------------- 姿勢とゲートの通り方 ----------------
DIRS = (1, -1) if ALLOW_REVERSE else (1,)


def _rot(v, deg):
    c, sn = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([v[0] * c - v[1] * sn, v[0] * sn + v[1] * c])


def _pass_prims(g, C, base_u, a_in, a_out, lead, trail, m, ea, ee, r):
    """ゲートの通過のプリミティブ。入りは向き a_in の直進で車軸が C(ゲートの線上)に着くまで、
    a_in != a_out なら C から半径 r の円弧で a_out に向きを変え、a_out の直進で抜ける。
    入りの条件: 先の端の角が線の CLR 手前、抜けの条件: 後の端の角が線を PM 越える(線に垂直な距離)。"""
    u_in, u_out = _rot(base_u, a_in), _rot(base_u, a_out)
    ci, si = math.cos(math.radians(a_in)), abs(math.sin(math.radians(a_in)))
    co, so = math.cos(math.radians(a_out)), abs(math.sin(math.radians(a_out)))
    S = C - u_in * ((lead * ci + HALF_WIDTH * si + CLR) / ci + ea)
    need = trail * co + HALF_WIDTH * so + PM
    prims = [('line', S, C, m)]
    P2 = C
    if abs(a_out - a_in) > 1e-9:
        th_in = math.atan2(u_in[1], u_in[0]); th_out = math.atan2(u_out[1], u_out[0])
        dd = nd(math.degrees(th_out - th_in))
        sgn = 1 if dd > 0 else -1
        arc = arc_prim(C, th_in, sgn, math.radians(abs(dd)), r, m)
        P2, _ = arc_end(arc)
        prims.append(arc)
    d2 = float(np.dot(P2 - C, base_u))
    L = (need - d2) / co
    if L < 0:
        return None
    E = P2 + u_out * (L + ee)
    prims.append(('line', P2, E, m))
    return prims


def _visit_candidates(g, pairs_override=None, only=None):
    """ゲートの通り方の候補。斜め(angles が0以外)は、車軸の中心がゲートの真ん中を通るときだけ(横のずれは0)。"""
    out = []
    modes = ['F', 'B', 'FB', 'BF'] if ALLOW_REVERSE else ['F']
    pairs = [(0.0, 0.0)]
    for a in ([] if pairs_override is not None else ANGLES):
        if abs(a) < 1e-9:
            continue
        if OBLIQUE_MODE in ('split', 'both'):
            pairs += [(a, 0.0), (0.0, a)]
        if OBLIQUE_MODE in ('straight', 'both'):
            pairs.append((a, a))
    for sg in (1, -1):
        base_u = g.n * sg
        for a_in, a_out, *rr in (pairs_override if pairs_override is not None else pairs):
            oblique = abs(a_in) > 1e-9 or abs(a_out) > 1e-9
            radii = (rr if rr else SPLIT_RADII) if abs(a_in - a_out) > 1e-9 else [0.0]
            for off in (OFFSETS if not oblique else [0.0]):
                C = g.c + g.t * off
                for mode in modes:
                    if only is not None and (sg, mode) not in only:
                        continue
                    if mode in ('F', 'FB'):
                        lead, trail, m = FRONT, REAR, 1
                    else:
                        lead, trail, m = REAR, FRONT, -1
                    for r in radii:
                        for ea in EXT:
                            for ee in EXT:
                                if mode in ('F', 'B'):
                                    prims = _pass_prims(g, C, base_u, a_in, a_out, lead, trail, m, ea, ee, r)
                                    if prims is None:
                                        continue
                                elif oblique:
                                    # 斜めの往復: 斜めの通過(抜けの余りなし)で抜けきったら止まり、同じ道を後退で戻って、
                                    # 入りの点からさらに ee だけ戻る(戻りも行きと同じ道なので、柱とのすき間も同じ)
                                    fwd = _pass_prims(g, C, base_u, a_in, a_out, lead, trail, m, ea, 0.0, r)
                                    if fwd is None:
                                        continue
                                    back = [_retrace(q) for q in reversed(fwd)]
                                    S0 = fwd[0][1]; u_in = _rot(base_u, a_in)
                                    prims = fwd + back + ([('line', S0, S0 - u_in * ee, -m)] if ee > 0 else [])
                                else:
                                    # 往復(垂直だけ): 抜けきったら戻って、元の側へ抜けきる
                                    u = base_u
                                    S = C - u * (lead + CLR + ea); T = C + u * (trail + PM)
                                    E = C - u * ((FRONT if mode == 'FB' else REAR) + CLR + ee)
                                    prims = [('line', S, T, m), ('line', T, E, -m)]
                                ok = True
                                for q in prims:
                                    if q[0] == 'line' and not line_ok(q[1], q[2], q[3], allow=g.color):
                                        ok = False; break
                                    if q[0] == 'arc' and not arc_ok(q, allow=g.color):
                                        ok = False; break
                                if not ok:
                                    continue
                                t = sum((prim_len(q) / (V_LINE if q[0] == 'line' else v_arc(q[2]))) * spd(prim_m(q)) for q in prims)
                                rv = sum(prim_len(q) for q in prims if prim_m(q) < 0)
                                if mode in ('FB', 'BF'):
                                    t += T_STOP + PEN_STOP
                                t += PEN_REV_M * rv / 100.0
                                S = prims[0][1]
                                last = prims[-1]
                                E = last[2]
                                hS = nd(math.degrees(math.atan2(*(prims[0][2] - prims[0][1])[::-1])) + (0 if prims[0][3] > 0 else 180))
                                hE = nd(math.degrees(math.atan2(*(last[2] - last[1])[::-1])) + (0 if last[3] > 0 else 180))
                                out.append(dict(color=g.color, mode=mode, sign=sg, off=off, angle=a_in, angle_out=a_out, ea=ea, ee=ee,
                                                prims=prims, legs=[(q[1], q[2], q[3]) for q in prims if q[0] == 'line'], cost=t,
                                                start=(S, hS, prims[0][3]), end=(E, hE, last[3])))
    return out


def pass_clearance(prims):
    """通過の間(プリミティブ列)の、車体と支柱の一番小さいすき間(前と後ろの小さい方、cm)。"""
    xs, ys, hs = [], [], []
    for q in prims:
        if q[0] == 'line':
            L = prim_len(q)
            k = np.linspace(0, 1, max(2, int(L / 0.5) + 1))
            h = math.degrees(math.atan2(*(q[2] - q[1])[::-1])) + (0 if q[3] > 0 else 180)
            xs.append(q[1][0] + (q[2][0] - q[1][0]) * k); ys.append(q[1][1] + (q[2][1] - q[1][1]) * k)
            hs.append(np.full(len(k), h))
        elif q[0] == 'arc':
            ax, ay, ah = arc_samples(q, 0.5)
            xs.append(ax); ys.append(ay); hs.append(ah)
    px, py, hh = np.concatenate(xs), np.concatenate(ys), np.radians(np.concatenate(hs))
    if GATE_MODEL == 'rect':
        cf = _rect_clear(px, py, hh, 0.0, FRONT); cr = _rect_clear(px, py, hh, -REAR, 0.0)
        return float(min(cf.min() - MARGIN_FRONT, cr.min() - MARGIN_REAR))
    c = np.cos(hh)[:, None]; sn = np.sin(hh)[:, None]
    dx = CAP[None, :, 0] - px[:, None]; dy = CAP[None, :, 1] - py[:, None]
    lx = dx * c + dy * sn; ly = -dx * sn + dy * c
    ey = np.maximum(np.abs(ly) - HALF_WIDTH, 0.0)
    cf = np.hypot(np.maximum(np.maximum(-lx, lx - FRONT), 0.0), ey) - POST_R
    cr = np.hypot(np.maximum(np.maximum(-REAR - lx, lx), 0.0), ey) - POST_R
    return float(min(cf.min() - MARGIN_FRONT, cr.min() - MARGIN_REAR))


OBLIQUE_CHOICE = []   # 記録: (色, 側, 通り方, 入り, 抜け, 半径, すき間)


def _retrace(q):
    """プリミティブを逆向きにたどる(後退と前進を入れ替える)。"""
    if q[0] == 'line':
        return ('line', q[2], q[1], -q[3])
    _, c, r, theta, sgn, sweep, m, p0 = q
    p_end, th_end = arc_end(q)
    return arc_prim(p_end, th_end + math.pi, -sgn, sweep, r, -m)


def visit_candidates(g):
    if OBLIQUE_POLICY != 'safest':
        return _visit_candidates(g)
    perp = _visit_candidates(g, pairs_override=[(0.0, 0.0)])
    have = {(v['sign'], v['mode']) for v in perp}
    modes = ['F', 'B'] if ALLOW_REVERSE else ['F']
    modes = ['F', 'B', 'FB', 'BF'] if ALLOW_REVERSE else ['F']
    missing = [(sg, mo) for sg in (1, -1) for mo in modes if (sg, mo) not in have]
    out = list(perp)
    lo, hi, step = ANGLE_SCAN
    angles = [a for a in np.arange(lo, hi + 1e-9, step) if abs(a) > 1e-9]
    for sg, mo in missing:
        base_u = g.n * sg
        lead, trail, m = (FRONT, REAR, 1) if mo in ('F', 'FB') else (REAR, FRONT, -1)
        # 入りを斜め / 抜けを斜め の両方から、通過の間の一番近い柱までの距離が最大のものを1つだけ選ぶ
        best = None
        # 候補: 入りだけ斜め / 抜けだけ斜め / (OBLIQUE_BOTH なら)入りも抜けも斜め(同じ角度の1本の線と、4°刻みの違う角度)
        trials = [(a, 0.0, r) for a in angles for r in SPLIT_RADII] + [(0.0, a, r) for a in angles for r in SPLIT_RADII]
        if OBLIQUE_BOTH:
            trials += [(a, a, 0.0) for a in angles]
            coarse = [a for a in angles if abs(round(a) % 4) < 1e-9]
            trials += [(a1, a2, 20.0) for a1 in coarse for a2 in coarse if abs(a1 - a2) > 1e-9]
        for a_in, a_out, r in trials:
            prims = _pass_prims(g, g.c, base_u, a_in, a_out, lead, trail, m, 0.0, 0.0, r)
            if prims is None:
                continue
            cl = pass_clearance(prims)
            # すき間が1mm以内で同じなら、斜めが片側だけの方、次に角度が小さい方を選ぶ
            pref = (int(abs(a_in) > 1e-9) + int(abs(a_out) > 1e-9), max(abs(a_in), abs(a_out)))
            if best is None or cl > best[0] + 0.1 or (cl > best[0] - 0.1 and pref < best[4]):
                best = (cl, a_in, a_out, r, pref)
        if best is None or best[0] < 0:
            continue
        OBLIQUE_CHOICE.append((g.color, sg, mo, float(best[1]), float(best[2]), best[3], round(best[0] + MARGIN_FRONT, 2)))
        out += _visit_candidates(g, pairs_override=[(best[1], best[2], best[3])], only={(sg, mo)})
    return out


def pose_key(p):
    return (round(float(p[0][0]), 3), round(float(p[0][1]), 3), round(float(p[1]), 3), p[2])


# ---------------- つなぎ区間の候補(当たりを無視した時間を配列で) ----------------
# 候補 = (族, m, r, 形 or 回る向き)。族: 'run' 1本の走り / 'PLP' 旋回-直進-旋回 / 'PLA' 旋回-直進-円弧 / 'ALP' 円弧-直進-旋回
OPTS = []
for m in DIRS:
    for r in RADII:
        for w in range(8):
            OPTS.append(('run', m, r, w))
if ALLOW_PIVOT:
    for m in DIRS:
        OPTS.append(('PLP', m, 0.0, 0))
        for r in RADII:
            for s1 in (1, -1):
                OPTS.append(('PLA', m, r, s1))
                OPTS.append(('ALP', m, r, s1))


def _tc(deg):
    return np.where(deg < 0.3, 0.0, T_TURN0 + T_TURN_DEG * deg)


def _piv_bad(dd, lmax, rmax):
    """向きの変化dd(符号つき、deg)が、その場で回れる範囲を超えるか(配列)。"""
    a = np.abs(dd)
    ok = (a < 0.3) | ((dd > 0) & (lmax >= dd - 0.5)) | ((dd < 0) & (rmax >= -dd - 0.5)) | \
         ((np.abs(a - 180) < 0.5) & ((lmax >= 180) | (rmax >= 180)))
    return ~ok


def approx_costs(E, S):
    """E, S: 姿勢の配列 (x, y, 車体の向きdeg, 走りの向き 0/±1)。戻り値 (N, len(OPTS))。"""
    ex, ey, eh, ed, el, er = E; sx, sy, sh, sd, sl, sr = S
    N = len(ex)
    out = np.full((N, len(OPTS)), INF)
    dcache = {}
    for j, (fam, m, r, w) in enumerate(OPTS):
        add = 0.0 if m > 0 else 180.0
        stopE_run = (ed != 0) & (ed != m)
        stopS_run = (sd == 0) | (sd != m)
        if fam == 'run':
            if (m, r) not in dcache:
                thE = np.radians(eh + add); thS = np.radians(sh + add)
                tx = sx - TAIL_CM * stopS_run * np.cos(thS); ty = sy - TAIL_CM * stopS_run * np.sin(thS)
                dcache[(m, r)] = dubins_vec(ex, ey, thE, tx, ty, thS, r)
            A, L = dcache[(m, r)]
            a, l = A[w], L[w]
            t = (r * a / v_arc(r) + (l + TAIL_CM * stopS_run) / V_LINE) * spd(m)
            st = stopE_run.astype(float) + stopS_run
            c = t + (T_STOP + PEN_STOP) * st + PEN_ARC_100DEG * np.degrees(a) / 100.0 \
                + (PEN_REV_M * (r * a + l + TAIL_CM * stopS_run) / 100.0 if m < 0 else 0.0)
            out[:, j] = np.where(a >= INF / 2, INF, c)
        elif fam == 'PLP':
            vx, vy = sx - ex, sy - ey
            L = np.hypot(vx, vy)
            hl = np.degrees(np.arctan2(vy, vx)) + add
            pbad = _piv_bad(nd(hl - eh), el, er) | _piv_bad(-nd(sh - hl), sl, sr)
            pe = np.abs(nd(hl - eh)); ps = np.abs(nd(sh - hl))
            stE = (ed != 0) & ~((pe < 0.3) & (ed == m))
            stS = ~((ps < 0.3) & (sd == m))
            c = _tc(pe) + _tc(ps) + L / V_LINE * spd(m) + (T_STOP + PEN_STOP) * (stE.astype(float) + stS) \
                + PEN_PIVOT * ((pe >= 0.3).astype(float) + (ps >= 0.3)) + (PEN_REV_M * L / 100.0 if m < 0 else 0.0)
            out[:, j] = np.where((L < TAIL_CM) | pbad, INF, c)
        elif fam == 'PLA':
            s1 = w
            thB = np.radians(sh + add)
            bx = sx - TAIL_CM * stopS_run * np.cos(thB); by = sy - TAIL_CM * stopS_run * np.sin(thB)
            c2x = bx - s1 * r * np.sin(thB); c2y = by + s1 * r * np.cos(thB)
            D = np.hypot(c2x - ex, c2y - ey)
            ok = D > r + 1e-9
            phi = np.arctan2(c2y - ey, c2x - ex) - np.arcsin(np.clip(s1 * r / np.where(ok, D, 1.0), -1, 1))
            p2x = c2x + s1 * r * np.sin(phi); p2y = c2y - s1 * r * np.cos(phi)
            a2 = _ccw_v(s1 * (thB - phi))
            L = np.hypot(p2x - ex, p2y - ey)
            pbad = _piv_bad(nd(np.degrees(phi) + add - eh), el, er)
            pe = np.abs(nd(np.degrees(phi) + add - eh))
            stE = (ed != 0) & ~((pe < 0.3) & (ed == m))
            c = _tc(pe) + (L / V_LINE + r * a2 / v_arc(r) + TAIL_CM * stopS_run / V_LINE) * spd(m) \
                + (T_STOP + PEN_STOP) * (stE.astype(float) + stopS_run) + PEN_PIVOT * (pe >= 0.3) \
                + PEN_ARC_100DEG * np.degrees(a2) / 100.0 + (PEN_REV_M * (L + r * a2) / 100.0 if m < 0 else 0.0)
            out[:, j] = np.where(ok & ~pbad, c, INF)
        else:  # ALP
            s1 = w
            thA = np.radians(eh + add)
            c1x = ex - s1 * r * np.sin(thA); c1y = ey + s1 * r * np.cos(thA)
            D = np.hypot(sx - c1x, sy - c1y)
            ok = D > r + 1e-9
            phi = np.arctan2(sy - c1y, sx - c1x) + np.arcsin(np.clip(s1 * r / np.where(ok, D, 1.0), -1, 1))
            p1x = c1x + s1 * r * np.sin(phi); p1y = c1y - s1 * r * np.cos(phi)
            a1 = _ccw_v(s1 * (phi - thA))
            L = np.hypot(sx - p1x, sy - p1y)
            pbad = _piv_bad(-nd(sh - (np.degrees(phi) + add)), sl, sr)
            ps = np.abs(nd(sh - (np.degrees(phi) + add)))
            stS = ~((ps < 0.3) & (sd == m))
            c = _tc(ps) + (L / V_LINE + r * a1 / v_arc(r)) * spd(m) \
                + (T_STOP + PEN_STOP) * (stopE_run.astype(float) + stS) + PEN_PIVOT * (ps >= 0.3) \
                + PEN_ARC_100DEG * np.degrees(a1) / 100.0 + (PEN_REV_M * (L + r * a1) / 100.0 if m < 0 else 0.0)
            out[:, j] = np.where(ok & (L >= TAIL_CM) & ~pbad, c, INF)
    return out


_OPT_FAM = np.array([{'run': 0, 'PLP': 1, 'PLA': 2, 'ALP': 3}[o[0]] for o in OPTS], np.int64)
_OPT_M = np.array([o[1] for o in OPTS], np.int64)
_OPT_R = np.array([float(o[2]) for o in OPTS], float)
_OPT_W = np.array([int(o[3]) for o in OPTS], np.int64)


def build_opt(opt, ep, sp):
    """候補をプリミティブの列にする。ep/sp = (点, 車体の向きdeg, 走りの向き)。"""
    fam, m, r, w = opt
    E, hE, dE = ep; S, hS, dS = sp
    add = 0.0 if m > 0 else 180.0
    stopS = dS == 0 or dS != m
    if fam == 'run':
        thE = math.radians(hE + add); thS = math.radians(hS + add)
        B = S - TAIL_CM * tdir(thS) if stopS else S
        prims = dubins_one(E, thE, B, thS, r, w, m)
        if prims is None:
            return None
        return prims + ([('line', B, S, m)] if stopS else [])
    if fam == 'PLP':
        hl = math.degrees(math.atan2(*(S - E)[::-1])) + add
        return [('pivot', E, hE, nd(hl)), ('line', E, S, m), ('pivot', S, nd(hl), hS)]
    if fam == 'PLA':
        s1 = w; thB = math.radians(hS + add)
        B = S - TAIL_CM * tdir(thB) if stopS else S
        c2 = B + s1 * r * nL(thB); D = float(np.hypot(*(c2 - E)))
        if D <= r + 1e-9:
            return None
        phi = math.atan2(*(c2 - E)[::-1]) - math.asin(s1 * r / D)
        p2 = c2 - s1 * r * nL(phi)
        pr = [('pivot', E, hE, nd(math.degrees(phi) + add)), ('line', E, p2, m),
              arc_prim(p2, phi, s1, ccw(s1 * (thB - phi)), r, m)]
        return pr + ([('line', B, S, m)] if stopS else [])
    s1 = w; thA = math.radians(hE + add)
    c1 = E + s1 * r * nL(thA); D = float(np.hypot(*(S - c1)))
    if D <= r + 1e-9:
        return None
    phi = math.atan2(*(S - c1)[::-1]) + math.asin(s1 * r / D)
    p1 = c1 - s1 * r * nL(phi)
    return [arc_prim(E, thA, s1, ccw(s1 * (phi - thA)), r, m), ('line', p1, S, m),
            ('pivot', S, nd(math.degrees(phi) + add), hS)]


# ---------------- 中継(1と2が全部当たるときだけ) ----------------
_WPT = None
_WPP = None
_via_from, _via_to = {}, {}


def _wp():
    global _WPT, _WPP
    if _WPT is None:
        pts, poses = [], []
        for x in np.arange(BOUNDS[0], BOUNDS[1] + 0.1, WP_STEP):
            for y in np.arange(BOUNDS[2], BOUNDS[3] + 0.1, WP_STEP):
                if in_keepout(np.array([x]), np.array([y]))[0]:
                    continue
                hs = np.arange(WP_HEADINGS) * 360.0 / WP_HEADINGS
                okh = [h for h in hs if poses_ok(np.array([x]), np.array([y]), np.array([h]))]
                if okh:
                    pts.append((x, y))       # 旋回の中継の候補(必要な角度だけ回れるかは、組み立てたあとに判定する)
                poses += [(x, y, h) for h in okh]
        _WPT = np.array(pts); _WPP = np.array(poses)
    return _WPT, _WPP


def via_options(ep, sp, limit=INF):
    """中継の候補を、当たりを無視した時間の順に返す: [(cost, ('via', ...))]
    limit: これより速い候補だけ要るとき(2026-11-09)。E→中継→S の直線距離を最高の速さで走っても limit に届かない
    中継の姿勢は、Dubins を計算しない(limit より速い候補は変わらない)。6周・ガレージで Dubins が計算の約4割だった。"""
    E, hE, dE = ep; S, hS, dS = sp
    pts, poses = _wp()
    out = []
    if ALLOW_PIVOT and len(pts):
        X, Y = pts[:, 0], pts[:, 1]
        for m1 in DIRS:
            for m2 in DIRS:
                h1 = np.degrees(np.arctan2(Y - E[1], X - E[0])) + (0 if m1 > 0 else 180)
                h2 = np.degrees(np.arctan2(S[1] - Y, S[0] - X)) + (0 if m2 > 0 else 180)
                L1 = np.hypot(X - E[0], Y - E[1]); L2 = np.hypot(S[0] - X, S[1] - Y)
                pe = np.abs(nd(h1 - hE)); pw = np.abs(nd(h2 - h1)); ps = np.abs(nd(hS - h2))
                c = _tc(pe) + _tc(pw) + _tc(ps) + (L1 * spd(m1) + L2 * spd(m2)) / V_LINE + (T_STOP + PEN_STOP) * 3 \
                    + PEN_PIVOT * 3
                c = np.where((L1 < TAIL_CM) | (L2 < TAIL_CM), INF, c)
                for i in np.argsort(c)[:WP_TRIES]:
                    out.append((float(c[i]), ('viaP', m1, m2, (float(X[i]), float(Y[i])))))
    if len(poses):
        X, Y, H = poses[:, 0], poses[:, 1], poses[:, 2]
        keep = None   # None = 全部の姿勢を使い、結果を _via_from/_via_to に残す
        if limit < INF / 2:
            d = np.hypot(X - E[0], Y - E[1]) + np.hypot(X - S[0], Y - S[1])
            keep = np.flatnonzero(d / (V_LINE * max(1.0, REV_SPEED_SCALE)) < limit)
            X, Y, H = X[keep], Y[keep], H[keep]
        for m1 in (DIRS if len(X) else ()):
            for m2 in DIRS:
                cusp = m1 != m2
                add1 = 0 if m1 > 0 else 180; add2 = 0 if m2 > 0 else 180
                stopE = dE != 0 and dE != m1
                stopS = dS == 0 or dS != m2
                k1 = (round(float(E[0]), 3), round(float(E[1]), 3), round(hE, 3), m1, cusp)
                if k1 in _via_from:
                    best1 = _via_from[k1] if keep is None else _via_from[k1][keep]
                else:
                    best1 = np.full(len(X), INF)
                    t1 = np.radians(H + add1)
                    tx = X - TAIL_CM * cusp * np.cos(t1); ty = Y - TAIL_CM * cusp * np.sin(t1)
                    for r in RADII:
                        A, L = dubins_vec(np.full(len(X), E[0]), np.full(len(X), E[1]),
                                          np.full(len(X), math.radians(hE + add1)), tx, ty, t1, r)
                        best1 = np.minimum(best1, (r * A / v_arc(r) + L / V_LINE).min(0) * spd(m1))
                    if keep is None:
                        _via_from[k1] = best1
                k2 = (round(float(S[0]), 3), round(float(S[1]), 3), round(hS, 3), m2, stopS)
                if k2 in _via_to:
                    best2 = _via_to[k2] if keep is None else _via_to[k2][keep]
                else:
                    best2 = np.full(len(X), INF)
                    ts = math.radians(hS + add2)
                    bx = S[0] - TAIL_CM * stopS * math.cos(ts); by = S[1] - TAIL_CM * stopS * math.sin(ts)
                    for r in RADII:
                        A, L = dubins_vec(X, Y, np.radians(H + add2), np.full(len(X), bx), np.full(len(X), by),
                                          np.full(len(X), ts), r)
                        best2 = np.minimum(best2, (r * A / v_arc(r) + L / V_LINE).min(0) * spd(m2))
                    if keep is None:
                        _via_to[k2] = best2
                c = best1 + best2 + (TAIL_CM * cusp * spd(m1) + TAIL_CM * stopS * spd(m2)) / V_LINE \
                    + (T_STOP + PEN_STOP) * (float(stopE) + cusp + stopS)
                for i in np.argsort(c)[:WP_TRIES]:
                    if c[i] < INF / 2:
                        out.append((float(c[i]), ('viaR', m1, m2, (float(X[i]), float(Y[i]), float(H[i])))))
    out.sort(key=lambda x: x[0])
    return out


_run_cache = {}


def _best_run(A, hA, B, hB, m, tail):
    key = (round(float(A[0]), 3), round(float(A[1]), 3), round(hA, 3), round(float(B[0]), 3), round(float(B[1]), 3),
           round(hB, 3), m, tail, _EXEMPT_ON)
    if key not in _run_cache:
        _run_cache[key] = _best_run0(A, hA, B, hB, m, tail)
    return _run_cache[key]


def _best_run0(A, hA, B, hB, m, tail):
    """A→B の1本の走り(当たらない最短)。tail>0なら最後を直進tailで終える。"""
    add = 0.0 if m > 0 else 180.0
    thA, thB = math.radians(hA + add), math.radians(hB + add)
    Bm = B - tail * tdir(thB) if tail > 0 else B
    cands = []
    for r in RADII:
        for w in range(8):
            pr = dubins_one(A, thA, Bm, thB, r, w, m)
            if pr is None:
                continue
            if tail > 0:
                pr = pr + [('line', Bm, B, m)]
            cands.append((sum((prim_len(q) / (V_LINE if q[0] == 'line' else v_arc(q[2]))) for q in pr), pr))
    cands.sort(key=lambda c: c[0])
    for _, pr in cands[:12]:
        if all(prim_ok(q) for q in pr):
            return pr
    return None


def build_via(opt, ep, sp):
    E, hE, dE = ep; S, hS, dS = sp
    if opt[0] == 'viaP':
        _, m1, m2, (x, y) = opt
        Wp = np.array([x, y])
        h1 = nd(math.degrees(math.atan2(y - E[1], x - E[0])) + (0 if m1 > 0 else 180))
        h2 = nd(math.degrees(math.atan2(S[1] - y, S[0] - x)) + (0 if m2 > 0 else 180))
        return [('pivot', E, hE, h1), ('line', E, Wp, m1), ('pivot', Wp, h1, h2), ('line', Wp, S, m2), ('pivot', S, h2, hS)]
    _, m1, m2, (x, y, h) = opt
    Wp = np.array([x, y])
    stopS = dS == 0 or dS != m2
    r1 = _best_run(E, hE, Wp, h, m1, TAIL_CM if m1 != m2 else 0.0)
    if r1 is None:
        return None
    r2 = _best_run(Wp, h, S, hS, m2, TAIL_CM if stopS else 0.0)
    if r2 is None:
        return None
    return r1 + r2


# ---------------- 表(つなぎ区間ごとの候補の状態) ----------------
class Table:
    def __init__(self, ends, starts, is_start=False, via_first=False):
        self.ends, self.starts = ends, starts
        self.is_start = is_start
        self.via_first = via_first   # スタート・ゴールの区間: 組み合わせが少ないので、最初から遠回りも比べる
        nE, nS = len(ends), len(starts)
        Ea = [np.repeat(np.array(v, float), nS) for v in zip(*[(p[0][0], p[0][1], p[1], p[2]) + pivot_interval(p[0], p[1]) for p in ends])]
        Sa = [np.tile(np.array(v, float), nE) for v in zip(*[(p[0][0], p[0][1], p[1], p[2]) + pivot_interval(p[0], p[1]) for p in starts])]
        ac = approx_costs(Ea, Sa).reshape(nE, nS, len(OPTS))
        self.order = np.argsort(ac, axis=2)
        self.approx = np.take_along_axis(ac, self.order, axis=2)
        self.ptr = np.zeros((nE, nS), int)
        self.C = self.approx[:, :, 0].copy()
        self.done = np.zeros((nE, nS), bool)
        self.prims = {}
        self.via = {}
        self.checks = 0
        self.allow_via = via_first
        self.pending = set()   # 1と2が全部当たり、中継はまだ試していない区間
        self.join = set()      # (e, s): 戻らずにそのまま続ける(self.C は重なる分だけ負の時間)
        if JOIN_MAX_CM > 0.0 and not is_start and starts and ends and starts[0][2] != 0:
            ex, ey, eh, ed = [np.array(v, float)[:, None] for v in zip(*[(p[0][0], p[0][1], p[1], p[2]) for p in ends])]
            sx, sy, sh, sd = [np.array(v, float)[None, :] for v in zip(*[(p[0][0], p[0][1], p[1], p[2]) for p in starts])]
            th = np.radians(eh + np.where(ed > 0, 0.0, 180.0))
            ux, uy = np.cos(th), np.sin(th)
            along = (sx - ex) * ux + (sy - ey) * uy
            lat = -(sx - ex) * uy + (sy - ey) * ux
            mask = (ed != 0) & (ed == sd) & (np.abs(nd(eh - sh)) < 0.01) & (np.abs(lat) < 0.3) & (along <= 1e-6) & (along >= -JOIN_MAX_CM)
            for e, s in zip(*np.nonzero(mask)):
                e, s = int(e), int(s)
                self.C[e, s] = float(along[e, s]) / V_LINE * spd(ends[e][2])
                self.done[e, s] = True
                self.prims[(e, s)] = []
                self.join.add((e, s))

    def _best_via(self, e, s, ep, sp, limit):
        """遠回りの候補のうち、limit より速く当たらない最初のもの。(cost, prims) か (INF, None)。"""
        for c0, opt in via_options(ep, sp, limit)[:VIA_MAX_TRIES]:
            if c0 >= limit:
                break
            pr = build_via(opt, ep, sp)
            self.checks += 1
            if pr is not None and all(prim_ok(q) for q in pr):
                c = seq_cost(ep[2], pr, sp[2])
                if c < limit:
                    return c, pr
        return INF, None

    def open_via(self):
        """2段目: 直接つなげなかった区間に、遠回りの見込みの時間(当たりを無視)を入れて、解き直せるようにする。"""
        global _EXEMPT_ON
        self.allow_via = True
        for (pe, ps) in self.pending:
            _EXEMPT_ON = self.is_start
            try:
                if (pe, ps) not in self.via:
                    self.via[(pe, ps)] = via_options(self.ends[pe], self.starts[ps])
            finally:
                _EXEMPT_ON = False
            v = self.via[(pe, ps)]
            self.C[pe, ps] = v[0][0] if v else INF
            self.done[pe, ps] = not v
        self.pending = set()

    def resolve(self, e, s):
        """(e,s) の今の候補を厳密に判定し、当たらない最初の候補に確定する。"""
        global _EXEMPT_ON
        if self.done[e, s]:
            return False
        _EXEMPT_ON = self.is_start
        try:
            return self._resolve(e, s)
        finally:
            _EXEMPT_ON = False

    def _resolve(self, e, s):
        ep, sp = self.ends[e], self.starts[s]
        old = self.C[e, s]
        fast = hr_kernels.AVAILABLE and GATE_MODEL == 'rect'
        while self.ptr[e, s] < len(OPTS) and self.approx[e, s, self.ptr[e, s]] < INF / 2:
            if fast:
                # 2026-11-09: 確実に柱に当たる円弧-直進-円弧の候補を、形を作らずにまとめて飛ばす(通れる候補は飛ばさない)
                self.ptr[e, s] = hr_kernels.next_option(
                    float(ep[0][0]), float(ep[0][1]), float(ep[1]), float(sp[0][0]), float(sp[0][1]), float(sp[1]), int(sp[2]),
                    self.order[e, s], int(self.ptr[e, s]), self.approx[e, s], INF / 2, _OPT_FAM, _OPT_M, _OPT_R, _OPT_W,
                    float(TAIL_CM), float(PIVOT_BACK_CM), float(FRONT), float(REAR), float(HALF_WIDTH), float(MARGIN_FRONT), float(MARGIN_REAR),
                    LEG_C, LEG_U, LEG_V, LEG_HU, LEG_HV, _LEG_CORNERS4, float(_RECT_REACH))
                if not (self.ptr[e, s] < len(OPTS) and self.approx[e, s, self.ptr[e, s]] < INF / 2):
                    break
            opt = OPTS[self.order[e, s, self.ptr[e, s]]]
            pr = build_opt(opt, ep, sp)
            self.checks += 1
            if pr is not None and all(prim_ok(q) for q in pr):
                c = seq_cost(ep[2], pr, sp[2])
                if c < INF / 2:
                    if self.via_first:
                        cv, pv = self._best_via(e, s, ep, sp, c)
                        if pv is not None:
                            c, pr = cv, pv
                    self.C[e, s] = c; self.done[e, s] = True; self.prims[(e, s)] = pr
                    return abs(c - old) > 1e-9
            self.ptr[e, s] += 1
        # 中継(1段目は使わない。経路が見つからないときだけ2段目で使う)
        if not self.allow_via:
            self.C[e, s] = INF; self.done[e, s] = True; self.pending.add((e, s))
            return True
        if (e, s) not in self.via:
            self.via[(e, s)] = via_options(ep, sp)
        for c0, opt in self.via[(e, s)][:VIA_MAX_TRIES]:
            pr = build_via(opt, ep, sp)
            self.checks += 1
            if pr is not None and all(prim_ok(q) for q in pr):
                c = seq_cost(ep[2], pr, sp[2])
                if c < INF / 2:
                    self.C[e, s] = c; self.done[e, s] = True; self.prims[(e, s)] = pr
                    return True
        self.C[e, s] = INF; self.done[e, s] = True
        return True


def arm_must_be_up(prims):
    """走り出しから順に prims(つなぎ・通過の順)をたどり、アームを下げたままの部分(車体の前の端から ARM_FRONT_CM、
    横幅 ±ARM_HALF_WIDTH_CM)と脚のすき間が下限(MARGIN_FRONT)を切る最初の点を探す。
    戻り値: dict(cm=そこまでの走った距離, sec=そこまでの見込みの時間) か、最後まで当たらなければ None。
    時間は直進・円弧の速さとその場旋回の時間だけで数える(止まる時間を入れないので、実際より短め=安全側)。"""
    dist = 0.0; tsec = 0.0
    x0, x1 = FRONT, FRONT + ARM_FRONT_CM
    for q in prims:
        if q[0] == 'line':
            L = prim_len(q); n = max(2, int(L / 0.5) + 1); k = np.linspace(0, 1, n)
            h = math.atan2(*(q[2] - q[1])[::-1]) + (0 if q[3] > 0 else math.pi)
            px = q[1][0] + (q[2][0] - q[1][0]) * k; py = q[1][1] + (q[2][1] - q[1][1]) * k; hh = np.full(n, h)
            ds = L * k; dt = L / V_LINE * spd(q[3]) * k; seg_d, seg_t = L, L / V_LINE * spd(q[3])
        elif q[0] == 'arc':
            px, py, hd = arc_samples(q, 0.5); hh = np.radians(hd)
            L = prim_len(q); k = np.linspace(0, 1, len(px))
            ds = L * k; dt = L / v_arc(q[2]) * spd(q[6]) * k; seg_d, seg_t = L, L / v_arc(q[2]) * spd(q[6])
        else:
            dd = nd(q[3] - q[2])
            if abs(dd) < 0.3:
                continue
            hd = q[2] + dd * np.linspace(0, 1, int(abs(dd)) + 2)
            px, py = _pivot_axle(q[1], q[2], hd); hh = np.radians(hd)
            ds = np.zeros(len(px)); dt = turn_cost(abs(dd)) * np.linspace(0, 1, len(px)); seg_d, seg_t = 0.0, turn_cost(abs(dd))
        c = _rect_clear(np.asarray(px, float), np.asarray(py, float), np.asarray(hh, float), x0, x1, hw=ARM_HALF_WIDTH_CM)
        bad = np.flatnonzero(c < MARGIN_FRONT)
        if len(bad):
            i = int(bad[0])
            return dict(cm=round(float(dist + ds[i]), 1), sec=round(float(tsec + dt[i]), 2))
        dist += seg_d; tsec += seg_t
    return None


def route_audit(conn_prims, visits, used):
    """できた経路全体を細かく(直進0.5cm・円弧と旋回1°)調べ、柱までの一番小さいすき間と、進入禁止へのはみ出しを出す。"""
    allp = []
    for i, (key, e, s) in enumerate(used):
        allp.append(('conn', key))
        if i < len(visits):
            allp.append(('visit', visits[i]))
    xs, ys, hs = [], [], []
    def add_prims(prims):
        for q in prims:
            if q[0] == 'line':
                L = prim_len(q); k = np.linspace(0, 1, max(2, int(L / 0.5) + 1))
                h = math.degrees(math.atan2(*(q[2] - q[1])[::-1])) + (0 if q[3] > 0 else 180)
                xs.append(q[1][0] + (q[2][0] - q[1][0]) * k); ys.append(q[1][1] + (q[2][1] - q[1][1]) * k); hs.append(np.full(len(k), h))
            elif q[0] == 'arc':
                ax, ay, ah = arc_samples(q, 0.5); xs.append(ax); ys.append(ay); hs.append(ah)
            else:
                dd = nd(q[3] - q[2])
                if abs(dd) >= 0.3:
                    hh = q[2] + dd * np.linspace(0, 1, int(abs(dd)) + 2)
                    ax, ay = _pivot_axle(q[1], q[2], hh); xs.append(ax); ys.append(ay); hs.append(hh)
    for i, (key, e, s) in enumerate(used):
        add_prims(_USED_PRIMS[(key, e, s)])
        if i < len(visits):
            add_prims(visits[i]['prims'])
    px, py, hh = np.concatenate(xs), np.concatenate(ys), np.concatenate(hs)
    h = np.radians(hh)
    if GATE_MODEL == 'rect':
        cf = _rect_clear(px, py, h, 0.0, FRONT); cr = _rect_clear(px, py, h, -REAR, 0.0)
    else:
        c = np.cos(h)[:, None]; sn = np.sin(h)[:, None]
        dx = CAP[None, :, 0] - px[:, None]; dy = CAP[None, :, 1] - py[:, None]
        lx = dx * c + dy * sn; ly = -dx * sn + dy * c
        ey = np.maximum(np.abs(ly) - HALF_WIDTH, 0.0)
        cf = (np.hypot(np.maximum(np.maximum(-lx, lx - FRONT), 0.0), ey) - POST_R).min(1)
        cr = (np.hypot(np.maximum(np.maximum(-REAR - lx, lx), 0.0), ey) - POST_R).min(1)
    ko = sum(1 for i in range(0, len(px), 4) if body_keepout(px[i:i + 1], py[i:i + 1], hh[i:i + 1]))
    i = int(np.argmin(np.minimum(cf, cr)))
    return dict(min_front=float(cf.min()), min_rear=float(cr.min()), keepout_hits=ko,
                worst_pose=[float(px[i]), float(py[i]), float(hh[i])])


_USED_PRIMS = {}


def solve_same_laps(T, vs, ends):
    """2周目〜最後の1つ前の周を同じ通り方にする最短。赤の入りの姿勢 s_r ごとに「s_r から1周して s_r に戻る時間」と
    「s_r から最後の周を走ってゴールする時間」を出し、1周目で s_r に着く時間と合わせて一番短い s_r を選ぶ。
    戻り値: (合計, used, picks)。経路がなければ (INF, None, None)。"""
    def layer(prev, tab, color):
        if hr_kernels.AVAILABLE:
            s_arr, e_arr, c_arr = vs[color]
            return hr_kernels.layer(prev, tab.C, s_arr, e_arr, c_arr, len(ends[color]))
        M = prev[:, None] + tab.C
        bs = M.min(0); be = M.argmin(0)
        s_arr, e_arr, c_arr = vs[color]
        tot = bs[s_arr] + c_arr
        cur = np.full(len(ends[color]), INF); arg = np.full(len(ends[color]), -1)
        order = np.argsort(tot)
        ue, first = np.unique(e_arr[order], return_index=True)
        win = order[first]
        cur[ue] = tot[win]; arg[ue] = win
        return cur, be, arg
    p0, be_r, arg_r = layer(np.zeros(1), T[('START', 'red')], 'red')
    pre = None
    if ('START', 'yellow') in T:
        y0, be_y0, arg_y0 = layer(np.zeros(1), T[('START', 'yellow')], 'yellow')
        p0b, be_r2, arg_r2 = layer(y0, T[('yellow', 'red')], 'red')
        pre = p0b < p0
        p0 = np.where(pre, p0b, p0)
    p1, be_b, arg_b = layer(p0, T[('red', 'blue')], 'blue')
    p2, be_y, arg_y = layer(p1, T[('blue', 'yellow')], 'yellow')
    Tyr = T[('yellow', 'red')].C
    M = p2[:, None] + Tyr
    P = M.min(0); P_e = M.argmin(0)
    srs = np.flatnonzero(P < INF / 2)
    if not len(srs):
        return INF, None, None
    # 2周目以降: s_r ごとに同時に計算する
    kmap = np.full(Tyr.shape[1], -1); kmap[srs] = np.arange(len(srs))
    s_arr, e_arr, c_arr = vs['red']
    cur = np.full((len(srs), len(ends['red'])), INF)
    keep = kmap[s_arr] >= 0
    np.minimum.at(cur, (kmap[s_arr[keep]], e_arr[keep]), c_arr[keep])
    for a, b in (('red', 'blue'), ('blue', 'yellow')):
        s_arr, e_arr, c_arr = vs[b]
        if hr_kernels.AVAILABLE:
            bs = hr_kernels.min_over_e(cur, T[(a, b)].C)
            cur = np.minimum(hr_kernels.visit_min(bs, s_arr, e_arr, c_arr, len(ends[b])), INF)
            continue
        bs = (cur[:, :, None] + T[(a, b)].C).min(1)
        tot = bs[:, s_arr] + c_arr
        order = np.argsort(e_arr, kind='stable')
        ue, first = np.unique(e_arr[order], return_index=True)
        cur = np.full((len(srs), len(ends[b])), INF)
        cur[:, ue] = np.minimum.reduceat(tot[:, order], first, axis=1)
    cyc = (cur + Tyr[:, srs].T).min(1)
    goal = T[('yellow', 'GOAL')].C.min(1)
    suf = (cur + goal[None, :]).min(1)
    score = P[srs] + (LAPS - 2) * cyc + suf
    k = int(np.argmin(score)); total = float(score[k])
    if total >= INF / 2:
        return INF, None, None
    sr = int(srs[k])
    # 復元: s_r から赤→青→黄を走る
    s_arr, e_arr, c_arr = vs['red']
    tot = np.where(s_arr == sr, c_arr, INF)
    order = np.argsort(tot)
    ue, first = np.unique(e_arr[order], return_index=True)
    cur1 = np.full(len(ends['red']), INF); arg1 = np.full(len(ends['red']), -1)
    cur1[ue] = tot[order[first]]; arg1[ue] = order[first]
    q1, be1, arg2 = layer(cur1, T[('red', 'blue')], 'blue')
    q2, be2, arg3 = layer(q1, T[('blue', 'yellow')], 'yellow')

    def trace(e_y):
        vy = int(arg3[e_y]); sy = int(vs['yellow'][0][vy]); eb = int(be2[sy])
        vb = int(arg2[eb]); sb = int(vs['blue'][0][vb]); er = int(be1[sb])
        vr = int(arg1[er])
        return vr, er, vb, sb, eb, vy, sy, e_y
    e_c = int(np.argmin(q2 + Tyr[:, sr])); e_g = int(np.argmin(q2 + goal))
    # 1周目
    e_y1 = int(P_e[sr]); vy = int(arg_y[e_y1]); sy = int(vs['yellow'][0][vy]); eb = int(be_y[sy])
    vb = int(arg_b[eb]); sb = int(vs['blue'][0][vb]); er = int(be_b[sb])
    if pre is not None and pre[er]:
        # スタート→黄→赤
        vr = int(arg_r2[er]); s0 = int(vs['red'][0][vr]); ey0 = int(be_r2[s0]); vy0 = int(arg_y0[ey0]); sy0 = int(vs['yellow'][0][vy0])
        used = [(('START', 'yellow'), 0, sy0), (('yellow', 'red'), ey0, s0)]
        picks = [('yellow', vy0)]
    else:
        vr = int(arg_r[er]); s0 = int(vs['red'][0][vr])
        used = [(('START', 'red'), 0, s0)]
        picks = []
    used += [(('red', 'blue'), er, sb), (('blue', 'yellow'), eb, sy), (('yellow', 'red'), e_y1, sr)]
    picks += [('red', vr), ('blue', vb), ('yellow', vy)]
    for lap in range(LAPS - 1):
        last = lap == LAPS - 2
        vr, er, vb, sb, eb, vy, sy, ey = trace(e_g if last else e_c)
        used += [(('red', 'blue'), er, sb), (('blue', 'yellow'), eb, sy)]
        used.append((('yellow', 'GOAL'), ey, int(np.argmin(T[('yellow', 'GOAL')].C[ey]))) if last else (('yellow', 'red'), ey, sr))
        picks += [('red', vr), ('blue', vb), ('yellow', vy)]
    return total, used, picks


def main():
    t0 = time.time()
    global MARGIN_FRONT, MARGIN_REAR, OBLIQUE_POLICY
    used_margin = None
    for i_mg, mg in enumerate(MARGIN_LIST or [None]):
        if i_mg == 1 and NARROW_SAFEST:
            # 狭い配置(最初のすき間で通れない): 垂直に通れないゲートだけ斜めを使う(計算量を約1/8にする。ユーザー決定 2026-11-09)
            OBLIQUE_POLICY = 'safest'
        if mg is not None:
            MARGIN_FRONT = MARGIN_REAR = float(mg)
            used_margin = float(mg)
            for memo in (_line_memo, _arc_memo, _pivot_cache, _piv_int, _run_cache, _arc_limit_memo):
                memo.clear()
        cands = {c: visit_candidates(g) for c, g in GATES.items()}
        if all(cands.values()):
            break
    if used_margin is not None:
        print('margin selected: %.1f' % used_margin, flush=True)
    if CANDIDATES_ONLY:
        json.dump(dict(margin=used_margin, counts={c: len(v) for c, v in cands.items()}), open(sys.argv[2], 'w'))
        return
    if VISIT_FILTER:
        for c in cands:
            allow_v = {tuple(x) for x in VISIT_FILTER.get(c, [])}
            cands[c] = [v for v in cands[c] if (v['sign'], v['mode'], v['angle'], v['angle_out']) in allow_v]
    print('candidates', {c: len(v) for c, v in cands.items()}, f'{time.time() - t0:.1f}s', flush=True)
    for oc in OBLIQUE_CHOICE:
        print('  oblique', oc, flush=True)
    for c in cands:
        if not cands[c]:
            print(f'NO ROUTE: no way through {c}'); return
    ends, starts, eidx, sidx = {}, {}, {}, {}
    for c in cands:
        ends[c], starts[c], eidx[c], sidx[c] = [], [], {}, {}
        for v in cands[c]:
            for pose, lst, idx, key in ((v['start'], starts[c], sidx[c], 's'), (v['end'], ends[c], eidx[c], 'e')):
                k = pose_key(pose)
                if k not in idx:
                    idx[k] = len(lst); lst.append(pose)
                v[key] = idx[k]
    start_pose = [(START, START_H, 0)]
    goal_pose = [(GOAL, float(h), 0) for h in GOAL_HEADINGS] if GOAL_HEADINGS else [(GOAL, GOAL_H, 0)]
    if GARAGE_BOX is not None and GARAGE_RULE == 'overlap':
        goal_pose = [(g, h, 0) for h in (GOAL_HEADINGS or [0.0, 90.0, 180.0, -90.0]) for g in garage_overlap_goals(float(h))]
        goal_pose = [(g, float(h), z) for g, h, z in goal_pose]
        print('garage (overlap >= %.1fcm) box x %.1f..%.1f y %.1f..%.1f, %d goal candidates' % (
            (GARAGE_OVERLAP_CM,) + GARAGE_BOX + (len(goal_pose),)), flush=True)
    elif GARAGE_BOX is not None:
        goal_pose = [(garage_goal(float(h)), float(h), 0) for h in (GOAL_HEADINGS or [GOAL_H])
                     if garage_goal(float(h)) is not None]
        if not goal_pose:
            raise SystemExit('garage: the body does not fit in the garage area in any of goal_headings')
        print('garage box x %.1f..%.1f y %.1f..%.1f, goals %s' % (GARAGE_BOX + (
            [(round(float(g[0][0]), 1), round(float(g[0][1]), 1), g[1]) for g in goal_pose],)), flush=True)
    T = {('START', 'red'): Table(start_pose, starts['red'], is_start=True, via_first=True),
         ('red', 'blue'): Table(ends['red'], starts['blue']),
         ('blue', 'yellow'): Table(ends['blue'], starts['yellow']),
         ('yellow', 'red'): Table(ends['yellow'], starts['red']),
         ('yellow', 'GOAL'): Table(ends['yellow'], goal_pose, via_first=True)}
    if START_PRELUDE:
        T[('START', 'yellow')] = Table(start_pose, starts['yellow'], is_start=True, via_first=True)
    print('tables', {k: v.C.shape for k, v in T.items()}, f'{time.time() - t0:.1f}s', flush=True)
    seq = ['red', 'blue', 'yellow'] * LAPS
    vs = {c: (np.array([v['s'] for v in cands[c]]), np.array([v['e'] for v in cands[c]]),
              np.array([v['cost'] for v in cands[c]])) for c in cands}
    it = 0
    phase = 1
    # 2026-11-09: 狭い配置(最初のすき間では通れなかった配置)は、遠回りなしでは経路がないことが多いので、最初から遠回りも使う
    if MARGIN_LIST and used_margin is not None and used_margin < MARGIN_LIST[0]:
        phase = 2
        for t in T.values():
            t.allow_via = True
    saved = None; t_phase2 = None
    while True:
        it += 1
        if saved is not None and time.time() - t_phase2 > VIA_BUDGET_SEC:
            total, used, visits, prims = saved
            for k in T:
                T[k].prims = prims[k]
            print(f'phase 2 over budget ({VIA_BUDGET_SEC:.0f}s): keep phase 1 route', flush=True)
            break
        prev = np.zeros(1); prev_c = 'START'; back = []
        used = None
        if SAME_LAPS and LAPS >= 3:
            total, used, picks = solve_same_laps(T, vs, ends)
            if used is not None:
                visits = [cands[c][v] for c, v in picks]
        for color in (() if SAME_LAPS and LAPS >= 3 else seq):
            tab = T[(prev_c, color)]
            M = prev[:, None] + tab.C
            bs = M.min(0); be = M.argmin(0)
            s_arr, e_arr, c_arr = vs[color]
            tot = bs[s_arr] + c_arr
            # 抜けの姿勢ごとに、時間が一番短い通り方を選ぶ(同じ時間なら argsort で先に来る方。1つずつ上書きしていた以前と同じ結果)
            cur = np.full(len(ends[color]), INF); arg = np.full(len(ends[color]), -1)
            order = np.argsort(tot)
            ue, first = np.unique(e_arr[order], return_index=True)
            win = order[first]
            cur[ue] = tot[win]; arg[ue] = win
            back.append((prev_c, color, be, arg))
            prev, prev_c = cur, color
        if not (SAME_LAPS and LAPS >= 3):
            tab = T[(prev_c, 'GOAL')]
            M = prev + tab.C.min(1)
            e_last = int(np.argmin(M)); total = float(M[e_last])
        if total >= INF / 2:
            bad = next((f'{a}->{b}' for a, b, _, arg in back if np.all(arg < 0)), 'goal')
            if phase == 1 and any(t.pending for t in T.values()):
                # 2段目: 中継を使えば通れるかもしれない区間を、当たりを無視した時間に戻して解き直す
                phase = 2
                for t in T.values():
                    t.open_via()
                print(f'phase 2 (detours)  {time.time() - t0:.1f}s', flush=True)
                continue
            print(f'NO ROUTE (stopped at {bad})  checks={sum(t.checks for t in T.values())} {time.time() - t0:.1f}s')
            return
        # 経路を復元して、使ったつなぎ区間を厳密に判定する
        if used is None:
            used = [(('yellow', 'GOAL'), e_last, int(np.argmin(tab.C[e_last])))]
            visits = []
            e = e_last
            for prev_c, color, be, arg in reversed(back):
                vi = arg[e]; v = cands[color][vi]
                visits.append(v)
                s = v['s']; ep = int(be[s])
                used.append(((prev_c, color), ep, s))
                e = ep
            used.reverse(); visits.reverse()
        changed = False
        for key, e, s in used:
            changed |= T[key].resolve(e, s)
        if not changed:
            if phase == 1 and VIA_ALWAYS and any(t.pending for t in T.values()):
                # 遠回りなしで経路が見つかっても、直接つなげなかった区間を遠回りで試し、速くなるなら採る
                saved = (total, list(used), list(visits), {k: dict(T[k].prims) for k in T})
                t_phase2 = time.time()
                phase = 2
                for t in T.values():
                    t.open_via()
                print(f'phase 1 {total:.1f}s -> phase 2 (detours)  {time.time() - t0:.1f}s', flush=True)
                continue
            break
    print(f'TOTAL hybrid laps={LAPS}: {total:.1f}s (cost)  iterations={it} checks={sum(t.checks for t in T.values())} '
          f'compute {time.time() - t0:.1f}s', flush=True)

    def ser(p):
        if p[0] == 'line':
            return dict(t='line', a=p[1].tolist(), b=p[2].tolist(), m=p[3])
        if p[0] == 'arc':
            xs, ys, _ = arc_samples(p, 2.0)
            return dict(t='arc', pts=[[float(a), float(b)] for a, b in zip(xs, ys)], m=p[6], r=p[2],
                        deg=math.degrees(p[5]), th0=math.degrees(p[3]), s=p[4])
        return dict(t='pivot', p=[float(p[1][0]), float(p[1][1])], deg=abs(nd(p[3] - p[2])), h_from=p[2], h_to=p[3])

    # そのまま続ける区間: 次のゲートの最初の直進を、前のゲートの抜けの点から始める(重なる分を二重に走らない)
    visits = list(visits)
    for i, (key, e, s) in enumerate(used):
        if (e, s) in T[key].join and i < len(visits):
            q = visits[i]['prims'][0]
            visits[i] = dict(visits[i], prims=[('line', T[key].ends[e][0], q[2], q[3])] + list(visits[i]['prims'][1:]))
    blocks = []
    for i, (key, e, s) in enumerate(used):
        blocks.append(dict(kind='conn', lap=None, color=None, prims=[ser(p) for p in T[key].prims[(e, s)]]))
        if i < len(visits):
            v = visits[i]
            blocks.append(dict(kind='visit', lap=(i - len(visits) % 3) // 3 + 1, color=v['color'], mode=v['mode'], sign=v['sign'], off=v['off'], angle=v['angle'], angle_out=v['angle_out'],
                               prims=[ser(p) for p in v['prims']]))
    for i, v in enumerate(visits[:3]):
        print(f'  lap1 {v["color"]:6s} {v["mode"]:2s} sign={v["sign"]:+d} off={v["off"]:+.1f} angle={v["angle"]:+.0f}/{v["angle_out"]:+.0f} ea={v["ea"]} ee={v["ee"]}')
    for key, e, s in used:
        _USED_PRIMS[(key, e, s)] = T[key].prims[(e, s)]
    audit = route_audit([p for key, e, s in used for p in T[key].prims[(e, s)]], visits, used)
    print('audit: min clearance front %.2fcm rear %.2fcm (needed %.1f / %.1f), keepout hits %d' % (
        audit['min_front'], audit['min_rear'], MARGIN_FRONT, MARGIN_REAR, audit['keepout_hits']), flush=True)
    seq = []
    for i, (key, e, s) in enumerate(used):
        seq += list(T[key].prims[(e, s)])
        if i < len(visits):
            seq += list(visits[i]['prims'])
    arm = arm_must_be_up(seq)
    print('arm: must be up by %s (arm front %.1fcm, half width %.1fcm)' % (
        'never' if arm is None else '%.1fcm / %.2fs' % (arm['cm'], arm['sec']), ARM_FRONT_CM, ARM_HALF_WIDTH_CM), flush=True)
    json.dump(dict(total=total, params=PARAMS, margin=MARGIN_FRONT, narrow=bool(MARGIN_LIST and used_margin is not None and used_margin < MARGIN_LIST[0]), audit=audit, arm=arm, blocks=blocks), open(sys.argv[2], 'w'))


if __name__ == '__main__':
    main()
