"""ETラリー経路計画 設定パラメータ。

このファイルの定数は、現行の経路計算(rule_route.py)と、現在は本番では
使われていない旧方式(planner.pyのbuild_graph/dijkstra_with_turns、
詳細はplanner.pyの先頭コメント参照)の両方から参照される。
GATE_CROSS_SAMPLES/OBSTACLE_BOUNDARY_SAMPLES/GATE_CROSSING_ANGLES_DEG/
OPEN_SPACE_GRID_SPACING_CM/OPEN_SPACE_GRID_MARGIN_CM/
POST_AVOID_RADIUS_TRANSLATION_CMは旧方式(build_graph)専用で、
rule_route.py側では使われていない。それ以外(コース・ロボットの物理定数、
START/GOAL、GATE_ORDER/LAPS、TURN_COST_PER_DEGREE_CM、
POST_AVOID_RADIUS_CM、SAFETY_MARGIN_CMなど)は両方から使われる共通部品。
"""

import math

# --- コース物理定数 ---
# 実機コースを実測した値に更新(2026-09時点)。
#   グレー●の直径: 4.4cm
#   グレー●4個ぶんを含む1辺の長さ: 102.8cm(布のコースのため102.5~102.9でぶれる)
#   → グリッド間隔は「1辺÷4」で逆算: 102.8 / 4 = 25.7cm
# 別途「グレーの端から隣のグレーの端(点+間+点)=28.9cm」からも逆算できる
# (28.9 - 直径4.4×2 = 20.1cmが点と点の隙間、+直径4.4で中心間隔24.5cmとなり、
# 上のグリッド間隔25.7cmと少しずれる)。布のコースなので多少の誤差が
# あるとのことなので、より直接的な「1辺の長さ」から逆算した25.7cmを採用。
GRID_PITCH_CM = 25.7          # グレー●同士の間隔(中心間)
GATE_INNER_WIDTH_CM = 22.0    # ゲート脚の内側間隔(実際に通過できる幅、現在は未使用)
POST_DIAMETER_CM = 2.0        # ゲート脚の太さ(未計測のため暫定値のまま)
POST_RADIUS_CM = POST_DIAMETER_CM / 2

# --- ロボット仕様(実測、2026-09時点) ---
ROBOT_LENGTH_CM = 25.0
ROBOT_WIDTH_CM = 13.7
ROBOT_HALF_WIDTH_CM = ROBOT_WIDTH_CM / 2

# タイヤ(=旋回軸)は前端から7.5cmの位置にある。つまり旋回中心は車体の
# 幾何中心ではなくタイヤ位置であり、前後で旋回半径が非対称になる
# (前:7.5cm、後ろ: 全長-7.5cm=18cm。後ろの方がずっと大きく振れる)。
ROBOT_FRONT_OVERHANG_CM = 7.5
ROBOT_REAR_OVERHANG_CM = ROBOT_LENGTH_CM - ROBOT_FRONT_OVERHANG_CM

# その場旋回したとき、車体の最も遠い角(後ろの角)がタイヤ位置を中心に
# 描く円の半径。旋回が起きる地点では、この半径ぶんの空間が必要になる。
ROBOT_PIVOT_SWEEP_RADIUS_CM = math.hypot(ROBOT_REAR_OVERHANG_CM, ROBOT_HALF_WIDTH_CM)

# --- 安全マージン(暫定・調整可能) ---
# 実機テストでゲート退出直後の直進区間(ゲート自身の支柱のすぐそば)で
# 実測約2cmの接触が発生したため、1.0cm -> 3.0cmに引き上げていた
# (2cmの実測誤差 + 念のための余裕1cm)。支柱の実際の太さ・車体の実測幅が
# 判明したら、そちらもconfig側を更新した上で、このマージンが適切かは
# 再度実機で確認すること。
#
# 実験用に1.5cmへ下げてみている(3.0cmだと安全ではあるが大回りが大きすぎる
# ため)。3.0cmの根拠だった「実測2cmの接触」をそのままカバーできる値
# ではないので、実機で試すなら再度接触リスクがある前提で確認すること。
SAFETY_MARGIN_CM = 1.5

# 障害物(ゲート脚)を回避する際の実効半径。用途によって2種類使い分ける。
#
# 1) POST_AVOID_RADIUS_TRANSLATION_CM: 純粋な直進(旋回を伴わない移動)の
#    間だけ必要なクリアランス。車体半幅ベース。
# 2) POST_AVOID_RADIUS_CM: 経路上のどの中継点でも旋回が起きる可能性がある
#    (実際に旋回するかは経路全体を計算してみないと分からない)ため、
#    経路探索グラフの構築には安全側でこちら(旋回スイープ半径ベース)を使う。
#    ゲート通過そのもの(entry->exitの直進区間)だけは旋回を伴わないと
#    分かっているので、そこだけ(1)を使う。
POST_AVOID_RADIUS_TRANSLATION_CM = ROBOT_HALF_WIDTH_CM + POST_RADIUS_CM + SAFETY_MARGIN_CM
POST_AVOID_RADIUS_CM = ROBOT_PIVOT_SWEEP_RADIUS_CM + POST_RADIUS_CM + SAFETY_MARGIN_CM

# --- スタート/ゴール(cm, グローバル座標。原点(0,0)はグレー●の左下) ---
# 角度は数学の標準系(反時計回りが正、+x方向を0度)で内部管理する。
#
# スタート/ゴールの基準グリッド辺((4,3)-(4,4)、(0,3)-(0,4))は、座標を
# 決めるための数式上の便宜(ゲートのentry計算式を流用しているだけ)であり、
# 実際にそこに支柱があるわけではない。ただしランダムなゲート配置では、
# 本物のゲートが偶然同じグリッド辺に生成されることがあり、その場合に
# 支柱への距離が近くなりすぎる(以前のROBOT_FRONT_OVERHANG_CM=7.5cmだと
# 実際に支柱に7.5cmまで近づくケースが見つかった)。そのため、ゲートの
# entry計算とは切り離した、スタート/ゴール専用のオフセット量を使う。
# スタートとゴールは元は同じ値を共用していたが、2026-09-06に実機で
# スタート地点(旋回中心/タイヤ位置)までの距離を実測したところ16.5cmだった
# ため、スタート側だけ実測値に更新し、ゴール側は据え置き(未実測)とした。
START_OFFSET_CM = 16.5
GOAL_OFFSET_CM = 13.0

# スタート: グリッド(4,3)-(4,4)を結ぶ線の中心点から、その線と垂直に
# START_OFFSET_CMだけ離れた点。
START_POS_CM = (4 * GRID_PITCH_CM + START_OFFSET_CM, 3.5 * GRID_PITCH_CM)
START_HEADING_DEG = 180.0   # -x方向を向く

# ゴール: スタートと同じ考え方で、グリッド(0,3)-(0,4)を結ぶ線の中心点から
# 垂直にGOAL_OFFSET_CMだけ離れた点。
GOAL_POS_CM = (0 * GRID_PITCH_CM - GOAL_OFFSET_CM, 3.5 * GRID_PITCH_CM)
GOAL_HEADING_DEG = 90.0     # +y方向を向く

# 「支柱のすぐ周りだけ」の迂回ノードだと、複数の障害物を大きく回り込んで
# 避けた方が良いケース(距離は伸びても旋回が大きく減る)を探索できないため、
# 開けた空間にも粗いグリッド状の中継点を追加する。範囲はコース+スタート/ゴール
# 周辺を十分覆うよう、コースの外側にも余裕を持たせる。
OPEN_SPACE_GRID_SPACING_CM = 20.0
OPEN_SPACE_GRID_MARGIN_CM = 40.0

# --- 探索パラメータ(精度と計算コストのトレードオフ) ---
# 旋回コストを考慮したDijkstra(状態=有向エッジ)に切り替えたためグラフが
# 大きくなる。計算時間を抑えるため、フェーズ1の垂直通過限定の頃より
# サンプル数を減らしている。
GATE_CROSS_SAMPLES = 7        # 各ゲート通過区間内の候補点の数
OBSTACLE_BOUNDARY_SAMPLES = 10  # 障害物円1本あたりの回避用サンプル点数

# ゲート通過の候補角度(ゲートの法線方向を0度とした、通過方向の振り幅)。
# 通過時の左右クリアランスは車体後端の張り出し(ROBOT_REAR_OVERHANG_CM=18cm)
# が支配的になるため、斜め通過できる余地はかなり狭い(数学的には±10.7度
# 程度が限界)。ここでは安全側に絞った角度候補のみを試す。
GATE_CROSSING_ANGLES_DEG = [-10.0, -5.0, 0.0, 5.0, 10.0]

# 旋回1度あたりのコストを、距離(cm)に換算していくらとみなすか。
# 実機の直進/旋回速度が未計測のため暫定値。値を大きくするほど、
# 経路は「多少遠回りしてでも旋回を減らす」方向に寄る。
TURN_COST_PER_DEGREE_CM = 1.0

# --- 周回・ゲート順序 ---
GATE_ORDER = ["red", "blue", "yellow"]
LAPS = 3

# --- turn()コマンドの符号規約 ---
# 右旋回(時計回り)を正、左旋回(反時計回り)を負とする(暫定)
