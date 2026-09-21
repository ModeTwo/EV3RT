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
#   グレー●5個ぶんの1辺の長さ: 102.8cm(布のコースのため102.5~102.9でぶれる、
#   測定位置は円の中心ではなく外側の端から端)
#   → 中心間の間隔は、端から端の102.8cmから直径ぶん(4.4cm)を引いてから
#   4間隔で割る必要がある: (102.8 - 4.4) / 4 = 24.6cm
# 別途「グレーの端から隣のグレーの端(点+間+点)=28.9cm」からも逆算できる
# (28.9 - 直径4.4×2 = 20.1cmが点と点の隙間、+直径4.4で中心間隔24.5cmとなり、
# 上の24.6cmとほぼ一致、布のブレの範囲内)。
#
# 2026-09-12〜15、上の「1辺の長さ102.8cm」を中心間の距離として誤って
# 102.8/4=25.7cmを採用していたことが判明(2026-09-15、実測値の定義を
# 再確認して発覚)。25.7cmは正しい24.6cm前後より約1.1cm(4.5%)大きく、
# 原点(0,0)から離れたゲートほど計算上の位置ズレが比例して大きくなる形で
# 実機の誤差(特に青・黄色ゲートの周回ごとの位置ズレ)に効いていたと考えられる。
GRID_PITCH_CM = 24.6          # グレー●同士の間隔(中心間)
# GATE_INNER_WIDTH_CM: 2026-09-15に実測(22.2cm)。rule_route.pyの安全性判定
# (POST_AVOID_RADIUS_CM等)には使われておらず、generate_diagram.pyが描く
# 「有効通過区間」の可視化線(Gate.valid_half_length()経由)にのみ使用。
GATE_INNER_WIDTH_CM = 22.2
# POST_DIAMETER_CM: 支柱本体の実測値は2.0cm(以前の暫定値と一致、2026-09-15確認)。
POST_DIAMETER_CM = 2.0
POST_RADIUS_CM = POST_DIAMETER_CM / 2
# POST_PIVOT_ARM_HALF_LENGTH_CM: 支柱の足元のT字パーツ(直径2cmの棒)を
# 旋回安全性の判定にだけ反映するための値。T字パーツは支柱の中心から見て
# ゲートの法線方向(侵入・退出方向)の両側に4.0cmずつ伸びている(支柱自身の
# 半径1.0cmぶんを除くと突き出しは片側3.0cm)。旋回安全性判定
# (planner.pyのpivot_turn_safe)側で「支柱の中心を通り、ゲート法線方向に
# 伸びる線分(半径はPOST_RADIUS_CMのまま)」との距離を見るカプセル型の
# チェックとして使う(Gate.posts()がPost(arm_dir=法線ベクトル)を返す)。
POST_PIVOT_ARM_HALF_LENGTH_CM = 4.0

# --- ロボット仕様(実測、2026-09時点) ---
ROBOT_LENGTH_CM = 25.0
ROBOT_WIDTH_CM = 13.7
ROBOT_HALF_WIDTH_CM = ROBOT_WIDTH_CM / 2

# タイヤ(=旋回軸)は前端から7.5cmの位置にある。つまり旋回中心は車体の
# 幾何中心ではなくタイヤ位置であり、前後で旋回半径が非対称になる
# (前:7.5cm、後ろ: 全長-7.5cm=18cm。後ろの方がずっと大きく振れる)。
ROBOT_FRONT_OVERHANG_CM = 7.5
ROBOT_REAR_OVERHANG_CM = ROBOT_LENGTH_CM - ROBOT_FRONT_OVERHANG_CM

# GATE_EXIT_OFFSET_CM: gate_entry_exit()のexit点(ゲート中心からの距離)専用の
# オフセット。支柱のT字パーツをpivot_turn_safeのカプセル判定に組み込むと、
# 支柱を点として扱う旧モデルで確保できていた安全マージンの多くが、T字パーツの
# 実際の到達範囲(支柱中心から±POST_PIVOT_ARM_HALF_LENGTH_CM)によって
# 食い潰されてしまう。ROBOT_REAR_OVERHANG_CM自体はロボットの実測寸法
# (旋回半径計算などにも使われる)であり、安全マージン調整のために書き換える
# べきではないため、gate_entry_exit()専用の別定数として用意する。
GATE_EXIT_OFFSET_CM = 22.0

# POST_ARM_STRAIGHT_MARGIN_CM: 直進区間の安全性チェックにT字パーツを
# 組み込んだ際(rule_route._segment_post_distance参照)、ゲートは人の手で
# 設置するため実際の支柱位置には誤差があり、理論上の閾値ギリギリでは接触
# リスクが残る。T字パーツとの直進クリアランス判定にだけ、この追加マージンを
# 上乗せする(支柱そのもの・旋回時の判定には影響しない)。
POST_ARM_STRAIGHT_MARGIN_CM = 3.0

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
# 2026-09-12に再実測し18.0cmに更新(グレー●の中心間距離ベースの
# (4,3)-(4,4)中点から垂直に18.0cm)。ゴール側は2026-09-15に実測し14.8cmに更新。
START_OFFSET_CM = 18.0
GOAL_OFFSET_CM = 14.8

# スタート: グリッド(4,3)-(4,4)を結ぶ線の中心点から、その線と垂直に
# START_OFFSET_CMだけ離れた点。
# 2026-09-21: スタート地点を、従来のスタート(X=4*GRID_PITCH_CM+START_OFFSET_CM, Y=3.5*GRID_PITCH_CM)から
# Y方向に-1cm移動した。さらに、Y方向に-1.7cm(グレーポイントの中点から-2.7cm)、X方向にETエリア側(-X)へ0.5cm移動した。
# 従来の位置に戻すときは START_SHIFT_CM を (0.0, 0.0) にする。
START_SHIFT_CM = (-0.5, -2.7)
START_POS_CM = (4 * GRID_PITCH_CM + START_OFFSET_CM + START_SHIFT_CM[0], 3.5 * GRID_PITCH_CM + START_SHIFT_CM[1])
START_HEADING_DEG = 180.0   # -x方向を向く

# ゴール: スタートと同じ考え方で、グリッド(0,3)-(0,4)を結ぶ線の中心点から
# 垂直にGOAL_OFFSET_CMだけ離れた点。
# 2026-09-20: ゴール地点を、従来のゴール(X=GOAL_LINE_X_CM, Y=3.5*GRID_PITCH_CM)から
# X方向に-61cm、Y方向に-115cm移動した(X=-75.8cm, Y=-28.9cm)。
# (2026-09-20: 当初X-58cmだったものを、さらに-3cm動かして-61cmにした。)
# 従来の位置に戻すときは GOAL_SHIFT_CM を (0.0, 0.0) にする。
GOAL_SHIFT_CM = (-61.0, -115.0)
# 従来のゴール位置のX座標(GOAL_SHIFT_CMを足す前の基準)。
GOAL_LINE_X_CM = 0 * GRID_PITCH_CM - GOAL_OFFSET_CM
GOAL_POS_CM = (GOAL_LINE_X_CM + GOAL_SHIFT_CM[0], 3.5 * GRID_PITCH_CM + GOAL_SHIFT_CM[1])
GOAL_HEADING_DEG = 90.0     # +y方向を向く

# --- 進入禁止エリア(2026-09-21) ---
# タイヤ中心(車軸)の軌跡だけを対象にする(車体の外形は見ない)。縁に触れるだけなら許容する。
# (xmin, xmax, ymin, ymax)のcm座標で、左コース基準。右コースはet_rally_runner.pyが左右反転する。
_INF = 1.0e6
KEEP_OUT_RECTS_CM = (
    # 11のグレーポイント(0, 0)から、下へ37cmより下(y < -37)。
    # ただし、ゴール側(-X)へ17cm以上(x <= -17)進んだ範囲は制限なし。
    (-17.0, _INF, -_INF, -37.0),
    # 従来のスタート位置(X=4*GRID_PITCH_CM+START_OFFSET_CM)から、ETエリアとは逆方向(+X)へ14cm以降のすべての範囲。
    # スタート位置をずらしても動かさないため、START_POS_CM ではなく、ずらす前の位置を基準にする。
    (4 * GRID_PITCH_CM + START_OFFSET_CM + 14.0, _INF, -_INF, _INF),
    # 11のグレーポイントから、ゴール側(-X)へ28cm以降(x <= -28)で、11の点より上(y >= 0)の範囲。
    (-_INF, -28.0, 0.0, _INF),
)
# ゴールへの最終区間の直進が禁止エリアに入るときに経由する中継点C。
# 14,15のグレーポイントの中点(x=0)から、ゴール側へGATE_EXIT_OFFSET_CMだけ離れたX、11の点のY(y=0)。
# ここまで来ればゴール側の制限のない範囲(x < -17)にいて、ゴールへは直進できる。
KEEP_OUT_GOAL_LANE_POINT_CM = (0.0 - GATE_EXIT_OFFSET_CM, 0.0)

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


# 2026-09-20: entry直前の直進(横移動を含む)で、車体footprintがtarget_gate自身の
# T字パーツから確保したい距離(cm)の「目標」(rule_route._try_raise_approach_run_for_arm)。
# 届かなければrule_route.MIN_BODY_CLEARANCE_FOR_STRONG_CM(3.0cm)を下限とする。
# 0なら無効(従来と完全に同じ経路になる)。60シードで比較したところ、有効にすると
# 平均+7.6cm(約1%)の遠回りで、3cm未満のゲート進入が28.6%から14.7%に減ったため、
# 2026-09-20に既定で有効(4.0)にした。--arm-clearance=0 で無効にできる。
ARM_BODY_CLEARANCE_TARGET_CM = 4.0

# 2026-09-20: ゴール地点に到達した後の旋回(GOAL_HEADING_DEGへ向く)を行うか。
# False: ゴール地点に着いた時点で終了し、向きは問わない(旋回ステップを出さない、
# 経路計画でもゴールでの旋回安全性は見ない)。
GOAL_FINAL_TURN = False

# 2026-09-20: T字パーツとの車体クリアランスの評価(rule_route._segment_body_clearance)で
# 使う、車体の後端の張り出し(cm)。0なら「旋回軸より後ろは見ない」(車体の前方だけ
# 評価)。従来の評価に戻すときは ROBOT_REAR_OVERHANG_CM にする。
ROBOT_REAR_OVERHANG_FOR_CLEARANCE_CM = 0.0

# 2026-09-20: target_gate以外のゲートのT字パーツにも、車体前方のクリアランスを
# ARM_BODY_CLEARANCE_MIN_CM以上確保する候補を優先する(rule_route._resolve_top_level_segment)。
# ARM_OTHER_GATES_LENGTH_BUDGET_CM: そのために許容する遠回り(最短候補からの増加分、cm)。
# 2026-09-22: 侵入側(8通り)の選択で、各ゲートのentryへ向かう区間(前のゲートのexitから、そのentryまで)が、
# そのゲート自身のT字パーツに、車体の側面から ARM_BODY_CLEARANCE_MIN_CM 以上の余裕を持つ組み合わせを、
# 優先する。ただし、そのために増える経路の長さ(最短の組み合わせからの増加分)が、この値(cm)以内のときだけ。
# 0以下なら、この優先を行わない(従来と同じ選び方)。
ARM_SIGN_PREFERENCE_BUDGET_CM = 60.0

# 2026-09-22: ゴールへの最終区間(最後のゲートのexitからゴールまで)の直進が、どの支柱(T字パーツ込み)にも、
# 物理的な最小距離(rule_route.STRAIGHT_CLEARANCE_CM、半幅+支柱半径)に、この値(cm)を足した距離以上
# 離れるように、まず試す。その条件で経路が作れなければ、従来の条件(余裕なし)で作る。
# 0以下なら、この余裕を求めない。
GOAL_STRAIGHT_EXTRA_MARGIN_CM = 1.0

# 2026-09-22: 「斜め→軸に平行な通路→軸に垂直→軸に沿ってentry」の候補
# (rule_route._try_diagonal_to_entry_axis の3つ目の形)の探索範囲。
# CORRIDOR_T_LIST_CM: 通路の、entry軸からの距離(cm)。CORRIDOR_SHIFT_LIST_CM: 通路へ乗る点の、
# aからの垂線の足に対するずらし量(cm)。CORRIDOR_MAX_CANDIDATES: 安全性の判定に回す最大の候補数。
CORRIDOR_T_LIST_CM = (14.0, 18.0, 22.0, 26.0, 30.0, 36.0, 44.0)
CORRIDOR_SHIFT_LIST_CM = (-40.0, -30.0, -20.0, -10.0, 0.0, 10.0, 20.0)
CORRIDOR_MAX_CANDIDATES = 4

ARM_CLEARANCE_OTHER_GATES = False  # 2026-09-20: 影響が大きい(68%の経路が変わり平均+2.2%)割に効果が小さいため一旦無効
ARM_BODY_CLEARANCE_MIN_CM = 3.0
ARM_OTHER_GATES_LENGTH_BUDGET_CM = 60.0

# 2026-09-20: 「斜めに進んでから、entryの軸に乗って直進する」候補
# (rule_route._try_diagonal_to_entry_axis)。entryの手前D(cm)の点へ斜めに進む。
DIAGONAL_ENTRY_ENABLED = True
DIAGONAL_ENTRY_D_MIN_CM = 8.0
DIAGONAL_ENTRY_D_MAX_CM = 48.0
DIAGONAL_ENTRY_D_STEP_CM = 8.0
# 斜め+ゲートの脇の線+entry軸(a -> p1 -> c -> b)の候補のパラメータ。
DIAGONAL_BYPASS_D_LIST_CM = (8.0,)
DIAGONAL_BYPASS_T_STEP_CM = 8.0
DIAGONAL_BYPASS_T_MAX_CM = 48.0
