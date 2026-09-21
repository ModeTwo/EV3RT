"""ETラリー フェーズ1で作った、可視グラフ+Dijkstraによる幾何最短経路の生成。

【重要】このファイルのplan_path_detailed()/plan_path()/build_graph()/
dijkstra_with_turns()(=このファイルの主目的だった経路計算そのもの)は、
現在はexport_plan.py/generate_diagram.py/run_random.pyのどこからも
呼ばれておらず、本番の経路計算には使われていない(rule_route.pyの
plan_route()に置き換わった)。角度をつけた斜め通過やDijkstra探索など、
このファイル固有のアイデアはもう採用していないので、参考・比較用として
このファイルに残している。

一方、以下はrule_route.py側から実際に使われている共通部品なので、
削除・変更する際は影響範囲に注意すること:
  - Gateクラス(ただしcandidate_points/candidate_crossings/
    valid_half_length/_crossing_reachの4メソッドは上記と同じく未使用。
    rule_route.py側はgate.center/gate.normal/gate.direction/gate.posts()
    だけを使い、entry/exitは自前のgate_entry_exit()で計算している)
  - grid_to_cm() / build_stage_sequence()
  - pivot_turn_safe()(および_corner_spec/_CORNER_SPECS)

以下、このファイル固有の(現在は不使用の)旧方式の設計メモ:
  - ロボットは直線移動 + その場旋回(任意角度)のみで動く。
  - 障害物(対象外のゲート脚)は円として近似し、回避半径ぶん膨らませる。
  - 可視グラフ(visibility graph)を構築し、Dijkstraで最短経路を求める。
    ノード = スタート/ゴール/各ゲート通過候補点/各障害物円の外周サンプル点
    エッジ = 2ノード間の直線が、どの障害物円の内部も通らない場合にのみ張る
  - ゲート通過順序(赤→青→黄 x3周)は「隣接ステージ間しか直接つながない」
    という制約をエッジ生成時に課すことで、Dijkstra一回で自然に守られる。

このファイル内の処理の大まかな流れ(plan_path_detailed()が(旧方式の)入口):
  1. Gateクラス: 1つのゲート(支柱2本)の形状・通過可能範囲を扱う
  2. build_graph(): 可視グラフ(ノードとエッジ)を組み立てる
  3. dijkstra_with_turns(): 距離+旋回コストが最小の経路を、旋回の安全性
     (pivot_turn_safe)も見ながら探索する
  4. plan_path_detailed(): 1〜3をまとめて呼び出し、結果を使いやすい形
     (座標列・距離・ラベル)に整えて返す
詳しい全体像・現行方式(rule_route.py)についてはREADME.mdを参照。
"""

import heapq
import math

import config
import geometry as geo


class Post(tuple):
    """支柱の位置(x, y)を表す、通常の2-tupleと完全互換のサブクラス。

    支柱の足元にはT字パーツ(ゲート法線方向に±POST_PIVOT_ARM_HALF_LENGTH_CM
    伸びる直径2cmの棒)が付いている(config.POST_PIVOT_ARM_HALF_LENGTH_CM参照)。
    これをpivot_turn_safeでカプセル状の障害物として扱うため、法線方向の
    単位ベクトルをarm_dir属性として追加で持たせる。タプル自体はあくまで
    (x, y)の2要素のみで構成されるため、添字アクセス・geo関数・JSON化など
    既存コードからは普通の(x, y)タプルと区別なく扱える。

    2026-09-18: colorに、この支柱が属するゲートの色("red"/"blue"/
    "yellow")を持たせる(rule_route._straight_clearance_thresholdが、
    「侵入/退出予定のゲート自身のT字パーツ」かどうかを判定するために使う。
    無関係な別ゲートの支柱にまで実機マージンを広げると経路構築が不安定に
    なることが分かったため、対象を絞り込む必要があった)。
    """

    def __new__(cls, point, arm_dir=(0.0, 0.0), color=None):
        obj = super().__new__(cls, (point[0], point[1]))
        obj.arm_dir = arm_dir
        obj.color = color
        return obj


class Gate:
    """1つのゲート(支柱2本1組)を表すクラス。

    foot_a/foot_bは支柱の位置(cm座標)。ゲートに「表裏」はなく、a->bの
    向きは単に2本の脚を区別するためのラベルに過ぎない(通過方向は
    candidate_crossings()で両方向とも候補に含める)。
    """

    def __init__(self, color, foot_a_grid, foot_b_grid):
        self.color = color
        # foot_*_grid: グリッド座標(整数の(col, row))のまま保持。ラベル表示用。
        self.foot_a_grid = foot_a_grid
        self.foot_b_grid = foot_b_grid
        # foot_*: 上記をcm座標(実際の計算で使う単位)に変換したもの。
        self.foot_a = grid_to_cm(foot_a_grid)
        self.foot_b = grid_to_cm(foot_b_grid)

    @property
    def center(self):
        """2本の支柱のちょうど中間点(ゲートの中心)。"""
        return geo.scale(geo.add(self.foot_a, self.foot_b), 0.5)

    @property
    def direction(self):
        """foot_a -> foot_b の単位ベクトル。"""
        d = geo.sub(self.foot_b, self.foot_a)
        length = geo.norm(d)
        return geo.scale(d, 1.0 / length)

    @property
    def normal(self):
        """ゲートの開口面(2本の脚を結ぶ線)に垂直な単位ベクトル。"""
        dx, dy = self.direction
        return (-dy, dx)

    def posts(self):
        """2本の支柱の座標(cm)をリストで返す。障害物としての位置に使う。

        各支柱をPost(arm_dir=ゲート法線の単位ベクトル)として返し、
        T字パーツの向きをpivot_turn_safe側で参照できるようにしている。
        """
        return [Post(self.foot_a, self.normal, self.color), Post(self.foot_b, self.normal, self.color)]

    def valid_half_length(self, angle_deg=0.0):
        """ロボットが安全に通過できる、中心からの片側最大距離。

        angle_degは、ゲートの法線方向からの通過角度のズレ。旋回軸(タイヤ)が
        前端寄りにあるため、車体は前後非対称(前7.5cm/後ろ18cm)。斜めに
        通過するとゲート沿い方向への張り出しは、長い方(後ろ)が支配的になる。
        """
        phi = math.radians(angle_deg)
        half = config.GATE_INNER_WIDTH_CM / 2.0
        swept_half_width = (
            config.ROBOT_REAR_OVERHANG_CM * abs(math.sin(phi))
            + config.ROBOT_HALF_WIDTH_CM * abs(math.cos(phi))
        )
        margin = swept_half_width + config.SAFETY_MARGIN_CM
        return half - margin

    def _crossing_reach(self, overhang_cm, angle_deg):
        """通過候補点Pから、車体の指定した端(前 or 後ろ)が線分に触れる/
        抜けきる位置までの、進行方向沿いの距離。overhang_cmは旋回軸から
        その端までの距離(前=ROBOT_FRONT_OVERHANG_CM、後ろ=ROBOT_REAR_OVERHANG_CM)。
        """
        phi = math.radians(angle_deg)
        return overhang_cm + config.ROBOT_HALF_WIDTH_CM * abs(math.tan(phi))

    def candidate_points(self, n, angle_deg=0.0):
        """通過可能区間(角度angle_degでの有効幅)をn点でサンプリングする。"""
        half = self.valid_half_length(angle_deg)
        c = self.center
        d = self.direction
        if n == 1:
            return [c]
        points = []
        for i in range(n):
            t = -half + (2 * half) * i / (n - 1)
            points.append(geo.add(c, geo.scale(d, t)))
        return points

    def candidate_crossings(self, n_samples, angles_deg):
        """通過候補ごとに (entry, exit, true_entry, true_exit) を列挙する。

        entry(=true_entry)は「旋回軸がここにある時、前端がちょうど線分に
        触れる」位置、exit(=true_exit)は「旋回軸がここにある時、後端が
        ちょうど線分を抜けきる」位置(この2点の間が、ロボット全体が線分を
        通過しきったと言える区間)。

        entry地点・exit地点では、その手前/先の経路との向きの違いぶん旋回が
        起きることがある。旋回はタイヤ位置を中心に起きるため車体後端が
        最大約19.2cm振れるが、これは「実際にその旋回で車体が届く範囲に
        支柱が入っているか」という、旋回角度に依存した判定が必要
        (dijkstra_with_turnsのpivot_turn_safeで行う)。ここでは一律に
        安全距離ぶん離す、という単純な対処はしない
        (支柱同士が近いゲート配置(隣接グリッド等)では、単純に距離だけで
        判定すると通過候補が全滅してしまう不具合が実際に発生したため)。

        ゲートに表裏はないので、法線の+方向/-方向どちらから進入するかも
        候補に含める(この場合「前」「後ろ」の役割はdの向きに従って
        自動的に入れ替わる)。斜め角度が大きいと物理的に通過不可能
        (valid_half_length<=0)になるので、その角度は候補から除外する。
        """
        results = []
        for angle_deg in angles_deg:
            valid = self.valid_half_length(angle_deg)
            if valid <= 0:
                continue
            front_reach = self._crossing_reach(config.ROBOT_FRONT_OVERHANG_CM, angle_deg)
            rear_reach = self._crossing_reach(config.ROBOT_REAR_OVERHANG_CM, angle_deg)
            for p in self.candidate_points(n_samples, angle_deg):
                for sign in (1.0, -1.0):
                    base_n = geo.scale(self.normal, sign)
                    d = geo.rotate(base_n, angle_deg)
                    true_entry = geo.sub(p, geo.scale(d, front_reach))
                    true_exit = geo.add(p, geo.scale(d, rear_reach))
                    entry_pt = true_entry
                    exit_pt = true_exit
                    results.append((entry_pt, exit_pt, true_entry, true_exit))
        return results


def grid_to_cm(grid_point):
    """グリッド座標(整数の(col, row))を、実際のcm座標に変換する。"""
    col, row = grid_point
    return (col * config.GRID_PITCH_CM, row * config.GRID_PITCH_CM)


def build_open_space_grid(all_posts):
    """コース+スタート/ゴール周辺を覆う、粗いグリッド状の中継点を作る。

    支柱の回避半径の内側に入ってしまう点(=そもそも障害物の中)は除外する。
    ここでの回避半径は車体半幅ベースの小さい方(POST_AVOID_RADIUS_TRANSLATION_CM)を
    使う。中継点自体での旋回が安全かどうかはpivot_turn_safeが個別に判定するため、
    ここでは「純粋な地点として支柱の中に入っていないか」だけを見ればよい
    (以前は旋回スイープ半径ベースの大きい方を使っていたが、支柱同士が近い配置で
    タイトな迂回ルートが幾何学的に構築不可能になる不具合があったため変更した)。
    """
    xs = [config.START_POS_CM[0], config.GOAL_POS_CM[0], 0.0, 4 * config.GRID_PITCH_CM]
    ys = [config.START_POS_CM[1], config.GOAL_POS_CM[1], 0.0, 4 * config.GRID_PITCH_CM]
    margin = config.OPEN_SPACE_GRID_MARGIN_CM
    x_min, x_max = min(xs) - margin, max(xs) + margin
    y_min, y_max = min(ys) - margin, max(ys) + margin

    spacing = config.OPEN_SPACE_GRID_SPACING_CM
    points = []
    x = x_min
    while x <= x_max + 1e-6:
        y = y_min
        while y <= y_max + 1e-6:
            p = (x, y)
            if all(geo.distance(p, post) > config.POST_AVOID_RADIUS_TRANSLATION_CM for post in all_posts):
                points.append(p)
            y += spacing
        x += spacing
    return points


def build_stage_sequence(gates_by_color, laps=None):
    """[(lap, color, Gate), ...] を 赤,青,黄 x laps 周分の順序で作る。
    laps省略時はconfig.LAPSを使う(2026-09-18: 地区大会で1周/2周狙いに
    切り替えられるよう、呼び出し側から周回数を指定できるようにした)。"""
    if laps is None:
        laps = config.LAPS
    sequence = []
    for lap in range(1, laps + 1):
        for color in config.GATE_ORDER:
            sequence.append((lap, color, gates_by_color[color]))
    return sequence


class _Node:
    __slots__ = ("point", "stage", "kind", "pair", "true_point")

    def __init__(self, point, stage, kind, pair=None, true_point=None):
        self.point = point
        # stage: -1=start, 0..N-1=ゲート通過ステージ, N=goal, None=障害物回避用(自由ノード)
        self.stage = stage
        # kind: "start" / "goal" / "gate_entry" / "gate_exit" / "free"
        self.kind = kind
        # gate_entry/gate_exitの場合、対になるノードidを持つ
        self.pair = pair
        # gate_entry/gate_exitの場合、「実際に車体の前端/後端が線分に
        # 触れる/抜けきる」点(pointと同一の値。通過判定の検証や図の
        # 通過マーカー表示のために、他のノード種別と統一的にアクセスできる
        # よう別フィールドとして保持している)。
        self.true_point = true_point


def build_graph(gates_by_color):
    """可視グラフを構築する。戻り値: (nodes, start_id, goal_id, stage_node_ids, adjacency)

    ゲート通過の表現(重要):
      「線分上の1点を通ればゲート通過」とすると、ロボットがその点に触れて
      すぐ引き返す経路も最短として選ばれてしまう(実際に発生した不具合)。
      ゲート通過の判定は「ロボット全体が線分を通過しきる」ことなので、
      各ゲートの通過候補点Pごとに、線分に垂直な方向へロボット全長ぶん
      離れた「entry(進入完了点)」と「exit(退出完了点)」のペアを作り、
      entry->exit を必ず直進で結ぶ(=ロボットの全長が線分の反対側まで
      抜けたことを表す)強制エッジとして経路に組み込む。
      ゲートに表裏はないので、両方向(+法線 / -法線)の2通りを候補に含める。

    障害物回避用ノードは「区間(レグ)ごとに別インスタンス」として複製する。
    共有(使い回し)にしてしまうと、障害物ノードがステージ制約を持たないことを
    利用して start -> 障害物ノード -> goal のように必須ゲートを飛び越える経路が
    最短として選ばれてしまうため(実際に発生した不具合)。レグごとに複製することで、
    障害物ノードは「そのレグの区間移動」にしか使えなくなり、ゲート順序が
    Dijkstra一回の探索でも必ず守られるようになる。
    """
    stage_sequence = build_stage_sequence(gates_by_color)
    num_stages = len(stage_sequence)

    all_posts = []
    for gate in gates_by_color.values():
        all_posts.extend(gate.posts())

    # 支柱をタイトにハグする中継点。車体半幅ベースの小さい方の半径で配置する
    # (このすぐ下のtry_add_edge/pivot_turn_safeの項も参照。旋回スイープ半径
    # ベースの大きい方で配置していた頃は、支柱同士が近いゲート配置で
    # タイトな迂回ルートが幾何学的に構築不可能になり、無駄に大きく迂回する
    # 経路しか選べなくなる不具合があった)。
    obstacle_boundary_pts = [
        geo.circle_boundary_points(post_center, config.POST_AVOID_RADIUS_TRANSLATION_CM, config.OBSTACLE_BOUNDARY_SAMPLES)
        for post_center in all_posts
    ]

    # 障害物のすぐ周りだけに迂回ノードを置くと、「複数の障害物を大きく回り込んで
    # 避ける」経路(距離は伸びるが旋回はずっと少ない)を見つけられない
    # (実際にこれが原因で、赤ゲート手前で不要に大きい旋回が残っていた)。
    # そこで、開けた空間にも粗いグリッド状の中継点を追加し、狭い場所での
    # 支柱回避(ピンポイントな迂回)と、広い場所での大回り(粗いグリッド)の
    # 両方を探索できるようにする。
    open_space_pts = build_open_space_grid(all_posts)
    free_base_pts = obstacle_boundary_pts + [[p] for p in open_space_pts]

    nodes = []
    nodes.append(_Node(config.START_POS_CM, -1, "start"))
    start_id = 0
    nodes.append(_Node(config.GOAL_POS_CM, num_stages, "goal"))
    goal_id = 1

    # (u, v, dist, directed) を後でまとめてadjacencyへ。directed=Trueはu->vのみ。
    #
    # gate_entry/gate_exitノードは有向エッジで扱う必要がある。無向(双方向)の
    # まま「同じレグの障害物回避ノード」につないでしまうと、entryノードに一度
    # 到着したあと、そのままペアのexitへ進まずに障害物ノード経由で別の
    # entry候補へ抜けてしまい、「線分に触れるだけで通過したことになる」という
    # 修正前と同じ抜け穴が別の形で再発する(実際に発生した不具合)。
    # そのため: 障害物ノード/直前レグ -> entry、exit -> 障害物ノード/次レグ、
    # entry -> exit(ペア) はすべて一方向のみとし、
    # 「entryに入ったら必ずペアのexitにしか進めない」ようにする。
    adjacency_pairs = []

    def segment_clear(pu, pv, radius):
        for post_center in all_posts:
            if geo.segment_blocked_by_circle(pu, pv, post_center, radius):
                return False
        return True

    def try_add_edge(u, v, directed=False):
        pu, pv = nodes[u].point, nodes[v].point

        # どのエッジも直線移動(旋回はノード上でのみ起きる)なので、常に
        # 車体半幅ベースの(小さい方の)クリアランスで判定する。旋回そのものの
        # 安全性は、通過するノードごとにdijkstra_with_turns内のpivot_turn_safeで
        # 別途チェックする(距離ではなく、実際に曲がる角度の範囲(円弧)に
        # 支柱が入っているかで判定するため、ここで一律に大きい方の半径を
        # 要求するより的確に安全性を判定できる)。
        #
        # 以前はfreeノード同士(自由に配置できるので支柱の近くを通る必要は
        # ない)には旋回スイープ半径ベースの大きい方を使っていたが、支柱同士が
        # 近いゲート配置では、それが原因でタイトな迂回ルートが幾何学的に
        # 構築不可能になり、大きく迂回する経路しか選べなくなる不具合が
        # 実際に発生した(赤ゲート退出直後の経路が、コース外まで大回りする形に
        # なっていた)。小さい方に統一することで、この問題を解消している。
        #
        # (計算コストの注記: 小さい方を全エッジに使うと、遠く離れたfree/
        # obstacleノード同士まで大量に「見通せる」ことになり、Dijkstraの
        # 状態数が増えて計算時間が伸びる。現時点では正確な経路を優先し、
        # 速度は度外視している。)
        if segment_clear(pu, pv, config.POST_AVOID_RADIUS_TRANSLATION_CM):
            d = geo.distance(pu, pv)
            adjacency_pairs.append((u, v, d, directed))

    entry_ids_by_stage = []
    exit_ids_by_stage = []
    for stage_idx, (lap, color, gate) in enumerate(stage_sequence):
        entry_ids, exit_ids = [], []
        for entry_pt, exit_pt, true_entry_pt, true_exit_pt in gate.candidate_crossings(
            config.GATE_CROSS_SAMPLES, config.GATE_CROSSING_ANGLES_DEG
        ):
            nodes.append(_Node(entry_pt, stage_idx, "gate_entry", true_point=true_entry_pt))
            entry_id = len(nodes) - 1
            nodes.append(_Node(exit_pt, stage_idx, "gate_exit", true_point=true_exit_pt))
            exit_id = len(nodes) - 1
            nodes[entry_id].pair = exit_id
            nodes[exit_id].pair = entry_id
            # entry -> exit の強制通過エッジ(旋回可能地点から旋回可能地点まで、
            # 間にtrue_entry/true_exitを含む一直線)。一方向のみ
            # (exitからentryへ逆走はできない)。
            if segment_clear(entry_pt, exit_pt, config.POST_AVOID_RADIUS_TRANSLATION_CM):
                adjacency_pairs.append((entry_id, exit_id, geo.distance(entry_pt, exit_pt), True))
                entry_ids.append(entry_id)
                exit_ids.append(exit_id)
        entry_ids_by_stage.append(entry_ids)
        exit_ids_by_stage.append(exit_ids)

    # レグ = (start含む/ゲート含む)チェックポイント間の1区間。
    # レグ数 = ゲートステージ数 + 1 (start->stage0, stage(k)->stage(k+1), stage(last)->goal)
    for leg in range(num_stages + 1):
        left_ids = [start_id] if leg == 0 else exit_ids_by_stage[leg - 1]
        right_ids = [goal_id] if leg == num_stages else entry_ids_by_stage[leg]

        # このレグ専用の中継ノード(支柱回避+開けた空間のグリッド)を複製する
        leg_free_ids = []
        for pts in free_base_pts:
            ids = []
            for pt in pts:
                nodes.append(_Node(pt, None, "free"))
                ids.append(len(nodes) - 1)
            leg_free_ids.append(ids)
        leg_obstacle_flat = [i for ids in leg_free_ids for i in ids]

        # 左チェックポイント群 -> 右チェックポイント群(直接見通せる場合。一方向)
        for u in left_ids:
            for v in right_ids:
                try_add_edge(u, v, directed=True)

        # 左チェックポイント群 -> このレグの障害物回避ノード(一方向: 出発側)
        for u in left_ids:
            for v in leg_obstacle_flat:
                try_add_edge(u, v, directed=True)

        # このレグの障害物回避ノード -> 右チェックポイント群(一方向: 到着側)
        for u in leg_obstacle_flat:
            for v in right_ids:
                try_add_edge(u, v, directed=True)

        # このレグの障害物回避ノード同士(複数障害物をまたいで迂回するため、双方向)
        for i in range(len(leg_obstacle_flat)):
            for j in range(i + 1, len(leg_obstacle_flat)):
                try_add_edge(leg_obstacle_flat[i], leg_obstacle_flat[j])

    adjacency = [[] for _ in nodes]
    for u, v, d, directed in adjacency_pairs:
        adjacency[u].append((v, d))
        if not directed:
            adjacency[v].append((u, d))

    return nodes, start_id, goal_id, adjacency


def _corner_spec(overhang_cm):
    """旋回軸から見た車体の隅(前 or 後ろ側の左右どちらか)の、半径と
    進行方向からの角度オフセットを返す。overhang_cmは符号付き
    (前方向を正)なので、後ろの隅はatan2に負の値が入り、90°を超える
    オフセット角(後方寄り)が自然に得られる。
    """
    radius = math.hypot(overhang_cm, config.ROBOT_HALF_WIDTH_CM)
    offset = math.degrees(math.atan2(config.ROBOT_HALF_WIDTH_CM, overhang_cm))
    return radius, offset


_CORNER_SPECS = [
    _corner_spec(config.ROBOT_FRONT_OVERHANG_CM),
    (_corner_spec(config.ROBOT_FRONT_OVERHANG_CM)[0], -_corner_spec(config.ROBOT_FRONT_OVERHANG_CM)[1]),
    _corner_spec(-config.ROBOT_REAR_OVERHANG_CM),
    (_corner_spec(-config.ROBOT_REAR_OVERHANG_CM)[0], -_corner_spec(-config.ROBOT_REAR_OVERHANG_CM)[1]),
]


def pivot_turn_safe(point, heading_in_deg, heading_out_deg, all_posts):
    """その場旋回中、車体の四隅が近くの支柱に接触しないかを判定する。

    各隅は旋回軸(point、=ノード座標)を中心に固定半径の円を描くが、実際に
    振れるのは heading_in_deg から heading_out_deg への旋回ぶんの弧だけ。
    そのため「支柱までの距離が必要クリアランス未満」かつ「支柱の方向が
    その弧の範囲内」の両方を満たす場合のみ危険と判定する(距離だけで一律に
    判定すると、実際の旋回角が小さいケースまで安全域を過大評価し、隣接
    グリッドのゲートなどで通過候補が全滅する不具合が実際に発生したため)。

    支柱の足元のT字パーツ(ゲート法線方向に±POST_PIVOT_ARM_HALF_LENGTH_CM
    伸びる棒、Gate.posts()参照)を考慮するため、post.arm_dirが
    ゼロベクトルでなければ「支柱中心を通りarm_dir方向に伸びる線分」との
    最近接点を使って距離・角度を計算する(=カプセル型のチェック)。半径は
    POST_RADIUS_CMのまま変えていない(T字パーツ自体も支柱と同じ直径2cmの
    棒のため)。arm_dirを持たない(普通の(x, y)タプルの)postは従来通り
    点として扱う。一度「全方向に太い円柱」として実装し、ゲート開口部が
    狭くなりすぎて600件中0件しか安全な経路が見つからなくなったことがある
    (config.POST_PIVOT_ARM_HALF_LENGTH_CMのコメント参照)ため、必ず
    「特定方向にだけ伸びた細い棒」のモデルを保つこと。

    heading_in_degからheading_out_degへの旋回がちょうど180度(ゴール到着時の
    完全な反転など)の場合、geo.normalize_degが常に-180度側に正規化して
    しまうため、実際には左右どちらに回ってもよい(同じ最終的な向きになる)
    にもかかわらず、片方向(正規化された側)の弧しか判定していなかった。
    そのため、危険な支柱がたまたま正規化される側にだけあり、逆方向(実機なら
    そちらへ回せば接触しない)に回れば安全なケースを、不要に「危険」と判定
    して迂回を強制していた。180度ちょうどの場合だけ、ごくわずかに角度を
    ずらした2方向(左回り・右回り相当)をそれぞれ判定し、どちらか一方が
    安全ならOKとする。
    """
    def _clear(h_out):
        for radius, offset in _CORNER_SPECS:
            needed = radius + config.POST_RADIUS_CM + config.SAFETY_MARGIN_CM
            arc_start = heading_in_deg + offset
            arc_end = h_out + offset
            for post in all_posts:
                arm_dir = getattr(post, "arm_dir", (0.0, 0.0))
                if arm_dir[0] or arm_dir[1]:
                    half = config.POST_PIVOT_ARM_HALF_LENGTH_CM
                    seg_a = geo.sub(post, geo.scale(arm_dir, half))
                    seg_b = geo.add(post, geo.scale(arm_dir, half))
                    nearest = geo.closest_point_on_segment(point, seg_a, seg_b)
                else:
                    nearest = post
                d = geo.distance(point, nearest)
                if d >= needed:
                    continue
                post_angle = math.degrees(math.atan2(nearest[1] - point[1], nearest[0] - point[0]))
                if geo.angle_in_arc(post_angle, arc_start, arc_end):
                    return False
        return True

    turn = geo.normalize_deg(heading_out_deg - heading_in_deg)
    if abs(abs(turn) - 180.0) < 1e-6:
        return _clear(heading_out_deg - 0.01) or _clear(heading_out_deg + 0.01)
    return _clear(heading_out_deg)


def dijkstra_with_turns(nodes, adjacency, start_id, goal_id, start_heading_deg, goal_heading_deg,
                         turn_cost_per_deg, all_posts):
    """旋回コストを考慮した最短経路探索。

    普通のDijkstraは「今どのノードにいるか」だけを状態にするが、それだと
    次にどちらへ曲がるかのコストを評価できない(直前にどこから来たかが
    分からないと、曲がる角度が定義できないため)。そこで状態を
    (直前ノード, 現在ノード) というペア(=有向グラフの辺)に拡張し、
    遷移コストを distance + turn_cost_per_deg * |旋回角| とする。
    スタート直後の最初の一歩は、直前ノードの代わりにSTART_HEADING_DEGを使う
    (prev=-1で表す)。ゴール到達後、指定の最終姿勢(goal_heading_deg)に
    向くための旋回コストも加算してから比較する。

    各ノードを通過するたびに、そこでの旋回(直前の進行方向heading_inから
    次の進行方向heading_outへの旋回)がpivot_turn_safeで安全かを確認し、
    安全でなければその遷移自体を候補から除外する(コストを足して不利に
    するのではなく、そもそも通れない経路として扱う)。
    """

    def heading_of(a_id, b_id):
        pa, pb = nodes[a_id].point, nodes[b_id].point
        return math.degrees(math.atan2(pb[1] - pa[1], pb[0] - pa[0]))

    start_state = (-1, start_id)
    best_cost = {start_state: 0.0}
    prev_state = {}
    pq = [(0.0, start_state)]

    best_total = math.inf
    best_final_state = None

    while pq:
        cost, (pv, cur) = heapq.heappop(pq)
        if cost > best_cost.get((pv, cur), math.inf):
            continue

        heading_in = start_heading_deg if pv == -1 else heading_of(pv, cur)

        if cur == goal_id:
            if not pivot_turn_safe(nodes[cur].point, heading_in, goal_heading_deg, all_posts):
                continue
            final_turn = abs(geo.normalize_deg(goal_heading_deg - heading_in))
            total = cost + turn_cost_per_deg * final_turn
            if total < best_total:
                best_total = total
                best_final_state = (pv, cur)
            continue  # goalに出ていくエッジはないので展開不要

        for nxt, w in adjacency[cur]:
            heading_out = heading_of(cur, nxt)
            if not pivot_turn_safe(nodes[cur].point, heading_in, heading_out, all_posts):
                continue
            turn = abs(geo.normalize_deg(heading_out - heading_in))
            new_cost = cost + w + turn_cost_per_deg * turn
            new_state = (cur, nxt)
            if new_cost < best_cost.get(new_state, math.inf):
                best_cost[new_state] = new_cost
                prev_state[new_state] = (pv, cur)
                heapq.heappush(pq, (new_cost, new_state))

    if best_final_state is None:
        raise RuntimeError("スタートからゴールへの経路が見つかりませんでした(障害物により経路が塞がれている可能性があります)")

    path_ids = []
    state = best_final_state
    while state != start_state:
        pv, cur = state
        path_ids.append(cur)
        state = prev_state[state]
    path_ids.append(start_id)
    path_ids.reverse()
    return path_ids, best_total


def plan_path(gates_by_color):
    """経路計画のエントリポイント。ウェイポイント座標のリストを返す。"""
    waypoints, total_dist, _labels, _true_points = plan_path_detailed(gates_by_color)
    return waypoints, total_dist


def plan_path_detailed(gates_by_color):
    """waypoints, total_dist_cm, labels, true_points を返す。

    探索自体は「距離 + 旋回コスト」の合計(config.TURN_COST_PER_DEGREE_CM)を
    最小化するが、ここで返す total_dist_cm は実際に走る物理距離(cm)のみの
    合計であり、旋回コストは含まない(表示・比較用の指標として素直な値にするため)。

    ラベルは "start" / "goal" / "lap{n}-{color}-entry" / "lap{n}-{color}-exit"
    (ゲート通過の進入完了点/退出完了点) / None (障害物回避用の中継点)。
    可視化やデバッグで、どの点がどのゲート通過に対応するかを示すために使う。

    true_pointsはwaypointsと同じ長さで、gate_entry/gate_exitの点については
    waypoints側と同じ「実際に車体の前端/後端が線分に触れる/抜けきる」座標
    (waypoints[i]と同一)、それ以外はNone。
    """
    stage_sequence = build_stage_sequence(gates_by_color)
    all_posts = [p for gate in gates_by_color.values() for p in gate.posts()]
    nodes, start_id, goal_id, adjacency = build_graph(gates_by_color)
    path_ids, _combined_cost = dijkstra_with_turns(
        nodes, adjacency, start_id, goal_id,
        config.START_HEADING_DEG, config.GOAL_HEADING_DEG, config.TURN_COST_PER_DEGREE_CM,
        all_posts,
    )
    waypoints = [nodes[i].point for i in path_ids]
    total_dist_cm = sum(geo.distance(waypoints[i], waypoints[i + 1]) for i in range(len(waypoints) - 1))

    labels = []
    true_points = []
    for i in path_ids:
        node = nodes[i]
        if node.kind == "start":
            labels.append("start")
        elif node.kind == "goal":
            labels.append("goal")
        elif node.kind in ("gate_entry", "gate_exit"):
            lap, color, _gate = stage_sequence[node.stage]
            suffix = "entry" if node.kind == "gate_entry" else "exit"
            labels.append(f"lap{lap}-{color}-{suffix}")
        else:
            labels.append(None)
        true_points.append(node.true_point)

    return waypoints, total_dist_cm, labels, true_points
