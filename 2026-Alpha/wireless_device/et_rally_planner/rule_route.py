"""ルールベースの経路構築(可視グラフ+Dijkstraの旧方式(planner.plan_path_detailed)
を置き換える新方式)。export_plan.py/generate_diagram.py/run_random.pyから
plan_route()またはplan_route_with_anchors()として呼ばれる、実際の出力経路の
本体。

基本設計:
  - ゲート通過は常に「ゲートの中心を通り、ゲートに垂直」に固定する
    (角度をつけたり中心からずらしたりする候補探索は行わない)。
  - entry(侵入完了点)は、ゲート中心Cから、進入側の法線n方向に
    前方オーバーハング(7.5cm)だけ離れた点(=車体前端がCに重なる位置)。
  - exit(通過完了点)は、Cから反対方向に後方オーバーハング(18cm)だけ
    離れた点(=車体後端がCに重なる位置)。entry->exitの距離は常に
    車体全長(25.5cm)になる。
  - 2点を直進で結ぶ線が、いずれかのゲートの支柱にぶつかる場合は、
    「手前の侵入ポイント」(ぶつかった支柱から、その支柱が属するゲートの
    法線方向にオフセットした点)を経由する(resolve_segment、長方形
    オフセット方式)。同じ支柱に別軸でぶつかる場合は、代わりにその軸
    方向へ直進を延長して回避する候補も試す
    (_resolve_segment_via_parallel_extension)。この迂回・延長を挟んでも
    まだ別の支柱にぶつかる場合は、同じ処理を再帰的に適用する。
  - ステージ間をつなぐ最上位区間(_resolve_top_level_segment)では、上記
    2方式に加えて「別のゲートを正式なentry/exitで経由してから向かう」
    候補(_resolve_via_other_gate)も常に試し、entry/ゴール到着時の旋回・
    経路中の旋回・他ゲートの脚への意図しない接触のいずれも起きない
    候補の中から最短のものを採用する。
  - どのゲートも「一方向からのみ侵入可能」(同じ色は3周とも同じ向きで
    通過し、無関係な区間で脚を結ぶ線を逆向きや斜めに横切ってはならない)
    というルールがあり、これは経路構築後に_fix_gate_crossingsで検出・
    修正する。

支柱への接触判定は「車体半幅+支柱半径」という物理的な最小値
(STRAIGHT_CLEARANCE_CM)のみで行い、実機での余裕(安全マージン)は
含めない(必要になれば別途調整する)。
"""

import itertools
import math

import config
import geometry as geo
from planner import Gate, build_stage_sequence, pivot_turn_safe

# 直進区間が支柱にぶつかったと判定するクリアランス。安全マージンは含めない
# (半幅+支柱半径という、物理的に触れずに済む最小距離のみ)。
STRAIGHT_CLEARANCE_CM = config.ROBOT_HALF_WIDTH_CM + config.POST_RADIUS_CM
# entry自身のゲートの支柱を避けるときの迂回点の最低オフセット。
# STRAIGHT_CLEARANCE_CMより大きければ理論上は足りるが、隣接区間との
# 干渉(直前の点がちょうど支柱と同じ高さにあるなど)を考慮した実測値。
MIN_ENTRY_AVOID_OFFSET_CM = config.ROBOT_REAR_OVERHANG_CM


def gate_entry_exit(gate, sign):
    """gateを法線方向signの側(+1 or -1)から通過する場合のentry/exitを返す。

    exit側はconfig.GATE_EXIT_OFFSET_CM(ROBOT_REAR_OVERHANG_CM+T字パーツの
    到達距離)を使う。ROBOT_REAR_OVERHANG_CMそのままだと、支柱の足元の
    T字パーツをpivot_turn_safeでカプセルとして正しく考慮した際、
    exit直後に旋回する経路の多くが安全マージンを使い果たしてしまうため
    (2026-09-15、詳細はconfig.GATE_EXIT_OFFSET_CMのコメント参照)。
    """
    n = geo.scale(gate.normal, sign)
    entry = geo.add(gate.center, geo.scale(n, config.ROBOT_FRONT_OVERHANG_CM))
    exit_ = geo.sub(gate.center, geo.scale(n, config.GATE_EXIT_OFFSET_CM))
    return entry, exit_


def _post_owner_gate(post, all_gates):
    """postがどのゲートの支柱かを特定して、そのGateを返す。"""
    for gate in all_gates:
        for p in gate.posts():
            if geo.distance(p, post) < 1e-6:
                return gate
    raise ValueError(f"post {post} の持ち主ゲートが見つかりません")


def _signed_offset(point, gate):
    """gate中心から見た、pointの法線方向への符号付き投影量(スカラー)。"""
    return geo.dot(geo.sub(point, gate.center), gate.normal)


def _project_param(a, b, p):
    """線分ab上で、点pに最も近い位置をab方向の0..1のtで返す(範囲外もありうる)。"""
    ab = geo.sub(b, a)
    ab_len2 = geo.dot(ab, ab)
    if ab_len2 == 0:
        return 0.0
    return geo.dot(geo.sub(p, a), ab) / ab_len2


def _segment_post_distance(a, b, post):
    """線分ab(直進区間、旋回軸の軌跡)と支柱postの最短距離。

    2026-09-17: 支柱のT字パーツ(post.arm_dir、pivot_turn_safeの
    カプセル型チェック参照)が、直進区間の安全性チェック(この関数の
    呼び出し元、_find_blocking_post/verify_straight_clearance)では
    一切考慮されていなかった不具合が、実機での接触(seed=9392783、
    黄色ゲート右支柱のT字パーツに車体前方が接触)で発覚した。
    旋回時はpivot_turn_safeがカプセル対応済みだったが、直進時は
    支柱の生座標(点)にしか距離を見ておらず、T字パーツの存在が
    透明になっていた。

    postにarm_dirがあれば、支柱中心を通りarm_dir方向に±POST_PIVOT_
    ARM_HALF_LENGTH_CM伸びる線分(T字パーツ)全体と線分abの最短距離
    (geo.segment_segment_distance)を、なければ従来通り点との距離
    (geo.point_segment_distance)を返す。

    2026-09-17追記: この関数が返す値は、_try_extendの二分探索や長方形
    オフセット計算など、経路「構築」側でも実際の幾何学的距離として
    使われている。一時的に「T字パーツなら結果からPOST_ARM_STRAIGHT_
    MARGIN_CM分差し引く」形で実装したところ、その値が真の距離でなく
    なったことで二分探索・反復修正ロジックが破綻し、一部シード(例:
    23758)でwaypointsが異常に増殖し旋回違反が21件に達するなど、
    経路構築そのものが崩壊する副作用が確認された。そのため、この関数
    自体は常に「真の幾何学的距離」を返すことに戻し、実機マージンの
    上乗せは呼び出し側の閾値比較(_straight_clearance_threshold)でのみ
    行う方式に変更した。
    """
    arm_dir = getattr(post, "arm_dir", (0.0, 0.0))
    if arm_dir[0] or arm_dir[1]:
        half = config.POST_PIVOT_ARM_HALF_LENGTH_CM
        seg_a = geo.sub(post, geo.scale(arm_dir, half))
        seg_b = geo.add(post, geo.scale(arm_dir, half))
        return geo.segment_segment_distance(a, b, seg_a, seg_b)
    return geo.point_segment_distance(post, a, b)


def _seg_hits_rect(a, b, rect):
    """線分abが長方形rect=(xmin, xmax, ymin, ymax)の内部に入るか。縁に触れるだけ(縁に沿う場合を含む)は入らない。"""
    eps = 1e-6
    x0, x1, y0, y1 = rect[0] + eps, rect[1] - eps, rect[2] + eps, rect[3] - eps
    dx, dy = b[0] - a[0], b[1] - a[1]
    t0, t1 = 0.0, 1.0
    for p_, q_ in ((-dx, a[0] - x0), (dx, x1 - a[0]), (-dy, a[1] - y0), (dy, y1 - a[1])):
        if p_ == 0.0:
            if q_ < 0.0:
                return False
        else:
            t = q_ / p_
            if p_ < 0.0:
                if t > t1:
                    return False
                if t > t0:
                    t0 = t
            else:
                if t < t0:
                    return False
                if t < t1:
                    t1 = t
    return True


def _keepout_hit(a, b):
    """線分abが、進入禁止エリア(config.KEEP_OUT_RECTS_CM)のどれかの内部に入るか。"""
    return any(_seg_hits_rect(a, b, rect) for rect in config.KEEP_OUT_RECTS_CM)


def _candidate_keepout_safe(r):
    """候補r=[(点, アンカー), ...]のどの区間も、進入禁止エリアに入らないか。"""
    pts = [pt for pt, _ in r]
    return not any(_keepout_hit(pts[i], pts[i + 1]) for i in range(len(pts) - 1))


def verify_keepout(waypoints):
    """経路が進入禁止エリアに入る区間を返す。戻り値: [(index, 始点, 終点), ...] (空なら全て安全)。"""
    return [(i, waypoints[i], waypoints[i + 1]) for i in range(len(waypoints) - 1)
            if _keepout_hit(waypoints[i], waypoints[i + 1])]


def _segment_body_clearance(a, b, post):
    """線分a->b(直進区間、旋回軸=タイヤ中心線の軌跡)を実際に車体が
    通過するときの、車体footprint(前端ROBOT_FRONT_OVERHANG_CM・後端
    ROBOT_REAR_OVERHANG_CM・半幅ROBOT_HALF_WIDTH_CMの長方形)とpost
    (T字パーツ込み)との最短距離。

    2026-09-18: 「entry候補同士でどちらがより安全か」を比較する際、
    _segment_post_distance(旋回軸=タイヤ中心線基準)の値だけで比較すると、
    斜めの区間で実際より安全に見えてしまうことが分かった(seed=9196925:
    旋回軸基準では12.3cmで一見余裕があった斜めの区間が、車体の角まで
    含めると実際には0.3cmしかなく、旋回軸基準では8.5cmとやや近いだけ
    だった軸沿いの別候補(車体footprint基準では1.65cm)の方が実際には
    安全だった)。直進(旋回を伴わない)区間では、車体は常にa->b方向を
    向いたまま平行移動するため、進行方向に垂直な半幅ぶんのオフセットと、
    前後の張り出し(オーバーハング)ぶんの延長を持つ長方形として車体を
    近似できる。この関数は「候補同士の安全性比較」用の補助であり、
    STRAIGHT_CLEARANCE_CM自体(旋回軸基準の物理的な最小距離)の判定
    ロジックは変更しない。
    """
    d = geo.sub(b, a)
    d_len = geo.norm(d)
    if d_len < 1e-9:
        return _segment_post_distance(a, b, post)
    d_unit = geo.scale(d, 1.0 / d_len)
    n_unit = (-d_unit[1], d_unit[0])
    half_w = config.ROBOT_HALF_WIDTH_CM
    # 2026-09-20: 「車体の前方が当たらなければ問題ない、後方は考慮不要」との指示で、
    # T字パーツとのクリアランス評価では車体の後端を見ない(後端の張り出しを
    # config.ROBOT_REAR_OVERHANG_FOR_CLEARANCE_CM、既定0cmにする。旋回安全性や
    # 直進のSTRAIGHT_CLEARANCE_CM判定側の後端の扱いは変えない)。
    rear = geo.sub(a, geo.scale(d_unit, config.ROBOT_REAR_OVERHANG_FOR_CLEARANCE_CM))
    front = geo.add(b, geo.scale(d_unit, config.ROBOT_FRONT_OVERHANG_CM))
    corners = [
        geo.add(rear, geo.scale(n_unit, half_w)),
        geo.add(front, geo.scale(n_unit, half_w)),
        geo.sub(front, geo.scale(n_unit, half_w)),
        geo.sub(rear, geo.scale(n_unit, half_w)),
    ]
    arm_dir = getattr(post, "arm_dir", (0.0, 0.0))
    if arm_dir[0] or arm_dir[1]:
        half_arm = config.POST_PIVOT_ARM_HALF_LENGTH_CM
        arm_a = geo.sub(post, geo.scale(arm_dir, half_arm))
        arm_b = geo.add(post, geo.scale(arm_dir, half_arm))
    else:
        arm_a = arm_b = post
    edges = [(corners[0], corners[1]), (corners[1], corners[2]),
             (corners[2], corners[3]), (corners[3], corners[0])]
    return min(geo.segment_segment_distance(e1, e2, arm_a, arm_b) for e1, e2 in edges)


def _label_gate_color(label):
    """"lap{n}-{color}-entry"/"lap{n}-{color}-exit"形式のラベルから
    {color}部分を取り出す(それ以外の形式や None なら None を返す)。
    """
    if label is None:
        return None
    parts = label.split("-")
    if len(parts) == 3 and parts[2] in ("entry", "exit"):
        return parts[1]
    return None


def _straight_clearance_threshold(post, margin_gate_color=None):
    """直進区間の安全判定に使う、post1本分のクリアランス閾値。

    2026-09-17: T字パーツ(arm_dir持ち)は、実機でゲートが人の手で設置
    されるため実際の位置に誤差があり、理論値ギリギリ(閾値7.85cmに対し
    実測6.89cm)では設置誤差を吸収できず実機接触が再発した
    (seed=9392783、黄色ゲートexit直後の直進)。そのため、より広い
    マージンが欲しい。

    ただし、このマージンを「T字パーツとの全ての直進判定」に無条件で
    適用すると、経路構築(8通りのsign探索+_try_extend等の迂回・延長
    ロジック)が閾値のわずかな変化に対して非常に脆弱であることが判明した
    (例: seed=23758は、マージンを+0.5cmにしただけで8通り全部が
    安全な経路を作れなくなった。原因はこのseedの違反箇所ではなく、
    無関係な場所(スタート直後の直進、黄色entry旋回)の副作用)。
    経路構築ロジック全体を作り直すのは大掛かりなため、実機で実際に
    問題が起きているパターンにだけ絞ってマージンを適用する。

    2026-09-18追記: 「ゲート侵入直前の直進(entry直前区間)」でも同種の
    実機接触が発生した(seed=2994807、青ゲート下側の脚のT字パーツ、
    実測クリアランス8.01cmで閾値7.85cmをわずかに超えていただけ
    だった)ため、対象を「-entryラベルの点で終わる区間」にも広げて
    みたが、2つの問題が見つかり撤回した。
      1. 区間に関係する「その区間が退出/侵入するゲートの色」を問わず
         一律マージンを掛けると、無関係な別ゲートの支柱にまで厳しい
         閾値が適用され、既存の迂回ロジックがその厳しさまで追いつかず
         僅かに閾値を割る違反が残る退行が起きた(stress_testで約20%の
         シードが新規に落ちた)。→ margin_gate_colorに一致するpost.
         colorの支柱にだけ絞ることで一旦解消。
      2. それでも、entry直前区間を対象に含めると、8通りのsign探索の
         優先順位(is_valid最優先、周回成立数はis_validが同点の場合の
         tie-breakでしかない)の下で「違反0件を達成できる候補」が
         大幅に減り、「違反は少ないが3周成立しない」候補が優先して
         選ばれてしまうことが判明した(周回不成立が1/200から38/200
         まで悪化)。周回成立の方が優先度が高いため、entry直前区間は
         対象から外し、exit直後の区間のみに戻した(margin_gate_color
         はexit側のためだけに使っている)。postにcolorがない
         (post.color is None)場合は対象外。
    """
    arm_dir = getattr(post, "arm_dir", (0.0, 0.0))
    post_color = getattr(post, "color", None)
    colors = margin_gate_color if isinstance(margin_gate_color, (set, frozenset, tuple, list)) \
        else ({margin_gate_color} if margin_gate_color is not None else ())
    if post_color is not None and post_color in colors and (arm_dir[0] or arm_dir[1]):
        return STRAIGHT_CLEARANCE_CM + config.POST_ARM_STRAIGHT_MARGIN_CM
    return STRAIGHT_CLEARANCE_CM


def _find_blocking_post(a, b, all_posts, exclude=(), anchor_a_post=None, anchor_b_post=None,
                         margin_gate_color=None):
    """線分abに一番手前(aに近い側)でぶつかる支柱を返す。なければNone。

    2種類の除外方法を使い分けられる(呼び出し元ごとにどちらか一方だけを
    渡す):
      - exclude: 座標が一致する支柱を無条件に除外する集合。a・bがすでに
        特定の支柱のためのオフセット点(手前の侵入ポイント)である場合に、
        その支柱自身を判定から除外するために使う(resolve_segmentの再帰。
        除外しないと、オフセット点自身が「その支柱にぶつかっている」と
        誤判定され続けて無限に再帰してしまう)。
      - anchor_a_post/anchor_b_post: a/bそれぞれの「意図して近づいている」
        支柱を1つだけ、実際にその端点の真近く(_project_paramの生値tで
        t<=0はa側、t>=1はb側)でしか近づいていない場合に限って除外する
        (_resegment_straight_collisionsの第2パスで使う。無条件除外だと、
        その支柱の真上を区間の途中(t=0.4のような場所)で通過してしまう
        ケースまで見逃してしまう)。
    """
    best_post = None
    best_t = None
    for post in all_posts:
        if any(geo.distance(post, ex) < 1e-6 for ex in exclude):
            continue
        d = _segment_post_distance(a, b, post)
        if d >= _straight_clearance_threshold(post, margin_gate_color):
            continue
        t = _project_param(a, b, post)
        if anchor_a_post is not None and geo.distance(post, anchor_a_post) < 1e-6 and t <= 0:
            continue
        if anchor_b_post is not None and geo.distance(post, anchor_b_post) < 1e-6 and t >= 1:
            continue
        if best_t is None or t < best_t:
            best_post, best_t = post, t
    return best_post


def resolve_segment(a, b, all_gates, all_posts, target_gate=None,
                     exclude_a=(), exclude_b=(), depth=0, max_depth=8):
    """a->bを直進で結んだときに支柱にぶつからないよう、必要なら「手前の
    侵入ポイント」を挟んで経路を組み立てる。戻り値: [a, ..., b] の座標リスト。

    target_gateは、bが「そのゲートの侵入ポイント」である場合にそのGateを
    渡す(それ以外、たとえばゴールに向かう場合はNone)。手前の侵入ポイントの
    オフセット量は次のように決める:
      - ぶつかった支柱が属するゲートがtarget_gateと同じ場合
        (=侵入予定のゲート自身の脚にぶつかった場合): bの、そのゲート中心
        からの符号付きオフセット(=前方オーバーハングぶん)を支柱に適用する。
      - それ以外(無関係な別ゲートの脚にぶつかった場合): aの、その脚が
        属するゲート中心からの符号付きオフセット(実際の距離)を支柱に適用する。
      いずれの場合も、「支柱・そのゲート中心・(a または b)・手前の侵入
      ポイント」の4点が長方形になる。

    (補足: 「ゲートに平行な方向へ、反対側の脚を超えるまで延長する」という
    別方式も試したが、200パターンのランダムテストで、延長した点がたまたま
    別ゲートの支柱と正確に重なって行き詰まるケースや、新たな旋回安全性
    違反が発生するケースが見つかり、この長方形オフセット方式より安定性が
    低かったため不採用とした。)

    2026-09-15: exclude_a/exclude_bへのblocking_postの無条件除外を、
    anchor_a_post/anchor_b_post方式(端点の近くでしか除外しない)に変更する
    案を試したが、pre_pointが同じ支柱・同じゲート法線方向から再構築され
    続けて無限に近い再帰(max_depth到達)を起こすケースが見つかり、
    ロールバックした(ユーザー確認済み)。この長方形オフセット方式は、
    「侵入予定ゲート自身の支柱にstartからの直線が別途近すぎる」ケースを
    正しく検出できない既知の制約として残っている。
    """
    if depth > max_depth:
        raise RuntimeError(
            "迂回の再帰が深くなりすぎました(ゲート配置が密集しすぎている可能性があります)"
        )

    exclude = list(exclude_a) + list(exclude_b)
    blocking_post = _find_blocking_post(a, b, all_posts, exclude)
    if blocking_post is None:
        return [(a, None), (b, None)]

    owner_gate = _post_owner_gate(blocking_post, all_gates)
    if owner_gate is target_gate:
        # bが侵入予定のゲート自身のentry(前方オーバーハング7.5cm)の場合、
        # そのまま使うと迂回点が支柱の高さから7.5cmしか離れず、必要な
        # クリアランス(7.85cm)にわずかに届かない(実際に見つかった不具合)。
        # entryへの走行中、この距離自体には意味がない(entry->exit間の
        # 直進が始まる基準点というだけ)ため、迂回点の構築にはexitと同じ
        # 後方オーバーハング(17.5cm、常にクリアランスより大きい)を仮の
        # 距離として使い、実際のentry(b)へは迂回点から改めて短く繋ぐ。
        b_offset = _signed_offset(b, owner_gate)
        offset_sign = 1.0 if b_offset >= 0 else -1.0
        offset = offset_sign * max(abs(b_offset), MIN_ENTRY_AVOID_OFFSET_CM)
    else:
        offset = _signed_offset(a, owner_gate)
    pre_point = geo.add(blocking_post, geo.scale(owner_gate.normal, offset))

    # 手前の侵入ポイントはaとbの間の中継点であり、それ自身は特定のゲートの
    # 侵入ポイントではないので、a側の再帰にはtarget_gateを引き継がない。
    # b側の再帰は、最終目的地がbのまま変わらないのでtarget_gateを引き継ぐ。
    left = resolve_segment(
        a, pre_point, all_gates, all_posts, target_gate=None,
        exclude_a=exclude_a, exclude_b=[blocking_post],
        depth=depth + 1, max_depth=max_depth,
    )
    right = resolve_segment(
        pre_point, b, all_gates, all_posts, target_gate=target_gate,
        exclude_a=[blocking_post], exclude_b=exclude_b,
        depth=depth + 1, max_depth=max_depth,
    )
    # pre_pointがどの支柱・ゲートを根拠に作られたかを記録しておく
    # (あとで、この点での旋回が実際に危険だと判明した場合にだけ、
    # ピンポイントでオフセットを引き延ばして直せるようにするため)。
    return left[:-1] + [(pre_point, (blocking_post, owner_gate))] + right[1:]


def _extend_point_for_straight_clearance(
    waypoints, anchors, idx, all_posts, target_post, target_gate,
    needed_clearance=None, max_extension=200.0, steps=60,
):
    """waypoints[idx]を、target_gate.directionに沿って延長し、その前後の
    区間(waypoints[idx-1]->waypoints[idx]、waypoints[idx]->waypoints[idx+1])が
    どちらもtarget_postからneeded_clearance以上離れるようにする。

    _try_extend(旋回安全性の修正)と同じ「ゲートに平行な方向へ延長する」
    考え方だが、こちらは「点そのものの支柱からの距離」ではなく「点を
    延長した結果できる2つの直進区間が、支柱からどれだけ離れるか」を
    最適化する必要があり、ピタゴラスの定理のような単純な閉じた式には
    ならない(区間のどちらの端も動くため)。そのため、延長量を二分探索で
    求める(1cmずつ様子を見て延長するのではなく、必要な最小延長量に
    直接収束させる)。

    waypoints[idx]より後ろの点の中に、waypoints[idx]の(target_gate.direction
    方向の)座標をそのまま引き継いで作られたもの(=直進で平行移動する区間の
    一方の端点)がある場合、waypoints[idx]だけをずらしてそれを据え置くと、
    本来まっすぐだった区間が斜めになり、旋回角が90度から崩れてしまう。
    そのため、まずはそれも巻き込んで同じ量だけ平行移動させることを試みる。
    ただし、後ろの点を動かすと、その点自身が持つ別のアンカー支柱との
    関係が崩れて新たな衝突を生むことがある(実際に見つかったケース)ため、
    それで解決できない場合はwaypoints[idx]だけを動かす(旋回角が90度から
    ずれることを許容する)方式にフォールバックする。

    成功したらwaypointsを書き換えてTrueを返す。どちらの方式でも直せない
    場合はFalseを返す(waypointsは変更しない)。
    """
    if needed_clearance is None:
        needed_clearance = _straight_clearance_threshold(target_post)
    if not (0 <= idx < len(waypoints)):
        return False
    point = waypoints[idx]
    prev_pt = waypoints[idx - 1] if idx > 0 else None
    next_pt = waypoints[idx + 1] if idx < len(waypoints) - 1 else None

    dir_ref = None
    if prev_pt is not None:
        dir_ref = geo.sub(point, prev_pt)
    elif next_pt is not None:
        dir_ref = geo.sub(next_pt, point)
    if dir_ref is None:
        return False
    sign = 1.0 if geo.dot(dir_ref, target_gate.direction) >= 0 else -1.0

    propagate_indices = []
    old_proj = geo.dot(point, target_gate.direction)
    j = idx + 1
    while j < len(waypoints) and anchors[j] is not None:
        if abs(geo.dot(waypoints[j], target_gate.direction) - old_proj) > 1e-6:
            break
        propagate_indices.append(j)
        j += 1

    for candidate_propagate in (propagate_indices, []):
        if _try_extend_with_propagation(
            waypoints, anchors, idx, point, prev_pt, next_pt, candidate_propagate,
            sign, target_gate, target_post, all_posts, needed_clearance,
            max_extension, steps,
        ):
            return True
    return False


def _try_extend_with_propagation(
    waypoints, anchors, idx, point, prev_pt, next_pt, propagate_indices,
    sign, target_gate, target_post, all_posts, needed_clearance, max_extension, steps,
):
    """_extend_point_for_straight_clearanceの実装本体。
    propagate_indicesに指定した後続の点も、waypoints[idx]と同じ量だけ
    平行移動させた上で、十分な延長量を二分探索する(propagate_indicesが
    空なら、waypoints[idx]だけを動かす従来の挙動になる)。
    """
    next_pt_propagates = next_pt is not None and (idx + 1) in propagate_indices

    def min_dist(ext):
        shift = geo.scale(target_gate.direction, sign * ext)
        new_point = geo.add(point, shift)
        d = float("inf")
        if prev_pt is not None:
            d = min(d, _segment_post_distance(prev_pt, new_point, target_post))
        if next_pt is not None:
            effective_next = geo.add(next_pt, shift) if next_pt_propagates else next_pt
            d = min(d, _segment_post_distance(new_point, effective_next, target_post))
        return d

    if min_dist(max_extension) < needed_clearance:
        return False  # 上限まで伸ばしても届かない

    lo, hi = 0.0, max_extension
    for _ in range(steps):
        mid = (lo + hi) / 2
        if min_dist(mid) >= needed_clearance:
            hi = mid
        else:
            lo = mid
    new_point = geo.add(point, geo.scale(target_gate.direction, sign * hi))
    delta = geo.sub(new_point, point)

    shifted = {idx: new_point}
    for k in propagate_indices:
        shifted[k] = geo.add(waypoints[k], delta)

    def shifted_point(k):
        return shifted.get(k, waypoints[k])

    j_after = idx + len(propagate_indices) + 1
    check_range = [idx] + propagate_indices
    if idx - 1 >= 0:
        check_range.append(idx - 1)
    if j_after < len(waypoints):
        check_range.append(j_after)

    for k in range(min(check_range), max(check_range)):
        a_pt = shifted_point(k)
        b_pt = shifted_point(k + 1)
        anchor_a_post = anchors[k][0] if anchors[k] is not None else None
        anchor_b_post = anchors[k + 1][0] if anchors[k + 1] is not None else None
        for p in all_posts:
            if geo.distance(p, target_post) < 1e-6:
                continue
            d = _segment_post_distance(a_pt, b_pt, p)
            if d >= _straight_clearance_threshold(p):
                continue
            # pがこの区間の端点自身のアンカー支柱で、かつ実際にその端点の
            # 真近く(t<=0 または t>=1)でしか近づいていない場合は、
            # 設計上意図された距離(例: 前方オーバーハング7.5cm)として除外する
            # (_find_blocking_postのanchor_a_post/anchor_b_postと同じ考え方)。そうしないと、
            # 「別の場所を直そうとしたら、たまたま隣にある既知の際どい点との
            # 距離が引っかかって直せない」という不具合が実際に発生した。
            t_raw = _project_param(a_pt, b_pt, p)
            if anchor_a_post is not None and geo.distance(p, anchor_a_post) < 1e-6 and t_raw <= 0:
                continue
            if anchor_b_post is not None and geo.distance(p, anchor_b_post) < 1e-6 and t_raw >= 1:
                continue
            return False

    for k, pt in shifted.items():
        waypoints[k] = pt
    return True


def _resegment_straight_collisions(waypoints, anchors, labels, true_points, all_gates, all_posts,
                                    max_iterations=20):
    """構築済みのwaypoints列を隣接ペアごとに見直し、直進区間の衝突を直す。

    resolve_segment内の再帰では、ある支柱を避けて作った点(手前の侵入
    ポイント)について、その支柱を以後ずっと判定から除外し続けてしまう
    (本来は「作った直後の1回の判定」のためだけに必要な除外のはずが、
    子の再帰にまで引き継がれる)。このため、後で別の支柱を避けるために
    座標の一方(x または y)を引き継いで次の点を作ると、その2点を結ぶ
    直線が最初に除外したはずの支柱の真上を通ってしまうことがあり、
    しかも除外されたままなので検知されない、という不具合が実際に見つかった
    (例: 青ゲートの下の脚、赤ゲートの左脚を、除外されたまま距離0で
    通過するケース)。

    この関数は経路構築が終わった後に、隣接する2点ごとに衝突を検証し
    直す2段階目のパスで、_find_blocking_postをanchor_a_post/anchor_b_post
    付きで使い、「その2点自身のアンカー支柱」であっても、実際にその端点の
    真近く(t<=0 または t>=1)でしか近づいていない場合だけ除外する
    (遠く離れた場所で作られた除外を無条件には引きずらない)。

    waypoints/anchorsだけでなくlabels/true_pointsも同じ位置に挿入して
    そろえる(これらを更新せずwaypointsだけ挿入すると、挿入が起きた
    区間より後ろでlabels[i]がwaypoints[i]とずれてしまう不具合があった)。
    """
    waypoints = list(waypoints)
    anchors = list(anchors)
    labels = list(labels)
    true_points = list(true_points)
    for _ in range(max_iterations):
        changed = False
        i = 0
        while i < len(waypoints) - 1:
            a, b = waypoints[i], waypoints[i + 1]
            anchor_a_post = anchors[i][0] if anchors[i] is not None else None
            anchor_b_post = anchors[i + 1][0] if anchors[i + 1] is not None else None
            # 2026-09-18: entry直前区間も対象に含めていたが、8通りのsign探索の
            # 優先順位(is_valid最優先、周回成立数はis_validが同点の場合の
            # tie-break)の下で「違反0件を達成できる候補」が大幅に減り、
            # 「違反は少ないが3周成立しない」候補が「違反はあるが3周成立
            # する」候補より優先して選ばれてしまい、周回不成立が1/200から
            # 38/200まで悪化した。周回成立の方が優先度が高いため、いったん
            # exit直後の区間のみに戻す(_straight_clearance_threshold参照)。
            margin_gate_color = {
                c for c in (_label_gate_color(labels[i]) if labels[i] is not None and labels[i].endswith("-exit") else None,)
                if c is not None
            }
            blocking_post = _find_blocking_post(
                a, b, all_posts, anchor_a_post=anchor_a_post, anchor_b_post=anchor_b_post,
                margin_gate_color=margin_gate_color,
            )
            if blocking_post is None:
                i += 1
                continue
            owner_gate = _post_owner_gate(blocking_post, all_gates)
            offset = _signed_offset(a, owner_gate)
            pre_point = geo.add(blocking_post, geo.scale(owner_gate.normal, offset))
            if geo.distance(pre_point, a) < 1e-6:
                # aが既にこの支柱自身のオフセット点であるケース(=aのオフセットを
                # 使っても同じ点が再生成されるだけで前進しない)。この場合は
                # 新しい点を挿入するのではなく、a自身をそのゲートに平行な
                # 方向へ延長し、a->bの区間がこの支柱から十分離れるようにする。
                needed_clearance = _straight_clearance_threshold(blocking_post, margin_gate_color)
                if _extend_point_for_straight_clearance(
                    waypoints, anchors, i, all_posts, blocking_post, owner_gate,
                    needed_clearance=needed_clearance,
                ):
                    changed = True
                    continue  # 同じiを再チェック(区間が変わったため)
                i += 1
                continue
            waypoints.insert(i + 1, pre_point)
            anchors.insert(i + 1, (blocking_post, owner_gate))
            labels.insert(i + 1, None)
            true_points.insert(i + 1, None)
            changed = True
            # iはそのまま(挿入したことでa->pre_pointの新しいペアを次に見る)
        if not changed:
            break
    return waypoints, anchors, labels, true_points


def _segment_crosses_gate_leg(a, b, gate):
    """線分abが、gateの脚を結ぶ線分(foot_a-foot_b、支柱2本を結ぶ直線)を
    横切るかどうかを調べる。横切る場合は(交点, 通過方向の符号)を、
    横切らなければNoneを返す。通過方向の符号は、gate.normal方向で
    aからbに向かって正方向へ抜けたら+1、負方向へ抜けたら-1。

    「横切る」の判定は、法線方向で見てaとbが線の反対側にあり、かつ
    交点が脚の区間(foot_a〜foot_b)の範囲内にあることで行う。
    """
    d1 = geo.dot(geo.sub(a, gate.foot_a), gate.normal)
    d2 = geo.dot(geo.sub(b, gate.foot_a), gate.normal)
    if d1 == 0 or d2 == 0 or (d1 > 0) == (d2 > 0):
        return None
    t = d1 / (d1 - d2)
    cross_pt = geo.add(a, geo.scale(geo.sub(b, a), t))
    along = geo.dot(geo.sub(cross_pt, gate.foot_a), gate.direction)
    span = geo.distance(gate.foot_a, gate.foot_b)
    if not (-1e-6 <= along <= span + 1e-6):
        return None
    direction_sign = 1.0 if d2 > d1 else -1.0
    return cross_pt, direction_sign


def _maximal_collinear_runs(points):
    """points(単純な(x,y)のリスト)を、進行方向が変わらない区間ごとに
    分割し、[(start_idx, end_idx), ...]を返す(隣り合う区間はend_idxと
    次のstart_idxを共有する)。距離0の区間(同じ点の重複)はスキップする。

    2026-09-15、複数のゲートのentry/exitが偶然同一直線上に並ぶ配置
    (例: 赤と黄がグリッド上同じ列にあり、赤entryが黄entry-exit間に来る
    ケース)で、「1回の旋回なしの直進で複数ゲートを跨いで正式に通過する」
    経路を認識するために追加。_span_covers_gate_crossingと組み合わせて使う。
    """
    n = len(points)
    runs = []
    i = 0
    while i < n - 1:
        base = geo.sub(points[i + 1], points[i])
        base_len = geo.norm(base)
        if base_len < 1e-9:
            i += 1
            continue
        base_unit = geo.scale(base, 1.0 / base_len)
        k = i + 1
        while k + 1 < n:
            seg = geo.sub(points[k + 1], points[k])
            seg_len = geo.norm(seg)
            if seg_len < 1e-9:
                break
            seg_unit = geo.scale(seg, 1.0 / seg_len)
            if abs(seg_unit[0] - base_unit[0]) > 1e-6 or abs(seg_unit[1] - base_unit[1]) > 1e-6:
                break
            k += 1
        runs.append((i, k))
        i = k
    return runs


def _span_covers_gate_crossing(a_pt, b_pt, gate):
    """直線a_pt->b_ptが、gateの正式なentry->exit(どちらかのsign)を、
    a_ptからb_ptへ向かう向きで内包しているかを判定する。
    a_pt/b_pt自身がentry/exitと一致している必要はなく(それより外側まで
    伸びていてもよい)、entry/exitが同一直線上に、この順番で乗っていれば
    よい。内包していれば(gate, sign)を、していなければNoneを返す。
    """
    for sign in (1.0, -1.0):
        entry, exit_ = gate_entry_exit(gate, sign)
        t_entry = geo.collinear_param(a_pt, b_pt, entry)
        t_exit = geo.collinear_param(a_pt, b_pt, exit_)
        if t_entry is None or t_exit is None:
            continue
        if -1e-6 <= t_entry <= t_exit <= 1 + 1e-6:
            return gate, sign
    return None


def _gate_official_directions(waypoints, labels, all_gates):
    """各ゲートの「公式な」通過方向(entry->exit区間が脚を結ぶ線を
    横切る向き)を返す。dict: color -> direction_sign。
    この方向と同じ向きであれば、公式区間以外がそのゲートの脚を結ぶ線を
    横切っても違反ではない(一方向からの侵入であれば問題ない、という
    ルールのため)。
    """
    directions = {}
    for i in range(len(waypoints) - 1):
        lab_a, lab_b = labels[i], labels[i + 1]
        if lab_a is None or lab_b is None:
            continue
        if not lab_a.endswith("-entry") or not lab_b.endswith("-exit"):
            continue
        prefix_a = lab_a.rsplit("-", 1)[0]
        prefix_b = lab_b.rsplit("-", 1)[0]
        if prefix_a != prefix_b:
            continue
        color = prefix_a.split("-", 1)[1]
        gate = next((g for g in all_gates if g.color == color), None)
        if gate is None:
            continue
        result = _segment_crosses_gate_leg(waypoints[i], waypoints[i + 1], gate)
        if result is not None:
            directions[color] = result[1]
    return directions


def _fix_gate_crossings(waypoints, anchors, labels, true_points, all_gates, max_iterations=20):
    """いずれかのゲートの脚を結ぶ線が、そのゲートの「公式な通過方向」
    (entry->exit区間が横切る向き)と異なる向きで横切られてしまうこと
    (=そのゲートに複数の方向から侵入してしまうこと)を防ぐ。

    同じゲートを、公式な通過(entry->exit)と同じ向きで横切る分には
    問題ない(一方向からのみの侵入であれば良い、というルールのため)。
    実際に、赤・青・黄いずれのゲートも、無関係な区間(別ゲートへ向かう
    途中の迂回など)で脚を結ぶ線を公式な通過とは逆向きに横切って
    しまうケースが見つかった。これが、ゲートの近くでの不要な旋回・
    往復の根本原因になっていた。

    横切ってしまう区間を見つけたら、交点に近い方の脚を「支柱」とみなし、
    その脚の外側(ゲートの区間の外)へ、aと同じ側を保ったまま迂回する
    点を挿入する。
    """
    waypoints = list(waypoints)
    anchors = list(anchors)
    labels = list(labels)
    true_points = list(true_points)

    official_directions = _gate_official_directions(waypoints, labels, all_gates)

    for _ in range(max_iterations):
        changed = False
        i = 0
        while i < len(waypoints) - 1:
            a, b = waypoints[i], waypoints[i + 1]
            blocking_gate = None
            cross_pt = None
            for gate in all_gates:
                result = _segment_crosses_gate_leg(a, b, gate)
                if result is None:
                    continue
                pt, sign = result
                if sign == official_directions.get(gate.color):
                    continue  # 公式な通過方向と同じ向きなので問題ない
                blocking_gate = gate
                cross_pt = pt
                break
            if blocking_gate is None:
                i += 1
                continue

            post = min(blocking_gate.posts(), key=lambda p: geo.distance(p, cross_pt))
            span = geo.distance(blocking_gate.foot_a, blocking_gate.foot_b)
            along_post = geo.dot(geo.sub(post, blocking_gate.foot_a), blocking_gate.direction)
            outside_sign = -1.0 if along_post < span / 2 else 1.0

            # 迂回点は支柱と同じ「ゲートの平面上」(法線方向オフセット0)に置く。
            # aの高さをそのまま引き継ぐと(以前の実装)、迂回点がゲートの
            # 平面から外れてしまい、次の区間(迂回点->b)が同じ場所を
            # 同じ向きでもう一度横切ってしまい、何度繰り返しても
            # 違反が解消しない不具合があった。支柱自体が法線オフセット0の
            # 位置にあるため、脚の方向にずらすだけで済む。
            # ずらす量は必要クリアランスちょうど(STRAIGHT_CLEARANCE_CM)では
            # なくMIN_ENTRY_AVOID_OFFSET_CM(exitと同じ後方オーバーハング分)
            # を使う。ちょうどの量だと、前後の区間との干渉でわずかに
            # (1cm未満)クリアランスを割り込むケースが実際にあった
            # (entry回避と同じ理由。押し出す距離を伸ばせば解消する)。
            pre_point = geo.add(
                post, geo.scale(blocking_gate.direction, outside_sign * MIN_ENTRY_AVOID_OFFSET_CM)
            )
            if geo.distance(pre_point, a) < 1e-6 or geo.distance(pre_point, b) < 1e-6:
                # 退避できない(同じ点になってしまう)場合は諦めてこの区間を
                # 残す(後続の検証で気付けるようにする)
                i += 1
                continue

            waypoints.insert(i + 1, pre_point)
            anchors.insert(i + 1, (post, blocking_gate))
            labels.insert(i + 1, None)
            true_points.insert(i + 1, None)
            changed = True
            # iはそのまま(挿入したことでa->pre_pointの新しいペアを次に見る)
        if not changed:
            break
    return waypoints, anchors, labels, true_points


def verify_gate_crossing_directions(waypoints, labels, all_gates):
    """各ゲートが、公式な通過方向(entry->exit区間が横切る向き)と異なる
    向きで脚を結ぶ線を横切っていないかを確認する(_fix_gate_crossingsで
    解決しきれなかった場合の検証用)。同じ向きでの複数回の通過は違反
    ではない。戻り値: 違反のリスト[(index, gate_color, cross_pt), ...]。
    """
    official_directions = _gate_official_directions(waypoints, labels, all_gates)

    violations = []
    for i in range(len(waypoints) - 1):
        a, b = waypoints[i], waypoints[i + 1]
        for gate in all_gates:
            result = _segment_crosses_gate_leg(a, b, gate)
            if result is None:
                continue
            cross_pt, sign = result
            if sign == official_directions.get(gate.color):
                continue
            violations.append((i, gate.color, cross_pt))
    return violations


def verify_gate_passage_order(waypoints, all_gates, gate_order=None):
    """経路上で発生する「正式な形(そのゲートの中心を垂直に通るentry->exitの
    形、公式ステージの通過だけでなく、経路の途中でたまたま同じ形になる
    非公式な通過も含む)」の通過を全て時系列で洗い出し、競技規約5.17.3の
    ゲート通過順序ルール(赤→青→黄の既定の順番で通過することで1周回が
    成立し、既定の順番以外の通過は進行中の周回をリセットする。ただし
    同色の連続通過は1回とみなす)に従って、実際に何周ぶん成立するかを
    シミュレートする。

    2026-09-15に発覚した不具合: resolve_segment/_resolve_via_other_gateは
    「物理的に安全で、かつそのゲート自身の正式なentry/exitの形であること」
    しかチェックしておらず、「今どの周回のどの順番を消化中か」を見ていない。
    そのため、例えば青ゲート通過後(次は黄色を期待している状態)に、
    無関係な赤ゲートを(他ゲート経由の迂回、あるいはたまたまの直線が)
    正式な形で通過してしまうことがあり、この場合競技規約上その周回の
    進行がリセットされてしまう。実際に600件中39件で、これが原因で
    3周とも1周も成立しない(!)経路が見つかった。

    経路生成側の迂回ロジック自体を「target_gate/prev_gate以外の正式通過を
    禁止する」ように直す案も試したが、迂回の選択肢が大きく狭まり、
    物理安全性チェックが600件中486件まで後退したため不採用とした
    (_candidate_crossing_safeのコメント参照)。代わりにこの関数で
    「実際に何周成立するか」を検証し、plan_route_with_anchors側の
    8通りの侵入方向探索で、成立周回数が最大になる組み合わせを優先する
    形にしている(迂回ロジック自体は変更していないので、signの組み合わせ
    次第では、この問題が解消しないまま残ることもある)。

    2026-09-15追記: 複数ゲートのentry/exitが偶然同一直線上に並ぶ配置
    (seed=237571: 赤entryが黄entry-exit間に来るケース)では、経路が
    「黄entry -> 赤entry -> 赤exit」のように、黄自身のexit地点を明示的な
    停止点として経由しないまま直進することがある(赤exitへ向かう区間が
    黄exitの座標を内包しているため、黄の通過はそこで暗に完了している)。
    隣接する2点の完全一致だけを見ていた旧実装ではこれを見逃す
    (黄entryと赤entryの間の区間・赤entryと赤exitの間の区間のどちらも、
    黄自身のentry->exitと完全一致しないため)。_maximal_collinear_runsで
    同一直線上の区間をまとめてから_span_covers_gate_crossingで判定する
    ことで、この「内包された通過」も正しく検出する。

    戻り値: 実際に成立した周回数(int)。config.LAPS未満なら要注意。
    """
    if gate_order is None:
        gate_order = config.GATE_ORDER

    sequence = []
    for start, end in _maximal_collinear_runs(waypoints):
        hits = []
        for gate in all_gates:
            hit = _span_covers_gate_crossing(waypoints[start], waypoints[end], gate)
            if hit is None:
                continue
            _, sign = hit
            _, exit_ = gate_entry_exit(gate, sign)
            t_exit = geo.collinear_param(waypoints[start], waypoints[end], exit_)
            hits.append((t_exit, gate.color))
        hits.sort(key=lambda h: h[0])
        sequence.extend(color for _, color in hits)

    expected_idx = 0
    completed_laps = 0
    prev_color = None
    for color in sequence:
        if color == prev_color:
            continue  # 同色の連続通過は1回とみなす(規約5.17.3)
        if color == gate_order[expected_idx]:
            expected_idx += 1
            if expected_idx == len(gate_order):
                completed_laps += 1
                expected_idx = 0
        else:
            # 既定の順番以外の通過: 進行中の周回はリセットする。ただし
            # 今回の通過自体が1番目(赤)なら、そのままそこから再スタートする。
            expected_idx = 1 if color == gate_order[0] else 0
        prev_color = color
    return completed_laps


def verify_straight_clearance(waypoints, all_posts, anchors=None, labels=None):
    """経路上の各直進区間が、支柱に十分な距離(STRAIGHT_CLEARANCE_CM)を
    保っているかを機械的に確認する。anchorsを渡した場合、その区間の
    端点自身のアンカー支柱は、実際にその端点の真近く(t<=0 または t>=1)
    でしか近づいていない場合に限って除外する(意図的に近い点のため)。

    labelsを渡した場合、直進区間の始点がゲートの退出点(ラベルが
    "-exit"で終わる)、または終点がゲートの侵入点(ラベルが"-entry"で
    終わる)であれば、T字パーツとの判定にPOST_ARM_STRAIGHT_MARGIN_CMの
    追加マージンを適用する(_straight_clearance_threshold参照。実機で
    ゲート退出直後の直進(seed=9392783)、ゲート侵入直前の直進
    (seed=2994807)がそれぞれT字パーツに接触した実例への対応。
    labelsを渡さない場合は従来通りマージンなし)。

    以前は「アンカーだから無条件に除外する」という単純な除外をしていたが、
    それだと区間の途中(tが0や1から離れた位置)で実際に支柱に接触している
    ケースまで見逃してしまう不具合があった(_resegment_straight_collisions側の
    _find_blocking_postでは元々t考慮していたが、こちらの検証用関数だけ
    考慮していなかったため、実際には直っていない違反を「0件」と誤って
    報告していた)。

    2026-09-16: 隣接する2区間(waypoints[i]->[i+1]、[i+1]->[i+2])が実は
    旋回なしで完全に同一直線上にある(_maximal_collinear_runs参照)配置
    (seed=5642644/1734262: 別々の支柱を避けるために順に作られた2つの
    迂回点が、たまたま同じ直線上に並んでしまうケース)では、各区間を
    個別に見ると、支柱への最接近点がその区間の端点(=直線上の途中の
    旋回しない点)にちょうど一致し、そこがどちらか一方の区間の
    アンカーと一致するため、両方の区間で「端点だから除外」判定が
    成立してしまい、実際には旋回せず直進で支柱のすぐそばを通過して
    いるにもかかわらず、検出をすり抜けていた(必要7.85cmに対し実際は
    2.6cmしかない例が2件見つかった)。そのため、個々の区間ではなく
    _maximal_collinear_runsでまとめた直進区間全体に対して支柱との距離を
    確認する(直線の内部にある人工的な分割点は、旋回を伴わない限り
    実際の進路には影響しないため)。アンカー除外も、その直進区間全体の
    始点・終点(=実際に旋回する点)のアンカーだけを見る。

    戻り値: 違反のリスト[(index, post, dist), ...] (空なら全て安全。
    indexは違反が見つかった直進区間の開始インデックス)。
    """
    violations = []
    for start, end in _maximal_collinear_runs(waypoints):
        run_a, run_b = waypoints[start], waypoints[end]
        anchor_a_post = anchors[start][0] if anchors is not None and anchors[start] is not None else None
        anchor_b_post = anchors[end][0] if anchors is not None and anchors[end] is not None else None
        # 2026-09-18: entry直前区間の対象は撤回(_resegment_straight_
        # collisions側のコメント参照。周回不成立が1/200->38/200まで
        # 悪化したため)。exit直後の区間のみを対象にする。
        margin_gate_color = set()
        if labels is not None:
            if labels[start] is not None and labels[start].endswith("-exit"):
                c = _label_gate_color(labels[start])
                if c is not None:
                    margin_gate_color.add(c)
        for post in all_posts:
            d = _segment_post_distance(run_a, run_b, post)
            if d >= _straight_clearance_threshold(post, margin_gate_color) - 1e-9:
                continue
            t_raw = _project_param(run_a, run_b, post)
            if anchor_a_post is not None and geo.distance(post, anchor_a_post) < 1e-6 and t_raw <= 0:
                continue
            if anchor_b_post is not None and geo.distance(post, anchor_b_post) < 1e-6 and t_raw >= 1:
                continue
            violations.append((start, post, d))
    return violations


def _try_extend(waypoints, idx, anchors, all_posts):
    """waypoints[idx]が「手前の侵入ポイント」(anchors[idx]が(post, gate))で
    あることを前提に、そこでの旋回が安全になるよう、ゲートに平行な方向へ
    延長を試みる。成功したらwaypoints[idx]を書き換えてTrueを返す。
    失敗(anchorがない/延長すると別の支柱にぶつかる/既に十分離れている)
    したらFalseを返す(waypointsは変更しない)。

    この点は「支柱から、そのゲートの法線方向にオフセットした点」として
    作られているため、前後どちらかの区間は必ずそのゲートに平行になっている
    (元になった点(aまたはb)からこの点への直線が、常にゲートに平行になる
    ため)。危険な場合は、法線方向にずらす(垂直距離を変える)のではなく、
    その平行な区間をそのまま延長する形で、支柱からの直線距離が旋回に
    必要な距離(config.POST_AVOID_RADIUS_CM)に達するまで直進を伸ばす
    (法線方向にずらす方式だと、無関係な別の支柱に新たに近づいてしまい
    別の衝突を生む不具合が実際に発生したため、この方式に変更した)。
    """
    if not (0 < idx < len(waypoints) - 1):
        return False
    anchor = anchors[idx]
    if anchor is None:
        return False
    post, gate = anchor
    point = waypoints[idx]
    prev_pt = waypoints[idx - 1]
    next_pt = waypoints[idx + 1]

    perp = _signed_offset(point, gate)
    if abs(perp) >= config.POST_AVOID_RADIUS_CM:
        return False  # すでに十分な距離なのに違反 = この方法では直せない

    dir_prev = geo.sub(point, prev_pt)
    dir_next = geo.sub(next_pt, point)
    # ゲートの法線方向成分が小さい(=ゲートに平行に近い)方を採用する
    use_prev = abs(geo.dot(dir_prev, gate.normal)) <= abs(geo.dot(dir_next, gate.normal))
    travel_dir = dir_prev if use_prev else dir_next
    preferred_sign = 1.0 if geo.dot(travel_dir, gate.direction) >= 0 else -1.0

    extra = math.sqrt(config.POST_AVOID_RADIUS_CM ** 2 - perp ** 2)

    # waypoints[idx]の前後には、waypoints[idx]の(gate.direction方向の)
    # 座標をそのまま引き継いで作られた点(=直進で平行移動する区間の
    # 一方の端点)がありうる。waypoints[idx]だけをずらしてそれを据え置くと、
    # 本来まっすぐだった区間が斜めになり、そちらで新たな(あるいはより
    # 悪化した)旋回危険を生む不具合が実際にあった。そのため、まずは
    # それも巻き込んで同じ量だけ平行移動させることを試み、それで
    # 解決できない場合だけwaypoints[idx]単独の移動にフォールバックする。
    old_proj = geo.dot(point, gate.direction)
    propagate_after = []
    j = idx + 1
    while j < len(waypoints) and anchors[j] is not None:
        if abs(geo.dot(waypoints[j], gate.direction) - old_proj) > 1e-6:
            break
        propagate_after.append(j)
        j += 1
    propagate_before = []
    k = idx - 1
    while k >= 0 and anchors[k] is not None:
        if abs(geo.dot(waypoints[k], gate.direction) - old_proj) > 1e-6:
            break
        propagate_before.append(k)
        k -= 1

    # travel_dirから決まる「自然な」延長方向をまず試すが、それがたまたま
    # 同じゲートの反対側の脚に近づく向きだったために失敗することが実際に
    # あった(延長すると別の支柱にぶつかる、と正しく判定されて諦めてしまう)。
    # その場合、逆方向(ゲートの反対側)への延長も試す。
    for sign in (preferred_sign, -preferred_sign):
        new_point = geo.add(point, geo.scale(gate.direction, sign * extra))
        delta = geo.sub(new_point, point)

        for candidates in (propagate_before + propagate_after, []):
            shifted = {idx: new_point}
            for c in candidates:
                shifted[c] = geo.add(waypoints[c], delta)

            def shifted_point(k, shifted=shifted):
                return shifted.get(k, waypoints[k])

            affected = sorted(shifted.keys())
            lo = affected[0] - 1 if affected[0] - 1 >= 0 else affected[0]
            hi = affected[-1] + 1 if affected[-1] + 1 < len(waypoints) else affected[-1]

            # anchor自身の支柱(post)は、平行移動している区間の「内部」
            # (両端とも今回shiftした点である区間)に限り判定から除外する。
            # 理由: ゲートに平行な直進区間は支柱からの垂直距離が常にperpの
            # ままなので、perpがクリアランスの最小値(車体半幅+支柱半径)を
            # わずかに下回るケースで、元々問題なかったはずの区間まで
            # 「ブロックされた」と誤判定されるのを防ぐため。
            # ただし固定された隣接点(shiftしていないprev_pt/next_pt)との
            # 「境界」区間は、shiftによって全く新しい角度でanchor自身の支柱に
            # 接近しうるため、この除外の対象に含めてはいけない(実際に、
            # 移動後の境界区間がanchor自身の支柱に7.577cm(必要7.85cm)まで
            # 接近する新たな違反を、この除外のせいで見逃していた不具合が
            # あった)。
            newly_blocked = False
            for m in range(lo, hi):
                a_pt, b_pt = shifted_point(m), shifted_point(m + 1)
                interior = m in shifted and (m + 1) in shifted
                for p in all_posts:
                    if interior and geo.distance(p, post) < 1e-6:
                        continue
                    if geo.segment_blocked_by_circle(a_pt, b_pt, p, STRAIGHT_CLEARANCE_CM):
                        newly_blocked = True
                        break
                if newly_blocked:
                    break
            if newly_blocked:
                continue  # 延長すると別の支柱にぶつかるので、この案は諦める

            for k2, pt in shifted.items():
                waypoints[k2] = pt
            return True

    return False


def _fix_pivot_violations(waypoints, anchors, all_posts, labels=None, max_iterations=5):
    """verify_pivot_safetyで検出された違反を、可能な範囲で自動的に直す。

    違反が起きた地点自身が「手前の侵入ポイント」(可動点)なら、そこを
    _try_extendで直接延長する。違反が起きた地点がstart/goal/ゲート自身の
    entry/exitのような固定点(anchorを持たない)の場合、その点自身は
    動かせないので、代わりに隣接する2点(1つ前・1つ後)のうち可動点の方を
    延長する(隣の可動点をゲートに平行な方向へ延長しても、固定点への
    到着/出発方向そのものは変わらない場合がある一方、延長によって
    固定点までの区間の向きが変わり、固定点での旋回角が緩和されることが
    ある。実際にstart地点の旋回違反は、隣の手前の侵入ポイントを延長した
    結果として解消した)。
    どちらの方法でも直せない違反は、元の位置のまま残る(呼び出し元で
    verify_pivot_safetyを改めて実行すれば確認できる)。

    2026-09-18追記: _try_extendは「延長した結果、別の支柱に新たにぶつから
    ないか」しか確認しておらず、「延長した結果、直進クリアランス側で
    新たな違反を生んでいないか」は一切確認していなかった。これが原因で、
    ある違反を直そうとして無関係な隣接点を大きく動かし、その隣接点が
    別の区間で新たな直進クリアランス違反を生む(=元々安全だった経路を
    かえって危険にする)ケースが実際に見つかった(seed=126705: entry
    直前の直進が8.44cmから7.73cmまで悪化)。

    当初は「_try_extendを1回呼ぶたびに直進クリアランスの違反件数を
    比較し、悪化していれば取り消す」形で実装したが、これでは防げない
    ケースがあった。1回あたりの移動量は小さく直進クリアランス違反を
    即座には生まないが、旋回違反自体が完全には解消されないまま
    max_iterations回のループで同じ点が毎回少しずつ同じ方向へ押し出され
    続け、結果的に100cm以上移動してしまう(=じわじわとした累積的な
    発散)ケースがあったため。そのため、チェックは「1回ごと」ではなく
    「この関数全体を通しての正味の結果」で行う。

    さらに、「旋回違反が解消しきれなかった場合に限り取り消す」形も
    試したが、これも不十分だった: 上記の発散パターンは、押し出しを
    続けた末に最終的には旋回違反を完全に解消してしまう(だからこそ
    ループが止まらずに発散する)ため、「旋回違反が残っているか」を
    条件にすると、まさに直したいこのケースを取りこぼしてしまう。

    さらに、「直進クリアランス違反の件数が開始時より増えたか」で
    判定する形も試したが、これも不十分だった: 発散前後で件数(1件)が
    たまたま同じまま、違反していた支柱・距離だけが入れ替わる
    (7.73cm→4.00cmのように、既存の違反がより悪化する)ケースが
    あったため、件数だけでは検知できなかった。そのため、比較は件数
    ではなく「その時点で最も厳しい(最小の)直進クリアランス距離」で
    行う: 開始時点の最小クリアランス距離を記録しておき、ループを終えた
    時点でそれより悪化していれば(違反が新たに増えた場合も、既存の
    違反がより深く食い込んだ場合も、どちらもこの値が悪化する形で
    捕捉できる)、この関数による変更を全て取り消し、開始時点の
    waypointsをそのまま返す(=直せなかった旋回違反は残る可能性が
    あるが、直進クリアランス側を余計に悪化させることはない。旋回違反が
    残った場合は、8通りのsign探索の中で他の候補と比較され、違反件数の
    少ない候補が選ばれる形で従来通り扱われる)。labelsを渡さない場合は
    従来通りチェックなしで動作する。
    """
    waypoints = list(waypoints)
    original_snapshot = list(waypoints)

    def worst_clearance(wp):
        violations = verify_straight_clearance(wp, all_posts, anchors, labels)
        if not violations:
            return float("inf")
        return min(v[2] for v in violations)

    original_worst = worst_clearance(waypoints) if labels is not None else None

    for _ in range(max_iterations):
        violations = verify_pivot_safety(waypoints, all_posts)
        if not violations:
            break
        made_progress = False
        for i, _point, _turn in violations:
            if _try_extend(waypoints, i, anchors, all_posts):
                made_progress = True
                continue
            # 自分自身は固定点(anchorなし)か、延長できなかった。
            # 隣接する2点のうち、可動点(anchorあり)の方を試す。
            if _try_extend(waypoints, i - 1, anchors, all_posts):
                made_progress = True
                continue
            if _try_extend(waypoints, i + 1, anchors, all_posts):
                made_progress = True
                continue
        if not made_progress:
            break

    if labels is not None:
        new_worst = worst_clearance(waypoints)
        if new_worst < original_worst - 1e-9:
            return original_snapshot

    return waypoints


def plan_route(gates_by_color, laps=None):
    """ルールベースで全waypointsを構築する。

    戻り値: (waypoints, total_dist_cm, labels, true_points)
    既存パイプライン(commands.py, generate_diagram.py)とインターフェースを
    揃えるため、旧plan_path_detailed()と同じ形式で返す。
    entry/exitは常にゲート中心を通る「本当の」接触点そのものなので、
    true_pointsはwaypointsと同じ値になる。
    laps: 周回数(省略時はconfig.LAPS)。plan_route_with_anchors参照。
    """
    waypoints, anchors, labels, true_points = plan_route_with_anchors(gates_by_color, laps)
    total_dist_cm = sum(
        geo.distance(waypoints[i], waypoints[i + 1]) for i in range(len(waypoints) - 1)
    )
    return waypoints, total_dist_cm, labels, true_points


def _segment_crosses_any_gate_leg(a, b, all_gates):
    """線分abが、all_gatesのいずれかのゲートの脚を結ぶ線を横切るかどうか。"""
    return any(_segment_crosses_gate_leg(a, b, gate) is not None for gate in all_gates)


def _try_parallel_extension_axis(a, b, all_gates, all_posts, target_gate, axis,
                                  max_extension=200.0, step=2.0):
    """_resolve_segment_via_parallel_extensionの、軸1本ぶんの実装本体。
    見つからなければ(fully_clear, fallback)の(None, None)を返す。
    """
    sign = 1.0 if geo.dot(axis, geo.sub(b, a)) >= 0 else -1.0
    b_offset_sign = None
    if target_gate is not None:
        b_offset_sign = 1.0 if _signed_offset(b, target_gate) >= 0 else -1.0

    fallback = None
    t = step
    while t <= max_extension:
        candidate = geo.add(a, geo.scale(axis, sign * t))
        same_side = (
            b_offset_sign is None
            or (1.0 if _signed_offset(candidate, target_gate) >= 0 else -1.0) == b_offset_sign
        )
        # a->candidate(軸方向にまっすぐ延長する区間)が、途中でどこかの
        # ゲートの脚を斜めに横切っていないかも確認する。軸方向への延長は
        # 通常どの支柱からも一定の距離を保つが、延長先がたまたま別ゲートの
        # 脚をまたぐ配置では、支柱への距離だけでは気づけない(実際に見つかった
        # 不具合: 黄ゲートexit->ゴールの延長候補が、支柱には十分離れていても
        # 青ゲートの脚を逆方向に横切ってしまい、それが後段のチェックで
        # 却下されて、より遠回りな迂回に頼らざるを得なくなっていた)。
        if same_side and all(geo.distance(candidate, p) >= STRAIGHT_CLEARANCE_CM for p in all_posts) \
                and not _segment_crosses_any_gate_leg(a, candidate, all_gates):
            # 中継点にはアンカー(最も近い支柱とそのゲート)を付与しておく。
            # これがないと、後段の_fix_pivot_violations(_try_extend)が
            # 「どの方向へ押し出せば安全になるか」を判断できず、旋回安全性
            # 違反が見つかっても一切修正されないまま残ってしまう
            # (実際に見つかった不具合: アンカーなしの中継点でpivot_turn_safe
            # がFalseになっても、_try_extendがanchor is Noneで即座に諦めていた)。
            nearest_post = min(all_posts, key=lambda p: geo.distance(candidate, p))
            anchor = (nearest_post, _post_owner_gate(nearest_post, all_gates))
            if (_find_blocking_post(candidate, b, all_posts, []) is None
                    and not _segment_crosses_any_gate_leg(candidate, b, all_gates)):
                return [(a, None), (candidate, anchor), (b, None)], None
            if fallback is None:
                try:
                    tail = resolve_segment(candidate, b, all_gates, all_posts, target_gate=target_gate)
                    fallback = [(a, None), (candidate, anchor)] + tail[1:]
                except RuntimeError:
                    pass
        t += step
    return None, fallback


def _resolve_segment_via_parallel_extension(a, b, all_gates, all_posts, target_gate, from_gate,
                                             max_extension=200.0, step=2.0):
    """resolve_segment(a, b, ...)が無限再帰(RuntimeError)で失敗した場合の
    フォールバック。

    from_gateが分かっていれば(=直前に通過したゲートがある)、aをその脚の
    方向(from_gate.direction、bに近づく向き)に沿って少しずつ伸ばし、
    その延長点からbへの直線がどの支柱にもぶつからなくなる場所を探す。
    from_gateの脚から真っ直ぐ離れる動きなので、from_gate自身の支柱に
    新たにぶつかることはない。

    from_gateが分からない場合(=経路の最初の区間で、直前に通過した
    ゲートがない場合)は、東西・南北どちらの軸でも安全な迂回が作れる
    可能性があるため、両方の軸を試して見つかった方(見つかれば実際の
    距離が短い方)を使う。

    (実際に見つかったケース: 別ゲートの支柱を避けるための「長方形
    オフセット」の迂回点が、たまたま別の1つのゲートの支柱と同じ座標に
    一致してしまい、その支柱を避けようとすると最初の支柱の迂回点に
    戻ってしまう、という無限ループが発生した。座標が偶然grid上で
    揃っている配置でのみ起きる、rectangle offset方式の弱点。)
    """
    if from_gate is not None:
        axes = [from_gate.direction]
    else:
        axes = [(1.0, 0.0), (0.0, 1.0)]

    fully_clear_candidates = []
    fallback_candidates = []
    for axis in axes:
        fully_clear, fallback = _try_parallel_extension_axis(
            a, b, all_gates, all_posts, target_gate, axis,
            max_extension=max_extension, step=step,
        )
        if fully_clear is not None:
            fully_clear_candidates.append(fully_clear)
        if fallback is not None:
            fallback_candidates.append(fallback)

    if fully_clear_candidates:
        return min(fully_clear_candidates, key=_segment_result_length)
    if fallback_candidates:
        return min(fallback_candidates, key=_segment_result_length)
    raise RuntimeError("並行移動による迂回でも経路を見つけられませんでした")


def _segment_result_length(result):
    return sum(geo.distance(result[i][0], result[i + 1][0]) for i in range(len(result) - 1))


def _departure_pivot_safe(result, prev_gate, gate_signs, all_posts):
    """resultの最初の点aが直前に通過したゲート(prev_gate)のexitである
    場合、そこから抜け出す(このセグメントの最初の区間へ向かう)旋回が
    安全かどうかを判定する。prev_gateやgate_signsが分からない場合、
    区間が短すぎる場合は常にTrue(判定対象外)。

    _entry_arrival_pivot_safeが「到着時の旋回」だけを見ていたのに対し、
    こちらは「出発時の旋回」を見る。実際に見つかった不具合: 直線候補
    自体は支柱に一切ぶつからず、entry到達時の旋回も(target_gateがない
    ため)判定対象外だったが、出発点(直前のゲートのexit)での旋回が
    -120.5度という危険な角度になっていたのに、どのチェックにも
    引っかからず採用されてしまっていた。
    """
    if prev_gate is None or gate_signs is None or len(result) < 2:
        return True
    sign = gate_signs.get(id(prev_gate))
    if sign is None:
        return True
    travel_dir = geo.scale(prev_gate.normal, -sign)
    a = result[0][0]
    next_pt = result[1][0]
    heading_in = math.degrees(math.atan2(travel_dir[1], travel_dir[0]))
    heading_out = math.degrees(math.atan2(next_pt[1] - a[1], next_pt[0] - a[0]))
    return pivot_turn_safe(a, heading_in, heading_out, all_posts)


def _entry_arrival_pivot_safe(result, target_gate, all_posts):
    """resultの最後の点bが侵入予定のゲート(target_gate)のentryである
    場合、そこに到達する直前の区間の向きから、entry->exitへ向かうための
    旋回が安全かどうかを判定する。target_gateがNone(ゴールへ向かう
    最終区間)の場合は、config.GOAL_HEADING_DEGへ向くための旋回が
    安全かどうかを判定する(ゴール到着時も所定の向きへ旋回する必要が
    あり、これも支柱への接触リスクがある旋回である点は他のゲートと
    変わらないため)。区間が短すぎる場合は常にTrue(判定対象外)。

    resolve_segmentは支柱への接触だけを見て「直線が空いているか」を
    判定するため、直線自体はどの支柱にもぶつからなくても、entryに
    到達したときの旋回角が非常に大きくなり、近くの支柱に対して旋回
    安全性違反を起こすケースが実際にあった(207.9度の旋回で、
    直線距離としては最短に見える候補が採用されてしまっていた)。
    """
    if len(result) < 2:
        return True
    prev_pt = result[-2][0]
    b = result[-1][0]
    if target_gate is None:
        if not config.GOAL_FINAL_TURN:
            return True  # ゴール到達後の旋回はしないので、旋回安全性を見る必要がない
        heading_out = config.GOAL_HEADING_DEG
    else:
        sign = 1.0 if _signed_offset(b, target_gate) >= 0 else -1.0
        travel_dir = geo.scale(target_gate.normal, -sign)
        heading_out = math.degrees(math.atan2(travel_dir[1], travel_dir[0]))
    heading_in = math.degrees(math.atan2(b[1] - prev_pt[1], b[0] - prev_pt[0]))
    return pivot_turn_safe(b, heading_in, heading_out, all_posts)


def _candidate_internal_pivot_safe(result, all_posts):
    """resultの中間点(最初と最後を除く)それぞれについて、そこでの旋回が
    安全かどうかを確認する。最初の点(a)は前の区間から引き継いだ向きが
    このセグメント単体では分からず、最後の点(b)はentry到達時の旋回として
    _entry_arrival_pivot_safeが別途判定するため、どちらも対象に含めない。

    実際に見つかった不具合: 長方形オフセット方式が作った中継点自体が、
    別ゲートの支柱に近すぎて危険な旋回(-86度)になっているのに、
    entry到達時の旋回だけを見る_entry_arrival_pivot_safeはこれを
    見逃しており、初期構築の時点で既に危険な経路が「安全」と判定されて
    採用されてしまっていた。
    """
    points = [pt for pt, _ in result]
    if len(points) < 3:
        return True
    for i in range(1, len(points) - 1):
        h_in = math.degrees(math.atan2(points[i][1] - points[i - 1][1], points[i][0] - points[i - 1][0]))
        h_out = math.degrees(math.atan2(points[i + 1][1] - points[i][1], points[i + 1][0] - points[i][0]))
        if abs(geo.normalize_deg(h_out - h_in)) < 1e-6:
            continue
        if not pivot_turn_safe(points[i], h_in, h_out, all_posts):
            return False
    return True


def _candidate_straight_clear_safe(result, all_posts):
    """resultの各区間が、resolve_segment/_resolve_segment_via_parallel_extension
    が本来保証すべき直進クリアランスを実際に満たしているかを確認する。

    実際に見つかった不具合: 長方形オフセット方式が「支柱からゲートの法線
    方向にMIN_ENTRY_AVOID_OFFSET_CMだけ離す」ことで迂回点自体は支柱から
    十分離れていても、そこへ到達するまでの直線(始点から迂回点へ向かう
    斜めの区間)が、同じ支柱のすぐそば(7.49cm、必要な7.85cmよりわずかに
    近い)をかすめてしまうケースがあった。迂回点だけを見る安全確認では
    見逃されるため、区間全体を verify_straight_clearance で確認する。
    """
    points = [pt for pt, _ in result]
    anchors = [a for _, a in result]
    return not verify_straight_clearance(points, all_posts, anchors)


def _candidate_crossing_safe(result, all_gates, gate_signs, target_gate=None, prev_gate=None):
    """resultの各区間が、いずれかのゲートの脚を結ぶ線を、そのゲートの
    公式な通過方向(gate_signsで分かっている)と逆向きに横切っていないかを
    確認する。gate_signsがNoneの場合は判定できないので常にTrue。

    実際に見つかった不具合: ある候補区間が単独では直進クリアランス上
    問題なくても、無関係な別ゲートの脚を逆方向に横切ってしまうことが
    あり、それが後段のfix_gate_crossingsで修正される際に、別の新たな
    直進クリアランス違反を生んでいた。候補を選ぶ時点でこれも確認すれば、
    そもそもそのような候補を避けられる。

    さらに、たとえ向きが公式方向と一致していても、この種の「たまたま
    脇道で別ゲートの脚を斜めに横切る」候補自体を一切許可しない。
    ゲートへの侵入は必ずそのゲートの中心を垂直に通る、正式なentry/exitの
    形でなければならない(ユーザーの明示的な要望)。斜めに掠めるだけの
    候補は、支柱からの距離がたまたま閾値を超えていても、個体差や実機の
    誤差を考えると危険なため、機械的に候補から除外し、後段の
    _resolve_via_other_gateフォールバック(そのゲートを正式なentry/exitで
    経由する形)に任せる。

    2026-09-15: target_gate/prev_gate以外の色を正式な形で経由すること自体を
    ここで一律禁止する案も試したが(競技規約5.17.3のゲート通過順序ルール
    対策)、600件中486件まで物理安全性チェックが後退した(近接した
    entry/exit間でほぼ180度旋回を強いられるケースが噴出したため)。
    影響を受けるのは元々600件中39件だけなので、ここは変更せず、
    plan_route_with_anchors側のスコアリングで「ゲート通過順序」を
    別途チェックし、順序が壊れないsignの組み合わせを優先させる形に
    変更した(verify_gate_passage_order参照)。

    2026-09-15追記: 複数ゲートのentry/exitが偶然同一直線上に並ぶ配置
    (例: 赤と黄が同じ列にあり、赤entryが黄entry-exit間に来るケース、
    seed=237571)では、resultの中に「別ゲート(黄)のentry->exitを内包する
    直進区間の途中に、このgate(target)の脚を跨ぐ」形の区間が現れることが
    ある。この場合もその内包しているゲート自身の脚を跨ぐのは正式な通過の
    一部であり、除外してよい(_maximal_collinear_runs/
    _span_covers_gate_crossing参照)。

    さらに、target_gateが分かっている場合(呼び出し元は常にresultの最後の
    点=target_gateのentryとして呼ぶ)、target_gate自身のexit(まだresultには
    含まれない、呼び出し元がこの後すぐ追加する点)を仮想的に延長して
    判定する。上記のケースで、黄のexitがresult自身の範囲(黄entry〜赤entry)
    を超えて、この後に続く赤entry〜赤exitの区間内に来る配置があるため
    (実際にseed=237571で発生)、これを考慮しないと黄の通過を認識できない。
    """
    points = [pt for pt, _ in result]
    check_points = points
    lead = 0
    if prev_gate is not None and gate_signs is not None and points:
        prev_sign = gate_signs.get(id(prev_gate))
        if prev_sign is not None:
            prev_entry, prev_exit = gate_entry_exit(prev_gate, prev_sign)
            if geo.distance(points[0], prev_exit) < 1e-6:
                check_points = [prev_entry] + check_points
                lead = 1
    if target_gate is not None and points:
        b = points[-1]
        for sign in (1.0, -1.0):
            entry, exit_ = gate_entry_exit(target_gate, sign)
            if geo.distance(b, entry) < 1e-6:
                check_points = check_points + [exit_]
                break
    runs = _maximal_collinear_runs(check_points)

    def _gate_covered_at(i, gate):
        i = i + lead
        for start, end in runs:
            if not (start <= i < end):
                continue
            if _span_covers_gate_crossing(check_points[start], check_points[end], gate) is not None:
                return True
        return False

    for i in range(len(points) - 1):
        a_pt, b_pt = points[i], points[i + 1]
        for gate in all_gates:
            cross = _segment_crosses_gate_leg(a_pt, b_pt, gate)
            if cross is None:
                continue
            # このgateの正式なentry->exit区間そのもの(_resolve_via_other_gate
            # が作る、中心を垂直に通る本物の通過)であれば許可する。
            is_official_pair = False
            for sign in (1.0, -1.0):
                entry, exit_ = gate_entry_exit(gate, sign)
                if geo.distance(a_pt, entry) < 1e-6 and geo.distance(b_pt, exit_) < 1e-6:
                    is_official_pair = True
                    break
            if is_official_pair:
                continue
            if _gate_covered_at(i, gate):
                continue
            return False
    return True


def _resolve_via_other_gate(a, b, all_gates, all_posts, target_gate, other_gate, other_sign, prev_gate=None,
                             gate_signs=None):
    """target_gate以外のゲート(other_gate)を、other_signの方向で経由して
    からbへ向かう経路を構築する。

    ゲート同士が近すぎて(例: 上下に隣接していて)、target_gateのentryへ
    直接向かうと近くの支柱のせいでどんな迂回でも旋回安全性を満たせない
    配置が実際にあった(赤と黄が25.7cmしか離れておらず、赤entryが
    その隙間に来るケース)。この場合、無関係な別のゲート(黄)を
    正式な通過順序より前に(ルール上、赤の前に他色を通過するのは
    問題ない)一度くぐっておくと、そこから先の直進がtarget_gateの
    entryへ旋回なしでほぼ一直線に繋がり、安全になることがある。

    前後2本の区間(a->other_entry、other_exit->b)自体にも、
    _resolve_top_level_segmentと同じ安全性判定(entry到達時の旋回・
    中継点の旋回・他ゲートの脚を斜めに横切っていないか)を適用する
    (単純なresolve_segmentだけだと、other_entryの手前でother_gate自身の
    脚を斜めに横切ってしまうケースを見逃していた)。ただし、これ以上
    「別のゲートを経由する」フォールバックへは進まない
    (allow_via_other=Falseで無限の連鎖を防ぐ)。

    2026-09-15追記: bがother_entry->other_exitの直線上(その範囲内)に
    乗っている配置(seed=237571: 赤entryが黄entry-exit間に来るケース)
    では、通常通りother_exitまで進んでからbへ向かうと、bがother_exitより
    手前にあるため後退(最悪ほぼ180度の反転)が必要になってしまう。
    この場合はother_exit自体には立ち寄らず、other_entryからbまで直進する
    候補を優先的に試す(other_gate自身の正式な通過は、bから先で
    other_exitの座標を通過点として内包する形で別途成立する。
    _candidate_crossing_safe/verify_gate_passage_orderのコメント参照)。
    """
    other_entry, other_exit = gate_entry_exit(other_gate, other_sign)

    t_b = geo.collinear_param(other_entry, other_exit, b)
    if t_b is not None and -1e-6 <= t_b <= 1 + 1e-6:
        other_gate_excluded = [g for g in all_gates if g is not other_gate]
        if (_find_blocking_post(other_entry, b, all_posts) is None
                and not _segment_crosses_any_gate_leg(other_entry, b, other_gate_excluded)):
            try:
                to_other = _resolve_top_level_segment(
                    a, other_entry, all_gates, all_posts, other_gate, prev_gate,
                    gate_signs=gate_signs, allow_via_other=False,
                )
                return to_other[:-1] + [(other_entry, None), (b, None)]
            except RuntimeError:
                pass

    try:
        to_other = _resolve_top_level_segment(
            a, other_entry, all_gates, all_posts, other_gate, prev_gate,
            gate_signs=gate_signs, allow_via_other=False,
        )
    except RuntimeError:
        return None
    try:
        to_b = _resolve_top_level_segment(
            other_exit, b, all_gates, all_posts, target_gate, other_gate,
            gate_signs=gate_signs, allow_via_other=False,
        )
    except RuntimeError:
        return None
    return to_other[:-1] + [(other_entry, None), (other_exit, None)] + to_b[1:]


def _required_entry_heading_deg(b, target_gate):
    """bへ正式な向きで侵入するために必要な進行方向(度)。
    target_gateがNoneの場合はゴールへの到着なのでGOAL_HEADING_DEGを使う
    (_entry_arrival_pivot_safeと同じ考え方)。
    """
    if target_gate is None:
        return config.GOAL_HEADING_DEG
    sign = 1.0 if _signed_offset(b, target_gate) >= 0 else -1.0
    travel_dir = geo.scale(target_gate.normal, -sign)
    return math.degrees(math.atan2(travel_dir[1], travel_dir[0]))


def _try_repair_candidate_offsets(result, all_posts, max_iterations=3):
    """候補resultが旋回安全性(_candidate_internal_pivot_safe)または
    直進クリアランス(_candidate_straight_clear_safe)のどちらかに違反する
    場合、resultの中継点(anchor付き、_try_extendで動かせる点)を延長
    修正できないか試す。

    2026-09-15: resolve_segment(長方形オフセット)が作る「侵入予定ゲート
    自身の支柱を避ける迂回点」は、直進クリアランス用のMIN_ENTRY_AVOID_
    OFFSET_CM(17.5cm、ROBOT_REAR_OVERHANG_CM)だけ支柱から離した点だが、
    これだけでは次の2つのケースに対応できないことがある(実際に見つかった
    不具合、いずれもこの迂回点自体は支柱から正確に17.5cm離れている):
      (a) その迂回点自体で大きな旋回が起きる場合、旋回に必要な距離
          (config.POST_AVOID_RADIUS_CM、約21.3cm)には足りない
          (seed=1737994)。
      (b) そこへ向かう直前の区間(a->迂回点)が、迂回点の元になったのと
          同じ支柱に区間の途中で近づきすぎる。resolve_segmentの再帰は
          この迂回点自体を「意図して近い点」として除外するため、この
          近接を見逃す(seed=6781816)。
    _fix_pivot_violations/verify_straight_clearanceは経路全体を組み立てた
    後にしか働かないため、この迂回点は_resolve_top_level_segmentの候補
    選定の時点で「安全でない候補」として丸ごと捨てられてしまい、直す機会
    すら与えられないまま、より遠回りな迂回に頼らざるを得なくなっていた。
    この関数は_fix_pivot_violationsと同じ_try_extendの仕組みを、候補
    選定の時点で(その候補内の全ての可動点に対して)先に試すことで、
    そのような候補を捨てる前に救えないか試みる(_try_extendは支柱からの
    距離を伸ばす操作なので、(a)(b)どちらのケースにも効く)。修正できても
    できなくても、修正を試みた後のresultを返す(呼び出し元で改めて
    _candidate_internal_pivot_safe/_candidate_straight_clear_safeを
    確認すること)。
    """
    points = [pt for pt, _ in result]
    anchors = [a for _, a in result]
    for _ in range(max_iterations):
        current = list(zip(points, anchors))
        if _candidate_internal_pivot_safe(current, all_posts) and _candidate_straight_clear_safe(current, all_posts):
            break
        made_progress = False
        for i in range(1, len(points) - 1):
            if anchors[i] is None:
                continue
            if _try_extend(points, i, anchors, all_posts):
                made_progress = True
        if not made_progress:
            break
    return list(zip(points, anchors))


def _try_midpoint_post_detour(a, b, all_gates, all_posts, target_gate):
    """a->bの直進が支柱P1にぶつかり、resolve_segmentと同じ式でP1を
    避ける迂回点を作っても、その先が別ゲートの支柱P2にぶつかってしまう
    配置(seed=6416989: 青ゲートを避けた迂回点が、たまたま赤ゲートを
    避けようとする計算で青ゲート自身の支柱の座標に戻ってきてしまう)
    では、P1・P2それぞれを個別に避けようとするとうまくいかないことが
    ある(_resolve_segment_via_parallel_extensionのdocstring「座標が
    偶然grid上で揃っている配置でのみ起きる、rectangle offset方式の
    弱点」も参照)。

    この関数は代わりに、P1とP2が共有している座標軸(X座標が一致していれば
    Y軸、Y座標が一致していればX軸。両ゲートの向きが違っても、支柱2点が
    たまたま軸を共有していればよい)のちょうど中間(どちらの支柱からも
    均等な距離)まで直進してから、その位置のまま垂直な方向に
    (_try_parallel_extension_axisと同じ仕組みで)抜け道を探す。
    P1とP2が座標軸を共有していない場合は対象外としてNoneを返す。

    見つかれば[(a,None),(corner1,anchor),...,(b,None)]を、
    見つからなければNoneを返す。
    """
    blocking1 = _find_blocking_post(a, b, all_posts)
    if blocking1 is None:
        return None
    gate1 = _post_owner_gate(blocking1, all_gates)
    if gate1 is None:
        return None

    if gate1 is target_gate:
        b_offset = _signed_offset(b, gate1)
        offset_sign = 1.0 if b_offset >= 0 else -1.0
        offset1 = offset_sign * max(abs(b_offset), MIN_ENTRY_AVOID_OFFSET_CM)
    else:
        offset1 = _signed_offset(a, gate1)
    pre1 = geo.add(blocking1, geo.scale(gate1.normal, offset1))

    blocking2 = _find_blocking_post(pre1, b, all_posts, exclude=[blocking1])
    if blocking2 is None:
        return None
    gate2 = _post_owner_gate(blocking2, all_gates)
    if gate2 is None or gate2 is gate1:
        return None

    # blocking1とblocking2が共有している座標軸を探す(X一致ならY軸、
    # Y一致ならX軸)。ゲート自体の向き(normal/direction)ではなく、
    # 支柱2点の実際の位置関係だけで判定する。
    if abs(blocking1[0] - blocking2[0]) < 1e-6:
        axis = (0.0, 1.0)
        lateral = (1.0, 0.0)
    elif abs(blocking1[1] - blocking2[1]) < 1e-6:
        axis = (1.0, 0.0)
        lateral = (0.0, 1.0)
    else:
        return None

    mid_val = (geo.dot(blocking1, axis) + geo.dot(blocking2, axis)) / 2.0
    a_val = geo.dot(a, axis)
    corner1 = geo.add(a, geo.scale(axis, mid_val - a_val))

    if (_find_blocking_post(a, corner1, all_posts) is not None
            or _segment_crosses_any_gate_leg(a, corner1, all_gates)):
        return None

    fully_clear, fallback = _try_parallel_extension_axis(corner1, b, all_gates, all_posts, target_gate, lateral)
    tail = fully_clear if fully_clear is not None else fallback
    if tail is None:
        return None

    nearest_post = min(all_posts, key=lambda p: geo.distance(corner1, p))
    anchor = (nearest_post, _post_owner_gate(nearest_post, all_gates))
    return [(a, None), (corner1, anchor)] + tail[1:]


def _try_loop_detour(a, b, all_gates, all_posts, target_gate, prev_gate, gate_signs,
                      max_extension=150.0, step=5.0):
    """aを出発する向き(prev_gateからの退出方向)と、bへの正式な侵入に
    必要な向き(_required_entry_heading_deg)が同じ軸上にある(平行、
    向きが同じでも真逆でも)配置(seed=6781816/6199669)向けの、
    探索的な迂回候補。

    L字コーナー(_try_axis_aligned_corner)は「aを通り出発方向の直線」と
    「bを通り侵入方向の直線」の交点を求めるが、この2直線が平行だと交点が
    存在しない(向きが同じ場合も、真逆の場合も同様)。実際に見つかった
    パターンは向きが同じ場合(例: 出発方向も侵入方向も南向きだが、bが
    aよりさらに南、つまり一旦bを通り過ぎてから戻ってくる形が必要)
    だったため、真逆だけでなく同じ向きの平行も対象に含める。

    aから出発方向へ少しずつ距離を伸ばし、その各地点から
    (_try_axis_aligned_cornerと同じ仕組みで)出発方向と垂直な2方向を
    それぞれ試して、bへ安全に到達できる組み合わせが最初に見つかった
    ところで打ち切る。既存のL字コーナーが「1点の計算」なのに対し、
    こちらは「複数の地点を試す」探索になっている。

    departure方向とentry方向が平行でない場合(=既存のL字コーナーで
    間に合うはずの配置)は、対象外としてNoneを返す。
    """
    if prev_gate is None or gate_signs is None:
        return None
    prev_sign = gate_signs.get(id(prev_gate))
    if prev_sign is None:
        return None
    departure_dir = geo.scale(prev_gate.normal, -prev_sign)

    required_heading = _required_entry_heading_deg(b, target_gate)
    rad = math.radians(required_heading)
    entry_axis = (math.cos(rad), math.sin(rad))

    # departureとentryが平行(向きが同じでも真逆でも、内積の絶対値が1に
    # 近い)場合だけを対象にする。垂直に近い配置は既存のL字コーナー
    # (1点の交点計算)で間に合うはず。
    if abs(abs(geo.dot(departure_dir, entry_axis)) - 1.0) > 0.1:
        return None

    # bへ最終的にentry_axis方向で(旋回なしで)侵入できる「足場」P を、
    # bからentry_axisと逆向きに(=侵入方向の手前側に)ずらした位置に探す。
    # 単純にaからdeparture_dir方向へ進むだけでは、bがaから見て
    # departure_dirと逆側にある場合(seed=6199669: 出発は南向きだが
    # bは北側にあり、南へ進むほどbから遠ざかる)に永久に届かないため、
    # 「bの手前側」を基準に足場を探す設計にした。
    def _post_anchor(pt):
        nearest_post = min(all_posts, key=lambda p: geo.distance(pt, p))
        return (nearest_post, _post_owner_gate(nearest_post, all_gates))

    def _segment_ok(p1, p2):
        return (_find_blocking_post(p1, p2, all_posts) is None
                and not _segment_crosses_any_gate_leg(p1, p2, all_gates))

    found = []
    d = step
    while d <= max_extension:
        staging = geo.add(b, geo.scale(entry_axis, -d))

        # aから足場stagingへの経路(直行、またはL字1回)を試す。
        approach_options = [[a, staging]]
        corner_h = (staging[0], a[1])
        corner_v = (a[0], staging[1])
        approach_options.append([a, corner_h, staging])
        approach_options.append([a, corner_v, staging])

        for approach in approach_options:
            if any(not _segment_ok(approach[i], approach[i + 1]) for i in range(len(approach) - 1)):
                continue
            if not _segment_ok(staging, b):
                continue
            candidate = [(a, None)]
            for pt in approach[1:]:
                candidate.append((pt, _post_anchor(pt)))
            candidate.append((b, None))
            if (_candidate_internal_pivot_safe(candidate, all_posts)
                    and _candidate_straight_clear_safe(candidate, all_posts)
                    and _departure_pivot_safe(candidate, prev_gate, gate_signs, all_posts)):
                found.append(candidate)
                break  # このdでは以降の(より遠回りな)approach_optionsは不要

        # departure_dir方向にaから足場corner_vまで直進したあと、bまで斜めに
        # 直進する候補も試す(seed=1641564: 足場からtangent方向・entry_axis
        # 方向の2辺で迂回するより、corner_vから直接bへ斜めに向かう方が
        # 大幅に短くなるケースが実際にあった)。bへの到達時の旋回は
        # ゼロにならないが、それは呼び出し元の_fully_safe内の
        # _entry_arrival_pivot_safeが別途確認する。
        if _segment_ok(a, corner_v) and _segment_ok(corner_v, b):
            candidate = [(a, None), (corner_v, _post_anchor(corner_v)), (b, None)]
            if (_candidate_internal_pivot_safe(candidate, all_posts)
                    and _candidate_straight_clear_safe(candidate, all_posts)
                    and _departure_pivot_safe(candidate, prev_gate, gate_signs, all_posts)):
                found.append(candidate)

        d += step
    if not found:
        return None
    return min(found, key=_segment_result_length)


def _try_extend_through_covered_gate(a, all_gates, all_posts, prev_gate, gate_signs):
    """aが直前に通過したゲート(prev_gate)のexitである場合、prev_gate自身の
    entry->exit区間の中に、別のゲート(other_gate)の正式なentry->exitが
    丸ごと収まっている配置(seed=720696: 黄の真上に赤があり、黄entry->exit
    の直進区間の中に赤entry->exitがすっぽり収まっているケース)では、
    prev_gateのexit(=a)へ向かう直進の中で、すでに正式な形でother_gateも
    通過している(_span_covers_gate_crossing/verify_gate_passage_order
    参照、隣接waypointである必要はなく、直線に収まっていればよい)。

    2026-09-15: これを「常に優先する」形で無条件に適用したところ、赤と黄が
    同じ列にある配置では8通りのsignの組み合わせすべてでこの整列が
    発生してしまい、本来問題なかった組み合わせにまで不要な迂回を
    強制してしまって全体が悪化した(ロールバック済み)。そのため、
    ここでは他の候補(直接goalへ向かう、等)と同列の「候補の一つ」として
    返すだけにとどめ、_resolve_top_level_segment側の安全性・距離比較で、
    実際に有利な場合だけ自然に選ばれるようにする。

    見つかれば([(a, None), (other_entry, None), (other_exit, None)], other_gate)を、
    見つからなければ(None, None)を返す(呼び出し元がother_exitから先の
    区間を別途解決する)。
    """
    if prev_gate is None or gate_signs is None:
        return None, None
    prev_sign = gate_signs.get(id(prev_gate))
    if prev_sign is None:
        return None, None
    prev_entry, prev_exit = gate_entry_exit(prev_gate, prev_sign)
    if geo.distance(prev_exit, a) > 1e-6:
        return None, None
    for other_gate in all_gates:
        if other_gate is prev_gate:
            continue
        for other_sign in (1.0, -1.0):
            other_entry, other_exit = gate_entry_exit(other_gate, other_sign)
            # other_entryがprev_gate自身のentry->exit区間の中に(端点含めて)
            # 乗っており、かつother_exitがそこからさらに同じ向きへ進んだ先に
            # ある(t_exit > t_entry、逆向きのゲートを誤って拾わないため)
            # 場合だけを対象にする。other_exit自体はprev_exitより先に出て
            # いて構わない(そここそがこの関数の存在意義)。
            t_entry = geo.collinear_param(prev_entry, prev_exit, other_entry)
            if t_entry is None or not (-1e-6 <= t_entry <= 1 + 1e-6):
                continue
            t_exit = geo.collinear_param(prev_entry, prev_exit, other_exit)
            if t_exit is None or t_exit <= t_entry + 1e-6:
                continue
            other_gates_excluded = [g for g in all_gates if g is not prev_gate and g is not other_gate]
            if (_find_blocking_post(a, other_exit, all_posts) is not None
                    or _segment_crosses_any_gate_leg(a, other_exit, other_gates_excluded)):
                continue
            # other_entry自体はprev_entry->prev_exitの区間内に収まって
            # いる(=すでに実質的に通過済み)ため、ここで改めて停止点として
            # 経由する必要はない(むしろaより手前にあるため、経由すると
            # その場で反転が必要になってしまう)。aからother_exitへ直進する
            # だけでよく、other_gateの正式な通過は_candidate_crossing_safe/
            # verify_gate_passage_order側が、この直進区間をprev_entryまで
            # 遡って(prev_gate引数経由で)判定することで認識する。
            return [(a, None), (other_exit, None)], other_gate
    return None, None


def _try_axis_aligned_corner(a, b, all_gates, all_posts, target_gate, axis):
    """aからaxis方向にまっすぐ進み、bへの正式な侵入方向(_required_entry_
    heading_deg)の直線とちょうど交わる1点だけで旋回し、そこからbへ直進する
    L字型の候補を試す。

    軸方向延長(_resolve_segment_via_parallel_extension)は「最初に見つかった、
    支柱にぶつからない点」で妥協するため、そこからbへの区間が正式な侵入
    方向とずれた斜めの直線になり、結局entry到達時の旋回で安全性違反を
    起こすことがある(実際に見つかった不具合、seed=23758: スタート直後、
    黄ゲートの上を素通りしてから1回だけ左に90度旋回して赤entryへ向かう
    経路が可能なのに、途中で妥協した点から斜めに向かってしまい、entry
    到達時に黄ゲートの支柱へ旋回で接触しかける経路が採用されていた)。

    この関数はその代わりに、2直線(aを通りaxis方向の直線、bを通り
    正式侵入方向の直線)の交点を直接計算することで、corner->bの旋回が
    ちょうど0度(=旋回不要)になる候補を1つ試す。無効(交点が後ろ向き・
    支柱に近すぎる・どこかのゲートの脚を斜めに横切る)ならNoneを返す。
    """
    required_heading = _required_entry_heading_deg(b, target_gate)
    rad = math.radians(required_heading)
    entry_axis = (math.cos(rad), math.sin(rad))

    denom = axis[0] * entry_axis[1] - axis[1] * entry_axis[0]
    if abs(denom) < 1e-9:
        return None
    dx, dy = b[0] - a[0], b[1] - a[1]
    t = (dx * entry_axis[1] - dy * entry_axis[0]) / denom
    if t <= 1e-6:
        return None
    corner = geo.add(a, geo.scale(axis, t))

    if (_find_blocking_post(a, corner, all_posts) is not None
            or _find_blocking_post(corner, b, all_posts) is not None
            or _segment_crosses_any_gate_leg(a, corner, all_gates)
            or _segment_crosses_any_gate_leg(corner, b, all_gates)):
        return None

    nearest_post = min(all_posts, key=lambda p: geo.distance(corner, p))
    anchor = (nearest_post, _post_owner_gate(nearest_post, all_gates))
    return [(a, None), (corner, anchor), (b, None)]


def _try_shifted_axis_corner(a, b, all_gates, all_posts, target_gate, axis,
                              max_shift=150.0, step=5.0):
    """_try_axis_aligned_cornerと同じ「aを通りaxis方向の直線」と「bを通り
    正式侵入方向の直線」の交点で1回だけ曲がる形を試みるが、_try_axis_
    aligned_cornerは常にaそのものを通る直線しか試せないため、aの座標が
    たまたま無関係なゲートの脚の並び上にあると(例: aのX座標が、別の
    ゲートの脚のX範囲内にある)、その直線が非公式にゲートの脚を横切って
    しまい候補を作れないことがあった(seed=2181252: 赤ゲートexitの
    X座標が、赤・黄ゲート自身の脚のX範囲内にあり、その位置から真北へ
    直進すると両ゲートの脚を非公式に横切ってしまっていた。実際には、
    先にaxisと垂直な方向へ少しだけ離れてから北上すれば、どちらの脚も
    横切らずに済んだ)。

    そのため、まずaをaxisに垂直な方向へstep刻みで少しずつずらし、その
    ずらした点を新たな起点として、_try_axis_aligned_cornerと同じ交点
    計算を試す(a自体はそのまま、a->ずらした点->交点->bという3区間・
    2回旋回の候補になる)。ずらす量が最初に成功した時点で打ち切る
    (=最小限のずらし量を採用する)。axisとentry_axisが平行に近い場合は
    _try_loop_detourの対象のためNoneを返す。
    """
    required_heading = _required_entry_heading_deg(b, target_gate)
    rad = math.radians(required_heading)
    entry_axis = (math.cos(rad), math.sin(rad))
    denom = axis[0] * entry_axis[1] - axis[1] * entry_axis[0]
    if abs(denom) < 1e-9:
        return None  # 平行に近い配置は_try_loop_detourの対象

    perp = (-axis[1], axis[0])

    # 2026-09-20: entry(target_gate)へ向かう候補は、そのゲート自身のT字パーツから車体
    # 前方でconfig.ARM_BODY_CLEARANCE_MIN_CM(3cm)以上離れる最初のずらし量を採用する
    # (seed=7595581: 5cm刻みで最初に安全だったずらし量が、T字パーツまで2.25cmだった)。
    # 見つかったら、1cm刻みで手前に戻って、ARM_BODY_CLEARANCE_TARGET_CM(4cm)を
    # 満たす最小のずらし量を探す。
    need_arm = target_gate is not None and config.ARM_BODY_CLEARANCE_TARGET_CM > 0
    arm_posts = [
        p for p in all_posts
        if need_arm and getattr(p, "color", None) == target_gate.color and (p.arm_dir[0] or p.arm_dir[1])
    ] if need_arm else []

    def arm_clearance(res):
        pts = [pt for pt, _ in res]
        return min(_segment_body_clearance(x, y, p) for x, y in zip(pts, pts[1:]) for p in arm_posts)             if arm_posts else float("inf")

    def make(sign, sv):
        shifted_a = geo.add(a, geo.scale(perp, sign * sv))
        if (_find_blocking_post(a, shifted_a, all_posts) is not None
                or _segment_crosses_any_gate_leg(a, shifted_a, all_gates)):
            return None
        dx, dy = b[0] - shifted_a[0], b[1] - shifted_a[1]
        t = (dx * entry_axis[1] - dy * entry_axis[0]) / denom
        if t <= 1e-6:
            return None
        corner = geo.add(shifted_a, geo.scale(axis, t))
        if (_find_blocking_post(shifted_a, corner, all_posts) is None
                and _find_blocking_post(corner, b, all_posts) is None
                and not _segment_crosses_any_gate_leg(shifted_a, corner, all_gates)
                and not _segment_crosses_any_gate_leg(corner, b, all_gates)):
            nearest_post = min(all_posts, key=lambda p: geo.distance(corner, p))
            anchor = (nearest_post, _post_owner_gate(nearest_post, all_gates))
            return [(a, None), (shifted_a, None), (corner, anchor), (b, None)]
        return None

    found = []
    for sign in (1.0, -1.0):
        s = step
        while s <= max_shift:
            res = make(sign, s)
            if res is not None and (not arm_posts or arm_clearance(res) >= config.ARM_BODY_CLEARANCE_MIN_CM):
                if arm_posts and step > 1.0:
                    s2 = s - step + 1.0
                    while s2 < s - 1e-9:
                        r2 = make(sign, s2)
                        if r2 is not None and arm_clearance(r2) >= config.ARM_BODY_CLEARANCE_TARGET_CM:
                            res = r2
                            break
                        s2 += 1.0
                found.append(res)
                break
            s += step

    if not found:
        return None
    return min(found, key=_segment_result_length)


def _try_gate_bypass_detour(a, b, all_gates, all_posts, target_gate):
    """bへの正式な侵入方向(entry_axis)の直線上に、いずれかのゲート
    (target_gate自身に限らない)の支柱がちょうど乗ってしまう配置
    (seed=6199669: 黄exitから赤entryへ向かう際、entry_axisの直線が
    赤ゲート自身の脚の真上を通ってしまうケース。seed=9207916:
    3周目の黄exitからゴールへ向かう際、entry_axis上に赤ゲート自身の脚が
    ある、あるいは途中のtangent方向の直線が別ゲート(青)の脚をかすめて
    しまうケース、target_gateがNone=ゴール到着でも同様に起こる)向けの
    迂回。_try_axis_aligned_cornerが「正式なentry/exit以外の形でどこかの
    ゲートの脚を横切る」候補として除外してしまう配置が対象。

    aから、いずれかのゲートの支柱の並び方向(tangent、entry_axisと垂直)に
    直進し、その支柱を十分な余裕を持って通り過ぎた位置まで進む → そこから
    entry_axis方向に、bよりさらに手前(ゲートから離れる側)の高さまで
    直進する → tangent方向にbと同じ位置まで直進する → 最後にbまで
    entry_axis方向に直進する、という4本の直線(いずれもentry_axisか
    tangentのどちらかに平行で、斜めの区間は一切ない)を試す。

    bの高さ(entry自身)でtangent方向に戻ると、bがゲートからわずか
    ROBOT_FRONT_OVERHANG_CM(7.5cm)しか離れていないため、通り過ぎた
    はずの支柱にその高さで再接近してしまう(実際に見つかった不具合:
    bと同じ高さで戻る3本構成だと、支柱からの距離が7.5cmしかなく
    STRAIGHT_CLEARANCE_CM(7.85cm)を割ってしまい候補にならなかった)。
    そのため、tangent方向に戻る高さをbよりさらに離す。

    target_gateだけでなく全ゲートそれぞれの支柱を回り込み候補として試す
    (どちらの支柱を回り込むかも2通り)。安全なものを全て候補として返し、
    _resolve_top_level_segment側で他候補と合わせて距離で比較される。
    """
    required_heading = _required_entry_heading_deg(b, target_gate)
    rad = math.radians(required_heading)
    entry_axis = (math.cos(rad), math.sin(rad))
    tangent = (-entry_axis[1], entry_axis[0])

    buffer = STRAIGHT_CLEARANCE_CM + config.SAFETY_MARGIN_CM
    clear_margin = STRAIGHT_CLEARANCE_CM + config.SAFETY_MARGIN_CM

    a_axis = geo.dot(a, entry_axis)
    b_axis = geo.dot(b, entry_axis)
    b_tan = geo.dot(b, tangent)
    clear_axis = b_axis - clear_margin

    results = []
    for bypass_gate in all_gates:
        post_tangents = [geo.dot(p, tangent) for p in bypass_gate.posts()]
        for bypass_tan in (min(post_tangents) - buffer, max(post_tangents) + buffer):
            corner1 = geo.add(geo.scale(entry_axis, a_axis), geo.scale(tangent, bypass_tan))
            corner2 = geo.add(geo.scale(entry_axis, clear_axis), geo.scale(tangent, bypass_tan))
            corner3 = geo.add(geo.scale(entry_axis, clear_axis), geo.scale(tangent, b_tan))
            segs = ((a, corner1), (corner1, corner2), (corner2, corner3), (corner3, b))
            if any(_find_blocking_post(p1, p2, all_posts) is not None
                   or _segment_crosses_any_gate_leg(p1, p2, all_gates) for p1, p2 in segs):
                continue
            nearest1 = min(all_posts, key=lambda p: geo.distance(corner1, p))
            nearest2 = min(all_posts, key=lambda p: geo.distance(corner2, p))
            nearest3 = min(all_posts, key=lambda p: geo.distance(corner3, p))
            results.append([
                (a, None),
                (corner1, (nearest1, _post_owner_gate(nearest1, all_gates))),
                (corner2, (nearest2, _post_owner_gate(nearest2, all_gates))),
                (corner3, (nearest3, _post_owner_gate(nearest3, all_gates))),
                (b, None),
            ])
    return results


def _try_axis_first_bypass_detour(a, b, all_gates, all_posts, target_gate, prev_gate, gate_signs, step=5.0):
    """aが直前に通過したゲート(prev_gate)の延長上にあり(aのtangent座標が
    別ゲートの正式なentry/exitの列とたまたま一致する)、そのままentry_axis
    方向に直進を続けると別のゲートを正式に(covered、_candidate_crossing_
    safe参照)通過できる配置(seed=9207916: 黄exit直後に赤entry/exitの
    列に乗っており、そのまま直進すれば赤を正式に通過できるのに、
    _try_gate_bypass_detourは「aの高さでまずtangent方向」という順序
    固定のため、aの高さが別ゲート(赤)の支柱に近すぎて候補にならない)
    向けの迂回。

    aからentry_axis方向にbの手前まで直進(aのtangent座標を保つ) →
    tangent方向にbのtangent座標まで直進 → 最後entry_axis方向でbへ直進、
    という3本の直線を試す。切り替え高さ(t)の候補は、固定刻みの
    ステップ探索だけでは、安全な範囲が数cmしかない(別ゲートの支柱の
    すぐ外側)ケースを飛び越えてしまう(実際に見つかった不具合:
    有効なtの範囲が83.15〜86.1cmの約3cm幅しかなく、5cm刻みの探索では
    一度も命中しなかった)。そのため、各ゲートの支柱のentry_axis座標
    ±マージンという「ちょうど支柱をかわせるはずの」値を直接候補に加え、
    ステップ探索はそれ以外の配置への保険として残す。

    bがaよりentry_axis方向で手前(逆側)にある場合は対象外(その場合は
    _try_loop_detour/_try_gate_bypass_detourの対象)。
    """
    if prev_gate is None or gate_signs is None:
        return None
    required_heading = _required_entry_heading_deg(b, target_gate)
    rad = math.radians(required_heading)
    entry_axis = (math.cos(rad), math.sin(rad))
    tangent = (-entry_axis[1], entry_axis[0])

    a_axis = geo.dot(a, entry_axis)
    a_tan = geo.dot(a, tangent)
    b_axis = geo.dot(b, entry_axis)
    b_tan = geo.dot(b, tangent)

    if b_axis <= a_axis + 1e-6:
        return None

    margin = config.POST_AVOID_RADIUS_CM
    t_candidates = []
    for gate in all_gates:
        for post in gate.posts():
            post_axis = geo.dot(post, entry_axis)
            for cand in (post_axis - margin, post_axis + margin):
                if a_axis < cand <= b_axis:
                    t_candidates.append(cand)
    t_candidates.sort()
    t_candidates += [a_axis + step * i for i in range(1, int((b_axis - a_axis) / step) + 1)]

    for t in t_candidates:
        corner1 = geo.add(geo.scale(entry_axis, t), geo.scale(tangent, a_tan))
        corner2 = geo.add(geo.scale(entry_axis, t), geo.scale(tangent, b_tan))
        if (_find_blocking_post(a, corner1, all_posts) is None
                and _find_blocking_post(corner1, corner2, all_posts) is None
                and _find_blocking_post(corner2, b, all_posts) is None):
            nearest1 = min(all_posts, key=lambda p: geo.distance(corner1, p))
            nearest2 = min(all_posts, key=lambda p: geo.distance(corner2, p))
            candidate = [
                (a, None),
                (corner1, (nearest1, _post_owner_gate(nearest1, all_gates))),
                (corner2, (nearest2, _post_owner_gate(nearest2, all_gates))),
                (b, None),
            ]
            if (_candidate_internal_pivot_safe(candidate, all_posts)
                    and _candidate_crossing_safe(candidate, all_gates, gate_signs,
                                                  target_gate=target_gate, prev_gate=prev_gate)):
                return candidate
    return None


def _try_departure_aligned_corner(a, b, all_gates, all_posts, target_gate, prev_gate, gate_signs):
    """出発方向(departure_dir、prev_gateからの退出方向)とbへの侵入方向
    (entry_axis)が垂直に近い(_try_axis_aligned_cornerの通常の対象)配置で、
    aを通り出発方向の直線とbを通り侵入方向の直線の交点で1回だけ旋回する
    候補を試す。_try_axis_aligned_cornerとの違いは、a->corner区間が
    別ゲート自身の脚を「非公式な形」で横切ると判定されてもそこで諦めず、
    _candidate_crossing_safeのcovered判定(prev_gateからの延長で正式な
    通過とみなせるかどうか)に委ねる点(seed=9207916: 黄exit直後の区間が
    赤entry/exitの列にちょうど乗っており、そのまま出発方向へ直進すれば
    赤を正式に通過できるのに、_try_axis_aligned_cornerの厳密な脚横断
    チェックでは非公式な横断として除外されてしまっていた)。

    departure_dirとentry_axisが平行に近い場合は_try_loop_detourの対象
    のためNoneを返す。
    """
    if prev_gate is None or gate_signs is None:
        return None
    prev_sign = gate_signs.get(id(prev_gate))
    if prev_sign is None:
        return None
    departure_dir = geo.scale(prev_gate.normal, -prev_sign)

    required_heading = _required_entry_heading_deg(b, target_gate)
    rad = math.radians(required_heading)
    entry_axis = (math.cos(rad), math.sin(rad))

    if abs(abs(geo.dot(departure_dir, entry_axis)) - 1.0) < 0.1:
        return None  # 平行に近い配置は_try_loop_detourの対象

    denom = departure_dir[0] * entry_axis[1] - departure_dir[1] * entry_axis[0]
    if abs(denom) < 1e-9:
        return None
    dx, dy = b[0] - a[0], b[1] - a[1]
    t = (dx * entry_axis[1] - dy * entry_axis[0]) / denom
    if t <= 1e-6:
        return None
    corner = geo.add(a, geo.scale(departure_dir, t))

    if (_find_blocking_post(a, corner, all_posts) is not None
            or _find_blocking_post(corner, b, all_posts) is not None):
        return None

    nearest_post = min(all_posts, key=lambda p: geo.distance(corner, p))
    anchor = (nearest_post, _post_owner_gate(nearest_post, all_gates))
    candidate = [(a, None), (corner, anchor), (b, None)]
    if (_candidate_internal_pivot_safe(candidate, all_posts)
            and _departure_pivot_safe(candidate, prev_gate, gate_signs, all_posts)
            and _candidate_crossing_safe(candidate, all_gates, gate_signs,
                                          target_gate=target_gate, prev_gate=prev_gate)):
        return candidate
    return None


def _shortcut_consecutive_corners(result, all_posts):
    """resultの中に、3点連続(A->B->C)でA->Cを直接結んでも支柱に
    ぶつからない(STRAIGHT_CLEARANCE_CM以上離れている)箇所があれば、
    Bを取り除いてA->Cの直進1本に短縮する。変化がなくなるまで繰り返す
    (連続する複数の無駄な折れ曲がりも1回でまとめて畳み込む)。

    2026-09-18: resolve_segment(長方形オフセット)は、支柱を1本ずつ
    独立に避けていく(ある支柱を避けた結果できた点から、次に別の支柱に
    ぶつかったらまた避ける…)ため、実際には迂回が不要な「クランク状の
    遠回り」(例: 左折→右折→左折という、斜めに直進すれば済むはずの
    階段状の経路)を生成することがあった(seed=4106249: スタートから
    赤entryまでの区間が、直接結んでも17.8cm以上離れているにも関わらず
    96.8cmのクランクになっていた。斜めに結べば68.7cmで済む)。
    この関数は候補生成の後処理として、そうした不要な折れ曲がりを
    機械的に取り除く。short cut後の形はこの関数の後で改めて
    _fully_safe(entry到達時の旋回・出発時の旋回・ゲート横切り方向など)
    による検証を受けるため、直進クリアランス以外の安全性はそちらで
    別途担保される。
    """
    result = list(result)
    changed = True
    while changed and len(result) > 2:
        changed = False
        i = 0
        while i < len(result) - 2:
            a_pt = result[i][0]
            c_pt = result[i + 2][0]
            if all(_segment_post_distance(a_pt, c_pt, p) >= STRAIGHT_CLEARANCE_CM for p in all_posts):
                del result[i + 1]
                changed = True
            else:
                i += 1
    return result


def _resolve_top_level_segment(a, b, all_gates, all_posts, target_gate, prev_gate, gate_signs=None,
                                allow_via_other=True, strict_safe=False):
    """ステージ間をつなぐ最上位区間(_build_route_with_signsが直接呼ぶ区間)
    についてだけ、resolve_segment(長方形オフセット)と
    _resolve_segment_via_parallel_extension(軸方向延長)の両方を試し、
    「entryへの旋回が安全な候補」があればその中から、なければ全候補の
    中から、実際の距離が短い方を採用する。

    軸方向延長の方が単純で短い迂回になるケースが実際にあった(例:
    スタート地点から、支柱が密集した対角線を避けて安全な軸方向へ
    まっすぐ進んでから曲がる方が、長方形オフセットの複雑な迂回より
    大幅に短くなるケース)。ただしこの比較を再帰の全階層で行うと、
    その場では短くても全体としては不安定になることが200パターンの
    テストで判明したため、ステージの最上位区間だけに限定している。

    また、resolve_segmentは支柱への接触だけで「直線か迂回か」を判断する
    ため、直線自体は支柱にぶつからなくても、entryに到達する旋回角が
    大きくなりすぎて旋回安全性違反を起こす候補を、距離が短いというだけで
    選んでしまうことがあった(実際に見つかった不具合)。そのため、
    entry到達時の旋回が安全かどうかを候補選択の第一基準にする。

    さらに、target_gate以外の各ゲートを既に決まっている侵入方向
    (gate_signs)で経由するルートも常に候補に加える(直接・軸方向延長で
    安全な候補が見つかっている場合でも、他ゲートを公式に経由した方が
    短くなるケースが実際にあったため、「安全な候補が1つもない場合の
    最後の手段」ではなく、常に比較対象に含める)。gate_signsが渡されない
    場合(このフォールバックの内部再帰呼び出しなど)はこの手段自体を
    無限に繰り返さないよう省略する。
    """
    rect_result = None
    try:
        rect_result = resolve_segment(a, b, all_gates, all_posts, target_gate=target_gate)
        if not (_candidate_internal_pivot_safe(rect_result, all_posts)
                and _candidate_straight_clear_safe(rect_result, all_posts)):
            rect_result = _try_repair_candidate_offsets(rect_result, all_posts)
    except RuntimeError:
        pass

    axis_result = None
    try:
        axis_result = _resolve_segment_via_parallel_extension(
            a, b, all_gates, all_posts, target_gate, prev_gate
        )
        if axis_result is not None and not (
            _candidate_internal_pivot_safe(axis_result, all_posts)
            and _candidate_straight_clear_safe(axis_result, all_posts)
        ):
            axis_result = _try_repair_candidate_offsets(axis_result, all_posts)
    except RuntimeError:
        pass

    corner_results = [
        _try_axis_aligned_corner(a, b, all_gates, all_posts, target_gate, axis)
        for axis in ((1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0))
    ]
    shifted_corner_results = [
        _try_shifted_axis_corner(a, b, all_gates, all_posts, target_gate, axis)
        for axis in ((1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0))
    ]

    midpoint_result = None
    try:
        midpoint_result = _try_midpoint_post_detour(a, b, all_gates, all_posts, target_gate)
    except RuntimeError:
        pass

    loop_result = None
    try:
        loop_result = _try_loop_detour(a, b, all_gates, all_posts, target_gate, prev_gate, gate_signs)
    except RuntimeError:
        pass

    bypass_results = []
    try:
        bypass_results = _try_gate_bypass_detour(a, b, all_gates, all_posts, target_gate)
    except RuntimeError:
        pass

    axis_first_result = None
    try:
        axis_first_result = _try_axis_first_bypass_detour(
            a, b, all_gates, all_posts, target_gate, prev_gate, gate_signs
        )
    except RuntimeError:
        pass

    departure_corner_result = None
    try:
        departure_corner_result = _try_departure_aligned_corner(
            a, b, all_gates, all_posts, target_gate, prev_gate, gate_signs
        )
    except RuntimeError:
        pass

    diagonal_results = _try_diagonal_to_entry_axis(a, b, all_gates, all_posts, target_gate)

    raw_candidates = [
        r for r in (rect_result, axis_result, midpoint_result, loop_result, axis_first_result,
                    departure_corner_result, *corner_results, *shifted_corner_results, *bypass_results,
                    *diagonal_results)
        if r is not None
    ]
    # 2026-09-18: 当初は各候補をその場でshortcut後の形に置き換えていたが、
    # これだと「中継点があることで出発/到着時の旋回が安全になっている」
    # 候補(例: _try_loop_detourが、出発方向と目標方向がほぼ逆向きな
    # 配置向けに意図して作る中継点)まで、直進クリアランスだけを見て
    # 機械的に間引いてしまい、安全な候補そのものを消してしまう不具合が
    # あった(seed=9196925: 黄色ゲート退出直後にゴールへ向かう区間で、
    # 中継点(61.5,26.1)を削って直結すると、退出時の旋回が支柱に
    # 近すぎる危険な旋回になっていたが、_shortcut_consecutive_corners
    # 自身は出発/到着時の旋回安全性を判定する材料(prev_gate/target_gate
    # の情報)を持たないため、それを検知できなかった)。
    # そのため、短縮版を元の候補の「置き換え」ではなく「追加の候補」として
    # 扱う。安全性の最終判定(_fully_safe、下記)は両方に対して個別に
    # 行われるため、短縮版が安全ならそちらが(より短いので)選ばれ、
    # 短縮版が退出/到着時の旋回を壊すなら元の(中継点を保った)候補だけが
    # 安全な候補として残る。
    candidates = list(raw_candidates)
    for r in raw_candidates:
        shortcut = _shortcut_consecutive_corners(r, all_posts)
        if len(shortcut) != len(r):
            candidates.append(shortcut)

    def _fully_safe(r):
        return (
            _entry_arrival_pivot_safe(r, target_gate, all_posts)
            and _departure_pivot_safe(r, prev_gate, gate_signs, all_posts)
            and _candidate_internal_pivot_safe(r, all_posts)
            and _candidate_crossing_safe(r, all_gates, gate_signs, target_gate=target_gate, prev_gate=prev_gate)
            and _candidate_straight_clear_safe(r, all_posts)
            and _candidate_keepout_safe(r)
        )

    safe_candidates = [r for r in candidates if _fully_safe(r)]

    # 「経由してよい他色ゲート」は、target_gateが周回の最初の色
    # (config.GATE_ORDER[0]、通常は赤)のときだけに限る。
    #
    # 最初は「target_gateより後(まだ正式通過の順番が来ていない色)なら
    # 常に許可」という条件にしていたが、それでも順序違反が残る配置が
    # 見つかった(seed=320358の別sign組み合わせ)。原因: verify_gate_passage_order
    # の状態機械はexpected_idx(次に来るべき色のGATE_ORDER内位置)を持ち、
    # 期待と違う色が来るたびexpected_idxを0にリセットする。target_gateが
    # 周回の2番目以降(例: 青)のとき、その手前で未来の色(黄)を経由すると、
    # 「赤(正式,idx0->1) -> 黄(経由,期待は青なので不一致->リセットidx=0)
    # -> 青(正式だが期待はもう赤...不一致) -> 黄(正式、これも不一致)」と
    # なり、せっかく正式に通過した赤の分すら失われて周回が壊れる。
    # 一方target_gateが周回の最初の色(赤)のときだけは、経由がexpected_idx
    # がまだ0の時点(=この周回でまだ何も正式通過していない時点)で起きる
    # ため、リセットしても失うものがなく安全(経由後の赤->青->黄が
    # そのまま正常に成立する。2026-09-15に見つかった元々の動機
    # (赤と黄が近すぎるケース)もちょうどこの形)。
    # target_gate=None(全周回を終えてゴールへ向かう最終区間)は、これ以上
    # 「今回の周回の正しい順序」を気にする必要がないため無制限のままでよい。
    via_allowed_colors = None
    if target_gate is not None:
        via_allowed_colors = (
            set(config.GATE_ORDER[1:])
            if config.GATE_ORDER and target_gate.color == config.GATE_ORDER[0]
            else set()
        )

    if gate_signs is not None and allow_via_other:
        for other_gate in all_gates:
            if other_gate is target_gate:
                continue
            if via_allowed_colors is not None and other_gate.color not in via_allowed_colors:
                continue
            other_sign = gate_signs.get(id(other_gate))
            if other_sign is None:
                continue
            via_result = _resolve_via_other_gate(
                a, b, all_gates, all_posts, target_gate, other_gate, other_sign,
                prev_gate=prev_gate, gate_signs=gate_signs,
            )
            if via_result is not None and _fully_safe(via_result):
                safe_candidates.append(via_result)

        # aへ到達するのに使った直進(prev_gateのentry->exit)の中に、別の
        # ゲートが丸ごと収まっている配置(seed=720696)では、そのゲートの
        # exitまではすでに実質的に到達しているとみなせる。そこから改めて
        # bへの区間を解決した候補も試す(_try_extend_through_covered_gateの
        # docstring参照。allow_via_other=Falseで無限の連鎖を防ぐ)。
        covered_head, covered_gate = _try_extend_through_covered_gate(
            a, all_gates, all_posts, prev_gate, gate_signs
        )
        if covered_head is not None:
            other_exit = covered_head[-1][0]
            try:
                tail = _resolve_top_level_segment(
                    other_exit, b, all_gates, all_posts, target_gate, covered_gate,
                    gate_signs=gate_signs, allow_via_other=False,
                )
                covered_result = covered_head[:-1] + tail
                if _fully_safe(covered_result):
                    safe_candidates.append(covered_result)
            except RuntimeError:
                pass

    if not candidates and not safe_candidates:
        raise RuntimeError("迂回経路を見つけられませんでした")

    if safe_candidates:
        pool = safe_candidates
    elif strict_safe:
        # 呼び出し側(ゴールへの最終区間)が、別の経路(中継点経由)を持っているため、
        # 安全でない候補へは落とさず、見つからなかったことを伝える。
        raise RuntimeError("進入禁止エリアを避けた安全な候補がありません")
    else:
        # 完全に安全な候補が無いときのフォールバックでも、進入禁止エリアに入らないものを優先する。
        keepout_ok = [c for c in candidates if _candidate_keepout_safe(c)]
        pool = keepout_ok if keepout_ok else candidates

    # 2026-09-18: 候補選択がSTRAIGHT_CLEARANCE_CM(物理的な最小距離)を
    # 満たしてさえいれば最短のものを選ぶため、target_gate自身のT字パーツに
    # 対してギリギリ(7.85cm前後)の距離しかない斜めの直進が、より安全な
    # 迂回(例: entry軸に揃えてから直進するL字型、_try_axis_aligned_corner
    # 参照)より短いというだけで選ばれてしまうことがあった(seed=2994807:
    # 青ゲートentry直前が8.01cm、実機で接触)。
    #
    # 一度は「target_gate自身のT字パーツにはSTRAIGHT_CLEARANCE_CM+
    # POST_ARM_STRAIGHT_MARGIN_CMを要求する」形で閾値自体を引き上げて
    # みたが、is_valid(=8通りのsign探索でのcompleted_lapsより優先される
    # 判定基準)がこの引き上げた閾値を使うことになり、「違反0件を達成
    # できる候補」が大幅に減った結果、3周成立より物理的な違反件数の
    # 少なさが優先されてしまい、周回不成立が1/200から38/200まで悪化した
    # (詳細はrun_random.py等の実行ログ、2026-09-18のセッション参照)。
    #
    # そのため今回は、is_valid/_fully_safe(閾値はSTRAIGHT_CLEARANCE_CMの
    # まま)には一切手を入れず、「既にfully safeな候補同士」の中でだけ、
    # target_gate自身のT字パーツから実質的な余裕(STRAIGHT_CLEARANCE_CM+
    # POST_ARM_STRAIGHT_MARGIN_CM)がある候補を優先する2段階選択にする。
    # 余裕のある候補が1つもない場合は、従来通りsafe_candidates全体から
    # 最短のものを選ぶ(=is_validの判定にも8通りのsign探索の結果にも
    # 一切影響しない)。
    target_arm_posts = [
        p for p in all_posts
        if target_gate is not None and getattr(p, "color", None) == target_gate.color
        and (p.arm_dir[0] or p.arm_dir[1])
    ]
    if target_arm_posts:
        desired_clearance = STRAIGHT_CLEARANCE_CM + config.POST_ARM_STRAIGHT_MARGIN_CM

        def entry_arm_clearance(result):
            points = [pt for pt, _ in result]
            worst = float("inf")
            for i in range(len(points) - 1):
                for p in target_arm_posts:
                    worst = min(worst, _segment_post_distance(points[i], points[i + 1], p))
            return worst

        def entry_arm_body_clearance(result):
            points = [pt for pt, _ in result]
            worst = float("inf")
            for i in range(len(points) - 1):
                for p in target_arm_posts:
                    worst = min(worst, _segment_body_clearance(points[i], points[i + 1], p))
            return worst

        # 2026-09-18: 既存の候補パターン(rect/axis/corner/loop/bypass等)の
        # どれも十分な余裕(desired_clearance)を作れない配置が多数見つかった
        # (601シード中434シードでentry直前がtarget_gate自身のT字パーツに
        # 10.85cm未満しか離れていない。うち一部は7.9〜8.7cm程度とかなり近い)。
        # そこで、pool中で最短の候補について、entryの1つ手前の点を
        # target_gate.direction(脚に平行な方向、_try_extendと同じ考え方)へ
        # 延長し、target_gate自身のT字パーツからdesired_clearance以上離れる
        # 形に改善できないか試す(全く新しい迂回パターンを増やすのではなく、
        # 既存の候補の「entry直前の1点」だけを動かす、影響範囲の狭い追加候補)。
        shortest_length = _segment_result_length(min(pool, key=_segment_result_length))
        base = min(pool, key=_segment_result_length)

        # 2026-09-19追記: 車体footprint基準で既に3.0cm(MIN_BODY_CLEARANCE_
        # FOR_STRONG_CM、実機で「これなら十分」と判断された基準そのもの)
        # 以上離れている最短候補があるのに、それより大きく遠回りな候補を
        # 優先してしまう例が見つかった(seed=7605978: 青→黄が本来83.8cmの
        # 直線的な斜め移動で済み、旋回軸基準クリアランスも10.6cm・車体
        # footprint基準でも3.75cmと「十分」の基準を満たしていたのに、
        # 旋回軸基準の目標値(desired_clearance=10.85cm)にわずか0.25cm
        # 届かないというだけで、118.1cm(+34cm、旋回も1つ増える)の
        # L字迂回が「strong」または「明確な改善」判定で選ばれていた)。
        # 「3cmで十分」という基準自体が実機検証済みの受け入れラインである
        # 以上、それを既に満たす最短候補があるなら、それ以上の余裕を
        # わずかでも稼ぐために大きく遠回りする理由はない。そのため、
        # 最短候補が既にこの基準を満たす場合は、以降の「より余裕のある
        # 候補を探して置き換える」処理自体を丸ごとスキップする。
        MIN_BODY_CLEARANCE_FOR_STRONG_CM = 3.0
        if entry_arm_body_clearance(base) >= MIN_BODY_CLEARANCE_FOR_STRONG_CM:
            return min(pool, key=lambda r: (round(_segment_result_length(r), 6), len(r)))

        extended = _try_extend_entry_arm_clearance(
            base, target_gate, all_posts, target_arm_posts, desired_clearance
        )
        candidates_for_arm = list(pool)
        if extended is not None and _fully_safe(extended):
            candidates_for_arm.append(extended)

        # 2026-09-21: 最短の候補が、向かうゲート自身のT字パーツまで車体3cm未満のとき、
        # 「車体クリアランスが方針の最小値(config.ARM_BODY_CLEARANCE_MIN_CM)以上を満たす、最短の候補」を
        # 採用する。従来は、旋回軸基準の強い条件(STRAIGHT_CLEARANCE_CM+POST_ARM_STRAIGHT_MARGIN_CM以上)
        # を満たす候補を優先していたため、方針を満たす短い経路(例: 赤のexitの真下に青ゲートがある
        # 配置で、20.4cm)を落として、48.3cmの迂回を選んでいた。既存の候補に、斜めの中継点を1つ置く
        # 候補の探索(_search_shortest_arm_safe_detour)を足して、その中で最短のものを選ぶ。
        # 3cmを満たす候補が1つも無いときは、従来の選び方(下)へ進む。
        min_body = config.ARM_BODY_CLEARANCE_MIN_CM
        upper_bound = min(
            (_segment_result_length(r) for r in candidates_for_arm
             if entry_arm_body_clearance(r) >= min_body),
            default=float("inf"))
        # 同じ配置の中で、同じ条件の探索が、sign組み合わせごとに何度も来るため、結果を覚えておく
        # (plan_route_with_anchorsの先頭で消す)。判定に効く条件は、全部キーに入れる。
        cache_key = (
            a, b, id(target_gate), id(prev_gate), id(all_gates), strict_safe, round(upper_bound, 6),
            tuple(sorted(gate_signs.items(), key=lambda kv: str(kv[0]))) if gate_signs else None,
        )
        if cache_key in _ARM_SEARCH_CACHE:
            searched = _ARM_SEARCH_CACHE[cache_key]
        else:
            searched = _search_shortest_arm_safe_detour(
                a, b, _fully_safe, entry_arm_body_clearance, min_body, upper_bound=upper_bound)
            _ARM_SEARCH_CACHE[cache_key] = searched
        if searched is not None:
            candidates_for_arm.append(searched)
        meeting_min = [r for r in candidates_for_arm if entry_arm_body_clearance(r) >= min_body]
        if meeting_min:
            return min(meeting_min, key=lambda r: (round(_segment_result_length(r), 6), len(r)))
        # 2026-09-18追記: 「desired_clearanceを満たす候補があれば無条件で
        # それを優先する」形にしたところ、そのために必要な迂回が異常に
        # 長くなるケースが見つかった(seed=5719257: 最短候補55.9cmに対し、
        # desired_clearanceを満たす唯一の候補が228.2cm、別のゲートを
        # 経由する大回りだった)。一方、最短候補自体もSTRAIGHT_CLEARANCE_CM
        # (7.85cm)は上回っており、実機マージン的にも多くの場合十分
        # (車体footprint基準で3cm前後)だったため、「大幅に遠回りしてまで
        # desired_clearanceぴったりを満たす」ことよりも「そこそこの長さで
        # そこそこの余裕を確保する」方を優先すべきだと判断した。
        # 具体的には、最短候補の長さ+60cmを「妥当な迂回」の上限とし、
        # その範囲内の候補の中でdesired_clearanceを満たすものがあれば
        # それを、なければその範囲内で最もクリアランスが大きいものを選ぶ。
        # 範囲を超える遠回り(228cmのような候補)は、desired_clearanceを
        # 満たしていても採用しない(=STRAIGHT_CLEARANCE_CMさえ満たせば
        # 元のpoolから最短のものが選ばれる、従来の動作にフォールバックする)。
        length_budget = shortest_length + 60.0
        within_budget = [r for r in candidates_for_arm if _segment_result_length(r) <= length_budget]
        # 2026-09-18追記: entry_arm_clearance(旋回軸基準)だけでdesired_
        # clearanceを満たすかどうかを判定すると、斜めの区間で騙される
        # ケースが見つかった(seed=9196925: 黄色exit->赤entry。旋回軸基準
        # では12.3cmでdesired_clearance(10.85cm)を満たす「強い候補」と
        # 判定されたが、車体footprint基準では実際には0.33cm(ほぼ接触)
        # しかなかった)。そのため、strong_candidatesの条件に車体
        # footprint基準の最低ライン(MIN_BODY_CLEARANCE_FOR_STRONG_CM、
        # 3cm。実機で「これなら十分」と判断された基準と同じ)も追加し、
        # 両方を満たす場合だけ「強い候補」として扱う
        # (MIN_BODY_CLEARANCE_FOR_STRONG_CMは上のbase判定と同じ定数)。
        strong_candidates = [
            r for r in within_budget
            if entry_arm_clearance(r) >= desired_clearance
            and entry_arm_body_clearance(r) >= MIN_BODY_CLEARANCE_FOR_STRONG_CM
        ]

        if strong_candidates:
            pool = strong_candidates
        elif within_budget:
            # 2026-09-18追記: desired_clearanceに届く候補が1つもない場合、
            # 「budget内で最もクリアランスが大きいもの」を無条件で優先
            # していたところ、その差がわずか(0.4cm程度)なのに、より短く
            # 素直な候補(例: _try_axis_aligned_cornerが見つけた19.7cmの
            # 候補)が、8.70cm vs 8.30cmというごくわずかな差だけで、より
            # 長く遠回りな候補(21.7cm)に負けてしまう例が見つかった
            # (seed=9196925: 青ゲートexit->黄色entry)。desired_clearanceに
            # 届かない以上、どちらも「ギリギリ」であることに変わりはなく、
            # 微妙な差のために長さを犠牲にする価値はない。そのため、
            # クリアランスの改善が明確(+1.5cm以上)な場合だけ優先し、
            # それ未満ならpoolを変更せず、通常の最短優先に任せる。
            #
            # 2026-09-18さらに追記: この「改善が明確かどうか」の比較を
            # entry_arm_clearance(旋回軸=タイヤ中心線基準)で行っていた
            # ところ、斜めの区間では実際の車体の角の位置を正しく反映しない
            # ことが分かった(seed=9196925: 黄色exit->赤entry。旋回軸基準
            # では12.3cmで8.5cmの候補より「明確に安全」に見えたが、車体
            # footprint基準では実際には0.3cm(ほぼ接触)しかなく、8.5cm側
            # (車体footprint基準1.65cm)の方が実際には安全だった)。その
            # ため、この「明確な改善かどうか」の比較にはentry_arm_
            # clearanceではなくentry_arm_body_clearance(車体footprint
            # 基準)を使う。desired_clearance/strong_candidatesの閾値判定
            # (旋回軸基準、601シードで検証済み)自体は変更しない。
            baseline_clearance = entry_arm_body_clearance(base)
            best_candidate = max(within_budget, key=entry_arm_body_clearance)
            best_clearance = entry_arm_body_clearance(best_candidate)
            if best_clearance >= baseline_clearance + 1.5:
                pool = [r for r in within_budget if entry_arm_body_clearance(r) >= best_clearance - 1e-9]

    # 2026-09-18: 距離が(ほぼ)同点の候補が複数あるとき、単純に
    # _segment_result_lengthだけで選ぶとどちらが選ばれるかは候補が
    # 生成された順序次第になる。実際に見つかった例(seed=9392783):
    # スタート->赤entryで、1回だけ曲がる素直な経路(長さ84.3cm)と、
    # 4回も折れ曲がる無意味に複雑な経路(同じく長さ84.3cm)が同点になり、
    # たまたま後者が先に候補リストに入っていたために選ばれてしまって
    # いた。安全性には影響しないが、実機での旋回回数が増えるだけ無駄で
    # 見た目にも不自然なので、距離が同点(浮動小数点誤差を許容して
    # ほぼ同点)の場合は経由点の少ない(=曲がる回数が少ない)方を優先する。
    def _sort_key(result):
        return (round(_segment_result_length(result), 6), len(result))

    # 2026-09-20: target_gate以外のゲートのT字パーツ(ゴールへの最終区間や、ゲート間の
    # 移動で近くを通る他ゲートのT字パーツ)に対しても、車体前方が
    # config.ARM_BODY_CLEARANCE_MIN_CM(3cm)以上離れている候補を優先する
    # (seed=20598: ゴールへの斜めの移動が、赤ゲートのT字パーツに1.36cmまで
    # 近づいていた)。最短候補が既に基準を満たしていれば何も変えない。満たさない
    # 場合だけ、最短+ARM_OTHER_GATES_LENGTH_BUDGET_CMの範囲内で、基準を満たす
    # 最短の候補を選ぶ。満たすものが無ければ、クリアランスが+1.5cm以上改善する
    # 候補があればそれを、なければ従来どおり最短を選ぶ。
    if config.ARM_CLEARANCE_OTHER_GATES and config.ARM_BODY_CLEARANCE_TARGET_CM > 0:
        target_ids = {id(p) for p in target_arm_posts}
        other_arm_posts = [
            p for p in all_posts
            if id(p) not in target_ids and (p.arm_dir[0] or p.arm_dir[1])
        ]

        def other_arm_clearance(result):
            points = [pt for pt, _ in result]
            worst = float("inf")
            for i in range(len(points) - 1):
                for p in other_arm_posts:
                    worst = min(worst, _segment_body_clearance(points[i], points[i + 1], p))
            return worst

        if other_arm_posts:
            best = min(pool, key=_sort_key)
            need = config.ARM_BODY_CLEARANCE_MIN_CM
            if other_arm_clearance(best) < need:
                shortest_len = _segment_result_length(best)
                within = [r for r in pool
                          if _segment_result_length(r) <= shortest_len + config.ARM_OTHER_GATES_LENGTH_BUDGET_CM]
                good = [r for r in within if other_arm_clearance(r) >= need]
                if good:
                    return min(good, key=_sort_key)
                if within:
                    top = max(within, key=other_arm_clearance)
                    if other_arm_clearance(top) >= other_arm_clearance(best) + 1.5:
                        return top

    return min(pool, key=_sort_key)


_ARM_SEARCH_CACHE = {}


def _search_shortest_arm_safe_detour(a, b, fully_safe, body_clearance, min_body, upper_bound=float("inf"),
                                      margin=20.0, step=6.0, refine_steps=(2.0, 1.0)):
    """aからbへ、中継点を1つ置く経路のうち、全ての安全判定(fully_safe)を通り、かつ向かうゲートの
    T字パーツまでの車体クリアランスがmin_body以上になる、最短のものを探す。見つからなければNone。

    中継点は、a・bを囲む範囲(margin広げる)のstep刻みの格子点。経路の長さの短い順に並べ、
    最初に条件を満たしたものを採用する(全ての格子点を判定するわけではない)。見つかった中継点の
    まわり(±step)は、refine_steps刻みで段階的に探し直し、より短いものがあれば、それに置き換える。
    upper_boundより長い候補は、探さない(すでに、もっと短い候補があるため)。
    """
    def path(w):
        return [(a, None), (w, None), (b, None)]

    def length(w):
        return geo.distance(a, w) + geo.distance(w, b)

    def ok(w):
        r = path(w)
        return body_clearance(r) >= min_body and fully_safe(r)

    x_lo, x_hi = min(a[0], b[0]) - margin, max(a[0], b[0]) + margin
    y_lo, y_hi = min(a[1], b[1]) - margin, max(a[1], b[1]) + margin
    points = []
    x = x_lo
    while x <= x_hi:
        y = y_lo
        while y <= y_hi:
            w = (x, y)
            if geo.distance(a, w) > 1.0 and geo.distance(b, w) > 1.0:
                l = length(w)
                if l < upper_bound - 1e-6:
                    points.append((l, w))
            y += step
        x += step
    points.sort()
    found = None
    for l, w in points:
        if found is not None and l >= found[0] - 1e-6:
            break
        if ok(w):
            found = (l, w)
            break
    if found is None:
        return None
    # 見つかった格子点のまわりを、段階的に細かく探して、より短いものがあれば置き換える。
    best_l, best_w = found
    radius = step
    for fine in refine_steps:
        n = int(round(radius / fine))
        center = best_w
        for i in range(-n, n + 1):
            for j in range(-n, n + 1):
                w = (center[0] + i * fine, center[1] + j * fine)
                if geo.distance(a, w) <= 1.0 or geo.distance(b, w) <= 1.0:
                    continue
                l = length(w)
                if l < best_l - 1e-6 and l < upper_bound - 1e-6 and ok(w):
                    best_l, best_w = l, w
        radius = fine
    return path(best_w)


def _try_extend_entry_arm_clearance(result, target_gate, all_posts, target_arm_posts, desired_clearance,
                                     max_extension=200.0, step=2.0):
    """resultのentry直前の点(最後から2番目)を、target_gate.direction方向へ
    動かし、entryへの最後の直進がtarget_gate自身のT字パーツから
    desired_clearance以上離れるようにする。

    _try_extend/_extend_point_for_straight_clearanceと同じ「ゲートに平行な
    方向へ延長する」考え方を、修正(repair)ではなく候補生成(candidate)側で
    使う。resultの点数が2未満(=直接entryに到達する経路で、手前の点が
    存在しない)場合はNoneを返す。

    2026-09-18: 当初は二分探索(単調に離れていく前提)で実装したが、
    ゲートには支柱が2本(両脚)あり、片方から離れるほどもう片方に近づく
    ケースがある(seed=2994807: 青ゲート下脚からは離れるほど安全になるが、
    ある地点を超えると今度は上脚に近づいていく)。そのため
    target_arm_posts全体でのmin距離は延長量に対して単調ではなく
    (山型)、二分探索では「最大延長でも届かない」場合に山の中腹にある
    有効な区間を全く探索できずに諦めてしまっていた。そのため二分探索
    ではなく、step刻みで先頭から順に探し、最初に条件を満たした点(=その
    向きでの最短延長)を採用するグリッド走査に変更した。

    2026-09-18追記: min_clearanceが「候補点->entry」の区間しか
    target_arm_postsとの距離を確認しておらず、「prev_pt->候補点」の
    区間は無視していたバグがあった。そのため、entryへの区間は十分
    離れているのに、その手前の区間(延長した結果できる方)がtarget_gate
    自身のT字パーツに近づいてしまっている延長を「成功」と誤判定して
    いた(seed=5719257で発覚。手前の区間のクリアランスは実際には
    8.3cmしかなかったのに、この関数は延長に成功したと報告し、それが
    strong_candidatesに入らなかった結果、無関係などこかの遠回り候補
    (228cm)が選ばれてしまっていた)。両方の区間を確認するよう修正した。
    """
    points = [pt for pt, _ in result]
    if len(points) < 3:
        return None
    idx = len(points) - 2
    point = points[idx]
    prev_pt = points[idx - 1]
    entry_pt = points[idx + 1]
    direction = target_gate.direction

    def min_clearance(candidate_pt):
        return min(
            min(_segment_post_distance(prev_pt, candidate_pt, p), _segment_post_distance(candidate_pt, entry_pt, p))
            for p in target_arm_posts
        )

    def other_posts_ok(prev_pt_, candidate_pt):
        for p in all_posts:
            if any(geo.distance(p, tp) < 1e-6 for tp in target_arm_posts):
                continue
            if geo.segment_blocked_by_circle(prev_pt_, candidate_pt, p, STRAIGHT_CLEARANCE_CM):
                return False
            if geo.segment_blocked_by_circle(candidate_pt, entry_pt, p, STRAIGHT_CLEARANCE_CM):
                return False
        return True

    best = None
    for sign in (1.0, -1.0):
        t = step
        while t <= max_extension:
            candidate_pt = geo.add(point, geo.scale(direction, sign * t))
            if min_clearance(candidate_pt) >= desired_clearance and other_posts_ok(prev_pt, candidate_pt):
                length = geo.distance(prev_pt, candidate_pt) + geo.distance(candidate_pt, entry_pt)
                if best is None or length < best[0]:
                    best = (length, candidate_pt)
                break  # この向きでは最初に見つかった(=最短の)ものを採用
            t += step

    if best is None:
        return None
    _, new_pt = best
    new_result = list(result)
    new_result[idx] = (new_pt, result[idx][1])
    return new_result


def _try_diagonal_to_entry_axis(a, b, all_gates, all_posts, target_gate):
    """aから、entryの軸(ゲートを通る向き)上の点c(entryからD手前)へ斜めに進み、
    そこからentryまで軸に沿ってまっすぐ進む、「斜め+まっすぐ」の候補を返す
    (2026-09-20: 「直進→90度→90度」の折れ線より、斜めに進んで軸に乗る方が
    短い配置がある。seed=4418202の青→黄で93.5cmが68.6cm、seed=1437404の赤→青で
    167.3cmが135.0cmになる)。

    Dはentryの手前の距離。近すぎると軸に乗る旋回が急になるので、config.
    DIAGONAL_ENTRY_D_MIN_CM以上、config.DIAGONAL_ENTRY_D_MAX_CMまでconfig.
    DIAGONAL_ENTRY_D_STEP_CMごとに候補を作る(安全性・T字パーツの余裕は、
    呼び出し側の_fully_safeと、T字パーツ用の候補選びが判断する)。
    ゴール(target_gateがNone)は対象外。aとbが既に軸上にある場合(直線で済む)も対象外。
    """
    if target_gate is None or not config.DIAGONAL_ENTRY_ENABLED:
        return []
    heading = _required_entry_heading_deg(b, target_gate)
    e = (math.cos(math.radians(heading)), math.sin(math.radians(heading)))
    # aがentry軸上(bからeの逆向きの直線上)にあれば、直線で足りる
    ab = geo.sub(b, a)
    if abs(ab[0] * e[1] - ab[1] * e[0]) < 1e-6 and ab[0] * e[0] + ab[1] * e[1] > 0:
        # 2026-09-22: 軸上にあっても、aがentryの「先(ゲートを通り過ぎた側)」にあるときは、直線では入れない
        # (ゲートの反対側から来ているため)。その場合は、下の候補の探索へ進む(seed=3689058)。
        return []
    results = []
    direct = geo.distance(a, b)
    max_len = 1.6 * direct + 30.0   # 遠回りすぎる候補は作らない(計算量を抑える)

    def ok_segments(pts):
        return (all(_find_blocking_post(x, y, all_posts) is None for x, y in zip(pts, pts[1:]))
                and not any(_segment_crosses_any_gate_leg(x, y, all_gates) for x, y in zip(pts, pts[1:])))

    D = config.DIAGONAL_ENTRY_D_MIN_CM
    while D <= config.DIAGONAL_ENTRY_D_MAX_CM + 1e-9:
        c = geo.sub(b, geo.scale(e, D))
        pts = [a, c, b]
        if (geo.distance(a, c) > 1e-6 and geo.distance(a, c) + D <= max_len and ok_segments(pts)):
            results.append([(a, None), (c, None), (b, None)])
        D += config.DIAGONAL_ENTRY_D_STEP_CM
    if results and min(geo.distance(r[0][0], r[1][0]) + geo.distance(r[1][0], r[2][0]) for r in results) <= 1.25 * direct + 10.0:
        return results  # 1つ目の形で、ほぼ直線に近い候補があれば、2つ目は試さない
    # 2つ目の形(1つ目が成立しない、または遠回りしかないときだけ): 斜めに進んで、entry軸に垂直な線(ゲートの脇)に乗り、
    # その線に沿って軸上の点cへ進み、軸に沿ってentryへ入る(a -> p1 -> c -> b)。
    # ゲートの脚を回り込む必要があるとき(seed=1437404の赤->青)用。
    tangent = (-e[1], e[0])
    for D in config.DIAGONAL_BYPASS_D_LIST_CM:
        c = geo.sub(b, geo.scale(e, D))
        for sign in (1.0, -1.0):
            t = config.DIAGONAL_BYPASS_T_STEP_CM
            while t <= config.DIAGONAL_BYPASS_T_MAX_CM + 1e-9:
                p1 = geo.add(c, geo.scale(tangent, sign * t))
                if geo.distance(a, p1) > 1e-6 and geo.distance(a, p1) + t + D <= max_len:
                    pts = [a, p1, c, b]
                    if ok_segments(pts):
                        results.append([(a, None), (p1, None), (c, None), (b, None)])
                t += config.DIAGONAL_BYPASS_T_STEP_CM
    # 3つ目の形(上の2つの形が作れない、または遠回りしかないときだけ): 斜めに進んで、entry軸と平行な
    # 通路(ゲートの脇、軸から t だけ離れた線)に乗り、その線に沿って進み、軸に垂直に軸上の点cへ戻り、
    # 軸に沿ってentryへ入る(a -> w1 -> w2 -> c -> b)。2026-09-22: 赤ゲートを南から入る組み合わせで、
    # 北側(黄のexit)から南側へ回り込む配置(seed=3689058)で、上の形では、長さの上限を超えて作れず、
    # 別のゲートを通り抜ける大回りが選ばれていた。長さの短い順に、最大CORRIDOR_MAX_CANDIDATES個だけ返す。
    if not results or min(_segment_result_length(r) for r in results) > 1.6 * direct:
        corridor = []
        tangent = (-e[1], e[0])
        for D in config.DIAGONAL_BYPASS_D_LIST_CM:
            c = geo.sub(b, geo.scale(e, D))
            for sign in (1.0, -1.0):
                for t in config.CORRIDOR_T_LIST_CM:
                    w2 = geo.add(c, geo.scale(tangent, sign * t))
                    foot = (a[0] - w2[0]) * e[0] + (a[1] - w2[1]) * e[1]
                    for shift in config.CORRIDOR_SHIFT_LIST_CM:
                        w1 = geo.add(w2, geo.scale(e, foot + shift))
                        if geo.distance(a, w1) < 1e-6 or geo.distance(w1, w2) < 1e-6:
                            continue
                        length = geo.distance(a, w1) + geo.distance(w1, w2) + t + D
                        corridor.append((length, [a, w1, w2, c, b]))
        corridor.sort(key=lambda item: item[0])
        taken = 0
        for length, pts in corridor:
            if taken >= config.CORRIDOR_MAX_CANDIDATES:
                break
            if ok_segments(pts):
                results.append([(p, None) for p in pts])
                taken += 1
    return results


def _resolve_goal_segment(current, all_gates, all_posts, prev_gate, gate_signs):
    """ゴールへの最終区間を作る。まず、直進の余裕を config.GOAL_STRAIGHT_EXTRA_MARGIN_CM だけ増やした条件で
    試し(全ての安全条件を満たす候補だけ)、作れなければ、従来の条件で作る。
    (STRAIGHT_CLEARANCE_CMは、この区間の探索の間だけ一時的に増やして、必ず元に戻す。)
    """
    global STRAIGHT_CLEARANCE_CM
    extra = config.GOAL_STRAIGHT_EXTRA_MARGIN_CM
    if extra > 0:
        base = STRAIGHT_CLEARANCE_CM
        STRAIGHT_CLEARANCE_CM = base + extra
        try:
            return _resolve_goal_segment_core(current, all_gates, all_posts, prev_gate, gate_signs, strict_only=True)
        except RuntimeError:
            pass
        finally:
            STRAIGHT_CLEARANCE_CM = base
    return _resolve_goal_segment_core(current, all_gates, all_posts, prev_gate, gate_signs)


def _resolve_goal_segment_core(current, all_gates, all_posts, prev_gate, gate_signs, strict_only=False):
    """最後のゲートの退出点からゴールへの区間を作る。

    ゴールへ直接向かう通常の候補(進入禁止エリアに入らないもの)と、中継点C
    (config.KEEP_OUT_GOAL_LANE_POINT_CM)まで通常の探索で進み、Cからゴールへ直進する候補
    (Cから先はゴール側の制限のない範囲で、支柱もない)を、両方作り、短いほうを使う。
    2026-09-22: 従来は、直接の候補が進入禁止エリアに入らなければ、Cを経由する候補と比べず、
    そのまま使っていた。そのため、直接の候補が大回り(例: 他のゲートを通って、遠い場所から長い
    斜めでゴールへ向かう)になる配置でも、それが選ばれていた。
    """
    goal = config.GOAL_POS_CM
    direct = None
    try:
        direct = _resolve_top_level_segment(
            current, goal, all_gates, all_posts, None, prev_gate, gate_signs=gate_signs,
            strict_safe=True)
    except RuntimeError:
        direct = None
    if direct is not None and not _candidate_keepout_safe(direct):
        direct = None
    lane = config.KEEP_OUT_GOAL_LANE_POINT_CM
    if direct is not None:
        # Cを経由する候補は、どうやっても「現在地→C→ゴール」の直線距離より短くならない。
        # それが、直接の候補以上なら、比べる必要がない(計算時間を増やさないため)。
        if (geo.distance(current, lane) + geo.distance(lane, goal) >= _segment_result_length(direct) - 1e-6):
            return direct
        # Cを経由する候補は、安全な候補が作れたときだけ比べる(作れなければ、直接の候補を使う)。
        try:
            to_lane = _resolve_top_level_segment(
                current, lane, all_gates, all_posts, None, prev_gate, gate_signs=gate_signs,
                strict_safe=True)
        except RuntimeError:
            return direct
        via_lane = list(to_lane) + [(goal, None)]
        if _segment_result_length(via_lane) + 1e-6 < _segment_result_length(direct):
            return via_lane
        return direct
    # strict_only: 安全な候補が作れないときは、作れないこと(RuntimeError)を、そのまま返す。
    to_lane = _resolve_top_level_segment(
        current, lane, all_gates, all_posts, None, prev_gate, gate_signs=gate_signs,
        strict_safe=strict_only)
    return list(to_lane) + [(goal, None)]


def _build_route_with_signs(gates_by_color, signs, laps=None):
    """signs(build_stage_sequence()の各ステージに対応する+1/-1のリスト、
    全9ステージ=3周×3ゲート分。laps指定時はlaps×3ステージ)を使って
    経路を1本組み立てる。

    同じ色のゲートでも、周回によって「現在地」(直前のゲートの退出点)が
    異なれば、近い方の侵入側を単純に選ぶと周回ごとに異なる側を選んで
    しまうことが実際にある(例: 1周目はスタートから来るので近い側、
    2周目以降は別のゲートから来るので反対側、という具合)。そのため、
    色ごとに1つのsignではなく、ステージ(周回×色)ごとに独立したsignを
    持たせている。
    plan_route_with_anchors()の内部処理を、entry側の決め方だけ差し替え
    られるように切り出したもの。
    戻り値: (waypoints, anchors, labels, true_points)
    """
    stage_sequence = build_stage_sequence(gates_by_color, laps)
    all_gates = list(gates_by_color.values())
    all_posts = [p for g in all_gates for p in g.posts()]
    # 各ゲート(オブジェクトのid)が最終的にどちらの向きで通過されるかを
    # 事前に引けるようにしておく(_resolve_top_level_segmentが、target_gate
    # 以外のゲートを経由するフォールバックを試すときに、その経由ゲートの
    # 侵入方向を「これから公式に使う向き」と一致させるために使う)。
    gate_signs = {
        id(g): signs[i] for i, (_, _, g) in enumerate(stage_sequence)
    }

    waypoints = [config.START_POS_CM]
    labels = ["start"]
    true_points = [None]
    anchors = [None]

    current = config.START_POS_CM
    prev_gate = None
    stage_idx = 0
    while stage_idx < len(stage_sequence):
        lap, color, gate = stage_sequence[stage_idx]
        sign = signs[stage_idx]
        entry, exit_ = gate_entry_exit(gate, sign)

        segment = _resolve_top_level_segment(
            current, entry, all_gates, all_posts, gate, prev_gate, gate_signs=gate_signs
        )
        for pt, anchor in segment[1:-1]:
            waypoints.append(pt)
            labels.append(None)
            true_points.append(None)
            anchors.append(anchor)
        waypoints.append(entry)
        labels.append(f"lap{lap}-{color}-entry")
        true_points.append(entry)
        anchors.append(None)

        # 次のステージのentry/exitが、今のゲートのentry->exitと同一直線上に
        # 並んでしまい、今のexit(固定点)が次のゲートのentry-exit区間の
        # "内側"に来る配置(seed=237571: 赤の真上に黄があり、黄exitが
        # 赤entry-exit間に来るケース)では、通常通りexitで停止してから
        # 次のentryへ向かおうとすると、次のentryがexitより手前にあるため
        # ほぼ180度の反転が2回(exit到達時・次のentry到達時)必要になって
        # しまう。この場合、exit/次のentryを単独の停止点(=旋回点)にはせず、
        # 今のentryから次のexitまでを1本の直進としてつなぐ(exit・次の
        # entry自体は、正式な通過の記録としてはこの直進区間上の通過点と
        # して残る。_span_covers_gate_crossing/_maximal_collinear_runs参照)。
        merged = False
        if stage_idx + 1 < len(stage_sequence):
            next_lap, next_color, next_gate = stage_sequence[stage_idx + 1]
            next_sign = signs[stage_idx + 1]
            next_entry, next_exit = gate_entry_exit(next_gate, next_sign)
            covers_this = _span_covers_gate_crossing(entry, next_exit, gate)
            covers_next = _span_covers_gate_crossing(entry, next_exit, next_gate)
            if covers_this is not None and covers_next is not None:
                other_gates = [g for g in all_gates if g is not gate and g is not next_gate]
                if (_find_blocking_post(entry, next_exit, all_posts) is None
                        and not _segment_crosses_any_gate_leg(entry, next_exit, other_gates)):
                    # exit_(今のゲート)とnext_entry(次のゲート)は、どちらが
                    # 空間的に手前(entryに近い)かは配置次第(このケースでは
                    # next_entryの方がexit_より手前)なので、直線上の
                    # パラメータで実際の順番に並べ直す。
                    middle = [
                        (geo.collinear_param(entry, next_exit, exit_), exit_, f"lap{lap}-{color}-exit"),
                        (geo.collinear_param(entry, next_exit, next_entry), next_entry,
                         f"lap{next_lap}-{next_color}-entry"),
                    ]
                    middle.sort(key=lambda item: item[0])
                    for _, pt, lab in middle:
                        waypoints.append(pt)
                        labels.append(lab)
                        true_points.append(pt)
                        anchors.append(None)

                    waypoints.append(next_exit)
                    labels.append(f"lap{next_lap}-{next_color}-exit")
                    true_points.append(next_exit)
                    anchors.append(None)

                    current = next_exit
                    prev_gate = next_gate
                    stage_idx += 2
                    merged = True

        if not merged:
            waypoints.append(exit_)
            labels.append(f"lap{lap}-{color}-exit")
            true_points.append(exit_)
            anchors.append(None)

            current = exit_
            prev_gate = gate
            stage_idx += 1

    segment = _resolve_goal_segment(current, all_gates, all_posts, prev_gate, gate_signs)
    for pt, anchor in segment[1:-1]:
        waypoints.append(pt)
        labels.append(None)
        true_points.append(None)
        anchors.append(anchor)
    waypoints.append(config.GOAL_POS_CM)
    labels.append("goal")
    true_points.append(None)
    anchors.append(None)

    # 直進区間の支柱への接触(resolve_segmentの除外リストが後の再帰にまで
    # 残り続けてしまう不具合)と、ゲートの脚を結ぶ線を意図しない方向から
    # 横切ってしまう問題(「ゲートへは一方向からのみ侵入可能」というルール)
    # は、片方を直すともう片方が新たに崩れることがあるため、どちらも
    # 変化がなくなるまで交互に繰り返す。ただし、ゲート同士が近すぎて
    # 両方を同時には満たせない配置では、この2つが互いに直しあって
    # 点が際限なく増え続けることが実際にあったため、進展が見られない
    # (直進クリアランス側で新たに挿入が起きていない)場合や、上限回数に
    # 達した場合はそこで打ち切り、残った違反はverify_*関数で検出できる
    # 状態のまま返す。
    # 直進クリアランス/一方向侵入ルールの修正(交互ループ)と、旋回安全性の
    # 修正(_fix_pivot_violations)は、どちらも他方が直したはずの違反を
    # 再び壊しうる(実際に見つかった不具合: 旋回安全性の修正が点を
    # ゲートの方向に沿って動かした結果、その点が別ゲートの脚を逆方向に
    # 横切ってしまうケース)。そのため、この2つ全体をさらに外側で
    # 繰り返し、両方とも違反がなくなるか、進展がなくなるまで続ける。
    prev_total_len = None
    for _ in range(3):
        prev_len = len(waypoints)
        for _ in range(5):
            waypoints, anchors, labels, true_points = _resegment_straight_collisions(
                waypoints, anchors, labels, true_points, all_gates, all_posts
            )
            waypoints, anchors, labels, true_points = _fix_gate_crossings(
                waypoints, anchors, labels, true_points, all_gates
            )
            crossing_ok = not verify_gate_crossing_directions(waypoints, labels, all_gates)
            no_progress = len(waypoints) == prev_len
            prev_len = len(waypoints)
            if crossing_ok or no_progress:
                break

        # ループの最後の操作が_fix_gate_crossings(迂回点の挿入)だと、その
        # 迂回点自身、あるいはその前後の区間が新たに直進クリアランス違反を
        # 起こしていても、それを_resegment_straight_collisionsで再チェック
        # しないまま抜けてしまう(実際に見つかった不具合: 一方向侵入違反が
        # 0件になった直後のfix_gate_crossingsの挿入点が支柱にギリギリ近すぎる
        # ケース)。そのため、抜けた後にもう一度だけ直進クリアランスを
        # 再チェック・修正する。
        waypoints, anchors, labels, true_points = _resegment_straight_collisions(
            waypoints, anchors, labels, true_points, all_gates, all_posts
        )

        # 続いて旋回の安全性を確認し、実際に危険だった地点(=手前の侵入
        # ポイントとして作られた点)だけを、ピンポイントでオフセットを引き延ばして
        # 修正する(距離だけを見て一律に引き延ばすと、元々安全だった地点まで
        # 無駄に動かしてしまい、別の支柱との新たな衝突を生む不具合が実際に
        # 発生したため、実際に危険と判明した地点だけを直す方式にしている)。
        waypoints = _fix_pivot_violations(waypoints, anchors, all_posts, labels)

        crossing_ok = not verify_gate_crossing_directions(waypoints, labels, all_gates)
        straight_ok = not verify_straight_clearance(waypoints, all_posts, anchors, labels)
        no_progress = len(waypoints) == prev_total_len
        prev_total_len = len(waypoints)
        if (crossing_ok and straight_ok) or no_progress:
            break

    return waypoints, anchors, labels, true_points, all_posts


def _route_turn_sum_deg(waypoints, start_heading_deg=config.START_HEADING_DEG,
                         goal_heading_deg=config.GOAL_HEADING_DEG):
    """経路全体の旋回角(絶対値)の合計を返す。候補経路の評価に使う
    (距離だけで比較すると、実質その場でほぼ反転するような大きな旋回を
    伴う経路を、距離が短いというだけで誤って「良い」と判断してしまう
    ことが実際にあったため、config.TURN_COST_PER_DEGREE_CMで距離に
    換算して合算する)。
    """
    def heading(a, b):
        return math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))

    seg_headings = [heading(waypoints[i], waypoints[i + 1]) for i in range(len(waypoints) - 1)]
    final_heading = goal_heading_deg if (config.GOAL_FINAL_TURN or not seg_headings) else seg_headings[-1]
    headings = [start_heading_deg] + seg_headings + [final_heading]
    return sum(
        abs(geo.normalize_deg(headings[i + 1] - headings[i]))
        for i in range(len(headings) - 1)
    )


def _route_min_arm_body_clearance(waypoints, labels, all_posts):
    """各ゲートのentryへ向かう区間(直前のexit、またはスタートから、そのentryまで)の全ての直進について、
    そのゲート自身のT字パーツと車体との最短クリアランスを返す(_segment_body_clearance、最小値)。
    経路が、entryへの最後の直線だけでなく、その手前の直線も、自分のゲートのT字パーツに近づいていないかを見る。
    """
    worst = float("inf")
    stage_start = 0
    for i, label in enumerate(labels):
        if not label:
            continue
        if label.endswith("-entry"):
            color = label.split("-")[1]
            arm_posts = [p for p in all_posts
                         if getattr(p, "color", None) == color and (p.arm_dir[0] or p.arm_dir[1])]
            for k in range(stage_start, i):
                for p in arm_posts:
                    worst = min(worst, _segment_body_clearance(waypoints[k], waypoints[k + 1], p))
        elif label.endswith("-exit"):
            stage_start = i
    return worst


def plan_route_with_anchors(gates_by_color, laps=None):
    """plan_route()と同じ経路構築を行うが、各waypointのアンカー情報も返す
    (テスト・検証コードがverify_straight_clearanceにanchorsを渡して、
    意図的に支柱へ近づいている点を正しく除外できるようにするため)。

    laps: 周回数(省略時はconfig.LAPS=3)。2026-09-18: 地区大会で1周/2周を
    狙う可能性があるため、呼び出し側から周回数を指定できるようにした。
    ゲート通過順序ルール(GATE_ORDER)自体は変わらず、build_stage_sequence
    が作るステージ数がlaps×3になるだけで、以降の経路構築・妥当性検証
    (verify_gate_passage_order等)はステージ数に依存しない一般的な実装
    なので、laps=1でもlaps=2でも変更なしにそのまま動く。

    ゲートの侵入側(entry/exitのどちら側から通るか)は、色ごとに+1/-1の
    2択があり、赤・青・黄の3ゲートで2^3=8通りの組み合わせがある。
    「ゲートへは一方向からのみ侵入可能」というルール(公式なentry->exit
    自体も、周回によって向きが変わってはならない)があるため、同じ色の
    ゲートは3周とも必ず同じsignを使う(ステージ単位で独立にsignを
    選べるようにすると、周回によって貪欲に近い側を選んでしまい、
    同じゲートを別々の周回で逆方向から通過する結果になることが実際に
    あった)。
    8通り全てで経路を組み立て、「直進クリアランス違反0件」かつ
    「旋回安全性違反0件(ゴール到着時の旋回は_entry_arrival_pivot_safeが
    候補選定の時点で既に考慮しているが、それでも避けられない配置が
    万一残った場合に備え、妥当性判定では例外的に許容する)」かつ
    「どのゲートも公式な通過以外で脚を結ぶ線を横切っていない」かつ
    「competition規約5.17.3のゲート通過順序ルール上、3周とも成立する
    (verify_gate_passage_order参照)」を満たす候補の中から、
    総移動距離 + 旋回角合計×TURN_COST_PER_DEGREE_CM が最小のものを選ぶ。
    単純な距離だけで比較すると、実質その場でほぼ反転するような経路
    (角度は変えずに黄色ゲートに正面から突っ込むように侵入方向を選んだ
    結果、entry地点でほぼ180度旋回が必要になるケース)を、距離が短いと
    いうだけで誤って選んでしまう不具合が実際に見つかったため、旋回角も
    評価に含めている。
    2026-09-15: ゲート通過順序を優先度の最上位(is_validの次)に置いている。
    経路生成側の迂回ロジック自体はゲート通過順序を考慮していない
    (verify_gate_passage_orderのdocstring参照)ため、8通りのうち
    成立周回数が最大のsignの組み合わせを選ぶことで、選べるならこの問題を
    回避する。ただし8通り全てで周回が壊れる配置では、この方式では解消
    しきれずに残ることがある。
    どの組み合わせも「ゴール以外」の違反を解消できず、かつ3周成立も
    できなかった場合は、違反件数が最も少ない・成立周回数が最も多い候補を
    (それでも直せなかった旨はverify_pivot_safety/verify_straight_clearance/
    verify_gate_crossing_directions/verify_gate_passage_orderで別途確認
    できる形で)返す。

    戻り値: (waypoints, anchors, labels, true_points)
    """
    stage_sequence = build_stage_sequence(gates_by_color, laps)
    colors = list(dict.fromkeys(color for _, color, _ in stage_sequence))  # 出現順・重複なし
    all_gates = list(gates_by_color.values())

    best = None  # (is_valid, violation_count, score, waypoints, anchors, labels, true_points)
    all_candidates = []
    _ARM_SEARCH_CACHE.clear()
    for combo in itertools.product((1.0, -1.0), repeat=len(colors)):
        color_signs = dict(zip(colors, combo))
        signs = [color_signs[color] for _, color, _ in stage_sequence]
        try:
            waypoints, anchors, labels, true_points, all_posts = _build_route_with_signs(
                gates_by_color, signs, laps
            )
        except RuntimeError:
            continue

        pivot_violations = verify_pivot_safety(waypoints, all_posts)
        straight_violations = verify_straight_clearance(waypoints, all_posts, anchors, labels)
        crossing_violations = verify_gate_crossing_directions(waypoints, labels, all_gates)
        completed_laps = verify_gate_passage_order(waypoints, all_gates)
        # ゴール到着時の旋回安全性は_entry_arrival_pivot_safeが候補選定の
        # 時点で既に考慮しているため、通常はここで違反として残らない。
        # それでも万一残った場合に備えた安全弁として、妥当性判定からは除外する。
        non_goal_pivot = [v for v in pivot_violations if labels[v[0]] != "goal"]
        keepout_violations = verify_keepout(waypoints)
        is_valid = (not straight_violations and not non_goal_pivot and not crossing_violations
                    and not keepout_violations)
        violation_count = (len(straight_violations) + len(non_goal_pivot) + len(crossing_violations)
                           + len(keepout_violations))

        dist = sum(geo.distance(waypoints[i], waypoints[i + 1]) for i in range(len(waypoints) - 1))
        turn_sum = _route_turn_sum_deg(waypoints)
        score = dist + config.TURN_COST_PER_DEGREE_CM * turn_sum

        # is_valid/violation_countの定義・優先順位(物理安全性)は変えず、
        # completed_lapsをそれらの"後"(-scoreの"前")に挿入する。
        # 2026-09-15の最初の実装ではcompleted_lapsをviolation_countより
        # 先に比較していたが、これだと「8通り全部で物理違反が出る」配置で
        # 周回成立を安全性より優先してしまい、stress_test側で186/200まで
        # 後退した。物理安全性は既存通り最優先のまま、violation_countが
        # 同点の候補同士の中でだけ周回成立数を比較する形に修正した。
        # 8通りの候補生成ロジック自体には一切手を入れていない。
        candidate = (is_valid, -violation_count, completed_laps, -score, waypoints, anchors, labels, true_points)
        if best is None or candidate[:4] > best[:4]:
            best = candidate
        arm_ok = _route_min_arm_body_clearance(waypoints, labels, all_posts) >= config.ARM_BODY_CLEARANCE_MIN_CM - 1e-9
        all_candidates.append(candidate + (arm_ok, dist))

    if best is None:
        raise RuntimeError("どの侵入側の組み合わせでも経路を構築できませんでした")

    # 2026-09-22: 最良の組み合わせが、どこかのゲートで、自分のT字パーツへの余裕(3cm)を満たしていないとき、
    # 同じ安全性・周回数の組み合わせのうち、全てのゲートで余裕を満たすものが、
    # 増える長さ(スコアの差)ARM_SIGN_PREFERENCE_BUDGET_CM以内にあれば、そちらを使う。
    if config.ARM_SIGN_PREFERENCE_BUDGET_CM > 0:
        best_entry = next(c for c in all_candidates if c[:8] == best)
        if not best_entry[8]:
            # 増える長さは、スコア(旋回のコストを含む)ではなく、実際の距離で比べる。
            same_safety = [c for c in all_candidates
                           if c[8] and c[:3] == best[:3] and c[9] <= best_entry[9] + config.ARM_SIGN_PREFERENCE_BUDGET_CM]
            if same_safety:
                best = max(same_safety, key=lambda c: c[3])[:8]

    _, _, _, _, waypoints, anchors, labels, true_points = best

    return waypoints, anchors, labels, true_points


def verify_pivot_safety(waypoints, all_posts, start_heading_deg=config.START_HEADING_DEG,
                         goal_heading_deg=config.GOAL_HEADING_DEG):
    """経路上の各旋回地点で、その場旋回が支柱に接触しないかを機械的に確認する
    (pivot_turn_safeを流用した事後検証。経路構築の各段階
    (_entry_arrival_pivot_safeなどの候補選定時チェック、_fix_pivot_violations
    による修正)でも旋回安全性を確保しようとするが、それでも直せなかった
    違反が残っていないかを最終確認するためのもの)。
    戻り値: 違反のリスト[(index, point, turn_deg), ...] (空なら全て安全)。
    """

    def heading(a, b):
        return math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))

    violations = []
    seg_headings = [heading(waypoints[i], waypoints[i + 1]) for i in range(len(waypoints) - 1)]
    final_heading = goal_heading_deg if (config.GOAL_FINAL_TURN or not seg_headings) else seg_headings[-1]
    headings = [start_heading_deg] + seg_headings + [final_heading]
    # headings[i]は「waypoints[i]に到着するときの向き」、headings[i+1]は
    # 「waypoints[i]から出ていくときの向き」に対応する(headings[0]=start_heading_deg)。
    for i in range(len(waypoints)):
        h_in = headings[i]
        h_out = headings[i + 1]
        if abs(geo.normalize_deg(h_out - h_in)) < 1e-6:
            continue
        if not pivot_turn_safe(waypoints[i], h_in, h_out, all_posts):
            violations.append((i, waypoints[i], geo.normalize_deg(h_out - h_in)))
    return violations
