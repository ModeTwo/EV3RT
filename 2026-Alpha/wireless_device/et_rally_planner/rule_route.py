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
    """gateを法線方向signの側(+1 or -1)から通過する場合のentry/exitを返す。"""
    n = geo.scale(gate.normal, sign)
    entry = geo.add(gate.center, geo.scale(n, config.ROBOT_FRONT_OVERHANG_CM))
    exit_ = geo.sub(gate.center, geo.scale(n, config.ROBOT_REAR_OVERHANG_CM))
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


def _find_blocking_post(a, b, all_posts, exclude=(), anchor_a_post=None, anchor_b_post=None):
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
        d = geo.point_segment_distance(post, a, b)
        if d >= STRAIGHT_CLEARANCE_CM:
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
        needed_clearance = STRAIGHT_CLEARANCE_CM
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
            d = min(d, geo.point_segment_distance(target_post, prev_pt, new_point))
        if next_pt is not None:
            effective_next = geo.add(next_pt, shift) if next_pt_propagates else next_pt
            d = min(d, geo.point_segment_distance(target_post, new_point, effective_next))
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
            d = geo.point_segment_distance(p, a_pt, b_pt)
            if d >= STRAIGHT_CLEARANCE_CM:
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
            blocking_post = _find_blocking_post(
                a, b, all_posts, anchor_a_post=anchor_a_post, anchor_b_post=anchor_b_post
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
                if _extend_point_for_straight_clearance(waypoints, anchors, i, all_posts, blocking_post, owner_gate):
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


def verify_straight_clearance(waypoints, all_posts, anchors=None):
    """経路上の各直進区間が、支柱に十分な距離(STRAIGHT_CLEARANCE_CM)を
    保っているかを機械的に確認する。anchorsを渡した場合、その区間の
    端点自身のアンカー支柱は、実際にその端点の真近く(t<=0 または t>=1)
    でしか近づいていない場合に限って除外する(意図的に近い点のため)。

    以前は「アンカーだから無条件に除外する」という単純な除外をしていたが、
    それだと区間の途中(tが0や1から離れた位置)で実際に支柱に接触している
    ケースまで見逃してしまう不具合があった(_resegment_straight_collisions側の
    _find_blocking_postでは元々t考慮していたが、こちらの検証用関数だけ
    考慮していなかったため、実際には直っていない違反を「0件」と誤って
    報告していた)。
    戻り値: 違反のリスト[(index, post, dist), ...] (空なら全て安全)。
    """
    violations = []
    for i in range(len(waypoints) - 1):
        a, b = waypoints[i], waypoints[i + 1]
        anchor_a_post = anchors[i][0] if anchors is not None and anchors[i] is not None else None
        anchor_b_post = anchors[i + 1][0] if anchors is not None and anchors[i + 1] is not None else None
        for post in all_posts:
            d = geo.point_segment_distance(post, a, b)
            if d >= STRAIGHT_CLEARANCE_CM - 1e-9:
                continue
            t_raw = _project_param(a, b, post)
            if anchor_a_post is not None and geo.distance(post, anchor_a_post) < 1e-6 and t_raw <= 0:
                continue
            if anchor_b_post is not None and geo.distance(post, anchor_b_post) < 1e-6 and t_raw >= 1:
                continue
            violations.append((i, post, d))
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


def _fix_pivot_violations(waypoints, anchors, all_posts, max_iterations=5):
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
    """
    waypoints = list(waypoints)
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
    return waypoints


def plan_route(gates_by_color):
    """ルールベースで全waypointsを構築する。

    戻り値: (waypoints, total_dist_cm, labels, true_points)
    既存パイプライン(commands.py, generate_diagram.py)とインターフェースを
    揃えるため、旧plan_path_detailed()と同じ形式で返す。
    entry/exitは常にゲート中心を通る「本当の」接触点そのものなので、
    true_pointsはwaypointsと同じ値になる。
    """
    waypoints, anchors, labels, true_points = plan_route_with_anchors(gates_by_color)
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


def _candidate_crossing_safe(result, all_gates, gate_signs):
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
    """
    points = [pt for pt, _ in result]
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
            return False
    return True


def _resolve_via_other_gate(a, b, all_gates, all_posts, target_gate, other_gate, other_sign, gate_signs=None):
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
    """
    other_entry, other_exit = gate_entry_exit(other_gate, other_sign)
    try:
        to_other = _resolve_top_level_segment(
            a, other_entry, all_gates, all_posts, other_gate, None,
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


def _resolve_top_level_segment(a, b, all_gates, all_posts, target_gate, prev_gate, gate_signs=None,
                                allow_via_other=True):
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
    except RuntimeError:
        pass

    axis_result = None
    try:
        axis_result = _resolve_segment_via_parallel_extension(
            a, b, all_gates, all_posts, target_gate, prev_gate
        )
    except RuntimeError:
        pass

    candidates = [r for r in (rect_result, axis_result) if r is not None]

    def _fully_safe(r):
        return (
            _entry_arrival_pivot_safe(r, target_gate, all_posts)
            and _departure_pivot_safe(r, prev_gate, gate_signs, all_posts)
            and _candidate_internal_pivot_safe(r, all_posts)
            and _candidate_crossing_safe(r, all_gates, gate_signs)
            and _candidate_straight_clear_safe(r, all_posts)
        )

    safe_candidates = [r for r in candidates if _fully_safe(r)]

    if gate_signs is not None and allow_via_other:
        for other_gate in all_gates:
            if other_gate is target_gate:
                continue
            other_sign = gate_signs.get(id(other_gate))
            if other_sign is None:
                continue
            via_result = _resolve_via_other_gate(
                a, b, all_gates, all_posts, target_gate, other_gate, other_sign,
                gate_signs=gate_signs,
            )
            if via_result is not None and _fully_safe(via_result):
                safe_candidates.append(via_result)

    if not candidates and not safe_candidates:
        raise RuntimeError("迂回経路を見つけられませんでした")

    pool = safe_candidates if safe_candidates else candidates
    return min(pool, key=_segment_result_length)


def _build_route_with_signs(gates_by_color, signs):
    """signs(build_stage_sequence()の各ステージに対応する+1/-1のリスト、
    全9ステージ=3周×3ゲート分)を使って経路を1本組み立てる。

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
    stage_sequence = build_stage_sequence(gates_by_color)
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
    for stage_idx, (lap, color, gate) in enumerate(stage_sequence):
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

        waypoints.append(exit_)
        labels.append(f"lap{lap}-{color}-exit")
        true_points.append(exit_)
        anchors.append(None)

        current = exit_
        prev_gate = gate

    segment = _resolve_top_level_segment(
        current, config.GOAL_POS_CM, all_gates, all_posts, None, prev_gate, gate_signs=gate_signs
    )
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
        waypoints = _fix_pivot_violations(waypoints, anchors, all_posts)

        crossing_ok = not verify_gate_crossing_directions(waypoints, labels, all_gates)
        straight_ok = not verify_straight_clearance(waypoints, all_posts, anchors)
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

    headings = [start_heading_deg] + [
        heading(waypoints[i], waypoints[i + 1]) for i in range(len(waypoints) - 1)
    ] + [goal_heading_deg]
    return sum(
        abs(geo.normalize_deg(headings[i + 1] - headings[i]))
        for i in range(len(headings) - 1)
    )


def plan_route_with_anchors(gates_by_color):
    """plan_route()と同じ経路構築を行うが、各waypointのアンカー情報も返す
    (テスト・検証コードがverify_straight_clearanceにanchorsを渡して、
    意図的に支柱へ近づいている点を正しく除外できるようにするため)。

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
    「どのゲートも公式な通過以外で脚を結ぶ線を横切っていない」を満たす
    候補の中から、総移動距離 + 旋回角合計×TURN_COST_PER_DEGREE_CM が
    最小のものを選ぶ。
    単純な距離だけで比較すると、実質その場でほぼ反転するような経路
    (角度は変えずに黄色ゲートに正面から突っ込むように侵入方向を選んだ
    結果、entry地点でほぼ180度旋回が必要になるケース)を、距離が短いと
    いうだけで誤って選んでしまう不具合が実際に見つかったため、旋回角も
    評価に含めている。
    どの組み合わせも「ゴール以外」の違反を解消できなかった場合は、
    違反件数が最も少ない候補を(それでも直せなかった旨は
    verify_pivot_safety/verify_straight_clearance/verify_gate_crossing_
    directionsで別途確認できる形で)返す。

    戻り値: (waypoints, anchors, labels, true_points)
    """
    stage_sequence = build_stage_sequence(gates_by_color)
    colors = list(dict.fromkeys(color for _, color, _ in stage_sequence))  # 出現順・重複なし
    all_gates = list(gates_by_color.values())

    best = None  # (is_valid, violation_count, score, waypoints, anchors, labels, true_points)
    for combo in itertools.product((1.0, -1.0), repeat=len(colors)):
        color_signs = dict(zip(colors, combo))
        signs = [color_signs[color] for _, color, _ in stage_sequence]
        try:
            waypoints, anchors, labels, true_points, all_posts = _build_route_with_signs(
                gates_by_color, signs
            )
        except RuntimeError:
            continue

        pivot_violations = verify_pivot_safety(waypoints, all_posts)
        straight_violations = verify_straight_clearance(waypoints, all_posts, anchors)
        crossing_violations = verify_gate_crossing_directions(waypoints, labels, all_gates)
        # ゴール到着時の旋回安全性は_entry_arrival_pivot_safeが候補選定の
        # 時点で既に考慮しているため、通常はここで違反として残らない。
        # それでも万一残った場合に備えた安全弁として、妥当性判定からは除外する。
        non_goal_pivot = [v for v in pivot_violations if labels[v[0]] != "goal"]
        is_valid = not straight_violations and not non_goal_pivot and not crossing_violations
        violation_count = len(straight_violations) + len(non_goal_pivot) + len(crossing_violations)

        dist = sum(geo.distance(waypoints[i], waypoints[i + 1]) for i in range(len(waypoints) - 1))
        turn_sum = _route_turn_sum_deg(waypoints)
        score = dist + config.TURN_COST_PER_DEGREE_CM * turn_sum

        candidate = (is_valid, -violation_count, -score, waypoints, anchors, labels, true_points)
        if best is None or candidate[:3] > best[:3]:
            best = candidate

    if best is None:
        raise RuntimeError("どの侵入側の組み合わせでも経路を構築できませんでした")

    _, _, _, waypoints, anchors, labels, true_points = best

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
    headings = [start_heading_deg] + [
        heading(waypoints[i], waypoints[i + 1]) for i in range(len(waypoints) - 1)
    ] + [goal_heading_deg]
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
