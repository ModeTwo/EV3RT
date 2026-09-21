"""ETラリーの経路(strategy)に対する、実機の癖の暫定補正。

sample_comment.py(ETラリー用にカスタムしたスタンドアロン版)のapply_caster_drag_
compensation/apply_lateral_drift_compensationを移植したもの。どちらも各stepの
distance_mm/target_heading_degだけを書き換えた新しいリストを返し、元のリストは
変更しない。ETラリー工程(features/execute_strategy.py)以外からは使わない。

initial_heading_deg: 経路の最初の旋回の「直前の向き」(ジャイロ座標系)。
sample_comment.pyは常に0度だったが、robot_programは開始ミッションごとに基準が
異なる(shared_communication.heading_frame.full_start_to_gyro)ため引数にした。
"""

import math


def apply_caster_drag_compensation(steps: list, mm_per_deg: float = 0.0944,
                                    initial_heading_deg: float = 0.0) -> list:
    """その場旋回のたびに、ロボット後方のボールキャスターが(横方向にしか転がりに
    くいため)車体を後方に引きずる問題への暫定的な補正。

    2026-09-12〜14の実機テスト(その場旋回だけを繰り返し、位置ズレを実測)で判明:
    - ±90度を交互に30回繰り返すと、後方に24cm/24.5cm/28cmの一貫したズレ
      (平均25.5cm、1回の90度旋回あたり約8.5mm)
    - 同方向に90度を28回連続(4回=360度ごとに元の向きへ戻る)だと、ズレはわずか
      約2cmまで激減 → ズレの向きがロボット自身の向きと一緒に回転している証拠
      (床やコースなどグローバルな要因ではなく、車体に固定されたキャスター由来と
      ほぼ確定)
    - 手でキャスターのボールを回すと、前後方向はスムーズだが左右方向にひっかかりが
      あることも確認済み(その場旋回はキャスターを左右方向に転がす動きを要求する
      ため、症状と一致)

    本番のコースは旋回の向き・角度がバラバラに混ざるため、このズレは打ち消し
    合わずに実質的に残り続ける前提で、各turnステップの直後にあるmoveステップの
    距離を、その旋回角度に比例した分だけ延長して打ち消す。補正の向きは「旋回後の
    新しい進行方向」=直後のmoveの向きと同じなので、そのmoveの距離に足すだけで済む。
    直後にmoveが続かない旋回(ゴールでの向き合わせなど)は位置に影響しないため
    補正しない。

    キャスターの清掃・潤滑などで機構側を直せば、この補正量自体を減らす/不要に
    できる可能性がある(根本対策はそちらが本命、これは暫定処置)。

    mm_per_deg: 90度旋回あたり実測約8.5mmから逆算(8.5mm / 90度 ≈ 0.0944mm/度)。
    0を渡せば補正を無効化できる。
    """
    compensated = []
    current_heading = initial_heading_deg
    pending_mm = 0.0
    for step in steps:
        step = dict(step)
        if step["type"] == "turn":
            delta = (step["target_heading_deg"] - current_heading + 180.0) % 360.0 - 180.0
            current_heading = step["target_heading_deg"]
            pending_mm = abs(delta) * mm_per_deg
        elif step["type"] == "move":
            if pending_mm > 0.0:
                step["distance_mm"] = step["distance_mm"] + pending_mm
                pending_mm = 0.0
        compensated.append(step)
    return compensated




def apply_lateral_drift_compensation(steps: list, mm_per_deg_a: float = 0.00496,
                                      mm_per_deg_b: float = 0.01278,
                                      sign: float = 1.0, max_correction_deg: float = 5.0,
                                      initial_heading_deg: float = 0.0) -> list:
    """その場旋回のたびに横方向(左右)へも車体がズレる問題への補正。

    背景(2026-09-19〜20、et_rally_planner環境でその場旋回のみを繰り返す
    テストを実施): apply_caster_drag_compensation(前後方向)は元々「後方への
    引きずり」だけをモデル化していたが、実際には**同方向に旋回を28回連続
    (7周ぶん)させただけでも、常に一定方向(「右」)へのズレが残る**ことが
    判明した。しかも旋回の向き(delta>0のA方向/delta<0のB方向)によって
    ズレの大きさが約2.1倍違う:
      - A方向(target_heading_degが増える向き): 8回の試行平均で1.25cm/28回
        (0.00496mm/度)、値は非常に安定(1.2〜1.3cmの範囲)。
      - B方向(target_heading_degが減る向き): 11回の試行のうち、最初の3回
        (4.9cm平均)だけ明確に高く、残り8回は2.0〜2.9cm(平均2.59cm)に
        まとまった。
    前後方向の引きずりと違い、A/CCW-B(前後で符号が逆だった)とは別の性質で、
    横方向はA/Bどちらの旋回方向でも同じ側(「右」)へズレる。そのため、この
    補正は旋回方向に関わらず常に同じ向き(sign)で、大きさだけを旋回方向
    (delta>0かdelta<0か)で使い分ける。

    実現方法: 非ホロノミックな車体は横方向に直接動けないため、旋回直後の
    moveの目標方位(target_heading_deg)を、そのmoveの距離に応じてわずかに
    傾けることで、直進しながら横ズレを打ち消す(distance_mm × sin(補正角)
    ≈ 打ち消したい横ズレ量、という近似)。moveの距離が短すぎると必要な
    補正角が大きくなりすぎるため、max_correction_degで上限をかける
    (それ以上は打ち消しきれず残る)。

    sign: 2026-09-20、実機の本番経路テスト(X座標が複数回とも完璧に一致)で
    sign=1.0が正しい向きだと確認済み。

    mm_per_deg_b: 2026-09-20、B方向係数を後半8回のクラスター(2.59cm/0.01028
    mm/度)で運用したところ、本番経路(plan_seed1650169_right_laps2.json、
    2周)を4回実走したうちY座標が3回とも+5cm前後の残差を残した
    (区間ごとの旋回方向を解析すると、Y方向の区間はX方向の区間よりB方向
    係数への依存度が高い(44.7% vs 37.5%)ため、この残差はB方向の過小補正が
    一因の可能性が高いと判断)。そのため、後半8回だけでなく最初の3回の
    外れ値も含めた11回全体の平均3.22cm(0.01278mm/度)に引き上げて再検証する。
    """
    compensated = []
    current_heading = initial_heading_deg
    pending_lateral_mm = 0.0
    for step in steps:
        step = dict(step)
        if step["type"] == "turn":
            delta = (step["target_heading_deg"] - current_heading + 180.0) % 360.0 - 180.0
            current_heading = step["target_heading_deg"]
            if delta == 0.0:
                pending_lateral_mm = 0.0
            else:
                rate = mm_per_deg_a if delta > 0.0 else mm_per_deg_b
                pending_lateral_mm = abs(delta) * rate
        elif step["type"] == "move":
            if pending_lateral_mm > 0.0 and step["distance_mm"] > 1e-6:
                correction_deg = sign * math.degrees(pending_lateral_mm / step["distance_mm"])
                correction_deg = max(-max_correction_deg, min(max_correction_deg, correction_deg))
                step["target_heading_deg"] = step["target_heading_deg"] + correction_deg
                pending_lateral_mm = 0.0
        compensated.append(step)
    return compensated

