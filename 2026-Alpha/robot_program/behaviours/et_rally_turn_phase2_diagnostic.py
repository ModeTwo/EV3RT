"""ETラリー旋回の、フェーズ2(ジャイロ仕上げ)が位置ズレ(特にB方向の横ズレ、
原因未特定)の原因になっていないかを切り分けるための、実機診断専用ツール。

【考え方】
et_rally_drive.pyのdocstring(83-89行目)に記録されている、過去に一度使った
切り分け手法(fine_tolerance_degを緩めて、フェーズ2をほぼ発動させない状態で
同じ旋回テストをやり直す)と同じ考え方を、今回のB方向横ズレにも適用する。

比較対象は、既存の`robot_program/tests/plan_spin_alternating30.json`
(--rally-plan-path で実行、fine_tolerance_deg=0.5の通常設定)。
このミッションは、全く同じ角度パターン(+-90度を30回)を、フェーズ2を
ほぼ無効化した状態(fine_tolerance_deg=8.0)で走らせる。

【使い方】
1. python alpha.py ... --rally-plan-path robot_program/tests/plan_spin_alternating30.json
   を実行し、開始位置と終了位置のズレ(特に横方向)を実測する。
2. python alpha.py ... --mission et-rally-turn-phase2-diagnostic
   を実行し、同様に実測する。
3. 2つの横ズレを比較する:
   - ほぼ同じなら、フェーズ2(≒LQIへの置き換え)は横ズレの原因ではない
     (フェーズ1側、または別の要因を疑う)。
   - 2の方が明確に小さいなら、フェーズ2の動き(細かい往復)が横ズレの一因である
     可能性が高く、その場合はLQI化や一発補正化の投資に意味が出てくる。

本番経路(features/execute_strategy.py)は変更していない。診断専用。
"""
from py_trees.composites import Sequence

from ..features.execute_strategy import SPIN_MAX_POWER
from ..types import HeadingType
from .et_rally_drive import EtRallySpinAroundByEncoder

# 本番の旋回パラメータ(execute_strategy.pyの既定値)と揃える。
# fine_tolerance_degだけ、過去の切り分け実績(8度)に合わせて緩める。
_MAIN_POWER = SPIN_MAX_POWER
_FINE_MAX_POWER = 60
_FINE_MIN_POWER = 50
_TURN_PID = (0.2, 0.00075, 0.03)
_DIAGNOSTIC_FINE_TOLERANCE_DEG = 8.0  # 2026-09-12の切り分けと同じ値(docstring参照)

# plan_spin_alternating30.jsonと同じパターン: +90/0度を30回繰り返す(alternating)。
_REPEAT_COUNT = 15  # 90->0を1往復として15回 = ターン数30(0/90を30回、と同数)


def build_et_rally_turn_phase2_diagnostic_sequence(
        repeat_count: int = _REPEAT_COUNT,
        fine_tolerance_deg: float = _DIAGNOSTIC_FINE_TOLERANCE_DEG) -> Sequence:
    root = Sequence(name="et_rally turn phase2 diagnostic", memory=True)
    children = []
    for i in range(repeat_count):
        for heading in (90.0, 0.0):
            children.append(
                EtRallySpinAroundByEncoder(
                    name="phase2_diag turn%d_%d" % (i, int(heading)),
                    target=heading,
                    main_power=_MAIN_POWER,
                    fine_max_power=_FINE_MAX_POWER,
                    fine_min_power=_FINE_MIN_POWER,
                    pid_p=_TURN_PID[0],
                    pid_i=_TURN_PID[1],
                    pid_d=_TURN_PID[2],
                    target_type=HeadingType.ABSOLUTE,
                    fine_tolerance_deg=fine_tolerance_deg,
                )
            )
    root.add_children(children)
    return root
