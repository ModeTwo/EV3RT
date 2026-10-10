"""Feature 12 subtree factory."""

import json
import math
from shared_communication.heading_frame import RALLY_START_HEADING, full_start_to_gyro
from pathlib import Path

from .bt_imports import Failure, HeadingType, Parallel, ParallelPolicy, Sequence
from ..behaviours.et_rally_drive import EtRallyRunByGyro, EtRallySpinAroundByEncoder
from ..behaviours.et_rally_turn_one_shot import EtRallyTurnOneShot
from ..behaviours.et_rally_turn_lqi import EtRallyTurnLqi
from ..behaviours.et_rally_turn_direction_test import EtRallySpinAroundDirectionTest
from ..behaviours.gyro_drive import SpinAround
from ..behaviours.conditions import IsDistanceEarned
from ..runtime import runtime
from ..et_rally_compensation import apply_caster_drag_compensation, apply_lateral_drift_compensation
from ..et_rally_fallback import fallback_planner_steps, fallback_strategy


# DD-19較正: --rally-turn-method で選べる旋回方式。spin_pidだけ構成要素(name,target,
# max_power,min_power,pid_p,pid_i,pid_d,target_type)がほかと異なるため、呼び出し側で分岐する。
# spin_encoder_dir_test: 検証専用(2026-10-10)。各turnステップの"direction"(+1/-1)で
# 回転方向を明示できる、EtRallySpinAroundByEncoderの検証用コピーを使う。本番では使わない。
RALLY_TURN_METHODS = ('spin_pid', 'spin_encoder', 'one_shot', 'lqi', 'spin_encoder_dir_test')


SPIN_MAX_POWER = 70         # その場回旋（スピン）するときの最大モーター出力
SPIN_MIN_POWER = 60         # その場回旋（スピン）するときの最低モーター出力
PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_PLAN_PATH = Path(__file__).resolve().parents[1] / "tests" / "plan_seed9392783.json"


class DeferredStrategySequence(Sequence):
    """Build motor behaviours only when this subtree starts running."""

    def __init__(self, name, loader, initial_heading_deg=0.0, fallback_loader=None,
                 turn_method='spin_encoder', compensation_enabled=True,
                 turn_fine_tolerance_deg=None, turn_decel_deg=None,
                 turn_backlash_preload_deg=None):
        super().__init__(name=name, memory=True)
        self._loader = loader
        # 経路をノードに変換できなかったとき(ETラリー開始時のエラー)に、代わりに使う固定ルート。
        self._fallback_loader = fallback_loader
        self._initial_heading_deg = initial_heading_deg
        self._turn_method = turn_method
        self._compensation_enabled = compensation_enabled
        self._turn_fine_tolerance_deg = turn_fine_tolerance_deg
        self._turn_decel_deg = turn_decel_deg
        self._turn_backlash_preload_deg = turn_backlash_preload_deg
        self._loaded = False

    def _load_children(self):
        # Sequence.tick()が現在の子を決める前に展開する。
        # initialise()内で追加すると、py_treesのcurrent_childとchildrenが
        # 食い違い「unknown / invalid state」になる。
        if self._loaded:
            return
        extra_kwargs = {}
        if self._turn_fine_tolerance_deg is not None:
            extra_kwargs["turn_fine_tolerance_deg"] = self._turn_fine_tolerance_deg
        if self._turn_decel_deg is not None:
            extra_kwargs["turn_decel_deg"] = self._turn_decel_deg
        if self._turn_backlash_preload_deg is not None:
            extra_kwargs["turn_backlash_preload_deg"] = self._turn_backlash_preload_deg
        try:
            nodes = steps_from_strategy(
                self._loader(), initial_heading_deg=self._initial_heading_deg,
                turn_method=self._turn_method, compensation_enabled=self._compensation_enabled,
                **extra_kwargs)
        except (KeyError, OSError, TypeError, ValueError) as error:
            self.logger.error("Strategy loading failed: %s" % error)
            nodes = [Failure(name="invalid strategy")]
            if self._fallback_loader is not None:
                try:
                    nodes = steps_from_strategy(
                        self._fallback_loader(), initial_heading_deg=self._initial_heading_deg,
                        turn_method=self._turn_method, compensation_enabled=self._compensation_enabled,
                        **extra_kwargs)
                    self.logger.error("ETラリー開始時にエラー(%s)。固定ルートでゴールへ向かいます" % error)
                except (KeyError, OSError, TypeError, ValueError) as fallback_error:
                    self.logger.error("Fallback route failed: %s" % fallback_error)
        self.add_children(nodes)
        self._loaded = True

    def tick(self):
        # 受信SEQはBT構築後にcontextへ入るため、最初のtick直前まで展開を遅らせる。
        self._load_children()
        yield from super().tick()


def build_execute_strategy(context, config, lap_number=None):
    # No.12は、PCが作成した全周回分のSEQを一度だけ実行する。
    source = config.et_rally_strategy_source
    if source == "received":
        # 補正関数が使う「最初の旋回の直前の向き」。受信SEQはラリー開始向きを基準に
        # 作られており、ジャイロ座標系での開始向きはmission_modeで決まる。
        start_heading = full_start_to_gyro(
            [{"type": "turn", "target_heading_deg": RALLY_START_HEADING}],
            config.mission_mode,
        )[0]["target_heading_deg"]
        return DeferredStrategySequence(
            name="execute_received_strategy",
            loader=lambda: full_start_to_gyro(context.strategy, config.mission_mode),
            initial_heading_deg=start_heading,
            fallback_loader=(
                (lambda: full_start_to_gyro(fallback_strategy(), config.mission_mode))
                if getattr(config, "et_rally_fallback_enabled", False) else None),
            turn_method=getattr(config, "et_rally_turn_method", "spin_encoder"),
            compensation_enabled=getattr(config, "et_rally_compensation_enabled", True),
            turn_fine_tolerance_deg=getattr(config, "et_rally_turn_fine_tolerance_deg", None),
            turn_decel_deg=getattr(config, "et_rally_turn_decel_deg", None),
            turn_backlash_preload_deg=getattr(config, "et_rally_turn_backlash_preload_deg", None),
        )
    if source == "file":
        plan_path = _resolve_plan_path(config.et_rally_plan_path)
        return DeferredStrategySequence(
            name="execute_file_strategy",
            loader=lambda: _load_plan_steps(plan_path),
            fallback_loader=(fallback_planner_steps
                             if getattr(config, "et_rally_fallback_enabled", False) else None),
            turn_method=getattr(config, "et_rally_turn_method", "spin_encoder"),
            compensation_enabled=getattr(config, "et_rally_compensation_enabled", True),
            turn_fine_tolerance_deg=getattr(config, "et_rally_turn_fine_tolerance_deg", None),
            turn_decel_deg=getattr(config, "et_rally_turn_decel_deg", None),
            turn_backlash_preload_deg=getattr(config, "et_rally_turn_backlash_preload_deg", None),
        )
    raise ValueError("Unknown ET rally strategy source: " + str(source))


def _resolve_plan_path(configured_path):
    # 相対パスは2026-Alpha直下を基準にし、起動ディレクトリへ依存させない。
    if configured_path is None:
        return DEFAULT_PLAN_PATH
    path = Path(configured_path)
    return path if path.is_absolute() else PROJECT_ROOT / path


def _load_plan_steps(plan_path):
    with open(plan_path, encoding="utf-8") as file:
        plan = json.load(file)
    if not isinstance(plan, dict) or "steps" not in plan:
        raise ValueError("Plan must contain steps")
    return plan["steps"]


def _finite_number(value, field_name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(field_name + " must be a number")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(field_name + " must be finite")
    return value


def steps_from_strategy(strategy, move_power=80, move_pid=(4.0, 0.6, 0.06),
                        # 2026-09(会話にもとづく): 発進・停止それぞれの直前直後だけを
                        # 短く加減速する。区間全体をゆっくり加減速するのではない
                        # (長い区間でも、この距離分しか加減速しない)。
                        # 【要実機調整】30mm/半分のパワー、はまだ最初の見立てで、
                        # 実測して詰める前提の値。
                        move_accel_mm=30.0, move_accel_min_power=40,
                        move_decel_mm=30.0, move_decel_min_power=40,
                        turn_main_power=SPIN_MAX_POWER,
                        turn_fine_max_power=60, turn_fine_min_power=50,
                        turn_fine_tolerance_deg=0.5,
                        turn_decel_deg=0.0,
                        turn_backlash_preload_deg=0.0,
                        turn_pid=(0.2, 0.00075, 0.03),
                        caster_drag_mm_per_deg=0.0502,
                        lateral_drift_mm_per_deg_a=0.00496,
                        lateral_drift_mm_per_deg_b=0.01278,
                        lateral_drift_sign=1.0,
                        initial_heading_deg=0.0,
                        turn_method='spin_encoder',
                        compensation_enabled=True):
    """Convert a received or file-loaded strategy to behaviour tree nodes.

    走行にはETラリー専用のEtRallyRunByGyro/EtRallySpinAroundByEncoder
    (behaviours/et_rally_drive.py)を使い、共通のgyro_drive.pyのクラスは使わない。
    既定値はsample_comment.pyのsteps_from_planと同じ(実機で較正済みの値)。
    caster_drag_mm_per_deg/lateral_drift_mm_per_deg_a,bに0を渡すと、その補正を無効にできる。

    turn_method(DD-19較正用): 'spin_pid'(無補正ベースライン、gyro_drive.SpinAround)/
    'spin_encoder'(既定、DD-13採用のEtRallySpinAroundByEncoder)/'one_shot'(一発パルス仕上げ)/
    'lqi'(LQI仕上げ)のいずれか。compensation_enabled=Falseで、両補正(DD-15/16)をまとめて無効化する
    (caster_drag_mm_per_deg等の引数は無視される)。
    """
    if turn_method not in RALLY_TURN_METHODS:
        raise ValueError("Unknown rally turn method: " + str(turn_method))
    if not isinstance(strategy, list) or not strategy:
        raise ValueError("Strategy must be a non-empty list")

    # 検証済みの新しいdictに詰め替えてから、実機の癖の補正を掛ける。
    steps = []
    for step in strategy:
        if not isinstance(step, dict):
            raise ValueError("Strategy step must be an object")
        step_type = step.get("type")
        if step_type not in ("move", "turn"):
            raise ValueError("Unknown strategy step type: " + str(step_type))
        checked = {
            "type": step_type,
            "target_heading_deg": _finite_number(step.get("target_heading_deg"), "target_heading_deg"),
            "label": step.get("label"),
        }
        if step_type == "move":
            distance = _finite_number(step.get("distance_mm"), "distance_mm")
            if int(round(distance)) <= 0:
                raise ValueError("distance_mm must be greater than zero")
            checked["distance_mm"] = distance
        elif step_type == "turn":
            # 検証専用(spin_encoder_dir_test)のときだけ使う、回転方向の明示指定(+1/-1、省略時0=自動)。
            direction = step.get("direction", 0)
            if direction not in (0, 1, -1):
                raise ValueError("direction must be 0, 1 or -1")
            checked["direction"] = direction
        steps.append(checked)
    # 横ズレはロボット固有の向き(物理的に常に右)に出る。runtime.courseが-1(右コース)だと
    # 「headingが増える向き」が物理的に逆になる(heading=-course*ジャイロ角)ため、
    # sample_comment.pyで較正した左コース基準のまま使うとA/B方向と補正の向きが逆になる。
    # 右コースではA/B係数を入れ替え、補正の符号も反転して物理的に同じ向きにする。
    if runtime.course == -1:
        lateral_drift_mm_per_deg_a, lateral_drift_mm_per_deg_b = (
            lateral_drift_mm_per_deg_b, lateral_drift_mm_per_deg_a)
        lateral_drift_sign = -lateral_drift_sign
    if not compensation_enabled:
        # DD-19較正用ベースライン: DD-15(キャスター引きずり)・DD-16(横ズレ)を両方無効化する。
        caster_drag_mm_per_deg = 0.0
        lateral_drift_mm_per_deg_a = lateral_drift_mm_per_deg_b = 0.0
    steps = apply_caster_drag_compensation(
        steps, mm_per_deg=caster_drag_mm_per_deg, initial_heading_deg=initial_heading_deg)
    steps = apply_lateral_drift_compensation(
        steps, mm_per_deg_a=lateral_drift_mm_per_deg_a,
        mm_per_deg_b=lateral_drift_mm_per_deg_b, sign=lateral_drift_sign,
        initial_heading_deg=initial_heading_deg)

    nodes = []
    for index, step in enumerate(steps):
        heading = step["target_heading_deg"]
        label = step["label"]
        label_text = "" if label is None else " " + str(label)

        if step["type"] == "move":
            distance_mm = int(round(step["distance_mm"]))
            # DD-19ベースライン: spin_pid指定時は直進PIDも、DD-14の調整値(4.0,0.6,0.06)ではなく
            # sample.pyのRunByGyroと同じ値(1.1,0.1,0.03)に揃える。
            active_move_pid = (1.1, 0.1, 0.03) if turn_method == 'spin_pid' else move_pid
            leg = Parallel(
                name="et_rally move%d%s" % (index, label_text),
                policy=ParallelPolicy.SuccessOnOne(),
            )
            leg.add_children(
                [
                    EtRallyRunByGyro(
                        name="et_rally run%d" % index,
                        target=heading,
                        power=move_power,
                        pid_p=active_move_pid[0],
                        pid_i=active_move_pid[1],
                        pid_d=active_move_pid[2],
                        target_type=HeadingType.ABSOLUTE,
                        distance_mm=step["distance_mm"],
                        accel_mm=move_accel_mm,
                        accel_min_power=move_accel_min_power,
                        decel_mm=move_decel_mm,
                        decel_min_power=move_decel_min_power,
                    ),
                    IsDistanceEarned(
                        name="et_rally dist%d" % index,
                        delta_dist=distance_mm,
                    ),
                ]
            )
            nodes.append(leg)
        else:
            turn_name = "et_rally turn%d%s" % (index, label_text)
            if turn_method == 'spin_pid':
                # DD-19ベースライン: DD-13以前の、フェーズ分けのない無補正PID旋回。
                # PID値はturn_pid(既定0.2,0.00075,0.03)を使う。これはproduction(spin_encoder)の
                # フェーズ2仕上げでも実際に使われ動作確認済みの値で、sample.py由来の値(0.4,0.001,0.03)
                # は使わない。
                # 許容誤差も、DD-19以降の改善(2度→0.5度)より前の2.0度に固定する
                # (turn_fine_tolerance_degの既定0.5は改善後の値のため、ここでは使わない)。
                nodes.append(
                    SpinAround(
                        name=turn_name,
                        target=heading,
                        max_power=turn_main_power,
                        min_power=turn_fine_min_power,
                        pid_p=turn_pid[0],
                        pid_i=turn_pid[1],
                        pid_d=turn_pid[2],
                        target_type=HeadingType.ABSOLUTE,
                        tolerance=2.0,
                    )
                )
            elif turn_method == 'one_shot':
                nodes.append(
                    EtRallyTurnOneShot(
                        name=turn_name,
                        target=heading,
                        target_type=HeadingType.ABSOLUTE,
                        main_power=turn_main_power,
                        fine_tolerance_deg=turn_fine_tolerance_deg,
                    )
                )
            elif turn_method == 'lqi':
                nodes.append(
                    EtRallyTurnLqi(
                        name=turn_name,
                        target=heading,
                        target_type=HeadingType.ABSOLUTE,
                        main_power=turn_main_power,
                        fine_tolerance_deg=turn_fine_tolerance_deg,
                    )
                )
            elif turn_method == 'spin_encoder_dir_test':
                # 検証専用: 各turnステップの"direction"(+1/-1、省略時0=自動)で回転方向を強制する。
                nodes.append(
                    EtRallySpinAroundDirectionTest(
                        name=turn_name,
                        target=heading,
                        main_power=turn_main_power,
                        fine_max_power=turn_fine_max_power,
                        fine_min_power=turn_fine_min_power,
                        pid_p=turn_pid[0],
                        pid_i=turn_pid[1],
                        pid_d=turn_pid[2],
                        target_type=HeadingType.ABSOLUTE,
                        fine_tolerance_deg=turn_fine_tolerance_deg,
                        decel_deg=turn_decel_deg,
                        forced_direction=step.get("direction", 0),
                    )
                )
            else:  # 'spin_encoder'(既定、DD-13)
                nodes.append(
                    EtRallySpinAroundByEncoder(
                        name=turn_name,
                        target=heading,
                        main_power=turn_main_power,
                        fine_max_power=turn_fine_max_power,
                        fine_min_power=turn_fine_min_power,
                        pid_p=turn_pid[0],
                        pid_i=turn_pid[1],
                        pid_d=turn_pid[2],
                        target_type=HeadingType.ABSOLUTE,
                        fine_tolerance_deg=turn_fine_tolerance_deg,
                        decel_deg=turn_decel_deg,
                        backlash_preload_deg=turn_backlash_preload_deg,
                    )
                )
    return nodes


def steps_from_plan(plan_path, **kwargs):
    """Compatibility helper that converts the legacy plan JSON."""
    return steps_from_strategy(_load_plan_steps(plan_path), **kwargs)
