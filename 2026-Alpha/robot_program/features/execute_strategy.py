"""Feature 12 subtree factory."""

import json
import math
from shared_communication.heading_frame import RALLY_START_HEADING, full_start_to_gyro
from pathlib import Path

from .bt_imports import Failure, HeadingType, Parallel, ParallelPolicy, Sequence
from ..behaviours.et_rally_drive import EtRallyRunByGyro, EtRallySpinAroundByEncoder
from ..behaviours.conditions import IsDistanceEarned
from ..et_rally_compensation import apply_caster_drag_compensation, apply_lateral_drift_compensation


SPIN_MAX_POWER = 70         # その場回旋（スピン）するときの最大モーター出力
SPIN_MIN_POWER = 60         # その場回旋（スピン）するときの最低モーター出力
PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_PLAN_PATH = Path(__file__).resolve().parents[1] / "tests" / "plan_seed9392783.json"


class DeferredStrategySequence(Sequence):
    """Build motor behaviours only when this subtree starts running."""

    def __init__(self, name, loader, initial_heading_deg=0.0):
        super().__init__(name=name, memory=True)
        self._loader = loader
        self._initial_heading_deg = initial_heading_deg
        self._loaded = False

    def _load_children(self):
        # Sequence.tick()が現在の子を決める前に展開する。
        # initialise()内で追加すると、py_treesのcurrent_childとchildrenが
        # 食い違い「unknown / invalid state」になる。
        if self._loaded:
            return
        try:
            nodes = steps_from_strategy(
                self._loader(), initial_heading_deg=self._initial_heading_deg)
        except (KeyError, OSError, TypeError, ValueError) as error:
            self.logger.error("Strategy loading failed: %s" % error)
            nodes = [Failure(name="invalid strategy")]
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
        )
    if source == "file":
        plan_path = _resolve_plan_path(config.et_rally_plan_path)
        return DeferredStrategySequence(
            name="execute_file_strategy",
            loader=lambda: _load_plan_steps(plan_path),
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


def steps_from_strategy(strategy, move_power=70, move_pid=(4.0, 0.6, 0.06),
                        turn_main_power=SPIN_MAX_POWER,
                        turn_fine_max_power=60, turn_fine_min_power=50,
                        turn_fine_tolerance_deg=0.5,
                        turn_pid=(0.2, 0.00075, 0.03),
                        caster_drag_mm_per_deg=0.0502,
                        lateral_drift_mm_per_deg_a=0.00496,
                        lateral_drift_mm_per_deg_b=0.01278,
                        lateral_drift_sign=1.0,
                        initial_heading_deg=0.0):
    """Convert a received or file-loaded strategy to behaviour tree nodes.

    走行にはETラリー専用のEtRallyRunByGyro/EtRallySpinAroundByEncoder
    (behaviours/et_rally_drive.py)を使い、共通のgyro_drive.pyのクラスは使わない。
    既定値はsample_comment.pyのsteps_from_planと同じ(実機で較正済みの値)。
    caster_drag_mm_per_deg/lateral_drift_mm_per_deg_a,bに0を渡すと、その補正を無効にできる。
    """
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
        steps.append(checked)
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
                        pid_p=move_pid[0],
                        pid_i=move_pid[1],
                        pid_d=move_pid[2],
                        target_type=HeadingType.ABSOLUTE,
                        distance_mm=step["distance_mm"],
                    ),
                    IsDistanceEarned(
                        name="et_rally dist%d" % index,
                        delta_dist=distance_mm,
                    ),
                ]
            )
            nodes.append(leg)
        else:
            nodes.append(
                EtRallySpinAroundByEncoder(
                    name="et_rally turn%d%s" % (index, label_text),
                    target=heading,
                    main_power=turn_main_power,
                    fine_max_power=turn_fine_max_power,
                    fine_min_power=turn_fine_min_power,
                    pid_p=turn_pid[0],
                    pid_i=turn_pid[1],
                    pid_d=turn_pid[2],
                    target_type=HeadingType.ABSOLUTE,
                    fine_tolerance_deg=turn_fine_tolerance_deg,
                )
            )
    return nodes


def steps_from_plan(plan_path, **kwargs):
    """Compatibility helper that converts the legacy plan JSON."""
    return steps_from_strategy(_load_plan_steps(plan_path), **kwargs)
