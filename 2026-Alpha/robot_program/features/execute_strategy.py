"""Feature 12 subtree factory."""

import json
import math
from pathlib import Path

from .bt_imports import Failure, HeadingType, Parallel, ParallelPolicy, Sequence
from ..behaviours.gyro_drive import RunByGyro, SpinAround
from ..behaviours.conditions import IsDistanceEarned


SPIN_MAX_POWER = 57         # その場回旋（スピン）するときの最大モーター出力
SPIN_MIN_POWER = 47         # その場回旋（スピン）するときの最低モーター出力
PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_PLAN_PATH = Path(__file__).resolve().parents[1] / "tests" / "plan_seed9392783.json"


class DeferredStrategySequence(Sequence):
    """Build motor behaviours only when this subtree starts running."""

    def __init__(self, name, loader):
        super().__init__(name=name, memory=True)
        self._loader = loader
        self._loaded = False

    def initialise(self):
        # 受信SEQはBT構築後にcontextへ入るため、実行直前まで展開を遅らせる。
        if self._loaded:
            return
        try:
            nodes = steps_from_strategy(self._loader())
        except (KeyError, OSError, TypeError, ValueError) as error:
            self.logger.error("Strategy loading failed: %s" % error)
            nodes = [Failure(name="invalid strategy")]
        self.add_children(nodes)
        self._loaded = True


def build_execute_strategy(context, config, lap_number=None):
    # No.12は、PCが作成した全周回分のSEQを一度だけ実行する。
    source = config.et_rally_strategy_source
    if source == "received":
        return DeferredStrategySequence(
            name="execute_received_strategy",
            loader=lambda: context.strategy,
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


def steps_from_strategy(strategy, move_power=50, move_pid=(1.1, 0.00075, 0.04),
                        turn_max_power=SPIN_MAX_POWER, turn_min_power=SPIN_MIN_POWER,
                        turn_pid=(0.2, 0.00075, 0.03)):
    """Convert a received or file-loaded strategy to behaviour tree nodes."""
    if not isinstance(strategy, list) or not strategy:
        raise ValueError("Strategy must be a non-empty list")

    nodes = []
    for index, step in enumerate(strategy):
        if not isinstance(step, dict):
            raise ValueError("Strategy step must be an object")
        step_type = step.get("type")
        heading = _finite_number(step.get("target_heading_deg"), "target_heading_deg")
        label = step.get("label")
        label_text = "" if label is None else " " + str(label)

        if step_type == "move":
            distance = _finite_number(step.get("distance_mm"), "distance_mm")
            distance_mm = int(round(distance))
            if distance_mm <= 0:
                raise ValueError("distance_mm must be greater than zero")
            leg = Parallel(
                name="et_rally move%d%s" % (index, label_text),
                policy=ParallelPolicy.SuccessOnOne(),
            )
            leg.add_children(
                [
                    RunByGyro(
                        name="et_rally run%d" % index,
                        target=heading,
                        power=move_power,
                        pid_p=move_pid[0],
                        pid_i=move_pid[1],
                        pid_d=move_pid[2],
                        target_type=HeadingType.ABSOLUTE,
                    ),
                    IsDistanceEarned(
                        name="et_rally dist%d" % index,
                        delta_dist=distance_mm,
                    ),
                ]
            )
            nodes.append(leg)
        elif step_type == "turn":
            nodes.append(
                SpinAround(
                    name="et_rally turn%d%s" % (index, label_text),
                    target=heading,
                    max_power=turn_max_power,
                    min_power=turn_min_power,
                    pid_p=turn_pid[0],
                    pid_i=turn_pid[1],
                    pid_d=turn_pid[2],
                    target_type=HeadingType.ABSOLUTE,
                )
            )
        else:
            raise ValueError("Unknown strategy step type: " + str(step_type))
    return nodes


def steps_from_plan(plan_path, move_power=50, move_pid=(1.1, 0.00075, 0.04),
                    turn_max_power=SPIN_MAX_POWER, turn_min_power=SPIN_MIN_POWER,
                    turn_pid=(0.2, 0.00075, 0.03)):
    """Compatibility helper that converts the legacy plan JSON."""
    return steps_from_strategy(
        _load_plan_steps(plan_path),
        move_power=move_power,
        move_pid=move_pid,
        turn_max_power=turn_max_power,
        turn_min_power=turn_min_power,
        turn_pid=turn_pid,
    )
