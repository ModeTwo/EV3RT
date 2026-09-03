"""Top-level robot mission composition."""

from .phases.bottle_and_rally_preparation import build_bottle_and_rally_preparation_phase
from .phases.et_rally import build_et_rally_phase
from .phases.et_sumo import build_et_sumo_phase
from .phases.finish import build_finish_phase
from .phases.lap_gate import build_lap_gate_phase


def build_mission_children(context, config):
    # この関数は統合担当者だけが変更し、各機能担当者はfeatures配下だけを変更する。
    children = []
    if config.lapgate:
        children.append(build_lap_gate_phase(context, config))
    # ボトル取得とヒント読取は同じ走行区間で行うため、一つの準備工程として扱う。
    if config.enable_bottle_delivery or config.enable_et_rally:
        children.append(build_bottle_and_rally_preparation_phase(context, config))
    if config.enable_et_rally and config.et_rally_laps > 0:
        children.append(build_et_rally_phase(context, config))
    if config.enable_et_sumo:
        children.append(build_et_sumo_phase(context, config))
    if config.enable_finish:
        children.append(build_finish_phase(context, config))
    return children
