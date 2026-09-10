"""Top-level robot mission composition."""

from .phases.bottle_and_rally_preparation import (
    build_bottle_and_rally_preparation_phase,
    build_bottle_delivery_final_phase,
)
from .phases.et_rally import build_et_rally_phase
from .phases.et_sumo import build_et_sumo_phase
from .phases.finish import build_finish_phase
from .phases.lap_gate import build_lap_gate_phase
from .phases.hint_collection import build_hint_collection_phase
from .features.catch_bottle import build_catch_bottle
from .features.to_hint_route import build_tantou_tree
from .behaviours.handoff import CaptureAtToHandoff


def build_mission_children(context, config):
    # この関数は統合担当者だけが変更し、各機能担当者はfeatures配下だけを変更する。
    if config.mission_mode not in (
        'at',
        'to',
        'configured',
        'hint2',
        'hint2-return',
        'bottle-final',
        'full',
    ):
        raise ValueError('Unknown mission mode: ' + config.mission_mode)
    if config.mission_mode == 'at':
        return [build_catch_bottle(context, config)]
    if config.mission_mode == 'to':
        # 単体試験ではタッチ開始後の配置位置・向きをAT終了状態として使う。
        return [CaptureAtToHandoff('TO standalone origin', context),
                build_tantou_tree(context, config)]
    if config.mission_mode in ('hint2', 'hint2-return'):
        return [build_lap_gate_phase(context, config), build_hint_collection_phase(context, config)]
    if config.mission_mode == 'bottle-final':
        return [build_bottle_delivery_final_phase(context, config)]
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
