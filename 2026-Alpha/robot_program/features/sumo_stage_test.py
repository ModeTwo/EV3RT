"""相撲の3工程を単独・連続で試すための既存featureへの接続。"""

from dataclasses import replace

from .sumo_delivery_transition import build_move_delivery_to_sumo_straight
from .sumo_rally_transition import build_move_sumo_to_rally_start
from .sumo_push_tree import build_sumo_push_tree
from .sumo_relative_push_settings import RelativePushSettings
from ..phases.et_sumo_after_delivery import build_et_sumo_after_delivery_phase


def configured_sumo_test(config):
    """configの試験条件を既存の②の設定へ変換する。"""
    s = config.sumo_stage_test
    if config.mission_mode in ('sumo-stage-all', 'sumo-stage-push') and s.push_distance_mm is None:
        raise ValueError('Set sumo_stage_test.push_distance_mm in config before this test')
    push = RelativePushSettings(push_distance_mm=s.push_distance_mm,
                                capture_max_mm=s.capture_max_mm,
                                power=s.push_power, turn_power=s.turn_power)
    align = replace(config.sumo_align, distance_mm=s.approach_distance_mm,
                    power=s.approach_power)
    return replace(config, sumo_push_calibration=None, sumo_push_execute=True,
                   sumo_relative_push=push, sumo_align=align)


def build_sumo_stage_test(context, config):
    """未実装の①③は各feature内で停止して失敗する。"""
    builders = {
        'sumo-stage-all': build_et_sumo_after_delivery_phase,
        'sumo-stage-entry': build_move_delivery_to_sumo_straight,
        'sumo-stage-push': build_sumo_push_tree,
        'sumo-stage-exit': build_move_sumo_to_rally_start,
    }
    return builders[config.mission_mode](context, config)
