"""Top-level robot mission composition."""

from .phases.bottle_and_rally_preparation import (
    build_bottle_and_rally_preparation_phase,
    build_bottle_delivery_final_phase,
)
from .phases.et_rally import build_et_rally_phase
from .phases.finish import build_finish_phase
from .phases.lap_gate import build_lap_gate_phase
from .phases.hint_collection import build_hint_collection_phase
from .features.catch_bottle import build_catch_bottle
from .features.to_hint_route import build_tantou_tree
from .behaviours.handoff import CaptureAtToHandoff
from .gyro_scale import EnableGyroScale


def build_mission_children(context, config):
    # この関数は統合担当者だけが変更し、各機能担当者はfeatures配下だけを変更する。
    from .config import INTEGRATION_MISSIONS
    if config.mission_mode.startswith('et-rally'):
        # 2026-11-16: ETラリーの診断用ミッションでも RALLY_PWR を出す(それ以外のミッションではETラリーの工程に入るまで出さない)。
        from . import power_log
        power_log.set_enabled(True)
    if config.mission_mode in ("sumo-stage-all", "sumo-stage-entry", "sumo-stage-push", "sumo-stage-exit"):
        from .features.sumo_stage_test import build_sumo_stage_test
        return [EnableGyroScale(), build_sumo_stage_test(context, config)]
    if config.mission_mode == "sumo-push-route-test":
        from .features.sumo_push_tree import build_sumo_push_tree

        return [EnableGyroScale(), build_sumo_push_tree(context, config)]
    if config.mission_mode == "sumo-sensor-test":
        from .diagnostics.sumo_sensor_test import build_sumo_sensor_test

        return [EnableGyroScale(), build_sumo_sensor_test(context, config)]
    if config.mission_mode == "sumo-turn-test":
        from .diagnostics.sumo_turn_test import build_sumo_turn_test

        return [EnableGyroScale(), build_sumo_turn_test(context, config)]
    if config.mission_mode == "sumo-catch-test":
        from .features.catch_sumo_bottle_test import build_sumo_catch_test

        return [EnableGyroScale(), build_sumo_catch_test(context, config)]
    if config.mission_mode == "sumo-align":
        from .features.align_sumo_bottle import build_align_sumo_bottle

        return [EnableGyroScale(), build_align_sumo_bottle(context, config)]
    if config.mission_mode in INTEGRATION_MISSIONS:
        from .integration_runs import build_integration_children
        # ラップゲートを通らない試験ミッションは、先頭からジャイロの倍率補正を有効にする。
        return [EnableGyroScale()] + list(build_integration_children(context, config))
    if config.mission_mode in ('lap-to', 'lap-to-sumo', 'full'):
        # LAP already performed bottle approach; carry only AT's handoff state.
        from .behaviours.motor_control import StopNow
        from .features.catch_bottle import RequireBottleColor
        from .diagnostics.lap_recovery_observation import FlushLapFloorDiagnostics, DumpTransitionDiagnostics
        from .behaviours.continue_after_failure import ContinueAfterFailure
        from py_trees.decorators import Timeout
        lap = build_lap_gate_phase(context, config)
        lap_phase = lap
        to = build_tantou_tree(context, config, include_exit=True)
        if config.mission_mode in ('lap-to-sumo', 'full'):
            lap = ContinueAfterFailure(name='continue after LAP failure', child=lap)
            to = ContinueAfterFailure(name='continue after TO failure',
                    child=Timeout(name='TO 60s limit', child=to, duration=60))
        children = [lap, EnableGyroScale(),
                RequireBottleColor(context),
                CaptureAtToHandoff('AT_TO capture boundary', context),
                to]
        if config.mission_mode in ('lap-to-sumo', 'full'):
            from .phases.et_sumo import build_sumo_after_delivery

            delivery = build_bottle_delivery_final_phase(context, config)
            sumo = build_sumo_after_delivery(context, config)
            children.extend([ContinueAfterFailure(name='continue after delivery failure',
                                child=Timeout(name='delivery 60s limit', child=delivery, duration=60)),
                             ContinueAfterFailure(name='finish after sumo failure', child=sumo)])
        if config.mission_mode == 'full':
            from .features.sumo_rally_transition import build_move_sumo_to_rally_start
            transition = build_move_sumo_to_rally_start(context, config)
            children.append(ContinueAfterFailure(name='continue after sumo-to-rally failure', child=transition))
            if config.enable_et_rally and config.et_rally_laps > 0:
                children.append(build_et_rally_phase(context, config))
            if config.enable_finish:
                children.append(build_finish_phase(context, config))
        children.append(StopNow(name='LAP to route complete brake'))
        if hasattr(lap_phase, 'floor_recorder'):
            children.extend([FlushLapFloorDiagnostics(lap_phase.floor_recorder),
                             DumpTransitionDiagnostics(lap_phase.lap_controller)])
        return children
    if config.mission_mode not in (
        'at',
        'to',
        'configured',
        'hint2',
        'hint2-return',
        'bottle-final',
        'rally-drive',
        'full',
        'et-rally-turn-calibration',
        'et-rally-marker-observe',
        'et-rally-turn-phase2-diagnostic',
        'et-rally-single-wheel-diagnostic',
        'et-rally-single-wheel-reversal-diagnostic',
        'et-rally-single-wheel-alternating-diagnostic',
        'et-rally-single-wheel-alternating-right-diagnostic',
        'et-rally-single-wheel-alternating-left-diagnostic',
    ):
        raise ValueError('Unknown mission mode: ' + config.mission_mode)
    if config.mission_mode == 'at':
        return [EnableGyroScale(), build_catch_bottle(context, config)]
    if config.mission_mode == 'to':
        # 単体試験ではタッチ開始後の配置位置・向きをAT終了状態として使う。
        return [EnableGyroScale(), CaptureAtToHandoff('TO standalone origin', context),
                build_tantou_tree(context, config)]
    if config.mission_mode in ('hint2', 'hint2-return'):
        # ラップゲートまでは従来の値のまま。通過後(Hint取得以降)から倍率補正を有効にする。
        return [build_lap_gate_phase(context, config), EnableGyroScale(),
                build_hint_collection_phase(context, config)]
    if config.mission_mode == 'bottle-final':
        return [EnableGyroScale(), build_bottle_delivery_final_phase(context, config)]
    if config.mission_mode == 'rally-drive':
        # Hint取得・Bottle Deliveryを通らず、受信待ちとSEQ実行だけを構成する。
        return [EnableGyroScale(), build_et_rally_phase(context, config)]
    if config.mission_mode == 'et-rally-turn-calibration':
        # ETラリー旋回の応答特性データ収集だけを実行する(PC通信・Hintは使わない)。
        from .behaviours.et_rally_turn_calibration import build_et_rally_turn_calibration_sequence
        return [EnableGyroScale(), build_et_rally_turn_calibration_sequence()]
    if config.mission_mode == 'et-rally-marker-observe':
        # マーカー検出の実機確認専用(走行体は動かさず、検出結果だけをログに出す)。
        from .behaviours.et_rally_camera_correction import (
            PrepareEtRallyMarkerCamera, EtRallyMarkerObserveLoop)
        return [EnableGyroScale(), PrepareEtRallyMarkerCamera(name="prepare et_rally marker camera"),
                EtRallyMarkerObserveLoop(name="et_rally marker observe")]
    if config.mission_mode == 'et-rally-turn-phase2-diagnostic':
        # フェーズ2が横ズレの原因かどうかの切り分け専用(plan_spin_alternating30.jsonと比較)。
        from .behaviours.et_rally_turn_phase2_diagnostic import (
            build_et_rally_turn_phase2_diagnostic_sequence)
        return [EnableGyroScale(), build_et_rally_turn_phase2_diagnostic_sequence()]
    if config.mission_mode == 'et-rally-single-wheel-diagnostic':
        # 片輪だけのエンコーダ駆動で、左右の実効的な移動量の差を直接確認する専用。
        from .behaviours.et_rally_single_wheel_test import (
            build_et_rally_single_wheel_diagnostic_sequence)
        return [EnableGyroScale(), build_et_rally_single_wheel_diagnostic_sequence()]
    if config.mission_mode == 'et-rally-single-wheel-reversal-diagnostic':
        # 片輪だけの順方向→逆方向の反転コストを、左右で比較する専用。
        from .behaviours.et_rally_single_wheel_test import (
            build_et_rally_single_wheel_reversal_diagnostic_sequence)
        return [EnableGyroScale(), build_et_rally_single_wheel_reversal_diagnostic_sequence()]
    if config.mission_mode == 'et-rally-single-wheel-alternating-diagnostic':
        # 片輪だけを1周ごとに反転させ続け、反転コストのサンプル数を増やす専用。
        from .behaviours.et_rally_single_wheel_test import (
            build_et_rally_single_wheel_alternating_diagnostic_sequence)
        return [EnableGyroScale(), build_et_rally_single_wheel_alternating_diagnostic_sequence()]
    if config.mission_mode == 'et-rally-single-wheel-alternating-right-diagnostic':
        from .behaviours.et_rally_single_wheel_test import (
            build_et_rally_single_wheel_alternating_right_diagnostic_sequence)
        return [EnableGyroScale(), build_et_rally_single_wheel_alternating_right_diagnostic_sequence()]
    if config.mission_mode == 'et-rally-single-wheel-alternating-left-diagnostic':
        from .behaviours.et_rally_single_wheel_test import (
            build_et_rally_single_wheel_alternating_left_diagnostic_sequence)
        return [EnableGyroScale(), build_et_rally_single_wheel_alternating_left_diagnostic_sequence()]
    children = []
    if config.lapgate:
        children.append(build_lap_gate_phase(context, config))
    # ラップゲート通過後(ET相撲以降)から、ジャイロの倍率補正を有効にする。ラップゲートまでは従来の値。
    children.append(EnableGyroScale())
    # 正式な工程順: LAPゲート通過後はET相撲を先に行い、そのあとでボトルキャッチ(AT)へ進む。
    if config.enable_et_sumo:
        from .phases.et_sumo import build_et_sumo_phase
        children.append(build_et_sumo_phase(context, config))
    # ボトル取得とヒント読取は同じ走行区間で行うため、一つの準備工程として扱う。
    if config.enable_bottle_delivery or config.enable_et_rally:
        children.append(build_bottle_and_rally_preparation_phase(context, config))
    if config.enable_et_rally and config.et_rally_laps > 0:
        children.append(build_et_rally_phase(context, config))
    if config.enable_finish:
        children.append(build_finish_phase(context, config))
    return children
