"""赤→白を通過後、上250mm・内側300mmを目指してラリー準備位置へ。"""

import math

from ..behaviours.bottle import MarkRallyReady
from ..behaviours.conditions import IsColorTransitionDetected
from ..behaviours.corrected_run import DeliveryEtRun
from ..behaviours.encoder_spin import delivery_encoder_turn
from ..behaviours.gyro_drive import RunByGyro
from ..types import HeadingType
from .bt_imports import Behaviour, BottleColor, Color, Parallel, ParallelPolicy, Sequence, Status, runtime

# 配置ライン基準: 0度が上、+90度がコース内側。左右差は既存の制御が扱う。
_CURVE_RADIUS_MM = 250.0
_CURVE_ARC_MM = math.pi * _CURVE_RADIUS_MM / 2.0
_INWARD_STRAIGHT_MM = 50.0
_CURVE_DISTANCE_MM = _CURVE_ARC_MM + _INWARD_STRAIGHT_MM
# Piで2回目以降に使用した値。旧ローカル版からの意図的な反映。
_APPROACH_POWER = 70
_CURVE_POWER = 70
# 11/5 各1回の白判定距離。実機再測定までの仮値。
_FALLBACK_MM = {BottleColor.RED.value: 403.0, BottleColor.BLUE.value: 592.0,
                BottleColor.YELLOW.value: 744.0}
_RED_TO_WHITE_FALLBACK_MM = 220.0


def _curve_heading(distance_mm):
    """0→90度の円弧後、90度のまま直進する目標方位（開始時からの相対角）。"""
    progress = max(0.0, min(float(distance_mm), _CURVE_ARC_MM))
    return math.degrees(progress / _CURVE_RADIUS_MM)


class RedWhiteOrDistance(Behaviour):
    """赤→白を優先し、未検知時は距離でカーブへ引き渡す。"""
    def __init__(self, name, context):
        super().__init__(name)
        self.context = context
        self.detector = IsColorTransitionDetected(
            name + " detector", Color.RED, Color.WHITE, 0.0, 0.02)

    def initialise(self):
        runtime.require("plotter")
        self.start_distance = runtime.plotter.get_distance()
        zone = getattr(self.context.selected_drop_zone, "value", self.context.selected_drop_zone)
        self.limit = _FALLBACK_MM.get(zone, _FALLBACK_MM[BottleColor.RED.value])
        self.red_distance = None
        # 条件クラスの状態は再実行時に自動初期化されない。
        self.detector.from_started_at = None
        self.detector.to_started_at = None
        self.detector.from_confirmed = False
        self.detector.classifier.window.clear()

    def update(self):
        current = runtime.plotter.get_distance()
        progress = abs(current - self.start_distance)
        self.detector.tick_once()
        if self.detector.status == Status.SUCCESS:
            self.logger.info("red-white transition: color distance=%.1f" % progress)
            return Status.SUCCESS
        if self.detector.status == Status.FAILURE:
            return Status.FAILURE
        if self.detector.from_confirmed and self.red_distance is None:
            self.red_distance = progress
        if self.red_distance is not None:
            if progress - self.red_distance >= _RED_TO_WHITE_FALLBACK_MM:
                self.logger.info("red-white transition: white-missed distance=%.1f red=%.1f" %
                                 (progress, self.red_distance))
                return Status.SUCCESS
        elif progress >= self.limit:
            self.logger.info("red-white transition: red-missed distance=%.1f limit=%.1f" %
                             (progress, self.limit))
            return Status.SUCCESS
        return Status.RUNNING

    def terminate(self, new_status):
        if self.detector.status != Status.INVALID:
            self.detector.stop(Status.INVALID)


class ContinuousCurve(RunByGyro):
    """単体実行の終了時は停止し、連続実行時は後続へ出力を渡す。"""
    def __init__(self, *args, stop_on_success=False, **kwargs):
        super().__init__(*args, **kwargs)
        self.stop_on_success = stop_on_success

    def terminate(self, new_status):
        if new_status == Status.SUCCESS and not self.stop_on_success:
            self.running = False
        else:
            super().terminate(new_status)


def build_move_to_rally_ready(context, config):
    settings = config.integration
    root = Sequence(name="move_to_rally_ready", memory=True)

    # 配置後は0度に合わせ、赤を見てから白へ抜けるまで距離を指定せず直進。
    root.add_child(delivery_encoder_turn("face course absolute 0", context, settings, 0.0))
    search = Parallel(name="drive until red then white", policy=ParallelPolicy.SuccessOnOne())
    search.add_children([
        DeliveryEtRun("red white search motor", context, target=0.0, power=_APPROACH_POWER,
                      pid_p=settings.delivery_drive_pid_p, pid_i=settings.delivery_drive_pid_i,
                      pid_d=settings.delivery_drive_pid_d),
        RedWhiteOrDistance("red then white or fallback", context),
    ])
    # 直進の次周期を待たず、同じtickでカーブの駆動に引き継ぐ。
    root.add_child(search)

    # 白検知後の位置から、半径250mmの1/4円弧＋90度方向50mm。
    # 理想軌跡の移動量は上250mm・内側300mm。実機誤差は別途測定する。
    root.add_child(ContinuousCurve(
        "curve to rally entry", target=_curve_heading, power=_CURVE_POWER,
        pid_p=1.8, pid_i=0.0, pid_d=0.03, target_type=HeadingType.RELATIVE,
        distance_limit_mm=_CURVE_DISTANCE_MM, feedforward_gain=1.0, wheel_tread_mm=117.0,
        stop_on_success=(config.mission_mode == 'bottle-final' or
                         (config.mission_mode == 'configured' and
                          not (config.enable_et_rally and config.et_rally_laps > 0) and
                          not config.enable_finish and not config.enable_et_sumo)),
    ))
    # 最後の停止旋回は行わず、走行終了時の実方位を記録する。
    root.add_child(MarkRallyReady("mark rally ready", context))
    return root
