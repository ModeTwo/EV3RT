"""Feature 12 subtree factory."""

import json
import math
import re
import time
from shared_communication.heading_frame import RALLY_START_HEADING, full_start_to_gyro
from pathlib import Path

from py_trees.behaviour import Behaviour
from py_trees.common import Status

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

_LAP_LABEL_RE = re.compile(r"^lap(\d+)")
# 2026-10-29: turn_forward_compensation有効時の、IsDistanceEarned側の余裕(mm)。
TURN_FORWARD_MARGIN_MM = 30
# 2026-10-29: これより短い直進は、move_stop_lead_mm手前で終えずに計画どおりの距離を走る。
# 手前で終える量(16mm)は巡航(約260mm/s)からの惰性に合わせた値で、短い直進は速度が
# 出ず惰性もほとんどない。以前は最低1mmまで削っていたため、ゴール手前の20mmの直進が
# 4mm(差し引き有効時は1mm)しか走らず、その分ゴールでスタート側に寄っていた。
SHORT_MOVE_NO_LEAD_MM = 50.0


class EtRallyPause(Behaviour):
    """ブレーキをかけ、左右のエンコーダが止まり、かつmin_seconds以上経つまで待つ(2026-10-29、
    スタート直後の直進確認で、前進と後退の間・後退のあとに止まるため)。止まったときの向きと位置をログに残す。

    2026-10-29: ジャイロのずれの検証用に、車輪が止まった時点の向きと、待ち終わりの向きの差
    (still_drift、止まっている間にジャイロの値がどれだけ動いたか)も記録する。
    log_interval_secを指定すると、待っている間もその間隔で向きをログに残す。"""

    STABLE_TICKS = 3
    MAX_SECONDS = 2.0

    def __init__(self, name, min_seconds=0.5, log_interval_sec=None):
        super().__init__(name)
        self.min_seconds = min_seconds
        # 長く止まるときは、車輪が止まらなくてもmin_secondsの1秒後には終える。
        self.max_seconds = max(self.MAX_SECONDS, min_seconds + 1.0)
        self.log_interval_sec = log_interval_sec
        self.started = None

    def update(self) -> Status:
        runtime.require("plotter", "gyro_sensor", "right_motor", "left_motor")
        now = time.monotonic()
        counts = (runtime.right_motor.get_count(), runtime.left_motor.get_count())
        heading = (-1) * runtime.course * runtime.gyro_sensor.get_angle()
        if self.started is None:
            for motor in (runtime.right_motor, runtime.left_motor):
                motor.set_power(0)
                motor.set_brake(True)
            self.started = now
            self.prev = counts
            self.stable = 0
            self.still_heading = None
            self.still_time = None
            self.next_log = None
            return Status.RUNNING
        self.stable = self.stable + 1 if counts == self.prev else 0
        self.prev = counts
        elapsed = now - self.started
        if self.still_heading is None and self.stable >= self.STABLE_TICKS:
            self.still_heading = heading
            self.still_time = now
            if self.log_interval_sec:
                self.next_log = now + self.log_interval_sec
        if self.next_log is not None and now >= self.next_log:
            self.logger.info("%+06d %s.still %.0fs heading=%+.2f drift=%+.2f" % (
                runtime.plotter.get_distance(), self.__class__.__name__, now - self.still_time,
                heading, heading - self.still_heading))
            self.next_log += self.log_interval_sec
        if elapsed < self.max_seconds and (elapsed < self.min_seconds or self.stable < self.STABLE_TICKS):
            return Status.RUNNING
        if self.still_heading is None:
            still_text = " still=none"
        else:
            still_text = " still=%.2fs still_drift=%+.2f" % (now - self.still_time, heading - self.still_heading)
        self.logger.info("%+06d %s.paused %.2fs heading=%+.2f%s loc=(%d,%d)" % (
            runtime.plotter.get_distance(), self.__class__.__name__, elapsed, heading, still_text,
            runtime.plotter.get_loc_x(), runtime.plotter.get_loc_y()))
        return Status.SUCCESS

    def terminate(self, new_status):
        self.started = None


class LogLapBoundary(Behaviour):
    """周回の切り替わりでplotterの位置・方位をログに残す(2026-10-24、周回ごとの
    位置ズレを診断するための目印。即SUCCESSを返すだけで、走行には影響しない)。"""

    def __init__(self, name, lap_tag):
        super().__init__(name)
        self.lap_tag = lap_tag

    def update(self) -> Status:
        runtime.require("plotter")
        self.logger.info("%+06d LAP_BOUNDARY %s loc=(%d,%d) degree=%d" % (
            runtime.plotter.get_distance(), self.lap_tag,
            runtime.plotter.get_loc_x(), runtime.plotter.get_loc_y(),
            runtime.plotter.get_degree()))
        return Status.SUCCESS


class DeferredStrategySequence(Sequence):
    """Build motor behaviours only when this subtree starts running."""

    def __init__(self, name, loader, initial_heading_deg=0.0, fallback_loader=None,
                 turn_method='spin_encoder', compensation_enabled=False,
                 turn_fine_tolerance_deg=None, turn_decel_deg=None,
                 turn_backlash_preload_deg=None, turn_fine_trim=None,
                 turn_main_power=None, turn_lag_compensation_deg=None,
                 turn_adaptive_lag_compensation=None, turn_left_target_scale=None,
                 turn_phase1_scale=None, turn_pre_stop=None, turn_forward_compensation=None,
                 start_check_mm=None, straight_pause_sec=None, straight_pause_min_mm=None,
                 gyro_still_sec=None):
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
        self._turn_fine_trim = turn_fine_trim
        self._turn_main_power = turn_main_power
        self._turn_lag_compensation_deg = turn_lag_compensation_deg
        self._turn_adaptive_lag_compensation = turn_adaptive_lag_compensation
        self._turn_left_target_scale = turn_left_target_scale
        self._turn_phase1_scale = turn_phase1_scale
        self._turn_pre_stop = turn_pre_stop
        self._turn_forward_compensation = turn_forward_compensation
        self._start_check_mm = start_check_mm
        self._optional_kwargs = {
            "straight_pause_sec": straight_pause_sec,
            "straight_pause_min_mm": straight_pause_min_mm,
            "gyro_still_sec": gyro_still_sec,
        }
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
        if self._turn_fine_trim is not None:
            extra_kwargs["turn_fine_trim"] = self._turn_fine_trim
        if self._turn_main_power is not None:
            extra_kwargs["turn_main_power"] = self._turn_main_power
        if self._turn_lag_compensation_deg is not None:
            extra_kwargs["turn_lag_compensation_deg"] = self._turn_lag_compensation_deg
        if self._turn_adaptive_lag_compensation is not None:
            extra_kwargs["turn_adaptive_lag_compensation"] = self._turn_adaptive_lag_compensation
        if self._turn_left_target_scale is not None:
            extra_kwargs["turn_left_target_scale"] = self._turn_left_target_scale
        if self._turn_phase1_scale is not None:
            extra_kwargs["turn_phase1_scale"] = self._turn_phase1_scale
        if self._turn_pre_stop is not None:
            extra_kwargs["turn_pre_stop"] = self._turn_pre_stop
        if self._turn_forward_compensation is not None:
            extra_kwargs["turn_forward_compensation"] = self._turn_forward_compensation
        if self._start_check_mm is not None:
            extra_kwargs["start_check_mm"] = self._start_check_mm
        for key, value in self._optional_kwargs.items():
            if value is not None:
                extra_kwargs[key] = value
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
            compensation_enabled=getattr(config, "et_rally_compensation_enabled", False),
            turn_fine_tolerance_deg=getattr(config, "et_rally_turn_fine_tolerance_deg", None),
            turn_decel_deg=getattr(config, "et_rally_turn_decel_deg", None),
            turn_backlash_preload_deg=getattr(config, "et_rally_turn_backlash_preload_deg", None),
            turn_fine_trim=getattr(config, "et_rally_turn_fine_trim_enabled", None),
            turn_main_power=getattr(config, "et_rally_turn_main_power", None),
            turn_lag_compensation_deg=getattr(config, "et_rally_turn_lag_compensation_deg", None),
            turn_adaptive_lag_compensation=getattr(config, "et_rally_turn_adaptive_lag_compensation", None),
            turn_left_target_scale=getattr(config, "et_rally_turn_left_target_scale", None),
            turn_phase1_scale=getattr(config, "et_rally_turn_phase1_scale", None),
            turn_pre_stop=getattr(config, "et_rally_turn_pre_stop", None),
            turn_forward_compensation=getattr(config, "et_rally_turn_forward_compensation", None),
            start_check_mm=getattr(config, "et_rally_start_check_mm", None),
            straight_pause_sec=getattr(config, "et_rally_straight_pause_sec", None),
            straight_pause_min_mm=getattr(config, "et_rally_straight_pause_min_mm", None),
            gyro_still_sec=getattr(config, "et_rally_gyro_still_sec", None),
        )
    if source == "file":
        plan_path = _resolve_plan_path(config.et_rally_plan_path)
        return DeferredStrategySequence(
            name="execute_file_strategy",
            loader=lambda: _load_plan_steps(plan_path),
            fallback_loader=(fallback_planner_steps
                             if getattr(config, "et_rally_fallback_enabled", False) else None),
            turn_method=getattr(config, "et_rally_turn_method", "spin_encoder"),
            compensation_enabled=getattr(config, "et_rally_compensation_enabled", False),
            turn_fine_tolerance_deg=getattr(config, "et_rally_turn_fine_tolerance_deg", None),
            turn_decel_deg=getattr(config, "et_rally_turn_decel_deg", None),
            turn_backlash_preload_deg=getattr(config, "et_rally_turn_backlash_preload_deg", None),
            turn_fine_trim=getattr(config, "et_rally_turn_fine_trim_enabled", None),
            turn_main_power=getattr(config, "et_rally_turn_main_power", None),
            turn_lag_compensation_deg=getattr(config, "et_rally_turn_lag_compensation_deg", None),
            turn_adaptive_lag_compensation=getattr(config, "et_rally_turn_adaptive_lag_compensation", None),
            turn_left_target_scale=getattr(config, "et_rally_turn_left_target_scale", None),
            turn_phase1_scale=getattr(config, "et_rally_turn_phase1_scale", None),
            turn_pre_stop=getattr(config, "et_rally_turn_pre_stop", None),
            turn_forward_compensation=getattr(config, "et_rally_turn_forward_compensation", None),
            start_check_mm=getattr(config, "et_rally_start_check_mm", None),
            straight_pause_sec=getattr(config, "et_rally_straight_pause_sec", None),
            straight_pause_min_mm=getattr(config, "et_rally_straight_pause_min_mm", None),
            gyro_still_sec=getattr(config, "et_rally_gyro_still_sec", None),
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


def _merge_consecutive_moves(steps):
    """同じ向きの直進が旋回を挟まずに続くとき、1本の直進にまとめる。

    2026-10-28: 計画では、ゲートの手前(entry)と通過(exit)が別の直進になっており、
    2周目以降の赤ゲートのように向きが変わらないと直進が2本続く。直進ごとに
    move_stop_lead_mm手前で終えると、1本目は止まらずに2本目へ続くため惰性が出ず、
    その分(周回コースのログで約8mm/周)足りなくなっていた。まとめれば、手前で
    終える処理は最後に1回だけになる。ラベルは1本目のものを残す(周回の区切りの判定に使う)。
    """
    merged = []
    for step in steps:
        prev = merged[-1] if merged else None
        if (step["type"] == "move" and prev is not None and prev["type"] == "move"
                and abs(step["target_heading_deg"] - prev["target_heading_deg"]) < 1e-6):
            prev["distance_mm"] += step["distance_mm"]
            continue
        merged.append(dict(step))
    return merged


def steps_from_strategy(strategy, move_power=80, move_pid=(4.0, 0.6, 0.06),
                        # 2026-09(会話にもとづく): 発進・停止それぞれの直前直後だけを
                        # 短く加減速する。区間全体をゆっくり加減速するのではない
                        # (長い区間でも、この距離分しか加減速しない)。
                        # 【要実機調整】30mm/半分のパワー、はまだ最初の見立てで、
                        # 実測して詰める前提の値。
                        move_accel_mm=30.0, move_accel_min_power=40,
                        # 2026-10-28: 30mmでは減速が間に合わず、目標到達時も約340mm/sで
                        # 走っていて、止まるまでに1〜2cm進んでいた(床の実測)。80mmに
                        # 延ばして到達時の速度を下げ、惰性を小さくする。
                        move_decel_mm=80.0, move_decel_min_power=40,
                        # 2026-10-28: 減速しても残る惰性と、目標到達の判定の行き過ぎ
                        # (20msごとの判定の刻み)の分だけ、直進を手前で終える量(mm)。
                        # 2026-10-28: タイヤ径55.56・減速80mmで300mm直進を床で実測すると317mm
                        # (ログ上も316mmで一致=タイヤ径は正しい)。目標到達時の行き過ぎ+惰性の
                        # 約16mmを手前で止める。
                        move_stop_lead_mm=16.0,
                        turn_main_power=SPIN_MAX_POWER,
                        turn_fine_max_power=60, turn_fine_min_power=50,
                        turn_fine_tolerance_deg=0.5,
                        turn_decel_deg=0.0,
                        turn_backlash_preload_deg=0.0,
                        turn_fine_trim=True,
                        turn_lag_compensation_deg=0.0,
                        turn_adaptive_lag_compensation=False,
                        turn_left_target_scale=1.0,
                        turn_phase1_scale=(1.0, 1.0),
                        turn_pre_stop=False,
                        # 2026-10-29: 旋回で前へ出た量を、次の直進から差し引く(one_shotのみ値が渡る)。
                        turn_forward_compensation=False,
                        # 2026-10-29: 0より大きければ、経路の前に、スタートの向き(0度)へこの距離だけ
                        # 前進→停止→同じ距離だけ後退→停止する(置いた向きがまっすぐかを目で確かめる用)。
                        start_check_mm=0.0,
                        # 2026-10-29: 0より大きければ、straight_pause_min_mm以上の直進の前後で、
                        # この秒数だけ止まる(直進の始めと終わりの位置を測るため)。
                        straight_pause_sec=0.0,
                        straight_pause_min_mm=400.0,
                        # 2026-10-29: 0より大きければ、経路の前に、この秒数だけ止まったまま
                        # 向きをログに残す(止まっているときのジャイロのずれの検証用)。
                        gyro_still_sec=0.0,
                        turn_pid=(0.2, 0.00075, 0.03),
                        caster_drag_mm_per_deg=0.0502,
                        lateral_drift_mm_per_deg_a=0.00496,
                        lateral_drift_mm_per_deg_b=0.01278,
                        lateral_drift_sign=1.0,
                        initial_heading_deg=0.0,
                        turn_method='spin_encoder',
                        compensation_enabled=False):
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
    steps = _merge_consecutive_moves(steps)

    def check_legs(prefix, heading, check_mm):
        # 指定の向きへcheck_mmだけ前進→停止→同じ距離だけ後退→停止する。
        legs = []
        for tag, reverse in (("forward", False), ("back", True)):
            check_distance_mm = max(check_mm - move_stop_lead_mm, 1.0)
            leg = Parallel(name="et_rally %s %s" % (prefix, tag), policy=ParallelPolicy.SuccessOnOne())
            leg.add_children([
                EtRallyRunByGyro(
                    name="et_rally %s %s run" % (prefix, tag), target=heading, power=move_power,
                    pid_p=move_pid[0], pid_i=move_pid[1], pid_d=move_pid[2],
                    target_type=HeadingType.ABSOLUTE, distance_mm=check_distance_mm,
                    accel_mm=move_accel_mm, accel_min_power=move_accel_min_power,
                    decel_mm=move_decel_mm, decel_min_power=move_decel_min_power,
                    reverse=reverse,
                ),
                IsDistanceEarned(name="et_rally %s %s dist" % (prefix, tag),
                                 delta_dist=int(round(check_distance_mm))),
            ])
            legs.append(leg)
            legs.append(EtRallyPause(name="et_rally %s %s pause" % (prefix, tag), min_seconds=1.0))
        return legs

    nodes = []
    if gyro_still_sec and gyro_still_sec > 0:
        nodes.append(EtRallyPause(name="et_rally gyro still", min_seconds=gyro_still_sec,
                                  log_interval_sec=5.0))
    if start_check_mm and start_check_mm > 0:
        nodes.extend(check_legs("start check", 0.0, start_check_mm))
    prev_lap = None
    for index, step in enumerate(steps):
        heading = step["target_heading_deg"]
        label = step["label"]
        label_text = "" if label is None else " " + str(label)

        # 周回ごとの位置ズレを診断するため、ラベル(lapN-color-entry/exit)の
        # 周回番号が切り替わるタイミングで、位置ログだけ残す目印ノードを挟む。
        lap_match = _LAP_LABEL_RE.match(label) if label else None
        cur_lap = lap_match.group(1) if lap_match else None
        if cur_lap is not None and cur_lap != prev_lap:
            nodes.append(LogLapBoundary(
                name="et_rally lap_boundary%d lap%s-start" % (index, cur_lap),
                lap_tag="lap%s-start" % cur_lap))
            prev_lap = cur_lap

        pause_this_move = (step["type"] == "move" and straight_pause_sec and straight_pause_sec > 0
                           and step["distance_mm"] >= straight_pause_min_mm)
        if pause_this_move:
            nodes.append(EtRallyPause(name="et_rally pause before move%d%s" % (index, label_text),
                                      min_seconds=straight_pause_sec))
        if step["type"] == "move":
            # 惰性で進む分を見込み、手前で直進を終える(短すぎる区間でも最低1mmは走る)。
            if step["distance_mm"] < SHORT_MOVE_NO_LEAD_MM:
                run_distance_mm = step["distance_mm"]
            else:
                run_distance_mm = max(step["distance_mm"] - move_stop_lead_mm, 1.0)
            distance_mm = int(round(run_distance_mm))
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
                        distance_mm=run_distance_mm,
                        accel_mm=move_accel_mm,
                        accel_min_power=move_accel_min_power,
                        decel_mm=move_decel_mm,
                        decel_min_power=move_decel_min_power,
                        turn_forward_compensation=turn_forward_compensation,
                    ),
                    # 差し引きが有効なときは、直進の終了はEtRallyRunByGyroが決める。
                    # IsDistanceEarnedは、旋回で後ろへ下がった(差し引きが負の)場合でも
                    # 先に終わらないよう、余裕を持たせた安全側の上限として残す。
                    IsDistanceEarned(
                        name="et_rally dist%d" % index,
                        delta_dist=distance_mm + (TURN_FORWARD_MARGIN_MM if turn_forward_compensation else 0),
                    ),
                ]
            )
            nodes.append(leg)
            if pause_this_move:
                nodes.append(EtRallyPause(name="et_rally pause after move%d%s" % (index, label_text),
                                          min_seconds=straight_pause_sec))
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
                        lag_compensation_deg=turn_lag_compensation_deg,
                        adaptive_lag_compensation=turn_adaptive_lag_compensation,
                        backlash_preload_deg=turn_backlash_preload_deg,
                        left_target_scale=turn_left_target_scale,
                        phase1_scale=turn_phase1_scale,
                        pre_stop=turn_pre_stop,
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
                        fine_trim=turn_fine_trim,
                        lag_compensation_deg=turn_lag_compensation_deg,
                        adaptive_lag_compensation=turn_adaptive_lag_compensation,
                        left_target_scale=turn_left_target_scale,
                    )
                )
    return nodes


def steps_from_plan(plan_path, **kwargs):
    """Compatibility helper that converts the legacy plan JSON."""
    return steps_from_strategy(_load_plan_steps(plan_path), **kwargs)
