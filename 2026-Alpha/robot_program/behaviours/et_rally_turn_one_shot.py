"""ETラリー旋回の、「一発サイズの補正」仕上げ版。EtRallySpinAroundByEncoder
(et_rally_drive.py)・EtRallyTurnLqi(et_rally_turn_lqi.py)のどちらも変更・継承せず、
この1クラスに独立して実装する(実測データがまだ無く、検証前のため)。

【位置づけ(2026-09、会話にもとづく優先順位の見直し)】
実機で確認された「旋回時、細かく左右に動く」現象は、毎tickフィードバックし
続ける制御則(PID・LQIどちらも同じ)の問題というより、最小出力(不感帯を超える
下限パワー)の粒度が粗いことが原因と診断した。この仕上げ版は、フィードバックを
繰り返す代わりに、較正データから計算した「狙った量だけ動く一発のパルス」を
与えて止める、という発想で作る。

構成:
  フェーズ1: 左右のエンコーダで「その場」を保証する。EtRallySpinAroundByEncoderと
             同じ理由(片輪が先に止まった後、もう片輪の反力で引き込まれて動く分)で
             lag_compensation_deg/adaptive_lag_compensationによる左右非対称の
             目標値補正を行う(2026-10-24追加。実測で確認された、phase1が
             大きな残差を残してしまう不具合への対処)。方向反転時のバックラッシュ
             先詰め(backlash_preload_deg)・左右の実効径差補正(left_target_scale)も
             EtRallySpinAroundByEncoderと同じ考え方で追加(2026-10-25、spin_encoderと
             同条件で比較するため)。
  フェーズ2: PULSE_TABLE(wireless_device/et_rally_lqi/one_shot_pulse.pyで、
             実測データから計算する)を引いて、残差にちょうど合うtick数だけ
             floor_powerを与えて停止する。1回で収まらなければ、残差を測り直して
             もう1回だけ試す(最大2回、それ以上は追いかけない=不感帯の粒度より
             細かい誤差を無理に追わない設計)。

PULSE_TABLE/FLOOR_POWERは、2026-10-24にet-rally-turn-calibrationの実機ログ
(run_pipeline.py)から実測したb, c, stiction_floorで計算した値(詳細は定数
定義の直前のコメント参照)。再較正する場合はwireless_device/et_rally_lqi/
one_shot_pulse.py の build_pulse_table() を実測値で再実行し、この表を差し替える。
"""
import math

from py_trees.behaviour import Behaviour
from py_trees.common import Status

from py_etrobo_util.plotter import WHEEL_TREAD

from ..gyro_scale import ensure_scaled_gyro
from ..et_rally_heading import et_pose_text, set_et_turning
from ..power_log import log_power_state
from ..runtime import runtime
from ..types import HeadingType
from .et_rally_drive import forward_mm_since, set_turn_forward_mm

# 2026-10-25(再較正): FLOOR_POWER=40は、tick数を10→40まで増やしても到達角度が
# 2〜3度のまま変わらない(失速する)不具合が、FixedTickPulse較正(et_rally_turn_calibration.py)
# の実機テストで2回連続で確認された。この回の不感帯閾値(CCW power=40)がFLOOR_POWERと
# 同じ値になっていたことと整合する。power=40,45,50,60で同じ測定をしたところ、45以上では
# tick数に比例して回り続ける(失速しない)ことを確認し、信頼性(失速しない)と分解能
# (補正の細かさ)のバランスからFLOOR_POWER=50を採用した。
#
# b, cは、このFLOOR_POWER=50でのFixedTickPulse実測(tick数5,10,...,40、方向・3回×)の
# (ticks, 到達角度)の組にモデルを直接フィットして求めた(run_20261025T092311、
# et-rally-turn-calibration): b=19.184446, c=14.042704 (R^2=0.97、n=48)。
# この b・c は、較正の時点でpower=50を直接入力として当てはめたため、不感帯分を引く
# u_eff = power - stiction_floor は使わない(stiction_floor=0としてbuild_pulse_table()を
# 実行する。FLOOR_POWER=50は不感帯の実測値(CW35/CCW40前後)より十分大きく、
# 不感帯の影響を受けない前提)。
FLOOR_POWER = 50
PULSE_TABLE = [
    (0.25, 1, 0.3837), (0.50, 1, 0.3837), (0.75, 2, 1.0433), (1.00, 2, 1.0433),
    (1.25, 2, 1.0433), (1.50, 3, 1.9014), (1.75, 3, 1.9014), (2.00, 3, 1.9014),
    (2.25, 3, 1.9014), (2.50, 4, 2.9021), (2.75, 4, 2.9021), (3.00, 4, 2.9021),
    (3.25, 4, 2.9021), (3.50, 5, 4.0055), (3.75, 5, 4.0055), (4.00, 5, 4.0055),
    (4.25, 5, 4.0055), (4.50, 5, 4.0055), (4.75, 6, 5.1827), (5.00, 6, 5.1827),
    (5.25, 6, 5.1827), (5.50, 6, 5.1827), (5.75, 6, 5.1827), (6.00, 7, 6.4129),
    (6.25, 7, 6.4129), (6.50, 7, 6.4129), (6.75, 7, 6.4129), (7.00, 7, 6.4129),
    (7.25, 8, 7.6813), (7.50, 8, 7.6813), (7.75, 8, 7.6813), (8.00, 8, 7.6813),
    (8.25, 8, 7.6813), (8.50, 9, 8.9772), (8.75, 9, 8.9772), (9.00, 9, 8.9772),
    (9.25, 9, 8.9772), (9.50, 9, 8.9772), (9.75, 10, 10.2928), (10.00, 10, 10.2928),
    (10.25, 10, 10.2928), (10.50, 10, 10.2928), (10.75, 10, 10.2928), (11.00, 11, 11.6226),
    (11.25, 11, 11.6226), (11.50, 11, 11.6226), (11.75, 11, 11.6226), (12.00, 11, 11.6226),
    (12.25, 11, 11.6226), (12.50, 12, 12.9626), (12.75, 12, 12.9626), (13.00, 12, 12.9626),
    (13.25, 12, 12.9626), (13.50, 12, 12.9626), (13.75, 13, 14.3100), (14.00, 13, 14.3100),
    (14.25, 13, 14.3100), (14.50, 13, 14.3100), (14.75, 13, 14.3100), (15.00, 14, 15.6626),
    (15.25, 14, 15.6626), (15.50, 14, 15.6626), (15.75, 14, 15.6626), (16.00, 14, 15.6626),
    (16.25, 14, 15.6626), (16.50, 15, 17.0190), (16.75, 15, 17.0190), (17.00, 15, 17.0190),
    (17.25, 15, 17.0190), (17.50, 15, 17.0190), (17.75, 16, 18.3782), (18.00, 16, 18.3782),
    (18.25, 16, 18.3782), (18.50, 16, 18.3782), (18.75, 16, 18.3782), (19.00, 16, 18.3782),
    (19.25, 17, 19.7393), (19.50, 17, 19.7393), (19.75, 17, 19.7393), (20.00, 17, 19.7393),
    (20.25, 17, 19.7393), (20.50, 18, 21.1018), (20.75, 18, 21.1018), (21.00, 18, 21.1018),
    (21.25, 18, 21.1018), (21.50, 18, 21.1018), (21.75, 18, 21.1018), (22.00, 19, 22.4654),
    (22.25, 19, 22.4654), (22.50, 19, 22.4654), (22.75, 19, 22.4654), (23.00, 19, 22.4654),
    (23.25, 20, 23.8297), (23.50, 20, 23.8297), (23.75, 20, 23.8297), (24.00, 20, 23.8297),
    (24.25, 20, 23.8297), (24.50, 20, 23.8297), (24.75, 21, 25.1945), (25.00, 21, 25.1945),
    (25.25, 21, 25.1945), (25.50, 21, 25.1945), (25.75, 21, 25.1945), (26.00, 22, 26.5597),
    (26.25, 22, 26.5597), (26.50, 22, 26.5597), (26.75, 22, 26.5597), (27.00, 22, 26.5597),
    (27.25, 23, 27.9251), (27.50, 23, 27.9251), (27.75, 23, 27.9251), (28.00, 23, 27.9251),
    (28.25, 23, 27.9251), (28.50, 23, 27.9251), (28.75, 24, 29.2908), (29.00, 24, 29.2908),
    (29.25, 24, 29.2908), (29.50, 24, 29.2908), (29.75, 24, 29.2908), (30.00, 25, 30.6566),
]
MAX_PULSE_ATTEMPTS = 2  # これ以上は追いかけない(不感帯より細かい残差を無理に追わない)
# 2026-10-31: phase1_model使用時、一定の流れを満額見込む旋回の大きさ(度)。これ未満は角度に比例して小さくする。
PHASE1_FLOW_FULL_DEG = 40.0

# 2026-10-25: phase1終了直後・各パルス直後は、ブレーキをかけてもロボットがまだ回っている
# (旋回30回テストで、pulse#2時点の残差が旋回方向に15回中15回とも同じ符号で行き過ぎ、
# パルスを撃たなかった回でも約40ms後に1〜2度流れていた)。heading がこのtick数だけ
# 続けて変わらなくなってから残差を測る。SETTLE_MAX_TICKSで打ち切る(1tick=0.02s)。
SETTLE_STABLE_TICKS = 3
SETTLE_MAX_TICKS = 15
# 2026-10-29: 1tickの向きの変化がこれ未満なら「変わらない」とみなす。四捨五入前の向き(--rally-gyro-float)
# では値が完全に同じになることがなく、毎回SETTLE_MAX_TICKSまで待っていた。整数の向き(1度刻み)では
# 変化は0か1度以上なので、従来と同じ判定になる。
SETTLE_STABLE_DEG = 0.05
# 2026-10-28: pre_stop有効時、phase1の前にブレーキをかけ、左右のエンコーダが
# このtick数だけ続けて変わらなくなる(=直進の惰性が止まる)まで待つ。PRE_STOP_MAX_TICKSで打ち切る。
PRE_STOP_STABLE_TICKS = 2
PRE_STOP_MAX_TICKS = 20

# DD-22較正用(et_rally_drive.pyと同じ考え方を、このクラス独立で持つ。
# クラス間の変更・継承はしない設計方針のため、モジュール変数も別に持つ)。
# phase1で先に止まった側が、もう片方の反力で引き込まれて動いてしまう分の
# 推定値(指数移動平均、target_motor_degに対する比率)。
# 2026-10-28: 初期値を、et_rally_drive.pyと同じ2026-10-10の実測値(22.67/121≈0.187)から、
# 現在の構成(タイヤ径55.56・phase1_scale 0.961/0.950・tolerance 0.6)での旋回30回テストで
# 落ち着いた値(後半20旋回の平均0.077)に変更。0.187のままだと走行開始直後の数旋回で
# phase1が6〜8度手前で止まっていた。周回コースでは0.04前後まで下がるため、周回の
# 実測を見て見直すこと。
_LAG_REFERENCE_MOTOR_DEG = 121.0
_lag_estimate_ratio = 0.078
_LAG_ESTIMATE_ALPHA = 0.2
# 2026-10-25較正用: et_rally_drive.pyと同じ不具合(引き込まれ量はほぼ一定なのに
# 小さい旋回のearly_baseで割るため比率が発散する、実機ログでratioが1.0超へ
# 急騰する不具合を確認)への対策。閾値・上限とも暫定値、実機データが増えたら見直す。
_LAG_ESTIMATE_MIN_EARLY_BASE_DEG = 20.0
_LAG_ESTIMATE_MAX_RATIO = 0.5

# DD-22較正用: 直前の旋回の方向(+1/-1)。0=まだ旋回していない(反転判定をしない)。
# et_rally_drive.pyの_last_turn_directionとは別物(クラス間でモジュール変数も
# 共有しない設計方針のため)。
_last_turn_direction = 0


def _current_heading():
    return (-1) * runtime.course * runtime.gyro_sensor.get_angle()


def _lookup_ticks(target_deg, table):
    """target_deg(符号つき)に一番近い表の行から、tick数を引く。0未満ならtick=0。"""
    target_abs = abs(target_deg)
    if target_abs < 1e-9 or not table:
        return 0
    best = min(table, key=lambda row: abs(row[0] - target_abs))
    return best[1]


class EtRallyTurnOneShot(Behaviour):
    def __init__(self, name: str, target: float, target_type: HeadingType,
                 main_power: int, decel_deg: float = 15.0, decel_power: int = 50,
                 floor_power: int = FLOOR_POWER, pulse_table=None,
                 fine_tolerance_deg: float = 0.5, max_pulse_attempts: int = MAX_PULSE_ATTEMPTS,
                 lag_compensation_deg: float = 0.0,
                 adaptive_lag_compensation: bool = False,
                 backlash_preload_deg: float = 0.0,
                 left_target_scale: float = 1.0,
                 phase1_scale=(1.0, 1.0),
                 pre_stop: bool = False,
                 settle_stable_ticks: int = SETTLE_STABLE_TICKS,
                 settle_max_ticks: int = SETTLE_MAX_TICKS,
                 phase1_model=None) -> None:
        super().__init__(name)
        # 2026-10-31: phase1の回転量を「実際の角度 = 係数×指令の角度 + 一定の流れ(度)」で見込む
        # ((headingが増える向きの係数, 流れ), (減る向きの係数, 流れ))。指定時はphase1_scaleを使わない。
        # 10/30 13:14周回の27旋回で、phase1の実際の回転には旋回の大きさによらない流れが加わっており、
        # 比例の係数(phase1_scale)では90度以外で外れていた(大きい左回りで7〜11度足りず、40度前後で
        # 2〜4度行き過ぎ)。指令の角度は左右の車輪の基準の回転量(引き込まれ補正で先に止まる側を
        # 短くする前)で、引き込まれ補正は通常どおりかける。
        # 2026-10-31: 引き込まれ補正を外した版(左右の目標を同じにした)では、±40/90/110度の旋回30回で
        # 車体が左へ6cm動いた(補正ありの従来方式は右0.3cm・前2.5cm)。先に止まった車輪は反対側に
        # 引きずられて12〜24度(エンコーダ)余分に回るため、補正がないと左右の回転量がそろわない。
        self.phase1_model = phase1_model
        self.target = target
        self.target_type = target_type
        self.main_power = main_power
        self.decel_deg = decel_deg
        self.decel_power = decel_power
        self.floor_power = floor_power
        self.pulse_table = pulse_table if pulse_table is not None else PULSE_TABLE
        self.fine_tolerance_deg = fine_tolerance_deg
        self.max_pulse_attempts = max_pulse_attempts
        # DD-22較正用: et_rally_drive.EtRallySpinAroundByEncoderと同じ、片輪が先に
        # 止まった後の引き込まれ分の補正。0.0/Falseなら無効(既定、挙動は変えない)。
        self.lag_compensation_deg = lag_compensation_deg
        self.adaptive_lag_compensation = adaptive_lag_compensation
        # DD-22較正用: 前の旋回と逆方向に切り替わる旋回だけ、phase1の目標エンコーダ
        # 角度にこの分を上乗せする(機構内のバックラッシュの先詰め)。0なら無効。
        self.backlash_preload_deg = backlash_preload_deg
        # DD-22較正用: 右輪の方が同じエンコーダ回転量に対して実際の方位を多く変える
        # (実効径・グリップの定常的な左右差)分の補正。常に左輪の目標値にだけかける。
        # 1.0なら無効(既定、挙動は変えない)。
        self.left_target_scale = left_target_scale
        # 2026-10-28: phase1の目標エンコーダ角度に掛ける係数(headingが増える向きの旋回,
        # 減る向きの旋回)。目標角度->エンコーダ角度の換算はタイヤ径と車輪間隔だけで
        # 決めているため、その場旋回の滑りや左右の止まり方の差で、向きごとに実際の
        # 回転量がずれる。旋回30回テストのログから
        # wireless_device/et_rally_lqi/phase1_scale_from_log.py で求めた値を使う。
        # (1.0, 1.0)なら無効(既定、挙動は変えない)。
        self.phase1_scale = phase1_scale
        # 2026-10-28: Trueなら、phase1の前に車体が止まるまで待つ。直進の直後の旋回では、
        # 直進の勢いが残ったまま回り始めるため、phase1後の流れが旋回単体のテストより
        # 大きく(最大8〜10度)、小さい旋回ほど行き過ぎていた(周回コースのログ)。
        # 止まってから回れば、旋回30回テスト(止まった状態から回る)と同じ条件になる。
        self.pre_stop = pre_stop
        self.pre_stopping = False
        self.settle_stable_ticks = settle_stable_ticks
        self.settle_max_ticks = settle_max_ticks
        self.settling = False
        self.running = False

    # --- フェーズ1: エンコーダ主導(EtRallySpinAroundByEncoder/EtRallyTurnLqiと同じ考え方) ---

    def _enter_phase1(self, current_heading):
        if self.target_type == HeadingType.RELATIVE:
            self.target_heading = current_heading + self.target
        else:
            self.target_heading = self.target
        error = (float(self.target_heading) - current_heading + 180.0) % 360.0 - 180.0
        arc_length_mm = math.radians(abs(error)) * (WHEEL_TREAD / 2.0)
        # 直進・EtRallySpinAroundByEncoderと同じタイヤ径を使う(固定値だと
        # --rally-gyro-tire-calibration offの切替が反映されない)。
        self.target_motor_deg = arc_length_mm / (math.pi * runtime.plotter.tire_diameter) * 360.0
        self.direction = 1 if error > 0 else -1
        if self.phase1_model is not None:
            gain, flow = self.phase1_model[0] if self.direction > 0 else self.phase1_model[1]
            # 小さい旋回は速度が上がりきらず流れも小さい(右回り9度で+1.4度、17〜25度で+2.7〜3.2度)ため、
            # PHASE1_FLOW_FULL_DEG未満では流れを角度に比例して小さく見込む。
            flow *= min(1.0, abs(error) / PHASE1_FLOW_FULL_DEG)
            # 流れだけで目標に届く旋回は、phase1を回さずにパルスで仕上げる。
            command_deg = max((abs(error) - flow) / gain, 0.0)
            self.applied_phase1_scale = command_deg / abs(error) if abs(error) > 1e-9 else 0.0
            self.target_motor_deg = (math.radians(command_deg) * (WHEEL_TREAD / 2.0)
                                     / (math.pi * runtime.plotter.tire_diameter) * 360.0)
        else:
            self.applied_phase1_scale = self.phase1_scale[0] if self.direction > 0 else self.phase1_scale[1]
            self.target_motor_deg *= self.applied_phase1_scale

        # DD-22較正用: et_rally_drive.EtRallySpinAroundByEncoderと同じ考え方(前の
        # 旋回と逆方向に切り替わるときだけ、機構内のバックラッシュを先に詰める分を
        # 上乗せする)。
        global _last_turn_direction
        if (self.backlash_preload_deg > 0.0 and _last_turn_direction != 0
                and self.direction != _last_turn_direction
                and (self.phase1_model is None or self.target_motor_deg > 0.0)):
            self.target_motor_deg += self.backlash_preload_deg
        _last_turn_direction = self.direction

        # DD-22較正用: et_rally_drive.EtRallySpinAroundByEncoderと同じ考え方(先に
        # 止まる側は、もう片方がまだ回っている間の反力で引き込まれて動く分だけ、
        # 目標値を低くしておく)。right_power_sign<0の側(=course*direction<0)が
        # 先に止まる(実測、2026-10-10)。
        global _lag_estimate_ratio
        compensation_ratio = (_lag_estimate_ratio if self.adaptive_lag_compensation
                               else self.lag_compensation_deg / _LAG_REFERENCE_MOTOR_DEG)
        self.right_base_motor_deg = self.target_motor_deg
        self.left_base_motor_deg = self.target_motor_deg * self.left_target_scale
        right_power_sign = runtime.course * self.direction
        self.early_side = 'right' if right_power_sign < 0 else 'left'
        if right_power_sign < 0:
            capped = min(self.right_base_motor_deg * compensation_ratio, self.right_base_motor_deg * 0.5)
            self.right_target_motor_deg = max(self.right_base_motor_deg - capped, 0.0)
            self.left_target_motor_deg = self.left_base_motor_deg
        else:
            self.right_target_motor_deg = self.right_base_motor_deg
            capped = min(self.left_base_motor_deg * compensation_ratio, self.left_base_motor_deg * 0.5)
            self.left_target_motor_deg = max(self.left_base_motor_deg - capped, 0.0)

        self.start_r = runtime.right_motor.get_count()
        self.start_l = runtime.left_motor.get_count()
        # 2026-10-30: ここから旋回が終わるまで、ETラリー用の位置(et_loc)は動かさない(et_rally_heading.py)。
        set_et_turning(True)
        self.right_done = self.right_target_motor_deg < 1e-6
        self.left_done = self.left_target_motor_deg < 1e-6
        self.right_done_at_count = self.start_r
        self.left_done_at_count = self.start_l
        # 2026-10-31: 旋回のどの区間で左右の車輪の回転量がずれるかを見るための記録(ログ専用)。
        # 0⇔110度の往復30回で車体が左へ10〜13cm動き、plotterの位置(旋回中もエンコーダを足す)も
        # 同じ向きに動いていたため、区間ごとの左右の回転量を残す。
        self.wheel_marks = [("start", self.start_r, self.start_l, current_heading)]
        self.phase = 1
        self.logger.info(
            "%+06d %s.phase1 started at heading=%d for %d (target_motor_deg=%.1f) phase1_scale=%.4f" % (
                runtime.plotter.get_distance(), self.__class__.__name__, current_heading,
                self.target_heading, self.target_motor_deg, self.applied_phase1_scale))

    def _tick_phase1(self):
        delta_r = abs(runtime.right_motor.get_count() - self.start_r)
        delta_l = abs(runtime.left_motor.get_count() - self.start_l)
        if not self.right_done and delta_r >= self.right_target_motor_deg:
            self.right_done = True
            self.right_done_at_count = runtime.right_motor.get_count()
            runtime.right_motor.set_power(0)
            runtime.right_motor.set_brake(True)
        if not self.left_done and delta_l >= self.left_target_motor_deg:
            self.left_done = True
            self.left_done_at_count = runtime.left_motor.get_count()
            runtime.left_motor.set_power(0)
            runtime.left_motor.set_brake(True)
        if (self.right_done != self.left_done) and len(self.wheel_marks) == 1:
            # 片方の車輪だけが止まった時点(ここから先は片輪だけが回る)。
            self._mark_wheels("both", _current_heading())
        if self.right_done and self.left_done:
            lag_right_delta = runtime.right_motor.get_count() - self.right_done_at_count
            lag_left_delta = runtime.left_motor.get_count() - self.left_done_at_count
            if self.adaptive_lag_compensation:
                global _lag_estimate_ratio
                observed = abs(lag_right_delta if self.early_side == 'right' else lag_left_delta)
                early_base = (self.right_base_motor_deg if self.early_side == 'right'
                               else self.left_base_motor_deg)
                if early_base > _LAG_ESTIMATE_MIN_EARLY_BASE_DEG:
                    observed_ratio = min(observed / early_base, _LAG_ESTIMATE_MAX_RATIO)
                    _lag_estimate_ratio = (_LAG_ESTIMATE_ALPHA * observed_ratio
                                           + (1.0 - _LAG_ESTIMATE_ALPHA) * _lag_estimate_ratio)
            self.logger.info(
                "%+06d %s.phase1 lag check: lag_right_delta=%+d lag_left_delta=%+d "
                "lag_estimate_ratio=%.3f" % (
                    runtime.plotter.get_distance(), self.__class__.__name__,
                    lag_right_delta, lag_left_delta, _lag_estimate_ratio))
            return True

        use_decel = self.decel_deg > 0 and self.target_motor_deg >= 2 * self.decel_deg
        right_power = left_power = 0
        if not self.right_done:
            p = (self.decel_power if use_decel and self.right_target_motor_deg - delta_r < self.decel_deg
                 else self.main_power)
            right_power = runtime.course * self.direction * p
            runtime.right_motor.set_power(right_power)
        if not self.left_done:
            p = (self.decel_power if use_decel and self.left_target_motor_deg - delta_l < self.decel_deg
                 else self.main_power)
            left_power = (-1) * runtime.course * self.direction * p
            runtime.left_motor.set_power(left_power)
        log_power_state(self.__class__.__name__ + ".phase1", right_power, left_power)
        return False

    # --- フェーズ2: 一発サイズのパルス補正 ---

    def _mark_wheels(self, label, current_heading):
        self.wheel_marks.append((label, runtime.right_motor.get_count(),
                                 runtime.left_motor.get_count(), current_heading))

    def _wheel_marks_text(self):
        """区間ごとの左右の回転量(度)・車体中心の前後(mm)・向きの変化(度)。"""
        mm_per_deg = math.pi * runtime.plotter.tire_diameter / 360.0
        first, last = self.wheel_marks[0], self.wheel_marks[-1]
        parts = ["total dR=%+d dL=%+d fwd=%+.1f dh=%+.1f" % (
            last[1] - first[1], last[2] - first[2],
            ((last[1] - first[1]) + (last[2] - first[2])) * mm_per_deg / 2.0, last[3] - first[3])]
        for prev, cur in zip(self.wheel_marks, self.wheel_marks[1:]):
            d_r, d_l = cur[1] - prev[1], cur[2] - prev[2]
            parts.append("%s dR=%+d dL=%+d fwd=%+.1f dh=%+.1f" % (
                cur[0], d_r, d_l, (d_r + d_l) * mm_per_deg / 2.0, cur[3] - prev[3]))
        return " | ".join(parts)

    def _enter_phase2(self, current_heading):
        if len(self.wheel_marks) == 1:
            # 左右が同じtickで止まった(片輪だけの区間がない)。
            self._mark_wheels("both", current_heading)
        self._mark_wheels("tail", current_heading)
        self.phase = 2
        self.pulse_attempt = 0
        self._begin_settle(current_heading, "phase1")

    def _begin_settle(self, current_heading, reason):
        self._stop_motors()
        self.settling = True
        self.settle_reason = reason
        self.settle_start_heading = current_heading
        self.settle_prev_heading = current_heading
        self.settle_stable_count = 0
        self.settle_ticks = 0

    def _tick_settle(self, current_heading):
        """静止を確認できたらTrue。headingは約1度刻みなので、値が同じかどうかで判定する。"""
        self.settle_ticks += 1
        if abs(current_heading - self.settle_prev_heading) < SETTLE_STABLE_DEG:
            self.settle_stable_count += 1
        else:
            self.settle_stable_count = 0
        self.settle_prev_heading = current_heading
        if (self.settle_stable_count < self.settle_stable_ticks
                and self.settle_ticks < self.settle_max_ticks):
            return False
        self.settling = False
        self._mark_wheels("coast" if self.settle_reason == "phase1" else self.settle_reason, current_heading)
        self.logger.info(
            "%+06d %s.phase2 settled after %s ticks=%d drift=%+.2f%s" % (
                runtime.plotter.get_distance(), self.__class__.__name__, self.settle_reason,
                self.settle_ticks, current_heading - self.settle_start_heading,
                "" if self.settle_stable_count >= self.settle_stable_ticks else " (timeout)"))
        return True

    def _start_pulse(self, current_heading):
        error = (float(self.target_heading) - current_heading + 180.0) % 360.0 - 180.0
        ticks = _lookup_ticks(error, self.pulse_table)
        self.pulse_ticks_remaining = ticks
        self.pulse_power = int(math.copysign(self.floor_power, error)) if ticks > 0 else 0
        self.pulse_attempt += 1
        self.logger.info(
            "%+06d %s.phase2 pulse#%d error=%+.2f ticks=%d power=%+d" % (
                runtime.plotter.get_distance(), self.__class__.__name__,
                self.pulse_attempt, error, ticks, self.pulse_power))
        if ticks == 0:
            self._stop_motors()

    def _stop_motors(self):
        runtime.right_motor.set_power(0)
        runtime.right_motor.set_brake(True)
        runtime.left_motor.set_power(0)
        runtime.left_motor.set_brake(True)

    def _tick_phase2(self, current_heading):
        if self.settling:
            if not self._tick_settle(current_heading):
                return "running"
            error = (float(self.target_heading) - current_heading + 180.0) % 360.0 - 180.0
            # 2026-10-28: 1回目のパルスの前にも許容範囲を確認する。headingは約1度刻みでしか
            # 読めないため、目盛り1つ分の誤差(例: 目標90度に対し89.57度=-0.57度)に1tickの
            # パルスを打っても、実機ではほぼ動かず時間だけかかっていた(旋回30回テストの
            # ログ、drift=+0.00)。--rally-turn-tolerance-deg 0.6 と組み合わせて使う。
            if self.pulse_attempt == 0 and abs(error) >= self.fine_tolerance_deg:
                self._start_pulse(current_heading)
                return "running"
            if abs(error) < self.fine_tolerance_deg or self.pulse_attempt >= self.max_pulse_attempts:
                turn_forward_mm = forward_mm_since(self.start_r, self.start_l)
                # 2026-10-29: 次の直進が(オプション有効時に)この分だけ短く走れるよう渡す。
                set_turn_forward_mm(turn_forward_mm)
                set_et_turning(False)
                self.logger.info(
                    "%+06d %s.phase2 done attempts=%d error=%+.2f fwd_mm=%+.1f loc=(%d,%d)%s" % (
                        runtime.plotter.get_distance(), self.__class__.__name__,
                        self.pulse_attempt, error, turn_forward_mm,
                        runtime.plotter.get_loc_x(), runtime.plotter.get_loc_y(), et_pose_text()))
                self.logger.info("%+06d %s.wheels %s" % (
                    runtime.plotter.get_distance(), self.__class__.__name__, self._wheel_marks_text()))
                return "done"
            # 1回で収まらなかった場合、残差を測り直してもう1回だけ試す。
            self._start_pulse(current_heading)
            return "running"

        if self.pulse_ticks_remaining > 0:
            right_power = runtime.course * self.pulse_power
            left_power = (-1) * runtime.course * self.pulse_power
            runtime.right_motor.set_power(right_power)
            runtime.left_motor.set_power(left_power)
            log_power_state(self.__class__.__name__ + ".phase2", right_power, left_power, interval_ticks=1)
            self.pulse_ticks_remaining -= 1
            return "running"

        self._begin_settle(current_heading, "pulse#%d" % self.pulse_attempt)
        return "running"

    # --- Behaviour本体 ---

    def update(self) -> Status:
        runtime.require("plotter", "gyro_sensor", "right_motor", "left_motor")
        if not self.running:
            ensure_scaled_gyro()
            self.running = True
            if self.pre_stop:
                self._stop_motors()
                self.pre_stopping = True
                self.pre_stop_ticks = 0
                self.pre_stop_stable = 0
                self.pre_stop_start_r = runtime.right_motor.get_count()
                self.pre_stop_start_l = runtime.left_motor.get_count()
                self.pre_stop_prev = (self.pre_stop_start_r, self.pre_stop_start_l)
                self.pre_stop_start_heading = _current_heading()
                return Status.RUNNING
            self._enter_phase1(_current_heading())

        if self.pre_stopping:
            counts = (runtime.right_motor.get_count(), runtime.left_motor.get_count())
            self.pre_stop_ticks += 1
            self.pre_stop_stable = self.pre_stop_stable + 1 if counts == self.pre_stop_prev else 0
            self.pre_stop_prev = counts
            if self.pre_stop_stable < PRE_STOP_STABLE_TICKS and self.pre_stop_ticks < PRE_STOP_MAX_TICKS:
                return Status.RUNNING
            current_heading = _current_heading()
            self.logger.info("%+06d %s.pre-stop settled ticks=%d fwd_mm=%+.1f drift=%+.2f%s" % (
                runtime.plotter.get_distance(), self.__class__.__name__, self.pre_stop_ticks,
                forward_mm_since(self.pre_stop_start_r, self.pre_stop_start_l),
                current_heading - self.pre_stop_start_heading,
                "" if self.pre_stop_stable >= PRE_STOP_STABLE_TICKS else " (timeout)"))
            self.pre_stopping = False
            self._enter_phase1(current_heading)

        current_heading = _current_heading()
        if self.phase == 1:
            if self._tick_phase1():
                self._enter_phase2(current_heading)
            return Status.RUNNING

        result = self._tick_phase2(current_heading)
        if result == "running":
            return Status.RUNNING
        return Status.SUCCESS

    def terminate(self, new_status: Status) -> None:
        if runtime.right_motor is not None:
            runtime.right_motor.set_power(0)
        if runtime.left_motor is not None:
            runtime.left_motor.set_power(0)
        self.settling = False
        self.pre_stopping = False
        self.running = False
        set_et_turning(False)
