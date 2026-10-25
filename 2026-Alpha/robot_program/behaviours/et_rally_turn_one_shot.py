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
from ..power_log import log_power_state
from ..runtime import runtime
from ..types import HeadingType

# 2026-10-24: et-rally-turn-calibrationの実機ログ(run_20261024T063919)から
# run_pipeline.pyで実測したb, c, stiction_floorでbuild_pulse_table()を実行し、
# 差し替えた(机上の仮値から実測値へ)。b=44.828271, c=28.634468
# (R^2=0.30、CW/CCW合算フィット)。stiction_floorはFindStictionThresholdの実測
# (CW power=30, CCW power=20で初動)に安全マージン3を足した33.0。
# floor_power=40(stiction_floor+7)はpower=50より1tick(0.02s)刻みの分解能が
# 細かく、仕上げで想定する数度以下の残差に向いているため採用。
#
# 2026-10-24: 実機テスト(one_shot初回投入)で、phase1の残差が5度を大きく超える
# ケース(最大29.55度)が頻発し、_lookup_ticks()が表の最大行(旧:5.00度→24tick)
# しか引けず大幅に補正不足になる不具合が見つかった。build_pulse_table()を
# max_target_deg=30.0で再実行し、表を0.25〜30.00度まで拡張した。
FLOOR_POWER = 40
PULSE_TABLE = [
    (0.25, 2, 0.3047), (0.50, 3, 0.5067), (0.75, 4, 0.7186), (1.00, 5, 0.9347),
    (1.25, 6, 1.1525), (1.50, 8, 1.5900), (1.75, 9, 1.8091), (2.00, 10, 2.0282),
    (2.25, 11, 2.2474), (2.50, 12, 2.4666), (2.75, 13, 2.6857), (3.00, 14, 2.9049),
    (3.25, 16, 3.3433), (3.50, 17, 3.5624), (3.75, 18, 3.7816), (4.00, 19, 4.0008),
    (4.25, 20, 4.2200), (4.50, 21, 4.4391), (4.75, 22, 4.6583), (5.00, 24, 5.0967),
    (5.25, 25, 5.3158), (5.50, 26, 5.5350), (5.75, 27, 5.7542), (6.00, 28, 5.9734),
    (6.25, 29, 6.1925), (6.50, 30, 6.4117), (6.75, 32, 6.8501), (7.00, 33, 7.0692),
    (7.25, 34, 7.2884), (7.50, 35, 7.5076), (7.75, 36, 7.7268), (8.00, 37, 7.9459),
    (8.25, 38, 8.1651), (8.50, 40, 8.6035), (8.75, 41, 8.8226), (9.00, 42, 9.0418),
    (9.25, 43, 9.2610), (9.50, 44, 9.4802), (9.75, 45, 9.6993), (10.00, 46, 9.9185),
    (10.25, 48, 10.3569), (10.50, 49, 10.5760), (10.75, 50, 10.7952), (11.00, 51, 11.0144),
    (11.25, 52, 11.2336), (11.50, 53, 11.4527), (11.75, 54, 11.6719), (12.00, 55, 11.8911),
    (12.25, 57, 12.3294), (12.50, 58, 12.5486), (12.75, 59, 12.7678), (13.00, 60, 12.9870),
    (13.25, 61, 13.2061), (13.50, 62, 13.4253), (13.75, 63, 13.6445), (14.00, 65, 14.0828),
    (14.25, 66, 14.3020), (14.50, 67, 14.5212), (14.75, 68, 14.7404), (15.00, 69, 14.9595),
    (15.25, 70, 15.1787), (15.50, 71, 15.3979), (15.75, 73, 15.8362), (16.00, 74, 16.0554),
    (16.25, 75, 16.2746), (16.50, 76, 16.4938), (16.75, 77, 16.7129), (17.00, 78, 16.9321),
    (17.25, 79, 17.1513), (17.50, 81, 17.5896), (17.75, 82, 17.8088), (18.00, 83, 18.0280),
    (18.25, 84, 18.2472), (18.50, 85, 18.4663), (18.75, 86, 18.6855), (19.00, 87, 18.9047),
    (19.25, 89, 19.3430), (19.50, 90, 19.5622), (19.75, 91, 19.7814), (20.00, 92, 20.0006),
    (20.25, 93, 20.2197), (20.50, 94, 20.4389), (20.75, 95, 20.6581), (21.00, 97, 21.0964),
    (21.25, 98, 21.3156), (21.50, 99, 21.5348), (21.75, 100, 21.7540), (22.00, 101, 21.9731),
    (22.25, 102, 22.1923), (22.50, 103, 22.4115), (22.75, 105, 22.8498), (23.00, 106, 23.0690),
    (23.25, 107, 23.2882), (23.50, 108, 23.5074), (23.75, 109, 23.7265), (24.00, 110, 23.9457),
    (24.25, 111, 24.1649), (24.50, 113, 24.6032), (24.75, 114, 24.8224), (25.00, 115, 25.0416),
    (25.25, 116, 25.2608), (25.50, 117, 25.4799), (25.75, 118, 25.6991), (26.00, 119, 25.9183),
    (26.25, 121, 26.3566), (26.50, 122, 26.5758), (26.75, 123, 26.7950), (27.00, 124, 27.0142),
    (27.25, 125, 27.2333), (27.50, 126, 27.4525), (27.75, 127, 27.6717), (28.00, 128, 27.8909),
    (28.25, 130, 28.3292), (28.50, 131, 28.5484), (28.75, 132, 28.7676), (29.00, 133, 28.9867),
    (29.25, 134, 29.2059), (29.50, 135, 29.4251), (29.75, 136, 29.6443), (30.00, 138, 30.0826),
]
MAX_PULSE_ATTEMPTS = 2  # これ以上は追いかけない(不感帯より細かい残差を無理に追わない)

# DD-22較正用(et_rally_drive.pyと同じ考え方を、このクラス独立で持つ。
# クラス間の変更・継承はしない設計方針のため、モジュール変数も別に持つ)。
# phase1で先に止まった側が、もう片方の反力で引き込まれて動いてしまう分の
# 推定値(指数移動平均、target_motor_degに対する比率)。初期値はet_rally_drive.py
# と同じ実測(2026-10-10、power=70・90度旋回・n=60、平均22.67度/121度)から。
_LAG_REFERENCE_MOTOR_DEG = 121.0
_lag_estimate_ratio = 22.67 / _LAG_REFERENCE_MOTOR_DEG
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
                 left_target_scale: float = 1.0) -> None:
        super().__init__(name)
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

        # DD-22較正用: et_rally_drive.EtRallySpinAroundByEncoderと同じ考え方(前の
        # 旋回と逆方向に切り替わるときだけ、機構内のバックラッシュを先に詰める分を
        # 上乗せする)。
        global _last_turn_direction
        if (self.backlash_preload_deg > 0.0 and _last_turn_direction != 0
                and self.direction != _last_turn_direction):
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
        self.right_done = self.right_target_motor_deg < 1e-6
        self.left_done = self.left_target_motor_deg < 1e-6
        self.right_done_at_count = self.start_r
        self.left_done_at_count = self.start_l
        self.phase = 1
        self.logger.info(
            "%+06d %s.phase1 started at heading=%d for %d (target_motor_deg=%.1f)" % (
                runtime.plotter.get_distance(), self.__class__.__name__, current_heading,
                self.target_heading, self.target_motor_deg))

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

    def _enter_phase2(self, current_heading):
        self.phase = 2
        self.pulse_attempt = 0
        self._start_pulse(current_heading)

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
        if self.pulse_ticks_remaining > 0:
            right_power = runtime.course * self.pulse_power
            left_power = (-1) * runtime.course * self.pulse_power
            runtime.right_motor.set_power(right_power)
            runtime.left_motor.set_power(left_power)
            log_power_state(self.__class__.__name__ + ".phase2", right_power, left_power, interval_ticks=1)
            self.pulse_ticks_remaining -= 1
            return "running"

        self._stop_motors()
        error = (float(self.target_heading) - current_heading + 180.0) % 360.0 - 180.0
        if abs(error) < self.fine_tolerance_deg or self.pulse_attempt >= self.max_pulse_attempts:
            self.logger.info(
                "%+06d %s.phase2 done attempts=%d error=%+.2f loc=(%d,%d)" % (
                    runtime.plotter.get_distance(), self.__class__.__name__,
                    self.pulse_attempt, error,
                    runtime.plotter.get_loc_x(), runtime.plotter.get_loc_y()))
            return "done"
        # 1回で収まらなかった場合、残差を測り直してもう1回だけ試す。
        self._start_pulse(current_heading)
        return "running"

    # --- Behaviour本体 ---

    def update(self) -> Status:
        runtime.require("plotter", "gyro_sensor", "right_motor", "left_motor")
        if not self.running:
            ensure_scaled_gyro()
            current_heading = _current_heading()
            self._enter_phase1(current_heading)
            self.running = True

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
        self.running = False
