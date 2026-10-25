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

# 2026-10-25: b, cの求め方を、隣接サンプルの差分でomegaを作る方式(fit_b_c())から、
# 各試行のheading(t)の軌跡全体にモデルを当てはめる方式(fit_b_c_trajectory())に変更した。
# 差分方式は、headingの生の分解能が粗い(約1度刻み、ジャイロ側の仕様と見られる)ため、
# omegaの値そのものに強い量子化ノイズが乗り(R^2=0.16程度)、特に出力レベルごとにb・cが
# 大きくばらつく不具合があった。軌跡全体フィットではR^2=0.98まで改善した。
# また、不感帯の閾値(power=35 CW/25 CCW)に近いpower=40の較正データは、滑らかな線形モデルで
# 説明できない(stick-slip的な)挙動を示すため、フィットから除外し、power=50/60/70のみ
# (run_20261025T083138、et-rally-turn-calibration)で求めた: b=10.360649, c=3.942615
# (R^2=0.98、theta軌跡基準、CW/CCW合算)。stiction_floorはFindStictionThresholdの実測
# (CW power=30, CCW power=20で初動)に安全マージン3を足した33.0(2026-10-24時点の値を維持。
# 直近の較正ではCW=35/CCW=25に上がっているが、今回はb・cの求め方の変更だけを検証するため、
# stiction_floor/FLOOR_POWERはあえて変更していない)。
# floor_power=40(stiction_floor+7)はpower=50より1tick(0.02s)刻みの分解能が
# 細かく、仕上げで想定する数度以下の残差に向いているため採用。
#
# 2026-10-24: 実機テスト(one_shot初回投入)で、phase1の残差が5度を大きく超える
# ケース(最大29.55度)が頻発し、_lookup_ticks()が表の最大行(旧:5.00度→24tick)
# しか引けず大幅に補正不足になる不具合が見つかった。build_pulse_table()を
# max_target_deg=30.0で再実行し、表を0.25〜30.00度まで拡張した(この拡張範囲は維持)。
FLOOR_POWER = 40
PULSE_TABLE = [
    (0.25, 4, 0.2681), (0.50, 6, 0.5352), (0.75, 7, 0.6960), (1.00, 9, 1.0655),
    (1.25, 10, 1.2715), (1.50, 11, 1.4904), (1.75, 12, 1.7210), (2.00, 13, 1.9624),
    (2.25, 14, 2.2138), (2.50, 15, 2.4744), (2.75, 16, 2.7434), (3.00, 17, 3.0203),
    (3.25, 18, 3.3043), (3.50, 19, 3.5949), (3.75, 20, 3.8917), (4.00, 20, 3.8917),
    (4.25, 21, 4.1940), (4.50, 22, 4.5015), (4.75, 23, 4.8138), (5.00, 24, 5.1304),
    (5.25, 24, 5.1304), (5.50, 25, 5.4511), (5.75, 26, 5.7756), (6.00, 27, 6.1034),
    (6.25, 27, 6.1034), (6.50, 28, 6.4344), (6.75, 29, 6.7683), (7.00, 30, 7.1049),
    (7.25, 30, 7.1049), (7.50, 31, 7.4440), (7.75, 32, 7.7853), (8.00, 33, 8.1288),
    (8.25, 33, 8.1288), (8.50, 34, 8.4741), (8.75, 35, 8.8213), (9.00, 36, 9.1700),
    (9.25, 36, 9.1700), (9.50, 37, 9.5203), (9.75, 38, 9.8720), (10.00, 38, 9.8720),
    (10.25, 39, 10.2250), (10.50, 40, 10.5791), (10.75, 40, 10.5791), (11.00, 41, 10.9343),
    (11.25, 42, 11.2905), (11.50, 43, 11.6477), (11.75, 43, 11.6477), (12.00, 44, 12.0056),
    (12.25, 45, 12.3644), (12.50, 45, 12.3644), (12.75, 46, 12.7239), (13.00, 47, 13.0841),
    (13.25, 47, 13.0841), (13.50, 48, 13.4448), (13.75, 49, 13.8061), (14.00, 50, 14.1680),
    (14.25, 50, 14.1680), (14.50, 51, 14.5303), (14.75, 52, 14.8931), (15.00, 52, 14.8931),
    (15.25, 53, 15.2562), (15.50, 54, 15.6198), (15.75, 54, 15.6198), (16.00, 55, 15.9837),
    (16.25, 56, 16.3479), (16.50, 56, 16.3479), (16.75, 57, 16.7124), (17.00, 58, 17.0771),
    (17.25, 58, 17.0771), (17.50, 59, 17.4421), (17.75, 60, 17.8074), (18.00, 61, 18.1728),
    (18.25, 61, 18.1728), (18.50, 62, 18.5385), (18.75, 63, 18.9043), (19.00, 63, 18.9043),
    (19.25, 64, 19.2703), (19.50, 65, 19.6364), (19.75, 65, 19.6364), (20.00, 66, 20.0027),
    (20.25, 67, 20.3691), (20.50, 67, 20.3691), (20.75, 68, 20.7356), (21.00, 69, 21.1022),
    (21.25, 69, 21.1022), (21.50, 70, 21.4689), (21.75, 71, 21.8358), (22.00, 71, 21.8358),
    (22.25, 72, 22.2027), (22.50, 73, 22.5697), (22.75, 73, 22.5697), (23.00, 74, 22.9367),
    (23.25, 75, 23.3038), (23.50, 76, 23.6710), (23.75, 76, 23.6710), (24.00, 77, 24.0383),
    (24.25, 78, 24.4056), (24.50, 78, 24.4056), (24.75, 79, 24.7729), (25.00, 80, 25.1403),
    (25.25, 80, 25.1403), (25.50, 81, 25.5077), (25.75, 82, 25.8752), (26.00, 82, 25.8752),
    (26.25, 83, 26.2427), (26.50, 84, 26.6102), (26.75, 84, 26.6102), (27.00, 85, 26.9778),
    (27.25, 86, 27.3453), (27.50, 86, 27.3453), (27.75, 87, 27.7130), (28.00, 88, 28.0806),
    (28.25, 88, 28.0806), (28.50, 89, 28.4482), (28.75, 90, 28.8159), (29.00, 91, 29.1836),
    (29.25, 91, 29.1836), (29.50, 92, 29.5513), (29.75, 93, 29.9190), (30.00, 93, 29.9190),
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
