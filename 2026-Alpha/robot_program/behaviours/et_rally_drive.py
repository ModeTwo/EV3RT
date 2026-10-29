"""ETラリー専用の走行ビヘイビア。

sample_comment.py(ETラリー用にカスタムしたスタンドアロン版)のSpinAroundByEncoder/
RunByGyroを、robot_programのruntime経由に移植したもの。共通のgyro_drive.pyの
SpinAround/RunByGyroとは別クラスで、既存クラスの挙動は変えない。
ジャイロ値は倍率補正済みの値(robot_program/gyro_scale.pyのScaledGyro。GYRO_SCALE_FACTOR適用済み)を読む点、旋回を
エンコーダ主導(フェーズ1)+ジャイロ仕上げ(フェーズ2)で行う点が共通版との違い。
"""

import math
import time

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from simple_pid import PID

from py_etrobo_util import SymmetricClamper
from py_etrobo_util.plotter import WHEEL_TREAD

from ..gyro_scale import ensure_scaled_gyro
from ..power_log import OscillationTracker, log_power_state
from ..runtime import runtime
from ..timing import CONTROL_INTERVAL_SEC as EXEC_INTERVAL
from ..types import HeadingType

# sample_comment.pyは制御周期0.03秒で較正した。robot_programの周期(0.02秒)でも
# 同じ「時間」で判定するため、tick数・1tickあたりの角度で持つ閾値を周期比で換算する。
_TUNED_INTERVAL_SEC = 0.03
_STALL_TICK_LIMIT = math.ceil(7 * _TUNED_INTERVAL_SEC / EXEC_INTERVAL - 1e-9)  # 0.03s x 7 = 約0.2秒
_STALL_PROGRESS_DEG = 0.3 * EXEC_INTERVAL / _TUNED_INTERVAL_SEC  # 1tickあたり0.3度=10度/秒未満

# DD-22較正用: 直前の旋回の方向(+1/-1)。0=まだ旋回していない(反転判定をしない)。
# EtRallySpinAroundByEncoderの各インスタンスは旋回ごとに別オブジェクトのため、
# 「前の旋回と逆方向か」をまたいで知るには、この窓口(モジュール変数)で持つ必要がある。
_last_turn_direction = 0

# DD-22較正用: 適応モード(adaptive_lag_compensation)で使う、引き込まれ量の
# 実測からの推定値(指数移動平均)。固定の度数だと、較正に使った90度旋回より
# 小さい旋回(フル周回には色々な大きさが混ざる)では過大になり、0にクランプされる
# 不具合が実機で発生した(2026-10-10)。そのため度数ではなく、その旋回の
# target_motor_degに対する比率(引き込まれ量÷先に止まる側の目標値)として持つ。
# 初期値は、2026-10-10の実機power=70の2回分(各n=30、計60サンプル、90度旋回・
# target_motor_deg≈121)の平均22.67度を121で割った比率。
_LAG_REFERENCE_MOTOR_DEG = 121.0
_lag_estimate_ratio = 22.67 / _LAG_REFERENCE_MOTOR_DEG
# 2026-10-10: 実測2回分からノイズ(標準偏差2.6〜4.0度)と傾向(1旋回あたり約0.17度の
# 緩やかな減少)を見積もり、ノイズ低減と追従遅れのトレードオフを数値最小化した結果、
# 0.3よりも0.2の方がわずかに良い(誤差推定1.48度→1.37度)と分かったため変更。
_LAG_ESTIMATE_ALPHA = 0.2
# 2026-10-25較正用: 引き込まれ量(observed)は旋回の大きさに関係なくほぼ一定
# (実測17〜31度)なので、early_baseが小さい旋回では比率が実態と無関係に発散する
# (実機ログで1.7度・4.3度の旋回直後にratioが1.0超へ急騰する不具合を確認、2026-10-25)。
# 旧ガード(early_base>1.0)では防げなかったため、閾値を上げるのに加えて、
# 1サンプルが動かせる比率そのものにも上限をかける(どちらの値も実機データが
# 増えたら見直す前提の暫定値)。
_LAG_ESTIMATE_MIN_EARLY_BASE_DEG = 20.0
_LAG_ESTIMATE_MAX_RATIO = 0.5


def forward_mm_since(start_r, start_l):
    """start_r/start_l(左右エンコーダ値)から今までに、車体中心が前後に動いた量(mm、前が+)。

    2026-10-28: 直進の終了直後、止まりきる前に旋回へ移ったときの「直進の惰性で前へ
    流れた量」を診断するため追加(plotterと同じく左右の回転量の平均から求める。
    その場旋回なら左右が打ち消し合って0に近くなる)。ログ専用で、制御には使わない。
    """
    delta = (runtime.right_motor.get_count() - start_r) + (runtime.left_motor.get_count() - start_l)
    return math.pi * runtime.plotter.tire_diameter * delta / 2.0 / 360.0


# 2026-10-29: 旋回中に車体が前へ出た量(mm)を、次の直進へ渡すための受け渡し場所。
# one_shot旋回は最後に片方の車輪だけが回るため、車体の中心が次に走る向きへ数mm出る
# (周回コースのログで-1〜+10mm)。コースの形によっては周回ごとに同じ向きへ積み重なる
# (2026-10-28、ゴール向きに約7mm/周)。EtRallyRunByGyro(turn_forward_compensation=True)が
# 直進の開始時に受け取り、その分だけ短く走る。
_pending_turn_forward_mm = None


def set_turn_forward_mm(value):
    global _pending_turn_forward_mm
    _pending_turn_forward_mm = value


def take_turn_forward_mm():
    """渡された量を受け取り、受け渡し場所を空にする(次の直進で二重に引かないため)。"""
    global _pending_turn_forward_mm
    value, _pending_turn_forward_mm = _pending_turn_forward_mm, None
    return value


class EtRallySpinAroundByEncoder(Behaviour):
    """左右のタイヤの回転量(エンコーダ)を揃えて回すことで、「その場旋回」であること
    そのものを幾何学的に保証する旋回ノード。SpinAround(上記)の代替。

    背景(2026-09-12): SpinAroundはジャイロの角度だけを見て、目標角度に届くまで
    左右対称のパワーを与え続ける方式。最終的な向きの精度はジャイロがそのまま
    保証するが、左右のタイヤの実効径やグリップにわずかでも差があると、実際の
    回転中心が車軸の中心からズレてしまい、「その場」旋回のはずが車体全体が
    平行移動する問題があった。et_rally_plannerでの検証(±90度の旋回のみを
    30回繰り返すテスト)
    (2回の試行で
    ほぼ同じ量・同じ向き)位置ズレが見つかっている。ランダムな滑りにしては
    再現性が高すぎるため、左右のタイヤの機械的な非対称性(回転中心のオフセット)
    が主因と推定した。

    このノードでは、目標の回転角度を「左右の車輪が地面の上で進むべき円弧の
    長さ(符号は逆・大きさは同じ)」に変換し(WHEEL_TREAD/runtime.plotter.tire_diameter、
    直進と同じタイヤ径。py_etrobo_util/plotter.py参照)、実際に左右のエンコーダがその量だけ回転
    するまで駆動する(フェーズ1)。左右対称に回転させている限り、タイヤの
    実効径の絶対値の較正誤差があっても「その場」であることには影響しない。

    ただし旋回中にタイヤが滑ると(エンコーダは回っても実際には転がっていない)
    実際の角度がズレる可能性があるため、フェーズ1完了後にジャイロで最終角度を
    確認し、2度以上のズレが残っていれば低パワー(fine_min_power〜fine_max_power、
    SpinAroundのSPIN_MIN_POWER=60より大幅に低い値を想定)で小さく仕上げ補正する
    (フェーズ2)。フェーズ2の判定基準(2度以内)はSpinAroundと同一なので、
    最終的な角度精度は変えずに、旋回による位置ズレだけを減らす狙い。
    """
    def __init__(self, name: str, target: int,
                 main_power: int, fine_max_power: int, fine_min_power: int,
                 pid_p: float, pid_i: float, pid_d: float, target_type: HeadingType,
                 fine_tolerance_deg: float = 2.0,
                 decel_deg: float = 0.0, decel_power: int = 50,
                 fine_trim: bool = True, backlash_preload_deg: float = 0.0,
                 lag_compensation_deg: float = 0.0,
                 adaptive_lag_compensation: bool = False,
                 left_target_scale: float = 1.0) -> None:
        super(EtRallySpinAroundByEncoder, self).__init__(name)
        # DD-22較正用: 片輪エンコーダテストで実測した、右輪の方が同じエンコーダ回転量に
        # 対して約0.9%多く実際の方位を変える(実効径・グリップの定常的な左右差)分を
        # 補正する。旋回のたびの「引き込まれ」(lag_compensation_deg)とは別の、
        # 物理的な車輪そのものの特性による差なので、どちらが先に止まるかとは無関係に
        # 常に左輪の目標値だけにこの係数をかける。1.0なら無効(既定、挙動は変えない)。
        self.left_target_scale = left_target_scale
        # DD-22較正用: 前の旋回と逆方向に切り替わる旋回だけ、フェーズ1の目標エンコーダ角度に
        # この分を上乗せする。機構内のバックラッシュ(前の旋回で片側に詰まった遊びを、
        # 逆方向ではまず詰め直す必要がある)による「エンコーダは回るが実際には車輪が
        # 転がっていない」区間を事前に見込むためのもの。0なら無効(既定、挙動は変えない)。
        self.backlash_preload_deg = backlash_preload_deg
        # DD-22較正用: 先に目標へ到達して止まった側が、まだ回っているもう片方の反力で
        # 「止まった後も同じ方向へ引き込まれて動いてしまう」分(実測、旋回ごとに17〜31度と
        # 幅がある、power依存、2026-10-10)を見越して、先に止まる側(その旋回でマイナス
        # 出力になる側、後述)の目標値をこの分だけ低く設定しておく。0なら無効。
        self.lag_compensation_deg = lag_compensation_deg
        # True: 上のlag_compensation_degを固定値として使う代わりに、モジュール変数
        # _lag_estimate_ratio(target_motor_degに対する比率)を使う。これは毎回の
        # 旋回終了後に実測の引き込まれ量で指数移動平均として更新していくため、
        # 固定値では追従できない旋回ごとのばらつき・power依存・旋回の大きさの違いを、
        # 実行中に自動で合わせ込む狙い。
        self.adaptive_lag_compensation = adaptive_lag_compensation
        # 2026-09-20: フェーズ1(全力寄りのmain_power)のまま目標のエンコーダ角度に
        # 到達してブレーキをかけると、慣性で行き過ぎてフェーズ2の仕上げ補正が
        # 毎回必要になる。残りのエンコーダ角度がdecel_deg未満になったら
        # decel_powerに落として勢いを減らしてからブレーキをかける(0で無効)。
        # 静止状態からdecel_powerで始めると静止摩擦で動けない恐れがあるため、
        # 目標がdecel_degの2倍未満の小さい旋回では使わない。
        self.decel_deg = decel_deg
        self.decel_power = decel_power
        self.target = target
        self.target_type = target_type
        self.main_power = main_power  # フェーズ1(エンコーダ主導・全力寄り)の駆動パワー
        self.pid_p = pid_p
        self.pid_i = pid_i
        self.pid_d = pid_d
        # 2026-09-12: フェーズ2(ジャイロの仕上げ補正)がその場旋回の位置ズレの
        # 原因になっていないか切り分けるための一時的な診断用パラメータ。
        # フェーズ2をほぼ発動させないよう緩めた状態(例:8度)で同じ旋回テストを
        # やり直し、後方への位置ズレ(24〜28cm)が縮むかどうかを見る。
        # 縮めばフェーズ2側(左右非対称な旧SpinAround方式のまま)が原因、
        # 縮まなければ別の要因(キャスター等)を疑う。切り分けが終わったら
        # 2.0に戻すこと。
        self.fine_tolerance_deg = fine_tolerance_deg
        # fine_trim=False: フェーズ1(エンコーダで回して停止)だけで終える。ジャイロでの仕上げ(行き過ぎた
        # ときに低出力で細かく動かして戻す動き)をしない。ボトルを先端で運んでいる区間で、
        # 小刻みな動きでボトルが離れないようにするためのスイッチ。
        self.fine_trim = fine_trim
        self.fine_clamper = SymmetricClamper(fine_min_power, fine_max_power)  # フェーズ2(仕上げ)専用
        # フェーズ1終了直後はブレーキで完全停止しているため、fine_min_power程度の
        # 弱い力では静止摩擦に勝てず、まったく動かないまま(=角度誤差が2度未満に
        # ならないまま)ノードが永久にRUNNINGを返し続けてスタックする恐れがある
        # (2026-09-12、実機でこの症状を確認)。一定時間ジャイロの角度が進まなければ、
        # フェーズ1のmain_powerに近い、動くことが分かっているパワーまで引き上げる
        # (エスカレーション)ための予備クランパー。
        self.escalated_clamper = SymmetricClamper(max(int(main_power * 0.8), fine_min_power), main_power)
        self.running = False

    def update(self) -> Status:
        runtime.require("plotter", "gyro_sensor", "right_motor", "left_motor")
        # ジャイロの倍率補正(GYRO_SCALE_FACTOR)は、読み取りの窓口(gyro_scale.ScaledGyro)で適用済み。
        # 単独で使われる場合(補正がまだ有効でない場合)も、ここで有効にして必ず補正済みの値を読む。
        if not self.running:
            ensure_scaled_gyro()
        current_heading = (-1) * runtime.course * runtime.gyro_sensor.get_angle()
        if not self.running:
            if self.target_type == HeadingType.RELATIVE:
                self.target_heading = current_heading + self.target
            else:
                self.target_heading = self.target
            error = (float(self.target_heading) - current_heading + 180.0) % 360.0 - 180.0

            # 目標角度(度)を、左右の車輪が地面の上で進むべき円弧の長さ(mm)に
            # 変換し、さらにエンコーダの回転角度(度)に変換する。
            arc_length_mm = math.radians(abs(error)) * (WHEEL_TREAD / 2.0)
            # 直進(Plotter.plot())と同じタイヤ径を使う。固定値を別途持つと、
            # --rally-gyro-tire-calibration offでの較正前/較正後の切替が旋回側に反映されない。
            self.target_motor_deg = arc_length_mm / (math.pi * runtime.plotter.tire_diameter) * 360.0
            self.direction = 1 if error > 0 else -1

            global _last_turn_direction
            if (self.backlash_preload_deg > 0.0 and _last_turn_direction != 0
                    and self.direction != _last_turn_direction):
                self.target_motor_deg += self.backlash_preload_deg
            _last_turn_direction = self.direction

            # DD-22較正用: right_power=course*direction*main_powerなので、この符号が
            # マイナスの側が先に目標へ到達して止まる(実測、2026-10-10)。先に止まる側は、
            # もう片方がまだ回っている間、反力で同じ方向へ引き込まれて動いてしまう分
            # (lag_compensation_deg/_LAG_REFERENCE_MOTOR_DEG、適応モードなら
            # _lag_estimate_ratio)を見越して、目標値を低くしておく。
            # 2026-10-10: 固定の度数ではなく、その旋回のtarget_motor_degに対する
            # 比率として持つことで、較正に使った90度旋回より小さい旋回でも
            # 自動的に適切な大きさに縮む(以前は0にクランプされる不具合があった)。
            global _lag_estimate_ratio
            compensation_ratio = (_lag_estimate_ratio if self.adaptive_lag_compensation
                                   else self.lag_compensation_deg / _LAG_REFERENCE_MOTOR_DEG)
            self.right_base_motor_deg = self.target_motor_deg
            self.left_base_motor_deg = self.target_motor_deg * self.left_target_scale
            right_power_sign = runtime.course * self.direction
            self.early_side = 'right' if right_power_sign < 0 else 'left'
            # 比率を使っても、万一1.0を超えるような値になった場合の保険として
            # 先に止まる側の目標値の半分までしか引かないようにする。
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
            self.right_done = (self.right_target_motor_deg < 1e-6)
            self.left_done = (self.left_target_motor_deg < 1e-6)
            # target_motor_deg≈0(回転不要)なら、上のright_done/left_doneがTrueのまま
            # 下の個別停止判定を一度も通らないため、ここでも基準値を用意しておく。
            self.right_done_at_count = self.start_r
            self.left_done_at_count = self.start_l
            self.phase = 1
            self.oscillation = OscillationTracker()

            self.running = True
            self.logger.info("%+06d %s.encoder-spin started at heading=%d for %d (target_motor_deg=%.1f)" % (
                runtime.plotter.get_distance(), self.__class__.__name__, current_heading, self.target_heading, self.target_motor_deg))

        if self.phase == 1:
            delta_r = abs(runtime.right_motor.get_count() - self.start_r)
            delta_l = abs(runtime.left_motor.get_count() - self.start_l)

            # 目標のエンコーダ角度に達した方から個別に止める
            # (左右のタイヤの実効径がわずかに違っても、片方だけ行き過ぎさせない)。
            # 目標値は、lag_compensation_deg適用時は先に止まる側だけ低くなっている。
            if not self.right_done and delta_r >= self.right_target_motor_deg:
                self.right_done = True
                # ユーザー仮説(2026-10-10)の検証用: 止めた瞬間のエンコーダ値を記録し、
                # もう片方がまだ動いている「遅れ」区間に、この止めたはずの輪が反力で
                # 動いていないか(片輪テストのbrake_side_deltaと同じ考え方)を後で見る。
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
                    observed = abs(lag_right_delta if self.early_side == 'right' else lag_left_delta)
                    early_base = (self.right_base_motor_deg if self.early_side == 'right'
                                   else self.left_base_motor_deg)
                    # early_baseが小さい旋回は、ほぼ一定の引き込まれ量をごく小さい値で割る
                    # ことになり比率が発散するので無視する。発散しなかった場合も念のため
                    # 比率そのものに上限をかけ、1サンプルで推定値を暴走させない。
                    if early_base > _LAG_ESTIMATE_MIN_EARLY_BASE_DEG:
                        observed_ratio = min(observed / early_base, _LAG_ESTIMATE_MAX_RATIO)
                        _lag_estimate_ratio = (_LAG_ESTIMATE_ALPHA * observed_ratio
                                               + (1.0 - _LAG_ESTIMATE_ALPHA) * _lag_estimate_ratio)
                self.logger.info(
                    "%+06d %s.encoder-spin phase1 lag check: lag_right_delta=%+d lag_left_delta=%+d "
                    "lag_estimate_ratio=%.3f" % (
                        runtime.plotter.get_distance(), self.__class__.__name__,
                        lag_right_delta, lag_left_delta, _lag_estimate_ratio))
                if not self.fine_trim:
                    # 仕上げなし: 停止したまま終了する。残った誤差はログにだけ残す。
                    residual = (float(self.target_heading) - current_heading + 180.0) % 360.0 - 180.0
                    self.logger.info("%+06d %s.encoder-spin ended without fine-trim at heading=%d, residual=%+.1f fwd_mm=%+.1f loc=(%d,%d)" % (
                        runtime.plotter.get_distance(), self.__class__.__name__, current_heading, residual,
                        forward_mm_since(self.start_r, self.start_l),
                        runtime.plotter.get_loc_x(), runtime.plotter.get_loc_y()))
                    return Status.SUCCESS
                self.phase = 2
                self.pid = PID(self.pid_p, self.pid_i, self.pid_d, setpoint=self.target_heading, sample_time=EXEC_INTERVAL)
                self.stall_ticks = 0
                self.escalated = False
                self.prev_heading_for_stall = current_heading
                # 2026-10-25較正用: フェーズ2が最初に行き過ぎて向きを反転させる直前の
                # 誤差を、「フェーズ2の1回目の精度」(一発パルス仕上げとの公平な比較用)
                # として一度だけログに残す。
                self.first_reversal_logged = False
                self.logger.info("%+06d %s.encoder-spin phase1 complete at heading=%d, entering gyro fine-trim" % (
                    runtime.plotter.get_distance(), self.__class__.__name__, current_heading))
                return Status.RUNNING

            use_decel = self.decel_deg > 0 and self.target_motor_deg >= 2 * self.decel_deg
            right_power = left_power = 0
            if not self.right_done:
                p = self.decel_power if use_decel and self.right_target_motor_deg - delta_r < self.decel_deg else self.main_power
                right_power = runtime.course * self.direction * p
                runtime.right_motor.set_power(right_power)
            if not self.left_done:
                p = self.decel_power if use_decel and self.left_target_motor_deg - delta_l < self.decel_deg else self.main_power
                left_power = (-1) * runtime.course * self.direction * p
                runtime.left_motor.set_power(left_power)
            log_power_state(self.__class__.__name__ + ".phase1", right_power, left_power)
            return Status.RUNNING

        # --- フェーズ2: ジャイロによる低パワーの仕上げ補正 ---
        error = (float(self.target_heading) - current_heading + 180.0) % 360.0 - 180.0
        if abs(error) < self.fine_tolerance_deg:
            # 2026-09-14: 判定が通った瞬間、モーターを明示的に止めずにSUCCESSを
            # 返していたため、fine_min_power(50前後)で回っていた勢いのまま
            # 判定後も惰性で回り続け、毎回ほぼ一定量(28x90度/14x180度テストで
            # 1回あたり平均0.57〜0.59度、旋回の回数にのみ比例し角度サイズには
            # 比例しない)オーバーしていたと判明。許容範囲を2.0→0.5度に狭めても
            # 改善しなかったのは、判定直前のパワーがfine_min_powerの下限に
            # 張り付いたまま変わらず、慣性の量もほぼ変わらなかったため
            # (許容範囲の広さでは効かない種類の誤差だった)。ここで明示的に
            # ブレーキをかけて惰性そのものを断つ。
            runtime.right_motor.set_power(0)
            runtime.right_motor.set_brake(True)
            runtime.left_motor.set_power(0)
            runtime.left_motor.set_brake(True)
            self.logger.info("%+06d %s.encoder-spin ended at heading=%d residual=%+.1f reversals=%d samples=%d fwd_mm=%+.1f loc=(%d,%d)" % (
                runtime.plotter.get_distance(), self.__class__.__name__, current_heading, error,
                self.oscillation.reversal_count, self.oscillation.sample_count,
                forward_mm_since(self.start_r, self.start_l),
                runtime.plotter.get_loc_x(), runtime.plotter.get_loc_y()))
            return Status.SUCCESS

        # スタック検知: ジャイロの角度がしばらく進んでいなければ、静止摩擦に
        # 負けて動けていないと判断し、動くことが分かっているパワーまで
        # 引き上げる(一度上げたら、このノードが終わるまで下げない)。
        heading_progress = abs((current_heading - self.prev_heading_for_stall + 180.0) % 360.0 - 180.0)
        if heading_progress < _STALL_PROGRESS_DEG:
            self.stall_ticks += 1
            if self.stall_ticks > _STALL_TICK_LIMIT:  # 約0.2秒進捗なし(2026-09-12: 元は15tick=0.45秒、体感の停止時間を減らすため短縮)
                self.escalated = True
                self.logger.info("%+06d %s.stalled in fine-trim, escalating power" % (
                    runtime.plotter.get_distance(), self.__class__.__name__))
        else:
            self.stall_ticks = 0
        self.prev_heading_for_stall = current_heading

        self.pid.setpoint = current_heading + error
        clamper = self.escalated_clamper if self.escalated else self.fine_clamper
        power = int(clamper.clamp(self.pid(current_heading)))
        right_power = runtime.course * power
        left_power = (-1) * runtime.course * power
        runtime.right_motor.set_power(right_power)
        runtime.left_motor.set_power(left_power)
        log_power_state(self.__class__.__name__ + ".phase2", right_power, left_power)
        self.oscillation.update(power)
        if not self.first_reversal_logged and self.oscillation.reversal_count >= 1:
            self.first_reversal_logged = True
            self.logger.info(
                "%+06d %s.phase2 first-pass error=%+.2f loc=(%d,%d)" % (
                    runtime.plotter.get_distance(), self.__class__.__name__, error,
                    runtime.plotter.get_loc_x(), runtime.plotter.get_loc_y()))
        return Status.RUNNING


    def terminate(self, new_status: Status) -> None:
        # 中断・完了時にモーター出力を残さず、次の実行(再入)に備えて状態を戻す。
        if runtime.right_motor is not None:
            runtime.right_motor.set_power(0)
        if runtime.left_motor is not None:
            runtime.left_motor.set_power(0)
        self.running = False


class EtRallyRunByGyro(Behaviour):
    """床の線を見ず、ジャイロセンサーの角度だけを頼りに指定の方角へ向かって真っ直ぐ直進するノード"""
    def __init__(self, name: str, target: int, power: int,
                 pid_p: float, pid_i: float, pid_d: float, target_type: HeadingType,
                 distance_mm: float = 0.0, decel_mm: float = 0.0, decel_min_power: int = 40,
                 accel_mm: float = 0.0, accel_min_power: int = 40,
                 turn_forward_compensation: bool = False, reverse: bool = False) -> None:
        super(EtRallyRunByGyro, self).__init__(name)
        self.target = target
        self.target_type = target_type
        self.power = power  # 直進の前進パワー(巡航)
        self.pid_p = pid_p
        self.pid_i = pid_i
        self.pid_d = pid_d
        # 2026-09-20: 巡航パワーを上げて速く走り、止まる手前で減速する。
        # distance_mm(この区間の走行距離)とdecel_mm(減速を始める手前の距離)が
        # 両方指定されたときだけ、残り距離がdecel_mm(短い区間では距離の6割)を
        # 切ったところから、残り距離に比例してパワーをdecel_min_powerまで
        # 直線的に落とす。ゲート通過中など、止まらずに次の区間へ続くmoveでは
        # decel_mm=0にして巡航のまま走らせる。
        self.distance_mm = distance_mm
        self.decel_mm = min(decel_mm, distance_mm * 0.6) if distance_mm > 0 else 0.0
        self.decel_mm_setting = decel_mm
        self.decel_min_power = min(decel_min_power, power)
        # 2026-09(会話にもとづく): 開始直後の急発進を避けるための、減速と対になる
        # 加速区間。区間全体をゆっくり加減速させるのではなく、あくまで開始直後・
        # 終了直前の「短い」区間だけで済ませる想定(accel_mm/decel_mmを小さく保つ)。
        # accel_mm=0なら従来通り、開始直後から巡航パワーで走る(無効化)。
        self.accel_mm = min(accel_mm, distance_mm * 0.6) if distance_mm > 0 else 0.0
        self.accel_mm_setting = accel_mm
        self.accel_min_power = min(accel_min_power, power)
        # 2026-10-29: Trueなら、直前の旋回で前へ出た量(take_turn_forward_mm)だけ短く走り、
        # 自分で距離に達したらSUCCESSを返す(IsDistanceEarnedは元の距離のままなので、
        # 短くした分はこのノードで終える)。Falseなら従来どおり(終了はIsDistanceEarned任せ)。
        self.turn_forward_compensation = turn_forward_compensation
        self.base_distance_mm = distance_mm
        # 2026-10-29: Trueなら後退する(スタート直後の直進確認用)。向きの補正(turn)は
        # その場で回る成分なので、進む向きに関係なく同じ符号のまま使える。
        # plotterの距離は絶対値で積算されるため、距離の判定もそのまま使える。
        self.reverse = reverse
        self.last_log_time = None
        self.running = False

    def update(self) -> Status:
        runtime.require("plotter", "gyro_sensor", "right_motor", "left_motor")
        # 2026-09-19: SpinAroundByEncoder(GYRO_SCALE_FACTOR適用済み)は旋回終了時点で
        # 真の向きをtarget_heading_degに正しく一致させているが、直後にRunByGyroが
        # 同じraw角度を無補正で読むと、raw角度はtargetよりGYRO_SCALE_FACTOR分
        # (約0.635%)少なく見える。RunByGyroのPIDはこれを実際の不足と誤認して
        # ロボットをさらに回してしまい、結果として真の向きがtarget_heading_degを
        # そのぶんオーバーシュートした状態で直進が続く(target_heading_degの符号・
        # 大きさに比例するズレなので、区間ごとに右にも左にもズレて見える一因になり
        # うる)。2026-09-14時点では適用すると本番経路でむしろ悪化したとのことだが、
        # 当時は他の較正(キャスター引きずり補正・タイヤ径較正)とも変更が重なって
        # いた可能性があるため、これ単体を切り分けて再検証する
        # (2026-09-19、et_rally_planner側の解析に基づき再度有効化)。
        if not self.running:
            ensure_scaled_gyro()
        current_heading = (-1) * runtime.course * runtime.gyro_sensor.get_angle()
        # 1秒ごとに現在の方角をログに吐き出してデバッグしやすくする
        if self.last_log_time == None or time.time() - self.last_log_time >= 1.0:
            self.logger.info("%+06d %s.current heading=%d" % (runtime.plotter.get_distance(), self.__class__.__name__, current_heading))
            self.last_log_time = time.time()
        if not self.running:
            if self.target_type == HeadingType.RELATIVE:
                self.target_heading = current_heading + self.target
            else:
                self.target_heading = self.target
            if self.turn_forward_compensation:
                turn_forward_mm = take_turn_forward_mm() or 0.0
                self.distance_mm = max(self.base_distance_mm - turn_forward_mm, 1.0)
                self.decel_mm = min(self.decel_mm_setting, self.distance_mm * 0.6)
                self.accel_mm = min(self.accel_mm_setting, self.distance_mm * 0.6)
                self.logger.info("%+06d %s.turn forward compensation: turn_fwd_mm=%+.1f distance %.1f -> %.1f" % (
                    runtime.plotter.get_distance(), self.__class__.__name__, turn_forward_mm,
                    self.base_distance_mm, self.distance_mm))
            # 算出した旋回量が直進パワーを超えて暴走しないよう、output_limitsでしっかりキャップをかける
            self.pid = PID(self.pid_p, self.pid_i, self.pid_d, setpoint=self.target_heading, sample_time=EXEC_INTERVAL, output_limits=(-self.power, self.power))
            self.logger.info("%+06d %s.gyro run started toward heading=%d target_dist=%d loc=(%d,%d)" % (
                runtime.plotter.get_distance(), self.__class__.__name__, self.target_heading, self.distance_mm,
                runtime.plotter.get_loc_x(), runtime.plotter.get_loc_y()))
            self.orig_dist = runtime.plotter.get_distance()
            # 2026-09-20: PID比較用に、区間ごとの向きの誤差と、その誤差による横ズレの
            # 推定値(進んだ距離 x sin(向き - 目標))を集計し、区間の最後に1行で出す。
            self.prev_dist_lat = self.orig_dist
            # 2026-10-28: 直進終了時点の速度(直近約0.1秒の平均)を診断ログに出すための履歴。
            self.speed_history = []
            self.lat_mm = 0.0
            self.err_sum = 0.0
            self.err_n = 0
            self.err_max = 0.0
            self.summarized = False
            self.running = True

        # 加速区間(開始直後)・減速区間(終了直前)、それぞれで現在のパワーの上限を計算し、
        # 小さいほう(より制約が強いほう)を採用する。短い区間で両方の窓が重なっても、
        # 単純により低いパワー側を使うだけで、破綻なく扱える。
        power_now = self.power
        if self.accel_mm > 0.0:
            traveled = runtime.plotter.get_distance() - self.orig_dist
            if traveled < self.accel_mm:
                ratio = max(traveled, 0.0) / self.accel_mm
                power_now = min(power_now, int(self.accel_min_power + (self.power - self.accel_min_power) * ratio))
        if self.decel_mm > 0.0:
            remaining = self.distance_mm - (runtime.plotter.get_distance() - self.orig_dist)
            if remaining < self.decel_mm:
                ratio = max(remaining, 0.0) / self.decel_mm
                power_now = min(power_now, int(self.decel_min_power + (self.power - self.decel_min_power) * ratio))

        # SpinAroundと同種の不具合(PID内部はsetpointとcurrent_headingの単純な引き算で、
        # 180度をまたぐ場合の短い方への正規化を行わない)を避けるため、ここでも
        # 短い方に正規化した誤差を計算し、それに一致するようsetpointを毎ティック補正する。
        # current_headingはジャイロの積算値で周回を重ねると360度を大きく超えることが
        # あるため、剰余演算(%)で一発で-180〜180度に正規化する
        # (1回きりのif文による補正では、360度以上ズレている場合に対応しきれない)。
        error = (float(self.target_heading) - current_heading + 180.0) % 360.0 - 180.0
        self.pid.setpoint = current_heading + error

        now_dist = runtime.plotter.get_distance()
        self.speed_history.append((time.monotonic(), now_dist))
        if len(self.speed_history) > 6:
            self.speed_history.pop(0)
        self.lat_mm += (now_dist - self.prev_dist_lat) * math.sin(math.radians(-error))
        self.prev_dist_lat = now_dist
        self.err_sum += error
        self.err_n += 1
        self.err_max = max(self.err_max, abs(error))
        if (not self.summarized and self.distance_mm > 0.0
                and now_dist - self.orig_dist >= self.distance_mm):
            self.summarized = True
            actual_dist = now_dist - self.orig_dist
            (t0, d0), (t1, d1) = self.speed_history[0], self.speed_history[-1]
            end_speed = (d1 - d0) / (t1 - t0) if t1 - t0 > 1e-6 else 0.0
            self.logger.info("%+06d %s.run summary: dist=%d target_dist=%d dist_err=%+d err_mean=%+.2f err_max=%.2f lat=%+.0fmm (+=目標より反時計回り側へ) end_speed=%.0fmm/s loc=(%d,%d)" % (
                now_dist, self.__class__.__name__, actual_dist, self.distance_mm, actual_dist - self.distance_mm,
                self.err_sum / max(self.err_n, 1), self.err_max, self.lat_mm, end_speed,
                runtime.plotter.get_loc_x(), runtime.plotter.get_loc_y()))

        turn = int(self.pid(current_heading))  # まっすぐ走るために必要な微修正の旋回量
        # 減速中は基準パワーが下がるので、旋回量も現在のパワー以内に収める
        # (PID自体の上限は巡航パワーで作ってあるため)。
        turn = max(-power_now, min(power_now, turn))
        base_power = -power_now if self.reverse else power_now
        right_power = base_power + runtime.course * turn
        left_power = base_power - runtime.course * turn
        runtime.right_motor.set_power(right_power)
        runtime.left_motor.set_power(left_power)
        log_power_state(self.__class__.__name__, right_power, left_power)
        if self.turn_forward_compensation and self.summarized:
            return Status.SUCCESS
        return Status.RUNNING

