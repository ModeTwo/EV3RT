"""ETラリー専用の走行ビヘイビア。

sample_comment.py(ETラリー用にカスタムしたスタンドアロン版)のSpinAroundByEncoder/
RunByGyroを、robot_programのruntime経由に移植したもの。共通のgyro_drive.pyの
SpinAround/RunByGyroとは別クラスで、既存クラスの挙動は変えない。
ジャイロ値にGYRO_SCALE_FACTORを掛けて「真の角度の推定値」として使う点、旋回を
エンコーダ主導(フェーズ1)+ジャイロ仕上げ(フェーズ2)で行う点が共通版との違い。
"""

import math
import time

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from simple_pid import PID

from py_etrobo_util import SymmetricClamper
from py_etrobo_util.plotter import ET_RALLY_TIRE_DIAMETER, GYRO_SCALE_FACTOR, WHEEL_TREAD

from ..runtime import runtime
from ..timing import CONTROL_INTERVAL_SEC as EXEC_INTERVAL
from ..types import HeadingType

# ETラリー工程の間はPlotter.tire_diameterもこの値になる(phases/et_rally.py)。
TIRE_DIAMETER = ET_RALLY_TIRE_DIAMETER

# sample_comment.pyは制御周期0.03秒で較正した。robot_programの周期(0.02秒)でも
# 同じ「時間」で判定するため、tick数・1tickあたりの角度で持つ閾値を周期比で換算する。
_TUNED_INTERVAL_SEC = 0.03
_STALL_TICK_LIMIT = math.ceil(7 * _TUNED_INTERVAL_SEC / EXEC_INTERVAL - 1e-9)  # 0.03s x 7 = 約0.2秒
_STALL_PROGRESS_DEG = 0.3 * EXEC_INTERVAL / _TUNED_INTERVAL_SEC  # 1tickあたり0.3度=10度/秒未満


class EtRallySpinAroundByEncoder(Behaviour):
    """左右のタイヤの回転量(エンコーダ)を揃えて回すことで、「その場旋回」であること
    そのものを幾何学的に保証する旋回ノード。SpinAround(上記)の代替。

    背景(2026-09-12): SpinAroundはジャイロの角度だけを見て、目標角度に届くまで
    左右対称のパワーを与え続ける方式。最終的な向きの精度はジャイロがそのまま
    保証するが、左右のタイヤの実効径やグリップにわずかでも差があると、実際の
    回転中心が車軸の中心からズレてしまい、「その場」旋回のはずが車体全体が
    平行移動する問題があった。et_rally_plannerでの検証(±90度の旋回のみを
    30回繰り返すテスト)で、旋回だけで最大約40cm・再現性の高い(2回の試行で
    ほぼ同じ量・同じ向き)位置ズレが見つかっている。ランダムな滑りにしては
    再現性が高すぎるため、左右のタイヤの機械的な非対称性(回転中心のオフセット)
    が主因と推定した。

    このノードでは、目標の回転角度を「左右の車輪が地面の上で進むべき円弧の
    長さ(符号は逆・大きさは同じ)」に変換し(WHEEL_TREAD/TIRE_DIAMETER、
    py_etrobo_util/plotter.py参照)、実際に左右のエンコーダがその量だけ回転
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
                 decel_deg: float = 0.0, decel_power: int = 50) -> None:
        super(EtRallySpinAroundByEncoder, self).__init__(name)
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
        # 2026-09-14: ジャイロの生値は実際の回転量を約0.6%過少に報告していると
        # 判明した(py_etrobo_util/plotter.pyのGYRO_SCALE_FACTOR参照)ため、
        # 全ての箇所でここを掛けた値を「真の角度の推定値」として使う。
        current_heading = (-1) * runtime.course * runtime.gyro_sensor.get_angle() * GYRO_SCALE_FACTOR
        if not self.running:
            if self.target_type == HeadingType.RELATIVE:
                self.target_heading = current_heading + self.target
            else:
                self.target_heading = self.target
            error = (float(self.target_heading) - current_heading + 180.0) % 360.0 - 180.0

            # 目標角度(度)を、左右の車輪が地面の上で進むべき円弧の長さ(mm)に
            # 変換し、さらにエンコーダの回転角度(度)に変換する。
            arc_length_mm = math.radians(abs(error)) * (WHEEL_TREAD / 2.0)
            self.target_motor_deg = arc_length_mm / (math.pi * TIRE_DIAMETER) * 360.0
            self.direction = 1 if error > 0 else -1

            self.start_r = runtime.right_motor.get_count()
            self.start_l = runtime.left_motor.get_count()
            self.right_done = (self.target_motor_deg < 1e-6)
            self.left_done = (self.target_motor_deg < 1e-6)
            self.phase = 1

            self.running = True
            self.logger.info("%+06d %s.encoder-spin started at heading=%d for %d (target_motor_deg=%.1f)" % (
                runtime.plotter.get_distance(), self.__class__.__name__, current_heading, self.target_heading, self.target_motor_deg))

        if self.phase == 1:
            delta_r = abs(runtime.right_motor.get_count() - self.start_r)
            delta_l = abs(runtime.left_motor.get_count() - self.start_l)

            # 目標のエンコーダ角度に達した方から個別に止める
            # (左右のタイヤの実効径がわずかに違っても、片方だけ行き過ぎさせない)
            if not self.right_done and delta_r >= self.target_motor_deg:
                self.right_done = True
                runtime.right_motor.set_power(0)
                runtime.right_motor.set_brake(True)
            if not self.left_done and delta_l >= self.target_motor_deg:
                self.left_done = True
                runtime.left_motor.set_power(0)
                runtime.left_motor.set_brake(True)

            if self.right_done and self.left_done:
                self.phase = 2
                self.pid = PID(self.pid_p, self.pid_i, self.pid_d, setpoint=self.target_heading, sample_time=EXEC_INTERVAL)
                self.stall_ticks = 0
                self.escalated = False
                self.prev_heading_for_stall = current_heading
                self.logger.info("%+06d %s.encoder-spin phase1 complete at heading=%d, entering gyro fine-trim" % (
                    runtime.plotter.get_distance(), self.__class__.__name__, current_heading))
                return Status.RUNNING

            use_decel = self.decel_deg > 0 and self.target_motor_deg >= 2 * self.decel_deg
            if not self.right_done:
                p = self.decel_power if use_decel and self.target_motor_deg - delta_r < self.decel_deg else self.main_power
                runtime.right_motor.set_power(runtime.course * self.direction * p)
            if not self.left_done:
                p = self.decel_power if use_decel and self.target_motor_deg - delta_l < self.decel_deg else self.main_power
                runtime.left_motor.set_power((-1) * runtime.course * self.direction * p)
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
            self.logger.info("%+06d %s.encoder-spin ended at heading=%d" % (
                runtime.plotter.get_distance(), self.__class__.__name__, current_heading))
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
        runtime.right_motor.set_power(runtime.course * power)
        runtime.left_motor.set_power((-1) * runtime.course * power)
        return Status.RUNNING


class EtRallyRunByGyro(Behaviour):
    """床の線を見ず、ジャイロセンサーの角度だけを頼りに指定の方角へ向かって真っ直ぐ直進するノード"""
    def __init__(self, name: str, target: int, power: int,
                 pid_p: float, pid_i: float, pid_d: float, target_type: HeadingType,
                 distance_mm: float = 0.0, decel_mm: float = 0.0, decel_min_power: int = 40) -> None:
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
        self.decel_min_power = min(decel_min_power, power)
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
        current_heading = (-1) * runtime.course * runtime.gyro_sensor.get_angle() * GYRO_SCALE_FACTOR
        # 1秒ごとに現在の方角をログに吐き出してデバッグしやすくする
        if self.last_log_time == None or time.time() - self.last_log_time >= 1.0:
            self.logger.info("%+06d %s.current heading=%d" % (runtime.plotter.get_distance(), self.__class__.__name__, current_heading))
            self.last_log_time = time.time()
        if not self.running:
            if self.target_type == HeadingType.RELATIVE:
                self.target_heading = current_heading + self.target
            else:
                self.target_heading = self.target
            # 算出した旋回量が直進パワーを超えて暴走しないよう、output_limitsでしっかりキャップをかける
            self.pid = PID(self.pid_p, self.pid_i, self.pid_d, setpoint=self.target_heading, sample_time=EXEC_INTERVAL, output_limits=(-self.power, self.power))
            self.logger.info("%+06d %s.gyro run started toward heading=%d" % (runtime.plotter.get_distance(), self.__class__.__name__, self.target_heading))
            self.orig_dist = runtime.plotter.get_distance()
            # 2026-09-20: PID比較用に、区間ごとの向きの誤差と、その誤差による横ズレの
            # 推定値(進んだ距離 x sin(向き - 目標))を集計し、区間の最後に1行で出す。
            self.prev_dist_lat = self.orig_dist
            self.lat_mm = 0.0
            self.err_sum = 0.0
            self.err_n = 0
            self.err_max = 0.0
            self.summarized = False
            self.running = True

        # 減速区間では、残り距離に比例して現在のパワーを下げる。
        power_now = self.power
        if self.decel_mm > 0.0:
            remaining = self.distance_mm - (runtime.plotter.get_distance() - self.orig_dist)
            if remaining < self.decel_mm:
                ratio = max(remaining, 0.0) / self.decel_mm
                power_now = int(self.decel_min_power + (self.power - self.decel_min_power) * ratio)

        # SpinAroundと同種の不具合(PID内部はsetpointとcurrent_headingの単純な引き算で、
        # 180度をまたぐ場合の短い方への正規化を行わない)を避けるため、ここでも
        # 短い方に正規化した誤差を計算し、それに一致するようsetpointを毎ティック補正する。
        # current_headingはジャイロの積算値で周回を重ねると360度を大きく超えることが
        # あるため、剰余演算(%)で一発で-180〜180度に正規化する
        # (1回きりのif文による補正では、360度以上ズレている場合に対応しきれない)。
        error = (float(self.target_heading) - current_heading + 180.0) % 360.0 - 180.0
        self.pid.setpoint = current_heading + error

        now_dist = runtime.plotter.get_distance()
        self.lat_mm += (now_dist - self.prev_dist_lat) * math.sin(math.radians(-error))
        self.prev_dist_lat = now_dist
        self.err_sum += error
        self.err_n += 1
        self.err_max = max(self.err_max, abs(error))
        if (not self.summarized and self.distance_mm > 0.0
                and now_dist - self.orig_dist >= self.distance_mm):
            self.summarized = True
            self.logger.info("%+06d %s.run summary: dist=%d err_mean=%+.2f err_max=%.2f lat=%+.0fmm (+=目標より反時計回り側へ)" % (
                now_dist, self.__class__.__name__, now_dist - self.orig_dist,
                self.err_sum / max(self.err_n, 1), self.err_max, self.lat_mm))

        turn = int(self.pid(current_heading))  # まっすぐ走るために必要な微修正の旋回量
        # 減速中は基準パワーが下がるので、旋回量も現在のパワー以内に収める
        # (PID自体の上限は巡航パワーで作ってあるため)。
        turn = max(-power_now, min(power_now, turn))
        runtime.right_motor.set_power(power_now + runtime.course * turn)
        runtime.left_motor.set_power(power_now - runtime.course * turn)
        return Status.RUNNING

