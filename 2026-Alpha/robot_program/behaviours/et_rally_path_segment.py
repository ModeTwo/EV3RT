"""ETラリー専用: 直進・円弧を、前進/後退・止まる/止まらずに続ける、の組み合わせで走るノード。

2026-11-01: 往復・後退でのゲート通過と、つなぎ区間の円弧化を試すために追加した
(計画JSONの move/arc に reverse・stop の指定があるときだけ execute_strategy が使う。
指定のない従来の計画は、これまでどおり EtRallyRunByGyro で走る)。

- 向きは進んだ距離に合わせて目標を動かし、EtRallyRunByGyroと同じくジャイロのPIDで追う。
  円弧では、半径から決まる左右の差(フィードフォワード)を足してから、PIDでずれだけを直す。
- 円弧では、外側の車輪が巡航の出力を超えないよう、中心の出力を 1/(1+半輪距/半径) に落とす。
- 止まらずに続く区間(stop=False)では、次の区間の距離を「この区間の計画上の終わり」から数える
  (区間の切り替わりの判定の刻み・行き過ぎが、区間ごとにたまらないように)。PIDも次の区間へ引き継ぐ。
- 減速は、止まるまでに続く区間全体の残り距離(remaining_after_mm を足したもの)で決める。
- stop=True の区間の終わりでは、ブレーキをかけて左右のエンコーダが止まるまで待ってから終える
  (直後に前後が入れ替わっても、惰性のまま逆へ出力をかけないため)。
- 2026-11-03: 計画の線からの横のずれを、目標の向きに戻す(lat_gain、既定0=従来どおり向きだけを追う)。
  向きだけを追うと、円弧の入り口・出口で向きが遅れた分の横のずれが戻らずに残り、実機で円弧の終わりに
  7〜15mm外れていた(11/03のarc8・arc_entryのテスト)。
  区間の始めの計画上の点を原点・進む向きをxとした座標で、自分の位置を(進んだ距離, ジャイロの向き)から積算し、
  計画の線(直線/円弧)の一番近い点からの横のずれを出す(向きのずれ×距離の積算だと、円弧が続くと見積もりが
  狂い、机上で8の字が逆に悪くなった)。止まらずに続く区間へは、次の区間の始めの点から見た位置を引き継ぐ。
  後退でも「進む向き」で考えれば同じ式になる(進む向きは向きと同じだけ回る)。
- 2026-11-03: 走りを滑らかにするオプション(どれも既定は従来どおり)。実機のログで、円弧に入ると外側の車輪が100近く、
  内側が0〜マイナス(逆回し)まで振れ、約0.5秒周期で2回ほど上下していた(run_20261103T061719)。
  計画上は直進→円弧・右回り→左回りの所で左右の速さが一瞬で変わるが、車輪が追いつけずに向きが遅れ、
  向きのPIDが強く取り返して行き過ぎるため。
  - arc_pid_p: 円弧の中だけ使う向きのPIDのP(Noneなら直進と同じ)。
  - ramp_mm: 目標の向き・左右の差・出力の倍率を、この距離の幅で前後に平均する(曲がり具合が一瞬で変わらず、
    この距離をかけて直線的に変わる)。区間の境目をまたぐので、前の区間の形も引き継ぐ。
    平均した線は計画の線から少し内側を通る(半径100mm・幅60mmで約1mm)。
  - ramp_sec: 平均の幅を時間でも決める(幅 = max(ramp_mm, ramp_sec × 今の速さ))。車輪が追いつくまでの時間は
    速さによらずほぼ同じなので、速度を上げても同じ滑らかさにするため。0なら使わない(従来どおり)。
    使うときは、今の速さの見積もりを区間をまたいで引き継ぐ(区間の始めで0に戻ると幅が一瞬縮むため)。
  - wheel_log_ticks: 区間の始めからこのtick数だけ、毎tickの出力と実際の車輪の回り(エンコーダの度/tick)・向きのずれを
    ためて、区間の終わりに1行で出す(0なら出さない=従来どおり)。円弧の入り口で内側の車輪が実際にいつ遅くなるかを見るため。
  - ff_scale: 円弧の左右の差(フィードフォワード)の倍率(1.0なら従来どおり 半輪距/半径)。
  - line_pid_p / line_pid_d: 直進の区間だけ使う向きのPIDのP・D(Noneなら pid_p / pid_d。2026-11-11、試す用)。
  - plan_start: 計画のこの区間の始めの、コース上の姿勢 (x, y, コースの向き)(et_rally_start_pose.annotate_plan_course)。
    これがあり、前から続かない区間(止まったあとの走り出し)なら、コース上の今の位置(et_rally_course_pose、カメラで直したもの)との差を
    自分の位置の初期値にし、前後の差は走る距離に入れる(2026-11-11: 止まるたびに「計画どおりの点にいる」とみなして数え直していたため、
    切り返しの多い配置37で、カメラで分かったずれが引き継がれず、1周ごとに約1cmずつたまって6周目に青ゲートに当たった)。
    差が CARRY_MAX_MM を超えるときは使わない(位置の取り違えのおそれ)。
  - line_boost_power / line_boost_sec: 直進の始めの line_boost_sec 秒だけ、出力を line_boost_power まで上げる
    (円弧のあと 150〜270mm/s から巡航の約420mm/s まで約0.3秒かかり、直進1本あたり約0.1秒遅れていた。2026-11-11、試す用)。
    長さ BOOST_MIN_LEN_MM 以上の前進の直進で、円弧のあと・止まった状態からのときだけ。終わりの BOOST_END_MARGIN_MM と、
    次の円弧への先読みで出力が下がり始めたら上げない。左右の差(曲がる分)は、これまでどおり出力を削って残す。
    11/03の車輪のログで、円弧の中の左右の実際の回りの比から逆算した輪距が約146〜158mm(実測の117mmより大きい。
    すべり・こすれのため)で、計算どおりの差では曲がり足りず、残りを向きのPIDが常に埋めていた
    (円弧の中の向きのずれの平均が約3°、入り口の遅れ・出力の急な変化の元)。
  - inner_min_power: 前進のとき、左右どちらの出力もこれより下げない(0なら逆回しをしない)。
    左右の差が足りない分は、全体の出力を下げて差を残す(曲がる量を優先)。
"""

import math
import time

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from simple_pid import PID

from py_etrobo_util.plotter import WHEEL_TREAD

from ..gyro_scale import ensure_scaled_gyro
from ..et_rally_heading import et_pose_text
from ..power_log import log_power_state
from ..runtime import runtime
from ..timing import CONTROL_INTERVAL_SEC as EXEC_INTERVAL
from . import et_rally_drive

HALF_TREAD_MM = WHEEL_TREAD / 2.0
# 2026-11-12: 前進の区間で、向きを直すために前進の出力を削るときの下限(出力約45のその場旋回は摩擦で止まるため)。
MIN_FORWARD_POWER = 30.0
# 止まるとき: 左右のエンコーダがこのtick数だけ続けて変わらなければ止まったとみなす(one_shotのpre_stopと同じ)。
STOP_STABLE_TICKS = 2
STOP_MAX_TICKS = 25
# 2026-11-11: 直進の始めの加速の上乗せをする直進の長さの下限と、終わりの手前で上乗せをやめる距離(mm)。
BOOST_MIN_LEN_MM = 150.0
# 2026-11-11: 止まったあとの走り出しで引き継ぐ、計画とのずれの上限(mm)。これを超えたら引き継がない。
CARRY_MAX_MM = 80.0
BOOST_END_MARGIN_MM = 100.0

# 止まらずに続く区間の受け渡し(次の区間の距離の起点と、引き継ぐPID)。
_chain_end_dist = None
# 前の区間の形 (kind, 長さ, 始めの向き, 向きの変化, 半径)。ramp_mmで境目の手前を平均するのに使う。
_chain_shape = None
# 前の区間の終わりの速さの見積もり(mm/s)。ramp_secを使うときだけ引き継ぐ。
_chain_speed = None
_chain_pid = None
# 次の区間の始め(計画上)から見た自分の位置 (前, 右) mm と、位置を積算した最後の距離・向き
# (次の区間が動き出すのは1tick後なので、その間の進みを次の区間で積算するため)。
_chain_lat = None


# 2026-11-05: 円弧・直進の区間の直進の向きのPIDのI(--rally-move-pid-i を付けないとき)。
# 11/05 実機で、I=8 では608mmの直進で出力が40↔100を約0.5秒周期で振れ(頭打ち43回)、I=2 で消えた。
PATH_LINE_PID_I = 2.0


def _line_pid_i(base_i):
    override = et_rally_drive._move_pid_i_override
    if override is not None:
        return override
    return PATH_LINE_PID_I if et_rally_drive._fine_power_enabled else base_i


# 2026-11-06: スタートで測った位置のずれ (前, 右) mm(et_rally_start_pose.py)。最初の区間(前から続かない区間)の
# 自分の位置の初期値にして、横のずれを戻すしくみで走りながら直す。使ったら None に戻す。
_start_position = None


def set_start_position(front_mm, right_mm):
    global _start_position
    _start_position = (float(front_mm), float(right_mm))


# 2026-11-10: 走行中のカメラの補正(et_rally_camera_correct.py)用。いま走っている区間。
_active = None


def active_kind():
    """いま走っている区間の種類("line" / "arc")。走っていなければ None。"""
    seg = _active
    if seg is None or not getattr(seg, "running", False):
        return None
    return seg.kind


def shift_active_position(dx_course, dy_course):
    """いま走っている区間の自分の位置を、コースの座標で (dx, dy) mm だけ動かす。走っていなければ何もしない。
    区間の座標: 始めの計画上の点が原点、x = 始めの進む向き、y = その右(計画の向き = スタートの向き - コースの向き)。"""
    seg = _active
    if seg is None or not getattr(seg, "running", False):
        return False
    from ..et_rally_start_pose import START_HEADING_DEG
    th = math.radians(START_HEADING_DEG - seg.start_heading + (180.0 if seg.reverse else 0.0))
    along = dx_course * math.cos(th) + dy_course * math.sin(th)
    seg.pos_x += along
    seg.pos_y += dx_course * math.sin(th) - dy_course * math.cos(th)
    # 2026-11-10: 進む向きの成分は、区間の終わりの距離にも入れる(実際は前にいる → その分早く終える)。
    # 区間の終わりは走った距離で決めているので、位置を動かすだけでは前後のずれが止まる位置に効かなかった
    # (実機: 補正で前に約15mmいると分かっていたのに、ゴールで前に12.5mm)。続く区間は終わりの距離から起点を
    # 引き継ぐので、そのまま次の区間にも効く。走った距離は後退でも増え、後退の区間の座標も進む向きにとってあるので、
    # 前進・後退とも同じ向きにずらす。
    seg.orig_dist -= along
    return True


def reset_chain():
    global _chain_end_dist, _chain_pid, _chain_lat, _chain_shape, _chain_speed
    _chain_shape = None
    _chain_speed = None
    _chain_end_dist = None
    _chain_pid = None
    _chain_lat = None


def _heading():
    return (-1) * runtime.course * runtime.gyro_sensor.get_angle()


class EtRallyPathSegment(Behaviour):
    def __init__(self, name: str, kind: str, end_heading_deg: float, length_mm: float, power: int,
                 pid_p: float, pid_i: float, pid_d: float,
                 sweep_deg: float = 0.0, radius_mm: float = 0.0, reverse: bool = False,
                 stop: bool = True, continue_from_prev: bool = False, remaining_after_mm: float = 0.0,
                 stop_lead_mm: float = 0.0, accel_mm: float = 0.0, accel_min_power: int = 40,
                 decel_mm: float = 0.0, decel_min_power: int = 40,
                 next_profile=(), lead_sec: float = 0.0, arc_pid_i: float = 0.6,
                 arc_speed_scale: float = 1.0, lat_gain: float = 0.0, lat_max_deg: float = 8.0,
                 arc_pid_p=None, ramp_mm: float = 0.0, inner_min_power=None, ramp_sec: float = 0.0,
                 wheel_log_ticks: int = 0, ff_scale: float = 1.0,
                 line_pid_p=None, line_pid_d=None, line_boost_power=None, line_boost_sec: float = 0.0,
                 plan_start=None, stop_stable_ticks=None, small_arc=None) -> None:
        super().__init__(name)
        if kind not in ("line", "arc"):
            raise ValueError("kind must be line or arc")
        self.kind = kind
        self.end_heading = float(end_heading_deg)
        self.sweep = float(sweep_deg) if kind == "arc" else 0.0
        self.start_heading = self.end_heading - self.sweep
        self.radius_mm = float(radius_mm)
        self.length_mm = float(length_mm)
        self.power = power
        self.pid_gains = (pid_p, pid_i, pid_d)
        self.reverse = reverse
        self.stop_at_end = stop
        self.continue_from_prev = continue_from_prev
        self.remaining_after_mm = remaining_after_mm
        # 止まる区間だけ、惰性の分を手前で終える(続く区間は計画どおりの距離で次へ渡す)。
        self.run_length_mm = max(self.length_mm - stop_lead_mm, 1.0) if stop else self.length_mm
        self.accel_mm = 0.0 if continue_from_prev else accel_mm
        self.accel_min_power = min(accel_min_power, power)
        self.decel_mm = decel_mm  # 続く区間でも、止まるまでの全体の残りが短ければ減速する
        self.decel_min_power = min(decel_min_power, power)
        if kind == "arc" and self.radius_mm <= 0:
            raise ValueError("radius_mm must be positive")
        # 先読み: 止まるまでに続く区間の形(kind, 長さ, 始めの向き, 向きの変化, 半径)。
        # 車輪の速さが出力に追いつくまでの遅れの分(lead_sec × 今の速さ)だけ先の曲がり具合で
        # 左右の差を出し、曲がり始め・曲がり終わりを早める(遅れたまま曲がると、その分だけ経路がずれる)。
        self.profile = [(kind, self.length_mm, self.start_heading, self.sweep, self.radius_mm)] + list(next_profile)
        self.lead_sec = lead_sec
        # 2026-11-01: 円弧の中だけ向きのPIDのIを弱くする(直進用の強いI(--rally-move-pid-i 8など)のままだと、
        # 目標の向きが動き続ける円弧で出力が約0.5秒周期で揺れた。run_20261101T061925)。
        self.arc_pid_i = arc_pid_i
        # 2026-11-02: 円弧の出力の倍率。1.0なら外側の車輪が直進の出力を超えない(従来)。大きくすると円弧が速くなる
        # (左右の出力は ±100 に収める処理で頭打ちになる)。
        self.arc_speed_scale = arc_speed_scale
        # 2026-11-12: 半径 small_arc[0] mm 以下の円弧だけ、出力の倍率を small_arc[1] にする(Noneなら全部 arc_speed_scale)。
        # 半径80mmを出力100で走ると外側100・内側ほぼ0で、入りで遅れた向きを取り戻す余地がほとんどないため。
        self.small_arc = tuple(small_arc) if small_arc else None
        # 2026-11-03: 横のずれ1mmあたり、目標の向きを何度ずらして戻すか(0なら戻さない)と、その上限(度)。
        self.lat_gain = lat_gain
        self.lat_max_deg = lat_max_deg
        self.arc_pid_p = arc_pid_p
        self.ramp_mm = max(0.0, float(ramp_mm))
        self.ramp_sec = max(0.0, float(ramp_sec))
        self.ramp_width = self.ramp_mm
        self.wheel_log_ticks = max(0, int(wheel_log_ticks))
        self.ff_scale = float(ff_scale)
        self.inner_min_power = inner_min_power
        self.line_pid_p = line_pid_p
        self.line_pid_d = line_pid_d
        self.line_boost_power = line_boost_power
        self.line_boost_sec = max(0.0, float(line_boost_sec or 0.0))
        self.plan_start = plan_start
        self.stop_stable_ticks = STOP_STABLE_TICKS if stop_stable_ticks is None else max(1, int(stop_stable_ticks))
        self.prev_shape = None
        self.running = False
        self.stopping = False

    # --- 内部 ---
    def _arc_scale(self, radius):
        if self.small_arc is not None and radius <= self.small_arc[0]:
            return self.small_arc[1]
        return self.arc_speed_scale

    def _profile_at(self, s):
        """この区間の始めから距離sの点の(目標の向き, 左右差の比, 出力の倍率)。続く区間へはみ出してもよい。"""
        if s < 0.0 and self.prev_shape is not None:
            kind, length, h0, sweep, radius = self.prev_shape
            if kind == "arc" and radius > 0 and length > 0:
                ratio = max(0.0, (length + s) / length)
                return (h0 + sweep * ratio, math.copysign(HALF_TREAD_MM / radius, sweep),
                        self._arc_scale(radius) / (1.0 + HALF_TREAD_MM / radius))
            return h0 + sweep, 0.0, 1.0
        last = len(self.profile) - 1
        for i, (kind, length, h0, sweep, radius) in enumerate(self.profile):
            if s > length and i < last:
                s -= length
                continue
            if s > length:
                # 止まるまでに続く区間の、さらに先。最後が円弧なら、その曲がり具合のまま左右の差をかけ続ける
                # (先読みの分だけ手前で左右の差が0になり、止まって終わる円弧が最後の約2cmで曲がりきらず、
                # 止まった位置が約1cmずれていた。run_20261101T063540のarc2・arc15)。目標の向きは終わりの向き。
                if kind == "arc" and radius > 0:
                    return (h0 + sweep, math.copysign(HALF_TREAD_MM / radius, sweep),
                            self._arc_scale(radius) / (1.0 + HALF_TREAD_MM / radius))
                return h0 + sweep, 0.0, 1.0
            if kind == "arc" and radius > 0 and length > 0:
                ratio = max(0.0, s / length)
                return (h0 + sweep * ratio, math.copysign(HALF_TREAD_MM / radius, sweep),
                        self._arc_scale(radius) / (1.0 + HALF_TREAD_MM / radius))
            return h0 + sweep, 0.0, 1.0
        return self.end_heading, 0.0, 1.0

    def _smooth_at(self, s):
        """_profile_at を ramp_mm の幅で平均したもの。向きは s の点の向きからの差で平均する
        (区間ごとに向きの表し方が360°ずれていることがあるため)。"""
        if self.ramp_width <= 0.0:
            return self._profile_at(s)
        n = 9
        h_c, _, _ = self._profile_at(s)
        dh = ff = sc = 0.0
        for k in range(n):
            h, f, c = self._profile_at(s + self.ramp_width * (k / (n - 1) - 0.5))
            dh += (h - h_c + 180.0) % 360.0 - 180.0
            ff += f
            sc += c
        return h_c + dh / n, ff / n, sc / n

    def _lateral(self):
        """今の位置から、計画の線の一番近い点までの (その点までの線の長さu, 横のずれ(右が+)) mm。
        円弧では中心から見た角度を前のtickから連続になるようにつなぐ(180°を超える円弧でも進みが飛ばないように)。"""
        x, y = self.pos_x, self.pos_y
        if self.kind != "arc":
            return x, y
        r = self.radius_mm
        if self.sweep >= 0:  # 右へ曲がる: 中心は右 (0, r)
            a = math.atan2(x, r - y)
            lat = r - math.hypot(x, y - r)
        else:                # 左へ曲がる: 中心は左 (0, -r)
            a = math.atan2(x, y + r)
            lat = math.hypot(x, y + r) - r
        prev = getattr(self, "_arc_angle", None)
        if prev is not None:
            a += 2.0 * math.pi * round((prev - a) / (2.0 * math.pi))
        self._arc_angle = a
        return r * a, lat

    def _pos_from_end(self):
        """この区間の計画上の終わりの点・終わりの進む向きから見た、今の位置 (前, 右)。"""
        if self.kind == "arc":
            d = math.radians(self.sweep)
            r = self.radius_mm
            sg = 1.0 if self.sweep >= 0 else -1.0
            ex, ey = r * math.sin(abs(d)), sg * r * (1.0 - math.cos(d))
        else:
            d = 0.0
            ex, ey = self.length_mm, 0.0
        dx, dy = self.pos_x - ex, self.pos_y - ey
        return dx * math.cos(d) + dy * math.sin(d), -dx * math.sin(d) + dy * math.cos(d)

    def _target_heading(self, s):
        if self.kind == "line" or self.length_mm <= 0:
            return self.end_heading
        return self.start_heading + self.sweep * max(0.0, min(1.0, s / self.length_mm))

    def _start(self):
        global _chain_pid, _chain_lat, _chain_shape, _chain_speed, _start_position
        ensure_scaled_gyro()
        now = runtime.plotter.get_distance()
        if self.continue_from_prev and _chain_end_dist is not None:
            self.orig_dist = _chain_end_dist
        else:
            self.orig_dist = now
        if self.continue_from_prev and _chain_pid is not None:
            self.pid = _chain_pid
            self.pid.output_limits = (-self.power, self.power)
        else:
            pid_i = _line_pid_i(self.pid_gains[1])
            self.pid = PID(self.pid_gains[0], pid_i, self.pid_gains[2], setpoint=self._target_heading(0.0),
                           sample_time=EXEC_INTERVAL, output_limits=(-self.power, self.power))
        _chain_pid = None
        # 区間をまたいで引き継いだPIDでも、Iの係数はこの区間の種類に合わせる(たまっているIの値はそのまま)。
        line_i = _line_pid_i(self.pid_gains[1])
        self.pid.Ki = self.arc_pid_i if self.kind == "arc" else line_i
        self.pid.Kp = self.arc_pid_p if (self.kind == "arc" and self.arc_pid_p is not None) else self.pid_gains[0]
        if self.kind == "line" and self.line_pid_p is not None:
            self.pid.Kp = self.line_pid_p
        self.pid.Kd = self.line_pid_d if (self.kind == "line" and self.line_pid_d is not None) else self.pid_gains[2]
        self.prev_shape = _chain_shape if self.continue_from_prev else None
        _chain_shape = None
        # 直進の始めの加速の上乗せ(円弧のあと・止まった状態からの前進の直進だけ。直進から続く直進は既に巡航の速さ)。
        self.boost_until = None
        self.boost_ticks = 0
        if (self.line_boost_power is not None and self.line_boost_sec > 0.0 and self.kind == "line"
                and not self.reverse and self.length_mm >= BOOST_MIN_LEN_MM
                and (self.prev_shape is None or self.prev_shape[0] == "arc")):
            self.boost_until = time.monotonic() + self.line_boost_sec
        self.saturated_ticks = 0
        self.forward_floor_ticks = 0
        self.right_residual = 0.0
        self.left_residual = 0.0
        self.err_sum = 0.0
        self.err_n = 0
        self.err_max = 0.0
        # 自分の位置(この区間の始めの計画上の点が原点、x=始めの進む向き、y=その右)。
        # 止まらずに続く区間では、前の区間から引き継ぐ(計画の線は区間をまたいでつながっているため)。
        self.pos_prev_dist = now
        self.prev_heading = _heading()
        carried = None
        if self.continue_from_prev and _chain_lat is not None:
            self.pos_x, self.pos_y, self.pos_prev_dist, self.prev_heading = _chain_lat
        elif _start_position is not None:
            self.pos_x, self.pos_y = _start_position
            self.logger.info("start position from camera: front %+.1f right %+.1f mm" % _start_position)
        elif self.plan_start is not None and not self.continue_from_prev:
            carried = self._carry_from_course()
            self.pos_x, self.pos_y = carried if carried is not None else (0.0, 0.0)
        else:
            self.pos_x, self.pos_y = 0.0, 0.0
        if carried is not None and self.kind == "line":
            # 前後の差: 計画より front mm 先にいるなら、その分だけ短く走る(走った距離の起点を前にずらす)。
            # 2026-11-11: 円弧ではしない。円弧で走った距離を進めると、目標の向きもその分だけ先に回り、止まった状態から
            # いきなり向きを合わせにいってその場旋回のようになっていた(配置37の青の後の円弧: 前に13〜23mm → 目標の向きが
            # 9〜16°先、左右の出力 -44/+51 や ±100 で約1.7倍遅く、ガクッとした)。円弧では位置のずれとしてだけ持つ。
            self.orig_dist = now - carried[0]
        _start_position = None
        _chain_lat = None
        self._arc_angle = None
        self.lat_mm = self._lateral()[1]
        self.wheel_log = []
        if self.wheel_log_ticks:
            self.wheel_prev = (runtime.right_motor.get_count(), runtime.left_motor.get_count())
        self.prev_dist = now
        self.speed_mm_s = 0.0
        if self.ramp_sec > 0.0 and self.continue_from_prev and _chain_speed is not None:
            self.speed_mm_s = _chain_speed
        _chain_speed = None
        self.start_heading_actual = _heading()
        self.running = True
        global _active
        _active = self
        self.logger.info("%+06d %s.%s started %s heading %.1f->%.1f length=%d radius=%d stop=%s cont=%s rest=%d loc=(%d,%d)%s" % (
            now, self.__class__.__name__, self.kind, "reverse" if self.reverse else "forward",
            self.start_heading, self.end_heading, self.length_mm, self.radius_mm, self.stop_at_end,
            self.continue_from_prev, self.remaining_after_mm,
            runtime.plotter.get_loc_x(), runtime.plotter.get_loc_y(), et_pose_text()))

    def _carry_from_course(self):
        """コース上の今の位置と計画の始めの位置の差 (前, 右) mm。使えなければ None(理由はログ)。"""
        from .. import et_rally_course_pose as cp
        from .. import et_rally_start_pose as sp
        cur = cp.current()
        if cur is None:
            return None
        front, right = sp.plan_offset_in_segment(self.plan_start, self.reverse, (cur[1], cur[2]))
        if math.hypot(front, right) > CARRY_MAX_MM:
            self.logger.warning("carry error: offset from plan (front %+.1f right %+.1f mm) too large, not used" % (front, right))
            return None
        self.logger.info("carry error: start from course pose, offset from plan front %+.1f right %+.1f mm" % (front, right))
        return front, right

    def _finish_log(self, now, tag=""):
        current = _heading()
        # 2026-11-05: 区間の終わりで、自分の位置(ログ上)が計画の終わりの点から (前, 右) に何mmずれているか。
        # 区間は走った距離で区切るため、前後のずれは直していない。それがどれだけたまるかを測る。
        # 止まる区間は、止め始めてからの惰性の分(位置を積算していない)を前に足す。
        end_front, end_right = self._pos_from_end()
        if self.stopping:
            end_front += now - getattr(self, "reached_dist", now)
        # along: 自分の位置で測った進み − 走った距離での進み(前後のずれのたまり。1tickの越えを含まない)。
        tag = " end_err=(%+.1f,%+.1f) along=%+.1f%s" % (end_front, end_right, getattr(self, "along_mm", 0.0), tag)
        if self.boost_until is not None:
            tag = " boost=%d" % self.boost_ticks + tag
        if self.forward_floor_ticks:
            tag = " floor=%d" % self.forward_floor_ticks + tag
        self.logger.info("%+06d %s.%s summary: dist=%d planned=%d end_heading=%.1f target=%.1f err_mean=%+.2f err_max=%.2f lat=%+.0fmm sat=%d loc=(%d,%d)%s%s" % (
            now, self.__class__.__name__, self.kind, now - self.orig_dist, self.run_length_mm,
            current, self.end_heading, self.err_sum / max(self.err_n, 1), self.err_max, self.lat_mm,
            self.saturated_ticks,
            runtime.plotter.get_loc_x(), runtime.plotter.get_loc_y(), et_pose_text(), tag))
        if self.wheel_log:
            # 1tickごとに 指示した出力R/L:そのtickに実際に回った度R/L:向きのずれ:横のずれmm(11/12追加)
            self.logger.info("WHEELS %s: %s" % (self.kind, " ".join(
                "%d/%d:%d/%d:%+.1f:%+.1f" % w for w in self.wheel_log)))

    def update(self) -> Status:
        global _chain_end_dist, _chain_pid, _chain_lat, _chain_shape, _chain_speed
        runtime.require("plotter", "gyro_sensor", "right_motor", "left_motor")
        if not self.running:
            self._start()

        if self.stopping:
            counts = (runtime.right_motor.get_count(), runtime.left_motor.get_count())
            self.stop_ticks += 1
            self.stop_stable = self.stop_stable + 1 if counts == self.stop_prev else 0
            self.stop_prev = counts
            if self.stop_stable < self.stop_stable_ticks and self.stop_ticks < STOP_MAX_TICKS:
                return Status.RUNNING
            self._finish_log(runtime.plotter.get_distance(), " stopped ticks=%d%s" % (
                self.stop_ticks, "" if self.stop_stable >= self.stop_stable_ticks else " (timeout)"))
            return Status.SUCCESS

        now = runtime.plotter.get_distance()
        s = now - self.orig_dist
        current = _heading()
        # 自分の位置を積算する(進む向きの、区間の始めの進む向きからの回り = 向きの回り。右が+)。
        # 区間の終わりの判定より前に積算する(最後のtickの進みを落とすと、次の区間へ引き継ぐ位置が約7mmずれる)。
        # 1tickの間の向きは、前のtickと今の向きの平均を使う(円弧では1tickで約4°回る)。
        turned = math.radians((current + self.prev_heading) / 2.0 - self.start_heading)
        self.prev_heading = current
        step = now - self.pos_prev_dist
        self.pos_prev_dist = now
        self.pos_x += step * math.cos(turned)
        self.pos_y += step * math.sin(turned)
        u, self.lat_mm = self._lateral()
        # 2026-11-05: 区間の終わりを位置で決める案を机上シミュで試したが、走った距離で区切っても前後のずれはたまらず
        # (4周で along 平均-0.3mm・範囲-2.8〜+1.6mm)、効果がなかったため入れていない。実機で同じかを along で見る。
        self.along_mm = u - s
        if s >= self.run_length_mm:
            if self.stop_at_end:
                _chain_end_dist = None
                _chain_pid = None
                _chain_lat = None
                _chain_shape = None
                runtime.right_motor.set_power(0)
                runtime.right_motor.set_brake(True)
                runtime.left_motor.set_power(0)
                runtime.left_motor.set_brake(True)
                self.stopping = True
                self.stop_ticks = 0
                self.stop_stable = 0
                self.stop_prev = (runtime.right_motor.get_count(), runtime.left_motor.get_count())
                self.reached_dist = now
                return Status.RUNNING
            # 続く区間: 次の区間の距離の起点を、計画上の終わりにする。モーターは止めない。
            _chain_end_dist = self.orig_dist + self.length_mm
            _chain_pid = self.pid
            _chain_lat = self._pos_from_end() + (self.pos_prev_dist, self.prev_heading)
            _chain_shape = self.profile[0]
            _chain_speed = self.speed_mm_s
            self._finish_log(now)
            return Status.SUCCESS

        self.speed_mm_s += ((now - self.prev_dist) / EXEC_INTERVAL - self.speed_mm_s) * 0.3
        if self.ramp_sec > 0.0:
            self.ramp_width = max(self.ramp_mm, self.ramp_sec * max(self.speed_mm_s, 0.0))
        # 目標の向きは今の位置のもの。左右の差(フィードフォワード)と出力の倍率だけを先読みする
        # (車輪の速さが出力に追いつくまでの遅れを打ち消す。目標の向きまで先読みすると曲がり始めが早すぎる)。
        target, _, here_scale = self._smooth_at(s)
        _, ff_ratio, look_scale = self._smooth_at(s + self.lead_sec * max(self.speed_mm_s, 0.0))
        error = (target - current + 180.0) % 360.0 - 180.0
        # 横のずれを戻す: 右にずれていれば目標の向きを左へ(進む向きを線へ向ける)。後退でも同じ向き。
        correction = 0.0
        if self.lat_gain > 0.0:
            correction = max(-self.lat_max_deg, min(self.lat_max_deg, self.lat_gain * self.lat_mm))
        self.pid.setpoint = current + error - correction
        self.prev_dist = now
        self.err_sum += error
        self.err_n += 1
        self.err_max = max(self.err_max, abs(error))

        # 出力: 巡航 → 円弧なら外側が巡航を超えない値 → 発進の加速・止まる前の減速の小さいほう
        power_now = self.power * min(look_scale, here_scale)
        if (self.boost_until is not None and min(look_scale, here_scale) >= 0.999
                and s < self.length_mm - BOOST_END_MARGIN_MM and time.monotonic() < self.boost_until):
            if self.line_boost_power > power_now:
                power_now = float(self.line_boost_power)
                self.boost_ticks += 1
        if self.accel_mm > 0.0 and s < self.accel_mm:
            ratio = max(s, 0.0) / self.accel_mm
            power_now = min(power_now, self.accel_min_power + (self.power - self.accel_min_power) * ratio)
        if self.decel_mm > 0.0:
            remaining = (self.run_length_mm - s) + self.remaining_after_mm
            if remaining < self.decel_mm:
                ratio = max(remaining, 0.0) / self.decel_mm
                power_now = min(power_now, self.decel_min_power + (self.power - self.decel_min_power) * ratio)

        output = self.pid(current)
        ff = abs(power_now) * ff_ratio * self.ff_scale
        turn = max(-power_now, min(power_now, output)) + ff
        base = -power_now if self.reverse else power_now
        # 2026-11-01: 左右の出力を±100に収める。左右の差(曲がる分)を優先して残し、はみ出す分は全体の出力を下げる
        # (円弧の入りで右+131・左-29のように上限を超え、超えた分が削られて曲がり方が計画と変わっていた)。
        if abs(turn) > 100.0:
            turn = math.copysign(100.0, turn)
        headroom = 100.0 - abs(turn)
        if abs(base) > headroom:
            base = math.copysign(headroom, base)
            self.saturated_ticks += 1
        if self.inner_min_power is not None and not self.reverse and base > 0.0:
            # 内側の車輪(base - |turn|)を inner_min_power より下げない。まず左右の差を base - 下限 までに抑える。
            # ただし計画の曲がり(ff)の分は残し、足りなければ全体を上げずに差を優先する(外側は100まで)。
            limit = max(base - self.inner_min_power, abs(ff))
            if abs(turn) > limit:
                turn = math.copysign(limit, turn)
        if not self.reverse and power_now > 0.0:
            # 2026-11-12: 前進の出力の下限。向きのずれが大きいと、上の「はみ出す分は前進を下げる」で前進が0まで
            # 削られたあとに左右の差だけ戻され、出力約45のその場旋回になって摩擦で約1秒止まった
            # (run_20261112T062851 の arc29、半径80の円弧で向きのずれ22°、右-38・左+44で車輪がほぼ回らず)。
            # 前進を MIN_FORWARD_POWER(止まる前の減速などで巡航がそれより低ければその値)より下げない。
            # ずれが小さいとき(前進がこれ以上残るとき)の出力は前と同じ。
            floor = min(MIN_FORWARD_POWER, power_now)
            if base < floor:
                base = floor
                if abs(turn) > 100.0 - base:
                    turn = math.copysign(100.0 - base, turn)
                self.forward_floor_ticks += 1
        self.right_residual += base + runtime.course * turn
        self.left_residual += base - runtime.course * turn
        right_power = int(round(self.right_residual))
        left_power = int(round(self.left_residual))
        self.right_residual -= right_power
        self.left_residual -= left_power
        runtime.right_motor.set_power(right_power)
        runtime.left_motor.set_power(left_power)
        log_power_state(self.__class__.__name__ + "." + self.kind, right_power, left_power)
        if len(self.wheel_log) < self.wheel_log_ticks:
            counts = (runtime.right_motor.get_count(), runtime.left_motor.get_count())
            self.wheel_log.append((right_power, left_power, counts[0] - self.wheel_prev[0],
                                   counts[1] - self.wheel_prev[1], error, self.lat_mm))
            self.wheel_prev = counts
        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        # 続く区間ではモーターを止めない(次の区間がすぐ出力を上書きする)。中断(INVALID)のときだけ止める。
        if new_status == Status.INVALID:
            if runtime.right_motor is not None:
                runtime.right_motor.set_power(0)
            if runtime.left_motor is not None:
                runtime.left_motor.set_power(0)
            reset_chain()
        self.running = False
        self.stopping = False
