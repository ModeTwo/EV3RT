"""検証専用: 片方の車輪だけをエンコーダで回し、もう片方はブレーキで固定して、
1周(既定360度)ごとのジャイロ方位の変化量をログに残す。

目的: 「エンコーダの測定自体は正確」であることと、「左右で同じエンコーダ回転量が
同じだけ実際の向きを変える」こととは別の話ではないか、という疑問の直接検証。
右輪だけでlaps回・左輪だけでlaps回を同じ条件(出力・周回数)で実行し、1周あたりの
実際の方位変化量(ジャイロ測定)を左右で比較する。差があれば、タイヤの実効径・
グリップ・バックラッシュの左右差が実在することの直接証拠になる。

本番では使わない。EtRallySpinAroundByEncoder等、既存クラスは変更しない。
"""

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence

from ..gyro_scale import ensure_scaled_gyro
from ..runtime import runtime

_DEFAULT_POWER = 50
_DEFAULT_LAPS = 10
_DEFAULT_DEG_PER_LAP = 360.0


def build_et_rally_single_wheel_diagnostic_sequence(
        power: int = _DEFAULT_POWER, laps: int = _DEFAULT_LAPS,
        deg_per_lap: float = _DEFAULT_DEG_PER_LAP) -> Sequence:
    # 右輪だけ→左輪だけ、の順でlaps回ずつ実行し、1周ごとの実際の方位変化量を
    # ログに残す(本ファイルのdocstring参照)。
    root = Sequence(name="et_rally single wheel diagnostic", memory=True)
    root.add_children([
        EtRallySingleWheelEncoderTest(
            name="single_wheel_test right", wheel="right", power=power,
            deg_per_lap=deg_per_lap, laps=laps),
        EtRallySingleWheelEncoderTest(
            name="single_wheel_test left", wheel="left", power=power,
            deg_per_lap=deg_per_lap, laps=laps),
    ])
    return root


def build_et_rally_single_wheel_reversal_diagnostic_sequence(
        power: int = _DEFAULT_POWER, laps_forward: int = 5, laps_reverse: int = 5,
        deg_per_lap: float = _DEFAULT_DEG_PER_LAP) -> Sequence:
    # 各輪ごとに、順方向にlaps_forward周→その場で逆方向にlaps_reverse周、を連続させる。
    # 逆方向側の1周目(反転した直後)が、2周目以降(定常)と比べて小さければ、
    # 反転の瞬間だけ余計にエンコーダが消費されている(バックラッシュ等)ことの
    # 直接証拠になる。右輪・左輪で同じ比較をすることで、反転コストの左右差も見える。
    root = Sequence(name="et_rally single wheel reversal diagnostic", memory=True)
    children = []
    for wheel in ("right", "left"):
        children.append(EtRallySingleWheelEncoderTest(
            name="single_wheel_reversal_test %s forward" % wheel, wheel=wheel, power=power,
            deg_per_lap=deg_per_lap, laps=laps_forward))
        children.append(EtRallySingleWheelEncoderTest(
            name="single_wheel_reversal_test %s reverse" % wheel, wheel=wheel, power=-power,
            deg_per_lap=deg_per_lap, laps=laps_reverse))
    root.add_children(children)
    return root


def _build_single_wheel_alternating_sequence(
        wheel: str, power: int, segments: int, deg_per_lap: float) -> Sequence:
    # 1周(laps=1)ずつ、符号を毎回反転させてsegments回繰り返す。最初の1回を除き、
    # すべての区間が「反転直後の1周」になるため、反転コストのサンプル数を最大化できる。
    root = Sequence(name="et_rally single wheel alternating diagnostic (%s)" % wheel, memory=True)
    children = []
    for i in range(segments):
        segment_power = power if i % 2 == 0 else -power
        children.append(EtRallySingleWheelEncoderTest(
            name="single_wheel_alt_test %s seg%d" % (wheel, i), wheel=wheel,
            power=segment_power, deg_per_lap=deg_per_lap, laps=1))
    root.add_children(children)
    return root


def build_et_rally_single_wheel_alternating_diagnostic_sequence(
        power: int = _DEFAULT_POWER, segments: int = 20,
        deg_per_lap: float = _DEFAULT_DEG_PER_LAP) -> Sequence:
    # 右輪→左輪の順で、両方まとめて実行する版(途中で物理的な位置ズレを測る間を
    # 空けたい場合は、下のright/left単独版を別々のミッションとして使う)。
    root = Sequence(name="et_rally single wheel alternating diagnostic", memory=True)
    root.add_children([
        _build_single_wheel_alternating_sequence("right", power, segments, deg_per_lap),
        _build_single_wheel_alternating_sequence("left", power, segments, deg_per_lap),
    ])
    return root


# DD-22較正用: 1回の反転試験あたりのsegments既定値を、plan_spin_alternating30.json
# (30回の旋回、ほぼ毎回方向反転)の旋回回数に合わせる。右輪だけ・左輪だけを別々の
# ミッションとして実行できるようにし、2つの間に実機の位置ズレを物理的に測る間を
# 空けられるようにする(2026-10-10、ユーザー要望)。
_ALTERNATING_SEGMENTS_MATCHING_TURN_COUNT = 30


def build_et_rally_single_wheel_alternating_right_diagnostic_sequence(
        power: int = _DEFAULT_POWER, segments: int = _ALTERNATING_SEGMENTS_MATCHING_TURN_COUNT,
        deg_per_lap: float = _DEFAULT_DEG_PER_LAP) -> Sequence:
    return _build_single_wheel_alternating_sequence("right", power, segments, deg_per_lap)


def build_et_rally_single_wheel_alternating_left_diagnostic_sequence(
        power: int = _DEFAULT_POWER, segments: int = _ALTERNATING_SEGMENTS_MATCHING_TURN_COUNT,
        deg_per_lap: float = _DEFAULT_DEG_PER_LAP) -> Sequence:
    return _build_single_wheel_alternating_sequence("left", power, segments, deg_per_lap)


class EtRallySingleWheelEncoderTest(Behaviour):
    def __init__(self, name: str, wheel: str, power: int,
                 deg_per_lap: float = 360.0, laps: int = 10) -> None:
        super().__init__(name)
        if wheel not in ("right", "left"):
            raise ValueError("wheel must be 'right' or 'left'")
        self.wheel = wheel
        self.power = power
        self.deg_per_lap = deg_per_lap
        self.laps = laps
        self.running = False

    def update(self) -> Status:
        runtime.require("gyro_sensor", "right_motor", "left_motor")
        drive_motor = runtime.right_motor if self.wheel == "right" else runtime.left_motor
        brake_motor = runtime.left_motor if self.wheel == "right" else runtime.right_motor

        if not self.running:
            ensure_scaled_gyro()
            brake_motor.set_power(0)
            brake_motor.set_brake(True)
            self.start_count = drive_motor.get_count()
            # 固定しているはずの側が、反力やブレーキのバックラッシュで実際に動いていないか
            # (ユーザー観察、2026-10-10)を直接確認するため、こちらもエンコーダを記録する。
            self.brake_start_count = brake_motor.get_count()
            self.lap_index = 0
            self.start_heading = (-1) * runtime.course * runtime.gyro_sensor.get_angle()
            self.prev_lap_heading = self.start_heading
            self.running = True
            self.logger.info(
                "%s started wheel=%s power=%d deg_per_lap=%.1f laps=%d heading0=%+.2f" % (
                    self.__class__.__name__, self.wheel, self.power,
                    self.deg_per_lap, self.laps, self.start_heading))

        current_heading = (-1) * runtime.course * runtime.gyro_sensor.get_angle()
        delta_count = abs(drive_motor.get_count() - self.start_count)
        target_count = self.deg_per_lap * (self.lap_index + 1)

        if delta_count >= target_count:
            lap_delta_heading = (current_heading - self.prev_lap_heading + 180.0) % 360.0 - 180.0
            total_delta_heading = (current_heading - self.start_heading + 180.0) % 360.0 - 180.0
            brake_delta_count = brake_motor.get_count() - self.brake_start_count
            self.logger.info(
                "%s wheel=%s lap=%d/%d encoder_total=%.1f heading=%+.2f "
                "lap_delta_heading=%+.2f total_delta_heading=%+.2f brake_side_delta=%+d" % (
                    self.__class__.__name__, self.wheel, self.lap_index + 1, self.laps,
                    delta_count, current_heading, lap_delta_heading, total_delta_heading,
                    brake_delta_count))
            self.prev_lap_heading = current_heading
            self.lap_index += 1
            if self.lap_index >= self.laps:
                drive_motor.set_power(0)
                drive_motor.set_brake(True)
                self.running = False
                return Status.SUCCESS

        drive_motor.set_power(self.power)
        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        if runtime.right_motor is not None:
            runtime.right_motor.set_power(0)
        if runtime.left_motor is not None:
            runtime.left_motor.set_power(0)
        self.running = False
