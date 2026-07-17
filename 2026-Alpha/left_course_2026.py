"""ETロボコン2026 レフトコース用走行プログラム。

sample.py と同じく、各動作を小さな Behaviour として定義し、
build_behaviour_tree() で Sequence / Parallel / Selector を組み立てる。
"""

from __future__ import annotations

import argparse
import json
import signal
import socket
import time
from typing import Optional

from py_trees.behaviour import Behaviour
from py_trees.common import ParallelPolicy, Status
from py_trees.composites import Parallel, Selector, Sequence
from py_trees.trees import BehaviourTree
from simple_pid import PID

import sample as base
from py_etrobo_util import BottleColor, Color, ColorClassifier, Hint, HintType, TargetInterested, TraceSide


# 実機走行で調整する暫定値。
LAP_GATE_DISTANCE_MM = 1800
HINT1_DIRECTION_DEG = 0
HINT2_TRACE_TIMEOUT_SEC = 8.0
QR_RETRY_INTERVAL_SEC = 2.0
QR_RETRY_ANGLE_DEG = 4.0
BOTTLE_DELIVERY_DISTANCE_MM = 250
SUMO_BACK_TIME_SEC = 1.5
SUMO_LEFT_RUN_TIME_SEC = 1.5
GARAGE_WHITE_RUN_TIME_SEC = 2.0
WIRELESS_SEND_RETRY_COUNT = 3


g_race_started_at: Optional[float] = None
g_wireless = None
g_route_laps: list[list[tuple[float, int]]] = []
g_hint1_sent = False
g_hint2_sent = False
g_sumo_heading = 0.0


class WirelessStrategyDevice:
    """ヒント送信と最適経路受信を担当するJSON/UDP通信アダプター。"""

    def __init__(self, host: str, port: int, listen_port: int) -> None:
        # host: 経路計算を担当する無線通信デバイスのIPアドレス。
        # port: ヒント値の送信先UDPポート番号。
        # listen_port: 最適経路を受信するローカルUDPポート番号。
        self.destination = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", listen_port))
        self.sock.setblocking(False)

    def send_hint(self, hint_type: HintType, value: str) -> None:
        # hint_type: 送信するヒントカードの種類。
        # value: QRコードから取得または復号したヒント値。
        payload = {"type": hint_type.value, "value": value}
        encoded = json.dumps(payload).encode("utf-8")
        # UDPの単発欠落に備え、同じヒントを複数回送る。サーバー側は重複を同一値として扱う。
        for _ in range(WIRELESS_SEND_RETRY_COUNT):
            self.sock.sendto(encoded, self.destination)

    def receive_route_laps(self) -> Optional[list[list[tuple[float, int]]]]:
        """最大3周分の(絶対方位角, 距離mm)配列を受信する。"""
        try:
            data, _ = self.sock.recvfrom(65535)
        except BlockingIOError:
            return None
        message = json.loads(data.decode("utf-8"))
        if message.get("type") != "route":
            return None
        if "laps" in message:
            return [
                [(float(theta), int(length)) for theta, length in lap["segments"]]
                for lap in message["laps"]
            ]
        segments = [(float(theta), int(length)) for theta, length in message["segments"]]
        return [segments, list(segments), list(segments)]


class StartRaceTimer(Behaviour):
    """スタート条件成立直後の時刻を競技開始時刻として保存する。"""

    def __init__(self, name: str) -> None:
        # name: 行動木上で表示するノード名。
        super(StartRaceTimer, self).__init__(name)

    def update(self) -> Status:
        global g_race_started_at
        if g_race_started_at is None:
            g_race_started_at = time.monotonic()
            self.logger.info("race timer started")
        return Status.SUCCESS


class SetCameraTarget(Behaviour):
    """Webカメラの画像処理対象を切り替える。"""

    def __init__(self, name: str, target: TargetInterested) -> None:
        # name: 行動木上で表示するノード名。
        # target: LINE、QRCODE、BOTTLEのいずれかの認識対象。
        super(SetCameraTarget, self).__init__(name)
        self.target = target

    def update(self) -> Status:
        base.g_video.set_target_interested(self.target)
        return Status.SUCCESS


class AlwaysSuccess(Behaviour):
    """条件付きサブツリーを実行しない場合にSelectorを成功させる。"""

    def __init__(self, name: str) -> None:
        # name: 行動木上で表示するノード名。
        super(AlwaysSuccess, self).__init__(name)

    def update(self) -> Status:
        return Status.SUCCESS


class HasHint(Behaviour):
    """指定したヒントが既に取得済みかを判定する。"""

    def __init__(self, name: str, hint_type: HintType) -> None:
        # name: 行動木上で表示するノード名。
        # hint_type: 取得済みかを確認するヒントカードの種類。
        super(HasHint, self).__init__(name)
        self.hint_type = hint_type

    def update(self) -> Status:
        value = base.g_hint1 if self.hint_type is HintType.HINT1 else base.g_hint2
        return Status.SUCCESS if value is not None else Status.FAILURE


class DecodeAndSendHint(Behaviour):
    """期待するQRヒントを復号し、無線通信デバイスへ一度だけ送信する。"""

    def __init__(self, name: str, hint_type: HintType, complete_on_decode: bool) -> None:
        # name: 行動木上で表示するノード名。
        # hint_type: このノードが待ち受けるヒントカードの種類。
        # complete_on_decode: 読取り成功時にSUCCESSを返すか、並行走行終了までRUNNINGを維持するか。
        super(DecodeAndSendHint, self).__init__(name)
        self.hint_type = hint_type
        self.complete_on_decode = complete_on_decode

    def update(self) -> Status:
        global g_hint1_sent, g_hint2_sent
        stored = base.g_hint1 if self.hint_type is HintType.HINT1 else base.g_hint2
        if stored is None:
            text = base.g_video.get_QR_text()
            if text:
                try:
                    detected_type, value = Hint(text).resolve(password=base.g_key)
                except (ValueError, UnicodeError):
                    return Status.RUNNING
                if detected_type is self.hint_type:
                    if self.hint_type is HintType.HINT1:
                        base.g_hint1 = value
                        g_hint1_sent = False
                    else:
                        base.g_hint2 = value
                        g_hint2_sent = False
                    stored = value
        if stored is not None:
            already_sent = g_hint1_sent if self.hint_type is HintType.HINT1 else g_hint2_sent
            if not already_sent:
                g_wireless.send_hint(self.hint_type, stored)
                if self.hint_type is HintType.HINT1:
                    g_hint1_sent = True
                else:
                    g_hint2_sent = True
            return Status.SUCCESS if self.complete_on_decode else Status.RUNNING
        return Status.RUNNING


class IsColorLost(Behaviour):
    """指定色のラインを抜けたことを連続判定する。"""

    def __init__(self, name: str, color: Color, stable_count: int = 3) -> None:
        # name: 行動木上で表示するノード名。
        # color: 検出されなくなったことを確認する色。
        # stable_count: 色消失確定に必要な連続回数。
        super(IsColorLost, self).__init__(name)
        self.color = color
        self.stable_count = stable_count
        self.count = 0
        self.classifier = ColorClassifier()

    def update(self) -> Status:
        h, s, v = base.g_color_sensor.get_raw_color_hsv()
        detected = self.classifier.classify(h, s, v)
        self.count = self.count + 1 if detected is not self.color else 0
        return Status.SUCCESS if self.count >= self.stable_count else Status.RUNNING


class ReceiveStrategyRoute(Behaviour):
    """ヒント1・2から計算されたETラリー最適経路を受信する。"""

    def __init__(self, name: str) -> None:
        # name: 行動木上で表示するノード名。
        super(ReceiveStrategyRoute, self).__init__(name)

    def update(self) -> Status:
        global g_route_laps
        if not g_route_laps:
            g_route_laps = g_wireless.receive_route_laps() or []
        return Status.SUCCESS if g_route_laps else Status.RUNNING


class AlignAndResetGyroByRedGates(Behaviour):
    """周回開始前に左右ゲートの赤色を基準として正面を合わせ、ジャイロを初期化する。

    現行Video APIは左右2つの赤ゲートを個別に返さないため、赤色物体の方位を
    仮の中央方位として使用する。専用API追加後はこのノードだけを置換する。
    """

    def __init__(self, name: str, lap_index: int, timeout_sec: float = 5.0) -> None:
        # name: 行動木上で表示するノード名。
        # lap_index: 補正後に走行するETラリー周回番号（0始まり）。
        # timeout_sec: 赤ゲートを認識できない場合の待機上限。
        super(AlignAndResetGyroByRedGates, self).__init__(name)
        self.lap_index = lap_index
        self.timeout_sec = timeout_sec
        self.running = False
        self.started_at = 0.0
        self.stable = 0

    def update(self) -> Status:
        if self.lap_index >= len(g_route_laps):
            return Status.SUCCESS
        if not self.running:
            self.running = True
            self.started_at = time.monotonic()
            base.g_video.set_target_interested(TargetInterested.BOTTLE)
            base.g_video.set_bottle_color(BottleColor.RED)
        insight, color, _, theta, _, _, _ = base.g_video.get_bottle_stamped()
        if insight and color is BottleColor.RED:
            self.stable = self.stable + 1 if abs(theta) <= 2.0 else 0
            power = max(-25, min(25, int(theta * 0.8)))
            base.g_right_motor.set_power(power)
            base.g_left_motor.set_power(-power)
        if self.stable >= 3 or time.monotonic() - self.started_at >= self.timeout_sec:
            base.g_right_motor.set_power(0)
            base.g_left_motor.set_power(0)
            base.g_gyro_sensor.reset()
            base.g_video.set_target_interested(TargetInterested.LINE)
            return Status.SUCCESS
        return Status.RUNNING


class ExecuteStrategyLap(Behaviour):
    """1周分の(絶対方位角, 距離mm)戦略配列を順番に機械走行する。"""

    TURN, DRIVE = range(2)

    def __init__(self, name: str, lap_index: int, power: int = 30) -> None:
        # name: 行動木上で表示するノード名。
        # lap_index: 実行する経路配列の周回番号（0始まり）。
        # power: 距離走行時の基準モーター出力。
        super(ExecuteStrategyLap, self).__init__(name)
        self.lap_index = lap_index
        self.power = power
        self.segment_index = 0
        self.state = self.TURN
        self.origin = 0
        self.pid = PID(1.1, 0.1, 0.03, setpoint=0.0, sample_time=base.EXEC_INTERVAL,
                       output_limits=(-40, 40))

    @staticmethod
    def _angle_error(target: float, current: float) -> float:
        return (target - current + 180.0) % 360.0 - 180.0

    def update(self) -> Status:
        if self.lap_index >= len(g_route_laps):
            return Status.SUCCESS
        route = g_route_laps[self.lap_index]
        if self.segment_index >= len(route):
            base.g_right_motor.set_power(0)
            base.g_left_motor.set_power(0)
            return Status.SUCCESS
        heading, distance = route[self.segment_index]
        current = base.g_gyro_sensor.get_angle()
        error = self._angle_error(heading, current)
        if self.state == self.TURN:
            if abs(error) <= 2.0:
                base.g_right_motor.set_power(0)
                base.g_left_motor.set_power(0)
                self.origin = base.g_plotter.get_distance()
                self.state = self.DRIVE
            else:
                turn = max(18, min(45, int(abs(error) * 0.8)))
                turn = turn if error > 0 else -turn
                base.g_right_motor.set_power(-turn)
                base.g_left_motor.set_power(turn)
            return Status.RUNNING
        if abs(base.g_plotter.get_distance() - self.origin) >= distance:
            self.segment_index += 1
            self.state = self.TURN
            return Status.RUNNING
        turn = int(self.pid(-error))
        base.g_right_motor.set_power(max(-100, min(100, self.power - turn)))
        base.g_left_motor.set_power(max(-100, min(100, self.power + turn)))
        return Status.RUNNING


class SearchAndFaceSumoBottle(Behaviour):
    """アームを上げた状態で左右へ首を振り、相撲ボトルを正面へ捉える。"""

    def __init__(self, name: str, search_angle: float = 8.0) -> None:
        # name: 行動木上で表示するノード名。
        # search_angle: ボトル未検出時に左右へ振る相対角度。
        super(SearchAndFaceSumoBottle, self).__init__(name)
        self.search_angle = search_angle
        self.direction = 1

    def update(self) -> Status:
        global g_sumo_heading
        base.g_video.set_target_interested(TargetInterested.BOTTLE)
        insight, _, _, theta, _, _, _ = base.g_video.get_bottle_stamped()
        if insight:
            if abs(theta) <= 2.0:
                base.g_right_motor.set_power(0)
                base.g_left_motor.set_power(0)
                g_sumo_heading = base.g_gyro_sensor.get_angle()
                return Status.SUCCESS
            power = max(-25, min(25, int(theta)))
        else:
            power = 20 * self.direction
            self.direction *= -1
        base.g_right_motor.set_power(power)
        base.g_left_motor.set_power(-power)
        return Status.RUNNING


class IsBlackPassed(Behaviour):
    """黒線を一度検出した後、再び黒以外へ出たことを判定する。"""

    def __init__(self, name: str, stable_count: int = 3) -> None:
        # name: 行動木上で表示するノード名。
        # stable_count: 黒線通過確定に必要な黒以外の連続検出数。
        super(IsBlackPassed, self).__init__(name)
        self.stable_count = stable_count
        self.black_seen = False
        self.count = 0
        self.classifier = ColorClassifier()

    def update(self) -> Status:
        h, s, v = base.g_color_sensor.get_raw_color_hsv()
        color = self.classifier.classify(h, s, v)
        if color is Color.BLACK:
            self.black_seen = True
            self.count = 0
        elif self.black_seen:
            self.count += 1
        return Status.SUCCESS if self.count >= self.stable_count else Status.RUNNING


def build_behaviour_tree() -> BehaviourTree:
    """レフトコースの全走行戦略を、sample.py形式の行動木として組み立てる。"""
    root = Sequence(name="ETロボコン2026 レフトコース", memory=True)
    calibration = Sequence(name="キャリブレーション", memory=True)
    start = Sequence(name="スタート", memory=True)
    lap_gate = Parallel(name="LAPゲートまでライントレース", policy=ParallelPolicy.SuccessOnOne())
    bottle_catch = Sequence(name="ボトル色検出とキャッチ", memory=True)
    hint1 = Sequence(name="ヒントカード1取得", memory=True)
    hint1_run_and_read = Parallel(name="ヒント1方向へ走行しながらQR読取", policy=ParallelPolicy.SuccessOnOne())
    hint1_end_colors = Parallel(name="ヒント1走行終了色", policy=ParallelPolicy.SuccessOnOne())
    hint1_ensure = Selector(name="ヒント1未取得時の再探索", memory=True)
    hint1_scan = Parallel(name="ヒント1を2秒間隔で首振り探索", policy=ParallelPolicy.SuccessOnOne())
    hint1_scan_motion = Sequence(name="ヒント1用QR首振り動作", memory=True)
    hint2 = Sequence(name="ヒントカード2取得", memory=True)
    hint2_trace_and_read = Parallel(name="ライントレースしながらヒント2読取", policy=ParallelPolicy.SuccessOnOne())
    hint2_ensure = Selector(name="ヒント2未取得時の再探索", memory=True)
    hint2_scan = Parallel(name="ヒント2を2秒間隔で首振り探索", policy=ParallelPolicy.SuccessOnOne())
    hint2_scan_motion = Sequence(name="ヒント2用QR首振り動作", memory=True)
    bottle_delivery = Sequence(name="ボトルデリバリー", memory=True)
    trace_to_blue1 = Parallel(name="第1青線までライントレース", policy=ParallelPolicy.SuccessOnOne())
    deliver_yellow = Selector(name="黄色ボトルなら第1青線で配送", memory=True)
    deliver_yellow_action = Sequence(name="黄色ボトル配送動作", memory=True)
    yellow_forward = Parallel(name="黄色ゾーンへ前進", policy=ParallelPolicy.SuccessOnOne())
    yellow_back = Parallel(name="黄色ゾーンからラインまで後退", policy=ParallelPolicy.SuccessOnOne())
    yellow_line_found = Parallel(name="黄色配送後のライン再検出", policy=ParallelPolicy.SuccessOnOne())
    leave_blue1 = Parallel(name="第1青線を抜ける", policy=ParallelPolicy.SuccessOnOne())
    trace_to_blue2 = Parallel(name="第2青線までライントレース", policy=ParallelPolicy.SuccessOnOne())
    deliver_blue = Selector(name="青色ボトルなら第2青線で配送", memory=True)
    deliver_blue_action = Sequence(name="青色ボトル配送動作", memory=True)
    blue_forward = Parallel(name="青色ゾーンへ前進", policy=ParallelPolicy.SuccessOnOne())
    blue_back = Parallel(name="青色ゾーンからラインまで後退", policy=ParallelPolicy.SuccessOnOne())
    blue_line_found = Parallel(name="青色配送後のライン再検出", policy=ParallelPolicy.SuccessOnOne())
    leave_blue2 = Parallel(name="第2青線を抜ける", policy=ParallelPolicy.SuccessOnOne())
    trace_to_blue3 = Parallel(name="第3青線までライントレース", policy=ParallelPolicy.SuccessOnOne())
    deliver_red = Selector(name="赤色ボトルなら第3青線で配送", memory=True)
    deliver_red_action = Sequence(name="赤色ボトル配送動作", memory=True)
    red_forward = Parallel(name="赤色ゾーンへ前進", policy=ParallelPolicy.SuccessOnOne())
    red_back = Parallel(name="赤色ゾーンからラインまで後退", policy=ParallelPolicy.SuccessOnOne())
    red_line_found = Parallel(name="赤色配送後のライン再検出", policy=ParallelPolicy.SuccessOnOne())
    before_next_gate = Sequence(name="ボトルデリバリー1周完了・次のLAPゲート通過前", memory=True)
    et_rally = Sequence(name="ETラリー3周", memory=True)
    rally_lap1 = Sequence(name="ETラリー第1周", memory=True)
    rally_lap2 = Sequence(name="ETラリー第2周", memory=True)
    rally_lap3 = Sequence(name="ETラリー第3周", memory=True)
    et_sumo = Sequence(name="ET相撲", memory=True)
    sumo_cross_black = Parallel(name="相撲ボトルを押して黒線を通過", policy=ParallelPolicy.SuccessOnOne())
    sumo_back = Parallel(name="アームを上げて後退", policy=ParallelPolicy.SuccessOnOne())
    sumo_left_run = Parallel(name="左向きで一定時間ジャイロ走行", policy=ParallelPolicy.SuccessOnOne())
    sumo_to_black = Parallel(name="アームを下げて黒線まで走行", policy=ParallelPolicy.SuccessOnOne())
    garage = Sequence(name="ガレージ停止", memory=True)
    garage_trace_to_white = Parallel(name="白までガレージライントレース", policy=ParallelPolicy.SuccessOnOne())
    garage_forward_on_white = Parallel(name="白検出後にガレージ中央へ直進", policy=ParallelPolicy.SuccessOnOne())

    calibration.add_children(
        [
            # ヒントカード2のQRコードを復号する4桁パスワードをPCから入力する。
            base.ReadKey(name="input hint2 password"),
            # アームを上端まで動かして機械端を確認する。
            base.ArmUpDownFull(name="arm up for calibration", direction=base.ArmDirection.UP),
            # アームを下端まで戻して初期姿勢を作る。
            base.ArmUpDownFull(name="arm down for calibration", direction=base.ArmDirection.DOWN),
            # モーター角度、ジャイロ、カメラのライン認識設定を初期化する。
            base.ResetDevice(name="reset devices"),
        ]
    )
    start.add_children(
        [
            # タッチセンサーが押されるまでスタートを待つ。
            base.IsTouchOn(name="wait touch start"),
            # スタート成立時刻を競技タイマーの原点として記録する。
            StartRaceTimer(name="start race timer"),
        ]
    )
    lap_gate.add_children(
        [
            # カラーセンサーでライン端を追従しながらLAPゲートへ進む。
            base.TraceLine(name="trace to LAP gate", target=base.TRACELINE_TARGET_V, power=55,
                pid_p=0.65, pid_i=0.000001, pid_d=0.045, trace_side=TraceSide.NORMAL),
            # 暫定距離へ到達した時点をLAPゲート通過とみなす。
            base.IsDistanceEarned(name="LAP gate passed", delta_dist=LAP_GATE_DISTANCE_MM),
        ]
    )
    bottle_catch.add_children(
        [
            # Webカメラをボトル認識モードへ切り替える。
            SetCameraTarget(name="camera bottle mode", target=TargetInterested.BOTTLE),
            # 色帯からボトル色を確定し、正面へ接近してキャッチする。
            base.CatchBottle(name="detect and catch delivery bottle", power=25,
                pid_p=0.8, pid_i=0.01, pid_d=0.04, catch_run_mm=150),
            # ボトルキャッチ完了時に左右モーターを停止する。
            base.StopNow(name="stop after bottle catch"),
        ]
    )
    hint1_run_and_read.add_children(
        [
            # ジャイロで既知のヒントカード1方向へ進み続ける。
            base.RunByGyro(name="run toward hint1", target=HINT1_DIRECTION_DEG, power=30,
                pid_p=1.1, pid_i=0.1, pid_d=0.03, target_type=base.HeadingType.ABSOLUTE),
            # 黒線または青線のどちらかを検出したらヒント1方向への走行を終了する。
            hint1_end_colors,
            # 走行を終了させずに、映ったヒント1を読み取り無線送信する。
            DecodeAndSendHint(name="read and send hint1 while running",
                hint_type=HintType.HINT1, complete_on_decode=False),
        ]
    )
    hint1_end_colors.add_children(
        [
            # カラーセンサーが黒線を検出したら走行終了条件を成立させる。
            base.IsColorDetected(name="hint1 black detected", color=Color.BLACK),
            # カラーセンサーが青線を検出したら走行終了条件を成立させる。
            base.IsColorDetected(name="hint1 blue detected", color=Color.BLUE),
        ]
    )
    hint1_scan_motion.add_children(
        [
            # QR読取り開始後、最初の画角で2秒間待機する。
            base.IsTimePassed(name="wait hint1 center angle", delta_time=QR_RETRY_INTERVAL_SEC),
            # 現在方位から右へ少し旋回し、QRが映る画角へ変更する。
            base.SpinAround(name="turn right a little for hint1", target=-QR_RETRY_ANGLE_DEG,
                max_power=base.SPIN_MAX_POWER, min_power=base.SPIN_MIN_POWER,
                pid_p=0.4, pid_i=0.001, pid_d=0.03, target_type=base.HeadingType.RELATIVE),
            # 右側の画角で機体を停止する。
            base.StopNow(name="stop at hint1 right angle"),
            # 右側の画角を2秒間維持してQR認識を待つ。
            base.IsTimePassed(name="wait hint1 right angle", delta_time=QR_RETRY_INTERVAL_SEC),
            # 右側から左側へ角度を変え、QRの別の画角を作る。
            base.SpinAround(name="turn left for hint1", target=QR_RETRY_ANGLE_DEG * 2,
                max_power=base.SPIN_MAX_POWER, min_power=base.SPIN_MIN_POWER,
                pid_p=0.4, pid_i=0.001, pid_d=0.03, target_type=base.HeadingType.RELATIVE),
            # 左側の画角で機体を停止する。
            base.StopNow(name="stop at hint1 left angle"),
            # 左側の画角を2秒間維持してQR認識を待つ。
            base.IsTimePassed(name="wait hint1 left angle", delta_time=QR_RETRY_INTERVAL_SEC),
            # 元の正面付近へ少し右旋回して戻る。
            base.SpinAround(name="return center for hint1", target=-QR_RETRY_ANGLE_DEG,
                max_power=base.SPIN_MAX_POWER, min_power=base.SPIN_MIN_POWER,
                pid_p=0.4, pid_i=0.001, pid_d=0.03, target_type=base.HeadingType.RELATIVE),
            # QR取得まで正面付近で停止状態を維持する。
            base.RunAsInstructed(name="hold still for hint1 QR", pwm_l=0, pwm_r=0),
        ]
    )
    hint1_scan.add_children(
        [
            # ヒント1を取得した時だけ再探索を終了する。
            DecodeAndSendHint(name="read and send hint1 while stopped",
                hint_type=HintType.HINT1, complete_on_decode=True),
            # 既存の待機・旋回・停止ノードを順に実行し、QRが映る画角を探す。
            hint1_scan_motion,
        ]
    )
    hint1_ensure.add_children(
        [
            # 走行中にヒント1を取得済みなら首振り探索を省略する。
            HasHint(name="hint1 already received", hint_type=HintType.HINT1),
            # 未取得なら静止し、取得できるまで左右へ画角を変える。
            hint1_scan,
        ]
    )
    hint1.add_children(
        [
            # Webカメラを高解像度のQRコード認識モードへ切り替える。
            SetCameraTarget(name="camera QR mode for hint1", target=TargetInterested.QRCODE),
            # 黒線または青線までジャイロ走行し、その間にヒント1を読み取る。
            hint1_run_and_read,
            # 並行走行終了時に左右モーターを停止する。
            base.StopNow(name="stop after hint1 direction run"),
            # 走行中に読めなかった場合だけ、2秒間隔の首振り探索を行う。
            hint1_ensure,
        ]
    )
    hint2_trace_and_read.add_children(
        [
            # カラーセンサー式ライントレースを続ける。WebカメラはQRモードのまま使用する。
            base.TraceLine(name="trace while reading hint2", target=base.TRACELINE_TARGET_V, power=30,
                pid_p=0.65, pid_i=0.000001, pid_d=0.045, trace_side=TraceSide.NORMAL),
            # ヒント2を復号・送信できた時点で並行走行を終了する。
            DecodeAndSendHint(name="read decrypt and send hint2",
                hint_type=HintType.HINT2, complete_on_decode=True),
            # 暫定時間を超えた場合も走行を止め、静止探索へ切り替える。
            base.IsTimePassed(name="hint2 trace timeout", delta_time=HINT2_TRACE_TIMEOUT_SEC),
        ]
    )
    hint2_scan_motion.add_children(
        [
            # QR読取り開始後、最初の画角で2秒間待機する。
            base.IsTimePassed(name="wait hint2 center angle", delta_time=QR_RETRY_INTERVAL_SEC),
            # 現在方位から右へ少し旋回し、QRが映る画角へ変更する。
            base.SpinAround(name="turn right a little for hint2", target=-QR_RETRY_ANGLE_DEG,
                max_power=base.SPIN_MAX_POWER, min_power=base.SPIN_MIN_POWER,
                pid_p=0.4, pid_i=0.001, pid_d=0.03, target_type=base.HeadingType.RELATIVE),
            # 右側の画角で機体を停止する。
            base.StopNow(name="stop at hint2 right angle"),
            # 右側の画角を2秒間維持してQR認識を待つ。
            base.IsTimePassed(name="wait hint2 right angle", delta_time=QR_RETRY_INTERVAL_SEC),
            # 右側から左側へ角度を変え、QRの別の画角を作る。
            base.SpinAround(name="turn left for hint2", target=QR_RETRY_ANGLE_DEG * 2,
                max_power=base.SPIN_MAX_POWER, min_power=base.SPIN_MIN_POWER,
                pid_p=0.4, pid_i=0.001, pid_d=0.03, target_type=base.HeadingType.RELATIVE),
            # 左側の画角で機体を停止する。
            base.StopNow(name="stop at hint2 left angle"),
            # 左側の画角を2秒間維持してQR認識を待つ。
            base.IsTimePassed(name="wait hint2 left angle", delta_time=QR_RETRY_INTERVAL_SEC),
            # 元の正面付近へ少し右旋回して戻る。
            base.SpinAround(name="return center for hint2", target=-QR_RETRY_ANGLE_DEG,
                max_power=base.SPIN_MAX_POWER, min_power=base.SPIN_MIN_POWER,
                pid_p=0.4, pid_i=0.001, pid_d=0.03, target_type=base.HeadingType.RELATIVE),
            # QR取得まで正面付近で停止状態を維持する。
            base.RunAsInstructed(name="hold still for hint2 QR", pwm_l=0, pwm_r=0),
        ]
    )
    hint2_scan.add_children(
        [
            # ヒント2を復号・送信した時だけ再探索を終了する。
            DecodeAndSendHint(name="read hint2 while stopped",
                hint_type=HintType.HINT2, complete_on_decode=True),
            # 既存の待機・旋回・停止ノードを順に実行し、QRが映る画角を探す。
            hint2_scan_motion,
        ]
    )
    hint2_ensure.add_children(
        [
            # ライントレース中にヒント2を取得済みなら首振り探索を省略する。
            HasHint(name="hint2 already received", hint_type=HintType.HINT2),
            # 未取得なら静止し、復号できるまで左右へ画角を変える。
            hint2_scan,
        ]
    )
    hint2.add_children(
        [
            # ライントレースとQR読取りを並行し、取得またはタイムアウトまで進む。
            hint2_trace_and_read,
            # QR取得またはタイムアウト直後に左右モーターを停止する。
            base.StopNow(name="stop after hint2 trace"),
            # タイムアウト時だけ、ヒント2を取得できるまで首振り探索する。
            hint2_ensure,
            # ボトルデリバリーへ戻るためカメラをライン認識モードへ切り替える。
            SetCameraTarget(name="camera line mode after hints", target=TargetInterested.LINE),
        ]
    )

    # 以下の配送サブツリーは、ボトル色が一致しなければAlwaysSuccessでその色をスキップする。
    yellow_forward.add_children(
        [
            # 右を向いた姿勢をジャイロで維持し、黄色ゾーンへ前進する。
            base.RunByGyro(name="yellow zone forward", target=0, power=25,
                pid_p=1.1, pid_i=0.1, pid_d=0.03, target_type=base.HeadingType.RELATIVE),
            # 指定距離へ到達した時点で黄色ゾーンへの前進を終了する。
            base.IsDistanceEarned(name="yellow zone distance", delta_dist=BOTTLE_DELIVERY_DISTANCE_MM),
        ]
    )
    yellow_back.add_children(
        [
            # 黄色ゾーンから左右輪を同じ出力で後退させる。
            base.RunAsInstructed(name="back to line from yellow", pwm_l=-22, pwm_r=-22),
            # 黒線または青線を検出したらラインへ戻ったと判定する。
            yellow_line_found,
        ]
    )
    yellow_line_found.add_children(
        [
            # 後退中に黒線を検出したらラインへ戻ったと判定する。
            base.IsColorDetected(name="black line found after yellow", color=Color.BLACK),
            # 後退中に青線を検出したらラインへ戻ったと判定する。
            base.IsColorDetected(name="blue line found after yellow", color=Color.BLUE),
        ]
    )
    deliver_yellow_action.add_children(
        [
            # キャッチしたボトルが黄色の場合だけ、後続の黄色配送動作へ進む。
            base.HasCaughtBottle(name="caught bottle is yellow", color=BottleColor.YELLOW),
            # レフトコース基準で右へ90度旋回し、黄色ゾーンを正面に捉える。
            base.SpinAround(name="turn right for yellow", target=-90, max_power=base.SPIN_MAX_POWER,
                min_power=base.SPIN_MIN_POWER, pid_p=0.4, pid_i=0.001, pid_d=0.03,
                target_type=base.HeadingType.RELATIVE),
            # 黄色ゾーンへ指定距離だけ前進する。
            yellow_forward,
            # 黄色ゾーンでボトルを配送する位置に停止する。
            base.StopNow(name="stop at yellow zone"),
            # カラーセンサーがラインを再検出するまで後退する。
            yellow_back,
            # ライン再検出位置で停止する。
            base.StopNow(name="stop on line after yellow"),
            # 左へ90度旋回し、ライントレースを再開できる正面姿勢へ戻す。
            base.SpinAround(name="turn left to front after yellow", target=90,
                max_power=base.SPIN_MAX_POWER, min_power=base.SPIN_MIN_POWER,
                pid_p=0.4, pid_i=0.001, pid_d=0.03, target_type=base.HeadingType.RELATIVE),
        ]
    )
    deliver_yellow.add_children(
        [
            # 黄色ボトルの場合は、右旋回・配送・後退・正面復帰を実行する。
            deliver_yellow_action,
            # 黄色以外のボトルなら黄色配送を成功扱いでスキップする。
            AlwaysSuccess(name="skip yellow delivery"),
        ]
    )

    blue_forward.add_children(
        [
            # 右を向いた姿勢をジャイロで維持し、青色ゾーンへ前進する。
            base.RunByGyro(name="blue zone forward", target=0, power=25,
                pid_p=1.1, pid_i=0.1, pid_d=0.03, target_type=base.HeadingType.RELATIVE),
            # 指定距離へ到達した時点で青色ゾーンへの前進を終了する。
            base.IsDistanceEarned(name="blue zone distance", delta_dist=BOTTLE_DELIVERY_DISTANCE_MM),
        ]
    )
    blue_back.add_children(
        [
            # 青色ゾーンから左右輪を同じ出力で後退させる。
            base.RunAsInstructed(name="back to line from blue", pwm_l=-22, pwm_r=-22),
            # 黒線または青線を検出したらラインへ戻ったと判定する。
            blue_line_found,
        ]
    )
    blue_line_found.add_children(
        [
            # 後退中に黒線を検出したらラインへ戻ったと判定する。
            base.IsColorDetected(name="black line found after blue", color=Color.BLACK),
            # 後退中に青線を検出したらラインへ戻ったと判定する。
            base.IsColorDetected(name="blue line found after blue", color=Color.BLUE),
        ]
    )
    deliver_blue_action.add_children(
        [
            # キャッチしたボトルが青色の場合だけ、後続の青色配送動作へ進む。
            base.HasCaughtBottle(name="caught bottle is blue", color=BottleColor.BLUE),
            # レフトコース基準で右へ90度旋回し、青色ゾーンを正面に捉える。
            base.SpinAround(name="turn right for blue", target=-90, max_power=base.SPIN_MAX_POWER,
                min_power=base.SPIN_MIN_POWER, pid_p=0.4, pid_i=0.001, pid_d=0.03,
                target_type=base.HeadingType.RELATIVE),
            # 青色ゾーンへ指定距離だけ前進する。
            blue_forward,
            # 青色ゾーンでボトルを配送する位置に停止する。
            base.StopNow(name="stop at blue zone"),
            # カラーセンサーがラインを再検出するまで後退する。
            blue_back,
            # ライン再検出位置で停止する。
            base.StopNow(name="stop on line after blue"),
            # 左へ90度旋回し、ライントレースを再開できる正面姿勢へ戻す。
            base.SpinAround(name="turn left to front after blue", target=90,
                max_power=base.SPIN_MAX_POWER, min_power=base.SPIN_MIN_POWER,
                pid_p=0.4, pid_i=0.001, pid_d=0.03, target_type=base.HeadingType.RELATIVE),
        ]
    )
    deliver_blue.add_children(
        [
            # 青色ボトルの場合は、右旋回・配送・後退・正面復帰を実行する。
            deliver_blue_action,
            # 青色以外のボトルなら青色配送を成功扱いでスキップする。
            AlwaysSuccess(name="skip blue delivery"),
        ]
    )

    red_forward.add_children(
        [
            # 右を向いた姿勢をジャイロで維持し、赤色ゾーンへ前進する。
            base.RunByGyro(name="red zone forward", target=0, power=25,
                pid_p=1.1, pid_i=0.1, pid_d=0.03, target_type=base.HeadingType.RELATIVE),
            # 指定距離へ到達した時点で赤色ゾーンへの前進を終了する。
            base.IsDistanceEarned(name="red zone distance", delta_dist=BOTTLE_DELIVERY_DISTANCE_MM),
        ]
    )
    red_back.add_children(
        [
            # 赤色ゾーンから左右輪を同じ出力で後退させる。
            base.RunAsInstructed(name="back to line from red", pwm_l=-22, pwm_r=-22),
            # 黒線または青線を検出したらラインへ戻ったと判定する。
            red_line_found,
        ]
    )
    red_line_found.add_children(
        [
            # 後退中に黒線を検出したらラインへ戻ったと判定する。
            base.IsColorDetected(name="black line found after red", color=Color.BLACK),
            # 後退中に青線を検出したらラインへ戻ったと判定する。
            base.IsColorDetected(name="blue line found after red", color=Color.BLUE),
        ]
    )
    deliver_red_action.add_children(
        [
            # キャッチしたボトルが赤色の場合だけ、後続の赤色配送動作へ進む。
            base.HasCaughtBottle(name="caught bottle is red", color=BottleColor.RED),
            # レフトコース基準で右へ90度旋回し、赤色ゾーンを正面に捉える。
            base.SpinAround(name="turn right for red", target=-90, max_power=base.SPIN_MAX_POWER,
                min_power=base.SPIN_MIN_POWER, pid_p=0.4, pid_i=0.001, pid_d=0.03,
                target_type=base.HeadingType.RELATIVE),
            # 赤色ゾーンへ指定距離だけ前進する。
            red_forward,
            # 赤色ゾーンでボトルを配送する位置に停止する。
            base.StopNow(name="stop at red zone"),
            # カラーセンサーがラインを再検出するまで後退する。
            red_back,
            # ライン再検出位置で停止する。
            base.StopNow(name="stop on line after red"),
            # 左へ90度旋回し、ライントレースを再開できる正面姿勢へ戻す。
            base.SpinAround(name="turn left to front after red", target=90,
                max_power=base.SPIN_MAX_POWER, min_power=base.SPIN_MIN_POWER,
                pid_p=0.4, pid_i=0.001, pid_d=0.03, target_type=base.HeadingType.RELATIVE),
        ]
    )
    deliver_red.add_children(
        [
            # 赤色ボトルの場合は、右旋回・配送・後退・正面復帰を実行する。
            deliver_red_action,
            # 赤色以外のボトルなら赤色配送を成功扱いでスキップする。
            AlwaysSuccess(name="skip red delivery"),
        ]
    )

    trace_to_blue1.add_children(
        [
            # カラーセンサーでライン端を追従し、第1青線へ進む。
            base.TraceLine(name="trace to first blue", target=base.TRACELINE_TARGET_V, power=30,
                pid_p=0.65, pid_i=0.000001, pid_d=0.045, trace_side=TraceSide.NORMAL),
            # 第1青線を検出した時点でライントレースを終了する。
            base.IsColorDetected(name="first blue detected", color=Color.BLUE),
        ]
    )
    leave_blue1.add_children(
        [
            # 正面方位を維持して直進し、第1青線から機体を出す。
            base.RunByGyro(name="leave first blue", target=0, power=20,
                pid_p=1.1, pid_i=0.1, pid_d=0.03, target_type=base.HeadingType.RELATIVE),
            # 青色が連続して見えなくなった時点で第1青線を抜けたと判定する。
            IsColorLost(name="first blue lost", color=Color.BLUE),
        ]
    )
    trace_to_blue2.add_children(
        [
            # カラーセンサーでライン端を追従し、第2青線へ進む。
            base.TraceLine(name="trace to second blue", target=base.TRACELINE_TARGET_V, power=30,
                pid_p=0.65, pid_i=0.000001, pid_d=0.045, trace_side=TraceSide.NORMAL),
            # 第2青線を検出した時点でライントレースを終了する。
            base.IsColorDetected(name="second blue detected", color=Color.BLUE),
        ]
    )
    leave_blue2.add_children(
        [
            # 正面方位を維持して直進し、第2青線から機体を出す。
            base.RunByGyro(name="leave second blue", target=0, power=20,
                pid_p=1.1, pid_i=0.1, pid_d=0.03, target_type=base.HeadingType.RELATIVE),
            # 青色が連続して見えなくなった時点で第2青線を抜けたと判定する。
            IsColorLost(name="second blue lost", color=Color.BLUE),
        ]
    )
    trace_to_blue3.add_children(
        [
            # カラーセンサーでライン端を追従し、第3青線へ進む。
            base.TraceLine(name="trace to third blue", target=base.TRACELINE_TARGET_V, power=30,
                pid_p=0.65, pid_i=0.000001, pid_d=0.045, trace_side=TraceSide.NORMAL),
            # 第3青線を検出した時点でライントレースを終了する。
            base.IsColorDetected(name="third blue detected", color=Color.BLUE),
        ]
    )
    bottle_delivery.add_children(
        [
            # 第1青線まで進み、黄色ボトルの場合だけ第1配送動作を行う。
            trace_to_blue1,
            # キャッチした色が黄色なら黄色ゾーンへ配送し、それ以外ならスキップする。
            deliver_yellow,
            # 第1青線の青色が見えなくなるまで正面へ直進する。
            leave_blue1,
            # 第2青線までライントレースする。
            trace_to_blue2,
            # キャッチした色が青なら青色ゾーンへ配送し、それ以外ならスキップする。
            deliver_blue,
            # 第2青線の青色が見えなくなるまで正面へ直進する。
            leave_blue2,
            # 第3青線までライントレースする。
            trace_to_blue3,
            # キャッチした色が赤なら赤色ゾーンへ配送し、それ以外ならスキップする。
            deliver_red,
        ]
    )

    before_next_gate.add_children([
        # ボトルデリバリー1周完了後、次のLAPゲートをくぐる前で完全停止する。
        base.StopNow(name="one lap complete before next LAP gate"),
        # 無線通信デバイスがヒント1・2から作った最適経路を受信する。
        ReceiveStrategyRoute(name="receive optimized rally route"),
    ])
    rally_lap1.add_children(
        [
            # 第1周開始前に両脇の赤ゲートを基準として正面を合わせ、ジャイロを0度へ戻す。
            AlignAndResetGyroByRedGates(name="align red gates before rally lap1", lap_index=0),
            # 第1周用の(絶対方位角, 距離)配列を先頭から順に実行する。
            ExecuteStrategyLap(name="execute optimized rally lap1", lap_index=0),
        ]
    )
    rally_lap2.add_children(
        [
            # 第2周開始前に両脇の赤ゲートを基準として正面を合わせ、ジャイロを0度へ戻す。
            AlignAndResetGyroByRedGates(name="align red gates before rally lap2", lap_index=1),
            # 第2周用の(絶対方位角, 距離)配列を先頭から順に実行する。
            ExecuteStrategyLap(name="execute optimized rally lap2", lap_index=1),
        ]
    )
    rally_lap3.add_children(
        [
            # 第3周開始前に両脇の赤ゲートを基準として正面を合わせ、ジャイロを0度へ戻す。
            AlignAndResetGyroByRedGates(name="align red gates before rally lap3", lap_index=2),
            # 第3周用の(絶対方位角, 距離)配列を先頭から順に実行する。
            ExecuteStrategyLap(name="execute optimized rally lap3", lap_index=2),
        ]
    )
    et_rally.add_children(
        [
            # 赤ゲート補正後に最適経路でETラリー第1周を走行する。
            rally_lap1,
            # 赤ゲート補正後に最適経路でETラリー第2周を走行する。
            rally_lap2,
            # 赤ゲート補正後に最適経路でETラリー第3周を走行する。
            rally_lap3,
        ]
    )

    sumo_cross_black.add_children(
        [
            # 相撲ボトルを正面から押し、ジャイロで方位を保って直進する。
            base.RunByGyro(name="push sumo bottle across black", target=0, power=25,
                pid_p=1.1, pid_i=0.1, pid_d=0.03, target_type=base.HeadingType.RELATIVE),
            # 黒線を検出した後、再び黒以外になった時点で押出しを終了する。
            IsBlackPassed(name="black line entered and exited"),
        ]
    )
    sumo_back.add_children(
        [
            # アームを上げた後、相撲ボトルから左右輪を同じ出力で後退させる。
            base.RunAsInstructed(name="back away from sumo bottle", pwm_l=-25, pwm_r=-25),
            # 暫定の後退時間が経過した時点で後退を終了する。
            base.IsTimePassed(name="sumo backward time", delta_time=SUMO_BACK_TIME_SEC),
        ]
    )
    sumo_left_run.add_children(
        [
            # 絶対角度で左を向いた姿勢を維持し、次の黒線方向へ進む。
            base.RunByGyro(name="run left after sumo", target=-90, power=25,
                pid_p=1.1, pid_i=0.1, pid_d=0.03, target_type=base.HeadingType.ABSOLUTE),
            # 暫定の走行時間が経過した時点で左方向走行を終了する。
            base.IsTimePassed(name="sumo left run time", delta_time=SUMO_LEFT_RUN_TIME_SEC),
        ]
    )
    sumo_to_black.add_children(
        [
            # アームを下げ、絶対左向きを維持して黒線へ接近する。
            base.RunByGyro(name="run to black after arm down", target=-90, power=25,
                pid_p=1.1, pid_i=0.1, pid_d=0.03, target_type=base.HeadingType.ABSOLUTE),
            # カラーセンサーが黒を検出した時点で接近走行を終了する。
            base.IsColorDetected(name="black detected after sumo", color=Color.BLACK),
        ]
    )
    et_sumo.add_children(
        [
            # カメラ視界を確保するためアームを上端まで上げる。
            base.ArmUpDownFull(name="arm up for sumo search", direction=base.ArmDirection.UP),
            # 左右へ首を振り、Webカメラで相撲ボトルを正面に捉える。
            SearchAndFaceSumoBottle(name="search and face sumo bottle"),
            # 相撲ボトルを押せる高さまでアームを下げる。
            base.ArmUpDownFull(name="arm down before sumo push", direction=base.ArmDirection.DOWN),
            # 黒線を一度検出し、さらに黒線外へ出るまでボトルを正面から押す。
            sumo_cross_black,
            # 黒線通過後に左右モーターを停止する。
            base.StopNow(name="stop after crossing black"),
            # ボトルから離れる前にアームを上端まで上げる。
            base.ArmUpDownFull(name="arm up after sumo push", direction=base.ArmDirection.UP),
            # 暫定時間だけ直線後退してボトルとの距離を確保する。
            sumo_back,
            # 後退完了位置で左右モーターを停止する。
            base.StopNow(name="stop after sumo back"),
            # ジャイロ絶対角度-90度へ旋回し、レフトコース基準の左を向く。
            base.SpinAround(name="face absolute left after sumo", target=-90,
                max_power=base.SPIN_MAX_POWER, min_power=base.SPIN_MIN_POWER,
                pid_p=0.4, pid_i=0.001, pid_d=0.03, target_type=base.HeadingType.ABSOLUTE),
            # 左向きを維持し、暫定時間だけジャイロ走行する。
            sumo_left_run,
            # 左方向の時間走行が完了した位置で停止する。
            base.StopNow(name="stop after sumo left run"),
            # 次の黒線をカラーセンサーで探せる高さまでアームを下げる。
            base.ArmUpDownFull(name="arm down before black approach", direction=base.ArmDirection.DOWN),
            # 絶対左向きを維持し、カラーセンサーが黒を検出するまで進む。
            sumo_to_black,
            # 黒線検出位置で左右モーターを停止する。
            base.StopNow(name="stop on black after sumo"),
            # 左へ相対90度旋回し、ガレージへ続くラインを正面に捉える。
            base.SpinAround(name="turn left 90 to garage line", target=90,
                max_power=base.SPIN_MAX_POWER, min_power=base.SPIN_MIN_POWER,
                pid_p=0.4, pid_i=0.001, pid_d=0.03, target_type=base.HeadingType.RELATIVE),
        ]
    )
    garage_trace_to_white.add_children(
        [
            # カラーセンサーでライン端を追従し、ガレージへ進む。
            base.TraceLine(name="trace line to garage white", target=base.TRACELINE_TARGET_V, power=25,
                pid_p=0.65, pid_i=0.000001, pid_d=0.045, trace_side=TraceSide.NORMAL),
            # カラーセンサーが白だけを検出した時点でライントレースを終了する。
            base.IsColorDetected(name="garage white detected", color=Color.WHITE),
        ]
    )
    garage_forward_on_white.add_children(
        [
            # 白検出後、左右輪を同じ出力でガレージ中央へ直進させる。
            base.RunAsInstructed(name="run forward inside garage", pwm_l=20, pwm_r=20),
            # 指定秒数が経過した時点でガレージ内の直進を終了する。
            base.IsTimePassed(name="garage forward time", delta_time=GARAGE_WHITE_RUN_TIME_SEC),
        ]
    )
    garage.add_children(
        [
            # カラーセンサーが白を検出するまで既存TraceLineでガレージへ進む。
            garage_trace_to_white,
            # 白検出後は既存の固定PWM走行と時間条件でガレージ中央へ入る。
            garage_forward_on_white,
            # ガレージ内で左右モーターを完全停止する。
            base.StopNow(name="garage complete stop"),
        ]
    )
    root.add_children(
        [
            # パスワード入力、アーム端確認、各デバイス初期化を行う。
            calibration,
            # タッチスタート成立時に競技タイマーを開始する。
            start,
            # カラーセンサーでラインを追従し、暫定距離のLAPゲートまで進む。
            lap_gate,
            # Webカメラでデリバリーボトルの色を確定し、正面からキャッチする。
            bottle_catch,
            # ヒント1方向へ走行しながらQRを読み、未取得時は停止して首振り探索する。
            hint1,
            # ライントレース中にヒント2を復号し、未取得時は停止して首振り探索する。
            hint2,
            # 青線を順に検出し、保持したボトル色に対応する色ゾーンだけで配送する。
            bottle_delivery,
            # 1周完了・次のLAPゲート通過前で停止し、最適経路を受信する。
            before_next_gate,
            # 毎周の赤ゲート補正後、受信した角度・距離配列でETラリーを3周する。
            et_rally,
            # カメラで相撲ボトルを正面へ捉え、黒線越しに押して次のラインへ移る。
            et_sumo,
            # ライントレースでガレージへ進み、白のみの検出後に数秒直進して停止する。
            garage,
            # すべての競技工程完了後、Ctrl+Cによる安全終了を待つ。
            base.TheEnd(name="end"),
        ]
    )
    return root


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ETロボコン2026 レフトコース")
    parser.add_argument("--wireless-host", default="127.0.0.1")
    parser.add_argument("--wireless-port", type=int, default=50000)
    parser.add_argument("--route-listen-port", type=int, default=50001)
    parser.add_argument("--logfile", type=str, default=None)
    args = parser.parse_args()

    base.g_course = 1
    g_wireless = WirelessStrategyDevice(
        host=args.wireless_host,
        port=args.wireless_port,
        listen_port=args.route_listen_port,
    )
    base.setup_thread()
    tree = build_behaviour_tree()
    signal.signal(signal.SIGTERM, base.sig_handler)
    try:
        etrobo = base.initialize_etrobo(backend="raspike_art")
        etrobo.add_handler(base.TraverseBehaviourTree(tree))
        etrobo.dispatch(interval=base.EXEC_INTERVAL, logfile=args.logfile)
    finally:
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        base.cleanup_thread()
        signal.signal(signal.SIGTERM, signal.SIG_DFL)
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        print(" -- exiting...")
