"""ETラリー用、カメラのマーカー(座標プリント)を使った走行体の向き補正(案A)。

【位置づけ(2026-09、実機未検証)】
このファイル一式は、実機でのカメラ較正(EXPECTED_MARKER_ANGLE_DEG・
ANGLE_TO_HEADING_GAIN、py_etrobo_util/et_rally_marker_pose.py参照)が
終わるまで、本番経路(features/execute_strategy.py)には組み込まない。
et_rally_turn_lqi.pyと同じく、実測データが無い間はプレースホルダーの
較正値のまま置いておく設計。

【「瞬間的」に見せるための設計】
- PrepareEtRallyMarkerCameraは、hint_reader.PrepareHintCameraと同じ考え方で、
  移動中にカメラモードの切替だけ先に済ませておく(切替コストを走行の裏に隠す)。
- EtRallyMarkerAngleCorrectionは、カメラの結果を待つ時間に上限(read_timeout_sec)を
  設け、間に合わなければ補正なしで即座に終わる(止まって待ち続けない)。
- 補正の実行そのものは、フェーズ1(エンコーダの大まかな回転)を経由せず、
  EtRallyTurnLqiをtarget_type=RELATIVEの小さい角度で直接使う。小角度なので
  フェーズ1はほぼ一瞬で終わり、実質的にフェーズ2(低出力のLQI仕上げ)だけが
  短く動く形になる想定(実機未検証)。
"""
import time

from py_trees.behaviour import Behaviour
from py_trees.common import Status

from py_etrobo_util import TargetInterested
from py_etrobo_util.et_rally_marker_pose import estimate_heading_correction_deg

from ..runtime import runtime
from ..types import HeadingType
from .et_rally_turn_lqi import EtRallyTurnLqi

TAG = "ET_MARKER"


class PrepareEtRallyMarkerCamera(Behaviour):
    """カメラのモード切替だけを先に済ませる(移動中に仕込み、待ち時間を作らない)。"""
    def update(self) -> Status:
        runtime.require("video")
        runtime.video.prepare_et_rally_marker_camera()
        return Status.SUCCESS


class EtRallyMarkerAngleCorrection(Behaviour):
    """マーカーを1回読み取り、信頼できる結果が得られれば、小さい向き補正を行う。

    読み取りに失敗/タイムアウト/補正量が大きすぎる(誤検出の疑い)場合は、
    補正を行わずSUCCESSで即終了する(フェイルセーフ: 判断に迷ったら何もしない)。
    """
    def __init__(self, name: str, *,
                 read_timeout_sec: float = 0.3,
                 main_power: int = 40, decel_deg: float = 5.0, decel_power: int = 30,
                 fine_max_power: int = 60, stiction_floor: float = 0.0,
                 fine_tolerance_deg: float = 0.5, fine_omega_tolerance_deg: float = 3.0,
                 max_fine_duration_sec: float = 1.5) -> None:
        super().__init__(name)
        self.read_timeout_sec = read_timeout_sec
        self._turn_kwargs = dict(
            main_power=main_power, decel_deg=decel_deg, decel_power=decel_power,
            fine_max_power=fine_max_power, stiction_floor=stiction_floor,
            fine_tolerance_deg=fine_tolerance_deg,
            fine_omega_tolerance_deg=fine_omega_tolerance_deg,
            max_fine_duration_sec=max_fine_duration_sec,
        )
        self.phase = None  # None -> 'waiting' -> 'correcting' -> 終了
        self.turn_child = None

    def _finish_without_correction(self, reason: str) -> Status:
        self.logger.info("%s %s.skip reason=%s elapsed=%.3fs" % (
            TAG, self.__class__.__name__, reason, time.monotonic() - self.started_at))
        return Status.SUCCESS

    def update(self) -> Status:
        runtime.require("video")
        if self.phase is None:
            self.started_at = time.monotonic()
            self.session = runtime.video.begin_et_rally_marker_read()
            self.phase = "waiting"
            self.logger.info("%s %s.start session=%d" % (TAG, self.__class__.__name__, self.session))

        if self.phase == "waiting":
            session, frame_id, markers = runtime.video.get_et_rally_marker_observation()
            if session == self.session and frame_id >= 0:
                if not markers:
                    return self._finish_without_correction("no_marker_detected")
                correction_deg, raw_angle = estimate_heading_correction_deg(markers[0]["corners"])
                if correction_deg is None:
                    self.logger.warning("%s %s.untrusted raw_angle=%s" % (
                        TAG, self.__class__.__name__, raw_angle))
                    return self._finish_without_correction("untrusted_correction")
                self.logger.info("%s %s.correction deg=%+.2f raw_marker_angle=%+.2f" % (
                    TAG, self.__class__.__name__, correction_deg, raw_angle))
                self.turn_child = EtRallyTurnLqi(
                    self.name + ".turn", target=correction_deg, target_type=HeadingType.RELATIVE,
                    **self._turn_kwargs)
                self.phase = "correcting"
                # このtickのうちに、そのまま最初のcorrecting tickへ進む。
            elif time.monotonic() - self.started_at >= self.read_timeout_sec:
                return self._finish_without_correction("timeout")
            else:
                return Status.RUNNING

        if self.phase == "correcting":
            status = self.turn_child.update()
            if status == Status.RUNNING:
                return Status.RUNNING
            self.logger.info("%s %s.done status=%s elapsed=%.3fs" % (
                TAG, self.__class__.__name__, status, time.monotonic() - self.started_at))
            return status

        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        if self.turn_child is not None:
            self.turn_child.terminate(new_status)
        if runtime.video is not None:
            runtime.video.set_target_interested(TargetInterested.LINE)
        self.phase = None
        self.turn_child = None


class EtRallyMarkerObserveLoop(Behaviour):
    """走行体を動かさず、マーカー検出だけを繰り返しログへ出す確認用ミッション
    (--mission et-rally-marker-observe)。EXPECTED_MARKER_ANGLE_DEG/
    ANGLE_TO_HEADING_GAINの較正前に、まず検出そのものが安定するかを見るためのもの。
    実際の向き補正(EtRallyMarkerAngleCorrection)は使わない。
    """
    def __init__(self, name: str, iterations: int = 20,
                 read_timeout_sec: float = 0.5, interval_sec: float = 0.5) -> None:
        super().__init__(name)
        self.iterations = iterations
        self.read_timeout_sec = read_timeout_sec
        self.interval_sec = interval_sec
        self.count = 0
        self.phase = None

    def update(self) -> Status:
        runtime.require("video")
        if self.count >= self.iterations:
            return Status.SUCCESS

        if self.phase == "cooldown":
            if time.monotonic() - self.cooldown_started_at < self.interval_sec:
                return Status.RUNNING
            self.phase = None

        if self.phase is None:
            self.started_at = time.monotonic()
            self.session = runtime.video.begin_et_rally_marker_read()
            self.phase = "waiting"

        session, frame_id, markers = runtime.video.get_et_rally_marker_observation()
        elapsed = time.monotonic() - self.started_at
        if session == self.session and frame_id >= 0:
            if markers:
                correction_deg, raw_angle = estimate_heading_correction_deg(markers[0]["corners"])
                self.logger.info(
                    "%s %s.observe[%d] found=%d corner_count=%s raw_angle=%+.2f "
                    "correction=%s elapsed=%.3fs" % (
                        TAG, self.__class__.__name__, self.count, len(markers),
                        markers[0]["corner_count"], raw_angle,
                        ("%+.2f" % correction_deg) if correction_deg is not None else "untrusted",
                        elapsed))
            else:
                self.logger.info("%s %s.observe[%d] found=0 elapsed=%.3fs" % (
                    TAG, self.__class__.__name__, self.count, elapsed))
            self.count += 1
            self.phase = "cooldown"
            self.cooldown_started_at = time.monotonic()
            return Status.RUNNING
        if elapsed >= self.read_timeout_sec:
            self.logger.info("%s %s.observe[%d] timeout elapsed=%.3fs" % (
                TAG, self.__class__.__name__, self.count, elapsed))
            self.count += 1
            self.phase = "cooldown"
            self.cooldown_started_at = time.monotonic()
            return Status.RUNNING
        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        if runtime.video is not None:
            runtime.video.set_target_interested(TargetInterested.LINE)
        self.phase = None
