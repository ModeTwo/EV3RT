"""ETラリー専用: コース上の自分の位置(経路を作る道具の座標)を持つ(2026-11-08、走りながらのカメラ補正の土台)。

今の走り(et_rally_path_segment.py)は、止まらずに続く区間のつながりの中でだけ「計画の線からのずれ」を持ち、
止まる区間のあとは計画どおりの点にいるとみなして数え直す。コース上のどこにいるかは持っていない。
カメラでマーカーを見て位置を直すには、撮った瞬間にコース上のどこにいたかが要るので、ここで持つ。

- 位置の元: EtRallyHeading の位置の積算(et_loc。エンコーダと四捨五入前の向き、その場旋回中は動かさない)。
  走り出しの時点の et_loc を、コース上の走り出しの姿勢(スタートで測った位置、なければ計画のスタート)に合わせる。
- 座標: x はゴール側が+、y はスタートとゴールの列の側が+(mm)、向きは左回りが+(度)。et_rally_start_pose.py と同じ。
  et_loc は plotter と同じ形(x += d·sin(a)、y += d·cos(a)、a = ジャイロの向き)なので、
  スタートの向き h0(START_HEADING_DEG)のとき、(u, v) = (et_loc の dy, course × et_loc の dx) を h0 だけ回したものがコースの dx, dy、
  コースの向き = h0 + course × ジャイロの向き(ジャイロの0がスタートの向き)。
- 補正量(camera_correction): カメラで分かったずれを足す(今は足すだけの入れ物。使い方は実機のデータを見て決める)。
- 履歴: 直近約2秒の (時刻, x, y, 向き, 走った距離) を残す。撮った時刻の位置を引くため。

ログ: et_pose_text() に " course=(x,y,向き)" を足す(区間の終わりのログに出る)。
"""

import math
import time
from collections import deque

from py_trees.behaviour import Behaviour
from py_trees.common import Status

from .runtime import runtime

_anchor = None            # dict(plot=(x0, y0), plan=(x, y), course=±1)
_correction = [0.0, 0.0, 0.0]
_history = deque(maxlen=200)    # 1tickに数回呼ばれることもあるので、約2秒分より多めに持つ
_last = None
_recorder = None          # 2026-11-10: 走行中の画像と突き合わせるため、位置を時刻つきでファイルに残す(dict(file=, last_t=))


def _on_pose(gyro, heading):
    """EtRallyHeading.get_angle から呼ばれる(heading はジャイロの向き)。"""
    global _last
    if _anchor is None:
        return
    c = _anchor["course"]
    dpx = gyro.pose_x - _anchor["plot"][0]
    dpy = gyro.pose_y - _anchor["plot"][1]
    cos_h0, sin_h0 = _anchor["rot"]
    u, v = dpy, c * dpx
    x = _anchor["plan"][0] + u * cos_h0 - v * sin_h0 + _correction[0]
    y = _anchor["plan"][1] + u * sin_h0 + v * cos_h0 + _correction[1]
    h = _anchor["h0"] + c * heading + _correction[2]
    try:
        dist = runtime.plotter.get_distance()
    except Exception:
        dist = None
    _last = (time.monotonic(), x, y, h, dist)
    _history.append(_last)
    rec = _recorder
    if rec is not None and _last[0] - rec["last_t"] >= 0.009:
        rec["last_t"] = _last[0]
        rec["file"].write("%.4f %.2f %.2f %.3f %s\n" % (_last[0], x, y, h, "" if dist is None else dist))


def anchor(plan_pose, logger=None):
    """今の et_loc を、コース上の姿勢 plan_pose=(x, y, 向き度) に合わせる。向きはジャイロ側で合わせてある前提
    (スタートの取り込みで set_heading_offset 済み、なければ計画のスタートの向き)。"""
    from .et_rally_start_pose import START_HEADING_DEG
    global _anchor, _last
    gyro = runtime.gyro_sensor
    if not hasattr(gyro, "pose_listener"):
        if logger is not None:
            logger.warning("course pose: EtRallyHeading not installed, not tracked")
        return False
    gyro.get_angle()   # ここまでの位置を確定させる
    _anchor = dict(plot=(gyro.pose_x, gyro.pose_y), plan=(float(plan_pose[0]), float(plan_pose[1])),
                   course=1 if getattr(runtime, "course", -1) == 1 else -1, h0=START_HEADING_DEG,
                   rot=(math.cos(math.radians(START_HEADING_DEG)), math.sin(math.radians(START_HEADING_DEG))))
    _correction[:] = [0.0, 0.0, 0.0]
    _history.clear()
    _last = None
    gyro.pose_listener = _on_pose
    gyro.get_angle()
    if logger is not None:
        logger.info("course pose: anchored at (%.1f, %.1f), heading %+.2f deg" % (
            plan_pose[0], plan_pose[1], _last[3] if _last else float("nan")))
    return True


def release():
    global _anchor, _last
    gyro = runtime.gyro_sensor
    if getattr(gyro, "pose_listener", None) is _on_pose:
        gyro.pose_listener = None
    _anchor = None
    _last = None
    _history.clear()


def current():
    """(時刻, x, y, 向き度, 走った距離) か None。"""
    return _last


def at(t):
    """時刻 t の姿勢(前後の記録の間を直線でつなぐ)。履歴の外なら一番近い端。履歴がなければ None。"""
    hist = list(_history)
    if not hist:
        return None
    if t <= hist[0][0]:
        return hist[0]
    for a, b in zip(hist, hist[1:]):
        if a[0] <= t <= b[0]:
            k = 0.0 if b[0] == a[0] else (t - a[0]) / (b[0] - a[0])
            dh = (b[3] - a[3] + 180.0) % 360.0 - 180.0
            dist = None if a[4] is None or b[4] is None else a[4] + (b[4] - a[4]) * k
            return (t, a[1] + (b[1] - a[1]) * k, a[2] + (b[2] - a[2]) * k, a[3] + dh * k, dist)
    return hist[-1]


def add_correction(dx, dy, dh=0.0):
    """カメラで分かったずれを足す(今は入れ物だけ。使い方は実機のデータで決める)。"""
    _correction[0] += dx
    _correction[1] += dy
    _correction[2] += dh


def text():
    if _last is None:
        return ""
    return " course=(%.1f,%.1f,%+.1f)" % (_last[1], _last[2], _last[3])


class EtRallyAnchorCoursePose(Behaviour):
    """走り出しの直前に置く。スタートで測った位置を取り込んでいればその位置、なければ計画のスタートに合わせる。"""

    def update(self):
        from . import et_rally_start_pose as sp
        pose = sp.applied_start_pose() or (sp.START_MM[0], sp.START_MM[1], sp.START_HEADING_DEG)
        anchor(pose, self.logger)
        return Status.SUCCESS


class EtRallyReleaseCoursePose(Behaviour):
    def update(self):
        last = current()
        if last is not None:
            self.logger.info("course pose: last (%.1f, %.1f, %+.2f deg)" % (last[1], last[2], last[3]))
        release()
        return Status.SUCCESS


# ---------------- 走行中のカメラの画像を、撮った時刻とコース上の位置つきで残す ----------------
class EtRallyCameraLogStart(Behaviour):
    """走行中、every_n 枚に1枚、画像(JPEG)と、その時刻・コース上の位置・走った距離を保存する(実機の確認用)。
    保存先: 2026-Alpha/run_logs/et_camera_<日時>/(frame_<番号>.jpg と meta.jsonl)。
    画像は video.py の ET_RALLY_MARKER の裏のスレッドで受け取る(et_rally_marker_detector.set_detector で差し替え)。
    時刻は裏のスレッドが画像を受け取った時刻(撮った瞬間より少し遅い。どれだけ遅いかは画像の中のマーカーの動きから調べる)。"""

    state = None

    def __init__(self, name, every_n=5, quality=90, exposure=None):
        super().__init__(name)
        self.every_n = max(1, int(every_n))
        self.quality = int(quality)
        # 2026-11-09: 露出を手動で固定する値(V4L2の単位、1=100マイクロ秒。50なら5ms)。None なら自動のまま。
        # カメラの設定は画像を受け取るスレッドからしか触れないので、最初の画像が来たときに設定する。
        self.exposure = exposure

    def update(self):
        # 2026-11-10: 画像は子プロセス(et_rally_camera_proc.py)が撮って保存する(meta.jsonl に撮った時刻)。
        # 親はコース上の位置を時刻つきで poses.txt に残す(「時刻 x y 向き 走った距離」、あとで時刻で突き合わせる)。
        global _recorder
        from . import et_rally_camera_proc
        proc = et_rally_camera_proc.current()
        if proc is None or proc.folder is None:
            self.logger.warning("camera log: camera process not running, skipped")
            return Status.SUCCESS
        _recorder = dict(file=open(proc.folder / "poses.txt", "w", encoding="utf-8"), last_t=0.0)
        proc.send("save 1")
        EtRallyCameraLogStart.state = dict(folder=proc.folder)
        self.logger.info("camera log: saving every %d frames to %s (camera %s)" % (self.every_n, proc.folder, proc.ready))
        return Status.SUCCESS


class EtRallyCameraLogStop(Behaviour):
    def update(self):
        global _recorder
        st = EtRallyCameraLogStart.state
        EtRallyCameraLogStart.state = None
        if st is None:
            return Status.SUCCESS
        from . import et_rally_camera_proc
        rec, _recorder = _recorder, None
        stopped = et_rally_camera_proc.close_shared()
        if rec is not None:
            rec["file"].close()
        self.logger.info("camera log: stopped %s, saved in %s" % (stopped, st["folder"]))
        return Status.SUCCESS


class EtRallyCameraProcessStart(Behaviour):
    """2026-11-10: ETラリーの最初に、カメラの子プロセスを起動する(開くのに約1秒かかるので早めに)。
    走行中の画像を残すときは、保存先のフォルダもここで決める(保存を始めるのは EtRallyCameraLogStart)。"""

    def __init__(self, name, log_every=None, exposure=None, calib_path=None, quality=90):
        super().__init__(name)
        self.log_every = log_every
        self.exposure = exposure
        self.calib_path = calib_path
        self.quality = quality

    def update(self):
        from pathlib import Path
        from . import et_rally_camera_proc
        folder = None
        if self.log_every:
            folder = Path(__file__).resolve().parents[1] / "run_logs" / ("et_camera_" + time.strftime("%Y%m%dT%H%M%S"))
        try:
            proc = et_rally_camera_proc.shared(folder=folder, every_n=self.log_every or 5, quality=self.quality,
                                               exposure=self.exposure, calib_path=self.calib_path,
                                               mirror=getattr(runtime, "course", -1) == 1)
            proc.folder = folder
        except Exception as error:
            self.logger.warning("camera process: start failed (%r)" % (error,))
            return Status.SUCCESS
        self.logger.info("camera process: started (log=%s, exposure=%s)" % (folder, self.exposure))
        return Status.SUCCESS


class EtRallyCameraProcessStop(Behaviour):
    """走行中の画像を残さないとき(スタートの取り込みだけ)に、走り出す前に子プロセスを止める。"""

    def update(self):
        from . import et_rally_camera_proc
        stopped = et_rally_camera_proc.close_shared()
        if stopped is not None:
            self.logger.info("camera process: stopped %s" % (stopped,))
        return Status.SUCCESS


def restore_video_mode(prev):
    """カメラの使い方を元に戻す(ET_RALLY_MARKER のままだと、裏のスレッドが毎フレーム検出を回して重いため)。"""
    video = getattr(runtime, "video", None)
    if video is None or prev is None:
        return
    try:
        video.set_target_interested(prev)
    except Exception:
        pass
