"""ETラリー専用: 走りながらカメラで位置と向きを直す(2026-11-10、試作。既定では使わない)。

カメラの子プロセス(et_rally_camera_proc.py)が、撮った画像からマーカーと灰色の点を見つけて送ってくる(track)。
ここでは、撮った時刻のコース上の位置(et_rally_course_pose の履歴)を予想として地図に結びつけ、
カメラで分かった位置・向きと、自分が思っている位置・向きの差の一部(gain)を、次の3か所に同じだけ入れる。
  - 走っている区間の自分の位置(et_rally_path_segment.shift_active_position)→ 横のずれを戻すしくみで線へ戻る
  - コース上の自分の位置(et_rally_course_pose.add_correction)→ 次の画像の予想。入れないと同じずれを何度も直す
  - 向き(EtRallyHeading.set_heading_offset に足す)
使わない画像(机上・実機の画像での結果から):
  - きつい円弧の最中(1秒に max_rate_deg 以上回っている): 6周の画像で左右のばらつき15mm・向き1.5°(直進は9mm・0.6°)
  - 結びついたものが2つ未満、当てはめの残差が max_resid_mm 以上
  - 差が max_pos_mm・max_head_deg を超える(取り違えか、まだ直し始めで大きいとき。後者はスタートの取り込みが受け持つ)
撮った時刻は子プロセスの time.monotonic()(親と同じ時計)。実際に撮ったのはそれより少し前なので lag_sec だけ前の位置を使う
(6周の画像で0〜0.03秒、直進の画像で約0.03秒)。
"""

import math
import time

from py_trees.behaviour import Behaviour
from py_trees.common import Status

from .runtime import runtime


def _nd(a):
    return (a + 180.0) % 360.0 - 180.0


class CameraCorrector:
    """走行中の補正の中身(Behaviour から呼ぶ。机上で記録を流して試せるように、木から切り離してある)。"""

    def __init__(self, gain_pos=0.5, gain_head=0.3, lag_sec=0.02, max_rate_deg=60.0, max_resid_mm=6.0,
                 max_pos_mm=40.0, max_head_deg=3.0, min_items=2, mirror=False,
                 at=None, apply=None, logger=None):
        self.gain_pos = float(gain_pos)
        self.gain_head = float(gain_head)
        self.lag_sec = float(lag_sec)
        self.max_rate_deg = float(max_rate_deg)
        self.max_resid_mm = float(max_resid_mm)
        self.max_pos_mm = float(max_pos_mm)
        self.max_head_deg = float(max_head_deg)
        self.min_items = int(min_items)
        self.mirror = bool(mirror)
        self.at = at            # at(t) -> (t, x, y, 向き, 走った距離) か None
        self.apply = apply      # apply(dx, dy, dh): コースの座標で、思っている位置・向きに足す量
        self.logger = logger
        self.counts = dict(seen=0, used=0, rate=0, items=0, resid=0, far=0, nopose=0)
        self.total = [0.0, 0.0, 0.0]

    def process(self, ev):
        """子プロセスからの dets を1つ処理する。使ったら (dx, dy, dh, 差) を返す。"""
        from . import et_rally_start_pose as sp
        self.counts["seen"] += 1
        t = float(ev.get("t", 0.0)) - self.lag_sec
        p, p1, p2 = self.at(t), self.at(t - 0.1), self.at(t + 0.1)
        if p is None or p1 is None or p2 is None or p1[0] >= p2[0]:
            self.counts["nopose"] += 1
            return None
        rate = _nd(p2[3] - p1[3]) / (p2[0] - p1[0])
        if abs(rate) >= self.max_rate_deg:
            self.counts["rate"] += 1
            return None
        dets = {"markers": ev.get("markers", []), "dots": [tuple(d) for d in ev.get("dots", [])]}
        r = sp.fit_pose(dets, nominal=(p[1], p[2], p[3]), mirror=self.mirror, fit_scale=False)
        if r is None or r["n_dots"] + r["n_markers"] < self.min_items:
            self.counts["items"] += 1
            return None
        if r["resid_mm"] >= self.max_resid_mm:
            self.counts["resid"] += 1
            return None
        ex, ey = r["pose"][0] - p[1], r["pose"][1] - p[2]
        eh = _nd(r["pose"][2] - p[3])
        if math.hypot(ex, ey) > self.max_pos_mm or abs(eh) > self.max_head_deg:
            self.counts["far"] += 1
            return None
        dx, dy, dh = self.gain_pos * ex, self.gain_pos * ey, self.gain_head * eh
        if self.apply is not None:
            self.apply(dx, dy, dh)
        self.counts["used"] += 1
        self.total = [self.total[0] + dx, self.total[1] + dy, self.total[2] + dh]
        return dx, dy, dh, (ex, ey, eh, r["n_dots"], r["n_markers"], r["resid_mm"])


def apply_to_robot(dx, dy, dh):
    """コースの座標で、思っている位置を (dx, dy)、向きを dh(左回りが+)だけ動かす。"""
    from . import et_rally_course_pose as cp
    from .behaviours import et_rally_path_segment as ps
    cp.add_correction(dx, dy, 0.0)
    ps.shift_active_position(dx, dy)
    gyro = runtime.gyro_sensor
    if dh and hasattr(gyro, "set_heading_offset"):
        # スタートの取り込みと同じ向き: コースの向きが +dh になるように、ジャイロの向きに course × dh を足す
        gyro.set_heading_offset(gyro.heading_offset + getattr(runtime, "course", -1) * dh)


class EtRallyCameraCorrect(Behaviour):
    """走行(execute)と並べて置く。毎tick、子プロセスから届いた dets を処理する。終わらない(並びの相手が終わったら止まる)。"""

    def __init__(self, name, gain_pos=0.5, gain_head=0.3, lag_sec=0.02):
        super().__init__(name)
        self.gain_pos, self.gain_head, self.lag_sec = gain_pos, gain_head, lag_sec
        self.proc = None
        self.corrector = None
        self.last_log = 0.0

    def initialise(self):
        from . import et_rally_camera_proc
        from . import et_rally_course_pose as cp
        self.proc = et_rally_camera_proc.current()
        self.corrector = CameraCorrector(self.gain_pos, self.gain_head, self.lag_sec,
                                         mirror=getattr(runtime, "course", -1) == 1, at=cp.at, apply=apply_to_robot,
                                         logger=self.logger)
        if self.proc is None:
            self.logger.warning("camera correct: camera process not running, no correction")
            return
        self.proc.poll()
        self.proc.send("track 1")
        self.logger.info("camera correct: started gain_pos=%.2f gain_head=%.2f lag=%.3fs" % (
            self.gain_pos, self.gain_head, self.lag_sec))

    def update(self):
        if self.proc is None:
            return Status.RUNNING
        for ev in self.proc.poll():
            if ev.get("ev") != "dets":
                continue
            res = self.corrector.process(ev)
            if res is not None:
                dx, dy, dh, (ex, ey, eh, nd, nm, resid) = res
                self.logger.info("camera correct: diff (%+.1f,%+.1f mm %+.2f deg) items D%d M%d resid %.1f -> applied (%+.1f,%+.1f %+.2f) total (%+.1f,%+.1f %+.2f)" % (
                    ex, ey, eh, nd, nm, resid, dx, dy, dh, *self.corrector.total))
        return Status.RUNNING

    def terminate(self, new_status):
        if self.proc is not None:
            self.proc.send("track 0")
        if self.corrector is not None:
            self.logger.info("camera correct: finished %s total (%+.1f,%+.1f mm %+.2f deg)" % (
                self.corrector.counts, *self.corrector.total))
