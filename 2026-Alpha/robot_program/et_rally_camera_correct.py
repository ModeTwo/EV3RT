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
11/10 夜: 子プロセスで画像を読む係を分けたあとは、本当の遅れはほぼ0(約0.005秒)。0.02秒のままだと、補正のたびに
「実際は前に約4mm」と見えて、思っている位置を前へずらし続けていた → 0 にした。
11/10 夜(カメラを固定し直し、床での下向きの補正つき)の6周の画像を流した机上の確認: 1枚ごとのばらつきは2.6〜2.8mm・0.33°。
カメラは6周で約14mm前へずれていく様子を捉えていた(ゴールのログに出ないずれ約17mm)。強さ0.3/0.2・残差8mm未満で、
最後に入る補正が+17.2mm(0.2/0.1・6mmでは+10.4mm、0.5/0.3では+19.7mmで行き来が大きい)。
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

    # 2026-11-11: far_confirm 枚が続けて(FAR_WINDOW_SEC 秒以内)互いに FAR_AGREE_MM 以内の同じずれに見えたら、
    # max_pos_mm を超えていても本当のずれとして受け入れる(FAR_ACCEPT_MAX_MM までは)。切り返しの多い配置37で、ずれが
    # 40mmを超えたあと補正が止まり(1回の走りで88枚を捨てた)、たまる一方になっていたため。
    FAR_AGREE_MM = 15.0
    FAR_WINDOW_SEC = 2.0
    FAR_ACCEPT_MAX_MM = 100.0

    def __init__(self, gain_pos=0.3, gain_head=0.2, lag_sec=0.0, max_rate_deg=60.0, max_resid_mm=8.0,
                 max_pos_mm=40.0, max_head_deg=3.0, min_items=2, mirror=False,
                 at=None, apply=None, logger=None, far_confirm=0):
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
        self.far_confirm = int(far_confirm or 0)
        self.recent_far = []
        self.counts = dict(seen=0, used=0, rate=0, items=0, resid=0, far=0, far_ok=0, nopose=0, not_line=0)
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
        far_ok = False
        if math.hypot(ex, ey) > self.max_pos_mm or abs(eh) > self.max_head_deg:
            if not self._confirm_far(t, ex, ey, eh):
                self.counts["far"] += 1
                return None
            far_ok = True
        else:
            self.recent_far = []
        dx, dy, dh = self.gain_pos * ex, self.gain_pos * ey, self.gain_head * eh
        if self.apply is not None and self.apply(dx, dy, dh) is False:
            self.counts["not_line"] += 1
            return None
        self.counts["used"] += 1
        if far_ok:
            self.counts["far_ok"] += 1
            self.recent_far = []
        self.total = [self.total[0] + dx, self.total[1] + dy, self.total[2] + dh]
        return dx, dy, dh, (ex, ey, eh, r["n_dots"], r["n_markers"], r["resid_mm"])


def _confirm_far_impl(self, t, ex, ey, eh):
    """大きいずれ (ex, ey) が、直前の大きいずれと続けて同じなら True(受け入れる)。"""
    if self.far_confirm <= 0 or abs(eh) > self.max_head_deg or math.hypot(ex, ey) > self.FAR_ACCEPT_MAX_MM:
        return False
    self.recent_far = [f for f in self.recent_far if t - f[0] <= self.FAR_WINDOW_SEC] + [(t, ex, ey)]
    if len(self.recent_far) < self.far_confirm:
        return False
    last = self.recent_far[-self.far_confirm:]
    mx = sum(f[1] for f in last) / len(last)
    my = sum(f[2] for f in last) / len(last)
    return all(math.hypot(f[1] - mx, f[2] - my) <= self.FAR_AGREE_MM for f in last)


CameraCorrector._confirm_far = _confirm_far_impl


def apply_to_robot(dx, dy, dh):
    """コースの座標で、思っている位置を (dx, dy)、向きを dh(左回りが+)だけ動かす。入れたら True。
    2026-11-10: 直進の区間を走っているときだけ入れる。円弧の中で区間の進んだ距離をずらすと、目標の向きが飛ぶ
    (実機: 円弧を0.9倍に遅くしたら円弧の中でも補正が入り、長さ63mmの円弧で向きのずれ最大31°になって止めた)。"""
    from . import et_rally_course_pose as cp
    from .behaviours import et_rally_path_segment as ps
    if ps.active_kind() != "line":
        return False
    cp.add_correction(dx, dy, 0.0)
    cp.shift_history(dx, dy, dh)   # 向きはジャイロの向きに入れるので、これから先の位置は自然にそろう
    ps.shift_active_position(dx, dy)
    gyro = runtime.gyro_sensor
    if dh and hasattr(gyro, "set_heading_offset"):
        # スタートの取り込みと同じ向き: コースの向きが +dh になるように、ジャイロの向きに course × dh を足す
        gyro.set_heading_offset(gyro.heading_offset + getattr(runtime, "course", -1) * dh)
    return True


class EtRallyCameraCorrect(Behaviour):
    """走行(execute)と並べて置く。毎tick、子プロセスから届いた dets を処理する。終わらない(並びの相手が終わったら止まる)。"""

    def __init__(self, name, gain_pos=0.5, gain_head=0.3, lag_sec=0.0, far_confirm=0):
        super().__init__(name)
        self.gain_pos, self.gain_head, self.lag_sec = gain_pos, gain_head, lag_sec
        self.far_confirm = int(far_confirm or 0)
        self.proc = None
        self.corrector = None
        self.ages = []
        self.last_log = 0.0

    def initialise(self):
        from . import et_rally_camera_proc
        from . import et_rally_course_pose as cp
        self.proc = et_rally_camera_proc.current()
        self.corrector = CameraCorrector(self.gain_pos, self.gain_head, self.lag_sec,
                                         mirror=getattr(runtime, "course", -1) == 1, at=cp.at, apply=apply_to_robot,
                                         logger=self.logger, far_confirm=self.far_confirm)
        if self.proc is None:
            self.logger.warning("camera correct: camera process not running, no correction")
            return
        self.proc.poll()
        # 2026-11-11: スタート位置の取り込みで当てはめた下向きを、走行中の補正にも使う(点が見えて当てはめたときだけ)。
        # 右seedの画像で、較正ファイルの下向きのままだと残差が大きく 28/111 枚を捨てていたのが、当てはめた値なら 2/110 枚。
        if getattr(runtime, "et_rally_camera_pitch_from_start", True):
            from . import et_rally_start_pose as sp
            m = sp.measured_start_pose()
            if m and m.get("n_dots", 0) >= 1 and abs(m.get("pitch_deg", 0.0)) <= 1.5 and m.get("pitch_deg"):
                self.proc.send("pitch %.3f" % m["pitch_deg"])
                self.logger.info("camera correct: using the start pose pitch %+.2f deg while driving" % m["pitch_deg"])
        self.proc.send("track 1")
        self.logger.info("camera correct: started gain_pos=%.2f gain_head=%.2f lag=%.3fs far_confirm=%d" % (
            self.gain_pos, self.gain_head, self.lag_sec, self.far_confirm))

    def update(self):
        if self.proc is None:
            return Status.RUNNING
        for ev in self.proc.poll():
            if ev.get("ev") != "dets":
                continue
            res = self.corrector.process(ev)
            self.ages.append(float(ev.get("age", 0.0)))
            if res is not None:
                dx, dy, dh, (ex, ey, eh, nd, nm, resid) = res
                self.logger.info("camera correct: diff (%+.1f,%+.1f mm %+.2f deg) items D%d M%d resid %.1f -> applied (%+.1f,%+.1f %+.2f) total (%+.1f,%+.1f %+.2f)" % (
                    ex, ey, eh, nd, nm, resid, dx, dy, dh, *self.corrector.total))
        return Status.RUNNING

    def terminate(self, new_status):
        if self.proc is not None:
            self.proc.send("track 0")
        if self.corrector is not None:
            ages = sorted(self.ages) or [0.0]
            self.logger.info("camera correct: finished %s total (%+.1f,%+.1f mm %+.2f deg) image age median %.3fs max %.3fs" % (
                self.corrector.counts, *self.corrector.total, ages[len(ages) // 2], ages[-1]))
