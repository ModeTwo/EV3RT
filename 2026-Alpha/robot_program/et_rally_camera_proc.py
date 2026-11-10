"""ETラリー専用: カメラを別のプロセスで動かす(2026-11-10)。

同じプロセスの中のスレッドでカメラの画像を取り込むと、PyPy では1度に1つの処理しか動けないため、
20msごとの走りの制御が最大200ms止まる(ロボットでの計測: スレッドだと20ms周期が30ms超え 6秒で7〜26回、
別のプロセスなら0回)。そのため、カメラは子プロセス(このファイルを -m で起動)が持ち、
親(走りの制御)とは1行のJSONだけをやり取りする。

子プロセスがすること:
  - 撮る: 640x480 MJPG。撮った直後の time.monotonic() を時刻にする(親と同じ時計)。
  - 保存(save): every_n 枚に1枚を JPEG で folder に保存し、meta.jsonl に {file, t, n} を書く。
  - 位置(detect): スタートで止まっている間、灰色の点とマーカーから位置と向きを出して親へ送る。
親からの命令(標準入力に1行ずつ): "save 1" / "save 0" / "detect 1" / "detect 0" / "quit"。
子からの知らせ(標準出力に1行ずつのJSON): ev=ready / pose / error / stopped。
親が終わって標準入力が閉じたら、子も終わる。
"""
import json
import os
import platform
import sys
import threading
import time
from collections import deque

if platform.python_implementation() == 'CPython':
    sys.path.append('/usr/lib/python3/dist-packages')
elif platform.python_implementation() == 'PyPy':
    sys.path.append('/usr/local/lib/pypy3/dist-packages')


# ---------------- 親(走りの制御)側 ----------------
class CameraProcess:
    """子プロセスを起動して、知らせを裏のスレッドで受け取る。受け取った知らせは poll() で取り出す。"""

    def __init__(self, folder=None, every_n=5, quality=90, exposure=None, calib_path=None, mirror=False, gates=None):
        import subprocess
        from pathlib import Path
        cfg = dict(folder=None if folder is None else str(folder), every_n=int(every_n), quality=int(quality),
                   exposure=exposure, calib_path=None if calib_path is None else str(calib_path), mirror=bool(mirror),
                   gates=gates)
        root = Path(__file__).resolve().parents[1]
        self.proc = subprocess.Popen([child_python(), '-m', 'robot_program.et_rally_camera_proc', json.dumps(cfg)],
                                     cwd=str(root), stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=subprocess.DEVNULL, bufsize=1, universal_newlines=True)
        self.folder = None if folder is None else Path(folder)
        if self.folder is not None:
            self.folder.mkdir(parents=True, exist_ok=True)   # 子がカメラを開く前に、親が位置の記録を書き始めることがある
        self.events = deque()
        self.ready = None
        self.stopped = None
        self.lock = threading.Lock()
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
        for line in self.proc.stdout:
            try:
                ev = json.loads(line)
            except ValueError:
                continue
            with self.lock:
                if ev.get('ev') == 'ready':
                    self.ready = ev
                elif ev.get('ev') == 'stopped':
                    self.stopped = ev
                self.events.append(ev)

    def send(self, command):
        try:
            self.proc.stdin.write(command + '\n')
            self.proc.stdin.flush()
            return True
        except (OSError, ValueError):
            return False

    def poll(self):
        with self.lock:
            out = list(self.events)
            self.events.clear()
        return out

    def alive(self):
        return self.proc.poll() is None

    def close(self, timeout=2.0):
        self.send('quit')
        try:
            self.proc.stdin.close()
        except Exception:
            pass
        try:
            self.proc.wait(timeout)
        except Exception:
            self.proc.kill()
        self.reader.join(0.5)
        return self.stopped


def child_python():
    """子プロセスを動かすPython。ロボットのPyPyはnumpyを使う処理がとても遅い(numpyの読み込み3.2秒など)ので、
    普通のPython(/usr/bin/python3、2026-11-10に python3-numpy・python3-opencv を入れた)があればそちらを使う。
    環境変数 ET_CAMERA_PYTHON で指定もできる。"""
    env = os.environ.get('ET_CAMERA_PYTHON')
    if env:
        return env
    if platform.python_implementation() == 'PyPy' and os.path.exists('/usr/bin/python3'):
        return '/usr/bin/python3'
    return sys.executable


_shared = None


def shared(**kwargs):
    """ETラリーの間に使う1つの子プロセス。なければ kwargs で起動する。"""
    global _shared
    if _shared is None or not _shared.alive():
        _shared = CameraProcess(**kwargs)
    return _shared


def current():
    return _shared if _shared is not None and _shared.alive() else None


def close_shared():
    global _shared
    p, _shared = _shared, None
    return None if p is None else p.close()


# ---------------- 子プロセス側 ----------------
def _child(cfg):
    out_lock = threading.Lock()

    def say(**ev):
        with out_lock:
            sys.stdout.write(json.dumps(ev) + '\n')
            sys.stdout.flush()

    try:
        import cv2
    except ImportError as error:
        say(ev='error', what='cannot import cv2 in %s (%r)' % (sys.executable, error))
        return
    state = dict(save=False, detect=False, track=False, quit=False, mirror=bool(cfg.get('mirror')))

    def commands():
        for line in sys.stdin:
            parts = line.split()
            if not parts:
                continue
            if parts[0] == 'quit':
                break
            if parts[0] in ('save', 'detect', 'track') and len(parts) > 1:
                state[parts[0]] = parts[1] == '1'
                if parts[0] == 'detect' and len(parts) > 2:   # "detect 1 1" = 左コース(見た左右を逆にする)
                    state['mirror'] = parts[2] == '1'
        state['quit'] = True   # 親が終わった(標準入力が閉じた)ときも終わる

    threading.Thread(target=commands, daemon=True).start()
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    def set_exposure():
        if cfg.get('exposure') is not None:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
            cap.set(cv2.CAP_PROP_EXPOSURE, float(cfg['exposure']))
        else:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)

    set_exposure()
    if not cap.isOpened():
        say(ev='error', what='camera not opened')
        return
    folder = cfg.get('folder')
    meta = None
    if folder:
        os.makedirs(folder, exist_ok=True)
        meta = open(os.path.join(folder, 'meta.jsonl'), 'a', encoding='utf-8')
    cam = None
    cnt = dict(n=0, saved=0, detected=0, tracked=0)
    latest = dict(frame=None, t=0.0, n=0)
    lock = threading.Lock()
    ready_ev = threading.Event()

    def reader():
        """2026-11-10: 画像を読み続けて、一番新しい1枚と時刻だけを持つ。位置の計算(1枚0.2〜0.5秒)と同じ流れで読むと、
        カメラにたまった古い画像を読み、読んだ時刻を撮った時刻としてしまっていた(走行中の補正が実機でほとんど入らなかった原因)。
        保存もここで行う(保存する画像の時刻も正しくなる)。"""
        first = True
        while not state['quit']:
            ok, frame = cap.read()
            t = time.monotonic()
            if not ok or frame is None:
                time.sleep(0.005)
                continue
            cnt['n'] += 1
            n = cnt['n']
            if n in (1, 5):
                # 撮り始める前に設定した露出は効かないことがある(ロボットで、露出50の指定なのに真っ白)。
                # 撮り始めてからもう一度設定し、効くまで数枚待ってから「使える」と知らせる。
                set_exposure()
                continue
            if first and n >= 10:
                first = False
                say(ev='ready', t=t, auto=cap.get(cv2.CAP_PROP_AUTO_EXPOSURE), exposure=cap.get(cv2.CAP_PROP_EXPOSURE),
                    gain=cap.get(cv2.CAP_PROP_GAIN), shape=list(frame.shape), brightness=round(float(frame[::8, ::8].mean()), 1))
            if first:
                continue
            with lock:
                latest.update(frame=frame, t=t, n=n)
            ready_ev.set()
            if state['save'] and meta is not None and n % cfg['every_n'] == 0:
                name = 'frame_%05d.jpg' % n
                cv2.imwrite(os.path.join(folder, name), frame, [cv2.IMWRITE_JPEG_QUALITY, cfg['quality']])
                meta.write(json.dumps({'file': name, 't': round(t, 4), 'n': n}) + '\n')
                cnt['saved'] += 1
                if cnt['saved'] % 25 == 0:
                    meta.flush()

    rt = threading.Thread(target=reader, daemon=True)
    rt.start()
    done_n = 0
    while not state['quit']:
        if not (state['track'] or state['detect']):
            time.sleep(0.01)
            continue
        if not ready_ev.wait(0.1):
            continue
        with lock:
            frame, t, n = latest['frame'], latest['t'], latest['n']
        if frame is None or n == done_n:
            time.sleep(0.003)
            continue
        done_n = n
        age = time.monotonic() - t
        if state['track'] and not state['detect']:
            # 走行中の補正(et_rally_camera_correct.py)。見つけたもの(車体の座標)だけを送り、
            # 地図への結びつけは親が撮った時刻の位置で行う。
            try:
                from robot_program import et_rally_start_pose as sp
                if cam is None:
                    cam = sp.GroundCamera(cfg.get('calib_path'), top_s=sp.FAST_TOP_S)
                    sp.set_gate_legs(cfg.get('gates'))
                dets = sp.detect_ground(cam, frame)
                if dets['markers'] or len(dets['dots']) >= 2:
                    say(ev='dets', t=t, n=n, age=round(age, 3),
                        markers=[{'xy': [round(v, 2) for v in m['xy']], 'rot': round(m['rot'], 3)} for m in dets['markers']],
                        dots=[[round(v, 2) for v in d] for d in dets['dots']])
                cnt['tracked'] += 1
            except Exception as error:
                say(ev='error', what='track failed: %r' % (error,))
                state['track'] = False
        if state['detect']:
            try:
                from robot_program import et_rally_start_pose as sp
                if cam is None:
                    cam = sp.GroundCamera(cfg.get('calib_path'), top_s=sp.FAST_TOP_S)
                    sp.set_gate_legs(cfg.get('gates'))
                dets = sp.detect_ground(cam, frame)
                r = sp.fit_pose_pitch(cam, dets, mirror=state['mirror'])
                r0 = sp.fit_pose_search(dets, mirror=state['mirror']) if r is not None else None
                say(ev='pose', t=t, n=n, r=_plain(r), r0=_plain(r0))
                cnt['detected'] += 1
            except Exception as error:   # 較正ファイルがないなど。1回だけ知らせて、位置を出すのはやめる
                say(ev='error', what='detect failed: %r' % (error,))
                state['detect'] = False
    rt.join(1.0)
    cap.release()
    if meta is not None:
        meta.close()
    say(ev='stopped', frames=cnt['n'], saved=cnt['saved'], detected=cnt['detected'], tracked=cnt['tracked'])


def _plain(r):
    if r is None:
        return None
    out = {}
    for k, v in r.items():
        if isinstance(v, (tuple, list)):
            out[k] = [float(x) for x in v]
        elif isinstance(v, (int, float)):
            out[k] = float(v)
    return out


def _check():
    """ロボットで動かす確認(モーターは使わない): 子プロセスで撮影・保存・位置の計算をしている間、
    20msごとの処理が乱れないかを測る。  python -m robot_program.et_rally_camera_proc --check"""
    def loop(sec):
        gaps = []
        s = last = nxt = time.monotonic()
        x = 0
        while time.monotonic() - s < sec:
            for i in range(3000):
                x += i * i
            nxt += 0.02
            d = nxt - time.monotonic()
            if d > 0:
                time.sleep(d)
            now = time.monotonic()
            gaps.append(now - last)
            last = now
        g = sorted(gaps)
        return 'mean %.1fms max %.1fms over30ms %d/%d' % (1000 * sum(g) / len(g), 1000 * g[-1], sum(v > .03 for v in g), len(g))

    from pathlib import Path
    folder = Path(__file__).resolve().parents[1] / 'run_logs' / ('et_camera_check_' + time.strftime('%Y%m%dT%H%M%S'))
    p = CameraProcess(folder=folder, every_n=2, exposure=50)
    t0 = time.monotonic()
    while p.alive() and p.ready is None and time.monotonic() - t0 < 10:
        time.sleep(0.05)
    print('camera ready after %.2fs: %s' % (time.monotonic() - t0, p.ready or p.poll()))
    print('no work  :', loop(3))
    p.send('save 1')
    print('saving   :', loop(6))
    p.send('save 0')
    p.poll()
    p.send('detect 1')
    print('detecting:', loop(4))
    ev = p.poll()
    poses = [e for e in ev if e.get('ev') == 'pose']
    print('pose results %d (with pose %d), errors %s' % (len(poses), sum(1 for e in poses if e.get('r')),
                                                        [e.get('what') for e in ev if e.get('ev') == 'error']))
    if poses and poses[-1].get('r'):
        print('last pose:', [round(v, 1) for v in poses[-1]['r']['pose']])
    p.send('detect 0')
    p.poll()
    p.send('track 1')
    print('tracking :', loop(4))
    ev = p.poll()
    dets = [e for e in ev if e.get('ev') == 'dets']
    ages = sorted(e.get('age', 0.0) for e in dets) or [0.0]
    print('track results %d, image age median %.3fs max %.3fs' % (len(dets), ages[len(ages) // 2], ages[-1]))
    p.send('track 0')
    print('stopped:', p.close(), 'images in', folder)


if __name__ == '__main__':
    if sys.argv[1:] == ['--check']:
        _check()
    else:
        _child(json.loads(sys.argv[1]))
