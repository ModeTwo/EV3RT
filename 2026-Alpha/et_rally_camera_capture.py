"""2026-11-03: ETラリーのカメラ較正用の撮影ツール(ロボットのラズパイで動かす)。

撮影の形式は走行中と同じ 640x480 MJPG(py_etrobo_util/video.py の LINE/BOTTLE/ET_RALLY_MARKER と同じ)。
レンズの性質は解像度とピントで変わるため、オートフォーカスは既定で止める(--focus で固定値)。

撮るたびにチェッカーボード(内側の角 9x6)を探し、見つかったか・ぶれ具合(数字が大きいほどくっきり)を表示する。
画面がなくても、ターミナルの表示だけで撮れたかがわかる。画像は加工せずにPNGで保存し、情報は meta.jsonl に残す。

使い方:
  python3 et_rally_camera_capture.py --tag lens            # Enterで1枚、qで終了
  python3 et_rally_camera_capture.py --tag lens --interval 3   # 3秒ごとに自動で撮る(qで終了)
  python3 et_rally_camera_capture.py --tag jig --count 1   # 較正台で1枚撮って終わる
  DISPLAYがあれば(XLaunchなど)、映像と見つかった角を表示する(--no-window で出さない)。
保存先: camera_calib/<日時>_<tag>/
"""
import sys
import platform

if platform.python_implementation() == 'CPython':
    sys.path.append('/usr/lib/python3/dist-packages')
elif platform.python_implementation() == 'PyPy':
    sys.path.append('/usr/local/lib/pypy3/dist-packages')

import argparse
import json
import os
import select
import threading
import time
from datetime import datetime

import cv2
import numpy as np

BOARD_INNER = (9, 6)       # チェッカーボードの内側の角(横, 縦)。A4・25mm・10x7マス
WIDTH, HEIGHT, FPS, FOURCC = 640, 480, 30, 'MJPG'
GRID = 3                   # 画面を3x3に分けて、ボードがまだ写っていない場所を知らせる


def open_camera(args):
    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*FOURCC))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
    if args.autofocus:
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)
    else:
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        if args.focus is not None:
            cap.set(cv2.CAP_PROP_FOCUS, args.focus)
    if not cap.isOpened():
        print('ERROR: カメラを開けませんでした')
        sys.exit(1)
    props = {
        'width': int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), 'height': int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        'fps': cap.get(cv2.CAP_PROP_FPS), 'autofocus': cap.get(cv2.CAP_PROP_AUTOFOCUS),
        'focus': cap.get(cv2.CAP_PROP_FOCUS), 'auto_exposure': cap.get(cv2.CAP_PROP_AUTO_EXPOSURE),
        'exposure': cap.get(cv2.CAP_PROP_EXPOSURE),
    }
    return cap, props


class Grabber(threading.Thread):
    """カメラから読み続けて、いつも最新の1枚を持っておく(古い画像がたまって遅れて撮れるのを防ぐ)。"""

    def __init__(self, cap):
        super().__init__(daemon=True)
        self.cap = cap
        self.lock = threading.Lock()
        self.frame = None
        self.stamp = 0.0
        self.running = True

    def run(self):
        while self.running:
            ok, frame = self.cap.read()
            if ok and frame is not None:
                with self.lock:
                    self.frame = frame
                    self.stamp = time.time()

    def latest(self):
        with self.lock:
            return (None if self.frame is None else self.frame.copy()), self.stamp


def find_board(gray):
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
    found, corners = cv2.findChessboardCorners(gray, BOARD_INNER, flags)
    if not found:
        return None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
    return cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), crit)


def grid_cells(corners):
    cells = set()
    for x, y in corners.reshape(-1, 2):
        cells.add((min(GRID - 1, int(y * GRID / HEIGHT)), min(GRID - 1, int(x * GRID / WIDTH))))
    return cells


SIZE_FAR = 0.25            # ボードの写る大きさ(画面の幅・高さに対する割合の大きい方)がこれ未満なら「遠い」
SIZE_NEAR = 0.85           # これを超えたら「近い」(端が切れやすい)
EDGE_PX = 8                # 角が画面の端からこれ以内なら「はみ出しそう」
SHARP_MIN = 100.0          # くっきり度がこれ未満なら「ぶれ・ぼやけ」


def board_size(corners):
    pts = corners.reshape(-1, 2)
    return max((pts[:, 0].max() - pts[:, 0].min()) / WIDTH, (pts[:, 1].max() - pts[:, 1].min()) / HEIGHT)


def advice(corners, sharp):
    """撮った1枚へのひとこと。問題がなければ空のリスト。"""
    out = []
    if corners is None:
        out.append('ボード全体が写っているか、傾けすぎていないか、ぶれていないかを確認')
    else:
        size = board_size(corners)
        pts = corners.reshape(-1, 2)
        if size < SIZE_FAR:
            out.append('遠い: もっと近づける(今は画面の%.0f%%、%.0f%%以上に)' % (size * 100, SIZE_FAR * 100))
        elif size > SIZE_NEAR:
            out.append('近い: 少し離す(今は画面の%.0f%%)' % (size * 100))
        if (pts[:, 0].min() < EDGE_PX or pts[:, 1].min() < EDGE_PX or
                pts[:, 0].max() > WIDTH - EDGE_PX or pts[:, 1].max() > HEIGHT - EDGE_PX):
            out.append('はみ出しそう: ボードの外側の白い縁まで画面に入れる')
    if sharp < SHARP_MIN:
        out.append('ぶれ・ぼやけ: 止めて撮る(くっきり度%.0f、%.0f以上に)' % (sharp, SHARP_MIN))
    return out


# レンズの較正で「十分」とみなす条件(OKの画像だけで数える)
NEED_GOOD = 15             # OKの枚数
NEED_TILT = 3              # 左右に傾けた枚数、上下に傾けた枚数(それぞれ)
TILT_MIN = 0.12            # 向かい合う辺の長さが12%以上違えば「傾けた」とみなす
NEED_SIZE_SPAN = 0.20      # 大きさの最大と最小の差(近くと遠くの両方で撮れているか)


def tilts(corners):
    """左右の傾き(左の列と右の列の長さの差)、上下の傾き(上の行と下の行の長さの差)を割合で返す。"""
    g = corners.reshape(BOARD_INNER[1], BOARD_INNER[0], 2)
    col_l = np.linalg.norm(g[-1, 0] - g[0, 0])
    col_r = np.linalg.norm(g[-1, -1] - g[0, -1])
    row_t = np.linalg.norm(g[0, -1] - g[0, 0])
    row_b = np.linalg.norm(g[-1, -1] - g[-1, 0])
    return abs(col_l - col_r) / max(1e-6, (col_l + col_r) / 2), abs(row_t - row_b) / max(1e-6, (row_t + row_b) / 2)


class Progress:
    def __init__(self):
        self.good = 0
        self.cells = set()
        self.tilt_a = 0
        self.tilt_b = 0
        self.sizes = []

    def add(self, corners):
        self.good += 1
        self.cells |= grid_cells(corners)
        ta, tb = tilts(corners)
        # 画面の中で縦横どちらに傾いて見えるかは、ボードの置き方で入れ替わるので、2種類を別に数える
        self.tilt_a += ta >= TILT_MIN
        self.tilt_b += tb >= TILT_MIN
        self.sizes.append(board_size(corners))

    def lines(self):
        span = (max(self.sizes) - min(self.sizes)) if self.sizes else 0.0
        items = [
            ('OKの枚数', '%d/%d' % (self.good, NEED_GOOD), self.good >= NEED_GOOD),
            ('写した場所', '%d/9' % len(self.cells), len(self.cells) >= GRID * GRID),
            ('傾き(片方の向き)', '%d/%d' % (self.tilt_a, NEED_TILT), self.tilt_a >= NEED_TILT),
            ('傾き(もう片方の向き)', '%d/%d' % (self.tilt_b, NEED_TILT), self.tilt_b >= NEED_TILT),
            ('近くと遠く', '差%.0f%%/%.0f%%' % (span * 100, NEED_SIZE_SPAN * 100), span >= NEED_SIZE_SPAN),
        ]
        return items, all(ok for _, _, ok in items)


def stdin_ready(timeout):
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    return bool(r)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tag', default='lens', help='保存フォルダの名前に付ける(lens / jig など)')
    ap.add_argument('--interval', type=float, default=0.0, help='0より大きいと、その秒数ごとに自動で撮る')
    ap.add_argument('--count', type=int, default=0, help='この枚数を撮ったら終わる(0は無制限)')
    ap.add_argument('--device', type=int, default=0)
    ap.add_argument('--autofocus', action='store_true', help='オートフォーカスを使う(既定は止める)')
    ap.add_argument('--focus', type=float, default=None, help='ピントの固定値(カメラによって範囲が違う)')
    ap.add_argument('--no-window', action='store_true')
    ap.add_argument('--out', default='camera_calib')
    args = ap.parse_args()

    out_dir = os.path.join(args.out, datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + args.tag)
    os.makedirs(out_dir, exist_ok=True)
    cap, props = open_camera(args)
    print('カメラ:', props)
    with open(os.path.join(out_dir, 'camera_props.json'), 'w') as f:
        json.dump(dict(props, board_inner=BOARD_INNER, tag=args.tag), f, indent=1)

    grabber = Grabber(cap)
    grabber.start()
    t0 = time.time()
    while grabber.latest()[0] is None and time.time() - t0 < 5.0:
        time.sleep(0.05)
    time.sleep(1.0)  # 露出が落ち着くのを待つ

    window = (not args.no_window) and bool(os.environ.get('DISPLAY'))
    print('保存先:', out_dir)
    if args.interval > 0:
        print('%.1f秒ごとに自動で撮ります。終わるときは q と Enter' % args.interval)
    else:
        print('Enter で1枚撮る。q と Enter で終わる' + ('(映像の画面では s で撮る、q で終わる)' if window else ''))

    progress = Progress()
    announced = False
    saved = 0
    next_auto = time.time() + args.interval
    meta = open(os.path.join(out_dir, 'meta.jsonl'), 'a')
    try:
        while True:
            shoot = False
            quit_ = False
            if window:
                frame, _ = grabber.latest()
                view = frame.copy()
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                small = cv2.resize(gray, None, fx=0.5, fy=0.5)
                ok, c = cv2.findChessboardCorners(small, BOARD_INNER, cv2.CALIB_CB_FAST_CHECK)
                if ok:
                    cv2.drawChessboardCorners(view, BOARD_INNER, c * 2.0, ok)
                    size = board_size(c * 2.0)
                    hint = 'FAR' if size < SIZE_FAR else ('NEAR' if size > SIZE_NEAR else 'OK')
                    cv2.putText(view, '%s %.0f%%' % (hint, size * 100), (8, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (0, 200, 0) if hint == 'OK' else (0, 140, 255), 2)
                else:
                    cv2.putText(view, 'NO BOARD', (8, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                for r_, c_ in progress.cells:
                    x0, y0 = c_ * WIDTH // GRID, r_ * HEIGHT // GRID
                    cv2.rectangle(view, (x0 + 2, y0 + 2), (x0 + WIDTH // GRID - 2, y0 + HEIGHT // GRID - 2), (0, 160, 0), 1)
                cv2.putText(view, 'saved %d' % saved, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.imshow('capture', view)
                k = cv2.waitKey(30) & 0xFF
                shoot = k == ord('s')
                quit_ = k == ord('q')
            if stdin_ready(0.0 if window else 0.05):
                line = sys.stdin.readline().strip().lower()
                if line == 'q':
                    quit_ = True
                elif args.interval <= 0:
                    shoot = True
            if args.interval > 0:
                remain = next_auto - time.time()
                if remain <= 0:
                    shoot = True
                    next_auto = time.time() + args.interval
            if quit_:
                break
            if not shoot:
                continue

            frame, stamp = grabber.latest()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            sharp = float(cv2.Laplacian(gray, cv2.CV_64F).var())
            corners = find_board(gray)
            saved += 1
            name = '%s_%03d.png' % (args.tag, saved)
            cv2.imwrite(os.path.join(out_dir, name), frame)
            rec = {'file': name, 'time': stamp, 'sharpness': round(sharp, 1), 'found': corners is not None}
            if corners is not None:
                pts = corners.reshape(-1, 2)
                rec['corners'] = [[round(float(x), 2), round(float(y), 2)] for x, y in pts]
                rec['bbox'] = [round(float(pts[:, 0].min()), 1), round(float(pts[:, 1].min()), 1),
                               round(float(pts[:, 0].max()), 1), round(float(pts[:, 1].max()), 1)]
                rec['size'] = round(float(board_size(corners)), 3)
            rec['advice'] = advice(corners, sharp)
            meta.write(json.dumps(rec) + '\n')
            meta.flush()
            msg = '[%03d] %s  ボード: %s  くっきり度: %.0f' % (saved, name, '見つかった' if corners is not None else '見つからない', sharp)
            if corners is not None:
                b = rec['bbox']
                msg += '  大きさ: 画面の%.0f%%  位置: 横%.0f〜%.0f 縦%.0f〜%.0f' % (board_size(corners) * 100, b[0], b[2], b[1], b[3])
            print(msg)
            notes = rec['advice']
            for n in notes:
                print('      → ' + n)
            if not notes:
                print('      → OK')
            if args.tag == 'lens':
                if corners is not None and not notes:
                    progress.add(corners)
                missing = [(r_, c_) for r_ in range(GRID) for c_ in range(GRID) if (r_, c_) not in progress.cells]
                names = {0: '上', 1: '真ん中', 2: '下'}, {0: '左', 1: '真ん中', 2: '右'}
                if missing:
                    print('      まだボードが写っていない場所: ' + ', '.join(names[0][r_] + names[1][c_] for r_, c_ in missing))
                items, done = progress.lines()
                print('      集まり具合: ' + '  '.join('%s %s%s' % (k, v, '' if ok else '(足りない)') for k, v, ok in items))
                if done and not announced:
                    announced = True
                    print('')
                    print('==== 十分集まりました。q と Enter で終わってください(続けて撮っても大丈夫です) ====')
                    print('')
            if args.count and saved >= args.count:
                break
    finally:
        grabber.running = False
        meta.close()
        grabber.join(timeout=1.0)
        cap.release()
        if window:
            cv2.destroyAllWindows()
    print('%d枚を %s に保存しました' % (saved, out_dir))


if __name__ == '__main__':
    main()
