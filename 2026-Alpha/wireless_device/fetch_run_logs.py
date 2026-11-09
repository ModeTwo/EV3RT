"""ラズパイの走行ログ(run_logs/)を PC へ取り込み、届いたことを確かめてからラズパイ側を消す(2026-11-09)。

usage:
  python fetch_run_logs.py            今あるログを1回取り込む
  python fetch_run_logs.py --watch    ずっと動かしておき、20秒おきに新しいログを取り込む(つながらない間は待つ)

取り込むもの: run_*.log と et_camera_*/ の中身。保存先は既定で ~/Downloads/et_run_logs。
書き込み中のログを消さないよう、更新から --min-age 秒たったファイルだけを扱う。
ラズパイ側を消すのは、PC に届いたファイルの SHA-256 がラズパイ側と一致したものだけ(--keep で消さない)。
パスワードなしで ssh できる必要がある(鍵の登録は初回に1度だけ手で行う)。
"""
import argparse
import hashlib
import os
import shlex
import subprocess
import sys
import tarfile
import time
from pathlib import Path


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def wanted(rel):
    return (rel.startswith('run_') and rel.endswith('.log') and '/' not in rel) or rel.startswith('et_camera_')


class Remote:
    def __init__(self, args):
        self.ssh = shlex.split(args.ssh) + ['-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5', args.host] if args.ssh == 'ssh' else shlex.split(args.ssh) + [args.host]
        self.dir = args.remote_dir

    def run(self, command, data=None, stdout=subprocess.PIPE):
        return subprocess.run(self.ssh + ['cd %s && %s' % (self.dir, command)], input=data, stdout=stdout, stderr=subprocess.PIPE)

    def listing(self, min_age):
        r = self.run("find . -type f -mmin +%.3f -exec sha256sum {} +" % (min_age / 60.0))
        if r.returncode != 0:
            raise ConnectionError(r.stderr.decode(errors='replace').strip())
        out = {}
        for line in r.stdout.decode().splitlines():
            digest, name = line.split(None, 1)
            name = name.lstrip('*')
            rel = name[2:] if name.startswith('./') else name
            if wanted(rel):
                out[rel] = digest
        return out


def fetch_once(remote, dest, min_age, keep):
    files = remote.listing(min_age)
    if not files:
        return 0
    names = sorted(files)
    tmp = dest / '.incoming.tar'
    with open(tmp, 'wb') as f:
        r = remote.run('tar cf - -T -', data='\n'.join(names).encode() + b'\n', stdout=f)
    if r.returncode != 0:
        tmp.unlink(missing_ok=True)
        raise ConnectionError(r.stderr.decode(errors='replace').strip())
    with tarfile.open(tmp) as t:
        t.extractall(dest, filter='data')
    tmp.unlink()
    good = [n for n in names if (dest / n).exists() and sha256(dest / n) == files[n]]
    bad = sorted(set(names) - set(good))
    if bad:
        print('NOT VERIFIED (kept on robot): %s' % ', '.join(bad), flush=True)
    for n in good:
        if n.endswith('.log'):
            print('fetched %s' % (dest / n), flush=True)
    cam = sum(1 for n in good if not n.endswith('.log'))
    if cam:
        print('fetched %d camera files' % cam, flush=True)
    if good and not keep:
        r = remote.run("xargs -d '\\n' rm -f -- && find . -mindepth 1 -type d -name 'et_camera_*' -empty -delete", data='\n'.join(good).encode() + b'\n')
        if r.returncode != 0:
            print('delete failed: %s' % r.stderr.decode(errors='replace').strip(), flush=True)
    return len(good)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default='etrobo-raspi.local')
    ap.add_argument('--remote-dir', default='EV3RT/2026-Alpha/run_logs')
    ap.add_argument('--dest', default=str(Path.home() / 'Downloads' / 'et_run_logs'))
    ap.add_argument('--watch', action='store_true')
    ap.add_argument('--interval', type=float, default=20.0)
    ap.add_argument('--min-age', type=int, default=10, help='更新からこの秒数たったファイルだけ扱う')
    ap.add_argument('--keep', action='store_true', help='ラズパイ側を消さない')
    ap.add_argument('--ssh', default='ssh', help=argparse.SUPPRESS)
    args = ap.parse_args()
    dest = Path(args.dest)
    dest.mkdir(parents=True, exist_ok=True)
    remote = Remote(args)
    last_error = None
    while True:
        try:
            n = fetch_once(remote, dest, args.min_age, args.keep)
            if last_error is not None or (not args.watch and n == 0):
                print('connected; %d files' % n, flush=True)
            last_error = None
        except (ConnectionError, OSError) as error:
            if str(error) != last_error:
                print('cannot reach robot: %s' % error, flush=True)
            last_error = str(error)
            if not args.watch:
                return 1
        if not args.watch:
            return 0
        time.sleep(args.interval)


if __name__ == '__main__':
    sys.exit(main())
