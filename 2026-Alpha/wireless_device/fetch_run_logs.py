"""ラズパイの走行ログ(run_logs/)を PC へ取り込み、届いたことを確かめてからラズパイ側を消す(2026-11-09)。

usage:
  python fetch_run_logs.py            今あるログを1回取り込む
  python fetch_run_logs.py --watch    ずっと動かしておき、20秒おきに新しいログを取り込む(つながらない間は待つ)

取り込むもの: run_*.log と et_camera_*/ の中身。保存先は既定で ~/Downloads/et_run_logs。
書き込み中のログを消さないよう、更新から --min-age 秒たったファイルだけを扱う。
ラズパイ側を消すのは、PC に届いたファイルの SHA-256 がラズパイ側と一致したものだけ(--keep で消さない)。
et_camera_*/ の中は、そのフォルダのどのファイルも --dir-idle 秒(既定600)更新されていないときだけ消す
(撮影中は記録ファイル meta.jsonl を開いたままなので、途中で消すとその後の記録が失われる。2026-11-10)。
PC に同じ中身のファイルがすでにあれば、取り込み直さない。
パスワードなしで ssh できる必要がある(鍵の登録は初回に1度だけ手で行う)。
"""
import argparse
import hashlib
import json
import os
import re
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


CACHE = Path.home() / '.et_run_logs_robot.json'


def _load_cache(path):
    try:
        return json.load(open(path, encoding='utf-8'))
    except (OSError, ValueError):
        return {}


def _neighbor_ips(mac):
    """PCの近くの機器一覧(ARP表・IPv6の近隣表)から、MACアドレスが mac の機器のIPアドレス(Windows)。"""
    if not mac:
        return []
    want = mac.lower().replace(':', '-')
    out = []
    for cmd in (['arp', '-a'], ['netsh', 'interface', 'ipv6', 'show', 'neighbors']):
        try:
            text = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, timeout=5).stdout.decode(errors='replace')
        except (OSError, subprocess.SubprocessError):
            continue
        for line in text.splitlines():
            if want in line.lower():
                ip = line.split()[0]
                if not ip.lower().startswith('fe80'):
                    out.append(ip)
    return out


class Remote:
    """つながる相手を順に試す(2026-11-10: ロボットの名前 etrobo-raspi / etrobo-raspi.local は、どちらが引けるかが
    時間によって変わり、両方とも引けないこともあった)。
    候補: 前回つながった相手 → --host(カンマ区切り) → 前回つながったときに記録したIPアドレス
          → PCの近くの機器一覧で、記録したMACアドレスの機器(IPアドレスが変わったとき)。
    つながったら、ロボットのIPアドレスとMACアドレスを ~/.et_run_logs_robot.json に記録する。"""

    def __init__(self, args):
        self.hosts = [h.strip() for h in args.host.split(',') if h.strip()]
        self.user = self.hosts[0].split('@', 1)[0] + '@' if '@' in self.hosts[0] else ''
        self.base = shlex.split(args.ssh) + (['-o', 'BatchMode=yes', '-o', 'ConnectTimeout=4'] if args.ssh == 'ssh' else [])
        self.dir = args.remote_dir
        self.cache_path = Path(os.environ.get('ET_FETCH_CACHE') or CACHE)
        self.cache = _load_cache(self.cache_path)
        self.cur = None

    def candidates(self):
        out = []
        for h in [self.cache.get('last')] + self.hosts + [self.user + ip for ip in self.cache.get('ips', [])] \
                + [self.user + ip for ip in _neighbor_ips(self.cache.get('mac'))]:
            if h and h not in out:
                out.append(h)
        return out

    def run(self, command, data=None, stdout=subprocess.PIPE, host=None):
        return subprocess.run(self.base + [host or self.cur, 'cd %s && %s' % (self.dir, command)], input=data, stdout=stdout,
                              stderr=subprocess.PIPE)

    def listing(self, min_age):
        errors = []
        for host in self.candidates():
            try:
                result = self._listing(min_age, host)
            except ConnectionError as error:
                errors.append('%s: %s' % (host, str(error).splitlines()[-1] if str(error) else 'failed'))
                continue
            if self.cur != host:
                print('connected via %s' % host, flush=True)
            self.cur = host
            return result
        raise ConnectionError(' / '.join(errors))

    def _remember(self, host, info):
        ips = [w for w in info.split() if re.fullmatch(r'[0-9.]+|[0-9a-fA-F:]+', w) and ':' not in w[:1]
               and not w.lower().startswith('fe80')]
        macs = re.findall(r'(?:[0-9a-f]{2}:){5}[0-9a-f]{2}', info.lower())
        new = dict(self.cache, last=host, ips=[ip for ip in ips if not re.fullmatch(r'(?:[0-9a-f]{2}:){5}[0-9a-f]{2}', ip.lower())],
                   updated=time.strftime('%Y-%m-%d %H:%M:%S'))
        if macs:
            new['mac'] = macs[0]
        if new != self.cache:
            self.cache = new
            try:
                json.dump(new, open(self.cache_path, 'w', encoding='utf-8'), indent=1)
            except OSError:
                pass

    def _listing(self, min_age, host):
        """(更新から min_age 秒たったファイル {名前: SHA-256}, 各ファイルの更新からの秒数 {名前: 秒})"""
        r = self.run("find . -type f -mmin +%.3f -exec sha256sum {} + ; echo ===; find . -type f -printf '%%T@ %%p\\n'; echo ===; date +%%s.%%N;"
                     " echo ===; hostname -I; cat /sys/class/net/wlan0/address 2>/dev/null" % (min_age / 60.0), host=host)
        if r.returncode != 0:
            raise ConnectionError(r.stderr.decode(errors='replace').strip())
        parts = r.stdout.decode().split('===\n')
        sums, times, now = parts[:3]
        if len(parts) > 3:
            self._remember(host, parts[3])
        now = float(now.strip())
        out, age = {}, {}
        for line in sums.splitlines():
            digest, name = line.split(None, 1)
            rel = _rel(name.lstrip('*'))
            if wanted(rel):
                out[rel] = digest
        for line in times.splitlines():
            t, name = line.split(None, 1)
            age[_rel(name)] = now - float(t)
        return out, age


def _rel(name):
    return name[2:] if name.startswith('./') else name


def deletable(names, age, dir_idle):
    """消してよいもの。run_*.log はすぐ、et_camera_*/ の中はそのフォルダ全体が dir_idle 秒更新されていないときだけ。"""
    busy = set()
    for rel, a in age.items():
        if '/' in rel and a < dir_idle:
            busy.add(rel.rsplit('/', 1)[0])
    return [n for n in names if '/' not in n or n.rsplit('/', 1)[0] not in busy]


def fetch_once(remote, dest, min_age, keep, dir_idle=600.0):
    files, age = remote.listing(min_age)
    have = [n for n in files if (dest / n).exists() and sha256(dest / n) == files[n]]
    names = sorted(set(files) - set(have))
    if not names:
        _delete(remote, [] if keep else deletable(have, age, dir_idle))
        return 0
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
    done = good + have
    bad = sorted(set(names) - set(good))
    if bad:
        print('NOT VERIFIED (kept on robot): %s' % ', '.join(bad), flush=True)
    for n in good:
        if n.endswith('.log'):
            print('fetched %s' % (dest / n), flush=True)
    cam = sum(1 for n in good if not n.endswith('.log'))
    if cam:
        print('fetched %d camera files' % cam, flush=True)
    if not keep:
        _delete(remote, deletable(done, age, dir_idle))
    return len(good)


def _delete(remote, names):
    if not names:
        return
    r = remote.run("xargs -d '\\n' rm -f -- && find . -mindepth 1 -depth -type d -empty -delete", data='\n'.join(names).encode() + b'\n')
    if r.returncode != 0:
        print('delete failed: %s' % r.stderr.decode(errors='replace').strip(), flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default='shunsuke09@etrobo-raspi,shunsuke09@etrobo-raspi.local',
                    help='カンマ区切りで複数。つながったものを使う')
    ap.add_argument('--remote-dir', default='EV3RT/2026-Alpha/run_logs')
    ap.add_argument('--dest', default=str(Path.home() / 'Downloads' / 'et_run_logs'))
    ap.add_argument('--watch', action='store_true')
    ap.add_argument('--interval', type=float, default=20.0)
    ap.add_argument('--min-age', type=int, default=10, help='更新からこの秒数たったファイルだけ扱う')
    ap.add_argument('--keep', action='store_true', help='ラズパイ側を消さない')
    ap.add_argument('--dir-idle', type=float, default=600.0, help='et_camera_*/ の中は、フォルダ全体がこの秒数更新されていないときだけ消す')
    ap.add_argument('--ssh', default='ssh', help=argparse.SUPPRESS)
    args = ap.parse_args()
    dest = Path(args.dest)
    dest.mkdir(parents=True, exist_ok=True)
    remote = Remote(args)
    last_error = None
    while True:
        try:
            n = fetch_once(remote, dest, args.min_age, args.keep, args.dir_idle)
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
