"""2026-11-19(ユーザー: 試走の前に、ラズパイで試走のコマンドがうまくいくか試したい): 走らせずに確かめる点検。
ロボットのラズパイで(~/EV3RT/2026-Alpha で):
    python3 preflight_1119.py
モーターは動かさない。カメラは --camera を付けたときだけ1枚撮る(camera_calib/<日時>_check/ に保存)。
最後に「全部 OK」か、NG の項目と対処が出る。
"""
import base64
import json
import os
import subprocess
import sys
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
NEED_COMMIT = '655684d0'
results = []


def check(name, fn):
    try:
        msg = fn()
        results.append((name, True, msg or ''))
        print('OK  %s %s' % (name, msg or ''), flush=True)
    except Exception as error:   # 1つ失敗しても残りは続ける
        results.append((name, False, '%s' % (error,)))
        print('NG  %s: %s' % (name, error), flush=True)


def git_version():
    head = subprocess.run(['git', 'log', '--oneline', '-1'], cwd=HERE, capture_output=True, text=True).stdout.strip()
    r = subprocess.run(['git', 'merge-base', '--is-ancestor', NEED_COMMIT, 'HEAD'], cwd=HERE, capture_output=True)
    if r.returncode != 0:
        raise RuntimeError('古い(%s)。git pull してください(%s 以降が要る)' % (head, NEED_COMMIT))
    return head


def alpha_imports():
    r = subprocess.run([sys.executable, 'alpha.py', '--help'], cwd=HERE, capture_output=True, text=True, timeout=120)
    if r.returncode != 0:
        raise RuntimeError('alpha.py が起動できない: %s' % (r.stderr.strip().splitlines() or ['?'])[-1])
    for opt in ('--rally-camera-log', '--rally-straight-pause-labels', '--rally-plan-path', '--lap-bottle-aim',
                '--rally-strategy', '--lap-capture-stop-mm', '--rally-camera-calib'):
        if opt not in r.stdout:
            raise RuntimeError('alpha.py にオプション %s がない' % opt)
    return '(試走のコマンドのオプションは全部ある)'


def libs():
    # 走る本体(python3 = ロボットでは PyPy)は py_etrobo_util/video.py と同じ場所から cv2 を読む
    import platform
    extra = '/usr/local/lib/pypy3/dist-packages' if platform.python_implementation() == 'PyPy' else '/usr/lib/python3/dist-packages'
    if os.path.isdir(extra) and extra not in sys.path:
        sys.path.append(extra)
    import cv2
    import numpy
    import Crypto
    try:
        import zxingcpp   # ヒントの QR を読む部品(ないと QR が読めない)
    except ImportError:
        raise RuntimeError('zxingcpp がない(ヒントの QR が読めない)。pip で入れるか、元のラズパイの環境を確認')
    msg = '本体 %s: cv2 %s, numpy %s, pycryptodome %s, zxingcpp あり' % (
        platform.python_implementation(), cv2.__version__, numpy.__version__, Crypto.__version__)
    # カメラの子・床の較正は /usr/bin/python3(LD_LIBRARY_PATH を外す)で動く
    if platform.python_implementation() == 'PyPy' and os.path.exists('/usr/bin/python3'):
        env = {k: v for k, v in os.environ.items() if k not in ('PYTHONPATH', 'PYTHONHOME', 'LD_LIBRARY_PATH')}
        r = subprocess.run(['/usr/bin/python3', '-c', 'import cv2, numpy; print(cv2.__version__)'],
                           capture_output=True, text=True, env=env, timeout=60)
        if r.returncode != 0:
            raise RuntimeError('/usr/bin/python3 で cv2 が読めない(カメラの補正・床の較正が動かない): %s' % (
                (r.stderr.strip().splitlines() or ['?'])[-1]))
        msg += '。/usr/bin/python3: cv2 %s' % r.stdout.strip()
    return msg


def hint2():
    from Crypto.Cipher import AES
    from Crypto.Hash import SHA256
    from Crypto.Protocol.KDF import PBKDF2
    from Crypto.Util.Padding import pad
    from shared_communication.hint_decoder import decode_hint2
    out = []
    for key in ('1234', 'T111', 'a-1!'):
        salt = os.urandom(8)
        k = PBKDF2(key.encode('utf-8'), salt, dkLen=16, count=10000, hmac_hash_module=SHA256)
        raw = base64.b64encode(b'Salted__' + salt + AES.new(k, AES.MODE_ECB).encrypt(pad(b'53,54/12,22', 16))).decode()
        if decode_hint2(raw, key) != '53,54/12,22':
            raise RuntimeError('キー %r で復号できない' % key)
        out.append(key)
    return '(キー %s で復号できた)' % ', '.join(out)


def calib():
    from robot_program import et_rally_start_pose as sp
    if not sp.DEFAULT_CALIB.exists():
        raise RuntimeError('較正ファイルがない: %s' % sp.DEFAULT_CALIB)
    d = json.load(open(sp.DEFAULT_CALIB, encoding='utf-8'))
    msg = '今の較正 %s(%s)' % (sp.DEFAULT_CALIB.name, sp.CALIB_SOURCE)
    if sp.CALIB_SOURCE == 'LOCAL_CALIB.txt':
        msg += '、床の較正あり start_fix=%s' % (d.get('start_fix'),)
    else:
        msg += '、床の較正はまだ(止まって撮る較正のあと --auto で入る)'
    return msg


def floor_calib_tool():
    r = subprocess.run([sys.executable, os.path.join('wireless_device', 'et_rally_camera_calib', 'day_floor_calib.py'), '--help'],
                       cwd=HERE, capture_output=True, text=True, timeout=120)
    if r.returncode != 0 or '--auto' not in r.stdout:
        raise RuntimeError('day_floor_calib.py が動かない: %s' % (r.stderr.strip().splitlines() or ['?'])[-1])
    return '(day_floor_calib.py --auto が使える)'


def selfcal_plan():
    p = os.path.join(HERE, 'robot_program', 'tests', 'plan_test_selfcal_stops_1114.json')
    plan = json.load(open(p, encoding='utf-8'))
    return '(止まって撮る較正の計画 %d 段)' % len(plan.get('steps', []))


def plans():
    from robot_program import et_rally_plan_select as s
    out = []
    for d in ('et_rally_plans', 'et_rally_plans_m20'):
        z = zipfile.ZipFile(os.path.join(HERE, d, 'plans.zip'))
        n = len([x for x in z.namelist() if x.endswith('.json')])
        if n != 4872:
            raise RuntimeError('%s の計画が %d 配置(4872 のはず)' % (d, n))
        out.append('%s %d' % (d, n))
    # 2回目のコマンドと同じ設定で、2cm 用が選ばれるか(2cm 版に差し替えた配置で確かめる)
    key = 'r0010-b0102-y0313'
    gates = json.loads(zipfile.ZipFile(os.path.join(HERE, 'et_rally_plans', 'plans.zip')).read('plan_%s.json' % key))['gates']
    old = os.environ.pop('ET_RALLY_PLANS_DIR', None)
    try:
        w1 = s.find_precomputed(gates)[1]
        os.environ['ET_RALLY_PLANS_DIR'] = 'et_rally_plans_m20'
        w2 = s.find_precomputed(gates)[1]
    finally:
        os.environ.pop('ET_RALLY_PLANS_DIR', None)
        if old is not None:
            os.environ['ET_RALLY_PLANS_DIR'] = old
    if 'et_rally_plans_m20' in w1 or 'et_rally_plans_m20' not in w2:
        raise RuntimeError('計画の選び方がおかしい(1回目 %s / 2回目 %s)' % (w1, w2))
    return '(%s。1回目は3cm、2回目のコマンドなら2cm 用を選ぶ)' % ', '.join(out)


def lap_files():
    import re
    src = open(os.path.join(HERE, 'robot_program', 'features', 'lap_camera_recovery.py'), encoding='utf-8').read()
    m = re.search(r'^CAPTURE_STOP_MM = ([0-9.]+)', src, re.M)
    if not m:
        raise RuntimeError('LAP のボトルを捕まえて止める設定が見つからない')
    for f in ('start_lap_profile_left.py', 'start_lap_profile_right.py', 'start_lap_profile_left_right_copy_1118.py'):
        if not os.path.exists(os.path.join(HERE, 'robot_program', f)):
            raise RuntimeError('robot_program/%s がない' % f)
    return '(左右の LAP 経路あり、ボトルを捕まえて止める %smm)' % m.group(1)


def camera():
    r = subprocess.run([sys.executable, 'et_rally_camera_capture.py', '--tag', 'check', '--count', '1'],
                       cwd=HERE, capture_output=True, text=True, timeout=120, input='\n')
    if r.returncode != 0:
        raise RuntimeError('カメラで撮れない: %s' % (r.stderr.strip().splitlines() or ['?'])[-1])
    return '(1枚撮れた)'


def main():
    check('1 リポジトリの版', git_version)
    check('2 部品(OpenCV・numpy・暗号)', libs)
    check('3 alpha.py の起動とオプション', alpha_imports)
    check('4 ヒント2の復号', hint2)
    check('5 カメラの較正', calib)
    check('6 床の較正の道具', floor_calib_tool)
    check('7 止まって撮る較正の計画', selfcal_plan)
    check('8 ETラリーの計画(1回目・2回目)', plans)
    check('9 LAP', lap_files)
    if '--camera' in sys.argv:
        check('10 カメラ', camera)
    ng = [r for r in results if not r[1]]
    print('')
    if ng:
        print('=== NG が %d 個: 上の NG の行を見てください(画面を写真で送ってもらえれば見ます) ===' % len(ng))
        sys.exit(1)
    print('=== 全部 OK: 試走のコマンドを打って大丈夫 ===')


if __name__ == '__main__':
    main()
