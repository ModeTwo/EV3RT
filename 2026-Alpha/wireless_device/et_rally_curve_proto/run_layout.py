"""2026-11-16: 1配置の経路を作って計画にするまでの決定版の流れ(全配置の事前計算・100配置の確認で使う)。

usage: python run_layout.py <gates.json> <出力フォルダ> <名前>
出力: <名前>.r.json(道具の出力)・<名前>.plan.json(計画)・<名前>.chk.json(check_plan)・<名前>.log(道具のログ)・
      <名前>.pick.json(例外のすき間を使ったか、選んだ理由)

流れ:
1. 道具(hybrid_route.py)を例外のすき間あり(fast_margins)で計算する。例外が使われなければ、それで決まり。
2. 例外が使われたときだけ、例外なしでも計算する。両方を計画にして check_plan にかけ、
   例外ありを採るのは、当たり判定が OK で、10周の合計が FAST_TOTAL_GAIN 秒以上速い(1周0.5秒)か、
   持ち時間70秒での周回数が増えるとき(ユーザー 11/16)。
設定(ユーザー 11/16 までの決定):
- つなぎ目の横のずれは join_lat_cm=0.02 まで(ゲートの中の円弧なしの折れをなくす)。
- ゲートのすき間は 3cm、通れないゲート(狭い配置)だけ 1.5cm(margin_list=3.0,1.5、ゲートごと)。
- つなぎの例外のすき間は 2.5 → 2.0cm(下限2cm)、その区間で0.5秒以上速いときだけ(fast_margins)。
"""
import json
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FAST_TOTAL_GAIN = 5.0   # 10周で5秒 = 1周0.5秒
BUDGET = 70.0

COMMON = dict(
    HR_PARAMS=os.path.join(HERE, 'params_day.json'),
    HR_ARC_FIRST='1', HR_VIA_EARLY='1', HR_VIA_GROUP_TRIES='30', HR_ARC_FIRST_TRIES='60', HR_ARC_FIRST_RUN_TOP='24',
    HR_EXIT_EARLY='-4', HR_NARROW_SAFEST='0', HR_OBLIQUE_POLICY='all', HR_PER_GATE_MARGIN='1',
    HR_PER_GATE_CONN_MODE='fallback', HR_JOIN_LAT_CM='0.02', HR_MARGIN_LIST='3.0,1.5',
    PYTHONUTF8='1', OMP_NUM_THREADS='1', NUMBA_NUM_THREADS='1',
)
FAST = '2.5,2.0'
TIMEOUT = int(os.environ.get('RUN_LAYOUT_TIMEOUT', '4000'))
# 2026-11-16(計算時間): 新しいやり方。道具の中で、numba で候補を先に飛ばし(run_kernel=1、結果は同じ)、例外のすき間の経路は
# 例外なしでも解き直して1秒以上速いときだけ残す(fast_keep_min_sec=1.0)。例外が残ったときだけ、5秒未満なら70秒の周回数で比べる
# ため例外なしでもう1回計算する。
NEW = os.environ.get('RUN_LAYOUT_FAST', '0') == '1'
if NEW:
    COMMON.update(HR_RUN_KERNEL='1', HR_LINE_KERNEL='1', HR_FAST_KEEP_MIN_SEC='1.0')
# 2026-11-16: 試し用。RUN_LAYOUT_EXTRA="HR_VIA_GROUP_TRIES=15;HR_ARC_FIRST_TRIES=30" のように、道具の設定を上書きする
for kv in filter(None, os.environ.get('RUN_LAYOUT_EXTRA', '').split(';')):
    k_, v_ = kv.split('=', 1)
    COMMON[k_.strip()] = v_.strip()


def tool(gates, out_r, log, fast):
    env = dict(os.environ, **COMMON)
    env.pop('HR_FAST_MARGINS', None)
    if fast:
        env['HR_FAST_MARGINS'] = fast
    with open(log, 'w', encoding='utf-8') as lf:
        try:
            subprocess.run([sys.executable, os.path.join(HERE, 'hybrid_route.py'), gates, out_r], stdout=lf,
                           stderr=subprocess.STDOUT, env=env, timeout=TIMEOUT)
        except subprocess.TimeoutExpired:
            lf.write('\nTIMEOUT %ds\n' % TIMEOUT)
    t = open(log, encoding='utf-8', errors='ignore').read()
    m = re.search(r'TOTAL hybrid laps=\d+: ([\d.]+)s', t)
    f = re.search(r'fast margins used: (.*)', t)
    return (float(m.group(1)) if m else None), (f.group(1).strip() if f else None)


def plan_and_check(r, gates, plan, chk):
    if INPROC:
        return _plan_and_check_inproc(r, gates, plan, chk)
    env = dict(os.environ, PYTHONUTF8='1')
    subprocess.run([sys.executable, os.path.join(HERE, 'to_plan.py'), r, gates, plan], capture_output=True, env=env)
    if not os.path.exists(plan):
        return None
    p = subprocess.run([sys.executable, os.path.join(HERE, 'check_plan.py'), plan, '--json', '--budget', str(BUDGET)],
                       capture_output=True, text=True, encoding='utf-8', env=env)
    try:
        d = json.loads(p.stdout)
    except ValueError:
        return None
    json.dump(d, open(chk, 'w', encoding='utf-8'), ensure_ascii=False)
    return d


INPROC = False   # True: 計画への変換と当たり判定を、この作業者のプロセスの中で行う(--batch のとき。読み込みの手間を省く)


def _plan_and_check_inproc(r, gates, plan, chk):
    """plan_and_check と同じことを、このプロセスの中で行う(to_plan.py を同じ引数で実行し、check_plan.check を呼ぶ)。
    道具の設定の環境変数(HR_*)は、別プロセスで実行するときと同じく外しておく。"""
    import contextlib, io, runpy
    saved = {k: os.environ.pop(k) for k in list(os.environ) if k.startswith('HR_')}
    argv = sys.argv
    try:
        sys.argv = [os.path.join(HERE, 'to_plan.py'), r, gates, plan]
        with contextlib.redirect_stdout(io.StringIO()):
            try:
                runpy.run_path(os.path.join(HERE, 'to_plan.py'), run_name='__main__')
            except SystemExit:
                pass
        if not os.path.exists(plan):
            return None
        sys.argv = [os.path.join(HERE, 'check_plan.py')]
        if HERE not in sys.path:
            sys.path.insert(0, HERE)
        import check_plan
        with contextlib.redirect_stdout(io.StringIO()):
            d = check_plan.check(plan, os.path.join(HERE, 'params_day.json'), BUDGET, 1.5, 0.93)
    except Exception as e:
        print('inproc plan/check failed for %s: %r' % (r, e), flush=True)
        return None
    finally:
        sys.argv = argv
        os.environ.pop('HR_PARAMS', None)
        os.environ.update(saved)
    json.dump(d, open(chk, 'w', encoding='utf-8'), ensure_ascii=False)
    return d


def main():
    if len(sys.argv) >= 3 and sys.argv[1] == '--batch':
        # usage: python run_layout.py --batch <一覧のファイル(1行に「gates.json 出力フォルダ 名前」)>
        global INPROC
        INPROC = True
        for line in open(sys.argv[2], encoding='utf-8'):
            a = line.split()
            if len(a) == 3:
                process(*a)
        return
    process(sys.argv[1], sys.argv[2], sys.argv[3])


def process(gates, out, name):
    os.makedirs(out, exist_ok=True)
    P = lambda suf: os.path.join(out, name + suf)
    tf, used = tool(gates, P('.fast.r.json'), P('.fast.log'), FAST)
    pick = dict(fast_total=tf, fast_used=used)
    chosen = 'fast'
    kept = None
    if NEW:
        # 道具の中で例外なしと比べ済み: "fast margins kept (X faster ...)" なら X 秒速い、"dropped" なら例外なしの経路を出している
        lt = open(P('.fast.log'), encoding='utf-8', errors='ignore').read()
        mk = re.search(r'fast margins kept \(([\d.]+)s faster', lt)
        kept = float(mk.group(1)) if mk else None
        if 'fast margins dropped' in lt:
            pick['tool'] = 'dropped in the tool'
    if NEW and kept is not None and kept >= FAST_TOTAL_GAIN - 1e-9:
        pick['why'] = 'kept in the tool (%.1fs faster)' % kept
    elif tf is not None and used and used != 'none':
        tb, _ = tool(gates, P('.base.r.json'), P('.base.log'), None)
        cf = plan_and_check(P('.fast.r.json'), gates, P('.fast.plan.json'), P('.fast.chk.json'))
        cb = plan_and_check(P('.base.r.json'), gates, P('.base.plan.json'), P('.base.chk.json')) if tb is not None else None
        pick.update(base_total=tb)
        lf = cf['budget']['laps'] if cf else None
        lb = cb['budget']['laps'] if cb else None
        pick.update(fast_laps70=lf, base_laps70=lb, fast_ng=(cf or {}).get('ng'), base_ng=(cb or {}).get('ng'))
        ok_f = cf is not None and not cf['ng']
        if cb is None or cb['ng']:
            chosen = 'fast' if ok_f else 'base'
            pick['why'] = 'base failed'
        elif ok_f and ((tb - tf) >= FAST_TOTAL_GAIN - 1e-9 or (lf or 0) > (lb or 0)):
            chosen = 'fast'
            pick['why'] = 'gain %.1fs, laps70 %s -> %s' % (tb - tf, lb, lf)
        else:
            chosen = 'base'
            pick['why'] = 'not enough: gain %.1fs, laps70 %s -> %s' % (tb - tf, lb, lf)
    else:
        pick['why'] = 'no exception used'
    pick['chosen'] = chosen
    src = '.fast' if chosen == 'fast' else '.base'
    for suf in ('.r.json', '.log'):
        if os.path.exists(P(src + suf)):
            with open(P(src + suf), 'rb') as a, open(P(suf), 'wb') as b:
                b.write(a.read())
    if os.path.exists(P('.r.json')):
        plan_and_check(P('.r.json'), gates, P('.plan.json'), P('.chk.json'))
    json.dump(pick, open(P('.pick.json'), 'w', encoding='utf-8'), ensure_ascii=False)
    print('done', name, chosen, pick.get('why'), flush=True)


if __name__ == '__main__':
    main()
