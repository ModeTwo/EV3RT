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


def main():
    gates, out, name = sys.argv[1], sys.argv[2], sys.argv[3]
    os.makedirs(out, exist_ok=True)
    P = lambda suf: os.path.join(out, name + suf)
    tf, used = tool(gates, P('.fast.r.json'), P('.fast.log'), FAST)
    pick = dict(fast_total=tf, fast_used=used)
    chosen = 'fast'
    if tf is not None and used and used != 'none':
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
