"""2026-11-16: 1配置の経路を作って計画にするまでの決定版の流れ(全配置の事前計算・100配置の確認で使う)。

usage: python run_layout.py <gates.json> <出力フォルダ> <名前>
出力: <名前>.r.json(道具の出力)・<名前>.plan.json(計画)・<名前>.chk.json(check_plan)・<名前>.log(道具のログ)・
      <名前>.pick.json(選んだ段と理由)。段ごとの <名前>.s1.* / <名前>.s2.* も残す。

2026-11-17: 今の流れは v2(下の STAGE1/STAGE2 の説明)。以下は前の流れ(RUN_LAYOUT_FLOW=v1)の説明。
前の流れ:
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
# 2026-11-17(計算時間の試し): RUN_LAYOUT_NOFAST=1 で、つなぎの例外のすき間を試さない(例外なしの1回だけ)
if os.environ.get('RUN_LAYOUT_NOFAST', '0') == '1':
    FAST = None
TIMEOUT = int(os.environ.get('RUN_LAYOUT_TIMEOUT', '4000'))
# 2026-11-16(計算時間): 新しいやり方。道具の中で、numba で候補を先に飛ばし(run_kernel=1、結果は同じ)、例外のすき間の経路は
# 例外なしでも解き直して1秒以上速いときだけ残す(fast_keep_min_sec=1.0)。例外が残ったときだけ、5秒未満なら70秒の周回数で比べる
# ため例外なしでもう1回計算する。
NEW = os.environ.get('RUN_LAYOUT_FAST', '0') == '1'
if NEW:
    COMMON.update(HR_RUN_KERNEL='1', HR_LINE_KERNEL='1', HR_FAST_KEEP_MIN_SEC='1.0')
# 2026-11-16: 試し用。RUN_LAYOUT_EXTRA="HR_VIA_GROUP_TRIES=15;HR_ARC_FIRST_TRIES=30" のように、道具の設定を上書きする
# (2026-11-17: 段ごとの設定 STAGE1/STAGE2 よりも後に上書きする)
EXTRA = {}
for kv in filter(None, os.environ.get('RUN_LAYOUT_EXTRA', '').split(';')):
    k_, v_ = kv.split('=', 1)
    EXTRA[k_.strip()] = v_.strip()

# 2026-11-17(ユーザー決定): 決定版の流れ v2。RUN_LAYOUT_FLOW=v1 で前の流れ(process_v1)。
# 1回目(STAGE1): 100配置の経路の傾向で候補を絞る(角度 0・±10・±14、ゲートの手前・奥の伸ばし 0〜14cm、斜めの通過の円弧10cm。
#   100配置で計算 約0.4倍、経路は合計+4秒)。狭いゲートだけ真ん中から横にずらす候補(narrow_offsets)とすき間の段 3.0→2.0→1.5
#   (2cm で通れる狭いゲートは2cm を守る。#244 など)。つなぎの例外のすき間は試さない(100配置で条件を満たした配置は0)。
# 2回目(STAGE2): 70秒の周回の境目に近い配置(経路が NEAR_FASTER だけ速ければ周回が増える)だけ、全部の候補と例外のすき間ありで
#   作り直し、周回が増えるか、例外なしで速いか、例外ありで10周 FAST_TOTAL_GAIN 秒以上速いときだけ採る。
FLOW = os.environ.get('RUN_LAYOUT_FLOW', 'v2')
_NOFF = '-1.0,-0.5,0.5,1.0'
_KER = dict(HR_RUN_KERNEL='1', HR_LINE_KERNEL='1', HR_FAST_KEEP_MIN_SEC='1.0')
STAGE1 = dict(_KER, HR_ANGLES='-14,-10,0,10,14', HR_EXT='0,4,8,14', HR_SPLIT_RADII='10',
              HR_MARGIN_LIST='3.0,2.0,1.5', HR_NARROW_OFFSETS=_NOFF)
STAGE2 = dict(_KER, HR_ANGLES='-30,-20,-16,-14,-10,0,10,14,16,20,30', HR_MARGIN_LIST='3.0,2.0,1.5', HR_NARROW_OFFSETS=_NOFF)
NEAR_FASTER = 0.03   # 経路が3%速いと70秒の周回が増える配置を「境目に近い」とする(絞りの差は最大約2%、例外は約1.5%)


def tool(gates, out_r, log, fast, stage=None):
    env = dict(os.environ, **COMMON)
    if stage:
        env.update(stage)
    env.update(EXTRA)
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


def laps_if_faster(plan, faster):
    """計画の経路が faster(割合)だけ速いときの、持ち時間 BUDGET での周回数(check_plan の速さの見込み ratio を変えて調べる)。"""
    ratio = 0.93 * (1.0 - faster)
    if INPROC:
        import contextlib, io
        saved = {k: os.environ.pop(k) for k in list(os.environ) if k.startswith('HR_')}
        try:
            if HERE not in sys.path:
                sys.path.insert(0, HERE)
            import check_plan
            with contextlib.redirect_stdout(io.StringIO()):
                return check_plan.check(plan, os.path.join(HERE, 'params_day.json'), BUDGET, 1.5, ratio)['budget']['laps']
        except Exception as e:
            print('laps_if_faster failed for %s: %r' % (plan, e), flush=True)
            return None
        finally:
            os.environ.pop('HR_PARAMS', None)
            os.environ.update(saved)
    p = subprocess.run([sys.executable, os.path.join(HERE, 'check_plan.py'), plan, '--json', '--budget', str(BUDGET),
                        '--ratio', str(ratio)], capture_output=True, text=True, encoding='utf-8', env=dict(os.environ, PYTHONUTF8='1'))
    try:
        return json.loads(p.stdout)['budget']['laps']
    except (ValueError, KeyError):
        return None


def _copy(src_prefix, dst_prefix):
    for suf in ('.r.json', '.log', '.plan.json', '.chk.json'):
        if os.path.exists(src_prefix + suf):
            with open(src_prefix + suf, 'rb') as a, open(dst_prefix + suf, 'wb') as b:
                b.write(a.read())


def process(gates, out, name):
    if FLOW == 'v1':
        return process_v1(gates, out, name)
    os.makedirs(out, exist_ok=True)
    P = lambda suf: os.path.join(out, name + suf)
    t1, _ = tool(gates, P('.s1.r.json'), P('.s1.log'), None, STAGE1)
    lt = open(P('.s1.log'), encoding='utf-8', errors='ignore').read() if os.path.exists(P('.s1.log')) else ''
    if 'DEFERRED narrow' in lt or 'TIMEOUT' in lt:
        why = 'narrow' if 'DEFERRED narrow' in lt else 'timeout %ds' % TIMEOUT
        json.dump(dict(deferred=why), open(P('.deferred.json'), 'w', encoding='utf-8'))
        print('deferred', name, why, flush=True)
        return
    c1 = plan_and_check(P('.s1.r.json'), gates, P('.s1.plan.json'), P('.s1.chk.json')) if t1 is not None else None
    if t1 is None or c1 is None:
        json.dump(dict(failed=True, stage=1), open(P('.failed.json'), 'w', encoding='utf-8'))
        print('failed', name, flush=True)
        return
    l1 = c1['budget']['laps']
    ln = laps_if_faster(P('.s1.plan.json'), NEAR_FASTER)
    pick = dict(flow='v2', s1_total=t1, s1_laps70=l1, s1_ng=c1['ng'], laps70_if_faster=ln, chosen='s1')
    if ln is not None and ln > l1:
        t2, used2 = tool(gates, P('.s2.r.json'), P('.s2.log'), FAST, STAGE2)
        c2 = plan_and_check(P('.s2.r.json'), gates, P('.s2.plan.json'), P('.s2.chk.json')) if t2 is not None else None
        exc = bool(used2) and used2 != 'none'
        pick.update(s2_total=t2, s2_used=used2, s2_laps70=(c2 or {}).get('budget', {}).get('laps'), s2_ng=(c2 or {}).get('ng'))
        if c2 is not None and not c2['ng']:
            l2 = c2['budget']['laps']
            if l2 > l1:
                pick.update(chosen='s2', why='near the lap boundary: laps70 %d -> %d (%s)' % (l1, l2, 'with exceptions' if exc else 'no exceptions'))
            elif not exc and t2 < t1 - 1e-9 and l2 >= l1:
                pick.update(chosen='s2', why='near the lap boundary: %.1fs faster with all candidates' % (t1 - t2))
            elif exc and t1 - t2 >= FAST_TOTAL_GAIN - 1e-9 and l2 >= l1:
                pick.update(chosen='s2', why='near the lap boundary: exceptions %.1fs faster' % (t1 - t2))
            else:
                pick['why'] = 'near the lap boundary, but stage 2 not better (%.1fs, laps70 %d)' % (t2, l2)
        else:
            pick['why'] = 'near the lap boundary, stage 2 failed'
    else:
        pick['why'] = 'stage 1 (not near the lap boundary)'
    _copy(P('.' + pick['chosen']), P(''))
    json.dump(pick, open(P('.pick.json'), 'w', encoding='utf-8'), ensure_ascii=False)
    print('done', name, pick['chosen'], pick['why'], flush=True)


def process_v1(gates, out, name):
    os.makedirs(out, exist_ok=True)
    P = lambda suf: os.path.join(out, name + suf)
    tf, used = tool(gates, P('.fast.r.json'), P('.fast.log'), FAST)
    # 2026-11-17: 後回し(狭い配置で defer_narrow、または打ち切り時間 RUN_LAYOUT_TIMEOUT を超えた)なら結果を書かずに印だけ残す
    # (<名前>.pick.json がないので、run_all.py の2回目で計算する)
    lt0 = open(P('.fast.log'), encoding='utf-8', errors='ignore').read() if os.path.exists(P('.fast.log')) else ''
    if 'DEFERRED narrow' in lt0 or 'TIMEOUT' in lt0:
        why = 'narrow' if 'DEFERRED narrow' in lt0 else 'timeout %ds' % TIMEOUT
        json.dump(dict(deferred=why), open(P('.deferred.json'), 'w', encoding='utf-8'))
        print('deferred', name, why, flush=True)
        return
    if tf is None:
        # 2026-11-17: 道具が経路を出せなかった(エラーなど)。<名前>.pick.json を書かない(run_all.py の再開で計算し直す)
        json.dump(dict(failed=True), open(P('.failed.json'), 'w', encoding='utf-8'))
        print('failed', name, flush=True)
        return
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
