"""シードごとの右コースのゲート配置を作り(ローカルplannerのrandom_gates→左右反転)、
今のplanner(devSS)の経路を、同じ近似式で時間にする。usage: gen_seeds.py seed...
"""
import json, sys, time, importlib

LOCAL = r'C:\Users\MSAD\et_rally_planner'
DEVSS = r'C:\Users\MSAD\EV3RT\2026-Alpha\wireless_device\et_rally_planner'


def gates_for(seed):
    sys.path.insert(0, LOCAL)
    import random_gates as rg
    g, _ = rg.random_gates(seed)
    sys.path.remove(LOCAL)
    for m in ('random_gates', 'planner', 'config', 'geometry', 'rule_route', 'commands'):
        sys.modules.pop(m, None)
    return {c: ((4 - x.foot_a_grid[0], x.foot_a_grid[1]), (4 - x.foot_b_grid[0], x.foot_b_grid[1])) for c, x in g.items()}


def old_plan_time(gv, laps):
    sys.path.insert(0, DEVSS)
    import config
    config.LAPS = laps
    ax = 2 * config.GRID_PITCH_CM
    mx = lambda v: 2 * ax - v
    config.START_POS_CM = (mx(config.START_POS_CM[0]), config.START_POS_CM[1])
    config.GOAL_POS_CM = (mx(config.GOAL_POS_CM[0]), config.GOAL_POS_CM[1])
    config.START_HEADING_DEG = (180.0 - config.START_HEADING_DEG) % 360.0
    config.GOAL_HEADING_DEG = (180.0 - config.GOAL_HEADING_DEG) % 360.0
    config.KEEP_OUT_RECTS_CM = tuple((mx(b), mx(a), c, d) for a, b, c, d in config.KEEP_OUT_RECTS_CM)
    config.KEEP_OUT_GOAL_LANE_POINT_CM = (mx(config.KEEP_OUT_GOAL_LANE_POINT_CM[0]), config.KEEP_OUT_GOAL_LANE_POINT_CM[1])
    from commands import waypoints_to_plan
    from planner import Gate
    from rule_route import plan_route
    gates = {c: Gate(c, a, b) for c, (a, b) in gv.items()}
    t0 = time.time()
    wp, _, labels, _ = plan_route(gates, laps)
    steps = waypoints_to_plan(wp, labels)
    dt = time.time() - t0
    sys.path.remove(DEVSS)
    for m in ('planner', 'config', 'geometry', 'rule_route', 'commands'):
        sys.modules.pop(m, None)
    # 計画のステップを近似式で時間にする(planner角そのままで差分を取るので符号の規約は関係しない)
    h = None; t = 0.0
    for s in steps:
        if s['type'] == 'turn':
            if h is not None:
                dd = abs((s['target_heading_deg'] - h + 180) % 360 - 180)
                if dd > 0.3:
                    t += 0.7 + 0.012 * dd
            h = s['target_heading_deg']
        elif s['type'] == 'move':
            if h is None:
                h = s['target_heading_deg']
            t += 0.5 + 0.0025 * s['distance_mm']
    return t, dt


res = {}
for seed in map(int, sys.argv[1:]):
    gv = gates_for(seed)
    json.dump({'gates': {'seed': seed, 'course': 'right', **{c: {'a': list(a), 'b': list(b)} for c, (a, b) in gv.items()}}},
              open(f'gates_{seed}.json', 'w'))
    r = {'gates': gv}
    for laps in (3, 4):
        try:
            t, dt = old_plan_time(gv, laps)
            r[f'old{laps}'] = round(t, 1); r[f'old{laps}_calc_s'] = round(dt, 1)
        except Exception as e:
            r[f'old{laps}'] = f'error: {e}'
    res[seed] = r
    print(seed, r, flush=True)
json.dump(res, open('seeds_old.json', 'w'))
