"""机上の確認: 計画JSONを steps_from_strategy でノードにし、仮のロボット(運動学のみ)の上で走らせる。
旋回(EtRallyTurnOneShot等)は、その場で目標の向きにぴったり合わせたことにする(ここでは確かめない)。
慣性・すべり・静止摩擦は無い。出力→車輪の速さは比例(出力80で360mm/s)+一次遅れ。
usage: sim_path.py <2026-Alpha> <plan.json> [kwargs-json] [out.json]
"""
import sys, json, math, time, os
sys.dont_write_bytecode = True
from unittest import mock

root, plan_path = sys.argv[1], sys.argv[2]
kwargs = json.loads(sys.argv[3]) if len(sys.argv) > 3 else {}
out_path = sys.argv[4] if len(sys.argv) > 4 else None
for name in ('cv2', 'etrobo_python', 'etrobo_python.backends', 'etrobo_python.backends.raspike_art',
             'etrobo_python.backends.raspike_art.device', 'picamera2'):
    sys.modules.setdefault(name, mock.MagicMock())

# 仮の時計
_now = [1000.0]
time.monotonic = lambda: _now[0]
time.time = lambda: _now[0]

sys.path.insert(0, root)
import robot_program.features.execute_strategy as es
from robot_program.runtime import runtime
from robot_program import gyro_scale
from py_trees.common import Status

DT = 0.02
TIRE = 55.56
TREAD = 117.0
MM_PER_POWER = 360.0 / 80.0
# 2026-11-03: SIM_TAU(なじむ時間、秒)と SIM_DELAY(出力が効き始めるまでのtick数)。
# SIM_DELAY=1 SIM_TAU=0.12 で、実機の円弧の出力の揺れ(約0.5秒周期・内側の車輪がマイナスになる割合約5%)とほぼ同じになった
# (run_20261103T061719のarc19)。既定は従来どおり(0, 0.06)。
TAU = float(os.environ.get('SIM_TAU', '0.06'))
PWR_LOG = []
P = 24.64


class Motor:
    def __init__(self):
        self.power = 0.0; self.speed = 0.0; self.count = 0.0; self.brake = False

    def set_power(self, p):
        self.power = max(-100, min(100, p))
        self.q = getattr(self, 'q', []) + [self.power]

    def eff_power(self):
        q = getattr(self, 'q', [self.power])
        k = int(os.environ.get('SIM_DELAY', '0'))
        self.q = q[-(k + 1):]
        return self.q[0]

    def set_brake(self, b):
        self.brake = b

    def et_set_speed(self, dps):
        # 速度制御(机上): 度/秒 → 出力の単位に直して同じ一次遅れで追う(ハブの速度制御の遅れは未確認)
        self.set_power(dps * (math.pi * TIRE / 360.0) / MM_PER_POWER)

    def get_count(self):
        return int(round(self.count))

    def reset_count(self):
        self.count = 0.0


class Gyro:
    def __init__(self, robot):
        self.robot = robot

    def get_angle(self):
        return -math.degrees(self.robot.phi)   # 時計回りが正(右コースでは heading = この値)

    def reset(self):
        pass


class Plotter:
    def __init__(self):
        self.distance = 0.0; self.tire_diameter = TIRE

    def get_distance(self):
        return int(self.distance)

    def get_loc_x(self):
        return 0

    def get_loc_y(self):
        return 0


class Robot:
    def __init__(self, x, y, phi):
        self.x, self.y, self.phi = x, y, phi
        self.trace = [(x, y)]

    def step(self, rm, lm, plotter):
        for m in (rm, lm):
            target = m.eff_power() * MM_PER_POWER
            m.speed += (target - m.speed) * min(1.0, DT / TAU)
            if m.power == 0 and m.brake:
                m.speed *= 0.5
            m.count += m.speed * DT * 360.0 / (math.pi * TIRE)
        vr, vl = rm.speed, lm.speed
        v = (vr + vl) / 2
        w = (vr - vl) / TREAD
        # 2026-11-03: 1tickの間の向きは、始めと終わりの平均を使う(終わりの向きだけだと、円弧で1tickに約4°回る分
        # 位置が内側へずれ、周回の机上の比較で数mm〜1cm以上の差になっていた)。
        mid = self.phi + w * DT / 2.0
        self.phi += w * DT
        self.x += v * DT * math.cos(mid) / 10.0
        self.y += v * DT * math.sin(mid) / 10.0
        plotter.distance += abs(v) * DT
        self.trace.append((self.x, self.y))


robot = Robot(4 * P - (4 * P + 17.8), 3.5 * P, 0.0)
runtime.course = -1
runtime.right_motor = Motor(); runtime.left_motor = Motor()
runtime.gyro_sensor = Gyro(robot); runtime.plotter = Plotter(); runtime.hub = None
gyro_scale.set_disabled(True)

import os
from robot_program.behaviours import et_rally_drive
if os.environ.get('SIM_PID_I'):
    et_rally_drive.set_move_tuning(True, float(os.environ['SIM_PID_I']))
steps = json.load(open(plan_path, encoding='utf-8'))['steps']
nodes = es.steps_from_strategy(steps, **kwargs)

# 計画上の位置(区間の終わり)を、計画JSONそのものから積算する(planner座標、gyro = -planner角)
def planned_points(steps):
    x, y, h = robot.x, robot.y, 0.0
    pts = []
    for s in steps:
        if s['type'] == 'turn':
            h = -s['target_heading_deg']; continue
        rev = s.get('reverse', False)
        if s['type'] == 'move':
            hh = -s['target_heading_deg']; th = math.radians(hh + (180 if rev else 0))
            L = s['distance_mm'] / 10
            x += L * math.cos(th); y += L * math.sin(th); h = hh
        else:
            sw = -s['sweep_deg']; r = s['radius_mm'] / 10
            th0 = math.radians(h + (180 if rev else 0)); sg = 1 if sw > 0 else -1
            cx, cy = x - sg * r * math.sin(th0), y + sg * r * math.cos(th0)
            th1 = th0 + math.radians(sw)
            x, y = cx + sg * r * math.sin(th1), cy - sg * r * math.cos(th1)
            h = h + sw
        pts.append((s['type'], s.get('label'), s.get('stop', True), round(x, 2), round(y, 2)))
    return pts


plan_pts = planned_points(steps)
results = []
seg_i = 0
for node in nodes:
    cls = type(node).__name__
    if cls.startswith('EtRallyTurn') or cls.startswith('EtRallySpin'):
        robot.phi = math.radians(-node.target)  # その場でぴったり回したことにする
        for m in (runtime.right_motor, runtime.left_motor):
            m.speed = 0.0; m.power = 0
        continue
    if cls in ('LogLapBoundary',):
        continue
    ticks = 0
    while True:
        for _ in node.tick():
            pass
        robot.step(runtime.right_motor, runtime.left_motor, runtime.plotter)
        PWR_LOG.append((round(_now[0]-1000.0,2), node.name, runtime.right_motor.power, runtime.left_motor.power))
        _now[0] += DT
        ticks += 1
        if node.status in (Status.SUCCESS, Status.FAILURE) or ticks > 3000:
            break
    if cls in ('EtRallyPathSegment', 'Parallel'):
        p = plan_pts[seg_i] if seg_i < len(plan_pts) else None
        seg_i += 1
        err = math.hypot(robot.x - p[3], robot.y - p[4]) * 10 if p else None
        results.append(dict(node=node.name, status=str(node.status), ticks=ticks, sim=(round(robot.x, 2), round(robot.y, 2)),
                            heading=round(-math.degrees(robot.phi), 2), plan=p, err_mm=round(err, 1) if err is not None else None))
for r in results:
    if r['plan'] and (r['plan'][2] or r['plan'][1]):
        print(f"{r['node']:28s} {r['status']:14s} ticks={r['ticks']:4d} sim={r['sim']} plan={r['plan'][3:]} err={r['err_mm']}mm heading={r['heading']}")
print('time %.1fs (旋回は0秒として)' % (_now[0] - 1000.0))
if out_path:
    json.dump(dict(trace=robot.trace[::5], results=results, power=PWR_LOG), open(out_path, 'w'))
