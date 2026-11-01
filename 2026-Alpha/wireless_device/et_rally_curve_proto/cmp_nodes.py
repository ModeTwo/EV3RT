"""execute_strategy.steps_from_strategy が作るノードを、指定したツリー(2026-Alpha)で作って要約を出す。
usage: cmp_nodes.py <2026-Alpha dir> <plan.json> [kwargs-json]
"""
import sys, json, types
sys.dont_write_bytecode = True
from unittest import mock

root, plan_path = sys.argv[1], sys.argv[2]
kwargs = json.loads(sys.argv[3]) if len(sys.argv) > 3 else {}
for name in ('cv2', 'etrobo_python', 'etrobo_python.backends', 'etrobo_python.backends.raspike_art',
             'etrobo_python.backends.raspike_art.device', 'picamera2', 'numpy.typing'):
    if name == 'numpy.typing':
        continue
    sys.modules.setdefault(name, mock.MagicMock())
sys.path.insert(0, root)
import robot_program.features.execute_strategy as es
from robot_program.runtime import runtime
runtime.course = -1

steps = json.load(open(plan_path, encoding='utf-8'))['steps']
nodes = es.steps_from_strategy(steps, **kwargs)

KEYS = ('target', 'power', 'distance_mm', 'decel_mm', 'accel_mm', 'reverse', 'delta_dist', 'kind', 'end_heading',
        'sweep', 'radius_mm', 'length_mm', 'run_length_mm', 'stop_at_end', 'continue_from_prev', 'remaining_after_mm',
        'main_power', 'pre_stop', 'min_seconds')


def summ(n, depth=0):
    attrs = {k: getattr(n, k) for k in KEYS if hasattr(n, k)}
    attrs = {k: (round(v, 2) if isinstance(v, float) else v) for k, v in attrs.items()}
    out = ['  ' * depth + f'{type(n).__name__} {n.name} {attrs}']
    for c in getattr(n, 'children', []):
        out += summ(c, depth + 1)
    return out


for n in nodes:
    print('\n'.join(summ(n)))
