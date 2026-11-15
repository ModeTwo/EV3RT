"""2026-11-15: 本番で、ヒントから経路の道具の計画(ファイルで走るときと同じ形)を用意して、丸ごとロボットへ送る(PC側)。

順番(どれを使うかはオプションで決める。ユーザーが最終決定前):
  1. plans_dir があれば、前もって作った計画 plan_<鍵>.json(または plans.zip の中)を探す(すぐ返せる)。
  2. compute_sec > 0 なら、その場で経路の道具(hybrid_route.py + to_plan.py、params_day.json)で作る(compute_sec 秒で打ち切り)。
  3. どちらもだめなら、予備の計画(スタートからガレージへ行くだけ、robot_program/tests/plan_garage_only_1114.json)。
戻り値の計画には gates(ゲートの位置)を必ず入れる(ロボットが脚の近くの点を使わないため)。
鍵・ヒントの読み方は shared_communication/et_rally_layouts.py(ロボットの予備の選び方と同じ)。
"""
import json
import logging
import os
import subprocess
import sys
import tempfile
import time
import zipfile
from pathlib import Path

from shared_communication.et_rally_layouts import gates_from_hints, layout_key

LOG = logging.getLogger(__name__)
ROOT = Path(__file__).resolve().parents[1]
CURVE = ROOT / "wireless_device" / "et_rally_curve_proto"
GARAGE_ONLY_PLAN = ROOT / "robot_program" / "tests" / "plan_garage_only_1114.json"
PLAN_FORMAT = "curve_plan_v1"


class CurvePlanProvider:
    def __init__(self, plans_dir=None, compute_sec=0.0, params=None, work_dir=None):
        self.plans_dir = Path(plans_dir) if plans_dir else None
        self.compute_sec = float(compute_sec or 0.0)
        self.params = Path(params) if params else CURVE / "params_day.json"
        self.work_dir = Path(work_dir) if work_dir else ROOT / "run_logs" / "curve_provider"

    # --- 1. 前もって作った計画 ---
    def find_precomputed(self, key):
        if self.plans_dir is None:
            return None, "no plans dir"
        name = "plan_%s.json" % key
        f = self.plans_dir / name
        try:
            if f.exists():
                return json.loads(f.read_text(encoding="utf-8")), str(f)
            z = self.plans_dir / "plans.zip"
            if z.exists():
                with zipfile.ZipFile(str(z)) as zf:
                    try:
                        return json.loads(zf.read(name).decode("utf-8")), "%s:%s" % (z, name)
                    except KeyError:
                        pass
        except (OSError, ValueError, zipfile.BadZipFile) as error:
            return None, "%s unreadable (%r)" % (name, error)
        return None, "%s not found" % name

    # --- 2. その場で作る ---
    def compute(self, gates, key):
        if self.compute_sec <= 0:
            return None, "compute off"
        work = self.work_dir / ("%s_%s" % (time.strftime("%Y%m%dT%H%M%S"), key))
        work.mkdir(parents=True, exist_ok=True)
        gp, rp, pp = work / "gates.json", work / "route.json", work / "plan.json"
        gp.write_text(json.dumps({"gates": gates}), encoding="utf-8")
        env = dict(os.environ, PYTHONUTF8="1", OMP_NUM_THREADS="1", NUMBA_NUM_THREADS="1", HR_PARAMS=str(self.params))
        t0 = time.time()
        try:
            with open(work / "log.txt", "w", encoding="utf-8") as log:
                subprocess.run([sys.executable, str(CURVE / "hybrid_route.py"), str(gp), str(rp)], env=env,
                               stdout=log, stderr=subprocess.STDOUT, timeout=self.compute_sec, check=True)
            left = max(5.0, self.compute_sec - (time.time() - t0))
            subprocess.run([sys.executable, str(CURVE / "to_plan.py"), str(rp), str(gp), str(pp)], env=env,
                           stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, timeout=left, check=True)
        except subprocess.TimeoutExpired:
            return None, "compute timed out after %.0fs" % (time.time() - t0)
        except subprocess.CalledProcessError as error:
            return None, "compute failed (%s, see %s)" % (error.returncode, work / "log.txt")
        return json.loads(pp.read_text(encoding="utf-8")), "computed in %.1fs (%s)" % (time.time() - t0, work)

    # --- まとめ ---
    def provide(self, hint1, hint2):
        """ヒント → (計画, どこから)。計画は必ず返す(最後は予備の計画)。"""
        try:
            gates = gates_from_hints(hint1, hint2)
        except ValueError as error:
            LOG.warning("hints not readable (%s): garage-only plan", error)
            return self._garage_only(None), "garage only (hints not readable: %s)" % error
        key = layout_key(gates)
        notes = []
        for step in ("precomputed", "compute"):
            plan, where = self.find_precomputed(key) if step == "precomputed" else self.compute(gates, key)
            if plan is not None and isinstance(plan.get("steps"), list) and plan["steps"]:
                plan = dict(plan, gates=plan.get("gates") or gates)
                LOG.info("plan for %s: %s", key, where)
                return plan, where
            notes.append(where)
        LOG.warning("plan for %s not available (%s): garage-only plan", key, "; ".join(notes))
        return self._garage_only(gates), "garage only (%s)" % "; ".join(notes)

    @staticmethod
    def _garage_only(gates):
        plan = json.loads(GARAGE_ONLY_PLAN.read_text(encoding="utf-8"))
        if gates is not None:
            plan["gates"] = gates
        return plan


def main():
    """確かめ用: python -m wireless_device.curve_plan_provider --hint1 21,31 --hint2 23,22/32,42 [--plans-dir D] [--compute-sec S]"""
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--hint1", required=True)
    ap.add_argument("--hint2", required=True)
    ap.add_argument("--plans-dir")
    ap.add_argument("--compute-sec", type=float, default=0.0)
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO)
    t0 = time.time()
    plan, where = CurvePlanProvider(a.plans_dir, a.compute_sec).provide(a.hint1, a.hint2)
    print("source:", where)
    print("steps %d, exit %s, gates %s, %.1fs" % (len(plan["steps"]), bool(plan.get("exit")), plan.get("gates"), time.time() - t0))


if __name__ == "__main__":
    main()
