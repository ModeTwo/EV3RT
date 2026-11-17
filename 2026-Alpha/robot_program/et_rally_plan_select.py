"""ETラリー専用: 計画の受け取りと、ロボットだけで選ぶ予備(2026-11-15)。

本番(--mission full)の流れ:
  1. ロボットがヒントを読んで PC へ送る(services/strategy_exchange.py)。
  2. PC は、経路の道具の計画(steps・exit・gates など、ファイルで走るときと同じ形)を丸ごと送り返す
     (planFormat = PLAN_FORMAT)。ロボットはそれを received_plan.json に書き、ファイルの計画と同じように走る。
  3. PC から届かなかったとき(つながらない・時間切れ・計算できない)は、ロボットが自分で読んだヒントから配置の鍵を出し、
     前もって作った計画(PLANS_DIR の plan_<鍵>.json か plans.zip の中)を探す。なければ、予備の計画(スタートから
     ガレージへ行くだけ、どの配置でも脚に当たらない)で走る。

前もって作った計画の置き場所: 環境変数 ET_RALLY_PLANS_DIR、なければ ~/et_rally_plans。
2026-11-17(ユーザー: 別のラズパイでも同じに動かしたい): リポジトリの 2026-Alpha/et_rally_plans/plans.zip(devSS で配る)を
先に見る。順番は ET_RALLY_PLANS_DIR → リポジトリの et_rally_plans → ~/et_rally_plans(どれかで見つかればそれを使う)。
  plan_<鍵>.json を並べるか、plans.zip(中に plan_<鍵>.json)を置く。鍵は shared_communication/et_rally_layouts.py。
"""
import json
import os
import time
import zipfile
from pathlib import Path

PLAN_FORMAT = "curve_plan_v1"
PROJECT_ROOT = Path(__file__).resolve().parents[1]
RUN_LOGS = PROJECT_ROOT / "run_logs"
GARAGE_ONLY_PLAN = PROJECT_ROOT / "robot_program" / "tests" / "plan_garage_only_1114.json"


REPO_PLANS_DIR = PROJECT_ROOT / "et_rally_plans"


def plans_dir():
    return Path(os.environ.get("ET_RALLY_PLANS_DIR") or (Path.home() / "et_rally_plans"))


def plans_dirs():
    """探す順の置き場所。"""
    out = []
    if os.environ.get("ET_RALLY_PLANS_DIR"):
        out.append(Path(os.environ["ET_RALLY_PLANS_DIR"]))
    out += [REPO_PLANS_DIR, Path.home() / "et_rally_plans"]
    return out


def validate_plan(plan):
    """経路の道具の計画として最低限の形か。合わなければ ValueError。"""
    if not isinstance(plan, dict):
        raise ValueError("plan must be an object")
    steps = plan.get("steps")
    if not isinstance(steps, list) or not steps:
        raise ValueError("plan has no steps")
    for i, st in enumerate(steps):
        if not isinstance(st, dict) or st.get("type") not in ("move", "arc", "turn"):
            raise ValueError("plan step %d is not move/arc/turn" % i)
    if plan.get("exit") is not None and not isinstance(plan["exit"], dict):
        raise ValueError("plan exit must be an object")
    return plan


def write_plan(plan, name):
    """計画をファイルに書く(書き終えてから置き換える)。戻り値: パス。"""
    RUN_LOGS.mkdir(parents=True, exist_ok=True)
    path = RUN_LOGS / name
    tmp = path.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(plan, ensure_ascii=False), encoding="utf-8")
    tmp.replace(path)
    return path


def store_received_plan(plan, mission_id=None):
    """PC から届いた計画を書いて、パスを返す。記録用に時刻つきの写しも残す。"""
    validate_plan(plan)
    path = write_plan(plan, "received_plan.json")
    try:
        write_plan(plan, "received_plan_%s_%s.json" % (time.strftime("%Y%m%dT%H%M%S"), mission_id))
    except OSError:
        pass
    return path


def gates_from_context(context):
    """ロボットが読んだヒントから、経路の道具の gates。読めなければ None。"""
    hint1 = getattr(context, "hint1", None)
    hint2 = getattr(context, "hint2_gate_info", None)
    if not hint2:
        # 2026-11-17(devTK で、PC の計画が届かずガレージへ直行した): 受信(received)のときヒント2は PC が解くので、
        # ロボットの context には暗号のままの hint2 しかなく、ここで None になって予備(ガレージだけ)を選んでいた。
        # 起動時に入れた4桁の復号キーがあれば、ここで解く。
        raw = getattr(context, "hint2", None)
        key = getattr(context, "decryption_key", None)
        if raw and key:
            try:
                from shared_communication.hint_decoder import decode_hint2
                hint2 = decode_hint2(raw, key)
                try:
                    context.hint2_gate_info = hint2
                except Exception:
                    pass
            except Exception as error:
                print(" -- ET rally: could not decode hint2 on the robot: %r" % (error,), flush=True)
                hint2 = None
    if not hint1 or not hint2:
        return None
    from shared_communication.et_rally_layouts import gates_from_hints
    try:
        return gates_from_hints(hint1, hint2)
    except ValueError:
        return None


def find_precomputed(gates):
    """前もって作った計画を探す。戻り値: (計画の dict, どこから) か (None, 理由)。"""
    from shared_communication.et_rally_layouts import layout_key
    key = layout_key(gates)
    name = "plan_%s.json" % key
    why = []
    for d in plans_dirs():
        f = d / name
        try:
            if f.exists():
                return validate_plan(json.loads(f.read_text(encoding="utf-8"))), str(f)
            z = d / "plans.zip"
            if z.exists():
                with zipfile.ZipFile(str(z)) as zf:
                    try:
                        data = zf.read(name)
                    except KeyError:
                        why.append("%s not in %s" % (name, z))
                        continue
                return validate_plan(json.loads(data.decode("utf-8"))), "%s:%s" % (z, name)
            why.append("not in %s" % d)
        except (OSError, ValueError, zipfile.BadZipFile) as error:
            why.append("%s unreadable in %s (%r)" % (name, d, error))
    return None, "%s: %s" % (name, "; ".join(why))


def select_on_robot(context):
    """PC の計画がないときに、ロボットだけで計画を選ぶ。戻り値: (パス, 説明)。パスは必ず返す(最後は予備の計画)。"""
    gates = gates_from_context(context)
    if gates is not None:
        plan, where = find_precomputed(gates)
        if plan is not None:
            plan = dict(plan, gates=plan.get("gates") or gates)
            return write_plan(plan, "selected_plan.json"), "precomputed %s" % where
        why = where
    else:
        why = "hints not readable"
    plan = json.loads(GARAGE_ONLY_PLAN.read_text(encoding="utf-8"))
    if gates is not None:
        plan["gates"] = gates   # 脚の近くの点・マーカーを使わないため(経路は配置によらない)
    return write_plan(plan, "selected_plan.json"), "garage only (%s)" % why


def plan_gates(path):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8")).get("gates")
    except (OSError, ValueError):
        return None
