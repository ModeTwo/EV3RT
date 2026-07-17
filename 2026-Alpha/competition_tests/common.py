"""競技別テストで共通利用する行動木抽出・実機起動処理。"""

from __future__ import annotations

import argparse
import json
import signal
from pathlib import Path
from typing import Iterable

from py_trees.trees import BehaviourTree

import left_course_2026 as strategy
import sample as base


def build_isolated_tree(
    test_name: str,
    section_names: Iterable[str],
    require_hint2_password: bool = False,
) -> BehaviourTree:
    """本番ルートを保ったまま対象外工程を除外し、競技単体の行動木を作る。"""
    root = strategy.build_behaviour_tree()
    root.name = test_name
    by_name = {child.name: child for child in root.children}
    required_names = ["キャリブレーション", "スタート", *section_names, "end"]
    missing = [name for name in required_names if name not in by_name]
    if missing:
        available = ", ".join(by_name)
        raise KeyError(f"本番行動木に {missing} がありません。利用可能: {available}")

    # 選択した本番サブツリーは元のrootに接続したまま残し、別の親へ付け替えない。
    selected = [by_name[name] for name in required_names]
    for child in root.children:
        if child not in selected:
            child.parent = None
    root.children[:] = selected

    # パスワードを使わない競技では、キャリブレーション内のReadKeyだけを除外する。
    calibration = by_name["キャリブレーション"]
    if not require_hint2_password:
        for child in list(calibration.children):
            if isinstance(child, base.ReadKey):
                calibration.children.remove(child)
                child.parent = None

    # 本番終端の直前へ強制停止を追加する。既存サブツリーの親子関係は変更しない。
    stop = base.StopNow(name="unit test complete stop")
    stop.parent = root
    root.children.insert(len(root.children) - 1, stop)
    return root


def reset_test_state(clear_route: bool = True) -> None:
    """同じPythonプロセスで再試験しても前回結果を引き継がないよう共有状態を戻す。"""
    if strategy.g_wireless is not None:
        strategy.g_wireless.sock.close()
    strategy.g_wireless = None
    strategy.g_race_started_at = None
    strategy.g_hint1_sent = False
    strategy.g_hint2_sent = False
    if clear_route:
        strategy.g_route_laps = []
    base.g_key = None
    base.g_hint1 = None
    base.g_hint2 = None
    base.g_bottle_color = strategy.BottleColor.NONE


def add_runtime_arguments(parser: argparse.ArgumentParser, wireless: bool = False) -> None:
    """実機テスト共通のコマンドライン引数を追加する。"""
    parser.add_argument("--logfile", type=str, default=None)
    if wireless:
        parser.add_argument("--wireless-host", default="127.0.0.1")
        parser.add_argument("--wireless-port", type=int, default=50000)
        parser.add_argument("--route-listen-port", type=int, default=50001)


def configure_wireless(args: argparse.Namespace) -> None:
    """ヒント送信・経路受信を行うテスト用無線アダプターを設定する。"""
    strategy.g_wireless = strategy.WirelessStrategyDevice(
        host=args.wireless_host,
        port=args.wireless_port,
        listen_port=args.route_listen_port,
    )


def load_route_file(path: str) -> None:
    """通信を使わないETラリーテスト用にJSON経路を直接読み込む。"""
    message = json.loads(Path(path).read_text(encoding="utf-8"))
    if "laps" in message:
        route_laps = [
            [(float(theta), int(length)) for theta, length in lap["segments"]]
            for lap in message["laps"]
        ]
    else:
        segments = [(float(theta), int(length)) for theta, length in message["segments"]]
        route_laps = [segments, list(segments), list(segments)]
    if len(route_laps) != 3 or any(not lap for lap in route_laps):
        raise ValueError("ETラリーテストには空でない3周分の経路が必要です")
    if any(length <= 0 for lap in route_laps for _, length in lap):
        raise ValueError("すべての経路距離は1mm以上で指定してください")
    strategy.g_route_laps = route_laps


def run_on_raspberry_pi(tree: BehaviourTree, logfile: str | None) -> None:
    """sample.pyと同じデバイス構成・制御周期で、単体テスト行動木を実行する。"""
    base.g_course = 1
    base.setup_thread()
    signal.signal(signal.SIGTERM, base.sig_handler)
    try:
        etrobo = base.initialize_etrobo(backend="raspike_art")
        etrobo.add_handler(base.TraverseBehaviourTree(tree))
        etrobo.dispatch(interval=base.EXEC_INTERVAL, logfile=logfile)
    finally:
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if base.g_right_motor is not None:
            base.g_right_motor.set_power(0)
        if base.g_left_motor is not None:
            base.g_left_motor.set_power(0)
        if base.g_arm_motor is not None:
            base.g_arm_motor.set_power(0)
        base.cleanup_thread()
        if strategy.g_wireless is not None:
            strategy.g_wireless.sock.close()
        signal.signal(signal.SIGTERM, signal.SIG_DFL)
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        print(" -- unit test exiting...")
