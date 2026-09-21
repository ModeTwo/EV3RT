"""Run with: python -m wireless_device --host ROBOT_IP."""

import argparse
import importlib
import json
import logging
import os
import threading
from pathlib import Path

from shared_communication.protocol import validate_strategy
from .application import WirelessDeviceApplication
from .transport import run_client


class FixturePlanner:
    # 通信試験専用。Hintから経路を計算したと誤認しないよう明示指定時だけ使用する。
    def __init__(self, path):
        self.strategy = validate_strategy(json.loads(Path(path).read_text(encoding="utf-8")))

    def calculate(self, hints):
        return self.strategy


def main():
    parser = argparse.ArgumentParser(description="PC hint/strategy TCP client")
    parser.add_argument("--host", required=True)
    parser.add_argument("--port", type=int, default=50000)
    parser.add_argument("--strategy-file", help="TEST ONLY: return a fixed JSON command list")
    parser.add_argument("--planner", help="Calculation callback as module:function")
    parser.add_argument(
        "--planner-runner",
        help=("Replaceable planner Python runner. Relative paths are resolved from "
              "2026-Alpha; env: ET_RALLY_PLANNER_RUNNER"),
    )
    parser.add_argument(
        "--strategy-log-dir",
        default=os.environ.get("ET_RALLY_STRATEGY_LOG_DIR"),
        help=("Optional JSON log directory. Also configurable with "
              "ET_RALLY_STRATEGY_LOG_DIR"),
    )
    parser.add_argument(
        "--et-rally-laps",
        type=int,
        choices=(1, 2, 3),
        default=os.environ.get("ET_RALLY_LAPS", "2"),
        help="Laps selected by the PC (default: 2; env: ET_RALLY_LAPS)",
    )
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    selected_backends = sum(bool(value) for value in (
        args.strategy_file, args.planner, args.planner_runner))
    if selected_backends > 1:
        parser.error(
            "Choose only one of --planner-runner, --planner, or --strategy-file")
    if args.strategy_file:
        logging.warning("TEST MODE: fixed strategy; not a calculated route")
        calculate = FixturePlanner(args.strategy_file).calculate
    elif args.planner:
        # 担当者の関数を差し込むだけでよく、通信側へアルゴリズムを記述しない。
        module_name, separator, function_name = args.planner.partition(":")
        if not separator:
            parser.error("--planner must be module:function")
        calculate = getattr(importlib.import_module(module_name), function_name)
        if not callable(calculate):
            parser.error("--planner must refer to a callable")
    else:
        app = WirelessDeviceApplication(planner_runner=args.planner_runner)
        calculate = app.prepare_strategy
    stop = threading.Event()
    try:
        run_client(
            args.host,
            args.port,
            calculate,
            stop,
            strategy_log_dir=args.strategy_log_dir,
            selected_laps=args.et_rally_laps,
        )
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()


if __name__ == "__main__":
    main()
