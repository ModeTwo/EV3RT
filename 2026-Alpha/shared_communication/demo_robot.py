"""Hardware-free robot-side exchange test; no motors or camera are opened."""

import argparse
import json
import logging
import time
from types import SimpleNamespace

from robot_program.services.strategy_exchange import StrategyExchange


def main():
    parser = argparse.ArgumentParser(description="Robot-side communication test only")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=50000)
    parser.add_argument("--hint1", required=True)
    parser.add_argument("--hint2", required=True)
    parser.add_argument("--key", required=True, help="Four-digit Hint 2 decryption key")
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO)
    context = SimpleNamespace(hint1=None, hint2=None, strategy=[], strategy_status="idle",
                              strategy_error=None, mission_id=None,
                              decryption_key=args.key, hint2_gate_info=None)
    exchange = StrategyExchange(args.host, args.port, args.timeout)
    exchange.start()
    try:
        # PC起動・接続を済ませてから、両Hintが揃うタイミングを手動で再現する。
        input("Start PC client, then press Enter to submit both hints: ")
        context.hint1, context.hint2 = args.hint1, args.hint2
        while context.strategy_status not in ("ready", "failed"):
            exchange.poll(context)
            time.sleep(0.02)
        print(json.dumps(vars(context), ensure_ascii=True, indent=2))
        return 0 if context.strategy_status == "ready" else 1
    finally:
        exchange.close()


if __name__ == "__main__":
    raise SystemExit(main())
