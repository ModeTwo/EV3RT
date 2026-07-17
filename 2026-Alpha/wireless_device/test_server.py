"""ヒント受信から経路返送までのUDPループバック結合テスト。"""

import argparse
import json
import socket
import threading
import time
import unittest
from pathlib import Path

from .server import run_server


class UdpServerTest(unittest.TestCase):
    def test_hints_are_acknowledged_and_route_is_returned(self) -> None:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
        probe.close()

        args = argparse.Namespace(
            config=str(Path(__file__).with_name("planner_config.json")),
            bind="127.0.0.1",
            port=port,
            allowed_robot="127.0.0.1",
            once=True,
        )
        server_thread = threading.Thread(target=run_server, args=(args,), daemon=True)
        server_thread.start()
        time.sleep(0.05)

        client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        client.bind(("127.0.0.1", 0))
        client.settimeout(5.0)
        server = ("127.0.0.1", port)
        try:
            client.sendto(json.dumps({"type": "hint1", "value": "25,35"}).encode(), server)
            self.assertEqual(json.loads(client.recvfrom(65535)[0])["type"], "ack")
            client.sendto(
                json.dumps({"type": "hint2", "value": "53,54/12,22"}).encode(),
                server,
            )
            received = [json.loads(client.recvfrom(65535)[0]) for _ in range(2)]
            route = next(message for message in received if message["type"] == "route")
            self.assertEqual(len(route["laps"]), 3)
        finally:
            client.close()
            server_thread.join(timeout=5.0)
        self.assertFalse(server_thread.is_alive())


if __name__ == "__main__":
    unittest.main()
