"""Asynchronous robot-side TCP server; never blocks the control tick."""

import logging
import queue
import secrets
import socket
import threading
import time

from shared_communication.protocol import FrameReader, encode, make_message, validate_strategy
from shared_communication.hint_decoder import decode_hint2

from shared_communication.heading_frame import HEADING_FRAME

LOG = logging.getLogger(__name__)


class StrategyExchange:
    def __init__(self, host="0.0.0.0", port=50000, timeout=5.0, retry_interval=2.0):
        self.host, self.port = host, port
        self.timeout, self.retry_interval = timeout, retry_interval
        self.mission_id = secrets.randbits(32)
        self.outbox = queue.Queue(maxsize=1)
        self.inbox = queue.Queue(maxsize=32)
        self.decoded_hint2_inbox = queue.Queue(maxsize=1)
        self.stop_event = threading.Event()
        self.ready_event = threading.Event()
        self.connected_event = threading.Event()
        self.thread = None
        self.started_at = None

    def start(self):
        if self.thread is None:
            self.thread = threading.Thread(target=self._worker, daemon=True,
                                           name="strategy-exchange")
            self.thread.start()

    def wait_for_connection(self):
        """Wait before password/device startup; no request or response timer yet."""
        self.start()
        while not self.connected_event.wait(0.1):
            if self.stop_event.is_set() or not self.thread.is_alive():
                raise RuntimeError("Strategy server stopped before PC connection; check address/port")

    def close(self):
        # joinしない。BT周期を止めず、通信側の短いsocket timeoutで終了させる。
        self.stop_event.set()
        self.connected_event.clear()

    def poll(self, context, course="left", laps=3):
        # この関数はBTスレッドだけから呼び、RaceContextの更新もここへ限定する。
        try:
            context.hint2_gate_info = self.decoded_hint2_inbox.get_nowait()
        except queue.Empty:
            pass
        if context.strategy_status in ("ready", "failed"):
            return
        has_hint2 = bool(context.hint2_gate_info or context.hint2)
        if self.started_at is None and context.hint1 and has_hint2:
            if not context.hint2_gate_info and not context.decryption_key:
                context.strategy_status = "failed"
                context.strategy_error = "Decryption key is not configured"
                self.close()
                return
            submission = {
                "hint1": context.hint1,
                "course": course,
                "laps": laps,
            }
            if context.hint2_gate_info:
                # 走行単体試験では復号済み値を渡し、QR読取と復号を試験範囲から外す。
                submission["hint2_decoded"] = context.hint2_gate_info
            else:
                # 競技時の復号処理は20ms周期外の通信スレッドで行う。
                submission["hint2_raw"] = context.hint2
                submission["decryption_key"] = context.decryption_key
            self.outbox.put_nowait(submission)
            self.started_at = time.monotonic()
            context.mission_id = self.mission_id
            context.strategy_status = "pending"
            LOG.info("Hints queued mission=%d", self.mission_id)

        # 制限時間以降の応答は採用しない。LOCAL計算未実装のため失敗で止める。
        if self.started_at is not None and time.monotonic() - self.started_at >= self.timeout:
            context.strategy_status = "failed"
            context.strategy_error = "Strategy response timed out"
            self.close()
            return
        for _ in range(32):
            try:
                message = self.inbox.get_nowait()
            except queue.Empty:
                break
            if message["missionId"] != self.mission_id:
                continue
            kind, payload = message["messageType"], message["payload"]
            if kind == "ERROR":
                context.strategy_status = "failed"
                context.strategy_error = payload.get("error", "PC calculation failed")
                self.close()
                return
            if kind == "STRATEGY":
                if self.started_at is None or message["sequenceVersion"] != 1:
                    continue
                try:
                    if payload.get("headingFrame") != HEADING_FRAME:
                        raise ValueError("Strategy heading frame mismatch; update PC and robot together")
                    strategy = validate_strategy(payload.get("strategy"))
                    selected_laps = payload.get("selectedLaps")
                    if type(selected_laps) is not int or not 1 <= selected_laps <= 3:
                        raise ValueError("selectedLaps must be 1, 2, or 3")
                except ValueError as error:
                    LOG.warning("Invalid strategy ignored: %s", error)
                    continue
                context.strategy = strategy
                context.selected_rally_laps = selected_laps
                context.strategy_status = "ready"
                LOG.info("Strategy received mission=%d laps=%d commands=%d",
                         self.mission_id, selected_laps, len(strategy))
                self.close()
                return

    def _take_request(self):
        try:
            submission = self.outbox.get_nowait()
        except queue.Empty:
            return None
        try:
            if "hint2_decoded" in submission:
                gate_info = submission["hint2_decoded"]
            else:
                # PBKDF2とAESは通信スレッドで実行し、走行制御周期をブロックしない。
                gate_info = decode_hint2(
                    submission["hint2_raw"], submission["decryption_key"])
            self.decoded_hint2_inbox.put_nowait(gate_info)
            return make_message(
                "HINTS",
                self.mission_id,
                0,
                {
                    "hint1": submission["hint1"],
                    "hint2": gate_info,
                    "course": submission["course"],
                    "laps": submission["laps"],
                },
            )
        except (KeyError, ValueError, queue.Full) as error:
            self.inbox.put_nowait(make_message(
                "ERROR", self.mission_id, 0,
                {"error": "Hint 2 decode failed: " + str(error)}))
            return None

    def _worker(self):
        # 接続待ち、送受信、再接続は走行制御とは別スレッドで実施する。
        request = None
        attempts = 0
        last_sent = 0.0
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
                listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                listener.bind((self.host, self.port))
                listener.listen(1)
                listener.settimeout(0.1)
                self.port = listener.getsockname()[1]
                self.ready_event.set()
                LOG.info("Strategy server listening on %s:%d", self.host, self.port)
                while not self.stop_event.is_set():
                    if request is None:
                        request = self._take_request()
                    try:
                        connection, _ = listener.accept()
                    except socket.timeout:
                        continue
                    with connection:
                        self.connected_event.set()
                        connection.settimeout(0.1)
                        reader = FrameReader()
                        last_sent = 0.0
                        while not self.stop_event.is_set():
                            if request is None:
                                request = self._take_request()
                            try:
                                if (request is not None and attempts < 3
                                        and time.monotonic() - last_sent >= self.retry_interval):
                                    outgoing = dict(request, sequenceNumber=attempts)
                                    attempts += 1
                                    last_sent = time.monotonic()
                                    try:
                                        connection.sendall(encode(outgoing))
                                    except OSError:
                                        # 途中まで送ったフレームへ再送フレームを継ぎ足さない。
                                        break
                                    LOG.info("Hints sent mission=%d attempt=%d", self.mission_id, attempts)
                                chunk = connection.recv(65536)
                                if not chunk:
                                    break
                                for message in reader.feed(chunk):
                                    self.inbox.put_nowait(message)
                            except socket.timeout:
                                continue
                            except (OSError, ValueError, queue.Full) as error:
                                LOG.warning("Strategy connection closed: %s", error)
                                break
                    self.connected_event.clear()
        except OSError as error:
            # bind失敗などは制御側へ通知する。スレッド内だけで黙って終了しない。
            self.inbox.put_nowait(make_message("ERROR", self.mission_id, 0,
                                              {"error": str(error)}))
