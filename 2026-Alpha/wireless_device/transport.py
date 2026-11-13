"""PC-side request processing independent of camera and robot packages."""

import json
import logging
import socket
from datetime import datetime, timezone
from pathlib import Path

from shared_communication.protocol import FrameReader, canonical, encode, make_message, validate_strategy

from shared_communication.heading_frame import HEADING_FRAME, planner_to_full_start

LOG = logging.getLogger(__name__)


class StrategyJsonLogger:
    """Optionally persist calculation input/output without driving execution."""

    def __init__(self, directory=None):
        self.directory = Path(directory) if directory else None

    @property
    def enabled(self):
        return self.directory is not None

    def write(self, request, planning_payload, result):
        if not self.enabled:
            return None
        self.directory.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now(timezone.utc)
        filename = "strategy_%010d_%s.json" % (
            request["missionId"], timestamp.strftime("%Y%m%dT%H%M%S_%fZ"))
        target = self.directory / filename
        temporary = target.with_suffix(".json.tmp")
        record = {
            "schemaVersion": 1,
            "loggedAt": timestamp.isoformat(),
            "missionId": request["missionId"],
            "requestPayload": request["payload"],
            "planningPayload": planning_payload,
            "response": result,
        }
        # 書込み途中のJSONを解析対象にしないよう、一時ファイル完成後に置換する。
        temporary.write_text(
            json.dumps(record, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        temporary.replace(target)
        return target


class RequestProcessor:
    def __init__(self, calculate, strategy_log_dir=None, selected_laps=2):
        # 経路計算は担当者が渡す関数へ委譲し、このモジュールでは実装しない。
        if type(selected_laps) is not int or not 1 <= selected_laps <= 3:
            raise ValueError("selected_laps must be an integer from 1 to 3")
        self.calculate = calculate
        self.cache = {}
        self.selected_laps = selected_laps
        self.strategy_logger = StrategyJsonLogger(strategy_log_dir)

    def write_log(self, request, result):
        planning_payload = dict(request["payload"], laps=self.selected_laps)
        try:
            path = self.strategy_logger.write(request, planning_payload, result)
            if path is not None:
                LOG.info("Strategy JSON log written: %s", path)
        except (OSError, TypeError, ValueError):
            # ログは解析用の複製であり、保存失敗によって走行指示の返信を止めない。
            LOG.exception("Strategy JSON log could not be written")

    def process(self, request):
        mission_id = request["missionId"]
        payload = request["payload"]
        fingerprint = canonical(payload)
        if mission_id in self.cache:
            old_fingerprint, result = self.cache[mission_id]
            if old_fingerprint != fingerprint:
                return make_message("ERROR", mission_id, 0, {"error": "Mission payload changed"})
            # 再送された同一ミッションでは高負荷な経路計算を繰り返さない。
            return result
        try:
            if not all(isinstance(payload.get(key), str) and payload[key]
                       for key in ("hint1", "hint2")):
                raise ValueError("Both hint strings are required")
            if payload.get("course") not in ("left", "right"):
                raise ValueError("Invalid course")
            # 周回数は無線通信デバイス側の設定を正とする。走行体から届く
            # 旧payloadのlapsは互換性のため受信できるが、経路計算には使用しない。
            planning_payload = dict(payload, laps=self.selected_laps)
            strategy = planner_to_full_start(
                validate_strategy(self.calculate(planning_payload)), payload["course"])
            result = make_message(
                "STRATEGY",
                mission_id,
                0,
                {"selectedLaps": self.selected_laps, "strategy": strategy,
                 "headingFrame": HEADING_FRAME},
            )
        except Exception as error:
            LOG.exception("Planning failed mission=%d", mission_id)
            result = make_message("ERROR", mission_id, 0, {"error": str(error)})
            planning_payload = dict(payload, laps=self.selected_laps)
        if len(self.cache) >= 16:
            del self.cache[next(iter(self.cache))]
        self.cache[mission_id] = (fingerprint, result)
        return result


def run_client(host, port, calculate, stop_event, strategy_log_dir=None, selected_laps=2):
    processor = RequestProcessor(
        calculate,
        strategy_log_dir=strategy_log_dir,
        selected_laps=selected_laps,
    )
    counters = {}
    while not stop_event.is_set():
        try:
            with socket.create_connection((host, port), timeout=1.0) as connection:
                LOG.info("Connected to robot %s:%d", host, port)
                connection.settimeout(0.2)
                reader = FrameReader()
                while not stop_event.is_set():
                    try:
                        chunk = connection.recv(65536)
                    except socket.timeout:
                        continue
                    if not chunk:
                        break
                    for request in reader.feed(chunk):
                        if request["messageType"] != "HINTS":
                            continue
                        # ACKは受信確認であり、経路の採用・計算完了ではない。
                        mission_id = request["missionId"]
                        if mission_id not in counters and len(counters) >= 16:
                            del counters[next(iter(counters))]
                        number = counters.get(mission_id, 0)
                        connection.sendall(encode(make_message(
                            "ACK", mission_id, number, {"for": "HINTS"})))
                        result = processor.process(request)
                        counters[mission_id] = number + 2
                        outgoing = dict(result, sequenceNumber=number + 1)
                        connection.sendall(encode(outgoing))
                        # 走行指示はメモリから先に送信し、JSONログは送信後に複製する。
                        processor.write_log(request, outgoing)
                        LOG.info("Response sent mission=%d type=%s",
                                 result["missionId"], result["messageType"])
        except (OSError, ValueError) as error:
            LOG.warning("Connection retry: %s", error)
        stop_event.wait(0.5)
