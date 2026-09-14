"""Length-prefixed UTF-8 JSON messages with CRC32."""

import json
import struct
import time
import zlib

MAX_BYTES = 1024 * 1024


def canonical(message):
    # CRC対象はcrc32を除いたJSON。送受信双方でキー順・空白・文字表現を統一する。
    return json.dumps(message, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=True, allow_nan=False).encode("utf-8")


def make_message(kind, mission_id, number, payload):
    return {"messageType": kind, "missionId": mission_id,
            "sequenceNumber": number, "sequenceVersion": 1, "mode": "PC",
            "timestamp": time.time(), "payload": payload}


def encode(message):
    body = dict(message)
    body.pop("crc32", None)
    body["crc32"] = zlib.crc32(canonical(body)) & 0xffffffff
    data = canonical(body)
    if len(data) > MAX_BYTES:
        raise ValueError("Message too large")
    return struct.pack("!I", len(data)) + data


def decode(data):
    # JSONのNaNなどを禁止し、破損・他形式のメッセージは採用しない。
    def invalid_constant(value):
        raise ValueError("Invalid JSON constant: " + value)
    message = json.loads(data.decode("utf-8"), parse_constant=invalid_constant)
    if not isinstance(message, dict):
        raise ValueError("Expected JSON object")
    checksum = message.pop("crc32", None)
    if type(checksum) is not int or checksum != zlib.crc32(canonical(message)) & 0xffffffff:
        raise ValueError("CRC32 mismatch")
    for name, limit in (("missionId", 2**32), ("sequenceNumber", 2**32),
                        ("sequenceVersion", 2**16)):
        value = message.get(name)
        if type(value) is not int or not 0 <= value < limit:
            raise ValueError("Invalid " + name)
    if message.get("messageType") not in ("HINTS", "STRATEGY", "ACK", "ERROR"):
        raise ValueError("Invalid messageType")
    if message.get("mode") != "PC" or not isinstance(message.get("payload"), dict):
        raise ValueError("Invalid mode or payload")
    if type(message.get("timestamp")) not in (int, float):
        raise ValueError("Invalid timestamp")
    return message


class FrameReader:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, chunk):
        # TCPで分割された本文を次回recvへ持ち越す。複数件同時受信にも対応する。
        self.buffer.extend(chunk)
        messages = []
        while len(self.buffer) >= 4:
            size = struct.unpack("!I", self.buffer[:4])[0]
            if not 0 < size <= MAX_BYTES:
                raise ValueError("Invalid message length")
            if len(self.buffer) < size + 4:
                break
            data = bytes(self.buffer[4:size + 4])
            del self.buffer[:size + 4]
            messages.append(decode(data))
        return messages


def validate_strategy(strategy):
    # 走行指令の詳細スキーマは担当アルゴリズムとNo.12で定義する。
    # 現時点で空・非オブジェクト・JSON化不能のSEQは実行対象にしない。
    if not isinstance(strategy, list) or not strategy:
        raise ValueError("Strategy must be a non-empty list")
    if any(not isinstance(command, dict) or not command for command in strategy):
        raise ValueError("Each command must be a non-empty object")
    canonical(strategy)
    return strategy

