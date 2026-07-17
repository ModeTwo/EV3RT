"""無線通信デバイス上で動作するヒント受信・最適経路返送UDPサーバー。"""

from __future__ import annotations

import argparse
import json
import logging
import socket
from dataclasses import dataclass
from pathlib import Path

from .planner import PlannerConfig, RoutePlanner
from .protocol import parse_hint_set


@dataclass
class RobotSession:
    """1台の走行体から受信したヒントを、両方がそろうまで一時保存する。

    属性:
        hint1: ヒントカード1から読み取った赤ゲートの位置文字列。
            未受信の場合は ``None`` を保持する。
        hint2: パスワードで復号したヒントカード2の青・黄ゲート位置文字列。
            未受信の場合は ``None`` を保持する。
    """

    hint1: str | None = None
    hint2: str | None = None


def configure_logging(log_file: str | None) -> None:
    """画面表示と任意のファイル出力を行うログ機能を初期化する。

    引数:
        log_file: ログを書き込むファイルのパス。``None`` または空文字列なら、
            ファイルには保存せず画面だけに表示する。

    戻り値:
        なし。

    例外:
        OSError: 指定されたログファイルを作成または開けない場合。
    """
    handlers: list[logging.Handler] = [logging.StreamHandler()]
    if log_file:
        handlers.append(logging.FileHandler(log_file, encoding="utf-8"))
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
        handlers=handlers,
    )


def send_json(sock: socket.socket, address: tuple[str, int], payload: dict) -> None:
    """辞書をUTF-8のJSONへ変換し、指定した走行体へUDPで送信する。

    引数:
        sock: 送信に使用するUDPソケット。
        address: 送信先を表す ``(IPアドレス, UDPポート番号)``。
        payload: ACK、エラー、または経路情報として送信する辞書。

    戻り値:
        なし。

    例外:
        TypeError: ``payload`` にJSONへ変換できない値が含まれている場合。
        OSError: UDP送信に失敗した場合。
    """
    sock.sendto(json.dumps(payload, ensure_ascii=False).encode("utf-8"), address)


def run_server(args: argparse.Namespace) -> None:
    """ヒントを受信し、両方がそろった走行体へ計算済み経路を返送する。

    送信元IPアドレスごとにヒントを保持するため、別の走行体から届いた
    ヒント同士を誤って組み合わせない。ヒント1とヒント2がそろうまでは
    受信確認だけを返し、そろった時点で検証と経路計算を行う。

    引数:
        args: サーバーの起動設定。次の属性を使用する。
            ``config`` は経路計算設定JSONのパス、``bind`` は待受IPアドレス、
            ``port`` は待受UDPポート、``allowed_robot`` は受信を許可する
            走行体IP、``once`` は経路を1回返送した後に終了するかを表す。

    戻り値:
        なし。通常は受信待ちを継続し、``once`` が有効な場合だけ返送後に終了する。

    例外:
        OSError: ソケットの作成、待受、受信または送信に失敗した場合。
        FileNotFoundError: 経路計算設定JSONが見つからない場合。
    """
    config = PlannerConfig.load(args.config)
    planner = RoutePlanner(config)
    # 同じ無線通信デバイスへ複数の送信元から届いても混ざらないよう、IP別に保持する。
    sessions: dict[str, RobotSession] = {}
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.bind, args.port))
    logging.info("無線通信デバイス経路サーバーを開始: %s:%d", args.bind, args.port)
    logging.info("外部PCやインターネットへ接続せず、走行体との通信だけに使用してください")
    try:
        while True:
            raw, address = sock.recvfrom(65535)
            robot_ip = address[0]
            if args.allowed_robot and robot_ip != args.allowed_robot:
                logging.warning("許可されていない送信元を無視: %s", address)
                continue
            try:
                # 走行体が送ったUTF-8のJSONを復元し、対応しているヒント種別か確認する。
                message = json.loads(raw.decode("utf-8"))
                message_type = message.get("type")
                if message_type not in ("hint1", "hint2"):
                    raise ValueError(f"未対応メッセージtype: {message_type!r}")
                value = str(message["value"]).strip()
                session = sessions.setdefault(robot_ip, RobotSession())
                if message_type == "hint1":
                    session.hint1 = value
                else:
                    session.hint2 = value
                logging.info("%sから%sを受信: %s", robot_ip, message_type, value)
                # 再送制御ができるよう、経路計算より先に受信済みのヒント種別を応答する。
                send_json(sock, address, {"type": "ack", "received": message_type})
                if session.hint1 is None or session.hint2 is None:
                    continue
                # 2種類のヒントがそろった時だけ、形式とゲート配置を検証して経路を作る。
                hints = parse_hint_set(session.hint1, session.hint2)
                route = planner.plan(hints)
                send_json(sock, address, route)
                logging.info("%sへ最適経路を返送: %s", robot_ip, json.dumps(route, ensure_ascii=False))
                if args.once:
                    return
            except (KeyError, TypeError, ValueError, UnicodeError, json.JSONDecodeError) as error:
                logging.exception("受信データまたは経路計算でエラー")
                send_json(sock, address, {"type": "error", "message": str(error)})
    finally:
        sock.close()


if __name__ == "__main__":
    default_config = Path(__file__).with_name("planner_config.json")
    parser = argparse.ArgumentParser(description="ETロボコン2026 無線通信デバイス経路サーバー")
    parser.add_argument("--bind", default="0.0.0.0", help="待受IPアドレス")
    parser.add_argument("--port", type=int, default=50000, help="ヒント受信UDPポート")
    parser.add_argument("--allowed-robot", default=None, help="受信を許可する走行体IP")
    parser.add_argument("--config", default=str(default_config), help="経路計算設定JSON")
    parser.add_argument("--log-file", default="wireless_device.log", help="実行ログの保存先ファイル")
    parser.add_argument("--once", action="store_true", help="経路を1回返送したら終了")
    parsed = parser.parse_args()
    configure_logging(parsed.log_file)
    run_server(parsed)
