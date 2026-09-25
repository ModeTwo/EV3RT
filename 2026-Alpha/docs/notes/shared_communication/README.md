# Hint送信・SEQ受信の通信部品

取得・計算・実行の各担当者は、まず[担当者向け組込みガイド v2](INTEGRATION_GUIDE_v2.md)を参照してください。変更箇所、データの渡し方、周回単位の扱い、段階別試験をまとめています。

## 責務と担当者との接続口

通信部はTCP送受信とHint2復号を担当します。ヒント撮影、経路計算、走行指令実行は担当しません。

- 起動処理：alpha.py起動時に4桁キーを入力・確認し、`context.decryption_key`へ保存する。
- 読取担当：同じRaceContextの`context.hint1`へHint1、`context.hint2`へ暗号化されたHint2を設定する。
- 通信部：両方が非空になった周期で通信スレッドへ渡し、Hint2を復号する。復号値は`context.hint2_gate_info`へ反映する。
- 通信部：PCへはHint1と復号済みHint2を一度だけ送信する。復号キーと暗号文字列は送信しない。
- 計算担当：ZIP提供の経路計算は`WirelessDeviceApplication.prepare_strategy`へ組込み済み。PC起動設定で選択した`laps`（1～3）と、受信した`hint1`、`hint2`、`course`を使用する。
- 通信部：計算関数の戻り値をJSON化して即送信する。関数内でソケットやjson.dumpsを扱う必要はない。
- 実行担当：`context.strategy_status == "ready"` のとき `context.strategy` を使う。No.11が到達前に受信済みでも保存する。

通常は計算ランナーを指定せず、組込み済みの計算を使用する。計算プログラム全体を更新・比較するときは`--planner-runner`で独立したランナーを選択でき、通信コードの変更は不要。詳細は[PC側ETラリー経路計算](../wireless_device/README.md)を参照する。
組込み済み計算は絶対方位`target_heading_deg`、距離`distance_mm`を含む`move`／`turn`ステップを返す。
通信部は非空list、各要素が非空dict、JSON化可能かまで検査する。通行可能な経路であることは保証しない。

## 通信だけを試す（実機不要）

2026-Alphaをカレントディレクトリにして、端末1：
```sh
python -m shared_communication.demo_robot --hint1 "11,22" --hint2 "BASE64_HINT2" --key "1234"
```

端末2：
```sh
python -m wireless_device --host 127.0.0.1 --strategy-file shared_communication/example_strategy.json
```

PCの接続ログを確認後、端末1でEnter。両Hintの送信→固定JSONの返信→受信したcontextの表示を確認できる。
このモードはカメラ・モーター・走行プログラムを起動しない。
example_strategy.jsonは通信試験用であり、競技経路ではない。実機走行に使用しない。

## 実際の担当関数をつなぐ

PC（組込み済み経路計算、3周）：
```sh
python -m wireless_device --host RASPBERRY_PI_IP --et-rally-laps 3
```

`WirelessDeviceApplication.prepare_strategy`が、Hint文字列・コース・PCで選択した周回数をZIP提供の計算本体へ渡す。走行体の旧HINTS payloadに含まれる`laps`は互換性のため受信するが、PC側の選択を上書きしない。PC側ではキー入力・Hint2復号を行わない。
実装構成と単体テストは[PC側ETラリー経路計算](../wireless_device/README.md)を参照する。

ETラリー準備を除外し、無線通信デバイスと受信SEQ走行だけを実機確認する場合は、走行体を`--mission rally-drive`で起動する。手順は[ETラリー走行・無線通信単体試験](../robot_program/ET_RALLY_DRIVE_STANDALONE_v1.md)を参照する。

解析用JSONログも残す場合は`--strategy-log-dir strategy_logs`を追加する。ログは走行指示をメモリからTCP送信した後に複製し、実行時の入力には使用しない。

ラズパイ：従来どおり`python alpha.py left`（またはright）。ETラリー有効時は起動直後に4桁キーを入力する。
`enable_et_rally=True` かつ `et_rally_laps > 0` の場合、競技ミッション開始時にTCPサーバーを開始する。
キャリブレーション・タッチ待ちの間はまだ接続待受けを開始していない。PCは接続まで自動再試行する。
Hint取得までの既存Featureと移動経路は変更していない。読み取りがダミーの場合は担当者がcontextを設定するまで送信しない。ETラリー工程は、受信した全周回SEQを一度だけ実行する構成へ更新済み。

走行体側の`RaceConfig.et_rally_strategy_source`は、競技時は`"received"`にする。この設定では受信した全周回SEQを`context.strategy`から一度だけ実行する。従来の固定planを使う試験では`"file"`に変更する。固定planモードはTCPサーバーを起動せず、No.11の受信待ちも行わない。

## 設定

RaceConfig：
- `strategy_host = "0.0.0.0"`
- `strategy_port = 50000`
- `strategy_timeout_s = 5.0`
- `et_rally_strategy_source = "received"`（競技用）または`"file"`（固定plan互換）
- `et_rally_plan_path = None`（固定plan互換時の既定JSON）

Hintが揃ってからの全体上限は5秒。計算時間も含む。
初回送信＋再送2回まで、2秒間隔で応答待ちの要求を再送する。ACKはHINTS受信確認だけであり、計算完了ではない。
PCは同一missionId・同一payloadの計算結果（またはエラー）を最大16件キャッシュする。再送で再計算しない。
TCP断では再接続するが、全体上限と最大送信回数はリセットしない。

タイムアウト、PCの計算エラー、待受け失敗は `strategy_status="failed"`。
No.11はまずStopNowで停止し、failedならFAILUREを返して周回を開始しない。
LOCAL経路計算への切替は未実装。通信側が架空の代替経路を生成したり、空SEQを成功扱いすることはない。
ready/failed確定後の遅延応答は採用しない。

## データ形式

4バイト符号なしビッグエンディアンの本文バイト数＋UTF-8 JSON。本文上限1MiB。
TCPの分割・連結受信に対応する。破損・スキーマ不一致は接続を閉じて破棄する。

共通フィールド：
- messageType: HINTS / ACK / STRATEGY / ERROR
- missionId: ラズパイが生成するuint32
- sequenceNumber: 送信側ごと・ミッション内の送信番号uint32
- sequenceVersion: 今回は1固定。再計算は行わない。
- mode: 今回はPC固定
- timestamp: Unix秒。タイムアウト判定には同期不要なtime.monotonicを使用
- payload: HINTSはhint1/復号済みhint2/course/旧laps、STRATEGYはselectedLaps/strategy、ERRORはerror
- crc32: uint32のJSON整数

CRC対象はcrc32自身を除く本文を、キー昇順・空白なし・ensure_ascii=True・NaN禁止でJSON化したUTF-8バイト列。
CRC32は破損検出であり、認証・暗号化ではない。

## 安全な接続

直接TCPは信頼できるネットワークで使用する。第三者が接続できるネットワークにはポートを公開しない。
SSHトンネルを使う場合はラズパイのstrategy_hostを127.0.0.1に設定：
```sh
ssh -L 50000:127.0.0.1:50000 user@RASPBERRY_PI_IP
python -m wireless_device --host 127.0.0.1
```

カメラ映像送信用X11はこのデータ通信には不要。

## ファイル

- shared_communication/protocol.py：フレーム化、JSON、CRC、最低限の検証
- shared_communication/hint_decoder.py：通信スレッドで実行するHint2復号
- robot_program/services/strategy_exchange.py：ラズパイの通信スレッドとBT側poll
- robot_program/services/strategy_tree.py：元の工程順を保持するBTラッパー
- robot_program/features/receive_strategy.py：停止して受信結果を待つNo.11
- wireless_device/transport.py：PC接続・受信・担当関数呼出し・返信
- wireless_device/__main__.py：PC側起動入口
- shared_communication/demo_robot.py：ハードウェア不要のラズパイ側試験
- shared_communication/test_exchange.py：プロトコル・TCP往復テスト

```sh
python -m unittest shared_communication.test_exchange -v
```
