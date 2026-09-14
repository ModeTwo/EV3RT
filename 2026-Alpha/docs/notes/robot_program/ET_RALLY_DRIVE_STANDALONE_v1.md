# ETラリー走行・無線通信単体試験 v1

## 試験範囲

`rally-drive`は、ETラリー準備工程を実行せず、次だけを実機で確認する専用ミッションです。

1. 走行体が復号済みHint1・Hint2をPCへ送信する
2. 無線通信デバイスが経路を計算する
3. 走行体が全周回分の走行指示SEQを受信する
4. 受信SEQをBehavior Treeへ展開する
5. 旋回・直進を実機で実行する

QR撮影、Hint2復号、Bottle Delivery、ET相撲、FINISHは実行しません。カメラとQRデコーダーも起動しません。競技用の`rally`・`full`の処理は変更せず、`rally-drive`でのみ復号済み情報を起動引数から投入します。

## 実機の準備

- 走行体を、経路計算プログラムが前提とするETラリー周回開始位置へ置く。
- 走行体の向きを、計算プログラムの開始方位0度に合わせる。
- キャリブレーション中にジャイロがリセットされるため、向きを合わせてから起動する。
- アームと機体周囲がゲートや障害物へ接触しないことを確認する。
- SEQ受信直後に走行を開始するため、タッチ操作前にコース内から人と物を退避する。
- 最初はPC側を1周指定にし、方向・距離・停止操作を確認してから2周、3周へ進む。

## 起動手順

2026-Alphaをカレントディレクトリにします。

PC側を先に起動します。PCは走行体が待受けを開始するまで再接続します。

```sh
python -m wireless_device --host RASPBERRY_PI_IP --et-rally-laps 1 --strategy-log-dir strategy_logs
```

走行体側では、Hint1と復号済みHint2のゲート情報を指定します。

```sh
python alpha.py left --mission rally-drive --rally-hint1 "25,35" --rally-hint2-gate-info "53,54/12,22"
```

Rightコースは`left`を`right`へ変更します。Hint値は実際に試験したい配置へ置き換えます。`--rally-hint2-gate-info`には暗号文字列ではなく、復号後の青ゲート・黄ゲート情報を指定します。このモードでは4桁復号キーの入力はありません。

キャリブレーション完了後、タッチセンサーを押すと通信待受けを開始します。PCからSEQを受信すると、そのままETラリー走行を開始します。

## 構成だけ確認する

次のコマンドは実機、カメラ、通信ソケットを開きません。Hint引数も省略できます。

```sh
python alpha.py left --mission rally-drive --check-tree
```

ツリーの対象工程が`et_rally`だけで、その中が`receive_strategy`、`execute_received_strategy`の順になっていることを確認します。

## 自動結合試験

次のテストはモーターを動かさず、復号済みHint投入、走行体TCPサーバー、PC接続、現行経路計算、SEQ返信、走行体Contextへの保存までを一続きで確認します。Windows側で実行できます。

```sh
python -m unittest wireless_device.test_strategy_planner -v
```

受信SEQからBehavior Tree走行ノードへの展開は、`py_trees`等の走行体依存ライブラリが導入されたRaspberry Pi側で次を実行します。

```sh
python -m unittest robot_program.tests.strategy_execution_tests -v
```

実機動作そのものはモーター・ジャイロ・走行距離に依存するため、これらの自動試験だけでは合格にしません。自動結合試験の後に1周実機試験を行います。

## ログの確認

- 走行体：通常の`run_logs`と必要に応じた`--logfile`を確認する。
- PC：`--strategy-log-dir`を指定した場合、送信したHint、選択周回数、返信SEQがJSONへ複製される。
- `Strategy received`が出ず5秒で失敗する場合は、IPアドレス、TCP 50000番、PCファイアウォール、同一ネットワークを確認する。
- SEQ受信後に期待と逆方向へ旋回する場合は、開始位置・開始方位・Left／Right指定を先に確認する。

## 通常競技との差

| 項目 | `rally-drive` | `rally`／`full` |
|---|---|---|
| Hint1 | 起動引数で復号済み値を投入 | カメラで取得 |
| Hint2 | 起動引数で復号済み値を投入 | カメラで取得後、走行体で復号 |
| 4桁キー | 不要 | 起動時に入力 |
| カメラ | 起動しない | 起動する |
| ETラリー準備 | 実行しない | 実行する |
| PC経路計算・通信 | 実行する | 実行する |
| 受信SEQ走行 | 実行する | 実行する |


## 2026-09-13 ETラリー方位契約

送信SEQは全体スタート方向0度、左右コース正規化済みの方位角。
ラリー入口の規定方向は-90度。PCと走行体を同時更新する。
headingFrame=full-start-course-normalized-v1がない旧PCの応答は採用しない。

計算担当のet_rally_planner/、et_rally_runner.pyは変更不要。
計算ランナー出力は従来どおり「ラリー開始方向0度・数学座標の反時計回り正・Rightは幾何鏡映済み」。
transport.pyがheading_frame.pyを介して一度だけ変換する。
別ランナー、--planner、--strategy-fileも同じ入力角度契約に従う。
送信後の共通方位SEQを--strategy-fileへ再投入すると二重変換になるので使用しない。
計算担当が角度規約を変更するときは共有境界heading_frame.pyだけを協議して更新する。

全体走行は起動時ジャイロ0基準を維持し、途中でリセットしない。
--mission rally-driveはラリー開始位置かつ規定の内向きに置いて起動・初期化する。
ジャイロ0を全体方位-90度と対応させるため、実行直前のSEQコピーから固定初期方位を引く。
受信ContextのSEQ、ログは全体方位のまま。実測到着角は変換に使用しない。
位置だけでなく向きも規定どおり合わせ、初期化後に車体を回さない。
既存fileモードのplanは従来契約を保持し、この受信SEQ変換は適用しない。

テストスクリプトはローカル保持・Git除外。実行用plan_seed9392783.jsonは追跡を維持する。
実機での左右方位・配置精度確認は未実施。
