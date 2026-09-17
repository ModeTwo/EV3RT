# ETロボコン本番 実行手順 v2

## 1. 対象と本番投入条件

この手順は、現在の`2026-Alpha`でRightまたはLeftコースの全工程を実行する当日用手順である。走行体は`--mission full`、PCは3周分のETラリー戦略計算を使用する。

現在のスタート～LAPは次の構造である。

```text
RunByGyro（公式PDFの黒線中心POINTS、0～5100mm）
  → TraceLine（最終直線途中から青LAPまで）
  → Bottle Delivery以降
```

次をすべて満たさない場合は本番版として使用しない。

- [ ] 本番で使うPC版とRaspberry Pi版が同じ内容である。
- [ ] `python alpha.py <course> --mission full --check-tree`が終了コード0で完了する。
- [ ] `WARNING: skipped unimplemented features`が表示されない。
- [ ] 本番と同じ機体・バッテリー・コースでLAP単体を走らせ、赤線が一度も緑へ出ない。
- [ ] 5100mm付近でライントレースへ切り替わり、黒線を捕捉する。
- [ ] 青LAPを検出して停止または次工程へ進む。
- [ ] 緊急停止担当者とPC担当者を決めている。

スタート～LAPのv12は作成時点で実機未検証である。上記LAP単体確認が本番投入前の必須残課題である。

## 2. 役割と端末

| 担当 | 端末 | 役割 |
|---|---|---|
| PC担当 | PC端末A | Hint受信、3周分の経路計算、Strategy送信、PCログ保存 |
| 走行体担当 | Raspberry PiへSSH接続した端末B | 走行体起動、4桁キー入力、走行ログ監視 |
| 緊急停止担当 | 端末Bを操作できる位置 | 異常時に`Ctrl+C`、停止確認 |

PCとRaspberry Piの双方で`2026-Alpha`をカレントディレクトリにして実行する。Raspberry Piの標準配置場所は次を想定する。

```sh
cd /home/pi/EV3RT/2026-Alpha
```

## 3. コード版の固定と確認

コードを変更した後は、確認済み一式をPCとRaspberry Piの両方へ配置する。走行開始直前にコードを編集しない。

Git管理された環境では、双方で次を実行してコミットIDを記録する。

```sh
git rev-parse --short HEAD
git status --short
```

- [ ] PCとRaspberry PiのコミットIDが一致する。
- [ ] `git status --short`に意図しない差分がない。
- [ ] 本番で使用する未コミット差分がある場合、PCとPiで対象ファイルの内容が一致することを別途確認した。

現在のスタート～LAP設定を確認する。

```sh
grep -n "start_lap_mode\|start_lap_later_turn_distance_scale\|start_lap_third_turn_distance_scale\|start_lap_second_turn_start_advance_mm\|start_lap_third_turn_start_delay_mm\|start_lap_line_trace_from_mm" robot_program/config.py
```

本番候補v12の値は次のとおり。

```text
start_lap_mode = profile
start_lap_later_turn_distance_scale = 1.0
start_lap_third_turn_distance_scale = 1.0
start_lap_second_turn_start_advance_mm = 0.0
start_lap_third_turn_start_delay_mm = 0.0
start_lap_line_trace_from_mm = 5100.0
```

## 4. 起動前点検

### 通信・PC

- [ ] PCとRaspberry Piが同じ競技用ネットワークへ接続されている。
- [ ] Raspberry PiのIPアドレスを確認した。
- [ ] PCからRaspberry PiへSSH接続できる。
- [ ] PCからRaspberry PiのTCP 50000番へ接続できる構成である。
- [ ] TCP 50000番を別プログラムが使用していない。
- [ ] PCのファイアウォール設定を確認した。
- [ ] PC側`strategy_logs/`とPi側`run_logs/`へ書き込める。

### 走行体

- [ ] バッテリー残量を確認した。
- [ ] 左右タイヤ、アーム、配線に緩みがない。
- [ ] カラーセンサー、タッチセンサー、ジャイロ、カメラ、左右モーターが接続されている。
- [ ] カラーセンサーの高さと向きが確認済み状態から変わっていない。
- [ ] タイヤ表面に大きな汚れや付着物がない。
- [ ] スタート位置はアーム先端基準で合わせ、車軸中心までの100mm補正を含む現設定と一致する。
- [ ] Left／Rightのコースを声に出して相互確認した。
- [ ] コース上と周囲に工具、人、ケーブルがない。

## 5. 走行前の構成確認

Pi側端末Bで、本番コースに合わせてどちらかを実行する。このコマンドはデバイスを開かず、走行しない。

Rightコース:

```sh
python alpha.py right --mission full --check-tree
```

Leftコース:

```sh
python alpha.py left --mission full --check-tree
```

- [ ] 終了コードが0である。
- [ ] Python例外がない。
- [ ] `WARNING: skipped unimplemented features`がない。
- [ ] ツリーに`start_to_line_trace`、`trace_line_to_lap_gate`、後続工程が表示される。

警告または例外があれば本番起動へ進まない。

## 6. 本番起動順

### 手順1: PC側を先に起動する

PC端末Aで、`RASPBERRY_PI_IP`を実際のIPアドレスへ置き換えて実行する。

```sh
python -m wireless_device --host RASPBERRY_PI_IP --port 50000 --et-rally-laps 3 --planner-runner wireless_device/et_rally_runner.py --strategy-log-dir strategy_logs
```

北山hotspot

```sh
python -m wireless_device --host 192.168.137.191 --port 50000 --et-rally-laps 3 --planner-runner wireless_device/et_rally_runner.py --strategy-log-dir strategy_logs
```

Pi側プログラムがまだ起動していない間の接続再試行は正常である。PCプログラムを終了せず、そのまま待機させる。

### 手順2: Pi側で全工程を起動する

Rightコース:

```sh
python alpha.py right --mission full
```

Leftコース:

```sh
python alpha.py left --mission full
```

`configured`、`lap`、`rally-drive`は本番全工程では使用しない。

起動直後に次の形式で自動ログの保存先が表示される。

```text
[run log] Recording: /home/pi/EV3RT/2026-Alpha/run_logs/run_....log
```

表示されない場合は走行を開始しない。

### 手順3: PC接続を確認してから4桁の復号キーを入力する

Pi側は先にTCPサーバーを起動し、`PC TCP connection confirmed`が出るまで待つ。先行起動したPCプログラム側の`Connected to robot`も確認する。接続後に初めてキー入力が表示される。接続待ち中はデバイスを初期化せず、Hint送信・5秒の応答待ちも開始しない。中断はCtrl+C。

Pi側へ当日提示された4桁キーを入力する。表示されたキーをもう一人が読み合わせ、正しい場合だけ`y`で確定する。間違えた場合は`n`で再入力する。

### 手順4: 初期化を確認する

走行体は、アーム上端、アーム下端、デバイス値リセット、タッチ待ちの順で進む。

- [ ] アームが上下端で停止した。
- [ ] 異音がない。
- [ ] 初期化例外がない。
- [ ] `waiting for touch`相当の待機状態になった。
- [ ] Pi側に`Strategy server listening on 0.0.0.0:50000`が表示された。
- [ ] PC側に`Connected to robot 192.168.137.191:50000`が表示された。

異常があればタッチセンサーを押さず、`Ctrl+C`で終了する。

Strategyサーバーは4桁キー入力前に起動し、接続した同じサーバーを初期化・タッチ待ち・走行中も使用する。接続確認だけではHintやStrategyを送信しない。Hint1とHint2が揃うまで接続を維持して待機する。

PC側に`Connected to robot`が出ない場合はタッチセンサーを押さない。別のPC端末で次を実行する。

```powershell
Test-NetConnection -ComputerName 192.168.137.191 -Port 50000
```

`TcpTestSucceeded : True`を確認してから次へ進む。Pi側の待受け確認は次を使う。

```sh
ss -ltnp | grep ':50000'
```

### 手順5: スタート位置を最終確認する

- [ ] コースと起動引数のLeft／Rightが一致する。
- [ ] 機体が規定の開始位置と開始方位にある。
- [ ] アームが想定した下端状態にある。
- [ ] PC側プログラムが接続再試行中または接続済みである。
- [ ] PC側の接続が成功済みで、TCP 50000番の確認が完了している。
- [ ] 緊急停止担当者が端末Bを操作できる。

### 手順6: タッチセンサーを押す

スターターの指示に従ってタッチセンサーを押す。押した後は機体へ触れない。

## 7. 走行中の確認

### スタート～LAP

- RunByGyroが黒線中心POINTSを目標に5100mmまで走る。
- 最終直線途中で`trace_line_to_lap_gate`が開始する。
- TraceLineが黒線へ収束し、青LAPを検出する。

緑へ少しでも出る、ピン・壁へ向かう、転倒する、ケーブルを引く、異音がする場合は直ちに`Ctrl+C`で停止する。

### Strategy通信

PC側の目安:

```text
Connected to robot ...
Response sent mission=... type=STRATEGY
```

Pi側の目安:

```text
Strategy server listening on ...:50000
Hints queued mission=...
Hints sent mission=... attempt=1
Strategy received mission=... laps=3 commands=...
```

`Strategy received`が出るまで、ETラリー開始位置で走行体が待つことがある。5秒タイムアウト、計算エラー、不正SEQの場合はETラリーを開始しない。

## 8. 緊急停止と再起動

### 緊急停止

Pi側端末Bで次を押す。

```text
Ctrl+C
```

PC側だけを止めても走行体の即時停止にはならない。停止後、左右モーターが回っていないことを目視確認する。

### 再起動する場合

1. 左右モーター停止を確認する。
2. PC側とPi側の両プログラムを終了する。
3. コースから走行体を回収する。
4. 原因と停止位置を記録する。
5. バッテリー、配線、センサー、タイヤ、コード版を再確認する。
6. PC側を先に起動し、Pi側、キー入力、初期化、配置、タッチの順を最初から繰り返す。

途中状態からプログラムだけ再開しない。

## 9. 終了後のログ回収

1. 走行体が完全に停止したことを確認する。
2. Pi側を`Ctrl+C`で終了する。
3. PC側を`Ctrl+C`で終了する。
4. Pi側の最新ログを確認する。

```sh
ls -lt run_logs | head
```

5. 最新ログを別名へ変更せず、そのままPCへ保存する。
6. PC側`strategy_logs/`の同じ走行回を保存する。
7. 走行日時、Left／Right、使用コミットID、バッテリー状態、停止位置、手動介入を記録する。

`run_logs/`は`.gitignore`対象でありGitへ追加しない。`strategy_logs/`も走行記録として保管し、機密情報や当日データを確認せずGitへ追加しない。

## 10. 当日使用しないコマンド・設定

- `--mission lap`: LAP単体確認用
- `--mission rally-drive`: ETラリー走行・通信単体確認用
- `--strategy-file`: 固定SEQ試験用
- `et_rally_strategy_source="file"`: 固定plan互換試験用
- `--check-tree`: 構成確認用。これ自体では走行しない
- 未検証の`--planner`または`--planner-runner`

## 11. 現在の残課題

- スタート～LAP黒線中心軌道v12は実機未検証。本番前にLAP単体合格が必要。
- 全工程で青を見逃した場合、スタート～LAPのTraceLineが青検出を待ち続ける。青を通過しても工程が進まない場合は緊急停止する。
- 実行環境には`etrobo_python`、`py_trees`、`simple_pid`、`py_etrobo_util`など実機依存パッケージが必要。PC開発環境だけでは`alpha.py`を実走起動できない場合がある。

## 12. 一行チェック

```text
同版確認 → 機体点検 → fullツリー確認 → PC起動 → Pi起動 → 4桁キー確認 → 初期化確認 → コース/位置確認 → タッチ → 監視 → Ctrl+C停止 → ログ回収
```
