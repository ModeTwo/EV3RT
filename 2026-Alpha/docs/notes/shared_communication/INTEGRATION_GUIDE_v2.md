# Hint取得・経路計算・経路実行 担当者向け組込みガイド v2

更新日：2026-09-07  
対象：2026-AlphaのHint／SEQ TCP通信実装  
本資料は説明資料です。読取・計算・走行アルゴリズムを追加するものではありません。

## 1. 最初に：各担当者がつなぐ場所はここ

| 担当 | 実行場所・主な変更ファイル | 通信との接続口 | 担当者が実装しなくてよいもの |
|---|---|---|---|
| ヒントカード取得 | ラズパイ・robot_program/features/read_hint.py。移動中に読む場合は該当Feature | 同じcontextのhint1とhint2へ文字列を保存 | PCへの送信、JSON化、CRC、再送 |
| 経路計算 | PC・自身の計算モジュール。既存のwireless_device/strategy_planner.py等の利用も可 | calculate(hints)へ辞書が渡される。走行指令のlist[dict]を返す | TCP受信、JSON解析、ラズパイへの送信 |
| 経路実行 | ラズパイ・robot_program/features/execute_strategy.py | ready確認後にcontext.strategyを参照し、PCが生成した全周回SEQを一度実行 | TCP受信、JSON解析、PC接続待ち |
| 通信・統合 | shared_communication/、wireless_device/transport.py、No.11、strategy_tree.py等 | Hint2復号、復号値のContext保存、上記3者の間を接続 | 認識、経路生成、モーター制御 |

各担当が別々にRaceContextを作成しないこと。Featureへ渡されたcontextをそのまま使う。
PCとラズパイではメモリを共有しない。PCの関数引数はTCP経由で復元した別の辞書である。

## 2. 全体の処理順

| 順番 | 処理 | 実施する側 |
|---|---|---|
| 1 | alpha.py起動時に4桁復号キーを入力・確認し、Contextへ保存 | 走行体起動処理 |
| 2 | 通常のキャリブレーション・タッチ待ちを実施 | 既存起動処理 |
| 3 | 競技ミッション開始時にTCP待受けを開始。PCは接続を再試行できる | 通信部 |
| 4 | Hint1を取得しcontext.hint1へ保存 | 取得担当 |
| 5 | 暗号化されたHint2を取得しcontext.hint2へ保存 | 取得担当 |
| 6 | 両方が非空になった周期で通信スレッドへ復号要求を渡す | ラズパイ通信部 |
| 7 | Hint2を復号し、context.hint2_gate_infoへ反映して復号済みHintを送信 | ラズパイ通信部 |
| 8 | 受信したHint辞書で担当者のcalculate関数を呼ぶ | PC通信部 |
| 9 | 経路計算を行い、指令リストをreturnする | 計算担当 |
| 10 | 戻り値をJSONとして返信 | PC通信部 |
| 11 | context.strategyとselected_rally_lapsへ保存し、strategy_statusをreadyにする | ラズパイ通信部 |
| 12 | No.11で停止し、readyなら次へ進む。pendingなら待つ | 既存受信Feature |
| 13 | No.12で、選択周回分を含む全走行指令を先頭から一度実行 | 実行担当 |

Hint取得と計算完了の間、ラズパイは既存の後続工程を継続できる。
No.11へ到達する前に結果が届いても保持される。No.11で初めて送信を開始する構成ではない。
カメラ画像そのものではなく、読み取った文字列をPCへ送る。

## 3. ヒントカード取得担当の作業

### 3.1 保存する値

取得した文字列を、Featureが受け取ったcontextへ設定する。

```python
# 画像から取り出した未復号の文字列を保存する。
context.hint1 = hint1_raw_text
context.hint2 = hint2_raw_text
```

これは保存箇所を示す例。hint1_raw_text／hint2_raw_textは担当者の認識処理で取得する。
Hint1とHint2が別の工程・別の時刻で揃っても問題ない。

- 未取得時はNoneのままにする。
- 空文字やダミー値で「取得済み」にしない。
- Hint2の暗号文字列を、そのまま`context.hint2`へ保存する。取得担当は復号しない。
- 復号済み値は通信部が`context.hint2_gate_info`へ保存する。取得担当はこのフィールドを書き換えない。
- 同じカードの古い認識結果をHint2へコピーしない。カード種別・新しい撮影結果かを読取側で確認する。
- カメラスレッドから値を受け取る場合も、最終的なcontextへの反映はBT側に寄せると扱いやすい。
- 両Hintを格納した後は同一ミッション中に書き換えない。現在の通信部は最初の組を送信対象として固定する。

### 3.2 既存ReadHintCardを使う場合

[既存Hint読取Behavior](../../../robot_program/behaviours/hint_reader.py)は、非空のQR文字列をcontextへ保存する部品である。
[No.6](../../../robot_program/features/read_hint.py)のPendingFeatureを置き換える場合、呼び出しは次の形。

```python
from ..behaviours.hint_reader import ReadHintCard

# build_read_hint(context, config, hint_number)内のrootへ追加する。
root.add_children([
    ReadHintCard(
        name=f"read_hint{hint_number}",
        hint_number=hint_number,
        context=context,
    )
])
```

この部品は読取位置への移動・探索距離上限・正しいカードかの保証をすべて提供するものではない。
その部分は取得担当が必要に応じて組み合わせる。

移動Feature内ですでに取得・保存する場合、No.6でも無条件に再読取しないよう、取得済み時の扱いを取得担当内で統一する。
同じヒントを別Featureから同時に更新しない。

### 3.3 自動送信が働く条件

- 通常のalpha.py→tree_builder経由で構築したミッションである。
- RaceConfig.enable_et_rallyがTrue。
- RaceConfig.et_rally_lapsが1～3。
- BTのtickが継続している。
- 同じcontext.hint1とcontext.hint2が両方非空である。
- alpha.py起動時に入力した4桁キーがcontext.decryption_keyへ保存されている。

Featureだけを単独で生成・実行しても、それだけでは通信の監視ラッパーは付かない。
単独通信テストには後述のdemo_robotを使う。

### 3.4 完了確認

- [ ] Hint1だけの間は送信されない。
- [ ] Hint2格納後、通信スレッドで復号される。
- [ ] 復号値がcontext.hint2_gate_infoへ保存され、PCのhints["hint2"]にも同じ値が渡る。
- [ ] Hint1/Hint2を逆順に取得しても、正しいフィールドへ保存できる。
- [ ] 未読取・誤認識時に適当な値を保存しない。
- [ ] 独自のsocket.sendやCRC計算をFeatureへ追加していない。

## 4. 経路計算担当の作業

### 4.1 入力と戻り値

PCの呼出し口は、引数1個の通常の同期関数。ZIP提供の経路計算はこの呼出し口へ組込み済み。

```python
# 通信部が実際に呼ぶ組込み済みの入口。
strategy = WirelessDeviceApplication().prepare_strategy({
    "hint1": "25,35",
    "hint2": "53,54/12,22",
    "course": "left",
    "laps": 3,
})
```

計算後はPythonのlist[dict]をreturnする。
json.dumpsの結果のstr、JSONファイルのパス、通信の外側のpayload辞書を返さない。
JSON化と送信は通信部が行う。

現在の通信部は、戻り値が非空listで各要素が非空dictかを検査する。
None、空リスト、例外、JSON化不能な値はERROR返信となる。
NumPyの配列・整数等を使用する場合、最後に標準Pythonのlist・dict・int・floatへ変換する。

### 4.2 起動する

PCの2026-Alphaをカレントディレクトリにし、通常は次のように起動する。

```sh
python -m wireless_device --host RASPBERRY_PI_IP
```

`--planner-runner`を省略すると、現行ロジックを接続した`wireless_device/et_rally_runner.py`が独立プロセスで呼ばれる。
計算プログラム全体を更新・比較する場合は、共通JSON契約を満たすランナーを用意して`--planner-runner wireless_device/new_planner_runner.py`のように指定する。通信コードの書換えは不要。
現行計算本体のAPIが変わらない更新は`wireless_device/et_rally_planner/`だけ、APIや配置も変わる更新はランナーだけを変更する。
組込み済み計算の構成は`wireless_device/README.md`を参照する。

### 4.3 キー入力・復号・計算時間・再送

- キー入力はalpha.py起動時に走行体側で完了し、PC側では入力しない。
- Hint2復号は走行体側の通信スレッドが担当する。20msのBT周期では復号しない。
- 計算担当へは復号済みHint2だけが渡り、復号キーや暗号文字列は送信されない。
- --planner-runnerは標準入力から1個のJSON objectを読み、標準出力へ走行指令listのJSONだけを返す。ログは標準エラーへ出す。
- 従来の--plannerは軽量な同期関数を同一PCプロセス内で試すために残している。計算プログラム全体の差替えには--planner-runnerを使用する。
- 現在の制限は、両Hintが揃ってからSEQを受信するまで合計5秒。計算時間だけの上限ではない。
- PC側には処理中の計算関数を5秒で強制停止する機能はない。5秒を超える結果はラズパイ側が採用しない。
- 同じミッションの再送にはキャッシュ済み結果を返す。計算関数の例外もキャッシュ対象。
- この構成は1回のHint組→1回のSEQ生成用。Hint更新後の再計算、各周回のPC再指示は別途設計が必要。

### 4.4 完了確認

- [ ] 固定入力から指令リストを返す単体テストができる。
- [ ] Hint1、復号済みHint2、course、lapsを正しく利用する。
- [ ] JSON文字列ではなくlist[dict]を返す。
- [ ] 実行担当と合意した指令スキーマを満たす。
- [ ] 予算時間内に計算を終え、人の入力待ちをしない。
- [ ] 空経路や計算失敗を成功として返さない。

## 5. 経路実行担当の作業

### 5.1 読み出す場所

[No.12](../../../robot_program/features/execute_strategy.py)のbuild_execute_strategy(context, config)が実装箇所。
[ETラリー工程](../../../robot_program/phases/et_rally.py)は次の順で一度ずつ呼び出す。

1. No.11：受信結果待ち
2. No.12：PCが選択した周回数分の全SEQを実行

実際の周回数はPC側の`--et-rally-laps`で決まり、返信の`selectedLaps`へ入る。走行体側は同じSEQを`config.et_rally_laps`回繰り返さない。

`RaceConfig.et_rally_strategy_source="file"`の場合は、従来の固定plan JSONを同じNo.12で実行する。このモードではNo.11とPC通信を省略する。

### 5.2 ツリー構築時にはSEQがまだ存在しない

build_execute_strategyは起動時に呼ばれる。
その時点のcontext.strategyは空なので、そこでfor command in context.strategyとして子ノードを作ると、空のままのツリーになる。

実装方針：

- 起動時：contextを保持する実行ノードを作る。
- 実行開始時（initialise等）：readyを確認し、受信済みSEQを読み出す。
- 内容を検証し、既存Behaviorを使った実行単位を全件準備する。
- 各tick：現在の指令を進め、未完了ならRUNNING、周回完了ならSUCCESS。
- 無効な指令・異常時：モーター停止と失敗通知を行う。

動的なサブツリーを作る場合も、受信後・該当周回開始時に構築する。毎tick全指令を作り直さない。
socket.recv、json.loads、無限while、長いtime.sleepをupdateへ書かない。

### 5.3 読み出しと状態

```python
# 実行開始時に参照する箇所の例。
if context.strategy_status == "ready":
    received_commands = context.strategy
```

received_commandsはすでにPythonのlist。json.loadsは不要。
context.strategy_statusを書き換えてreadyにしたり、受信リストをpopして後続指令を壊したりしない。
実行中の指令番号は担当Behaviorのself.index等へ保持する。

| strategy_status | 意味 | 実行側の扱い |
|---|---|---|
| idle | 両Hintがまだ揃っていない | 実行しない |
| pending | 送信要求済み、結果待ち | 実行しない |
| ready | 通信側が受信結果を格納済み | 指令詳細を検証してから実行 |
| failed | 通信・計算失敗等 | 実行せず停止方針に従う |

readyは「通信上受け取れた」を意味し、経路の安全性・実行可能性の保証ではない。

### 5.4 既存Behaviorを使う

- 旋回：SpinAround
- 方位維持走行：RunByGyro
- 指定距離での終了判定：IsDistanceEarned
- 停止：StopNow

既存gyro_drive.pyやplotter.pyを書き換えるのではなく、No.12側で受信指令を既存部品の引数へ変換する。
通信境界は時計回り正だが、既存ジャイロ部品はruntime.courseを含む座標規則を持つ。
受信angle_degを無条件でSpinAroundのtargetへ渡さない。絶対／相対方位、Left／Rightの鏡像変換を実行担当と計算担当で合わせる。

### 5.5 完了確認

- [ ] 起動時に空だったSEQを、受信後に読み出せる。
- [ ] No.12は受信した全SEQを先頭から一度だけ実行する。
- [ ] 不明な指令、NaN/無限値、範囲外の距離・角度を走行前に拒否する。
- [ ] 共通Behaviorを再利用し、停止・中断時にモーター出力を残さない。
- [ ] 全周回分のSEQを周回ごとに繰り返していない。
- [ ] Left／Right双方で角度と進行方向を確認する。

## 6. 計算担当と実行担当で先に合意するもの

ここは通信実装だけでは決まらない。以下を確定してから実機走行へ進む。

| 項目 | 現状・確認内容 |
|---|---|
| 指令種別とキー | `type=move`は`target_heading_deg`と`distance_mm`、`type=turn`は`target_heading_deg`を使用 |
| 周回の区切り | PCが選択した周回数分を一つのlistへ格納し、走行体は全件を順番に一度だけ実行 |
| 絶対／相対角度 | `target_heading_deg`はETラリー開始方位を0度とする絶対方位 |
| 単位・符号 | 距離mm、角度degree、通信境界は時計回り正 |
| Right変換 | PCかNo.12か、どちらで鏡像化するかを1箇所に決める |
| 周回間・終端移動 | 補正、次周回開始位置、ETラリー終了位置への移動を誰が生成・実行するか |
| 速度・制御値 | 送信指令に含めるか、実行側の設定値を使うか |
| 無効値・異常時 | モーター停止、FAILURE後の全体制御、再実行の可否 |

周回区切りの提案例（未採用・未実装、走行用にそのまま使わない）：

```json
[
  {"lap": 1, "type": "turn", "angle_deg": 90},
  {"lap": 1, "type": "straight", "distance_mm": 100},
  {"lap": 2, "type": "turn", "angle_deg": -90}
]
```

この案ならNo.12がlap_numberと一致する指令だけ選ぶ。通信側がlapを自動付与・検証するわけではない。
既存example_strategy.jsonにはlapがなく、あくまで送受信確認専用である。

## 7. 確認手順：段階を分けて組み込む

### A. 通信だけ（同じPCの2端末・モーターなし）

すべて2026-Alphaから実行する。

端末1：
```sh
python -m shared_communication.demo_robot --hint1 "11,22" --hint2 "BASE64_HINT2" --key "1234"
```

端末2：
```sh
python -m wireless_device --host 127.0.0.1 --strategy-file shared_communication/example_strategy.json
```

接続後に端末1でEnter。strategy_statusがready、commandsが2なら固定SEQの往復成功。
端末1は1回で終了する。端末2のConnection retryは終了済みの相手を探しているためで、受信成功が取り消されたわけではない。端末2はCtrl+Cで止める。
BASE64_HINT2には、指定した4桁キーで復号できる実際のOpenSSL形式Hint2を指定する。

### B. PCの計算関数だけ接続（モーターなし）

端末2は`--strategy-file`を外し、組込み済み計算を使って起動する。

```sh
python -m wireless_device --host 127.0.0.1
```

端末1には実際に解析できるHint組を渡す。
まず通信無しの関数単体試験、その後TCP経由で同じ結果が返ることを確認する。

### C. PCと実ラズパイ間のTCP通信（まだモーターなし）

ラズパイ：
```sh
python -m shared_communication.demo_robot --host 0.0.0.0 --hint1 "11,22" --hint2 "BASE64_HINT2" --key "1234"
```

PC：
```sh
python -m wireless_device --host RASPBERRY_PI_IP --strategy-file shared_communication/example_strategy.json
```

127.0.0.1は実行中の機器自身を指す。PCから実ラズパイへ直接接続するときはラズパイのIPを使う。
第三者が接続可能なネットワークへ公開しない。SSHトンネル利用時は[通信README](README.md)を参照。

### D. 取得担当との結合

読取担当が格納した2文字列を、PC担当が受信することを確認する。
alpha.pyを使う試験は実機走行を伴う可能性があるため、担当者が試験範囲・安全確保を行う。
固定SEQ試験を実モーター実行につながない。No.12の実機駆動はこの段階では切り離す。

### E. 実行担当との結合

まず既知の小さな指令リストを注入したNo.12単体試験で、周回選択・角度変換・停止を確認する。
その後、実際の計算結果を使って取得→計算→実行を結合する。
計算単体・通信単体・走行単体のどこで失敗したか分かるログを残す。

## 8. 現在の制約と、成功とみなしてはいけないもの

- No.6とNo.12には現在PendingFeatureが残る。通信追加だけでは読取・走行は完成しない。
- 全体上限は両Hint成立から5秒。No.11到達時から5秒ではない。設定変更は通信・統合担当と調整する。
- 通信失敗時のLOCAL計算は未実装。PCから結果が来なければ架空のSEQで走らせない。
- No.11のFAILUREはPythonプロセス終了や恒久的な非常停止を意味しない。親BTが次のtickでどう扱うか、統合担当が中止・再始動方針を確認する。
- 現在はSEQ受信後に通信接続を閉じる。周回報告、2回目のHint組、追加の経路計算を同じミッションで往復する実装ではない。
- 再試行では新しい走行プログラム／新しいContextと通信サービスを作る。statusだけidleへ戻して使い回さない。
- CRC32は認証ではない。外部からの不正な指令を防ぐにはネットワーク制限・SSHトンネル等が必要。
- readyとテスト用JSONの受信は、「正しい経路計算」「安全な実機走行」の合格判定ではない。

## 9. 3担当での最終持寄りチェック

- [ ] 取得担当：実データのHint1/Hint2の組と、未取得・誤認識時の扱い
- [ ] 計算担当：呼出しモジュール・関数名、入力例、出力例、計算時間
- [ ] 実行担当：対応指令一覧、周回の区切り、角度・距離・速度の解釈
- [ ] 計算＋実行：同じ出力例を同じ走行内容として説明できる
- [ ] 統合担当：No.11失敗時の全体停止、再試験方法、ネットワーク・ポート設定
- [ ] 全員：通信試験用の固定SEQを競技経路として実行しない

通信実装のテスト：
```sh
python -m unittest shared_communication.test_exchange shared_communication.test_tree_exchange -v
```
