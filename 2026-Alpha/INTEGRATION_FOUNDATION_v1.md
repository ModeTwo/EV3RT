# alpha.py 統合基盤 v1（2026-09-05）

現在は基盤修正と移植準備の段階です。AT・TOの連続走行は未接続です。
走行体はB以外、採用元はRE=`gyro_line_0826.py`、AT=`bottle_catch.py`、TO=`tantou3.py` + `sample2.py`です。
`tantou4.py`の機体B向け値は使用しません。担当元ファイル・video.pyは変更していません。

## 実行前の構築確認

2026-Alpha内で、実機依存ライブラリのある環境から実行します。

```text
python alpha.py left --check-tree
python alpha.py right --check-tree
```

ツリーを表示するだけでカメラ・ETRobo・モーターを起動しません。ただしimportには従来のOpenCV等が必要です。
未実装工程がある現在は一覧を表示し、終了コード2を返します。通常起動も走行前に同じ検査で停止します。
構築成功は走行成功を意味しません。`PendingFeature`はFAILUREを返し、未実装区間を通過させません。

## 今回反映した内容

- alpha.pyが共有runtimeインスタンスを使用するよう修正。
- 実機参照を設定してからカメラ処理スレッドを開始。カメラ例外を周期処理へ伝播。
- 走行失敗、例外、Ctrl+C、終了時に左右とアームへ停止・制動を試行。一つのAPIが失敗しても残りを実行し、失敗理由を表示。
- 起動途中でもカメラ終了処理が可能。joinは2秒上限で、停止不能を表示。ハードウェアが応答しない場合の物理停止を保証するものではない。
- 旧start_to_lap_gate.pyの二重ノード登録による構築エラーを解消し、指定RE版のスタート部分へ整理。
- stop_in_garage.pyの参照漏れと終端制動を修正。目標輝度75は従来alpha定数からの暫定値で、ガレージ最新版・実測値は未確定。
- 既存相撲テストのimport漏れを修正。

## RE区間の採用値と差異

`start_to_lap_gate.py`は元コードのASTから対象ノードだけを抽出しました。
走行順はsquare（5区間）→100mm低速トレース→2460mmトレース→青検知です。

| ジャイロ区間 | 方位 | 距離mm | PWM |
|---|---:|---:|---:|
| edge_01 | 0 | 500 | 70 |
| edge_02 | -45 | 200 | 70 |
| edge_03 | -90 | 550 | 70 |
| edge_04 | -135 | 230 | 70 |
| edge_05 | -180 | 300 | 60 |

ゲイン・目標輝度65・距離は採用元に準拠。元のTraceLineに合わせ、RE区間では輝度LPFを無効化。
共通RunByGyro/TraceLineにPID sample_interval_secを追加し、REでは0.03、既存利用箇所では0.02を維持。
ただしalphaのdispatchは20msなので、30msのPID更新閾値は実際には約40ms毎の更新になり得ます。
元の30ms dispatchと同じ時間応答ではありません。AT/TO移植時に工程ごとのtick周期を設計・実測してから走行版とします。
RE元ファイルのlap3以降（ボトル・QR）は採用していません。共通制御への置換による終端出力等も実機確認対象です。

## AT→TOを調整するとき

設定場所は `robot_program/integration_settings.py`、参照口は `RaceConfig.integration`。
以下は**移植用の準備値で、現在のAT/TO走行には未接続**です。変更しても担当単独スクリプトは変わりません。

| 設定名 | 初期値mm | 調整する区間 |
|---|---:|---|
| at_gate_forward_mm | 100 | RE青検知後、ゲートを越える前進 |
| at_recognition_reverse_mm | 200 | ボトルを見るための後退 |
| at_to_transfer_trace_mm | 460 | 認識後からAT終了位置まで。引渡し位置を直す第1候補 |
| to_first_black_limit_mm | 565 | TO最初の黒線への接近上限。ATの終了位置が合っていてTO側でずれる場合に確認 |

AT終了とTO開始の向きは一致する前提で、追加の向き補正は設けません。
移植時はATの完全停止直後に `CaptureAtToHandoff` を配置し、`AT_TO handoff`ログへ累積距離・方位・ボトル色を記録します。
TOの局所絶対角度は `context.at_to.absolute_heading(元の角度)` で変換する準備ができています。
元のResetDeviceでエンコーダー・ジャイロを再初期化せず、引渡し時方位を局所0度として扱います。
これは現在のTO動作へまだ挿入していません。

位置ずれの切り分けは「AT停止位置→TO最初の旋回→TO黒線接近」の順に行い、一度に両方の距離を変えません。
ログの距離は累積移動量であり、コース上の絶対座標ではありません。

## 残作業

1. ATの100mm前進・200mm後退・赤青黄認識・460mm走行を移植し、設定とContextへ接続。
2. AT_TO境界記録を配置し、tantou3.pyの処理をHint1/2移動・取得へ移植。QR保存・古いQR除外も実装。
3. RE30ms、AT20ms、TO50msの工程周期と制御計算を整合させ、単独試験の挙動と照合。
4. ガレージ最新版・開始位置を確定。Hint2後の復帰動作と未実装の配送・ラリーを別途接続。
5. RE単独→AT単独→TO単独→RE+AT→RE+AT+TOの実機確認。

## テストと復元

EV3RT直下から実行：

```text
.venv\Scripts\python.exe -B -m unittest discover -s 2026-Alpha\robot_program\tests -t 2026-Alpha -v
```

基盤テストでは画像依存を置換し、実際のalpha.py・py_trees・共通制御を読み込んで構築と例外経路を検査します。
カメラ認識・実機制動・走行の試験ではありません。

変更前は `backups/pre_foundation_20260905_v1.zip` へ保存。まず新しいフォルダへ展開して内容を比較してください。
新規追加ファイルはZIP内のmanifestで区別します。既存画像3件の削除は今回の対象外です。
