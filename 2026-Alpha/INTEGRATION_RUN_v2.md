# RE・AT・TO連続走行版 v2（2026-09-05）

`alpha.py`の標準実行を、スタート→LAP→ボトル認識・捕捉走行→Hint1→Hint2→停止へ接続しました。
採用元はRE=`gyro_line_0826.py`、AT=`bottle_catch.py`、TO=`tantou3.py` + `sample2.py`です。
走行体Bの値は使用していません。各担当の単独スクリプトは変更していません。
以下はソフトウェア実装・模擬試験済みの状態で、実機の走行成功・捕捉成功は未確認です。

## 起動

2026-Alpha内で、従来の実機依存ライブラリがある環境から実行します。

```text
python alpha.py left
python alpha.py right
```

現在の標準`--mission configured`は`RaceConfig`の工程フラグに従います。RE→AT→TO接続試験を行う場合は`--mission hint2`を明示し、初期化・タッチ待ちを一度だけ実施してHint2取得後に停止します。
ガレージ・ETラリー・ET相撲には進みません。
元TOのHint2後の動作（相対-25度→600mmライントレース→停止）まで行う場合：

```text
python alpha.py left --mission hint2
python alpha.py left --mission hint2-return
```

実機やカメラを開かずに構成確認する場合：

```text
python alpha.py left --check-tree
python alpha.py right --mission hint2-return --check-tree
```

上記の構築確認は終了コード0が正常です。ただしimportには従来のOpenCV等が必要です。
`--mission configured`はRaceConfig工程スイッチを使います。`--mission full`は全工程を明示的に有効化します。未実装区間は警告を表示して動作せず、次の工程へ進みます。
`hint2`と`hint2-return`はRE/AT/TOを必ず一続きに実行する専用構成で、従来の工程スイッチは参照しません。

QRデコーダー`zxingcpp`がない場合は、タッチ待ちやモーター駆動へ進む前にエラー終了します。
両Hintの生文字列とボトル色は`RaceContext`へ保存し、完了時に`HINT_COMPLETE`ログで表示します。
復号・PC送信・経路生成は今回の範囲に含みません。

## 周期は1か所

`robot_program/timing.py`の次の値だけを変更し、プログラムを再起動します。

```python
CONTROL_INTERVAL_SEC = 0.02
```

全工程のETRobo dispatch、PIDのsample_time、制御用LPF、カメラ処理ループの待機目標へ同じ値を使用します。
RE30ms・TO50msの工程別指定は撤去しました。原本の単独スクリプトは対象外です。
カメラ機器のFPS（QRモードは従来5fps）、画像復号の所要時間、静止待ち0.5秒などは制御周期とは別です。
20msの設定で画像が必ず20msごとに更新されるわけではありません。
周期統一によりRE/TOの元環境とPID応答が変わり得るため、ゲイン・停止位置は実機確認が必要です。

## 実行順と調整場所

距離設定は `robot_program/integration_settings.py` の `IntegrationSettings`。
今回は以下の設定を実際のFeatureへ接続済みです。

|順番|区間|設定・実行値|
|---|---|---|
|1|RE スタートから青ライン検知|`start_to_lap_gate.py`。指定RE版の区間・値を維持|
|2|AT ゲートを越える前進|`at_gate_forward_mm=100`、PWM60|
|3|AT ボトル認識位置まで後退|`at_recognition_reverse_mm=200`、PWM-60|
|4|AT 停止して色認識|赤・青・黄の同じ色を、新しい3フレームで確認。面積150以上|
|5|AT 捕捉・引渡し位置までトレース|`at_to_transfer_trace_mm=460`、PWM60、輝度75|
|6|AT→TO 完全停止・境界記録|`AT_TO handoff`に距離・方位・色を記録|
|7|TO 最初の旋回|引渡し時を局所0度として絶対90度|
|8|TO 最初の接近|`to_first_black_limit_mm=565`、局所0度を目標としたジャイロ走行|
|9|TO Hint1への正対・取得|局所絶対0度へ旋回→0.5秒静止→停止読取|
|10|TO Hint1後の直進|`to_after_hint1_mm=385`、局所0度・PWM60|
|11|TO 旋回・Hint2へのトレース|局所絶対90度→`to_hint2_trace_mm=1000`、PWM60、輝度65|
|12|TO Hint2への正対・取得|相対+25度→0.5秒静止→停止読取→完全停止|
|13|任意の元TO出口動作|相対-25度→`to_exit_trace_mm=600`、PWM50→完全停止|

TOの旋回PWMは`to_spin_min_power=55`～`to_spin_max_power=60`。各ゲインは採用元の有効値を保持しています。
ATは独立したアーム捕捉判定を持たず、アームを下げた状態で460mm進む間にボトルを捕捉・運搬する元方式です。
色認識成功だけでは物理的な捕捉を保証しません。

### 接続位置を直すとき

1. AT停止位置が想定からずれるなら、`at_to_transfer_trace_mm`と`AT_TO transfer trace`ログを確認。
2. AT停止位置が合っていてTOの最初の接近終端がずれるなら、`to_first_black_limit_mm`と`TO first approach`ログを確認。
3. AT終了とTO開始の向きは一致する前提。追加の方向補正は入れていません。

両方の距離を同時に変更せず、どの区間でずれたかを分けて確認してください。
TO開始時にジャイロ・エンコーダーはリセットせず、境界の方位から局所絶対角度を変換します。
ログの距離は累積移動量で、コース上の絶対座標ではありません。

## 元コードとの意図的な差

- REとATに重複していた青ライン検知はREだけで実施。ATは100mm前進から開始。
- TOのHint1前ライントレースは原本でコメントアウトされており、停止読取を採用。
- TOの最初の接近・Hint1後直進は原本で色検知が無効のため距離で終了。「黒線接近上限」という設定名でも黒検知は行いません。
- 原本コメントの1200mmに対して有効値は1000mm。コメントではなく実際の値を採用。
- 共通走行部品を使用。停止後のブレーキ解除と中断時PWMゼロを追加。
- ボトルは同一色・異なるフレームで3回確認。旧コードの同一フレーム再カウントを防止。
- QR取得はセッション番号とフレームIDで新規読取を確認。Hint2ではHint1と同じ文字列も拒否。
- QRワーカーはVideoごとに一つを維持し、モード切替時の1秒joinとスレッド増殖を解消。終了時に停止要求を送る。
- 各AT/TO移動に30秒、ボトル認識に15秒、各QR読取に20秒の失敗上限を追加。`motion_timeout_sec`、`bottle_timeout_sec`、`qr_timeout_sec`で調整可能。
- 標準はHint2取得直後に停止。元TO出口動作は`hint2-return`指定で保持。

## 確認範囲・保存

実際のBehavior Tree・共通制御を模擬デバイスで実行し、左右両コースのRE→AT→TO完了、任意出口、距離調整、前後進・制動、認識失敗時停止、古いQR除外を確認します。
QRワーカーは実際のVideoクラスのメソッドを画像取得なしで実行し、復号中のモード切替、古い結果の破棄、ワーカー再利用・停止を検査します。
これらは実走・画像認識精度の検証ではありません。

EV3RT直下でのテスト：

```text
.venv\Scripts\python.exe -B -m unittest discover -s 2026-Alpha\robot_program\tests -t 2026-Alpha -v
```

変更前は`backups/pre_at_to_20260905_v2.zip`へ保存。復元時は新しいフォルダへ展開し、内容を比較してください。
v1バックアップ・原本単独スクリプトも保持しています。
