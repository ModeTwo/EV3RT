# ET相撲 接近動画撮影テスト v2

## 構成

`alpha_sumo_approach_video_test_v2.py` は、2026-Alphaの`alpha.py`を土台にした試験用コピーです。alphaと同じ`initialize_etrobo`、`TraverseBehaviourTree`、シグナル停止処理を使用します。初期化は実際の`ArmUpDownFull`（アーム上げ→下げ）と`robot_program.behaviours.device_control.ResetDevice`、開始待ちはalphaの`IsTouchOn`をそのまま使用します。独自のIMU待機コールバックは廃止しました。

実行順は「アーム上げ→アーム下げ→ResetDevice→タッチ開始→動画準備→指定距離直進→停止・動画確定」です。走行・撮影だけを`RecordStraightApproach`ノードにまとめています。カメラワーカーはアーム校正後・タッチ待ち前に開始し、タッチ後からファイルへの記録を開始します。黒候補は映像とCSVへの記録用です。

## 起動

2026-Alphaリポジトリルートから実行します。

```console
python alpha_sumo_approach_video_test_v2.py right --distance-mm 100 --power 80
```

- `--distance-mm`: エンコーダ由来の直進目標。初回は安全空間で短くし、実測した開始位置と接触までの余裕に合わせて指定します。既定値500mmをそのまま使うことは推奨しません。
- `--power`: 基準PWM。既定80。ジャイロ補正で左右出力は概ね60〜100の範囲です。
- `--output-dir`: AVI/CSVの保存先。既定はスクリプト隣の`sumo_approach_video_runs/`。
- `--logfile`: ETRoboの機器ログ保存先。

上の100mmは操作例です。100mm以上の障害物のない空間を確保できない場合は実行せず、実際の停止余裕を測って指定距離を決めてください。

起動するとalphaと同じアーム上下の校正が行われます。`TraverseBehaviorTree initialization complete`、各校正ノードのログ、`ResetDevice.complete`、`IsTouchOn.waiting for touch...`の順に進みます。タッチセンサーDを押すと`-- touch start; waiting for video writer`、準備後に`-- run start ...`が出て直進します。旧版の独自ログ`waiting for IMU stationary ... samples=...`は出ません。

通常のalpha.pyが進む実機で試験版だけ静止待ちに留まったため、初期化を模倣していた独自ハンドラーを除去し、alphaと同じ実装・順序へ統一しました。静止フラグを無視して走行する処理は追加していません。

## 出力と範囲

- `*_approach.avi`: 入力640×480動画に検出枠、画像上の候補角、距離、ジャイロ、左右PWMを重ねたもの。
- `*_frames.csv`: フレームごとの時刻、距離、方位、出力、黒候補の有無・位置・形状値。

接近距離到達で停止し、動画を閉じます。実機でのモーター・カメラ・保存確認は未実施です。エンコーダ距離と実距離、PWM80時の速度と惰性、Pi上のOpenCV/ETRobo実行環境は未検証です。最初は車輪を浮かせた状態または障害物のない安全な範囲で起動手順を確認してください。

## 修正の確認範囲

機器登録、TraverseBehaviourTree、信号・例外時停止の定義がalpha.pyとASTで一致することを確認しました。模擬デバイスによる撮影直進ノードの実行で、動画準備前の停止、PWM80の直進、800mmでの停止、ファイル確定後の完了を確認しています。PiでのIMU待機解消、実距離、実動画保存は未確認です。


## カメラプレビュー

既定で`ET Sumo Approach Camera`ウィンドウを表示します。alphaのアーム上下・ResetDeviceの完了後、タッチ待ち中は生映像、走行中は保存AVIと同じ黒候補枠・距離・方位・左右PWM付き映像を表示します。走行後も映像を表示し、Ctrl+Cで終了するとウィンドウを閉じます。

PiのデスクトップまたはVNCから通常のコマンドで起動してください。SSHなどDISPLAY/WAYLAND_DISPLAYがない環境では表示を自動無効化し、録画処理を継続します。明示的に表示を止める場合は`--no-preview`を追加します。PC上では表示呼出し・無効化・GUI例外の模擬確認と構文確認のみ実施。Piでの画面表示・FPSは未確認です。
