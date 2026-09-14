# 走行関連の補足資料・変更履歴

2026-09-14更新: devRE最新`dd1f880`の方位角制御を正としてdevTKを調整。距離‐方位角プロファイル、POINTS、距離終了、全体方位基準は維持し、`SpinAround`をdevREの連続PID方式へ戻した。スタート～LAPのPIDをP=1.1/I=0.1/D=0.03へ戻し、devTK独自の曲率フィードフォワードと推定横ずれ補正は標準設定で無効化。固定角度`RunByGyro`の左右出力もdevRE方式へ整合。構文、設定値、±180度境界の方向補正、距離関数互換をPC上で確認。実機未検証。

2026-09-13更新: ETラリー受信SEQの遅延展開を修正。空の`py_trees.Sequence`へ`initialise()`中に子ノードを追加すると、最初の旋回開始直後に`Sequence reached an unknown / invalid state`となるため、`Sequence.tick()`が内部状態を選ぶ前に一度だけ展開する。Piへの配備対象は`robot_program/features/execute_strategy.py`。PC環境は`py_trees`未導入のため既存戦略テストは依存関係不足で開始できず、構文・展開順・差分を確認。実機再試走が必要。

本番起動は [RACE_STARTUP_v2.md](RACE_STARTUP_v2.md) を参照。v1は旧版として保持する。

2026-09-13更新: Strategyサーバーを全走行ツリーの外側へ移し、4桁キー確定・デバイス起動後、タッチ待ちより前にPC接続を確認できるようにした。接続成功を確認するまでタッチしない。

資料を元の所属フォルダ別にまとめています。旧版も含むため、各文書の版と日付を確認してください。文中の実行コマンドは従来どおり2026-Alphaを基準に実行します。

- [AT_TO_SOURCE_STYLE_v1.md](AT_TO_SOURCE_STYLE_v1.md)
- [AT_TO_SOURCE_STYLE_v2.md](AT_TO_SOURCE_STYLE_v2.md)
- [AT_TO_STANDALONE_v1.md](AT_TO_STANDALONE_v1.md)
- [AT_TO_STANDALONE_v2.md](AT_TO_STANDALONE_v2.md)
- [INTEGRATION_FOUNDATION_v1.md](INTEGRATION_FOUNDATION_v1.md)
- [INTEGRATION_RUN_v2.md](INTEGRATION_RUN_v2.md)
- [MISSION_SELECTION_v6.md](MISSION_SELECTION_v6.md)
- [MOTION_TRANSITION_v5.md](MOTION_TRANSITION_v5.md)
- [RESTORED_BEFORE_RIGHT_TRIAL.md](RESTORED_BEFORE_RIGHT_TRIAL.md)
- [RIGHT_COURSE_TRIAL_v3.md](RIGHT_COURSE_TRIAL_v3.md)
- [robot_program/ET_RALLY_DRIVE_STANDALONE_v1.md](robot_program/ET_RALLY_DRIVE_STANDALONE_v1.md)
- [robot_program/RUN_LOGGING_v1.md](robot_program/RUN_LOGGING_v1.md)
- [robot_program/START_LAP_ABSOLUTE_HEADING_v5.md](robot_program/START_LAP_ABSOLUTE_HEADING_v5.md)
- [robot_program/START_LAP_CALIBRATION_v4.md](robot_program/START_LAP_CALIBRATION_v4.md)
- [robot_program/START_LAP_GUIDE_v2.md](robot_program/START_LAP_GUIDE_v2.md)
- [robot_program/START_LAP_TUNING_v3.md](robot_program/START_LAP_TUNING_v3.md)
- [robot_program/START_LAP_TURN_DISTANCE_v6.md](robot_program/START_LAP_TURN_DISTANCE_v6.md)
- [robot_program/SUMO_BEARING_v1.md](robot_program/SUMO_BEARING_v1.md)
- [shared_communication/INTEGRATION_GUIDE_v1.md](shared_communication/INTEGRATION_GUIDE_v1.md)
- [shared_communication/INTEGRATION_GUIDE_v2.md](shared_communication/INTEGRATION_GUIDE_v2.md)
- [SHUTDOWN_v7.md](SHUTDOWN_v7.md)
- [SHUTDOWN_v8.md](SHUTDOWN_v8.md)
- [SUMO_RESTORE_v9.md](SUMO_RESTORE_v9.md)
- [TASK_TIMEOUTS_REMOVED.md](TASK_TIMEOUTS_REMOVED.md)

## Markdown保存ルール（2026-09-12）

2026-Alpha内のすべての.mdはこのdocs/notes配下に保存します。README、起動手順、変更記録も例外にせず、ルートやコードのフォルダへ作成しません。新しい記録は既存資料への追記を優先します。

- [RACE_STARTUP_v1.md](RACE_STARTUP_v1.md)
- [RACE_STARTUP_v2.md](RACE_STARTUP_v2.md)
- [README.md](PROJECT_README.md)
- [robot_program/README.md](robot_program/README.md)
- [robot_program/START_LAP_CURVE_TIMING_v7.md](robot_program/START_LAP_CURVE_TIMING_v7.md)
- [shared_communication/README.md](shared_communication/README.md)
- [wireless_device/README.md](wireless_device/README.md)
- [robot_program/START_LAP_CENTERLINE_HANDOFF_v12.md](robot_program/START_LAP_CENTERLINE_HANDOFF_v12.md) — 黒線中心のRunByGyro軌道と最終直線途中のライントレース切替


## 2026-09-13 devTKへ取り込まれた担当別原本の退避

2026-Alpha直下の担当別原本をarchive/branches/devAT・devRE・devTOへ移動しました。
分類は追加コミットの作成者とAT/RE/TOのマージ履歴に基づきます。Gitは作成時のブランチ名を記録しないため、フォルダ名は担当別の由来分類です。
下表の追加コミット・最終変更コミット・SHA-256で追跡できます。保存内容は現在のdevTK版で、取り込み後の修正も含みます。
alpha.py、sample.py、共通ライブラリ、現行robot_program、devTKの撮影・監視ツールは移動していません。
退避コードは参照保存用です。tantou群とsample2.pyは同じ場所に保持していますが、旧単体コードを再実行する場合は共通ライブラリのimportパスや作業ディレクトリを別途設定してください。
旧資料・ソース説明に記載された元ファイル名は、下表で読み替えてください。現行の走行処理からの直接importがないことと、移動前後の全ファイルのハッシュ一致を確認しました。

| 元パス（2026-Alpha基準） | 退避先（同基準） | 追加コミット | 最終変更コミット | SHA-256 |
|---|---|---|---|---|
| bottle_catch.py | archive/branches/devAT/bottle_catch.py | 8e1cc701cec9089692680af4f6a250a030872868 | 8e1cc701cec9089692680af4f6a250a030872868 | fb65678a9141d88f4c4c88e976ed1b2836383c3bcd527c2b8b4771633295ac5f |
| bottle_delivery.py | archive/branches/devRE/bottle_delivery.py | 4b28c43d19c138897586080c95acab84678b1a45 | 4b28c43d19c138897586080c95acab84678b1a45 | adc851b2fd75b5d3ba778c1c13af340e1744b14ae62f188e0669d0cf0c377b45 |
| bottle_delivery_0825.py | archive/branches/devRE/bottle_delivery_0825.py | 4b28c43d19c138897586080c95acab84678b1a45 | 4b28c43d19c138897586080c95acab84678b1a45 | 99bbb4f9e72cea7e693eae9ace5a62c1f16c84231703b288c9de6e0a4e4fd229 |
| bottle_delivery_taniguchi.py | archive/branches/devRE/bottle_delivery_taniguchi.py | 4b28c43d19c138897586080c95acab84678b1a45 | 4b28c43d19c138897586080c95acab84678b1a45 | ce0bd7ebe7139a9686dcb0b097a2659f3c93bbea9f5f50266015f88a155b7578 |
| capture_0.jpg | archive/branches/devRE/capture_0.jpg | 4b28c43d19c138897586080c95acab84678b1a45 | 4b28c43d19c138897586080c95acab84678b1a45 | 544e846744420c0875a2544e37944c3eba19eb2322ee7c72a631947fa64f81c3 |
| capture_0.png | archive/branches/devRE/capture_0.png | 4b28c43d19c138897586080c95acab84678b1a45 | 4b28c43d19c138897586080c95acab84678b1a45 | cae96d4ef7a0401f417b06ef4d882e959e38e68d3e6eea16a1373ddfd4de7774 |
| capture_1.jpg | archive/branches/devRE/capture_1.jpg | 4b28c43d19c138897586080c95acab84678b1a45 | 4b28c43d19c138897586080c95acab84678b1a45 | b4592a703cbf6d3845fe1090132cdb96f4e56fd05a8f0abdab55c29c7729e011 |
| gyro_line_0820_adv.py | archive/branches/devRE/gyro_line_0820_adv.py | 4b28c43d19c138897586080c95acab84678b1a45 | 4b28c43d19c138897586080c95acab84678b1a45 | c60c190ccbcc25d08644fa3608141005ebcd0f231b7f9c75ef05bea15b6161f9 |
| gyro_line_0826.py | archive/branches/devRE/gyro_line_0826.py | 4b28c43d19c138897586080c95acab84678b1a45 | 4b28c43d19c138897586080c95acab84678b1a45 | 3a26964ae052045e6c728c20bd36e5fc917b16146e4aa67a58ab60d44c8fb624 |
| gyro_line_0904.py | archive/branches/devRE/gyro_line_0904.py | 4b28c43d19c138897586080c95acab84678b1a45 | 18c8090a5a1ab5bf753527c05cfc67d275aaea7c | ff2e077210966d9f25ab9e26ec4b7b477860b1dd923b2b7f12727dd0ddf91daa |
| gyro_line_gyro_0825.py | archive/branches/devRE/gyro_line_gyro_0825.py | 4b28c43d19c138897586080c95acab84678b1a45 | 4b28c43d19c138897586080c95acab84678b1a45 | f99c92c46a7ca9f2599fe46dd1906a8dd53b9eccc1d5b0d326c8404abef87628 |
| gyro_line_gyro_0825_adv.py | archive/branches/devRE/gyro_line_gyro_0825_adv.py | 4b28c43d19c138897586080c95acab84678b1a45 | 4b28c43d19c138897586080c95acab84678b1a45 | 3d66d5b8316baca24bdb6d94b36183fc227a23743205e1f2bb09161c9244c49a |
| hint_read.py | archive/branches/devTO/hint_read.py | dc05593dc34c825b019dddf80dff6af2731de241 | dc05593dc34c825b019dddf80dff6af2731de241 | ae93316d9ae474a059de543acf17234b491f6bb5114ecb413c749f7d49bed55b |
| hint_read2.py | archive/branches/devTO/hint_read2.py | dc05593dc34c825b019dddf80dff6af2731de241 | dc05593dc34c825b019dddf80dff6af2731de241 | ae93316d9ae474a059de543acf17234b491f6bb5114ecb413c749f7d49bed55b |
| measure_hue.py | archive/branches/devRE/measure_hue.py | 4b28c43d19c138897586080c95acab84678b1a45 | 4b28c43d19c138897586080c95acab84678b1a45 | b2fe37641dda668190125e5f1622165f5733461a7078fb206876b0801df24755 |
| readKey.py | archive/branches/devAT/readKey.py | 59bb04ea3009550df21c5af5f560f65d55905023 | 8e1cc701cec9089692680af4f6a250a030872868 | 95df01dced0fe5d24710d93a07676886c451301c3392cd083d6697f4b0407f91 |
| sample コメント付.py | archive/branches/devAT/sample コメント付.py | 59bb04ea3009550df21c5af5f560f65d55905023 | 8e1cc701cec9089692680af4f6a250a030872868 | fa8d08a35af13d3ade36909f9461a7567e30591c419198d70b231f81dd842972 |
| sample 戸田.py | archive/branches/devAT/sample 戸田.py | 59bb04ea3009550df21c5af5f560f65d55905023 | 8e1cc701cec9089692680af4f6a250a030872868 | cdab6c4c03d2b9b6d80a71d234b07b3b1c86dbe27f3d2c54904c75fce7ddfbac |
| sample2.py | archive/branches/devTO/sample2.py | dc05593dc34c825b019dddf80dff6af2731de241 | dc05593dc34c825b019dddf80dff6af2731de241 | a3b7ccf782ba0cfbcfd349a28f726e25022a8d324630fd161987847ee101fa09 |
| sample_allgyro.py | archive/branches/devRE/sample_allgyro.py | d2a0bbc351ad1ebec49a8fb2e3ce5b7e15d35b20 | d2a0bbc351ad1ebec49a8fb2e3ce5b7e15d35b20 | 7511d8068f43cb5dd0aa337948cf064609e1d73854e43f87ae3d31429a0754eb |
| sample_gyro_line.py | archive/branches/devRE/sample_gyro_line.py | d2a0bbc351ad1ebec49a8fb2e3ce5b7e15d35b20 | d2a0bbc351ad1ebec49a8fb2e3ce5b7e15d35b20 | 9abd040c2b5cc153575faa7e15938134f3a2a73a11b683727f5fc9fa9b3cba19 |
| sample_gyro_line_0819.py | archive/branches/devRE/sample_gyro_line_0819.py | d2a0bbc351ad1ebec49a8fb2e3ce5b7e15d35b20 | d2a0bbc351ad1ebec49a8fb2e3ce5b7e15d35b20 | 18975e74d4f149d8ab5899dc4bef2f7cd896d6859f3e0d487c084a36b666f1e4 |
| sample_gyro_line_0819_bottle.py | archive/branches/devRE/sample_gyro_line_0819_bottle.py | d2a0bbc351ad1ebec49a8fb2e3ce5b7e15d35b20 | d2a0bbc351ad1ebec49a8fb2e3ce5b7e15d35b20 | a1aaf4eb91ad0f7628256c33c16ede7ea9287a26777bb80d6c5d2ded2f66d5aa |
| sample_gyro_line_0820.py | archive/branches/devRE/sample_gyro_line_0820.py | 4b28c43d19c138897586080c95acab84678b1a45 | 4b28c43d19c138897586080c95acab84678b1a45 | f2c76ef82e26aae17bc3a1324cde38595af544a959b7975333c443a1fd555b4b |
| sample_gyro_line_gyro.py | archive/branches/devRE/sample_gyro_line_gyro.py | d2a0bbc351ad1ebec49a8fb2e3ce5b7e15d35b20 | d2a0bbc351ad1ebec49a8fb2e3ce5b7e15d35b20 | 471f78f19089dadcd968dc05fbf7330994771550b69a8c38eb79e9e241cfd0f2 |
| sample_gyro_line_gyro_20cm.py | archive/branches/devRE/sample_gyro_line_gyro_20cm.py | d2a0bbc351ad1ebec49a8fb2e3ce5b7e15d35b20 | d2a0bbc351ad1ebec49a8fb2e3ce5b7e15d35b20 | 62a272e3be121d7738e91b0084b99beae2655e181b5f6f33bd90423e54c573d0 |
| sample_linetrace.py | archive/branches/devRE/sample_linetrace.py | d2a0bbc351ad1ebec49a8fb2e3ce5b7e15d35b20 | d2a0bbc351ad1ebec49a8fb2e3ce5b7e15d35b20 | 0d4797bb3d26271e28ce62b0cd2980563a2e89335d47cc3138df4fac416d800f |
| take_photo.py | archive/branches/devRE/take_photo.py | a2c5bce68a73f5428cbb0ad7474d79450605761e | 4b28c43d19c138897586080c95acab84678b1a45 | 592e56e375d7baac25911b4e57c1290c118dcfa0fe01d192f391554b42f076e7 |
| tantou2.py | archive/branches/devTO/tantou2.py | dc05593dc34c825b019dddf80dff6af2731de241 | dc05593dc34c825b019dddf80dff6af2731de241 | cbc8a982aa3ed7247d95290409b6ddff85fd6c91a3d9bf0cdae91bd5b7f0c61b |
| tantou3 copy.py | archive/branches/devTO/tantou3 copy.py | dc05593dc34c825b019dddf80dff6af2731de241 | dc05593dc34c825b019dddf80dff6af2731de241 | cc19c83ec8ddb6dd3d8c9b6da55f934f4b302d956f7b3e0ab296b3ecc8e3b37f |
| tantou3.py | archive/branches/devTO/tantou3.py | dc05593dc34c825b019dddf80dff6af2731de241 | dc05593dc34c825b019dddf80dff6af2731de241 | 5b667e34aa8dc0372053d171d912624101ded598389eeddba951ce711c0d9394 |
| tantou4.py | archive/branches/devTO/tantou4.py | dc05593dc34c825b019dddf80dff6af2731de241 | dc05593dc34c825b019dddf80dff6af2731de241 | 3eb389d20571421aaaf35dd1fda11fb25694d3514b8023cdfc8216b483536244 |


## 2026-09-13 devTKの診断ツール退避

全体走行から使用されない撮影・センサー表示・相撲カメラ監視の3入口をdevTK別の保管場所へ移動しました。内容は変更していません。sample.pyは共通配布元のサンプルでdevTK固有ではないため保持しています。

- capture_normal_image.py → archive/branches/devTK/capture_normal_image.py（追加コミット 73d3264b8af05d1123871a642e7d2f731215cf8d、SHA-256 575cf0c94a9360f59fe6ab5998fc86fc9c8b60433a87f5858127f5545a67713a）
- sensor_monitor.py → archive/branches/devTK/sensor_monitor.py（追加コミット 73d3264b8af05d1123871a642e7d2f731215cf8d、SHA-256 7407386e4d4916d08771356b59511806e8700bd50b3b85eccf0bbb995e19ab31）
- sumo_bottle_camera_monitor.py → archive/branches/devTK/sumo_bottle_camera_monitor.py（追加コミット d792f0c651af84e0c4d678c110bdefa7134884b4、SHA-256 7479c80c949162af9f751b92183fb337ed6df0991c65e73ec6d4021da4460a46）

補助ツールを再使用する際は2026-Alphaを作業ディレクトリとし、モジュール起動します。例: `python -m archive.branches.devTK.sensor_monitor --help`。通常の実機依存が必要です。ハードウェアでの再実行は未確認。ローカル除外対象のtest_sumo_black_bottle.pyには旧sumo_bottle_camera_monitorのimportが残ります。テストを使わない方針に従い変更せず、再使用時に参照先変更が必要です。


## 2026-09-13 旋回補正の刻み改善・回数上限撤去

共通SpinAroundとボトル終盤DeliveryPulseTurnの補正を変更。
残角に応じて40～120ms駆動し、許容角へ入る／目標を跨ぐ場合は次の制御周期で早期ブレーキ。
許容±2度の標準設定では残角3度で60ms、5度で100ms、6度以上で120msを上限とする。
既存の最低PWM・方位原点・目標角・距離走行は維持する。実機未検証の調整値。
補正12回と安定待ち2秒によるHOLDを撤去。回数で永久停止せず、未達なら停止後の再計測・補正を続ける。
現在値が許容角内なら追加駆動せず、安定判定の履歴が揃うまで待つ。
安定していなければブレーキで待ち、安定後は自動で判定を再開する。未達の成功扱いはしない。
Ctrl+C/通常中断の停止は保持。実機では各工程の収束と行き過ぎを確認する。
配備対象: robot_program/behaviours/gyro_drive.py、robot_program/behaviours/delivery_turn.py。


## 2026-09-13 旋回をsample.py方式へ復帰（最新）

共通SpinAroundとボトル終盤のパルス補正を撤去した。
sample.pyと同じ連続PID出力＋SymmetricClamperと、標準誤差2度未満で即完了する方式。
補正パルス、安定待ち、補正回数、HOLD、可変40～120msは使用しない。
明示toleranceを渡す相撲呼出しはその値を維持。統合ツリーの完了・中断時出力0と駆動時ブレーキ解除は維持。
ボトル終盤の絶対方位・単体/結合原点、共有直進RunByGyro、通信等の後続変更は巻き戻していない。
ボトル終盤の既存旋回後StopNow＋0.5秒待ちも維持。
配備対象: robot_program/behaviours/gyro_drive.pyとrobot_program/behaviours/delivery_turn.py。
PC反映済み、Pi転送・実機未確認。旧方式と同じ停止後の惰性誤差はあり得る。


## 2026-09-14 パスワード入力前のPC接続確認

PCプログラムを先行起動する。PiはTCP接続確認→4桁キー入力→デバイス初期化→タッチ待ちの順。接続待ちではHint送信・応答期限を開始しない。rally-driveも接続確認を行い、キー入力は省略。check-treeと通信不要モードは接続待ちなし。Ctrl+C/起動失敗時も通信を閉じる。TCP接続の確認であり、計算完了の保証ではない。
