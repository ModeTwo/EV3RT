# 走行関連の補足資料・変更履歴

2026-09-15更新: ETラリーの絶対方位が360度境界をまたぐとき、`SpinAround`の終了判定だけでなく、`SpinAround`と固定角度`RunByGyro`のPID目標も現在方位に最も近い等価角へ変換するよう修正した。たとえば現在314度・受信目標-44度はPID目標316度となり、-358度方向の大回転を指示しない。開始ログへ`requested`、`resolved`、`delta`を追加。経路計算、相対旋回、距離プロファイル走行は変更していない。PC上で構文と境界値を確認済み、実機未検証。

2026-09-14更新: devREコミット`334b33d`「9月13日スタート～LAPの完成版プロファイルです」を正としてLAP走行を復元。`start_lap_profile_v1.py`はpull後すでに同一。RunByGyro一つ、相対方位、青検知開始・見逃し上限、LAP単体距離終了を同コミットへ合わせ、P=1.8/I=0/D=0.03、FF=1、横ずれlookahead=300、power=80を採用。5100mmライントレース切替と未使用の個別カーブ調整設定を撤去。構文、完成版との差分、実効距離を確認。実機未検証。

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
## 2026-09-14 青検知区間をライントレース化

- `build_start_to_lap_gate` は、校正後の青予測位置250mm手前（現在約4606mm）まで `RunByGyro` で走る。
- そこから速度45の `TraceLine` に切り替え、ラインを追従しながらLAP青マーカーを検知する。
- 明度85以上の白面が2制御周期続いた場合は旋回量35でラインへ復帰し、ラインを再取得すると通常のPID追従へ戻る。
- 青を検知できない場合は、従来と同じ校正後LAP位置+100mm（現在約5006mm）で失敗停止する。
- 調整値は `robot_program/config.py` の `start_lap_line_*` に集約した。
- 走行パワーは最低50とし、復帰方向は一時的に通常判定の逆向きへ設定した。Rコースでは右旋回、Lコースでは左旋回となる。その場旋回ではなく、基準50・補正35（概ね左右85/15）で前進しながら探す。
- 4回の実績ログでは全て青未検知のまま距離上限で停止した。白2周期（約0.04秒）の判定は通常のライン端追従まで逸脱扱いにするため、15周期（約0.3秒）へ変更した。復帰開始・黒線再取得をログへ出す。右2回目は5008mmで黒を拾った直後に旧上限へ達したため、青未検知上限をLAP+500mm（現在約5406mm）へ延長した。


## 2026-09-14 AT・TO・ボトル配置の結合実行

- `python alpha.py left --mission at-to` : AT単体と同じ青線手前の配置から、AT→TO出口移動まで連続実行し停止。
- `python alpha.py left --mission at-to-bottle` : 同じ開始位置からAT→TO→ボトル配置→ラリー開始位置復帰まで実行し停止。
- `python alpha.py left --mission to-bottle --bottle-color red --delivery-initial-heading ANGLE` : AT終了位置へボトル保持状態で配置し、TO→ボトル配置→ラリー開始位置復帰。ANGLEは実際の配置方位をボトル配置用座標（ライン方向0度、内向き90度、左右コースで鏡像）で指定。推測値を使わない。
- 右コースは`right`。各コマンドに`--check-tree`を追加すると機器を開かず工程順を確認できる。to-bottleのツリー表示用の色・方位仮値は実走の配置条件を保証しない。
- 初期化・タッチ待ちは一度だけ。AT終了時の距離・方位、ボトル色、TO取得Hintを同じRaceContextで引き継ぐ。工程間に追加のリセット・待ち・停止を挟まない。既存工程内の停止はそのまま。
- `INTEGRATION START/END`に工程名、状態、所要秒、距離、生ジャイロ角、引継ぎ情報を出力し、既存run_logsへ保存。ATの色・原点、TOの異なる2つのHint、配置後のbottle_delivered/rally_readyを確認し、不足時はFAILUREで後続へ進まない。
- 結合対象は既存のAT、TO、bottle-finalと同じビルダー。AT開始時のみ既存単体モードの青線探索を使用。本番fullや制御パラメータは変更しない。未実装を含む結合実走は開始しない。
- この3モードはラリー走行を含まず、PC接続・復号キーは不要。QR読み取りは行う。ボトル配置の範囲は既存bottle-finalに合わせ、配置後のラリー開始位置復帰までを含む。
- ログのSUCCESSはソフトウェア上の成立。接続位置・停止姿勢・実物の配置成功は実機で確認する。Pi転送・実機確認は未実施。


## 2026-09-14 後半工程の結合実行（v2）

- ボトル→ラリー: `python alpha.py left --mission bottle-rally --bottle-color red --rally-hint1 "25,35" --rally-hint2-gate-info "53,54/12,22"`
- ラリー→相撲: `python alpha.py left --mission rally-sumo --rally-hint1 "25,35" --rally-hint2-gate-info "53,54/12,22"`
- 相撲→ガレージの構造確認: `python alpha.py left --mission sumo-garage --check-tree`。参考プログラム待ちのため、実走は既存drive_to_garage_pendingを検出して機器起動前に拒否する。成功扱いで読み飛ばさない。
- Hintの数値は形式例。実際の復号済み座標に置換。bottle-rally/rally-sumoはPC先行起動・TCP接続確認が必要、QR/4桁キー入力は不要。カメラは既存走行/相撲処理用に起動する。
- bottle-rallyはbottle-finalと同じTO出口の配置位置、ライン方向0度、ボトル保持状態から開始。配置→ラリー開始位置復帰→PC受信SEQ走行で停止。ラリーの目標角は本番開始原点から180度ずれた初期方位へ変換する。
- rally-sumoはrally-driveと同じラリー入口・内向きから開始。受信SEQ走行→相撲進入→捕捉/離脱→ガレージ側黒線への復帰で停止。開始時の相撲用方位はLeft=270/Right=90度で登録し、ラリー終了時にリセットしない。
- sumo-garageはsumo単体と同じ進入開始位置・上向き0度が既定。--sumo-initial-bearingで配置方位を指定可能。参考コード受領後、FINISH前段と既存青マーカー→650mm直進の責務重複・方位を照合して接続する。それまでは実走不可。
- 新しい手入力Hint結合2モードではタッチ後に送信・応答期限を開始し、初期化/配置待ち中には開始しない。受信は走行と並行し、5秒期限は送信投入時から。接続確認と応答期限は別。
- 相撲はgarage_line_found/line_trace_ready/transport_completedが揃った場合のみ結合成功。捕捉なしの読み飛ばしは接続成功とせずFAILURE。既存full/単体の制御・スキップ方針は変更しない。
- PC上の構成・方位・状態検査のみ。Pi転送・実機確認未実施。初期方位誤りや物理位置ずれは実機検証が必要。
## 2026-09-14 LAP手前の復帰速度調整

- 復帰旋回は基準パワー50・旋回補正25（概ね左右75/25）へ低速化した。
- 黒線を再取得した後の通常ライントレースは基準パワー60へ上げた。
- `TraceLine` に任意の `recover_power` を追加し、LAP手前だけ50を指定した。


## 2026-09-14 Hint2出口を白検出・方位走行へ変更

Hint2読取→従来のTO基準90度復帰/待機→90度保持で低速直進→白連続検出→追加前進→TO基準190度への低速前進旋回→黒線再取得→80mm低速追従→停止→既存ボトル配置、の順に変更。hint2単体の読取終了動作は維持。hint2-return、TO出口を含むfull/結合モードへ適用する。出口前の90度復帰旋回は既存動作。

設定はrobot_program/integration_settings.pyのto_exit_*。初期値はPWM25、補正上限15、出力変化40 PWM/秒。左右とも前進し、アーム出力は変更しない。PWMは速度単位ではなく、保持荷重・床面による旋回半径/脱落防止は実機未確認。ゲートをセンサーで回避する機能はない。ボトル/アームを含む通過範囲と競技上の通過条件を確認すること。

白判定はHSV明度V>=85の3周期（20ms周期で約60ms）、直進30mm以降、V<=75の線側を一度観測してから有効。白の確定までの走行距離も旋回位置に含まれる。追加前進は未実測につき0mm。真の白色識別ではないので実路面で閾値を調整する。旋回目標は90度復帰と同じTO基準の絶対190度（180度+10度）。左右コース鏡像とAT→TO原点を保持。ユーザー指定により追加前進0mm。旋回途中でも黒を3周期検出した時点で低速ライントレースへ切り替える。

白探索600mm、旋回500mm、黒探索200mm、各段階20秒でFAILURE停止。上限は成功扱いにしない。旋回中または190度到達後の直進探索中にV<=65を3周期確認後、通常側エッジをPWM25で80mm追従。追従中に白15周期でFAILURE。正常終了/中断/異常は出力0とブレーキ。通常終了後の青探索・色別配置は既存動作。

まずhint2-returnで、白検出位置、追加前進距離、TO基準190度への向き、低速で動く最低PWM、黒線再取得、保持状態とゲート余裕を実機確認する。センサー位置と車軸位置の差だけでは追加前進量は決まらず、前進旋回の軌跡も含めて調整する。Pi未転送。実機合格前の調整候補。


## 2026-09-14 相撲後のゴール走行・8/20参考コード反映（最新）

参考待ちの記述を更新。FINISHはNo.19で青まで通常側ライントレース（PWM50、PID 0.55/0.0000009/0.015）、No.20で青検知地点から800mmジャイロ走行（PWM60、PID 1.1/0.1/0.03）して停止。既存650mmは今回の添付800mmへ更新。青を2回探索しない。初期化・アーム上下・タッチ待ちを再挿入しない。

添付の絶対0度は、既存相撲出口と同じコース図下向き180度（sumo.garage_bearing_deg）へ変換。登録済み方位をRunAtBearingで使用し、sumo-garageとfullの原点差を吸収する。分岐の直線選択は添付に実装がなく、追加していない。

調整値はRaceConfigのgarage_*。800mm/明度75は実機未校正。青探索30秒・直進20秒でFAILURE停止する。sumo-garageは既存相撲出口状態の確認後にFINISHへ接続し、PendingFeatureによる拒否を解除。実行例: python alpha.py right --mission sumo-garage（通常の相撲単体と同じ配置）、構造確認は--check-treeを追加。fullも同じFINISHを使う。finish単体はガレージ側黒線上・下向きの配置が前提。Pi転送・実機走行は未実施。


## 2026-09-14 Hint2出口 v2（実走不具合修正・最新）

実走でPWM25が不足し、黒捕捉後の白連続判定に続いてTypeErrorが発生した。独自stop(reason)をfail(reason)へ改名し、py_treesのstop(new_status)を復元。終了は親ライフサイクルからterminateを呼び、成功・失敗・中断ともブレーキ停止する。

出口は基準PWM50、補正上限25（定常指令25～75）、変化120 PWM/秒へ変更。0から50までは約0.42秒。ボトル保持を考慮して出力変化制限と前進旋回は維持するが、荷重・ゲートとの余裕は実機再確認が必要。

黒を3周期検出したら、既存TraceLineのPIDとフィルタ・NORMAL側追従を使用する。配置側と同じP=.65/I=.000001/D=.045。出力は出口専用アダプタで±25と変化制限を適用し、旋回時の左右出力から連続させる。共通TraceLineの変更はモーター出力をメソッドへ分離しただけで、通常呼出しの出力とLAP復帰設定は保持。旧to_exit_line_kpは互換保存のみで出口FOLLOWには使用しない。

追加前進0mm、TO基準190度、旋回途中黒検出で切替、80mm追従、白15周期失敗、距離/時間上限は維持。終了直前の白は成功扱いにしない。0.2秒ごとに工程・区間距離・明度・方位・左右出力・白黒カウントを記録する（出力は直前周期の指令）。Pi未転送。実機で自力走行・黒捕捉後追従・ボトル保持・ゲート余裕を確認する。


## 2026-09-14 ゴール青検知までのPIDをボトル設置に統一

drive_to_garageのTraceLineをdrop_bottleと同じP=0.65/I=0.000001/D=0.045へ変更。旧値は0.55/0.0000009/0.015。基準PWM50、明度目標、通常側、青検知とTimeout、工程順は保持。青後の180度ジャイロ直進はP=1.1/I=0.1/D=0.03、PWM60のまま。まずライン追従の変更のみ実機評価する。P増強は角度追従遅れには有効な可能性があるが、横ずれやジャイロ基準ずれを修正するものではなく、増やしすぎると蛇行する。PC反映、Pi未転送・実機未確認。


## 2026-09-14 Hint2出口 v3（その場旋回・最新）

カード2への接触を受け、TURN中の基準前進出力を0へ変更。白検出後（追加前進0mm）は左右出力0/ブレーキ→0.15秒待機→左右同量・逆方向へ出力する。前進時の指令履歴を0へ戻してから120 PWM/秒で上げるため、旋回中の左右指令の和は0。旋回PWMは残角比例・最低40/最大50（立上り中を除く）。小残角で動けなくなることを避ける初期値で、実機調整が必要。

TO基準190度、旋回途中の黒3周期で即PID追従への切替、黒未検出なら目標方位から最大200mm前進探索、追従PWM50・80mm、距離/時間上限、ログは保持。停止は固定待機であり静止のセンサー確認ではない。黒検出後と旋回完了後の探索では前進を再開する。車体中心の移動は抑えるが、滑り・惰性とアーム/ボトルの回転範囲による接触は実機確認が必要。Pi未転送。


## 2026-09-14 Hint2旋回出力 v4（最新）

ユーザー指定でto_exit_pivot_min_powerを40→55、maxを50→60へ変更し既存TO旋回と合わせた。立上りの出力変化制限120 PWM/秒は維持。その場旋回・190度・白/黒判定・追従切替条件・探索上限は今回変更しない。コード差分は設定2値のみ、AST/設定生成/元TO値一致を確認。Pi未転送・実機未確認。

10:51起動ログは青検知による旋回ではない。stop_at_blueは工程名で、直前の青検知ノードはコメントアウトされ385mm距離完了で停止する。Hint2出口はHSV明度Vだけで白/黒を判定している。ライントレース逸脱は白継続・補正飽和・ジャイロ角変化・探索区間等の組合せで検出可能だが、その白が目的の曲がり場所かは単一明度だけでは識別できない。現行のHint2出口WHITEはライントレースではなく90度保持直進。早期旋回/早期黒切替の対策は未実装。

## 2026-09-14 devAK引き継ぎ書

- [DEVAK_HANDOFF_v1.md](DEVAK_HANDOFF_v1.md) — Codex / Claude Code向け。ボトルデリバリー終盤No.7～9、色別青線選択、配置・復帰、方位基準、単体・結合起動、未解決事項と受入確認。説明対象はdevTK作業ツリーでありdevAK反映済みではない。
## 2026-09-15 LAP手前のカメラライン復帰 v5

- 規定距離（現在約4606mm）後の固定方向復帰旋回を廃止し、`2026base/camera_trace_demo.py` の `TraceLineCam` を基準にしたカメラライントレースへ変更した。
- BASEと同じカメラ左右エッジ選択、theta PID（P=2.0/I=0/D=0.06）、傾きフィードフォワード（gain=8/cap=8）、非検出4フレーム目以降の出力55%制限を使用する。基準パワーは過去指示どおり50。
- カメラでラインへ寄せながらカラーセンサーのV<=65を3周期連続確認し、確認後はカラーセンサー式TraceLine（power=60）へ引き渡して青を検知する。
- カメラがラインを見ていない場合も固定左右旋回へ切り替えず、BASE同様に最後のtheta方向を制限付きで保持する。
- 追加実装は `robot_program/behaviours/camera_line_trace.py`。調整値は `robot_program/config.py` の `start_lap_camera_*`。


## 2026-09-15 Hint1→Hint2の直線距離補正 v1

カード1後の90度旋回からカード2手前まで（line_trace_120）の終了判定を、Plotterの浮動小数点XY座標差をTO基準90度へ投影した距離へ変更。to_hint2_trace_mm=1000を維持。左右コースとAT→TO原点を変換し、蛇行の横移動を除外、後退は進捗から減算する。QR1直後の385mm、PWM60/PID、QR2向け115度旋回、読取/出口、共通の積算距離は変更なし。

PROJECTED_DISTANCEログは0.5秒ごとと完了時にprojected_mm（補正距離）、travelled_mm（従来距離）、target_mm、raw_heading_degを記録。実機では同じ開始位置から左右コースで停止位置と両距離を比較する。滑り・ジャイロの基準誤差は補正できない。PC実装候補。Pi未転送・実機未確認。
## 2026-09-15 青検知前の姿勢安定化 v6

- カメラ開始を青予測位置の250mm手前から500mm手前（現在約4356mm）へ早めた。
- カメラ終了は、カラーセンサーV<=65、カメラ認識中、abs(theta)<=5度、abs(論理方位角)<=5度が10周期（約0.2秒）連続した場合だけ成立する。一瞬の黒線横断では終了しない。
- 一度黒線を取得した後は、2026-BASEのカメラtheta制御へジャイロ0度補正P=0.3（上限8）を加える。
- 青検知はラッチし、その後に論理方位角が0±5度で3周期安定してから次工程へ渡す。
- 主な調整値は `RaceConfig.start_lap_camera_*`、`start_lap_heading_tolerance_deg`、`start_lap_blue_heading_stable_samples`。
## 2026-09-15 カメラ復帰実走分析とv7

- 右2走とも開始時はV=11/13、方位=-2/-4度で既にライン上かつ姿勢良好だったが、カメラinsight=0の古いtheta（+21.1/-12.8度）を使用し、最大42/43の操舵で先に右へ外れた。その後insight=1のtheta=-11～-29度に最大50を出し、左へ急切込みした。
- カメラ非認識時は古いthetaを即時不使用としてcamera turn=0にした。ラインが近すぎてカメラから見えなくても、カラーセンサーV<=65とジャイロ0±5度が10周期安定すれば通常TraceLineへ渡す。
- カメラ認識時の操舵上限を50から20へ制限した。固定復帰旋回およびblind時の方向保持は使用しない。

## 2026-09-15 LAP手前S字復帰 v8

- カメラ復帰をSEEKとALIGNへ分離した。SEEKはPWM50・最終操舵上限30でラインへ入り、V<=65を3周期検出するとALIGNへ移る。
- ALIGNはカメラ操舵を停止し、PWM35・ジャイロP=0.8・最終操舵上限25で絶対方位0度へ切り返す。黒線やcamera thetaの継続は要求せず、0±5度を3周期確認して通常TraceLineへ渡す。
- 青検知はカメラ復帰開始と同時に並行起動する。復帰中に青を踏んだ場合もラッチし、0±5度を3周期確認後に次工程へ渡す。
- PID・傾きFF・ジャイロを合算した後のturnをフェーズ別上限で再制限する。ログのphase=SEEK/ALIGN、line acquired、heading stableで状態遷移を確認する。

## 2026-09-15 LAPカラーセンサー追従 v9

- ALIGN後から青検知までのカラーセンサー式TraceLineを、ボトルデリバリー終盤と同じ target=75、power=50、P=0.65、I=0.000001、D=0.045、TraceSide.NORMALへ統一した。
- 既に一致していた目標明度、PID、追従側は維持し、差があったpowerだけ60から50へ変更した。カメラSEEKのPWM50・上限30、ALIGNのPWM35・ジャイロ補正上限25は変更していない。

## 2026-09-16 LAPカラー引渡し安定化 v10

- 実走で通常TraceLineが白V=92または黒中央V=11から始まり、青より前に右へ脱線したため、ALIGNと通常TraceLineの間にHANDOFFを追加した。
- HANDOFFはPWM35、目標V=75、P=0.3、最終turn上限10でライン端へ穏やかに寄せ、絶対方位0度へのジャイロ補正も同じ上限内で合成する。V=65～85かつ方位0±5度が5周期連続してから通常TraceLineへ渡す。
- 通常TraceLineはユーザーが戻したPWM60、target=75、P=0.65、I=0.000001、D=0.045、NORMALを維持。ログはphase=HANDOFFとcolor補正値を出す。


## 2026-09-15 TO目標角へのAT終了方位加算を廃止 v1

ユーザー指定により、HandoffState.absolute_headingはAT終了時の実測角を加算せず、指定目標角をそのまま返す。AT_TOのheading_degは診断記録・引渡し完了確認用として保持する。起動時からの左右コース共通の論理方位を使用し、実測18度でも指令0/90/115/190度はそのまま0/90/115/190度になる。

LocalSpin/LocalDrive、Hint2出口の直進・旋回、投影距離の軸が同じ方位基準になる。相対旋回、ジャイロリセット、工程順、距離、PIDは変更しない。過去の「AT終了方位をTOの局所0度にする」説明は本変更で失効。投影距離の既存試験も実測原点を加算しない走行へ更新。PC反映候補、Piへの転送・実機確認は別途必要。直進追従不足は残課題。ラリーの360度境界問題は後続の2026-09-15更新で修正し、実機確認待ち。

## 2026-09-15 devAK引き継ぎ書 v2（現devTK基準・最新）

- [DEVAK_HANDOFF_v2.md](DEVAK_HANDOFF_v2.md) — 現行devTK作業ツリーのボトル終盤No.7～9、TO共通方位、投影距離、ジャイロ360度境界修正、単体/結合手順と未解決事項。v1は旧版として保持。
- [DEVAK_STATE_v2.json](DEVAK_STATE_v2.json) — 確認日時・HEAD・未コミット状態・134 PythonのSHA256。コード一式やPi配備済みの証明ではない。
- 今回実コードではLAPのstart_lap_line_power=60、配送delivery_trace_power=50。過去のLAP50追記との相違をv2へ記録し、設定変更は行っていない。


## 2026-09-15 QR1撮像準備の先行・QR1後青検知 v1

TO開始時にQR用撮像設定を非同期で準備し、最初の旋回・565mm直進中にカメラ再オープンと5枚破棄を済ませる。qr_prepareセッションでは復号しない。停止待機後のReadHintCardが新規qrセッションを開始するため、走行中の画像・結果を採用しない。パワー・距離・固定待機は維持。QR1後385mm直進は青検知または距離到達で終了。PC実装、Pi未転送・実機未確認。QR_PREPARE→VID cap openedがTO走行中に出て、read_qr1開始後のelapsedが短くなることを確認する。


## 2026-09-15 QR1後青検知のimport漏れ修正

前回追加したIsColorDetectedのconditionsからのimportが不足していたため修正。青または385mmの終了条件は維持。構文だけでは未定義名を検出できないため、検証コードへ呼出し名のimportと定義元の対応確認を追加。PCのpy_trees未導入により実環境インポート・BT全体起動は未検証。Piにはto_hint_route.pyを再配備する必要あり。


## 2026-09-15 QR2カメラ準備の先行 v1

カード1後の青/385mm直進と90度旋回を終え、1000mm投影距離のカラーセンサー追従開始直前にPrepareHintCameraを追加。QR2も走行中に撮像準備を行い、停止待機後のReadHintCardが新規セッションの画像で確定する。QR1と同じ既存クラスを使用し、速度・距離・旋回・待機・Hint2出口条件は維持。PC反映済み、Pi未転送・実機未確認。配備対象はrobot_program/features/to_hint_route.py（QR1先行準備版のvideo.py/hint_reader.pyが既に配備済みであること）。QR_PREPAREとVID cap openedがread_qr2より前に出ること、QR_SUCCESS hint=2のelapsed短縮を確認する。


## 2026-09-15 QR1→QR2の撮像設定維持 v1（前のQR2先行方式を更新）

read_qr1にkeep_qr_ready=Trueを指定。成功時terminateでLINEへ戻さずqr_prepareへ移り、QR撮像設定を維持しながら旧結果を破棄する。移動中は復号せず、QR2停止後の新規セッションで読取。1000mm追従前の重複準備ノードは撤去。QR2成功と読取中断/失敗はLINEへ戻す。完了済み読取ノードの再無効化では後続カメラ状態を変更しない。配備対象はrobot_program/behaviours/hint_reader.pyとrobot_program/features/to_hint_route.py。既存QR先行版video.pyが前提。PC反映、Pi未転送・実機未確認。QR1成功からQR2開始の間にVID cap openedのMJPG/YUYV往復がないことを実機確認する。


## 2026-09-16 AT移動中ボトル認識

bottle_catch2の意図をfeatures/catch_bottleへ移植。AT単体はタッチ（alpha）→PWM45/OPPOSITE追従→青検知、統合はREの青検知後から開始。PWM60/OPPOSITE・目標75・PID .65/.000001/.045で400mm追従しながら色認識。100mm前進・200mm後退・停止認識を撤去。同色3新規フレームで赤/青/黄を確定し、確定後も距離まで走る。400mmで停止・ブレーキし、色未取得ならFAILUREとしてTOへ渡さない。成功時だけ既存AT_TOを記録。新しい認識セッション・古いフレーム拒否・LINEへの復帰を保持。旧距離2設定は互換用に残すが未使用。QR改善は保持。実機未確認のためOPPOSITE追従、青から400mmの停止位置、色取得、TOへの接続位置を確認する。


## 2026-09-16 カメラプレビュー非表示モード

起動コマンドへ `--no-preview` を追加する。例: `python alpha.py left --mission full --no-preview`（右はright、普段の他オプションも併用可）。省略時は従来どおり表示する。撮影・QR読取・ボトル認識・カメラ追従は継続し、表示用縮小・文字領域合成・imshow/waitKey・ウィンドウ終了処理だけを省略する。認識途中の図形描画は保持しており、認識高速化の効果は未測定。Videoを直接使うコードは `Video(preview_enabled=False)` でも指定できる。カメラ不要ミッションの撮影停止条件は従来どおり。

PC反映。配備対象はalpha.pyとpy_etrobo_util/video.py。模擬カメラで表示ON/OFFのLINE/BOTTLE/QR観測一致、画像取得失敗時と終了時のGUI呼出し抑止を検証。Pi転送・実カメラ・実走行は未確認。


## 2026-09-16 AT元コードの形を保持する修正

ユーザー指示によりcatch_bottleの短縮・共通化を撤回。bottle_catch2のbuild_behaviour_treeからコメント、空行、変数名、引数の並べ方、Parallel/add_childrenの組立順を保持して移植。450表記の不整合のみ有効距離400へ統一。タッチのalpha委譲、統合時の青工程省略、context共有、設定距離、未取得確認とAT_TO記録を【統合差分】で明示。IsDistanceReachedとDetectBottleColorWhileMovingの元の呼出し名を薄い接続クラスで保持。移動中認識・400mm停止など前版の動作は維持。


## 2026-09-16 AT色未取得時は青で継続

ユーザー指定により400mm停止後に色が未取得ならBottleColor.BLUE.valueを設定し、ログへdefaulting to BLUEを出してAT_TOへ正常に引き渡す。取得済みの赤・青・黄は保持。前項の未取得FAILURE方針を本変更で更新。元作者のツリー構造、距離、追従、移動中認識は維持。実機未確認。


## 2026-09-16 精度確認のため青デフォルトを撤回

ユーザー指示により直前の青補完を撤回。400mm停止時に色未取得ならFAILUREとし、AT_TOを記録せずTOへ進まない。取得済み色は保持。移動中認識と元作者のツリー構造は維持。青デフォルトは現在無効。


## 2026-09-16 受領tantou4を選択統合

作者の工程順・変数名・ツリー構造を維持し、最初の直進550mm/PWM55、QR1後340mm、QR2手前1200mm、旋回I=.005、直進P=.001/I=.01を反映。QR2手前は既存の投影距離判定を維持するため添付の積算1200mmと停止位置は同一とは限らない。角度は添付対応sample2未確認のため共通方位90/0/90/115/90を維持し、ResetDeviceは導入しない。QR先行準備・撮像維持・新規セッション・Context保存、QR未読時の待機、Hint2Exitの白→旋回→黒再取得→追従は保持。QR未読リカバリの新規実装は今回含まない。現場で距離・PID・停止位置を検証する。


## 2026-09-16 青・グレー区間293mmを絶対0度走行

LAP青検知で方位安定を待たずATへ引渡し、青検知起点の最初293mmをRunByGyroの絶対0度、PWM60、P=1.8/I=0/D=.03で走る（試走初期値）。その後従来のOPPOSITEカラー追従へ戻し、合計400mmで停止する。移動中のボトル認識、未取得時FAILURE、AT_TO記録は維持。AT単体も青検知から同じ処理。距離は積算走行距離で、青の物理先端と検知位置の差は実機で調整する。配備対象はfeatures/catch_bottle.py、features/start_to_lap_gate.py、integration_settings.py。Pi未配備・実機未確認。


## 2026-09-16 TOの90度旋回オーバーシュート・直進ドリフトを修正

ユーザー報告（左90°旋回が明らかに90°より大きい、ヒント1読取後の直進が明らかに右にずれる）を添付ログで解析し、原因2件を修正した。

1. `behaviours/gyro_drive.py`の`SpinAround.terminate()`/`RunByGyro.terminate()`が`set_power(0)`のみでブレーキ未設定だった。旋回完了(SUCCESS)直後、次の`StopNow`がブレーキをかけるまでコーストし、運動量でオーバーシュートしていた。両`terminate()`へ`set_brake(True)`を追加し、完了直後に即時ブレーキするよう修正。
2. `features/to_hint_route.py`の`run_to_black`/`run_after_qr1`（受領tantou4統合時に採用したpid_p=.001/pid_i=.01）は、直進中の数度の姿勢誤差に対する補正がint切り捨てでほぼゼロになり、方位ホールドが実質無効化されていた。ログ上もQR1後の直進でheadingが0度目標から-4→-13度へ拡大し続けている。同ファイル内の旧実装（コメントアウト済み）で使われていたpid_p=1.1/pid_i=.00075/pid_d=.04へ戻した。

検証はPC上でpy_trees/simple_pid/py_etrobo_utilを最小スタブに置換し、`gyro_drive.py`の実クラスを直接importして実施（work/to-hint-brake-pid-fix-v1/verify_gyro_drive_fix.py、ETロボコン側）。terminate()直後の即時ブレーキ、および簡易プラントモデルでの新旧ゲイン比較（旧P=.001は3秒後も誤差-4.0度のまま、新P=1.1は-0.9度まで収束）を確認。実機・実カメラでの再現テストは未実施、PID値は現場再調整が前提。

全変更ファイル（修正）: `robot_program/behaviours/gyro_drive.py`、`robot_program/features/to_hint_route.py`。ETロボコン側HANDOFF.mdにも同内容を記録。Pi転送・実機確認・PIDチューニングが残課題。既存の未コミット変更（alpha.py等）は保持。


## 2026-09-16 Hint2Exit(TURN)の早期離脱・距離上限FAILUREを修正

ユーザー添付ログでHint2読取後のTURN→FOLLOW引継ぎ直後にヘディングが大きく振れて脱線する挙動を確認し、ユーザー指示で2点修正した。

1. `behaviours/hint2_exit.py`のTURNフェーズが黒未検出・目標方位未到達のまま距離上限(`to_exit_turn_limit_mm`=500mm)に達すると、従来は`FAILURE`（ミッション失敗）だった。黒線を見失ったまま停止するより復帰の可能性を残すため、`FOLLOW`（ライントレース）へフォールバックするよう変更。
2. TURN序盤（目標方位=190度local基準までの誤差が大きい間）の一時的な低明度3周期連続検出で即座にFOLLOWへ切り替わっていたため、実ログでは旋回開始87度→112度（誤差103度→78度）というごく早い段階でライントレースへ引き渡され、以後55mmの間にheadingが112→162度と大きく振れていた。新設定`to_exit_turn_black_detect_max_error_deg`（既定60.0度）を追加し、目標方位までの誤差がこの値以下になるまで黒検出カウントを進めないようにした。

検証はPC上でpy_trees/simple_pid/py_etrobo_utilを最小スタブに置換し、`Hint2Exit`の実クラスを直接importして実施（ETロボコン側work/hint2-exit-turn-fallback-v1/verify_hint2_exit_fix.py）。誤差103度(ゲート外)でのv低下3周期はTURN継続、誤差55度(ゲート内)なら従来どおりFOLLOWへ遷移、黒未検出のままTURN距離上限に達してもFAILUREにならずFOLLOWへ遷移することを確認。実機・実カメラでの再現テストは未実施、ゲート値は現場再調整が前提。

全変更ファイル（修正）: `robot_program/behaviours/hint2_exit.py`、`robot_program/integration_settings.py`。ETロボコン側HANDOFF.mdにも同内容を記録。Pi転送・実機確認・ゲート値チューニングが残課題。既存の未コミット変更は保持。


## 2026-09-16 実走ログ分析: 早期離脱ゲート後の「ライン見失い」FAILUREを確認

上記修正後の実走ログをユーザーから受領。旋回はheading86→131度(誤差59度、新ゲート60度のすぐ内側)で明度低下を検出しFOLLOWへ切替。その後v=11→11→39→99と明度が安定せずheadingが131→184度まで約53度振れ、`ExitTraceLine`の白連続検出が`to_exit_lost_cycles`(15周期)に達し`HINT2_EXIT failed: line lost during low-speed follow`でFAILURE、`alpha.py`のミッション失敗ガードにより`RuntimeError: Mission failed; motors stopped.`で全停止した。TURN→FOLLOW切替自体は意図どおりで、停止原因はFOLLOW側の別の安全装置（白連続検出）であることをユーザーへ回答した。コード変更なし（分析のみ）。


## 2026-09-16 TURNを190度まで実施し、黒未検出でもFOLLOWへ引き渡すよう変更

ユーザー指示「190°実施して、条件を満たさなくてもそのままライントレースしてください」により、`behaviours/hint2_exit.py`のTURN→BLACK遷移条件（誤差<=`to_exit_heading_tolerance_deg`で`phase='BLACK'`へ移行し前進しながら黒探索を継続）を変更した。目標方位(190度local基準)まで旋回できた時点で、黒検出条件(3周期連続)を満たしていなくても直ちに`FOLLOW`へ引き渡すようにした。これにより`BLACK`フェーズ（`to_exit_search_limit_mm`等を使う前進探索）はこの経路からは到達しなくなった（コード・設定は残置、明示的な削除は未実施）。

検証は`work/hint2-exit-turn-fallback-v1/verify_hint2_exit_fix.py`にTest4を追加し、誤差が許容値(5度)まで縮まった時点でv=99(白のまま、黒未検出)でも`FOLLOW`へ遷移し`black_count`が0のままであることを確認。既存Test1〜3も再確認済み。実機・実カメラでの再現テストは未実施。

全変更ファイル（修正）: `robot_program/behaviours/hint2_exit.py`。ETロボコン側HANDOFF.mdおよびwork/hint2-exit-turn-fallback-v1/verify_hint2_exit_fix.pyにも記録。Pi転送・実機での190度到達時FOLLOW引き渡しの確認が残課題。`BLACK`フェーズ関連コードの整理は指示範囲外のため保留。既存の未コミット変更は保持。


## 2026-09-16 実走ログ分析: TURN→FOLLOW後の「ライン見失い」FAILUREが再発

190度実施の修正後の実走ログ2件をユーザーから受領。いずれもTURNはheading87→121→132度付近(誤差58〜59度、ゲート60度のすぐ内側)で黒っぽい明度を検出し正常にFOLLOWへ引き渡していたが、その後明度が白へ回復し、`ExitTraceLine`の白連続検出が`to_exit_lost_cycles`(15周期)に達して`line lost during low-speed follow`でFAILURE、ミッション失敗で全停止した。当時残っていた失敗経路（初期化異常／センサー非数／フェーズ共通タイムアウト20秒／WHITE距離上限600mm／FOLLOWのライン見失い15周期）を列挙してユーザーへ回答した。コード変更なし（分析のみ）。


## 2026-09-16 Hint2Exit全体を「TURN開始後は停止しない」設計へ変更

ユーザー指示「ターンを始めたら止まることなくライントレース、および次の工程に確実に引き渡すようにしてください」により、`behaviours/hint2_exit.py`の`update()`を再構成した。フェーズ共通タイムアウト(20秒)と各フェーズの距離上限チェックを一本化し、打ち切りが発生した場合に`FAILURE`ではなく必ず次の状態へ前進するようにした：WHITE/OFFSETの打ち切り→TURNへ、TURN/BLACKの打ち切り→FOLLOWへ（既存の黒検出ゲート・190度到達条件と共存）、FOLLOWの打ち切り→SUCCESS（その場の姿勢のまま完了）。FOLLOW内の「白連続検出によるライン見失い」判定も、従来のFAILUREからSUCCESS（見失いを許容してそのまま完了）へ変更した。

初期化時の異常（方位未確定・course不正）とセンサー値が非数(NaN/inf)の場合は、危険な状態でモーターを動かし続けないよう従来どおりFAILUREのまま残した。ユーザー指示の対象は探索・追従の打ち切り条件であり、ハードウェア/設定異常とは別種の問題と判断したため。

検証はETロボコン側work/hint2-exit-turn-fallback-v1/verify_hint2_exit_fix.pyにTest5（WHITE距離上限打ち切り→TURNへ前進）とTest6（FOLLOWでライン見失い15周期→SUCCESSで完了）を追加し、既存Test1〜4と合わせて全6件成功。実機・実カメラでの再現テストは未実施。**注意**: 見失った位置のまま次工程（ボトル配置ゾーン判定・搬送）に進むため、大きくコースアウトした状態で完了扱いになる可能性があり、実機での挙動確認を強く推奨する。

全変更ファイル（修正）: `robot_program/behaviours/hint2_exit.py`（タイムアウト/距離上限/ライン見失いをすべて前進・完了扱いへ統一）。ETロボコン側HANDOFF.mdおよびverify_hint2_exit_fix.py（Test5・Test6追加）にも記録。既存の未コミット変更は保持。
