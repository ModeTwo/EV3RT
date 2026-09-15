# Ctrl+C / 異常終了 修正 v7

起動ログにshutdown-v7が表示されることを確認する。alpha.pyだけでなくrobot_program/services/shutdown.pyも走行体へ反映する。

- Hint1タイムアウト等のBT失敗、カメラ処理例外、制御コールバック例外は、USB受信処理が生きているコールバック内で停止命令を1回だけ送ってから例外を返す。
- Ctrl+C / SIGTERMも通信終了前に停止処理を行い、SystemExitでバックエンドのKeyboardInterrupt後の無期限joinを回避する。
- mainのfinallyは停止命令を再送しない。USB受信終了後のACK待ちを防ぐ。カメラ終了を行う。
- モーター停止APIはdaemonスレッドで最大2秒待機。完了できなければphysical stop not confirmedと表示する（実際に止まったことを保証しない）。カメラ終了待機は従来の最大5秒。
- LinuxのCtrl+Z (SIGTSTP) は、一時停止によるプレビュー残留を避けるため終了処理へ変換する。このプログラムではCtrl+Zでの一時停止・再開は使用しない。
- SHUTDOWN motor stop begin/result → SHUTDOWN camera cleanup begin → exiting... shutdown-v7のログを追加。
- 既に旧プログラムがCtrl+Zで停止している場合、そのプロセスに今回の変更は適用されない。新しいプログラムへ置き換えて起動する必要がある。

検証: 模擬通信の受信終了前に停止し、その後再送しないこと、実SIGINTの処理順序、モーターAPI停止応答なし時の待機上限をテスト。実機のPyPy/USB/OpenCVは未検証。ネイティブAPIがPython全体を停止させる障害や、OSからシグナルが届かない状態まで保証するものではない。

走行経路、ミッション選択、PendingFeatureの現行仕様は変更しない。
