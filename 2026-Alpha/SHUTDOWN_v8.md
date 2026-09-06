# Ctrl+C / ACK待ち対策 v8

現在の走行調整復元版に、終了処理だけを修正。相撲・RE/AT/TO・距離・PID・共通周期は変更しない。

## 実機で確認した停止
メインスレッドがraspike_wait_ack(cmd=102)から期限なしの条件変数待機に入り、受信スレッドは全スレッド一覧に見当たらなかった。Python側の正確なフレームは未解決だが、従来のmain finallyによる受信終了後の停止命令再送と整合する。

## 変更
- 制御コールバックの失敗・例外が通信ライブラリに戻る前に停止する。
- SIGINT（Ctrl+C）も受信処理が終了する前に停止。SystemExitでバックエンドのKeyboardInterrupt捕捉と無期限受信joinを回避する。
- main finallyはモーター停止命令を再送せず、カメラ終了処理だけを行う。
- 停止要求は1回に制限、ACK待ちは最大2秒。タイムアウトなら physical stop not confirmed と表示し、停止成功と扱わない。
- カメラ終了待ちは最大5秒。VideoThreadが2秒以内に終了しなければ、並行してcap.release/destroyAllWindowsを実行せず終了へ進む。
- LinuxのCtrl+Zも終了処理につなぐ（ジョブの一時停止にはしない）。
- 起動・終了ログのshutdown-v8で実行版を確認できる。

## 実機への反映
alpha.pyとrobot_program/services/shutdown.pyを両方反映する。現在停止中の旧プロセスには新しいコードは適用されない。新しいプロセスで、まず走行開始前のCtrl+Cを確認する。
期待する順序はSHUTDOWN motor stop begin/result → SHUTDOWN camera cleanup begin (no motor resend) → exiting... shutdown-v8。

## 検証範囲
ハードウェアAPIを模擬した回帰テストに加えて、インストール済み実Connectorクラスの制御フローでSIGINT→停止→terminated→受信join回避を確認。相撲関連の回帰テストを含む。実機PyPy/USB/OpenCVでの再確認は未実施。ネイティブAPIがPython全体を止める障害や、インタープリタ自体の終了ハングを強制終了で隠す対応はしていない。
