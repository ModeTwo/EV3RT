# 工程の追加時間制限を撤去（2026-09-08）

担当別の元ソース（bottle_catch.py、tantou3.py、sample2.py、gyro_line_0826.py）とd3eb494の担当統合前履歴を照合し、統合時に追加した時間切れ中断を撤去。

- ATの前進・後退はDriveDistanceを直接実行。distance_motionとTO旋回のTimeoutを撤去。
- ボトル色・ヒント1/2は取得まで待つ。15秒/20秒によるFAILUREなし。
- 相撲のカメラ探索・接近・フレーム更新待ち、連続ソナー走査の時間切れ中断を撤去。
- motion_timeout_sec、bottle_timeout_sec、qr_timeout_sec、camera_detection_timeout_sec、camera_approach_timeout_sec、continuous_scan_timeout_secを設定から削除。
- ユーザー指定によりPC戦略受信の工程制限は維持。終了処理のACK/カメラ待機上限やsocketの短時間待機も維持。
- 距離・角度・読取条件、制御周期、旋回後の静止待ち、フレームによる確認・見失い判定は維持。時間切れで次へ進めない仕組みだけを外す変更であり、全コードを昔の版に置き換えるものではない。

Ctrl+Cによるshutdown-v8は維持。以前の各文書のAT/TO/相撲タイムアウトの説明より本書を優先する。

検証：模擬1時間経過でも走行/認識を継続し、距離・角度・読取成立で完了する。実機検証は未実施。
