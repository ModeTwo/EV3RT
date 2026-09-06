# 右コース調整前への復元

ユーザー指定により、pre_right_trial_20260906_v3.zipの変更前コードへ復元。

- RE直進の固定PWM補正を撤去、元のジャイロ出力計算へ復元。
- LAPカーブは元の速度60 / P0.55 / D0.08 / 平滑化なし。
- ATボトル認識後の距離は左右とも460mm。
- Hint2読取タイムアウトは元の20秒。
- 後続のライン取得確認・TO減速/停止角確認も撤去。
- Ctrl+C関連v4/v7修正も戻した。終了不能問題への対策は現在のコードに含まれない。
- 相撲本体、相撲設定、共用部品の独立した相撲向け変更、ミッション選択（--mission sumo等）、現行のPendingFeature仕様は維持。
- RE→AT→TO統合と共通20ms周期は調整前からあるため維持。

過去のRIGHT_COURSE_TRIAL_v3.md、MOTION_TRANSITION_v5.md、SHUTDOWN_v7.mdは履歴資料であり現在の動作仕様ではない。追加モジュールとテストはbackups/rollback_before_right_retired/へ保存し、通常の実行・テスト対象から外した。復元直前ファイルはbackups/pre_restore_before_right.zip。

検証：復元コピーの全38テスト成功（相撲関連テストを含む）。実機走行未検証。
