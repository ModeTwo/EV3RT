# AT・TOの編集箇所（元コードの書式を保持） v1

- AT: `robot_program/features/catch_bottle.py`。元の `bottle_catch.py::build_behaviour_tree` の変数名、コメント、引数の縦並び、処理順を使用。
- TO: `robot_program/features/to_hint_route.py`。非B版 `tantou3.py::build_tantou_tree` の `turn_left_55`、`go_to_black`、`trace_to_qr1` 等の定義と全体順序を1ファイルに保持。
- `move_to_hint1.py`、`read_hint.py`、`move_to_hint2.py`、`move_after_hint2.py` はalphaから必要な範囲を呼ぶ窓口。走行の調整は上記2ファイルで行う。

## 調整方法

各ブロックのtarget、power、PID、検出条件を元と同じ位置で編集する。距離とTO旋回出力は既存 `robot_program/integration_settings.py` の値を参照し、参照箇所に元の数値を記載。元の変数名が実値と異なる箇所（backward 10cmは200mm、turn_left_55は90度等）は実引数を優先する。

## 統合に必要な差分

`【統合差分】` コメントで表示。タッチ待ちと青線までの走行はalpha/REが実行済み。機器は共有し、TO先頭でジャイロを再初期化しない。TOのSpinAround/RunByGyroは `behaviours/section_motion.py` のLocalSpin/LocalDriveを元の名前で呼び、AT終了方位を基準にする。QR読取は `behaviours/hint_reader.py`、色認識は `behaviours/detect_bottle_color.py` で取得値を共有Contextへ保存する。ATの距離判定は元のIsDistanceReachedという名前の薄い接続クラスを使用。

TOのsection選択は工程管理に必要な接続のみ。距離後の停止、読取前停止、ヒント2後の出口走行の既存責務を維持。各タスクのタイムアウトは追加しない。共通周期、左右コース対応、終了処理、最新のRE・相撲・PC戦略受信処理は変更しない。

元の単体ファイルbottle_catch.py、tantou3.py、sample2.pyは保存版として変更しない。alpha.py実行時に編集が反映されるのは上記featuresのファイル。

## 確認

AT・TOの13件の統合テスト（取得値、距離指定、左右コース、共通周期、工程接続、停止等）を実行。RE独自の新しい走行方式や相撲に依存しないよう、AT・TOのテスト条件はlegacy RE/hint2を明示。実機の走行確認は未実施。
