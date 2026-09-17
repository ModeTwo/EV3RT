# スタート～LAPの角度追従・横ずれ調整 v3

## 今回の変更

角度を測れていても、従来のPIDだけでは曲がり続けるための出力を作るのに誤差が必要になる。実績ログでは曲線で10～19度程度遅れ、その後は積分項の残留が疑われる行き過ぎがあった。

1. Pを1.1→1.8、Iを0.1→0、Dは0.03のまま。積分残留をなくし、角度誤差の補正を強める。
2. コースの曲率から基本旋回出力を計算し、PID出力へ加える（フィードフォワード）。角度差が出る前から曲線に必要な出力を用意する。後方30mmの平均曲率を使い、PDFの細かなつなぎ目による急増を抑える。最初の500mmより前には曲げない。
3. ジャイロと距離で実経路を推定し、同じ距離の計画経路との横ずれを計算する。横ずれを戻す向きへ最大±8度だけ目標角を補正する。

featureは引き続き `RunByGyro(target=profile.heading_at, ...)` 一つ。補助計算は `path_tracking.py`、毎周期の呼出しは `behaviours/gyro_drive.py`、数値調整は `config.py` に分離している。

## 調整する設定

|config.pyの項目|初期値|意味|
|---|---:|---|
|start_lap_power|33|前進出力（変更なし）|
|start_lap_pid_p|1.8|角度差に応じた補正|
|start_lap_pid_i|0.0|積分は今回は使わない|
|start_lap_pid_d|0.03|旋回の変化を抑える項（変更なし）|
|start_lap_feedforward_gain|1.0|基本旋回出力の倍率。0で無効|
|start_lap_wheel_tread_mm|110.0|左右車輪間隔。既存Plotterの値を仮採用。実機寸法を確認|
|start_lap_cross_track_lookahead_mm|300.0|横ずれを戻す緩やかさ。大きいほど弱く、0で無効|
|start_lap_max_heading_correction_deg|8.0|横ずれ補正角の上限|
|start_lap_log_interval_sec|0.2|制御診断ログの間隔（秒）|

基本旋回出力は `power × 車輪間隔/2 × 曲率(rad/mm) × gain`。PWMと車輪速度が比例するという近似であり、実機で完全には一致しない。合計旋回出力と左右PWMを範囲内に制限し、PIDの出力範囲も基本旋回分を考慮する。

## ログの読み方

- `nominal`: 表の計画角f(s)。
- `correction`: 横ずれを戻すための追加角度。最大±8度。
- `target`: nominal+correction。PIDが実際に追従する角度。
- `actual`: 開始からのジャイロ連続方位。
- `error`: target-actual。角度追従を見るときはこの差を確認する。
- `xte_est`: 推定横ずれmm。正なら進行方向右側。Rコースも内部は同じ基準に正規化する。
- `ff`: 基本旋回出力。`p/i/d`: PIDの各成分。`turn`: 合計旋回出力。
- `left/right`: モーターへ出したPWM。

横ずれを戻すときは、意図的にnominalとは少し違う方向へ向く。nominalとactualが一時的に違うことだけで追従失敗とは判断せず、targetとactualを比較する。

## 実機で確認する順序

Rコースの同じ設置位置から、まずLAP単体を走らせる。`python alpha.py right --mission lap`。調整後の実走ログはまだないため、以下は確認項目であり完了宣言ではない。

1. 0～500mmでnominal=0、ff=0となること。
2. 最初のカーブのerrorが旧ログの約15度から減るか、カーブ後の約7度の行き過ぎが減るか。
3. 最終直線で実際のラインとの横ずれが減るか。xte_estの符号・値と目視を照合する。
4. 青の検知と次工程の引渡しを確認する。LAP単体は距離で停止するため、青検知の検証には全体/Hint工程の設定が必要。

切り分ける場合は一度に一つだけ変更する。横補正の効果を見るなら `start_lap_cross_track_lookahead_mm=0` で比較する。基本旋回の効果を見るなら `start_lap_feedforward_gain=0`。元のv2制御へ戻すにはP=1.1/I=0.1/D=0.03、feedforward_gain=0、cross_track_lookahead_mm=0にする。出力33と元の距離表は同じなので、コードを巻き戻す必要はない。

## 検証済みの範囲と限界

実py_trees/simple_pidの8契約テストが成功。差動走行の簡易モデルで旋回効率3値×応答遅れ2値を比較し、角度平均誤差・終点横ずれが6条件とも減少、L/Rの鏡像一致を確認した。モデルは実機校正済みではなく、実走で同じ数値になるとは言えない。

横ずれはエンコーダ/IMUからの推定であり、線の観測ではない。タイヤの横滑り、ジャイロドリフト、初期設置の横ずれ、PDFの縮尺誤差は区別できない。その場合はラインセンサー等による位置補正を別途検討する。単一実測500mmでコース全体を比例換算した表は変更していない。

SpinAroundの停止精度改善と、他工程の固定角度RunByGyroの更新処理は保持。終了距離、青の検知窓、ATの前後移動も保持している。

Piへは変更コード4ファイルを一緒に転送する:

- `robot_program/behaviours/gyro_drive.py`
- `robot_program/path_tracking.py`（新規・必須）
- `robot_program/config.py`
- `robot_program/features/start_to_lap_gate.py`

テスト: `python -B -m unittest discover -s robot_program/tests -p test_gyro_target.py`。
