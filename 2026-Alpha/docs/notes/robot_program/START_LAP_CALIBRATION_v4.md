# 開始距離の補正 v4

実測500mmは下ろしたアーム先端からカーブ始点まで。車軸中心からアーム先端までは進行方向100mmなので、車軸がカーブ始点に達するまで600mm走る。

config.py の編集項目:

- `start_lap_first_straight_mm = 600.0`: 車軸中心から最初のカーブまでの距離。
- `start_lap_route_scale = 1.0`: 最初のカーブ以降だけの距離倍率。実測がないため今回は維持する。

旧表の500mm以降は100mm後ろへ移す。後続の直線長・曲率・角度は変えない。青検知開始とLAP終了位置も同じ100mmだけ移す。LAP単体終了は5631.717mm、青見逃し上限は5711.717mmになる。

featureは引き続き `RunByGyro(target=profile.heading_at, ...)` 一つ。start_lap_calibration.pyは走行前に表を補正するだけで、毎周期の制御はRunByGyroが担当する。PDF由来のstart_lap_profile_v1.pyは保存版として編集しない。

旧表は最初の500mmから全体縮尺も算出していた。今回その縮尺が正しいと確認できたわけではない。カーブ後の直線が長いという観察については、後続直線の接線点間の実測や走行距離の校正が必要。今回の変更だけで直線長の問題まで解決したとは扱わない。

Piへconfig.py、features/start_to_lap_gate.py、start_lap_calibration.pyを転送し、`python alpha.py right --mission lap` で単体確認する。PIDと速度はv3のまま。実機で最初の曲がり始めを確認してから追加調整する。旧挙動への復帰はfirst_straight_mm=500、route_scale=1。
