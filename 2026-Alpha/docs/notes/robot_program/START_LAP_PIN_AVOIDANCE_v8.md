# スタート～LAP ピン衝突回避調整 v8

実走軌跡 `09121929.jpg` では、下側直線から左側を上がった後の3番目のカーブが内側へ切れ込み、中央寄りのコース外ピンへ衝突した。

3番目だけ次の補正を行う。

- 距離倍率を `0.75` から `1.0` へ戻し、旋回半径を広げる
- 旋回開始を従来の80mm遅延から250mm遅延へ増やす
- 2番目の「開始60mm前倒し、終了位置維持」は変更しない

## 調整する場所

`robot_program/config.py` の次の2値を変更する。

```python
start_lap_third_turn_distance_scale: float = 1.0
start_lap_third_turn_start_delay_mm: float = 250.0
```

まだ内側へ入る場合は `start_lap_third_turn_start_delay_mm` を大きくする。外側へ膨らみ過ぎる場合は小さくする。曲がり方が急な場合は `start_lap_third_turn_distance_scale` を大きくし、緩過ぎる場合は小さくする。

この補正は `build_start_to_lap_gate()` の方位角プロファイルだけに適用する。
