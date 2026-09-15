# スタート～LAP ライン収束設計 v11

## 判定の訂正

`09122024.jpg`の赤線は上端と左側で緑へ出ている。白領域から少しでも緑へ出た走行は失敗と判定する。v10で「上側への直進は解消」とした判断は誤り。

## 原因

距離とIMU方位角から得られるのは進行方向であり、黒線に対する左右位置ではない。タイヤ径差、滑り、床面、開始位置の差で横ずれが生じても、方位角だけでは黒線の位置へ戻せない。従来の推定横ずれもエンコーダとIMUからの推定であり、コースを直接観測していない。

## 修正方針

1. スタートから400mmは方位角0度で直進する。
2. 最初のカーブ開始600mmより200mm手前の黒線上で、カラーセンサーによる`TraceLine`へ切り替える。
3. 以後は黒線端を追従する。
4. 後続工程がある場合は青LAPを検出して引き渡す。LAP単体では補正済みLAP距離で終了する。

400mmは直線上で切替を安定させるための初期値。切替時にカラーセンサーが黒線端へ載っていない場合は、`start_lap_line_trace_from_mm`を50mm単位で前後させる。

## 調整値

`robot_program/config.py`へ次を集約した。

```python
start_lap_line_trace_from_mm: float = 400.0
start_lap_line_target_v: int = 65
start_lap_line_power: int = 33
start_lap_line_pid_p: float = 0.55
start_lap_line_pid_i: float = 0.0000009
start_lap_line_pid_d: float = 0.015
```

この変更により`build_start_to_lap_gate()`は、`RunByGyro`一つではなく、`RunByGyro → TraceLine`の順序構造になる。黒線上を通ることを優先するための意図的な構造変更である。
