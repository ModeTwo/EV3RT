# スタート～LAP 方位角制御 v5

`build_start_to_lap_gate()`が生成するプロファイル走行だけを、開始時からの相対角ではなくIMU方位角で制御する。

```python
return RunByGyro(
    target=profile.heading_at,
    target_type=HeadingType.ABSOLUTE,
    ...
)
```

`ResetDevice`が競技開始前にジャイロをリセットするため、スタート地点は方位角0度となる。`POINTS`の角度はこの0度を基準とする連続方位角で、`RunByGyro`は毎周期、次を直接比較する。

- 目標: `profile.heading_at(開始からの走行距離mm)`
- 実績: `-runtime.course * runtime.gyro_sensor.get_angle()`を連続化したIMU方位角

R/Lの符号変換は従来どおり`runtime.course`で行う。開始時のIMU角度を目標や実績へ加算しない。

変更対象はスタート～LAPの`build_start_to_lap_gate()`だけ。他工程が固定角度または距離関数へ`HeadingType.RELATIVE`を指定した場合は、従来どおりその命令開始時を0度とする。legacy版の5区間走行も変更しない。

`RunByGyro`の距離関数モードはABSOLUTE/RELATIVEの両方を解釈できる。ABSOLUTEでは連続IMU方位角、RELATIVEでは開始時方位を差し引いた角度をPIDの実績値にする。PID、速度、フィードフォワード、横ずれ補正、終了距離は変更していない。

実機確認ではログの`actual`が開始時からの相対変化ではなくIMU方位角であることを確認する。開始直前に機体を動かした場合は、ResetDevice完了後の0度基準が崩れるため、スタート姿勢を直してミッションを再起動する。
