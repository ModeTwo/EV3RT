# スタート～LAP カーブ別タイミング調整 v7

実走結果に合わせ、2番目と3番目のカーブを別々に補正する。

- 2番目: 到着点を維持し、旋回開始だけ60mm前倒し
- 3番目: カーブ区間全体を80mm後ろへ移動

## 初学者が変更する場所

`config.py` の次の値だけを変更する。

```python
start_lap_second_turn_start_advance_mm: float = 60.0
start_lap_third_turn_start_delay_mm: float = 80.0
```

どちらも `0.0` で位置補正なしになる。2番目の値を大きくすると早く曲がり始め、3番目の値を大きくすると遅く曲がり始める。

後続3カーブを短くする共通倍率は、引き続き次の値で調整する。

```python
start_lap_later_turn_distance_scale: float = 0.75
```

この処理は `build_start_to_lap_gate()` の方位角プロファイルだけに適用する。
