# スタート～LAPを修正する人へ（構造整理 v2）

## 最初に読むファイル

まず `features/start_to_lap_gate.py` を読む。このファイルは「どの目標角で、いつまで走るか」を決め、走行命令 `RunByGyro` を一つ返す。

```python
profile = HeadingProfile(POINTS)
return RunByGyro(
    name='start_to_lap_gate',
    target=profile.heading_at,   # 関数そのものを渡す。ここでは呼ばない。
    power=config.start_lap_power,
    pid_p=config.start_lap_pid_p,
    pid_i=config.start_lap_pid_i,
    pid_d=config.start_lap_pid_d,
    target_type=HeadingType.RELATIVE,
    distance_limit_mm=distance_limit_mm,
    completion_condition=finish_condition,
    completion_min_mm=finish_check_from_mm,
)
```

`RunByGyro` が毎周期 `profile.heading_at(開始からの走行距離mm)` を呼び、戻り値の角度をPIDへ渡す。関数は、入力された距離に対して有限な角度を返せばよい。コース表や補間計算をモーター制御へ書き込む必要はない。

## 何を変えたいかで編集場所を選ぶ

|変えたい内容|ファイルと項目|
|---|---|
|速度|`config.py`: `start_lap_power`|
|PIDの効き方|`config.py`: `start_lap_pid_p` / `start_lap_pid_i` / `start_lap_pid_d`|
|青の検知を始める距離|`features/start_to_lap_gate.py`: `BLUE_SEARCH_BEFORE_MM`|
|LAP単体で停止する位置|同ファイル: `LAP_PASS_MARGIN_MM`|
|青を見逃したときの走行上限|同ファイル: `BLUE_MISS_MARGIN_MM`|
|距離に対応する角度|`start_lap_profile_v1.py`: `POINTS`。改訂時はv2等の別名へ保存しfeatureのimportを変更|
|表の間の角度計算|`heading_profile.py`: `HeadingProfile.heading_at()`|
|目標角から左右モーター出力を計算|`behaviours/gyro_drive.py`: `RunByGyro._update_distance_target()`|

例: Iの影響だけを比較したいなら `start_lap_pid_i` だけを変更し、P/D/速度/角度表を同時に変えない。**今回の構造整理ではIを0にしていない。** 値は従来のP=1.1、I=0.1、D=0.03、出力33を維持している。

## 距離と角度の約束

- 距離はこの命令の開始時を0mmとした積算距離。世界座標のXではない。
- 関数が返す角度は開始時を0度とする連続角。距離関数では `HeadingType.RELATIVE` を指定する。
- 表はL基準で保持し、Rコースへの変換はRunByGyroで行う。R用に表の符号をさらに反転させない。
- `POINTS` は `(距離mm, 角度deg)` の組で、最初は `(0, 0)`。距離は必ず増加させ、同じ距離を二度書かない。
- 例: `(500, 0)` と `(700, -40)` の間では600mmで-20度になる。
- 角度は一周しても連続にする。例えば359→0のつもりなら表では359→360と書く。
- 表の終点より先では最後の角度を維持する。停止距離は `distance_limit_mm` で別に指定する。
- この表は最初の直線500mmを基準にPDF全体を比例換算した設計値。コース実測の保証ではない。

## 終了の約束

LAP単体は `completion_condition=None` なので、ゲート位置+20mmでSUCCESSになり出力0。全体/Hint工程では青検知を指定し、予測位置250mm手前から検知する。青検知でSUCCESS、青未検知のままゲート+100mmまで達するとFAILUREで出力0になる。後続ATの100mm前進/200mm後退は変更していない。

モーター出力0は走行体の幾何学的な停止位置を保証するものではない。ゲートとカラーセンサー、車軸の位置関係は実機で確認する。

## 他工程と旧方式

他工程の `RunByGyro(target=90, ...)` は従来どおり固定角度として扱う。ABSOLUTE/RELATIVEの意味も従来どおり。関数の場合の終了オプションを固定角度へ混ぜると設定エラーにする。

`config.py` の `start_lap_mode='legacy'` で旧5区間＋ライントレースへ戻せる。旧工程は `features/start_to_lap_gate_legacy.py` へ分けた。`behaviours/distance_heading_drive.py` は旧クラス名を使うコード向けの互換入口だけで、制御処理は持たない。新しいfeatureでは使わない。

PIDを毎周期作り直すと内部状態が失われるので、目標角だけ更新する。通常の調整でPID生成処理や左右出力式を変更する必要はない。

## 確認方法

実機用依存ライブラリが利用できる2026-Alphaで、まずハードウェアを開かないテストを実行する。

```sh
python -B -m unittest discover -s robot_program/tests -p test_gyro_target.py
```

その後 `python alpha.py right --mission lap` 等で実機確認する。今回の同値比較では20ms刻みの疑似入力5934サンプルについて、旧版と新版の目標角・状態・左右PWM・PID内訳が一致した。実測で見られた追従遅れ、行き過ぎ、横ずれの改善は今回の対象外である。

Piへ配備する変更コード: `behaviours/gyro_drive.py`、`behaviours/distance_heading_drive.py`、`features/start_to_lap_gate.py`、`features/start_to_lap_gate_legacy.py`、`config.py`。既存の `heading_profile.py` と `start_lap_profile_v1.py` も引き続き必要。
