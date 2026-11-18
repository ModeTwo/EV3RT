# 11/19(本番の日)の流れ

ロボットは C。コマンドはロボットでは `~/EV3RT/2026-Alpha`、PC では `2026-Alpha` で打つ。

## 0. 会場に着いたら

ロボットで:

```
git pull
git log --oneline -1
```

2c7ca02e 以降なら次のものが入っている。

- ボトルを捕まえた位置から40mmで止める(LAP)
- 左コースの新しい LAP 経路
- full で止まって撮る較正

## 1. 較正台で撮る(ロボット)

1枚ごとに較正台に置き直し、3回くり返す。

```
python3 et_rally_camera_capture.py --tag jig --count 1
```

## 2. 較正し直す(PC)

`--keep-floor` は付けない。

```
python wireless_device/et_rally_camera_calib/recalibrate_day.py --base wireless_device/et_rally_camera_calib/calib_C_day_jig.json --out calib_C_day_jig2.json --set-default
git add wireless_device/et_rally_camera_calib/calib_C_day_jig2.json robot_program/et_rally_start_pose.py
git commit -m "当日の較正台の較正(C、11/19)"
git push
```

ロボットで `git pull`。

## 3. 試走

### 3-1. 右: 止まって撮る較正(1〜2回)

ETラリーのスタートから前へ15cm×5で止まって撮る。75cm 進んで止まるので、手で回収する。

```
python3 alpha.py right --mission full --lap-bottle-aim --rally-plan-path robot_program/tests/plan_test_selfcal_stops_1114.json --rally-camera-log 1 --rally-straight-pause-sec 1.5 --rally-straight-pause-labels cal1,cal2,cal3,cal4,cal5
```

PC で(画像のフォルダの日時は走ったときのもの):

```
scp -r "shunsuke09@10.56.69.191:EV3RT/2026-Alpha/run_logs/et_camera_<日時>*" day_logs/
python wireless_device/et_rally_camera_calib/day_floor_calib.py day_logs/et_camera_<1回目> [day_logs/et_camera_<2回目>]
```

カメラの補正が小さく(1°・数mm 程度)、スタートの読み違いが回ごとに揃っていれば、既定にする。

```
python wireless_device/et_rally_camera_calib/day_floor_calib.py day_logs/et_camera_<1回目> [day_logs/et_camera_<2回目>] --set-default
git add wireless_device/et_rally_camera_calib/calib_*_floor.json robot_program/et_rally_start_pose.py
git commit -m "当日の床での較正(C、full でスタートまで走って撮った)"
git push
```

ロボットで `git pull`。

### 3-2. 左右でいつもの full(時間があるだけ。LAP の止め方は左右3回ずつが目標)

```
python3 alpha.py left --mission full --lap-bottle-aim --rally-strategy local --rally-camera-log 5
python3 alpha.py right --mission full --lap-bottle-aim --rally-strategy local --rally-camera-log 5
```

`--rally-camera-log 5` はスタートのずれが残ったときの原因調べ用。本番では付けない。

### 毎回ログで見ること

- ヒント2が解けたか(QR の時間、QR_STATS、QR_RECOVER)
- LAP: `LAP_CAPTURE`(捕まえた位置)と止まった理由(`source=bottle_capture` / `LAP_CAPTURE_STOP` / 黒から120mm)。ヒント1のオブジェクトとアームの先の間は目で見る。
- 左: 2.9〜3.8m で緑の禁止エリアに入らないか(目視)
- 運搬のあとの赤→白
- 相撲の捕まえた判定
- ETラリー: カメラ補正(used/seen・resid)、スタートのずれ(下)、周回中のずれ

スタートのずれを見る(PC):

```
python wireless_device/et_rally_camera_calib/lap_start_fix.py day_logs/run_*.log
```

左右とも、残りが数mm・0.3°以内なら較正は合っている。

## 4. 本番の前に決めること

1. ボトルを捕まえた位置で止める: 有効のまま、または `--lap-capture-stop-mm 0` で止める。
2. 左の LAP 経路: 新しいまま使う、または元に戻す(ロボットで、戻したら commit しない場合は本番の前に必ず確認)。
   ```
   cp robot_program/start_lap_profile_left_right_copy_1118.py robot_program/start_lap_profile_left.py
   ```
3. 較正: 3-1 の較正で、左右ともスタートのずれが小さいか。だめなら周回のログから `lap_start_fix.py ... --out calib_C_day_lap.json --set-default`。
4. 2回目に攻めるか: すき間2cm の計画(計算中、11/19 朝に結果)を使うか。

## 5. 本番

1回目(いつもの):

```
python3 alpha.py <left|right> --mission full --lap-bottle-aim --rally-strategy local
```

2回目は、1回目が成功して攻めると決めたときだけ、すき間2cm の計画で走る(コマンドは計算が終わってから追記する)。
