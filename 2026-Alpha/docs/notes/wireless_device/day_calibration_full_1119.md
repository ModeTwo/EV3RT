# 当日のカメラ較正(full で ETラリーのスタートまで走ってから、止まって撮る)

2026-11-19。ロボット C。作業者は LAP のスタートにしか立てないので、full で相撲・青い丸を通って ETラリーのスタートに着いたあと、
前へ15cmずつ5回止まって撮る(床での較正の試験と同じ動き)。ETラリーのスタートの行にはゲートの脚が来ない前提。

コマンドは PC では 2026-Alpha で、ロボットでは `~/EV3RT/2026-Alpha` で打つ。

## 1. 較正台で撮る(ロボット)

1枚ごとに較正台に置き直し、これを3回くり返す。

```
python3 et_rally_camera_capture.py --tag jig --count 1
```

## 2. 較正し直す(PC)

**`--keep-floor` は付けない**(11/18 は 11/16 のスタートの補正を引き継いで、向きが約1°・位置が1〜4cm ずれた)。

```
python wireless_device/et_rally_camera_calib/recalibrate_day.py --base wireless_device/et_rally_camera_calib/calib_C_day_jig.json --out calib_C_day_jig2.json --set-default
git add wireless_device/et_rally_camera_calib/calib_C_day_jig2.json robot_program/et_rally_start_pose.py
git commit -m "当日の較正台の較正(C、11/19)"
git push
```

ロボットで `git pull`。

## 3. full で走って、ETラリーのスタートで止まって撮る(ロボット、右コース)

```
python3 alpha.py right --mission full --lap-bottle-aim --rally-plan-path robot_program/tests/plan_test_selfcal_stops_1114.json --rally-camera-log 1 --rally-straight-pause-sec 1.5 --rally-straight-pause-labels cal1,cal2,cal3,cal4,cal5
```

- ETラリーのスタートから前へ 75cm(15cm×5)進んで止まる。ガレージには行かないので、止まったら手で回収する。
- できれば2回。2回目は1回目と違う状態(相撲・青い丸のあとの向きなど)になるので、そのままでよい。
- 左コースは PC の道具(day_floor_calib.py)が左右反転に対応していないので、右コースで撮る。

## 4. 較正ファイルに書く(PC)

画像のフォルダを取ってくる(日時は走ったときのもの)。

```
scp -r "shunsuke09@10.56.69.191:EV3RT/2026-Alpha/run_logs/et_camera_<日時>*" day_logs/
python wireless_device/et_rally_camera_calib/day_floor_calib.py day_logs/et_camera_<1回目> day_logs/et_camera_<2回目>
```

- 「点の重なりの残り」が小さくなり、カメラの補正(向き・傾き・高さ)が小さい(1°・数mm 程度)ことを確かめる。
- スタートの読み違い(前・左・向き)が回ごとに揃っていることを確かめる。

よければ既定にする。

```
python wireless_device/et_rally_camera_calib/day_floor_calib.py day_logs/et_camera_<1回目> day_logs/et_camera_<2回目> --set-default
git add wireless_device/et_rally_camera_calib/calib_*_floor.json robot_program/et_rally_start_pose.py
git commit -m "当日の床での較正(C、full でスタートまで走って撮った)"
git push
```

ロボットで `git pull`。

## 5. 周回で確かめる

いつもの full で左右を走り、ログを lap_start_fix.py で見るだけ実行する(左右とも、スタートの残りが数mm・0.3°以内なら良い)。

```
python wireless_device/et_rally_camera_calib/lap_start_fix.py day_logs/run_*.log
```
