# 当日のカメラ較正(床で止まって撮る試験ができないとき)

2026-11-18。ロボット C。床で止まって撮る試験(day_floor_calib.py)の代わりに、較正台の較正と周回の試走のログで start_fix を決める。

コマンドは PC では 2026-Alpha で、ロボットでは `~/EV3RT/2026-Alpha` で打つ。

## 1. 較正台で撮る(ロボット)

1枚ごとに較正台に置き直し、これを3回くり返す。

```
python3 et_rally_camera_capture.py --tag jig --count 1
```

## 2. 較正し直す(PC)

床での較正の値(取り付けの補正と start_fix)は、今の C のファイルから仮に引き継ぐ。

```
python wireless_device/et_rally_camera_calib/recalibrate_day.py --base wireless_device/et_rally_camera_calib/calib_11161509_floor.json --keep-floor --out calib_C_day_jig.json --set-default
```

- 「引き継ぎ元との差」と警告を確認する。下向きが 16〜20° くらいなら、カメラの向きは普段どおり。
- ばらつきの警告が出たら、1 から撮り直す。

結果を commit・push してから、ロボットで pull する。

```
git add wireless_device/et_rally_camera_calib/calib_C_day_jig.json robot_program/et_rally_start_pose.py
git commit -m "当日の較正台の較正(C)"
git push
```

ロボットで:

```
git pull
```

## 3. スタートで確かめる(ロボット)

スタートに目視でまっすぐ置き、アームを動かすだけの計画で取り込みを1回する。

```
python3 alpha.py right --mission rally-drive --rally-gyro-hub-recal-sec 2 --rally-gyro-drift-settle-sec 1.0 --rally-start-pose on --rally-camera-log 5 --rally-camera-exposure 30 --rally-arm-stop-ticks 3 --rally-arm-power 40 --rally-arm-travel-deg 260 --rally-plan-path robot_program/tests/plan_test_arm_only_1111.json
```

ログの `start pose:` で、次の2つを満たしていれば合格。

- 点が1つ以上見えている(dots>=1)。
- 計画のスタートからのずれ(plan start …)が数mm〜1cm。

## 4. 周回の試走を3〜5回(いつもどおり)

スタートには目視で置いてよい。2 のあとの回だけを使う。

## 5. 周回のログから start_fix を直す(PC)

ログを取ってくる。日時は試走したときのものにする。

```
mkdir day_logs
scp "shunsuke09@10.56.69.191:EV3RT/2026-Alpha/run_logs/run_<日付>T*.log" day_logs/
```

まず見るだけ実行する。

```
python wireless_device/et_rally_camera_calib/lap_start_fix.py day_logs/run_*.log
```

- 回ごとの「残り」(左・向き)と平均を確認する。
- 警告が出たら確認する。「較正ファイルが違う」と出たら、2 の前の回が混ざっている。
- 左が数mm・向きが 0.2° 以内なら、直さなくてもよい。

よければ、較正ファイルに書いて既定にする。

```
python wireless_device/et_rally_camera_calib/lap_start_fix.py day_logs/run_*.log --out calib_C_day_lap.json --set-default
git add wireless_device/et_rally_camera_calib/calib_C_day_lap.json robot_program/et_rally_start_pose.py
git commit -m "当日の start_fix を周回のログから(C)"
git push
```

ロボットで:

```
git pull
```

## 補足

- 前後(front)は既定では元の値のままにする。走りながら読むと、前へ4〜6mm出る癖があるため。
- 11/17 の C の周回3本で試した結果、左 +15.3mm・向き -1.19° になった。今の値(+15.0・-1.19)とほぼ同じ。
- 回ごとのばらつき(標準偏差)は 5.3mm・0.24° だった。3〜5回の平均で、左は約2〜3mm・向きは約0.1〜0.15° の精度になる見込み。
