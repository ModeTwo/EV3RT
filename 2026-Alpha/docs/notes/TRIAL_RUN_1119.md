# 11/19 試走でやること

ロボットは C。コマンドはすべてロボットのラズパイで、`~/EV3RT/2026-Alpha` で打つ。
PC と git のやり取りは最初の `git pull` だけで、あとはロボットの上で完結する。

## 時間が足りないときの順番

1. 準備(pull と点検)
2. 止まって撮る較正(右で1回)と `--auto`
3. 右の full を1回
4. 左の full を1回(左の新しい LAP 経路と、LAP の止め方を左右1回ずつ見られる)
5. 時間があれば、左右とも3回ずつになるまで full をくり返す

## 1. 準備(試走の前、机の上で)

```
cd ~/EV3RT/2026-Alpha
git pull
python3 preflight_1119.py --camera
```

- 最後に `=== 全部 OK: 試走のコマンドを打って大丈夫 ===` と出れば準備完了。モーターは動かない。
- NG が出たら、その画面を写真で送る。
- 確かめる中身: 版(655684d0 以降)、部品(OpenCV・numpy・暗号・QR を読む zxingcpp)、alpha.py のオプション、
  ヒント2の復号、カメラの較正、床の較正の道具、計画(1回目3cm・2回目2cm の選び方)、LAP の経路、カメラで1枚撮れるか。

## 2. 止まって撮る較正(右で1回)

LAP のスタートに置いて走らせる。ETラリーのスタートから、15cm 進んでは止まって撮るのを5回くり返す。
75cm 進んで止まるので、手で回収する。

```
python3 alpha.py right --mission full --lap-bottle-aim --rally-plan-path robot_program/tests/plan_test_selfcal_stops_1114.json --rally-camera-log 1 --rally-straight-pause-sec 1.5 --rally-straight-pause-labels cal1,cal2,cal3,cal4,cal5
```

続けて、ロボットの上で計算する(数分かかることがある):

```
python3 wireless_device/et_rally_camera_calib/day_floor_calib.py --auto --runs 1
```

- `=== 合格 ===`: 終わり。次の走行から自動でこの較正を使う。コマンドに何も付けなくてよく、git も要らない。
- `=== 不合格 ===`: 理由が出る。2の走行からもう1回やり直す。それでも不合格なら、そのまま3へ進んでよい(床の補正なしで走る)。
- 「参考: 1回だけ」と出るのは問題ない。
- 合格した較正はそのラズパイの中だけ(LOCAL_CALIB.txt)に記録される。`git pull` しても消えない。

## 3. 左右でいつもの full

```
python3 alpha.py right --mission full --lap-bottle-aim --rally-strategy local
python3 alpha.py left --mission full --lap-bottle-aim --rally-strategy local
```

### 毎回見ること

- 使った較正: ログの最初に `ET rally camera calibration: calib_..._floor.json (LOCAL_CALIB.txt)` と出るか
  (2が不合格だったときは `calib_C_day_jig2.json (default)`)。
- ヒント2: 復号できたか(ログの `QR_STATS`・`QR_RECOVER`)。復号キーは4文字なら記号入りでも入力できる。
- LAP: `LAP_CAPTURE`(ボトルを捕まえた位置)と、止まった理由(`LAP_CAPTURE_STOP` / `source=bottle_capture` / 黒から120mm)。
  ヒント1のオブジェクトにアームの先が当たらないかは目で見る。
- 左コース: 2.9〜3.8m で、緑の禁止エリアに入らないか(目視)。
- 運搬のあと、床の赤→白を見つけたか。
- 相撲: 捕まえた判定が出たか。
- ETラリー: ゲートに当たらないか、周回中に大きな補正が入らないか(目視)。

### スタートのずれを見る(走ったあと)

```
ls -t run_logs/run_*.log | head -1
python3 wireless_device/et_rally_camera_calib/lap_start_fix.py run_logs/run_<日時>.log
```

- 残りが数mm・0.3°以内なら、較正は合っている。
- 左コースは、右で求めた補正を左右反転して使っている。左で残りが 1cm・0.5° を超えたら、そのログを送る。
- 左右とも大きくずれ、床の較正で悪くなったと思ったら、元の較正に戻す:
  ```
  python3 wireless_device/et_rally_camera_calib/day_floor_calib.py --clear
  ```

## 4. 試走のあいだに決めること(本番の前に)

1. LAP の止め方: ボトルを捕まえた位置から40mm で止めるのを使い続けるか。
   やめるときは本番のコマンドの最後に `--lap-capture-stop-mm 0` を付ける。
2. 左の LAP 経路: 新しい経路のまま使うか。悪くなったら元に戻す(commit はしない):
   ```
   cp robot_program/start_lap_profile_left_right_copy_1118.py robot_program/start_lap_profile_left.py
   ```
3. 床の較正: そのまま使うか、`--clear` で元に戻すか。

## 困ったとき

ログのファイル(`run_logs/run_<日時>.log`)を1つ送ってもらえれば見る。

本番のコマンド(1回目・2回目)は docs/notes/ET_RALLY_PLAN_1119.md の「5. 本番」。
