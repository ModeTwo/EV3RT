# ETラリー 当日の手順（v3、2026-11-15）

v2（11/14）からの主な変更:
- 本番の `--mission full` で、新しい経路の計画（PC から丸ごと届く）とカメラ（ETラリーの始めに本体から受け取る）が使えるようになった。rally-drive で個別に確かめ済み、`full` の通しはまだ（TODO）。
- 試走の較正は「止まって撮る試験 → `day_floor_calib.py`」の1本になった（カメラの取り付けの補正とスタートの読み違いの補正 `start_fix` を一度に出す）。
- 走りの既定が増えた（5章）。

コマンドはすべて `2026-Alpha` フォルダで動かす（PC は `C:\Users\MSAD\EV3RT\2026-Alpha`、ロボットは `~/EV3RT/2026-Alpha`）。
**PC の `python -m wireless_device ...` を別のフォルダで動かすと `No module named wireless_device` になる。**
未確定の所は「TODO」。

---

## 0. 前の日まで
1. devSS の最新を、PC とロボットの両方に入れる（ロボットで `git pull`）。
2. 電池を充電する。周回数は走りながら持ち時間に合わせる。
3. **ハブの版を確かめる**: ロボットで何か走らせて、起動のメッセージが `SPIKE asp.bin version=0.1.1 expected=0.1.1` であること。
   `0.1.0` なら、ハブが元の版に戻っている（11/08・11/15 に起きた）。書き込み直す手順は `hub_firmware/README.md`
   （ハブを書き込みモードにして `cd ~/libraspike-art/spike && sudo ../scripts/deploy-dfu.sh ~/asp_backup/asp_brakecal_904096d_v011.bin`）。
4. **カメラの取り付けを確かめる**: 指で軽く押して角度が変わらないこと。11/15 は1日で2回動いた（動くと較正のやり直しが要る）。
5. **前もって作った計画を用意する**（PC から送る分と、ロボットの予備の分）。
   - PC: `C:\Users\MSAD\EV3RT\et_rally_plans`（`plan_<配置の鍵>.json`。11/15 時点で1000配置のうち999）。
     作り直したときは `python wireless_device/et_rally_curve_proto/export_plans.py <計画のフォルダ> --out <出力> --check-json <results.jsonl>`。
   - ロボット（PC の計画が届かないときの予備）: `~/et_rally_plans` に同じものを置く（`plans.zip` だけでもよい）。
     例: `scp C:\Users\MSAD\EV3RT\et_rally_plans\plan_*.json shunsuke09@<ロボット>:~/et_rally_plans/`
   - TODO: 全4872配置の計算は、経路の点検のあと。計画の探し方の順番（前もって作った計画が先か、その場で計算が先か）は未決。

## 1. 自由時間（会場の床、較正台）
カメラに触れた・ぶつけた・向きが変わったかもしれないときだけ行う。迷ったら行う（5分）。
1. 較正台の画像を3枚撮る（1枚ごとに置き直す、アームは上げる）。ロボットで:
   ```
   python3 et_rally_camera_capture.py --tag jig --count 1
   ```
2. **PC で**較正し直す(ロボットでは動かない。ロボットには cv2 がない。画像は PC がロボットから取ってくる):
   ```
   cd C:\Users\MSAD\EV3RT\2026-Alpha
   python wireless_device/et_rally_camera_calib/recalibrate_day.py --set-default
   ```
   - ばらつきの警告が出たら撮り直す。commit・push し、ロボットで `git pull`。
   - **較正台で較正し直したら、2章の床での較正を必ずやり直す。** `recalibrate_day.py` は、取り付けの補正と `start_fix` を引き継がない。

## 2. 試走（コースの上、約30分）
### 2-1. 止まって撮る試験（2回、約1分）
- ゲートを外し、ロボットをスタートに置く（数cm ずれてよい）。2回目は左に3cm くらいずらして置くとよい。
  ```
  python3 alpha.py right --mission rally-drive --rally-start-pose on --rally-camera-log 1 --rally-camera-exposure 30 --rally-straight-pause-sec 1.5 --rally-straight-pause-labels cal1,cal2,cal3,cal4,cal5 --rally-plan-path robot_program/tests/plan_test_selfcal_stops_1114.json
  ```
  15cm 進んで1.5秒止まる、を5回して終わる。**最後まで走らせる**（途中で止めると止まった場所が足りない）。
- PC でログを取ってくる: `python wireless_device/fetch_run_logs.py`(名前が引けないときは `--host shunsuke09@<ロボットのIP>`)。
  保存先は既定で `C:\Users\MSAD\Downloads\et_run_logs`。

### 2-2. 床での較正（PC、約1分）
```
python wireless_device/et_rally_camera_calib/day_floor_calib.py C:\Users\MSAD\Downloads\et_run_logs\et_camera_<1回目> C:\Users\MSAD\Downloads\et_run_logs\et_camera_<2回目> --set-default
```
- 出る値の目安（11/15）: 取り付けの補正 向き +0.4°・傾き −0.6°・高さ +2.5mm、`start_fix` 前 −3・左 +17mm・向き −1.3°。
- 「回ごとのばらつきが大きい」の警告が出たら、もう1回走らせて3回分で出し直す。
- commit・push し、ロボットで `git pull`。

### 2-3. 確かめの走行
- 通しで走らせる（できれば本番と同じ `full` で）。ログで:
  - `start pose:` で位置が出ていること（`not enough frames` が出ていない）。
  - `camera handover: main camera closed`（`full` のとき）。
  - `camera correct: started ... fix_heading=True`、ガレージで止まったあとの `path: resync ...`。

## 3. 本番
1. **PC**（ヒントを受けて計画を送る役）を先に起動する。`2026-Alpha` フォルダで:
   ```
   python -m wireless_device --host <ロボットのIP> --curve-plans-dir C:\Users\MSAD\EV3RT\et_rally_plans
   ```
   - その場で計算もさせるなら `--curve-compute-sec <秒>` を足す（配置36で約16秒）。
   - ロボットの IP は `ping` で探せる（11/15 は 10.56.69.191。名前 `etrobo-raspi.local` が引けない日がある）。
2. **ロボット**: `python3 alpha.py <right|left> --mission full`（画面がなければ映像の表示は自動でやめる）。
   ロボットは PC がつながるまで待つ（`Waiting for PC connection` → `PC TCP connection confirmed`）。
   `full` では、つながったあとにロボットの画面で **4桁の復号キー**（ヒント2を解くもの）の入力と確認（y）を求められる。
3. 流れ（ロボットが自動で行う）:
   - 前の工程でヒントを読んだら PC へ送る。PC は配置の鍵で計画を探し（なければ計算・ガレージだけ）、丸ごと送り返す。
     PC のログ `Curve plan mission=... source=...`、ロボットは `run_logs/received_plan.json` に書く。
   - PC から届かなければ、ロボットが自分のヒントで `~/et_rally_plans` を探し、なければガレージだけの計画で走る。
   - ETラリーの始めに、本体のカメラを閉じて ETラリーのカメラが開く（約2秒。ジャイロの測り直しの間に収まる）。
   - スタートで位置と向きを取り込み（`start_fix` で読み違いを直す）、ずれていればつなぎを引き直す。
   - アームは、届いた計画で間に合うなら走りながら上げる（ログ `arm (received plan): ...`）。
   - 持ち時間 = 120 − 競技の開始（タッチ）からの秒。各周の終わりに、間に合わなければガレージへ切り上げる（1周目のあとは切り上げない）。
4. TODO（未確認）: `--mission full` の通し（前の工程からつないで）、左コースで今日の新しい設定を通すこと。

## 4. 走ったあと
- 毎回、カメラ補正が効いていたかも見る: `camera correct: finished {...}` の used が seen の4割前後、resid(合わずに捨てた数)が20前後まで、`pitch now ... (median resid ...)` が3〜6mm、`start pose:` の pitch_fix が +0.4〜+1.5° くらい。外れていればカメラが動いた疑い → 1章・2章の較正(11/16 は走行の途中で動いた)。
`python wireless_device/fetch_run_logs.py`。見る所: `TIME BUDGET lapN done`、`camera correct: finished`、`path: resync`、`RALLY_BATT`。

## 5. 参考: 今の既定（2026-11-15）
- 走り: 円弧・直進は出力100、円弧の先読み 0.08秒。止まった状態からの走り出しは最初の80mm を出力40から上げる（スタートの空回りを防ぐ）。
- 位置: 止まらずに続く区間の始めに、横のずれをコース上の位置（カメラで直したもの）から取り直す（周ごとに通過位置がゴールから離れる側へずれるのを防ぐ）。
- カメラの補正: 位置は向きをジャイロに固定して当てはめる（1か所の画像では向きと左右の区別がつきにくい）。向きの補正は弱く（10%）入れる。円弧を回り終えて0.5秒は強い補正をしない。
- スタート: 見えたもの2つ以上で下向きも当てはめる。較正ファイルの `start_fix` で読み違いを直す。ジャイロの測り直しが終わったら（6枚以上あれば）撮影を打ち切る。
- 経路の道具: すき間3cm から（通れなければ斜めの通過 → 2.5 → 2.0 → 1.5cm）。最後の周の黄は2cm 以上。
- 試験の道具: 止めて定規で測る `plan_layout36_to_{red,blue,yellow}_1115.json` と `ruler_check.py`、まとめた当てはめ `floor_fit2.py`。
