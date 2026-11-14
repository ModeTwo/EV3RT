# ETラリー 当日の手順（v2、2026-11-14）

v1（11/12）からの主な変更:
- 当日の時間の流れ（自由時間 → 試走 → 本番）に合わせて章を分けた。
- 試走でコースの上のカメラのずれを測る（`floor_selfcal.py`）を追加した。11/14、カメラは較正から向き約2°・左右・前後が数mm ずれていた。較正台だけでは直らない。
- 予備の計画（スタートからガレージへ行くだけ）を追加した。

コマンドはすべて `2026-Alpha` フォルダで動かします（PC は `C:\Users\MSAD\EV3RT\2026-Alpha`、ロボットは `~/EV3RT/2026-Alpha`）。
**未確定の所は「TODO」と書いています。**

---

## 0. 前の日まで
- devSS の最新を、PC とロボットの両方に入れておく（ロボットで `git pull`）。
- 電池を充電しておく。電圧で速さが数%変わるが、周回数はロボットが走りながら合わせ直す（`--rally-time-budget`）。
- TODO: 全配置の計画を前もって作り、ロボットに入れておくか。計画1つは約35KB で、4872配置で約180MB、zip にすると約11MB。ロボットの容量では問題ない。git には入れず、zip で別に持たせる。
- TODO: ヒントからゲートの配置を読み、計画を選ぶ流れ（ヒントからETラリー開始まで約15秒）。まだ作っていない。

## 1. 自由時間（会場の床、較正台）
カメラに触れた・ぶつけた・向きが変わったかもしれないときだけ行う。迷ったら行う（5分）。

1. **較正台の画像を3枚撮る**。1枚ごとに置き直し、アームは上げた状態で撮る。台が床でガタつかないことを確かめる。ロボットで:
   ```
   python3 et_rally_camera_capture.py --tag jig --count 1
   ```
2. **PC で較正し直す**:
   ```
   python wireless_device/et_rally_camera_calib/recalibrate_day.py --set-default
   ```
   - ばらつきの警告が出たら撮り直す。
   - 問題なければ commit・push し、ロボットで `git pull`。
   - 較正台で戻せるのは大きなずれまで。向き・左右・前後の細かいずれは、2章の試走で詰める。

## 2. 試走（コースの上、約30分）
目的は2つ。(a) コースの上でカメラのずれを測って直す、(b) 通しで走れることを確かめる。

### 2-1. 1回目: カメラのずれを測る走行
- 競技の開始から通しで走らせる。ETラリーのコマンドに `--rally-camera-log 1` を付ける（画像を毎枚保存する）。
- できれば、ETラリーのスタート位置にロボットを置いた状態で（数mm ずれてよい）、走り出す前に2秒止まる。走行の最初の画像が、止まった画像として使える。
- 走ったら、PC でログを取ってくる:
  ```
  python wireless_device/fetch_run_logs.py
  ```

### 2-2. PC でずれを出し、較正ファイルに書く
`<dir>` には、取ってきた `run_logs/et_camera_<時刻>` を入れる。
```
python wireless_device/et_rally_camera_calib/floor_selfcal.py motion <dir>
python wireless_device/et_rally_camera_calib/floor_selfcal.py static <dir>
python wireless_device/et_rally_camera_calib/floor_selfcal.py write --yaw <motionの値> --lat <staticの右> --front <staticの前> --out wireless_device/et_rally_camera_calib/calib_day_selfcal.json
```
- **motion**（向き）: 置き方によらない。「中央値の誤差の目安」が ±0.5° より大きければ、画像が足りない。もう1回走らせるか、向きの補正だけは入れない。
- **static**（前後・左右）: スタートに置いた正確さ（数mm）がそのまま入る。値が 3mm 未満なら書かなくてよい（`--lat 0 --front 0`）。
- 書いたら、`robot_program/et_rally_start_pose.py` の `DEFAULT_CALIB` を新しいファイルにして、commit・push し、ロボットで `git pull`。
- TODO（11/15 に実機で確かめる）: 毎枚保存したとき、ロボットの処理が遅れないか。motion の精度。左右のずれが場所で変わる件。

### 2-3. 2回目: 確かめの走行
- 同じく `--rally-camera-log 1` で通しで走らせ、2-2 の motion をもう一度出す。±0.5° 以内なら合格。
- ログの `start pose:` で、点かマーカーが3つ以上見えて（`items>=3`）、食い違い（`resid`）が 5mm 以下であること。下向き（`pitch_fix`）は、ロボットが覚えて次から使う。

## 3. 本番
1. **計画を選ぶ**。
   - TODO: ヒントからの自動の選び方はまだない。今は、配置に合わせた計画（`robot_program/tests/plan_<name>.json`）を走らせる。
   - 計画が見つからない・経路が作れない配置（例: 942）のときは、**予備の計画** `robot_program/tests/plan_garage_only_1114.json` を使う。スタートからガレージへ行くだけで、ガレージで止まる5点だけを取りにいく。約5.5秒。どの配置でも脚に当たらない（950配置で最小3.6cm）。
2. **スタートに置く**。
   - スタートのカメラで、前後・左右・向きを取り込み、ずれていれば、スタートの直後のつなぎを引き直す。
   - 引き直しでは、最初のゲートまでの脚とのすき間（目標3cm）を見る。
   - 大きくずれていると（黄の側に4cm など）、走り出しにその場旋回や後退が入る。その計算に約1秒かかる。
   - 本番はタッチなしで始まる。試験で使っている `--rally-pre-move-mm 50` は付けない。TODO: 本番の流れの中でのつなぎ方。
3. **持ち時間**: `--rally-time-budget` を「120 − ETラリーが始まるまでの秒」にする。
   - ロボットは各周の終わりに残り時間を見て、次の周とガレージまで行けるかを判断する。行けなければ、その周の黄ゲートの抜けからガレージへ向かう（1周目のあとには切り上げない）。
   - ガレージは120秒までに止まればよい。
   - TODO: 「120 − 開始までの秒」の受け取り方は未実装。

## 4. 走ったあと
```
python wireless_device/fetch_run_logs.py
```
- 周回の判断は `TIME BUDGET lapN done`、電圧は `RALLY_BATT`。
- 走行中のカメラの補正の最後の `camera correct: finished` に、使った枚数・強く直した回数・最後の下向きが出る。

## 参考: 今の既定（2026-11-14）
- 円弧・直進は出力100、円弧の先読み `lead_sec` 0.08。脚とのすき間が2.5cm 未満の直進は、出力の上限92。
- 経路の道具:
  - すき間は3cm から。3cm で通れない配置は、3cm のまま全部のゲートで斜めの通過を試す（narrow_oblique_grid）。それでも通れなければ2.5→2.0→1.5cm。
  - 選んだすき間で作れなければ、次のすき間で作り直す。
  - 最後の周の黄は、すき間2cm 以上の通り方だけ。
- 計画の独立検査: `wireless_device/et_rally_curve_proto/check_plan.py <plan> wireless_device/et_rally_curve_proto/params_day.json`。day_route.py は最後に自動で行い、結果を表示するだけ（NG でも走らせる）。
- スタートの取り込み:
  - 見えたもの（点+マーカー）が2つ以下なら、前に床で当てはめて覚えた下向きで写し直す（11/14 修正）。
  - 点が見えずマーカーだけなら、向きと左右だけを入れる。
