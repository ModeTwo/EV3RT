# ETラリー 往復・後退・曲線の経路 引き継ぎ（2026-11-01）

検討セッションで、ゲートの往復通過・後退・つなぎ区間の円弧化を試作し、実機で4周まで走らせた結果のまとめです。
**4周＋ゴールを、タッチから64.4秒で走りきりました**（目標の70秒以内）。ゴールのずれは後ろへ50mm・左右1〜2mmで、前後の精度はまだ合わせ込む必要があります。

## 1. 置き場所

| もの | 場所 |
|---|---|
| ブランチ | `devSS-curve`（devSS の 4aa64d3 から分岐。push済み） |
| PCのworktree | `C:\Users\MSAD\EV3RT\.claude\worktrees\devSS-curve` |
| ラズパイ | `~/EV3RT-curve`（`git worktree add ~/EV3RT-curve devSS-curve` で作った別フォルダ。`~/EV3RT` の devSS には触れていない） |
| 走行ログ | ラズパイの `~/EV3RT-curve/2026-Alpha/run_logs/`。PCには `Downloads\run_20261101T06*.log` |
| 経路の試作ツール | `2026-Alpha/wireless_device/et_rally_curve_proto/`（README.md あり） |

## 2. 実装（devSS-curve）

| ファイル | 内容 |
|---|---|
| `robot_program/behaviours/et_rally_path_segment.py`（新規） | ETラリー専用 `EtRallyPathSegment`。直進と円弧を、前進/後退・止まる/止まらずに続ける、で走る |
| `robot_program/features/execute_strategy.py` | 計画JSONの `type: "arc"`、`reverse`、`stop` を読む。**どれかがある計画のときだけ**新しいクラスを使う |
| `robot_program/config.py`、`alpha.py` | `--rally-path-lead-sec`（既定0.06）、`--rally-path-arc-pid-i`（既定0.6） |

**今の3周版への影響はありません。** `plan_seed9392783_right_1031b.json` から作られるノードは、devSS と完全に一致することを確かめました（`et_rally_curve_proto/cmp_nodes.py`）。

### 計画JSONの追加項目
```json
{"type":"move", "target_heading_deg":-90.0, "distance_mm":705, "reverse":true, "stop":true, "label":"lap2-red"}
{"type":"arc",  "target_heading_deg":-143.88, "sweep_deg":-143.88, "radius_mm":100, "reverse":false, "stop":false}
```
- `target_heading_deg` の規約は今と同じです（右コースは gyro = −planner角）。arc では終わりの向き、`sweep_deg` は符号付きの向きの変化（始めの向き = 終わり − sweep）、`radius_mm` は車体中心の半径です。
- `stop:false` の区間は止まらずに次へ続けます。次がturn・計画の終わり・前後が逆になる区間なら、自動で `stop:true` になります。

### EtRallyPathSegment の動き
- 向きは、進んだ距離に合わせて目標を動かし、今の直進と同じジャイロのPIDで追います。円弧では、半径から決まる左右の差（フィードフォワード）を足します。
- 円弧の中の出力は、外側の車輪が巡航を超えないよう 1/(1+58.5mm/半径) にします。左右の出力は、左右の差を優先して±100に収めます（はみ出したtick数は summary の `sat=`）。
- 左右の差だけを `lead_sec × 今の速さ` だけ先読みします（車輪の遅れの分、曲がり始めと終わりを早める）。目標の向きは先読みしません（先読みすると曲がり始めが早すぎた）。
- 止まらずに続く区間では、次の区間の距離を「計画上の終わり」から数え、PIDも引き継ぎます（区間の切り替わりの刻みが、たまらないように）。減速は、止まるまでに続く区間全体の残り距離で決めます。
- 円弧の中だけ、向きのPIDのIを `--rally-path-arc-pid-i`（既定0.6）にします。直進は `--rally-move-pid-i` のままです。
- 止まる区間の終わりではブレーキをかけ、左右のエンコーダが止まってから次へ進みます（前後の切り替えに備える）。手前で終える量は今の直進と同じ（`move_stop_lead_mm` 16mm、50mm未満は `--rally-move-short-lead`）。

## 3. 試走の結果（seed9392783 右コース）

| ログ | 計画 | 設定 | タッチ→終了 | ゴール（実測） |
|---|---|---|---|---|
| 061925 | 1周 | 先読み0.06、円弧も直進用のI=8 | 25.3秒 | 未計測（ログ上約36mm） |
| 062754 | 1周 | 0.06、円弧I=0.6、±100に収める | 24.5秒 | 後ろ22mm（左右は測り間違い） |
| 063540 | 1周 | 0.12 | 25.7秒 | 後ろ20mm、左右2〜3mm |
| 064715 | 1周 `_lap1_tail` | 0.12、止まる直前は直進 | 25.9秒 | **前5mm、左2mm** |
| 065319 | **4周** `_tail` | 同上 | **64.4秒** | **後ろ50mm、右1〜2mm** |

4周の内訳は、準備6.1秒（アーム上げ約2.1秒＋ジャイロの静止測定4秒）、走行58.3秒です。旋回は全部で6回だけです（スタート直後1回、赤を出たところ4回、ゴール1回）。

起動コマンド（4周）:
```
cd ~/EV3RT-curve/2026-Alpha
python3 alpha.py right --mission rally-drive --rally-plan-path robot_program/tests/plan_seed9392783_right_curve_1101_tail.json --rally-hint1 25,35 --rally-hint2-gate-info 53,54/12,22 --rally-turn-method one_shot --rally-turn-phase1-model 0.961,6.9,0.913,8.9 --rally-turn-lag-compensation-adaptive on --rally-turn-left-target-scale 1.009 --rally-turn-backlash-preload-deg 4.2 --rally-turn-tolerance-deg 0.6 --rally-turn-pre-stop on --rally-gyro-float on --rally-gyro-hub-recal-sec 3 --rally-move-carry-i on --rally-move-fine-power on --rally-move-pid-i 8 --rally-turn-wheel-sync 2 --rally-turn-sync-decel 40,40 --rally-turn-power 50 --rally-move-short-lead 10 --rally-path-lead-sec 0.12
```
（10/31の1031bの起動オプションから、5秒停止のオプションを外し、計画と `--rally-path-lead-sec` を変えたもの）

## 4. わかったこと

1. **止まって終わる円弧は、そのたびに約1cmずれる。** 063540までの3回とも、赤の手前とゴールの手前の円弧で、後ろへ約+10〜13mmずつ増えていた。止まる直前を40mmの直進にし、スタート直後を「直進→止まる→その場で95°旋回」にした `_tail` 版で、1周のゴールが後ろ20mm→前5mmになった（2つを同時に変えたので、それぞれの効き目は分けていない）。理由は特定できていない（終わりの減速・手前で終える処理・止まるときの惰性が円弧の中で起きること）。
2. **先読み0.12秒で、途中の円弧ではずれがたまらなくなった。** 0.06秒だと円弧のたびに数mmずつ増えた。円弧の入りでの向きの遅れ（err_max）は、まだ5〜8°ある。
3. **円弧は遅い。** 半径10cmで約175mm/s（直進は約360mm/s）。止まらずに走る区間の時間の約半分が円弧。
4. **4周の前後のずれ（後ろ50mm）の内訳**:
   - ログに出ている約24mm: 1周に約+5mmずつ増える。増えるのは毎周「黄を通って止まる→前後を切り替えて後退の円弧」のところ。
   - ログに出ていない約26mm（1周約6.5mm）: **後退の距離の較正**が一番疑わしい（1周に後退が約1m。タイヤ径は前進の惰性込みで較正した値）。まだ測っていない。確認用に `plan_path_straight_1000_forward.json` / `_reverse.json` を追加した（新しいクラスで1000mmを前進/後退）。
5. 左右は実測でよく合っていた（1〜2mm）。ログでは赤を出たところの旋回のたびに横へ約10mm跳ぶが、実際の位置には出ていない（旋回の中心が接地点の約9.5mm後ろにある分が、ログの計算に出ているだけと思われる）。

## 5. 経路の作り方の要点（試作ツール）
- 赤→青→黄の順番は規則で決まっているので、ゲートごとの通り方（前進・後退・前進→後退の往復・後退→前進の往復 × どちら側から）を、動的計画法で選ぶ。前の部分は支柱（T字パーツ込み）から1.5cm、後ろの部分は0.4cm離す。黄の奥の制限は入れていない（ユーザー了承）。
- seed9392783 では往復は選ばれず、「黄を出たら後退で赤の上へ戻り、そのまま後退で赤を通る」形になった。ほかの6シードでは、6つとも往復（主に青）が選ばれた。近似式では、曲線版は7シードすべてで今のplannerの3周より短い（4周で48.7〜65.8秒＋準備）。
- 計算は1本あたり数秒〜40秒で、本番のplannerの目安（5秒）より遅い。

## 6. 次の候補
1. 後退1000mmの較正（実測と前進との差）→ 4周の前後のずれの約半分の説明がつくか。
2. 「黄で止まる→後退の円弧」の部分のずれ（1周約+5mm、ログに出ている）の切り分け。
3. 円弧の半径を15cmにした経路（円弧が速くなり、内側の車輪の負担も減る）。
4. 4周の繰り返し（ばらつき）と、周回ごとの位置の実測。
5. 本番のplannerへの組み込み（今の試作は検討用で、計算時間も長い）。
