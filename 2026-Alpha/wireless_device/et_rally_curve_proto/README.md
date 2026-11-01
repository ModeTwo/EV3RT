# ETラリー 往復・後退・曲線の経路の試作ツール（2026-11-01）

検討用の試作です。本番のplanner（`wireless_device/et_rally_planner/`）とは独立していて、どこからも読み込まれません。
ここで作った計画JSONは、`robot_program/tests/plan_seed9392783_right_curve_1101*.json` にあります。

座標は右コースのplanner座標です（plan JSONの `gates` そのまま。スタートとゴールと進入禁止は `et_rally_runner.py` の左右反転のあと）。
Pythonは `C:\Users\MSAD\AppData\Local\Programs\Python\Python312\python.exe`（numpyが必要）です。

## 流れ

```
# 1. ゲートの通り方（前進・後退・前進→後退の往復・後退→前進の往復）を、4周＋ゴールの時間が最短になるよう選ぶ
#    引数: ゲートのJSON マージン(前の部分のすき間cm) 周回数 出力 後ろの部分のすき間cm
python -I shuttle_plan.py gates_9392783.json 1.5 4 route.json 0.4        # 1本あたり数秒〜40秒

# 2. つなぎ区間を「円弧→直進→円弧」などに置き換える（後処理）
#    CR_START_FWD=1: スタート直後は前進だけ。CR_TAIL_CM: 止まる直前に足す直進(cm、既定4)
CR_START_FWD=1 python -I curve_route.py gates_9392783.json route.json curved.json

# 3. 実機の計画JSON（move/turn/arc、reverse/stop）にする。右コースは gyro = -planner角
python -I to_plan.py curved.json gates_9392783.json plan.json
```

| ファイル | 内容 |
|---|---|
| shuttle_plan.py | 通り方の探索（動的計画法）。時間の近似式: 旋回 0.7+0.012×度 秒、直進 0.5+0.0025×mm 秒 |
| curve_route.py | 曲線への置き換え。半径は20/15/12/10cmから、当たらずに最速のもの。円弧は外側の車輪が巡航を超えない速さ（1/(1+5.85cm/半径)） |
| to_plan.py | 計画JSONへの書き出し。止まる場所（旋回の前・前後の切り替え・最後）を決め、ゴールの向きの旋回を足す |
| sim_path.py | 机上の確認。仮のロボット（運動学だけ、慣性・すべりなし）で、`steps_from_strategy` が作るノードを実際に走らせる。旋回は目標の向きにぴったり合ったことにする |
| cmp_nodes.py | 2つのツリー（例: devSSとこのブランチ）で、同じ計画から作られるノードを比べる（今の3周版が変わっていないことの確認用） |
| seg_err.py | 走行ログの区間ごとの位置（et_loc）と、計画上の位置の差 |
| gen_seeds.py | ほかのシードのゲート配置を作り（ローカルplannerの random_gates）、今のplannerの経路を同じ近似式で時間にする |
| draw_route.py | 経路図のSVG |

## わかっている限界
- 探索は、つなぎ区間を「直進」か「中継点1つを経由する直進2本」に限っています。
- 曲線は、通り方を決めたあとの後処理です。曲線を前提に通り方から探し直すと、選ばれる通り方が変わる可能性があります。
- ぶつかる判定は、車体の前の部分が支柱（T字パーツ込み）から1.5cm、後ろの部分が0.4cm（今のplannerは後ろを見ない）。進入禁止は車軸だけを見ます。
- 本番のplannerの目安（5秒）より遅いです。
