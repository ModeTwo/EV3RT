# ETラリー 制御モデル再較正ログ(DD-19〜)

v1(`ET_RALLY_MODEL_SURVEY_v1.md`)のDD-01〜18のうち、DD-13(旋回方式)・DD-14(直進PIDゲイン)・
DD-15(キャスター引きずり補正)・DD-16(横ズレ補正)・DD-17(ジャイロ/タイヤ径較正)を再オープンし、
補正なしベースラインから1つずつ積み上げ直す。新しい決定はこのファイルにDD-19以降として記録する。

## 進め方

1. 補正・工夫を一切行わない状態(ベースライン)で、直進・旋回それぞれの誤差を実測する
2. 誤差の原因候補を1つずつ検証し(仮説→実測→棄却/特定)、特定できたものから対策を入れる
3. 対策ごとに、ベースラインと比較してどれだけ誤差が縮んだかを記録する
4. 6枚目(制御モデル)の最終的な記述は、この一連の記録から構成する

## 今回整えた試験環境

- `--mission rally-drive --rally-plan-path <ファイル>` で、PC通信なしに経路ファイル単体のテストができる(DD-12で既存)。直進専用: `robot_program/tests/plan_straight_100cm.json`/`200cm.json`、旋回専用: `plan_spin_alternating30.json`、旋回回数比較用: `plan_fewturns_square.json`(4旋回)/`plan_manyturns_octagon.json`(8旋回、総距離・総旋回角度は前者と同じ)、フル周回: `plan_seed9392783.json`(3周・旋回22回・約8m)
- `--rally-turn-method {spin_pid,spin_encoder,one_shot,lqi}`(新設)で旋回方式を切替。`spin_pid`がDD-13以前の無補正PIDベースライン(gyro_drive.SpinAround)
- `--rally-compensation {on,off}`(新設)でDD-15/16の両補正(キャスター引きずり・横ズレ)をまとめて無効化
- `--rally-gyro-tire-calibration {on,off}`(新設)でDD-17(ジャイロ倍率補正・タイヤ径較正)を無効化
- モーター出力・バッテリー電圧/電流を`RALLY_PWR`タグでログ出力するよう各旋回・直進クラスに追加(`robot_program/power_log.py`)。`wireless_device/et_rally_lqi/parse_power_log.py`でログを要約できる
- ゴール位置はDD-19較正の間、本番用のGOAL_SHIFT_CM(ガレージ位置への補正)を外し、「14,15のグレーポイントの中点からマイナスX方向へ14.6cm」の基準点そのものを使う(2026-10-10、`wireless_device/et_rally_planner/config.py`)
- 位置ズレの記録方向: ゴールに正対した状態を基準に、+X=スタート方向へ戻る側／-X=ゴールを行き過ぎる側、+Y=北／-Y=南、角度は真上(+Y方向)を0度として左右

### ベースライン実行コマンド例

```
pypy3 alpha.py right --mission rally-drive \
  --rally-plan-path robot_program/tests/plan_seed9392783.json \
  --rally-hint1 "25,35" --rally-hint2-gate-info "53,54/12,22" \
  --rally-turn-method spin_pid --rally-compensation off --rally-gyro-tire-calibration off
```

## ベースライン計測結果(2026-10-10、補正なし5回、plan_seed9392783.json・右コース)

旋回PID=(0.2, 0.00075, 0.03)(execute_strategy.pyのturn_pid既定値、production/spin_encoderのフェーズ2仕上げと同じ値)、直進PID=(1.1, 0.1, 0.03)(sample.pyのRunByGyroと同じ値)、`--rally-compensation off --rally-gyro-tire-calibration off`で5回完走。ゴール地点との誤差（ゴールに正対した基準、+X=スタート方向/-X=ゴール行き過ぎ、+Y=北/-Y=南、角度は真上0度）:

| 回 | -Y(cm) | -X(cm) | 角度(左へ) | 合成距離(cm) |
|---|---|---|---|---|
| 1 | 8.2 | 4.3 | 12° | 9.3 |
| 2 | 12.0 | 8.6 | 12° | 14.8 |
| 3 | 11.8 | 4.3 | 10° | 12.6 |
| 4 | 9.8 | 3.1 | 10° | 10.3 |
| 5 | 11.1 | 4.4 | 8° | 11.9 |
| **平均** | **10.6** | **5.0** | **10.4°** | **11.8** |

**重要な所見**: 5回とも誤差の向きが同じ(-Y・-X・左回転)。バラツキではなく系統的なバイアスであることを示している。

**次に確認すること(仮説)**: この系統的なバイアスは、無効化しているDD-15(キャスター引きずり補正)・DD-16(横ズレ補正)が本来打ち消しているはずの、既知の実機の癖(旋回のたびに後方・横方向へ一定方向にズレる現象)が、3周・旋回22回という規模で積み上がって現れたものではないか。

**進め方の修正(2026-10-10)**: いきなり`--rally-compensation on`で比較すると、「個々の指示(直進・旋回)自体のズレ」と「補正の有無」という2つの要因が混ざってしまう。5-2(旧モデルの個別検証)と同じ順序に合わせ、先に直進単体(`plan_straight_200cm.json`)・旋回単体(`plan_spin_alternating30.json`)の精度を同条件(spin_pid・補正オフ・較正オフ)で実測し、フル周回の11.8cmのうち「個々のズレの積み上げで説明できる分」と「それでは説明できない残差」を切り分けてから、compensationの効果を見る。

## 重要な設計判断の一覧(DD-19〜)

| ID | 課題 | 検討した候補 | 採用案 | 理由(記録の有無を明記) | 現在の状態 | 根拠 |
|---|---|---|---|---|---|---|
| DD-19 | 補正なしベースラインのゴール地点誤差を実測する | spin_pid・補正オフ・較正オフで3周フル経路を実行 | 上記「ベースライン計測結果」参照 | 記録あり(2026-10-10、実機5回、上表) | 完了。平均11.8cm・系統的に-Y-X左回転方向へズレることを確認 | 本ファイル該当セクション、`run_logs/run_2026101009{46,48}*.log` |

