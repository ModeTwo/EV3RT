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

- `--mission rally-drive --rally-plan-path <ファイル>` で、PC通信なしに経路ファイル単体のテストができる(DD-12で既存)。直進専用: `robot_program/tests/plan_straight_100cm.json`/`200cm.json`、旋回専用: `plan_spin_alternating30.json`
- `--rally-turn-method {spin_pid,spin_encoder,one_shot,lqi}`(新設)で旋回方式を切替。`spin_pid`がDD-13以前の無補正PIDベースライン
- `--rally-compensation {on,off}`(新設)でDD-15/16の両補正をまとめて無効化
- モーター出力・バッテリー電圧/電流を`RALLY_PWR`タグでログ出力するよう各旋回・直進クラスに追加(`robot_program/power_log.py`)。5-5要素技術③(バッテリー出力に応じた調整)の検証材料として使う

### ベースライン実行コマンド例

```
python alpha.py left --mission rally-drive \
  --rally-plan-path tests/plan_spin_alternating30.json \
  --rally-turn-method spin_pid --rally-compensation off
```

## 重要な設計判断の一覧(DD-19〜)

| ID | 課題 | 検討した候補 | 採用案 | 理由(記録の有無を明記) | 現在の状態 | 根拠 |
|---|---|---|---|---|---|---|
| DD-19 | （次の実測から記入） | | | | | |

