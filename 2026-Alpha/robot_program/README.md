AT・TO単体実行: `pypy3 alpha.py right --mission at` / `--mission to`。配置と操作は[単体実行手順](../AT_TO_STANDALONE_v1.md)。

TOは工程分割せず元のツリーを直接接続します。[最新版の説明](../AT_TO_SOURCE_STYLE_v2.md)。

# スタート距離補正 v4

最新の開始距離設定は [START_LAP_CALIBRATION_v4.md](START_LAP_CALIBRATION_v4.md) を参照。

> 2026-09-10 [実行ログの自動保存](RUN_LOGGING_v1.md): 通常のalpha.py実行でrun_logsへ日時付きログを保存。Ctrl+C時も終了処理し、.gitignoreでGit対象から除外する。

AT・TO担当者の編集箇所は [AT_TO_SOURCE_STYLE_v1.md](../AT_TO_SOURCE_STYLE_v1.md) を参照してください。

> 2026-09-10 スタート～LAPの追従調整: [調整ガイド v3](START_LAP_TUNING_v3.md)。P=1.8/I=0/D=0.03、曲率による基本旋回出力と最大8度の推定横ずれ補正を追加。速度33、距離表は維持。以下のv2の「ゲイン不変」は過去版の記録。新規path_tracking.pyも配備する。

> 2026-09-10 スタート～LAPの構造整理: [初学者向け編集ガイド v2](START_LAP_GUIDE_v2.md)。featureは `RunByGyro(target=profile.heading_at, ...)` を一つ返す。速度・PIDはconfig.py、旧方式は別ファイル。以下の距離方位クラスv1説明より本ガイドを優先する。

> 2026-09-09 最新相撲仕様：[方位角・単体試走・復元手順](SUMO_BEARING_v1.md)。相撲のみ上0度・時計回りの方位角へ移行。下記の相撲旧ジャイロ絶対角度・相対20度説明より同資料を優先します。

> 2026-09-08 最新・相撲500mm走行：黒ボトルを検出して方位を確定した位置から、キャッチ・押し出し込みで `capture_and_push_distance_mm=500.0` だけ走り、停止後に設定距離を後退する。画像消失・死角に依存せず毎制御周期で距離を確認する。旧死角150mmと押し出し距離は追加しない。500mmは開始位置からの350mmやカメラ用後退を含まない。以下の旧追加直進仕様より本項を優先する。

> 2026-09-08 最新相撲開始動作：No.15初期位置から`start_straight_distance_mm=350.0`だけジャイロ直進し、停止して土俵側へ相対90度旋回する。旧黒→白0.5秒判定と追加75mmは使用しない。以下の旧開始条件説明より本項を優先する。

> 2026-09-08 最新ガレージ復帰：黒検知・停止→`garage_line_entry_distance_mm=30.0`だけ現在方位で追加直進→停止→絶対180度旋回→停止→既存TraceLineで100mm試走→停止。追加30mmは進入量不足を調整する暫定値で、0にすれば追加距離なし。試走距離は`line_rejoin_trace_distance_mm`、出力は`line_rejoin_trace_power=50`。黒未検知時は復帰動作へ進まない。Left／Rightは既存部品のcourse補正を使用する。

> 2026-09-08 相撲の暫定運用：ボトル検出確定後は、死角進入時の角度ずれ・見失いを捕捉失敗とせず、固定方位の規定距離走行へ移る。下記の旧「正面±8度」「角度ずれで停止」の記載より本項を優先する。`camera_blind_max_theta_deg`は未使用。現設定では死角直進150mm＋押し出し300mmの後、150mm直線後退する。捕捉・押し出しは実測判定ではなく仮定。初回検出待ちと、死角／見失い条件までの接近は従来どおり。

> 2026-09-08：AT・TO・相撲の追加工程タイムアウトを撤去。[現行仕様](../TASK_TIMEOUTS_REMOVED.md)。PC戦略受信の制限とCtrl+C終了処理は維持。

> 2026-09-06 復元：①〜④の調整前へ復元済み。相撲・ミッション選択は維持。現状は [復元記録](../RESTORED_BEFORE_RIGHT_TRIAL.md) を優先してください。

> 2026-09-05 v2更新：最新は[RE・AT・TO連続走行版](../INTEGRATION_RUN_v2.md)です。標準alpha.pyはHint2取得後に停止します。周期はtiming.pyのCONTROL_INTERVAL_SECへ統一しました。以下の旧版説明よりv2文書を優先してください。
> 2026-09-05更新：現在は統合基盤・移植準備版です。最新状況は[統合基盤v1](../INTEGRATION_FOUNDATION_v1.md)を参照してください。未実装工程があるため通常起動は走行前に終了コード2で停止します。下記の「仮処理は成功」「スタートのみ実装済み」等の過去説明より、この案内とv1文書を優先してください。
# 2026-Alpha 走行体プログラム

## 1. この構成の目的

2026-Alphaは、複数人が同時に担当機能を開発しても、同じファイルで競合しにくい構成にしています。

- `alpha.py`には、競技開始に必要な基本処理とBehavior Tree全体の入口を残します。
- 競技工程の順序は`tree_builder.py`と`phases/`で管理します。
- 各担当者は原則として、自分が担当する`features/`内のファイルだけを変更します。
- 複数機能で再利用する走行・停止・センサー判定は`behaviours/`へ配置します。
- 工程間の値は`RaceContext`を介して渡します。
- PC側のパスワード入力と走行指示SEQ生成は`wireless_device/`で管理します。

現時点では、`sample.py`から移植したスタート～LAPゲートの`lap2`・`lap3`だけが実装済みです。それ以外の機能ファイルには、仮の`PendingFeature`が配置されています。

## 2. 実行方法

Leftコース：

```text
python alpha.py left
```

Rightコース：

```text
python alpha.py right
```

ログファイルを指定する場合：

```text
python alpha.py left --logfile logs/run.log
```

`--logfile`へ出力したログはテキストとして保存されるため、後から生成AIやスクリプトへ渡して解析できます。

### センサー値を常時表示する

実機調整時は、競技プログラムとは別の`sensor_monitor.py`を実行します。コマンド実行で計測を開始し、任意のタイミングで`Ctrl+C`を押すと終了します。計測中は、タッチ、カラーHSVと判定色、距離、ジャイロ、各モーターエンコーダー、IMU静止判定が同じ画面上で更新され続けます。モーター駆動やセンサー値のリセットは行いません。

```text
python sensor_monitor.py
```

標準の表示更新間隔は0.1秒です。変更する場合：

```text
python sensor_monitor.py --interval 0.05
```

画面表示と同じ値をCSVにも保存する場合：

```text
python sensor_monitor.py --csv logs/sensors.csv
```

ANSI画面クリアに対応していない端末では、次の指定で各サンプルを追記表示できます。

```text
python sensor_monitor.py --no-clear
```

距離センサー値はmmとして扱うため、ET相撲側の換算係数は`1.0`です。モニターの生値とmm表示が一致することを、既知距離でも確認してください。

### 力士ボトルの黒テープをカメラで確認する

`sumo_bottle_camera_monitor.py`は、走行体を動かさず、USBカメラのプレビュー上で力士ボトルの黒テープと距離センサー値を同時確認する単体プログラムです。ETRoboにはHubとFポートの距離センサーだけを登録し、モーターとアームは登録も操作もしません。

```text
python sumo_bottle_camera_monitor.py
```

画面左側が判定枠付きのカメラ画像、右側が黒領域マスクです。黒テープ候補を黄色、異なる3フレームで連続確認した対象を緑で囲み、`DETECTED`と表示します。左側には距離センサーの生値、mm換算値、競技プログラムと同じ50～800mmの有効範囲判定、最新値の経過時間も表示します。`Q`または`Esc`、端末の`Ctrl+C`で終了します。

照明に合わせて黒判定を調整する例：

```text
python sumo_bottle_camera_monitor.py --black-max-value 80 --min-area 100
```

力士ボトル専用の判定は`robot_program/vision/sumo_black_bottle.py`に配置し、Bottle Delivery用の赤・青・黄判定を持つ`py_etrobo_util/video.py`とは設定・責務を分離しています。この確認プログラムで閾値を変更しても3色ボトル認識には影響しません。

## 3. 機能の有効・無効とETラリー周回数

設定は`robot_program/config.py`の`RaceConfig`で変更します。

```python
@dataclass(frozen=True)
class RaceConfig:
    mission_mode: str = "configured"
    lapgate: bool = True
    enable_bottle_delivery: bool = True
    enable_et_rally: bool = True
    et_rally_laps: int = 3
    et_rally_strategy_source: str = "file"
    et_rally_plan_path: Optional[str] = None
    enable_et_sumo: bool = True
    enable_finish: bool = True
```

要求上の設定名との対応は次のとおりです。

| 要求上の設定 | `RaceConfig`のフィールド | 設定値 |
|---|---|---|
| スタート～LAPゲート | `lapgate` | `True` / `False` |
| `ENABLE_BOTTLE_DELIVERY` | `enable_bottle_delivery` | `True` / `False` |
| `ENABLE_ET_RALLY` | `enable_et_rally` | `True` / `False` |
| `ET_RALLY_LAPS` | `et_rally_laps` | `0` / `1` / `2` / `3` |
| ETラリーSEQ取得元 | `et_rally_strategy_source` | `"received"` / `"file"` |
| 固定planのパス | `et_rally_plan_path` | `None` / JSONファイルのパス |
| `ENABLE_ET_SUMO` | `enable_et_sumo` | `True` / `False` |
| `ENABLE_FINISH` | `enable_finish` | `True` / `False` |

`mission_mode="configured"`では、スタート～LAPゲートを含むすべての工程が上記フラグに従います。すべて`False`なら、キャリブレーションとタッチスタートの後に終了し、LAP走行は開始しません。

`enable_et_rally=False`の場合、`et_rally_laps`の値にかかわらずETラリー周回は実行されません。`et_rally_laps=0`の場合もETラリー周回は実行されません。

現在は無線通信デバイス連携を一時的に横へよけているため、標準設定を`et_rally_strategy_source="file"`としています。`robot_program/tests/plan_seed9392783.json`の固定planを使用し、復号キー入力、PC接続、受信待ちは行いません。

無線通信デバイス連携を再開する場合は`et_rally_strategy_source="received"`へ戻します。PCから受信した`context.strategy`を実行直前にBehavior Treeへ展開し、PCが選択した1～3周分の全SEQを一度だけ実行します。

従来の固定planを使う場合は`et_rally_strategy_source="file"`にします。この場合はPC接続と受信待ちを行いません。`et_rally_plan_path=None`なら`robot_program/tests/plan_seed9392783.json`を使用します。別ファイルを指定する相対パスは`2026-Alpha`直下が基準です。固定planの`steps`全体を一度実行するため、`et_rally_laps`によるJSON内容の切出しは行いません。

設定ファイルを変更せずに工程を単体実行する場合は、`alpha.py`の`--mission`を使用します。

| 工程 | コマンド例 |
|---|---|
| 設定フラグどおり | `python alpha.py left --mission configured` |
| LAP | `python alpha.py left --mission lap` |
| Bottle Delivery | `python alpha.py left --mission bottle` |
| Bottle Delivery後半 | `python alpha.py left --mission bottle-final --bottle-color red` |
| ETラリー準備＋周回 | `python alpha.py left --mission rally` |
| ET相撲 | `python alpha.py left --mission sumo` |
| FINISH | `python alpha.py left --mission finish` |
| 全工程 | `python alpha.py left --mission full` |

`--mission`を省略した場合も`configured`です。Rightコースは`left`を`right`へ置き換えます。どの工程でも共通のキャリブレーションとタッチスタートは先に実行されます。単体工程は上流工程の動作を実行しないため、走行体をその工程の開始位置・開始方位・アーム状態へ手動で置いてから開始してください。

`hint2`と`hint2-return`はRE→AT→TO接続試験を維持する専用モードであり、工程フラグを参照しません。未実装の`PendingFeature`は警告を表示して何もせず成功扱いとなり、次の工程へ進みます。詳細は[工程単体実行ガイド](../MISSION_SELECTION_v6.md)を参照してください。

## 4. プログラムが実行される順序

```text
alpha.py
└─ 実機・カメラ・ログを初期化
   └─ build_behaviour_tree()
      ├─ calibration
      │  ├─ ArmUpDownFull: arm up
      │  ├─ ArmUpDownFull: arm down
      │  └─ ResetDevice
      ├─ start
      │  └─ IsTouchOn
      ├─ robot_program/tree_builder.py
      │  └─ build_mission_children(context, config)
      │     ├─ lap_gate
      │     ├─ bottle_and_rally_preparation
      │     ├─ et_rally
      │     ├─ et_sumo
      │     └─ finish
      └─ TheEnd
```

実行中は、ETRoboが`TraverseBehaviourTree`を一定周期で呼び出します。

```text
ETRobo.dispatch()
└─ TraverseBehaviourTree.__call__()
   ├─ 初回: 実機参照をruntimeへ設定
   └─ 2回目以降: tree.tick_once()
      └─ 現在実行中のBehavior.update()
```

カメラ処理は`VideoThread`でBehavior Treeとは別に実行されます。

## 5. `alpha.py`に残すもの

`alpha.py`は次の基本処理を持ちます。

- 実行引数の読み取り
- Left／Rightコースの設定
- モーター・センサーのポート設定
- カメラスレッドの開始と終了
- Behavior Treeの基本構造
- アーム初期化、デバイス初期化
- タッチセンサーによる競技開始
- Behavior Treeの周期実行
- 実行終了処理

競技固有の走行処理は`alpha.py`へ追加せず、`features/`または`behaviours/`へ配置します。

`ArmDirection`と`ArmUpDownFull`はキャリブレーションでのみ使用するため、共通`behaviours/`へ分割せず`alpha.py`に配置しています。`ResetDevice`は単体テスト可能な共通Behaviorとして`behaviours/device_control.py`に配置し、`alpha.py`のキャリブレーションから呼び出します。

`ResetDevice`自身がデバイス値をグローバル変数として保持する必要はありません。`runtime`に設定済みの同一デバイス参照を使い、モーターのエンコーダー値とジャイロ角度は各デバイス内部の`reset_count()`／`reset()`でゼロ化します。走行途中で実行すると`Plotter`の累積走行値と基準がずれるため、競技開始前のキャリブレーションでだけ使用してください。

`TraceLineCam`、`IsJunction`、`CatchBottle`は現在の走行戦略と合致しないため、`alpha.py`には配置していません。

## 6. ファイル階層と責務

```text
2026-Alpha/
├─ alpha.py                         # 実行入口と競技開始の基本処理
├─ sample.py                        # 移植元の参照用コード
├─ robot_program/
│  ├─ config.py                     # 工程の有効・無効、ETラリー周回数
│  ├─ context.py                    # 工程間で共有する値
│  ├─ decryption_key.py             # 起動時の4桁復号キー入力・確認
│  ├─ runtime.py                    # 実機、センサー、Plotter、Videoの参照
│  ├─ tree_builder.py               # 有効な競技工程を実行順に並べる
│  ├─ placeholder.py                # 未実装機能用の仮Behavior
│  ├─ types.py                      # 共通の列挙型
│  ├─ phases/                       # 工程内の機能実行順
│  ├─ features/                     # 担当機能ごとのサブツリー
│  ├─ behaviours/                   # 再利用可能な単一動作・条件
│  ├─ services/                     # タイマー等の共通サービス
│  └─ tests/                        # 単体テスト
└─ wireless_device/
   ├─ application.py                # PC側処理の統合
   └─ strategy_planner.py           # 走行指示SEQ生成
```

### `phases/`

工程内の実行順だけを管理します。モーター制御や画像処理は実装しません。

| ファイル | 工程 |
|---|---|
| `lap_gate.py` | スタート～LAPゲート |
| `bottle_and_rally_preparation.py` | Bottle DeliveryとETラリー準備 |
| `et_rally.py` | 走行指示SEQ受信、補正、周回走行 |
| `et_sumo.py` | ET相撲 |
| `finish.py` | ガレージ走行・停止 |

### `features/`

各担当者が主に変更する場所です。一つの機能は一つのファイル内で完結させます。

新しいFeatureは`features/feature_template.py`を複製して作成します。すべてのFeatureは`features/bt_imports.py`から次の基本部品をあらかじめ読み込みます。

- `Behaviour`
- `Sequence`
- `Parallel`
- `Selector`
- `ParallelPolicy`
- `Status`
- `Success`
- `Failure`
- `Running`
- `time`
- `runtime`
- `HeadingType`
- `TraceSide`
- `TargetInterested`
- `Color`
- `BottleColor`

標準importは次の1行にまとめられています。Feature内でSequenceやParallelが後から必要になっても、追加importは不要です。

```python
from .bt_imports import Behaviour, BottleColor, Color, Failure, HeadingType, Parallel, ParallelPolicy, Running, Selector, Sequence, Status, Success, TargetInterested, TraceSide, runtime, time
```

#### 必要な場合だけ追加するimport

Feature固有の処理に応じ、次の基準で個別importを追加します。

PIDでモーター出力や操舵量を計算する場合：

```python
from simple_pid import PID
```

PID出力を最小値・最大値の範囲へ制限する場合：

```python
from py_etrobo_util import SymmetricClamper
```

カラーセンサーのHSV値を色へ分類する場合：

```python
from py_etrobo_util import ColorClassifier
```

センサー値や操舵値を平滑化する場合：

```python
from py_etrobo_util import LowPassFilter
```

三角関数、角度変換、座標計算をFeature内で行う場合：

```python
import math
```

Feature独自の状態を列挙型で管理する場合：

```python
from enum import Enum, IntEnum, auto
```

走行体側でHint文字列の解釈が本当に必要な場合：

```python
from py_etrobo_util import Hint, HintType
```

現在の責務分担では、Hint2の復号は走行体側通信スレッド、走行指示SEQ生成はPC側が担当します。Hint読取Featureでは暗号文字列を`context.hint2`へ保存するだけで、復号処理を追加しないでください。

既存の共通Behaviorを利用する場合は、使用するものだけを明示的にimportします。

```python
from ..behaviours.conditions import IsColorDetected, IsDistanceEarned
from ..behaviours.gyro_drive import RunByGyro, SpinAround
from ..behaviours.line_trace import TraceLine
from ..behaviours.motor_control import RunAsInstructed, StopNow
```

`ETRobo`、`Hub`、`Motor`、各Sensor型、`Video`、`Plotter`はFeatureへ直接importしません。実機へのアクセスには標準import済みの`runtime`を使用します。

例：

| 機能 | ファイル |
|---|---|
| スタート～LAPゲート | `start_to_lap_gate.py` |
| ボトル取得 | `catch_bottle.py` |
| Hint 1位置への移動 | `move_to_hint1.py` |
| Hintカード読取 | `read_hint.py` |
| ボトル配置 | `drop_bottle.py` |
| 走行指示SEQ受信 | `receive_strategy.py` |
| 走行指示SEQ実行 | `execute_strategy.py` |
| ET相撲ボトル探索 | `locate_sumo_bottle.py` |
| ガレージ走行 | `drive_to_garage.py` |

未実装ファイルでは、次の`PendingFeature`を実際のサブツリーへ置き換えます。

```python
root.add_children([PendingFeature(name="feature_name_pending")])
```

### `behaviours/`

複数の機能から利用できる単一動作・条件を配置します。

| ファイル | 主なBehavior |
|---|---|
| `line_trace.py` | `TraceLine` |
| `gyro_drive.py` | `RunByGyro`、`SpinAround` |
| `motor_control.py` | `StopNow`、`RunAsInstructed` |
| `device_control.py` | `ResetDevice` |
| `conditions.py` | `IsDistanceEarned`、`IsColorDetected`、`IsColorTransitionDetected`、`IsTimePassed` |
| `bottle.py` | `IsBottleInsight`、`HasCaughtBottle`、`SelectBottleDropZone`、配置先判定・完了記録 |
| `hint_reader.py` | `ReadHintCard` |

`RunAsInstructed`は従来から使用している名称を維持しています。

### Bottle Delivery No.7～9

ボトルのキャッチは既存の`catch_bottle.py`を使用します。新たに実装したBottle Delivery後半は、Hintカード2の読取と`move_after_hint2.py`による既存の移動（相対-25度旋回、600mmライントレース）が完了した後から始まります。

```text
move_after_hint2.py
└─ Hint2後の既存移動を完了
   └─ select_drop_zone.py
      ├─ 最初の青ライン（黄ドロップゾーン前）を検知
      ├─ 黄ボトル: その位置を選択
      ├─ 青ボトル: 青ラインを1本通過して選択
      └─ 赤ボトル／不正な認識値: 青ラインを2本通過して最上段を選択
         └─ drop_bottle.py
            ├─ 選択した青ラインの中央まで移動
            ├─ ドロップゾーン側へ90度旋回
            ├─ 規定距離を前進して配置
            ├─ 同じ距離を後退
            └─ ライン進行方向へ90度戻す
               └─ move_to_rally_ready.py
                  ├─ 黄・青からは赤ドロップゾーン前までライントレース
                  ├─ 最上段の青ライン中央で停止
                  └─ コース内側へ90度旋回してETラリーへ引き渡す
```

Left／Rightの旋回方向は`SpinAround`内の`runtime.course`で鏡像化されます。Feature側では同じ相対角度`+90度`を指定し、両コースで内側を向く設計です。青ラインを通過する前に固定距離を進むため、同じ青ラインを次のラインとして再検知しません。

実機で調整する値は`IntegrationSettings`へ集約しています。初期値は、ライントレース目標V値75、出力50、青ライン全幅120mm、手前端から中央60mm、ドロップゾーン進入距離250mm、進入出力50、配置側旋回-90度、ラリー内向き旋回+90度です。コード内の経路順序を変更せず、次のフィールドだけで調整できます。

- `delivery_trace_target_v`
- `delivery_trace_power`
- `delivery_marker_full_width_mm`
- `delivery_marker_half_width_mm`
- `delivery_drop_distance_mm`
- `delivery_drive_power`
- `delivery_drop_turn_deg`
- `delivery_inward_turn_deg`

配置先、配置完了、ETラリー開始姿勢の成立はそれぞれ`RaceContext.selected_drop_zone`、`bottle_delivered`、`rally_ready`へ記録します。ボトル色が赤・青・黄のいずれでもない場合は、事前に決めた既定動作として赤ドロップゾーンを選択します。

#### 最後の直線部分だけを試す

`bottle-final`は、ボトル認識、キャッチ、Hintカード読取、Hint2後の旋回・600mm移動を実行しません。次の3サブツリーだけを実行します。

```text
bottle_delivery_final
├─ select_drop_zone
├─ drop_bottle
└─ move_to_rally_ready
```

走行体は、`move_after_hint2.py`が完了した位置へ、ドロップゾーン列の進行方向を向けて設置してください。アームが下端にあり、ボトルを保持済みであることを開始条件とします。共通キャリブレーションはタッチ待ちより前に実行されるため、実物のボトルを使う場合は、アームの上下キャリブレーションが終わってタッチ待ちになった後にボトルをセットしてください。

コマンドで色まで指定する場合：

```console
python alpha.py left --mission bottle-final --bottle-color red
python alpha.py left --mission bottle-final --bottle-color blue
python alpha.py left --mission bottle-final --bottle-color yellow
```

色を省略した場合は、デバイス初期化前に端末へ入力します。

```console
python alpha.py left --mission bottle-final
Bottle color (red/blue/yellow): blue
```

Rightコースでは先頭の`left`を`right`へ変更します。このモードは画像認識結果の代わりに入力色を`RaceContext.bottle_color`へ格納するため、カメラとQRデコーダーを起動しません。`--bottle-color`は`bottle-final`以外では指定できません。

### ET相撲 No.15～18

ET相撲は距離センサーを使用せず、カメラで力士ボトルの黒テープを捕捉します。ETラリー終了位置は青円上を想定するため、開始直後はライントレースせず、ジャイロで方位を維持して直進します。黒ラインを確認して白地へ抜けた後、調整可能なクリアランス距離だけ追加直進して土俵方向へ90度旋回し、さらに50mm後退してカメラ視野を広げます。

ET相撲でも、旋回は`behaviours/gyro_drive.py`の`SpinAround`、方位維持走行は`RunByGyro`、距離終了判定は`IsDistanceEarned`、停止は`StopNow`を利用します。Feature内にモーター出力やPID旋回を重複実装しません。

Feature内に残すのは、黒テープの連続フレーム確認、停止中画像からのボトル絶対方位算出、カメラ死角へ入った後の捕捉距離設定など、ET相撲に固有の処理だけです。P1・P2の2地点推定、座標経路計画、`sumo_geometry.py`は使用しません。旧距離センサー方式のファイルは比較・復帰用に残していますが、ET相撲の実行木からは外しています。

```text
move_to_sumo_start.py
├─ 青円上のETラリー終了位置からジャイロ直進
├─ 黒を確認した後、白を0.5秒連続確認して黒ライン終端と判定
├─ 白地上を調整可能なクリアランス距離だけ追加直進
├─ Leftでは左、Rightでは右へ90度旋回して土俵方向を向く
├─ 両コースで50mm後退して制動停止
└─ capture_sumo_bottle_camera.py
   ├─ 静止状態で黒テープを異なる3フレーム連続確認
   ├─ 停止中の画像角度とジャイロ方位からボトル絶対方位を固定
   ├─ PWM75・左右輪50～100を維持し、固定方位へジャイロ操舵して前進
   ├─ 正面±8度以内で黒テープが死角へ入った場合だけ捕捉工程を継続
   └─ 保存方位を維持して調整可能な距離を直進し、下端アーム内へ捕捉
      └─ move_to_sumo_exit.py
         ├─ ボトル捕捉完了位置から300mm直進して押し出す
         ├─ 旋回せず直線後退してアームから離脱
         ├─ 現在方位からガレージ側へ20度旋回して停止
         ├─ 旋回後の絶対方位を維持して直進し、生V値で復帰用黒ラインを検知
         ├─ 300mm以内に検知できなければ停止して失敗終了
         └─ 検知時だけ短距離ライントレースで姿勢を整えて停止
```

調整値は`RaceConfig.sumo`の`SumoSettings`へ集約しています。ET相撲開始位置の黒線退出は、共通色分類のWHITEではなくカラーセンサーの生V値を使用します。Vが`line_black_max_value`以下になった後、`line_white_min_value`以上の状態が`line_exit_white_duration_sec`継続した場合に明るい路面へ抜けたと判定します。判定中はHSVと段階を0.25秒間隔でログへ出します。白地確認後は`post_line_clearance_distance_mm`だけ追加直進してから90度旋回します。後退距離は`camera_retreat_distance_mm`（初期値50mm）、後退出力は`camera_retreat_power`（初期値60）です。

カメラは`begin_sumo_bottle_read()`から開始し、黒だけに追跡対象を固定します。Bottle Deliveryは`begin_bottle_read()`で色固定を解除して赤・青・黄を判定するため、ET相撲の黒判定が残りません。同一画像を複数回数えず、異なる3フレームで確定してから前進します。

ET相撲の駆動出力設定はすべて50以上です。相撲単体の`--mission sumo`でも、カメラ撮影・画像処理・プレビュー用スレッドを起動します。黒テープを3秒以内に確定できない場合、または接近が8秒を超えた場合はモーターを停止し、捕捉と後続運搬を省略します。

No.16・17では90度旋回後の探索旋回を行いません。黒テープを停止中に3フレーム確認した時点で、画像角度、ジャイロ方位、Left／Rightのcourse符号からボトル絶対方位を一度だけ算出します。接近中は遅延した画像角度を目標として追い続けず、この固定方位との差を20ms周期のジャイロ値で補正します。基準PWM75、左右輪50～100、最大左右差±25は変更しておらず、速度と最低出力を落としていません。

ログには画像角度、画像下端位置、面積に加え、固定目標方位、現在の方位誤差、左右PWMを0.25秒間隔で出します。黒テープが画面下端へ到達しても、最後の画像角度が`camera_blind_max_theta_deg`（初期値8度）を超えている場合は横を通過していると判断し、捕捉成功にせず停止・省略します。正面条件を満たした場合だけ、固定したボトル絶対方位を既存の`RunByGyro`へ渡して死角区間を直進します。

No.18の押し出し完了条件には黒ラインを使用しません。捕捉完了位置から`push_out_distance_mm`（初期値300mm）だけ方位維持直進して押し出します。その後は旋回せず、`release_reverse_distance_mm`（初期値150mm）だけ真っすぐ後退してアームから離脱します。押し出し側黒ラインを取り逃がしても直進を継続し続けない構成です。

離脱後は`garage_return_turn_deg`（初期値20度）だけガレージ側へその場旋回し、完全停止してから`garage_return_drive_power`（初期値60）で直進します。旋回は既存の`SpinAround`を使用し、Left／Rightは`course`により鏡像化されます。直進は既存の`RunByGyro`が開始時のジャイロ角を絶対目標として保持するため、緩いカーブではなく、ガレージ方向へ向けた方位を維持したまま黒ラインを探します。復帰用黒ラインは彩度を条件にせず、生のV値が`garage_line_black_max_value`（初期値45）以下の状態を`garage_line_confirm_samples`（初期値2回）連続確認して確定します。検知ログにはHSV、連続回数、検知時の絶対方位を出します。

黒ライン検知後は`line_rejoin_trace_distance_mm`（初期値100mm）だけ既存の`TraceLine`で走り、FINISH工程へ渡せる姿勢に整えて停止します。`garage_line_search_max_distance_mm`（初期値300mm）以内に黒ラインを検知できなければ、モーター停止後に`garage_line_not_found`としてミッションを失敗終了し、不明な位置からFINISHへ進みません。300mmの押し出し距離、ガレージ側への旋回角度20度、150mmの離脱距離、300mmの探索上限、100mmの安定化距離はレプリカコースで調整してください。

Contextには工程ごとに`bottle_pushed_out`、`bottle_released`、`garage_line_found`、`line_trace_ready`を記録します。正常完了時は`transport_completed=True`、`bottle_held_at_exit=False`となります。ログでは距離押し出し、直線後退による離脱、ガレージ側黒ライン復帰を個別に確認できます。

ET相撲開始時にはアームが下端にあることを上流工程の事後条件とし、ET相撲内ではアームモーターを動かしません。捕捉成功を直接検知するセンサーは使わず、黒テープが死角へ入った後に`camera_blind_capture_distance_mm`へ到達したことを捕捉成立として扱います。50mm後退量、黒テープ閾値、死角後150mmの初期値、出口運搬経路は実機で確認・調整してください。

`alpha.py`の`ArmUpDownFull`は、エンコーダー回転量の変化が5度未満の状態を5周期連続で検知すると、機械端へ到達したと判断してPWMを0にし、ブレーキを有効にします。実行周期が20msの場合、終端到達後の判定時間は約0.1秒です。

## 7. 工程間で値を渡す方法

他工程へ渡す値は`RaceContext`へ保存します。担当機能同士を直接importして値を参照しないでください。

現在の共有項目：

| フィールド | 内容 |
|---|---|
| `bottle_color` | 認識したボトル色 |
| `selected_drop_zone` | Bottle Deliveryで確定した配置先（赤／青／黄） |
| `bottle_delivered` | 配置用の前進・後退・復帰旋回が完了したか |
| `rally_ready` | 最上段青ライン中央で内向きになったか |
| `decryption_key` | alpha.py起動時に入力・確認した4桁復号キー |
| `hint1` | Hintカード1の読取結果 |
| `hint2` | Hintカード2から読み取った暗号文字列 |
| `hint2_gate_info` | 通信スレッドが復号したHintカード2のゲート情報 |
| `strategy` | 走行指示SEQ |
| `selected_rally_laps` | PCが選択し、受信SEQに含めた周回数 |
| `rally_lap` | 現在のETラリー周回 |
| `timer` | 競技時間計測 |

値を書き込む例：

```python
class DetectBottleColor(Behaviour):
    def __init__(self, name, context):
        super().__init__(name)
        self.context = context

    def update(self):
        # 認識結果を後続工程へ渡す。
        self.context.bottle_color = "red"
        return Status.SUCCESS
```

値を読み取る例：

```python
class SelectDropZone(Behaviour):
    def __init__(self, name, context):
        super().__init__(name)
        self.context = context

    def update(self):
        bottle_color = self.context.bottle_color
        return Status.SUCCESS
```

共有する値を追加する場合だけ`context.py`を変更します。その機能内だけで使用するPID状態、開始距離、連続検知回数などはBehaviorインスタンスのメンバーに保持します。

## 8. `lap2`・`lap3`の移植例

`sample.py`の`lap2`・`lap3`は`features/start_to_lap_gate.py`へ移植しています。

| 元の内容 | 現在の配置 | 役割 |
|---|---|---|
| `lap2` | `features/start_to_lap_gate.py` | ライントレースと青ライン検知 |
| `lap3` | `features/start_to_lap_gate.py` | 絶対角度3度を維持して370mm走行 |
| `TraceLine` | `behaviours/line_trace.py` | ライントレース制御 |
| `RunByGyro` | `behaviours/gyro_drive.py` | ジャイロ直進制御 |
| `IsColorDetected` | `behaviours/conditions.py` | 青ライン検知 |
| `IsDistanceEarned` | `behaviours/conditions.py` | 走行距離判定 |

## 9. 担当者の編集ルール

1. 原則として、自分の担当する`features/`のファイルだけを変更します。
2. 既存の再利用Behaviorが使える場合は`behaviours/`からimportします。
3. 他工程へ渡す値は`RaceContext`を使います。
4. 定数や調整値は担当機能のファイル内、または共通化が必要な場合は`RaceConfig`へ置きます。
5. 工程の追加・削除・実行順変更が必要な場合は、統合担当者が`phases/`または`tree_builder.py`を変更します。
6. Pythonの識別子、実行時文字列、docstringは英語を使用します。日本語の説明はコメントへ記載します。
7. 新規Featureは`feature_template.py`を複製し、`bt_imports.py`からの標準importを削除しないでください。

## 10. 単体テスト

リポジトリ直下から次のコマンドで実行します。

```text
.venv\Scripts\python.exe -m unittest discover -s 2026-Alpha\robot_program\tests -t 2026-Alpha -v
```

実機を使わない単体テストでは、`runtime`へFakeMotor、FakeSensor、FakeVideoなどを設定してBehavior単体を確認します。

実機調整が必要な値は、Behaviorのコンストラクタ引数または設定値として外から変更できる形にしてください。

## 11. Hint・走行指示SEQ通信

ETラリー有効時は、`alpha.py`が実機初期化と20ms周期開始の前に4桁復号キーを入力・確認します。Hintカード1・2が`RaceContext`へそろうと、走行体側のバックグラウンド通信スレッドがHint2を復号し、復号値を`hint2_gate_info`へ反映してからPCへ送信します。PC側は経路計算担当の関数を呼び、計算結果のJSONを走行体へ返します。Behavior Treeの周期処理では復号やソケット待機を行わず、キューを短時間確認するだけです。

受信した`strategy`は`RaceContext`へ保存され、No.11の受信待ち成功後にNo.12が走行命令へ変換します。PC側の`selectedLaps`も`selected_rally_laps`へ保存します。SEQは選択周回分を一つの配列として受信するため、走行体側では同じ配列を周回数分繰り返しません。

担当別の組込み方法、JSON形式、通信試験手順は次を参照してください。

- [通信担当・Hint読取担当・経路計算担当・経路実行担当向けガイド](../shared_communication/INTEGRATION_GUIDE_v2.md)
- [通信方式と実行コマンド](../shared_communication/README.md)

通信試験用の固定経路は`shared_communication/example_strategy.json`です。本番では`python -m wireless_device --host ROBOT_IP --planner module:function`の形式で経路計算担当の関数を指定します。

## 2026-09-09 スタート～LAP: 距離・方位角プロファイル v1

- 標準は `RaceConfig.start_lap_mode="profile"`。`"legacy"` で従来の5区間ジャイロ＋ライントレースへ戻せる。初期出力は `start_lap_power=33`。
- `start_lap_profile_v1.py` は公式50%コースPDFのL黒ラインを抽出した `(距離mm, 連続方位角deg)` 表。5mm間隔、1105点。開始0mm/0度、最初のカーブ入口500mm。Rightは既存の `-runtime.course * gyro_angle` と左右出力の符号で鏡像化する。
- 500mmはPDFのSTART印～最初の円弧接線点に対応すると仮定して全経路を比例換算した。倍率は0.7422056898 mm/PDF point（PDFをそのまま印刷した寸法の2.10389倍）。50%印刷そのものの実寸ではない。青先端5384.957mm、ゲート5511.717mmはこの仮定から計算した設計値であり、実測値ではない。START設置基準・コース縮尺が違う場合は表を再生成する。
- `RunByDistanceHeading` はRunByGyroを継承した別クラス。開始時の距離と方位を原点にし、距離から目標角を線形補間する。実方位も折返しを解消して連続値にする。共有RunByGyro/Plotter/IMU原点は変更していない。
- `--mission lap` はゲート位置+20mm、5531.717mmで停止する。全体・Hint工程は青先端の250mm手前から青を判定し、青検知で既存AT工程へ渡す。青未検知のままゲート+100mmまで達するとFAILURE/出力0。ATの100mm前進・200mm後退は変更していない。この縮尺では青先端～ゲートは126.759mmなので、ATの100mmのみでゲート通過済みとは判定しない。走行体・センサー・ゲート判定の位置関係を実機確認する。
- 途中停止・完了・失敗時はモーター出力0。PWMは0～100内に制限し、後退しない。距離減少・非有限センサー値・course未設定はFAILURE。1秒ごとに距離/目標角/実角/turnを記録する。
- 表の補間・左右操舵・角度折返し・PWM制限・再開始・中断停止・青検知窓・青見逃し・従来方式の構築を疑似センサーで検証。PIDの実機調整、横ずれ・スリップ・ジャイロドリフト、実際のゲート通過は未検証。方位追従だけでは横位置誤差は補正できない。
- 生成元/確認図/テストはETロボコン作業リポジトリの `work/distance-heading-v1/`。既存の工程順序とATへの青検知引渡しを照合し維持した。LAP単体の終了だけをゲート通過距離へ明確化した。
