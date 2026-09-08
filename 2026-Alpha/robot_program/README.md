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
| `ENABLE_ET_SUMO` | `enable_et_sumo` | `True` / `False` |
| `ENABLE_FINISH` | `enable_finish` | `True` / `False` |

`mission_mode="configured"`では、スタート～LAPゲートを含むすべての工程が上記フラグに従います。すべて`False`なら、キャリブレーションとタッチスタートの後に終了し、LAP走行は開始しません。

`enable_et_rally=False`の場合、`et_rally_laps`の値にかかわらずETラリー周回は実行されません。`et_rally_laps=0`の場合もETラリー周回は実行されません。

設定ファイルを変更せずに工程を単体実行する場合は、`alpha.py`の`--mission`を使用します。

| 工程 | コマンド例 |
|---|---|
| 設定フラグどおり | `python alpha.py left --mission configured` |
| LAP | `python alpha.py left --mission lap` |
| Bottle Delivery | `python alpha.py left --mission bottle` |
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
   ├─ password_input.py             # 復号キー入力
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

現在の責務分担では、Hintの復号と走行指示SEQ生成はPC側が担当します。走行体側Featureへ`Hint`と`HintType`を追加する前に、PC側で処理できないか確認してください。

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
| `bottle.py` | `IsBottleInsight`、`HasCaughtBottle` |
| `hint_reader.py` | `ReadHintCard` |

`RunAsInstructed`は従来から使用している名称を維持しています。

### ET相撲 No.15～18

ET相撲は距離センサーを使用せず、カメラで力士ボトルの黒テープを捕捉します。ETラリー終了位置は青円上を想定するため、開始直後はライントレースせず、ジャイロで方位を維持して直進します。黒ラインを確認して白地へ抜けた後、調整可能なクリアランス距離だけ追加直進して土俵方向へ90度旋回し、さらに50mm後退してカメラ視野を広げます。

ET相撲でも、旋回は`behaviours/gyro_drive.py`の`SpinAround`、方位維持走行は`RunByGyro`、距離終了判定は`IsDistanceEarned`、停止は`StopNow`を利用します。Feature内にモーター出力やPID旋回を重複実装しません。

Feature内に残すのは、黒テープの連続フレーム確認、画像中央へ寄せる前進操舵、カメラ死角へ入った後の捕捉距離設定など、ET相撲に固有の処理だけです。P1・P2の2地点推定、座標経路計画、`sumo_geometry.py`は使用しません。旧距離センサー方式のファイルは比較・復帰用に残していますが、ET相撲の実行木からは外しています。

```text
move_to_sumo_start.py
├─ 青円上のETラリー終了位置からジャイロ直進
├─ 黒を確認した後、白を0.5秒連続確認して黒ライン終端と判定
├─ 白地上を調整可能なクリアランス距離だけ追加直進
├─ Leftでは左、Rightでは右へ90度旋回して土俵方向を向く
├─ 両コースで50mm後退して制動停止
└─ capture_sumo_bottle_camera.py
   ├─ 静止状態で黒テープを異なる3フレーム連続確認
   ├─ 首振り旋回をせず、画像角度を0度へ寄せながら前進
   ├─ 黒テープが画面下端の死角へ入った時の方位を保存
   └─ 保存方位を維持して調整可能な距離を直進し、下端アーム内へ捕捉
      └─ move_to_sumo_exit.py
         ├─ ボトルを保持したまま押し出し側の黒ラインまで直進
         ├─ 黒ライン検知後、30mm追加直進して押し出す
         ├─ 旋回せず直線後退してアームから離脱
         ├─ 後退しながらガレージ側へ緩く曲がり、復帰用黒ラインを検知
         └─ 短距離ライントレースで姿勢を整えて停止
```

調整値は`RaceConfig.sumo`の`SumoSettings`へ集約しています。ET相撲開始位置の黒線退出は、共通色分類のWHITEではなくカラーセンサーの生V値を使用します。Vが`line_black_max_value`以下になった後、`line_white_min_value`以上の状態が`line_exit_white_duration_sec`継続した場合に明るい路面へ抜けたと判定します。判定中はHSVと段階を0.25秒間隔でログへ出します。白地確認後は`post_line_clearance_distance_mm`だけ追加直進してから90度旋回します。後退距離は`camera_retreat_distance_mm`（初期値50mm）、後退出力は`camera_retreat_power`（初期値60）です。

カメラは`begin_sumo_bottle_read()`から開始し、黒だけに追跡対象を固定します。Bottle Deliveryは`begin_bottle_read()`で色固定を解除して赤・青・黄を判定するため、ET相撲の黒判定が残りません。同一画像を複数回数えず、異なる3フレームで確定してから前進します。

ET相撲の駆動出力設定はすべて50以上です。相撲単体の`--mission sumo`でも、カメラ撮影・画像処理・プレビュー用スレッドを起動します。黒テープを3秒以内に確定できない場合、または接近が8秒を超えた場合はモーターを停止し、捕捉と後続運搬を省略します。

No.16・17では90度旋回後の探索旋回を行いません。カメラで得た黒テープの角度に応じ、基準PWM75へ最大±25の差を付けて両輪50以上のまま前進操舵します。0.25秒間隔で画像角度、画像下端位置、面積、左右PWMをログへ出します。テープが死角へ入った後は、最後の観測方位を既存の`RunByGyro`へ渡して規定距離だけ直進します。

No.18では、捕捉したボトルを保持したまま黒ラインまで直進し、黒ライン検知後に`push_out_after_line_distance_mm`（初期値30mm）だけ追加直進して押し出します。その後は旋回せず、`release_reverse_distance_mm`（初期値150mm）だけ真っすぐ後退してアームから離脱します。

離脱後は`garage_return_reverse_left_pwm`と`garage_return_reverse_right_pwm`の左右差を使い、後退しながら少しガレージ側へそれて復帰用黒ラインを探します。この左右差はRightコースで鏡像化されます。黒ライン検知後は`line_rejoin_trace_distance_mm`（初期値100mm）だけ既存の`TraceLine`で走り、FINISH工程へ渡せる姿勢に整えて停止します。ガレージ側へ曲がる向き、150mmの離脱距離、100mmの安定化距離はレプリカコースで調整してください。

Contextには工程ごとに`bottle_pushed_out`、`bottle_released`、`line_trace_ready`を記録します。最後は`transport_completed=True`、`bottle_held_at_exit=False`となります。ログでは押し出し線への接近、直線後退による離脱、ガレージ側黒ライン復帰を個別に確認できます。

ET相撲開始時にはアームが下端にあることを上流工程の事後条件とし、ET相撲内ではアームモーターを動かしません。捕捉成功を直接検知するセンサーは使わず、黒テープが死角へ入った後に`camera_blind_capture_distance_mm`へ到達したことを捕捉成立として扱います。50mm後退量、黒テープ閾値、死角後150mmの初期値、出口運搬経路は実機で確認・調整してください。

`alpha.py`の`ArmUpDownFull`は、エンコーダー回転量の変化が5度未満の状態を5周期連続で検知すると、機械端へ到達したと判断してPWMを0にし、ブレーキを有効にします。実行周期が20msの場合、終端到達後の判定時間は約0.1秒です。

## 7. 工程間で値を渡す方法

他工程へ渡す値は`RaceContext`へ保存します。担当機能同士を直接importして値を参照しないでください。

現在の共有項目：

| フィールド | 内容 |
|---|---|
| `bottle_color` | 認識したボトル色 |
| `hint1` | Hintカード1の読取結果 |
| `hint2` | Hintカード2の読取結果 |
| `strategy` | 走行指示SEQ |
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
