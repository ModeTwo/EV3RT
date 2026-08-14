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

## 3. 機能の有効・無効とETラリー周回数

設定は`robot_program/config.py`の`RaceConfig`で変更します。

```python
@dataclass(frozen=True)
class RaceConfig:
    enable_bottle_delivery: bool = True
    enable_et_rally: bool = True
    et_rally_laps: int = 3
    enable_et_sumo: bool = True
    enable_finish: bool = True
```

要求上の設定名との対応は次のとおりです。

| 要求上の設定 | `RaceConfig`のフィールド | 設定値 |
|---|---|---|
| `ENABLE_BOTTLE_DELIVERY` | `enable_bottle_delivery` | `True` / `False` |
| `ENABLE_ET_RALLY` | `enable_et_rally` | `True` / `False` |
| `ET_RALLY_LAPS` | `et_rally_laps` | `0` / `1` / `2` / `3` |
| `ENABLE_ET_SUMO` | `enable_et_sumo` | `True` / `False` |
| `ENABLE_FINISH` | `enable_finish` | `True` / `False` |

スタート～LAPゲートは競技の基本工程なので、設定にかかわらず常にツリーへ追加されます。

`enable_et_rally=False`の場合、`et_rally_laps`の値にかかわらずETラリー周回は実行されません。`et_rally_laps=0`の場合もETラリー周回は実行されません。

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

`ArmDirection`と`ArmUpDownFull`はキャリブレーションでのみ使用するため、共通`behaviours/`へ分割せず`alpha.py`に配置しています。

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
| `conditions.py` | `IsDistanceEarned`、`IsColorDetected`、`IsTimePassed` |
| `bottle.py` | `IsBottleInsight`、`HasCaughtBottle` |
| `hint_reader.py` | `ReadHintCard` |

`RunAsInstructed`は従来から使用している名称を維持しています。

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

## 10. 単体テスト

リポジトリ直下から次のコマンドで実行します。

```text
.venv\Scripts\python.exe -m unittest discover -s 2026-Alpha\robot_program\tests -t 2026-Alpha -v
```

実機を使わない単体テストでは、`runtime`へFakeMotor、FakeSensor、FakeVideoなどを設定してBehavior単体を確認します。

実機調整が必要な値は、Behaviorのコンストラクタ引数または設定値として外から変更できる形にしてください。
