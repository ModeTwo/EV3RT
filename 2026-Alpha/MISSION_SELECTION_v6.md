# 工程単体実行ガイド v6

## 修正理由

従来の標準値 `mission_mode="hint2"` は工程フラグより優先され、すべてのフラグを `False` にしてもLAPから始まっていました。標準を `configured` に変更し、通常時は `RaceConfig` のフラグを必ず参照するようにしました。

## 実機での選択

2026-Alphaをカレントディレクトリとして、対象コースと工程を指定します。

```sh
python alpha.py left --mission lap
python alpha.py left --mission bottle
python alpha.py left --mission rally
python alpha.py left --mission sumo
python alpha.py left --mission finish
```

Rightコースは `left` を `right` にします。単体指定時は設定ファイルの他工程フラグにかかわらず、対象だけを選択します。`rally`はヒント取得等のETラリー準備と周回を一組で構成します。設定上の周回数が0の場合、明示的な `rally` 指定では最低1周にします。

全工程、RE→AT→TO接続試験、設定フラグどおりの起動は次のとおりです。

```sh
python alpha.py left --mission configured
python alpha.py left --mission full
python alpha.py left --mission hint2
python alpha.py left --mission hint2-return
```

`--mission`を省略した場合は`configured`です。

## RaceConfigを直接使う場合

`robot_program/config.py`の`RaceConfig`を変更します。例えばET相撲だけなら次の状態です。

```python
mission_mode: str = "configured"
lapgate: bool = False
enable_bottle_delivery: bool = False
enable_et_rally: bool = False
et_rally_laps: int = 0
enable_et_sumo: bool = True
enable_finish: bool = False
```

この状態で `python alpha.py left` を実行するとET相撲だけが選択されます。すべてを`False`にすると、キャリブレーションとタッチスタート後に終了し、LAPは実行しません。

## 単体試験の開始条件

工程選択は上流工程を疑似実行しません。実機は各工程の想定開始位置へ置き、方位とアーム状態も合わせてください。

| 工程 | 想定する開始状態 |
|---|---|
| LAP | 競技スタート位置 |
| Bottle Delivery | LAPゲート工程終了位置 |
| ETラリー | Bottle Delivery／ETラリー準備の開始位置。ヒント取得から実施 |
| ET相撲 | ETラリー終了位置、所定方位、アーム下端 |
| FINISH | ET相撲終了位置 |

キャリブレーションとタッチスタートは、どの指定でも共通して実行されます。

## 未実装工程

現在、Bottle Delivery、ETラリー、FINISHには`PendingFeature`が残っています。ユーザー方針により、該当ノードは警告を表示し、モーターやセンサーを操作せず`SUCCESS`を返して次の工程へ進みます。

```sh
python alpha.py left --mission bottle --check-tree
```

`--check-tree`で表示された `*_pending` が実装対象です。LAPとET相撲には現在`PendingFeature`がありません。

実行開始時には次の警告を表示しますが、終了コード2では停止しません。

```text
-- WARNING: skipped unimplemented features: ...
```

未実装動作を実施したことにはなりません。例えばボトル配置がPendingなら、配置せずに次の移動へ進みます。未実装工程の前後で想定位置や保持状態が成立しない可能性があるため、全工程を連続実行する場合は接触・逸脱に注意し、すぐ停止できる状態で試験してください。

QRデコーダーの事前確認は、`rally`、`hint2`、`hint2-return`のようにHint読取を含む場合だけ行います。LAP・ET相撲・FINISH単体ではQRデコーダーを起動条件にしません。カメラ本体は現在の共通初期化・ResetDeviceとの互換性のため生成されます。

## 構成だけ確認する

```sh
python alpha.py left --mission lap --check-tree
python alpha.py left --mission sumo --check-tree
```

`--check-tree`では実機、モーター、カメラを開きません。

## 復元

修正前の2026-Alpha全体は次へ保存しています。

- `backups/pre_feature_selection_20260906_v1.zip`
- SHA-256：`AC8169C6A66F097783A800C704CA7A86EB117B873718C50C8732D1F53DEAEC1F`

既存ファイルへ直接上書きせず、別の空フォルダへ展開して比較してください。
