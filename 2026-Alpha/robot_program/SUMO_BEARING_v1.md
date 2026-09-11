# ET相撲だけの方位角移行・単体試走

## 基準と適用範囲

コース図の上0度、右90度、下180度、左270度。時計回りが正です。
相撲のFeatureは方位角を指定し、`features/sumo_bearing_motion.py`が既存の
`-course * gyro`基準へ変換して、変更していない`SpinAround`・`RunByGyro`を呼びます。
他工程の`ABSOLUTE`や`RELATIVE`、共通runtime・ResetDevice・Plotterは変更しません。
旧距離センサー探索は実行木に入っておらず、今回の移行対象外です。

## 全体走行

```sh
python alpha.py left --mission full
python alpha.py right --mission full
```

全体スタートの下向き180度を前提に、ResetDevice完了直後の実ジャイロ値と対応づけます。
`configured`も同じ全体スタート前提です。上流工程から相撲へ到着しても再登録しません。
相撲に入るまでにジャイロを再リセットしたり、持ち上げて向きを変えたりしないでください。
現在の上流コードでは途中のジャイロリセットは見つかっていません。
Plotterの初回リセットはキャリブレーション中の方位登録より前です。

## 相撲単体の実機試走

相撲開始地点に機体を置き、その場で起動・キャリブレーション・タッチ開始します。
全体スタート位置へ持っていく操作は不要です。

```sh
# 相撲開始地点でコース図の上を向ける（既定値も0度）
python alpha.py left --mission sumo --sumo-initial-bearing 0
python alpha.py right --mission sumo --sumo-initial-bearing 0

# 構成だけ確認し、モーター・カメラは起動しない
python alpha.py left --mission sumo --sumo-initial-bearing 0 --check-tree
```

`--sumo-initial-bearing`はリセット時の実際の配置方位であり、走行目標ではありません。
上向き以外に置く場合もその実方位を指定できますが、開始直進の目標は0度なので、
通常の試走は上向き配置を使ってください。キャリブレーション後に向きを変更しないでください。
この引数は`--mission sumo`だけで使用でき、他ミッションで指定すると起動前にエラーになります。

## 相撲内の処理順

1. 方位0度を維持して初期位置から350mm。
2. 土俵方位へ旋回：Left270度／Right90度。設定距離だけ後退して撮影。
3. カメラの画像右を正とする角度差を現在方位へ足し、ボトル目標方位を固定。
4. 検出確定位置からキャッチ・押し出し込みで500mm。出力設定は従来どおり。
5. 規定距離後退。後退完了時の方位±50度から、180度との最短角度差が大きい方を選択。
   同点はLeftで反時計回り、Rightで時計回り。例えば160度なら110度、200度なら250度。
6. 選んだ方位へ旋回後、方位維持直進して黒を探索。
7. 黒検知後30mm追加直進、方位180度合わせ、100mmライントレース、停止。

相撲の設定は`SumoSettings`の`entry_bearing_deg`、`ring_bearing_left_deg`、
`garage_bearing_deg`、`garage_search_offset_deg`。相対20度の旧設定は使いません。
方位差は計算に必要な値なので方位角と区別します。モーター命令へは変換済みの目標を渡します。
旧`camera_capture_heading_deg`は旧方式用で、現行は`camera_capture_bearing_deg`に方位角を保存します。

## 検証・実機確認

```sh
python -B robot_program/tests/test_sumo_bearing.py
python -B robot_program/tests/test_sumo_total_distance.py
```

上記はハードウェア不要の模擬テストです。実コードの座標変換・旧走行処理の出力方向を
模擬PID・デバイスで確認し、実際の制御周期、制動の行き過ぎ、画像認識精度は保証しません。
初回は基準登録ログの`bearing`と配置方向が一致し、手動の時計回り90度で方位が90度増えることを確認してください。
画像角度も左右コースで反転しないため、ボトルが画像右なら目標方位は時計回り側になります。
方位ログと、既存Behaviorの`heading`（旧ジャイロ基準ログ）は単位が同じでも基準が異なります。

## 元に戻す

復元ポイント：`rollback/sumo-bearing-20260909/`。
`before.zip`は今回変更した既存ファイルの変更前の内容、`manifest.json`は前後ハッシュと新規ファイル一覧です。

```powershell
# EV3RT/2026-Alphaで実行。実行中のロボットプログラムは先に終了する。
powershell -NoProfile -ExecutionPolicy Bypass -File rollback/sumo-bearing-20260909/restore.ps1
```

復元は今回変更したファイルのみが対象です。変更後に他の編集が入っていれば上書きせず中断します。
新規追加ファイルは削除せず、同フォルダ内の退避ZIPへ保存してから取り外します。
バックアップZIP、復元スクリプト、履歴は残します。他工程の変更やGitの状態をリセットしません。
Linuxへ持ち込んだ場合はこのPowerShell手順をWindowsのリポジトリで実行し、復元結果を再配備してください。
