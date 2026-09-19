# PC側ETラリー経路計算

通常起動では、走行体から受信したHint1と復号済みHint2を、ZIP提供の経路計算へ渡して走行指示SEQを返信する。

```sh
python -m wireless_device --host RASPBERRY_PI_IP
```

計算ランナーの指定は不要。`WirelessDeviceApplication`から`StrategyPlanner`が呼ばれ、次の入力を使用する。

- `hint1`: 赤ゲート。`XY,XY`形式
- `hint2`: 青ゲート、黄ゲートの順。`XY,XY/XY,XY`形式。走行体側で復号済み
- `course`: `left`または`right`
- `laps`: PC側で選択した1～3。走行体から受信した値ではなく、PC起動設定を使用

周回数は無線通信デバイス側で指定する。省略時は3周。

```sh
python -m wireless_device --host RASPBERRY_PI_IP --et-rally-laps 3
```

環境変数`ET_RALLY_LAPS`でも設定できる。PCが選択した周回数は返信payloadの`selectedLaps`へ格納され、その周回数で最後の完了位置まで計算した全走行戦略を`strategy`へ格納する。

走行体のETラリー走行部分だけと結合する場合は、PCを上記コマンドで起動し、走行体を`--mission rally-drive`で起動する。復号済みHintの指定、実機配置、確認順序は[ETラリー走行・無線通信単体試験](../robot_program/ET_RALLY_DRIVE_STANDALONE_v1.md)を参照する。

## ファイルの責務

- `strategy_planner.py`: 通信プログラムから呼ばれる安定した入口。ランナーの実装詳細を持たない。
- `planner_process.py`: 選択した計算ランナーを独立プロセスで起動し、共通JSON契約を処理する安定層。
- `et_rally_runner.py`: 現行計算専用の変換アダプター。Hint文字列、コース、周回数を計算本体の入力へ変換する。
- `et_rally_planner/`: ZIPから取り出した現行の計算本体。今回の構造変更では内容を変更していない。
- `transport.py`: TCP受信、ACK、計算呼出し、走行指示SEQ返信を担当する。

計算本体は`config.py`など一般的なトップレベル名をimportする構造になっている。PC通信プロセス内の別モジュールと衝突させないため、計算時だけ独立した短命プロセスを使用する。これはHintが揃った後の経路計算時に1回起動するだけであり、走行体の20ms制御周期では動作しない。同一missionIdの再送は`transport.py`のキャッシュを返すため、経路を再計算しない。

## 経路計算プログラムを更新する

通信側と計算側の境界は、計算ランナーの標準入力・標準出力だけに固定している。計算担当者はTCP、CRC、ACK、再送処理を変更しない。

```text
transport.py
  -> application.py
    -> strategy_planner.py
      -> planner_process.py
        -> 選択された計算ランナー
          -> 任意の経路計算本体
```

ランナー契約は次のとおり。

- 標準入力：UTF-8 JSONのobject。`hint1`、復号済み`hint2`、`course`、`laps`を含む。
- 標準出力：UTF-8 JSONの走行指令`list[dict]`を1個だけ出力する。
- 診断ログ：標準エラーへ出力する。標準出力へログを混在させない。
- 失敗：0以外の終了コードにする。通信側はERROR返信へ変換する。

現行計算本体の内部実装だけを変更し、呼出し方法が変わらない場合は`et_rally_planner/`を更新するだけでよい。以前の固定ハッシュ検査は、正当な更新のたびに通信コードの変更を要求していたため廃止した。変更検出と履歴管理はGit、動作確認は下記単体テストで行う。

計算本体の配置やAPIまで変わる場合は、`et_rally_runner.py`だけを新APIに合わせる。または新しいランナーを別ファイルで追加し、次のように選択して比較できる。相対パスは起動位置ではなく`2026-Alpha`基準。

```sh
python -m wireless_device --host RASPBERRY_PI_IP --planner-runner wireless_device/new_planner_runner.py
```

環境変数でも選択できる。

```sh
set ET_RALLY_PLANNER_RUNNER=wireless_device/new_planner_runner.py
python -m wireless_device --host RASPBERRY_PI_IP
```

指定がなければ`wireless_device/et_rally_runner.py`を使用する。ランナーは計算ごとに新規起動するため、開発中に内容を更新すれば、PC通信コードを再編集せず次の計算から更新後の実装が使われる。既に処理済みの同一missionIdは通信キャッシュを返すため、更新確認には新しいミッションを開始する。

`right`ではZIP付属`export_plan_right_course.py`と同じく、開始・終了位置、方位、ゲート配置を左右反転してから計算する。

## 解析用JSONログ

実走では、経路計算結果をPythonの`list[dict]`としてメモリ上で受け取り、そのまま既存TCP通信へ渡す。JSONログは連携に使用せず、設定した場合だけ同じ計算入力と返信内容をファイルへ複製する。

コマンドラインで有効化する場合：

```sh
python -m wireless_device --host RASPBERRY_PI_IP --strategy-log-dir strategy_logs
```

環境変数で常時有効化する場合：

```sh
set ET_RALLY_STRATEGY_LOG_DIR=strategy_logs
python -m wireless_device --host RASPBERRY_PI_IP
```

どちらも未指定ならJSONファイルは作成しない。保存ファイル名は`strategy_<missionId>_<UTC時刻>.json`で、次を含む。

- `requestPayload`: 走行体から受信した元のHint1、復号済みHint2、コース等
- `planningPayload`: PCが選択した周回数を反映した実際の計算入力
- `response`: PCが返す走行指示SEQ、または計算エラー
- `missionId`、ログ形式バージョン、記録時刻

メモリ上の走行指示SEQをTCP送信した後でJSONログを保存する。保存に失敗した場合はエラーを通常ログへ記録するが、送信済みの走行指示には影響しない。走行時にこのJSONを読み戻す処理はない。復号済みゲート情報を含むため、不要になったログの取扱いには注意する。

## テスト

```sh
python -m unittest wireless_device.test_strategy_planner -v
python -m unittest shared_communication.test_exchange -v
```

通信だけを固定SEQで確認するときは従来どおり`--strategy-file`を使用できる。計算プログラム全体の差替えには`--planner-runner`を使用する。軽量な同期関数を同一PCプロセス内で比較する場合に限り、従来の`--planner module:function`も使用できる。


## 2026-09-13 ETラリー方位契約

送信SEQは全体スタート方向0度、左右コース正規化済みの方位角。
ラリー入口の規定方向は-90度。PCと走行体を同時更新する。
headingFrame=full-start-course-normalized-v1がない旧PCの応答は採用しない。

計算担当のet_rally_planner/、et_rally_runner.pyは変更不要。
計算ランナー出力は従来どおり「ラリー開始方向0度・数学座標の反時計回り正・Rightは幾何鏡映済み」。
transport.pyがheading_frame.pyを介して一度だけ変換する。
別ランナー、--planner、--strategy-fileも同じ入力角度契約に従う。
送信後の共通方位SEQを--strategy-fileへ再投入すると二重変換になるので使用しない。
計算担当が角度規約を変更するときは共有境界heading_frame.pyだけを協議して更新する。

全体走行は起動時ジャイロ0基準を維持し、途中でリセットしない。
--mission rally-driveはラリー開始位置かつ規定の内向きに置いて起動・初期化する。
ジャイロ0を全体方位-90度と対応させるため、実行直前のSEQコピーから固定初期方位を引く。
受信ContextのSEQ、ログは全体方位のまま。実測到着角は変換に使用しない。
位置だけでなく向きも規定どおり合わせ、初期化後に車体を回さない。
既存fileモードのplanは従来契約を保持し、この受信SEQ変換は適用しない。

テストスクリプトはローカル保持・Git除外。実行用plan_seed9392783.jsonは追跡を維持する。
実機での左右方位・配置精度確認は未実施。
