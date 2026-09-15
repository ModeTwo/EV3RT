# devAK 引き継ぎ書 v2 — 現在のdevTK・ボトルデリバリー終盤

作成日: 2026-09-15。対象: Codex、Claude Code、担当者。会話履歴や特定AIのツールに依存せず再開するための文書。

## 0. v2で確定した現状と前版からの差

本書と同じフォルダの`DEVAK_STATE_v2.json`に、確認日時（UTC）、ブランチ、HEAD、Git状態、実コードのSHA-256を記録した。ハッシュはPC上のその時点の内容を識別するためのもので、Pi配備済み・実機合格を意味しない。コード本体は同梱していない。現在の一式を他PCへ受け渡す際はこの一覧と照合する。

| 項目 | 現在のdevTKで確認した状態 | devAKへの影響 |
|---|---|---|
| No.7/8/9 | 色別選択→配置→赤前中央・内向き。60/120/250mm、配送PWM50を維持 | 主担当手順は第3節を使用 |
| TO方位 | HandoffState.absolute_headingは指令角をそのまま返し、AT終了実測角を加算しない | 旧「AT終了を局所0度」の説明を使用しない |
| QR1→QR2 | line_trace_120は90°軸の投影進捗1000mmで終了 | 従来の実走1000mmとは停止位置が変わり得る |
| 共通ジャイロ | 絶対目標を現在方位に最も近い360°等価角へ解決 | 配送旋回も共通SpinAround経由で影響あり |
| Hint2出口 | 白→その場旋回→黒探索/追従。早期判定課題は残る | 配送入口の成立を別途確認 |
| LAP手前 | カメラSEEK→ジャイロALIGN→カラー追従。青は並列ラッチ | lap単体だけでは本番の復帰経路を検証できない |

**記録との不一致:** 9/15のREADME/HANDOFFにはLAPカラー追従PWMを50へ変更した記録があるが、本書作成時の実ファイル`config.py`は`start_lap_line_power=60`。配送の`delivery_trace_power`は50。コードを正として60を記載し、今回は設定を変更していない。変更の経緯・意図は未確定。上書きで50に戻さず担当者の作業履歴と照合する。

現在のdevTKを再現するには未コミット・未追跡コードも必要。特に`camera_line_trace.py`、`hint2_exit.py`、`projected_distance.py`、`integration_runs.py`は確認時点で未追跡。`git pull`やHEADコミットだけの転送では不足する。既存の別作業差分をまとめてコミットせず、共有する版と範囲を確認する。

### 起動時の方位を混同しない

`--delivery-initial-heading 180`は配送座標の登録値だけを変える。ジャイロ自体を180°へ変更したり、TO指令角へ180°を足したりする指定ではない。

right、reset gyro=0、配送登録180の場合:

- TO論理方位 = gyro。TO目標0°は起動時の向き。
- 配送方位 = 180 + gyro。配送目標0°は起動時の反対向き。
- TOの190°は配送では370°（10°相当）。ただしHint2Exitは目標到達前の黒検出でFOLLOWへ移るため、終了姿勢が必ず配送10°になるわけではない。
- AT終了実測18°は記録には残るが、次のTO目標90°を108°へ変更しない。

配送前の見た目に合わせて基準を勝手に再登録すると通し走行の絶対方位契約が崩れる。開始位置・初期方位・実測ログをセットで判断する。

### 周辺制御の詳細と残課題

1. TOのrun_to_blackは目標90°・距離565mm、run_after_qr1は目標0°・距離385mm。どちらもPWM60、P=.0001/I=.00001/D=.04。現在の黒/青検知ノードはコメントアウトされており、名前だけで色検知終了と判断しない。P補正が極めて弱く、24°誤差でもP出力.0024で整数化される。9/15実走ログにはQR1後の0°近傍から−24°への逸脱があるが、Pi実ファイルとの一致は未確認。
2. line_trace_120はTraceLine（目標明度65、PWM60、PID=.65/.000001/.045）とIsProjectedDistanceEarnedの並列。浮動小数点XY差をTO90°軸へ投影する。名前dist_1200でも現行目標は1000mm。横移動は進捗に含まず後退は減算。専用の実走距離上限・時間上限がないため、軸に直交/逆方向へ逸脱すると終わらない可能性が残る。PROJECTED_DISTANCEのprojected_mmとtravelled_mmを両方確認する。
3. 共通gyro_driveのSpinAroundと固定角RunByGyroは、例えば現在314°・指令−44°なら316°へ解決し初期差を+2°とする。開始ログのrequested/resolved/deltaで確認。相対角と距離関数モードをこの修正と混同しない。これは初期PID目標の解決であり、すべての走行中の方位逸脱を直したものではない。
4. 本番LAPは青予測位置の500mm手前（現在約4356mm）からSEEK。PWM50/turn上限30、V<=65を3周期でALIGN。ALIGNはPWM35/gyro P=.8/turn上限25、0±5°を3周期確認してカラー追従へ。青検知を並列ラッチし0±5°の3周期安定で終了、復帰全体Timeout10秒。カメラframe_id/captured_atの鮮度検査はなく、停止した画像でinsight=Trueが残る問題は未対処。通常lap単体はこの復帰分岐を通らない。
5. ATの前進100mm・後退200mmは配送と同じDriveDistance（左右同PWM、方位補正なし）。配送の250mm往復とともに物理的ずれの要確認箇所。Hint2、LAP、ATを配送の変更だけで改善済みと扱わない。

## 1. 最初に確認すること

devAKの担当はボトルデリバリー終盤。主担当範囲はNo.7「配置先選択」、No.8「設置」、No.9「ラリー開始位置へ復帰」。TOのHint2出口を入口境界、ETラリーを出口境界とする。Hint取得・復号・経路計算は既存担当の処理を使う。

**本書はdevAKブランチの完成報告ではない。** 確認した実装はローカルdevTKの作業ツリー（HEAD `93d266aa2d50defb2d7f4b15679f53f6435fe499`＋未コミット変更）。ローカル参照の`origin/devAK`は`3b8c3e2`であり、devTKと多数の差がある。fetchは未実施なのでリモートの最新状態は未確認。devAKへ実装を取り込み済みと判断しないこと。

Gitのルートは`EV3RT`、実行ディレクトリはその中の`2026-Alpha`。以下、コードパスと実行コマンドは原則`2026-Alpha`基準。

1. 適用されるAGENTS.md等を確認し、`docs/notes/README.md`を読む。
2. 本書、`MISSION_SELECTION_v6.md`、`INTEGRATION_RUN_v2.md`を読む。起動全般は`RACE_STARTUP_v2.md`も参照する。ただし同文書には旧LAP 5100mm設定等が残っており、現行コードと最新履歴を優先する。
3. 資料リポジトリもある場合は、その`README.md`、`HANDOFF.md`、`references/README.md`を読む。HANDOFFには先頭の新規記録と末尾の追記が混在するので日付だけで最新実装を決めない。
4. 次を実行し、ブランチ・未コミット差分・実ファイルを確認する。

```sh
git rev-parse --show-toplevel
git branch --show-current
git rev-parse HEAD
git status --short
git diff --stat
git diff --cached --stat
git log -1 --oneline origin/devAK
```

既存の作業ツリー上でcheckout、reset、clean、全ファイルの上書きをしない。必要なら新しいcloneまたは別worktreeでdevTKの受領環境を準備し、現行一式を照合後に担当ブランチdevAKへの統合を行う。取得先は`https://github.com/ModeTwo/EV3RT.git`。新規cloneの例（既存ディレクトリがない場所で実行）:

```sh
git clone --branch devTK https://github.com/ModeTwo/EV3RT.git EV3RT-devTK-handoff
cd EV3RT-devTK-handoff/2026-Alpha
```

cloneだけでは本書のdevTK作業ツリーは再現できない。現行統合版を共有コミットとして受領するか、差分と新規ファイルを含む配備一式を受領して照合する。`git diff`だけでは未追跡ファイルが含まれない。ブランチ名だけを根拠に古い単体スクリプトへ戻さない。

## 2. 入口・出口の契約

入口は「Hint2後の移動が完了し、黄ゾーン前の最初の青ラインより手前で、配送ラインの進行方向に向き、ボトルを保持している」。開始位置が青線上・青線通過後だと番号を一つずらして数える可能性がある。位置をコードが自動認識するわけではない。

必要な共有状態は`context.bottle_color`と登録済み`context.delivery_heading`。色はRED/BLUE/YELLOW（CLIはred/blue/yellow）。No.7が`selected_drop_zone`、No.8が`bottle_delivered=True`、No.9が`rally_ready=True`を設定する。後二つは制御完了の記録であり、現物の設置・位置・向きのセンサー保証ではない。

出口は「最上段＝赤ゾーン前の青ライン中央でラリー内側を向き停止」。後続は既存のラリー工程へ接続する。`rally_ready=True`はStrategy受信済みを意味しない。工程境界でジャイロ、距離、デバイス、共有contextをリセットしない。

## 3. 処理順序の詳細

構成元は`robot_program/phases/bottle_and_rally_preparation.py`の`build_bottle_delivery_final_phase`。順序は`build_select_drop_zone → build_drop_bottle → build_move_to_rally_ready`、memory=TrueのSequence。

### No.7 色別の青ライン選択

`robot_program/features/select_drop_zone.py`。

全色とも最初に黄ゾーン前の青ラインを検知するまで通常側ライントレース。検知した周期で停止する。青ボトルの色認識と路面の青線検知は別の情報。

| ボトル色 | 最初の青線から配置先へ | 置いた後のNo.9 |
|---|---|---|
| 黄 | 最初の青線の手前端で停止 | 中央から60mm→次の青線→120mm→赤前青線→60mm中央 |
| 青 | 120mm通過→次の青線の手前端で停止 | 中央から60mm→赤前青線→60mm中央 |
| 赤 | 120mm通過→次の青線→120mm通過→赤前青線で停止 | すでに赤前中央なので追加直進なし |

120mmは青線を抜けて同じ線を再検知しないための距離であり、隣のゾーンまでの間隔ではない。中間は青検知まで追従する。無効色は`SelectBottleDropZone`で警告し赤へフォールバックする。結合入口には有効色チェックもあるため、どの入口でも無効色が赤になると決めつけない。

青検知は`IsColorDetected(Color.BLUE)`によるHSV分類。終盤側のこの判定には連続検知回数条件がない。青探索に専用距離上限・Timeoutは付いていないので、見逃し時に必ず自動停止すると説明しない。

### No.8 配置

`robot_program/features/drop_bottle.py`。

1. 選択した青線の手前端からライントレースで60mm進み中央へ。
2. 配送方位の絶対目標−90°へ旋回。停止＋0.5秒待機。
3. `DriveDistance`で250mm前進（PWM +50）。
4. 同じ距離250mm後退（PWM −50）して青線中央へ戻る。
5. 絶対目標0°へ復帰旋回。停止＋0.5秒待機。
6. `bottle_delivered=True`。

現行は前進・後退による配置であり、No.8にはアーム開閉命令やボトル解放確認はない。`DriveDistance`は左右同PWM・距離差の絶対値で終了し、ジャイロ直進補正ではない。左右差や床面で前後移動がずれても同じ位置へ戻った保証はない。

### No.9 ラリー開始位置

`robot_program/features/move_to_rally_ready.py`。上表の経路で赤前青線中央へ移動し、絶対目標+90°へ旋回、停止＋0.5秒待機、明示停止、`rally_ready=True`。ボトルなしの分岐も存在するが、devAKの通常試験では色選択済み分岐を使う。

## 4. 方位と調整箇所

配送基準はライン進行方向0°、ドロップ側−90°、ラリー内側+90°。生ジャイロ角を直接目標に入れない。`robot_program/delivery_heading.py`の換算は `登録方位 − course × (現在gyro − 登録gyro)`、right=-1、left=+1。

`alpha.py`でデバイスリセット後に登録する。bottle-final/bottle-rallyの初期配送方位は0°、その他は標準180°。to-bottleだけは`--delivery-initial-heading`で実際の設置方位を明示する。TO共通方位190°と配送方位190°は同じ座標系ではない。TOはAT終了方位を加算しない方式へ変更済み。

`robot_program/behaviours/delivery_turn.py`のクラス名は`DeliveryPulseTurn`だが、現行はパルス補正を使わず共通`SpinAround`の連続PIDへ委譲する。名称から過去の実装を推測しない。配送旋回P=.2/I=.00075/D=.03、最小55/最大60、後段待機0.5秒。共通SpinAroundの変更は他担当にも波及する。現在は近傍360度等価角の解決も共通実装に含まれ、配送旋回もその変更の影響を受ける。

| 設定（IntegrationSettings） | 現行値 | 調整の意味 |
|---|---:|---|
| delivery_trace_target_v | 75 | 明度目標 |
| delivery_trace_power | 50 | ライン追従PWM |
| delivery_marker_full_width_mm | 120 | 手前端から線を抜ける距離 |
| delivery_marker_half_width_mm | 60 | 手前端→中央／中央→線外 |
| delivery_drop_distance_mm | 250 | 前進と後退の両方に適用 |
| delivery_drive_power | 50 | 前進PWM、後退は負符号 |
| delivery_drop_turn_deg | −90 | 配置方向の絶対目標 |
| delivery_inward_turn_deg | +90 | ラリー内向き絶対目標 |

設定元は`robot_program/integration_settings.py`。追従PID=.65/.000001/.045はNo.7/8/9各ファイル内にある。NORMAL側追従を維持する。共通の`to_spin_*`はTOと共用なので配送だけの調整に使うと影響する。変更は一度に一要因、前後値と理由を記録する。120/60/250mmは現行コード値であり、本書で公式寸法として再認定したものではない。

## 5. 実行手順

Pythonと実機依存（etrobo_python、py_trees、simple_pid等）が入った既存走行環境を使う。`py_etrobo_util`はリポジトリ内にもある。固定依存版の一覧は本書では確定できていないため、動作済みPiのPython/パッケージ版を記録して揃える。PCのcheck-treeもimport依存が必要。最新依存を無条件で入れ替えない。

まず静的構成確認（走行しない）:

```sh
python alpha.py right --mission bottle-final --bottle-color yellow --check-tree
python alpha.py left --mission bottle-final --bottle-color blue --check-tree
python alpha.py right --mission bottle-final --bottle-color red --check-tree
```

実機単体（入口位置・配送方位0°に設置）:

```sh
python alpha.py right --mission bottle-final --bottle-color yellow
```

leftおよびblue/redも同様。初期化ではアームが動くため、保持ボトルとの干渉を確認する。初期化後に入口姿勢とボトル保持を確認し、タッチで開始する。bottle-finalはHint撮影やラリー走行を含まない。Ctrl+CはPiの走行プロセスへ送り、左右モーター停止を確認する。

TOからの結合は、物理的なTO開始位置と配送座標での初期方位を測定してから実施する。次の`<実測方位>`は置換必須で、そのまま実行しない。

```text
python alpha.py right --mission to-bottle --bottle-color yellow --delivery-initial-heading <実測方位>
python alpha.py right --mission at-to-bottle
```

at-to-bottleはATから色を取得するので`--bottle-color`を指定しない。構成確認は末尾に`--check-tree`を付ける。to-bottleのcheck-treeで省略できる初期方位0は検査用の既定値であり、実走の正しい設置角ではない。

実際にTO開始姿勢を配送180°として合わせた場合のコマンド例（9/15ログと同じ引数。適切な物理配置を確認して使う）:

```sh
python alpha.py right --mission to-bottle --bottle-color red --delivery-initial-heading 180 --check-tree
python alpha.py right --mission to-bottle --bottle-color red --delivery-initial-heading 180
```

hint2-returnは出口だけを開始するモードではなく、現行tree_builderではLAPとヒント収集を含む。Hint2直後へ置いてこの名前のモードを起動しない。

bottle-rallyは配送入口からラリーまで。`--rally-hint1`、`--rally-hint2-gate-info`の入力形式は`alpha.py`と既存通信資料で確認し、実データを使用する。Hint1形式例は`25,35`、Hint2形式例は`53,54/12,22`（alpha.pyのcheck-tree用値。競技の実データではない）。構成確認は次で可能:

```sh
python alpha.py right --mission bottle-rally --bottle-color red --check-tree
```

実走の引数例は次の各プレースホルダーを実データへ置換する:

```text
python alpha.py right --mission bottle-rally --bottle-color red --rally-hint1 "<復号済みHint1>" --rally-hint2-gate-info "<復号済みHint2>"
```

通信が必要な試験はPCを先行起動する。

```text
python -m wireless_device --host <PiのIP> --port 50000 --et-rally-laps 3 --planner-runner wireless_device/et_rally_runner.py --strategy-log-dir strategy_logs
```

## 6. 直近の未解決事項と切り分け

Hint2出口の旋回・検出条件はv4相当（その場旋回55～60）のまま。現在はTO方位のAT終了角加算廃止も適用されている。PCとPiの同版確認・現行一式での実機確認は未完。前版実走では白面を早期検出して旋回を開始し、目標角到達前にV<=65の黒判定で追従へ切り替え、白継続で失敗した。v4は出力2値の変更だけで、早期白判定と角度条件なしの黒捕捉は残る。配送単体が成功してもTOからの入口成立を保証しない。

関連: `robot_program/behaviours/hint2_exit.py`、`features/to_hint_route.py`。現行WHITEは、一度V<=75を読んだ後、区間距離30mm以降でV>=85を3周期確認するとTURNへ移る。追加前進0mm、TO目標190°、追従80mm、白15周期で失敗。TURN/BLACKは角度に関係なくV<=65を3周期確認するとFOLLOWへ切り替わる。TURNからFOLLOWへの逆転出力履歴を保持する点も未解決。これらをdevAKで無断に全面改変せず、TOとの入口位置・向きの合意を先に行う。

| 症状 | 最初に確認する点 |
|---|---|
| 一つ先のゾーンに置く | 開始位置、最初の青検出、120mm後の同一線再検出 |
| 配置後に青線へ戻れない | 実際の−90°、左右同PWMの前後ずれ、ボトル解放 |
| 単体成功／結合で逆を向く | delivery_heading登録、to-bottleの実設置角、ジャイロ再リセット |
| 青線を見逃して走り続ける | HSV実測、センサー高さ、NORMAL側、探索上限未実装 |
| BOTTLEに到達しない | INTEGRATION TOの終了状態、Hint2出口ログ。配送PIDから直さない |
| SUCCESSなのに置けていない | 完了フラグは制御完了のみ。物理的保持／解放と250mmを確認 |

## 7. 受入確認と次の担当者への記録

左右×3色の6条件で、選択青線、中央停止、配置後の現物、復帰位置、最終内向き姿勢を確認する。単体→TO結合→必要なラリー結合の順に拡大する。成功・失敗・Ctrl+C中断の停止を確認する。青未検出、無効色の入口差も確認対象。実機未実施の項目を合格扱いにしない。

各回に日時、course、色、mission、Gitコミットと差分、変更設定、初期位置/方位、バッテリー状態、青検出位置、終了位置/角度、手押しの有無を記録。Piの`run_logs/`を保存する。結合では`INTEGRATION START/END stage=BOTTLE`、`delivered`、`ready`を確認。手押しした走行の距離を自力走行の合格根拠にしない。

共有前は対象ファイルのハッシュまたは差分をPC/Piで照合する。コミットIDが同じでも未コミット差分があれば同版ではない。コピーは元と同じ相対パスへ行い、別ファイル内容の誤配置を防ぐ。

変更範囲の中心はNo.7/8/9、delivery_heading、delivery_turn、integration_settings。統合の確認先はalpha.py、tree_builder.py、context.py、integration_runs.py、services/strategy_tree.py。共通line_trace、gyro_drive、section_motion、conditions、ColorClassifierの変更は既存工程との互換確認が必要。`archive/branches/`は由来確認用で、現行入口に差し替えない。

2026-AlphaのMarkdownはすべて`docs/notes/`配下。ルートへAGENTS.md/CLAUDE.md/README.mdを新設しない。この文書を最初のプロンプトで明示して読ませる。大きな改訂はv3など新しい版にし、入口READMEを更新する。資料リポジトリのHANDOFFにも実施内容・理由・未確認事項・最新成果物・全変更パスを追記する。outputs更新時は生成コードと記録を含めcommit/pushが必要。既存他作業を一括add/commitしない。

## 8. 別AIへ渡す開始プロンプト

> このリポジトリの2026-Alpha/docs/notes/README.mdとDEVAK_HANDOFF_v2.mdを読み、devAKのボトルデリバリー終盤（No.7～9）を担当してください。最初に現在ブランチ、未コミット差分、devAKと統合版の差を確認してください。本書は2026-09-15のdevTK作業ツリーの説明で、devAKへの反映済みを意味しません。現行ファイルを照合し、色別選択→設置→ラリー開始位置の順序、方位登録、共有状態を維持してください。単体と結合の確認を分け、実機未検証を明記し、他担当の変更を保持してください。文書はdocs/notes配下へ保存し、変更した全ファイルを区分付きで報告してください。

本書作成時は現行ソース・既存記録の照合、対象PythonのAST構文確認、ハッシュ採取を実施。check-tree・実BT試験・実機試験は実施していない。走行コードの改変、devAKへのマージ、Pi転送、実機試験は実施していない。
