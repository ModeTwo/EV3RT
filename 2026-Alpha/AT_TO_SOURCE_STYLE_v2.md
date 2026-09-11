# AT・TO編集案内 v2

ATの編集先はrobot_program/features/catch_bottle.py、TOはrobot_program/features/to_hint_route.py。
TOは元のbuild_tantou_treeを直接接続。sectionsとsection_names、および4つの工程呼出しファイルは廃止した。
統合時に追加したstop_black、TO stop before hint1、TO stop before hint2は削除。元からある旋回後・QR読取後・ゴール等のStopNowは維持。
通常はTOの出口走行まで続ける。既存のhint2単体モードだけinclude_exit=FalseでQR2読取後に終了する。hint2-returnと通常のボトル工程は出口走行を含む。
共通周期、時間制限なし、AT終了方位基準、色・ヒントの共有保存、終了処理は維持。v1の工程窓口・追加停止の説明は本版で置き換える。
