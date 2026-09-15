# AT・TOの単体実行 v1

2026-Alphaフォルダで、普段と同じPython環境から実行する。

```sh
pypy3 alpha.py right --mission at
pypy3 alpha.py right --mission to
```

左コースではrightをleftに変更する。起動後は通常のアーム上下初期化・センサーリセットが行われ、タッチ待ちになる。配置やボトルのセットは初期化完了後、タッチする前に行う。

## AT

青線検知直後に相当する位置と向きに配置する。スタート地点からのRE走行や青線探索は行わない。タッチすると100mm前進→200mm後退→色認識→460mmライントレース→停止までを実行する（距離はIntegrationSettingsで変更可能）。ボトル色はAT bottle_colorログで確認できる。TOには進まない。
編集先はrobot_program/features/catch_bottle.py。

## TO

AT終了位置・向きに配置し、必要なボトルをセットする。タッチ開始後にその位置・向きを基準として記録。元のTOツリーをそのまま実行し、ヒント1→ヒント2→出口走行→停止まで進む。AT走行、色認識、ボトル配置、相撲、PC戦略受信は実行しない。ボトル色の入力は不要。QR認識のログで取得値を確認する。
編集先はrobot_program/features/to_hint_route.py。単体実行のための別コピーは作らず、通常統合と同じコードを使用する。

## 構成だけの確認

```sh
pypy3 alpha.py right --mission at --check-tree
pypy3 alpha.py right --mission to --check-tree
```

共通周期・Ctrl+Cの終了処理・カメラ片付けは通常のalpha.pyと共通。新しい工程時間制限は設けない。コードと疑似センサーによる検証済み、実機は未検証。
