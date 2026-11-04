# ET相撲：デリバリー後の3工程の引き継ぎ

このフェーズは `robot_program/phases/et_sumo_after_delivery.py` の `build_et_sumo_after_delivery_phase(context, config)`。元の設計と同じくphaseは順序だけを表し、各動作はfeaturesに置く。

1. `features/sumo_delivery_transition.py` の `build_move_delivery_to_sumo_straight`：デリバリー終了位置から②の直進開始位置まで。担当者が走行ツリーを実装。
2. `features/sumo_push_tree.py` の `build_sumo_push_tree`：実装済みの直線接近・正対・捕捉接近・押し出し・正対位置付近への復帰。詳細は `SUMO_STRAIGHT_HANDOFF_v1.md`。
3. `features/sumo_rally_transition.py` の `build_move_sumo_to_rally_start`：②の終了位置からETラリー開始位置まで。担当者が走行ツリーを実装。

①③は暫定で `StopNow` の後に `Failure` を返す。担当者は各featureの2ノードを実測済みの走行ツリーに置き換える。現在は動かず未実装のまま後続工程へ通過しない。②の復帰先は②の正対完了位置付近で、厳密な位置保証はない。緑への不侵入と物理的押し出し成功は未保証。

結合担当者はデリバリー終了後にこのphaseを組み込み、共有 `context` と `config` を渡す。既存の `build_et_sumo_phase` や `tree_builder.py` の旧順序には自動接続されていない。接続先の変更は他の通しミッションに影響するため、全体工程の担当者が行う。既存のボトルデリバリーphaseにはラリー開始位置へ動く処理が含まれるので、このphaseの①入口と物理的に一致するよう呼出し元を調整する。

②を構築する `config` は、`mission_mode` を単体の `sumo-push-route-test` 以外とし、`sumo_push_calibration=None`、`sumo_push_execute=True`、`sumo_relative_push.push_distance_mm` に明示値を設定する。未指定ならツリー構築時にエラーとする。押し出し距離に仮値を自動設定しない。機器初期化、gyro倍率、ResetDevice後の相撲方位登録、タッチ開始、制御周期は既存のalpha・全体ツリーが所有する。全体開始基準の初回直進目標はLEFT -90°、RIGHT +90°の実装だが、物理コースとの整合は実機確認が必要。

元の `build_et_sumo_phase` はラリー出口から相撲へ進む従来経路であり、この新phaseとは別物。単体 `sumo-push-route-test` は従来どおり使える。①③の配置・経路・左右差、②③の受渡し精度、結合走行は未検証。

## ①②③の単体・連続試験（全体ミッションへの接続は保留）

`robot_program/features/sumo_stage_test_settings.py` の `SumoStageTestSettings` に試験条件を記入する。`push_distance_mm` は捕捉後に追加で押す距離で、初期値300mm。②単体と①→②→③の試験で使用する。`approach_distance_mm`・出力・捕捉上限・押し出し出力・旋回出力も同じ設定にまとめる。①③は担当者が経路と開始・終了位置の数値を対応するfeaturesへ実装する。

タッチ開始時の車体の床面方位はleft=270°、right=90°、ジャイロをリセットした向きからの②初回直進目標差は両コース0°が初期値。いずれも同設定ファイルの `initial_bearing_*_deg` と `approach_offset_*_deg` で変更できる。テスト時には開始位置・向きを各工程の入口に合わせて配置する。①→②→③試験では①の到達向きもこの初回目標に合う必要がある。通しの全体スタート基準とは異なるため、結合時に整合確認が必要。

```bash
cd /home/pi/EV3RT/2026-Alpha
pypy3 alpha.py left --mission sumo-stage-entry --no-preview
pypy3 alpha.py left --mission sumo-stage-push --no-preview
pypy3 alpha.py left --mission sumo-stage-exit --no-preview
pypy3 alpha.py left --mission sumo-stage-all --no-preview
```

leftはrightに変更可能。各コマンドの`--no-preview`を`--check-tree`に変えるとツリーを表示する。押し距離をNoneに変更した場合は②を含むツリーは構築されない。①③は未実装ならStopNow後にFAILUREで停止し、次へ進まない。configの距離・出力はこの4モードだけに適用し、既存のsumo-push-route-testや本来の全体ミッションの設定は変わらない。現在の変更はPCの模擬検証のみで、Pi上での実行や実走は別途確認が必要。


2026-11-04: 試験用設定の押し出し距離初期値を300mmに変更。測距・旋回の診断プログラムは `robot_program/diagnostics/` に整理した。
