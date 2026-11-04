# ET相撲：デリバリー後の3工程の引き継ぎ

このフェーズは `robot_program/phases/et_sumo_after_delivery.py` の `build_et_sumo_after_delivery_phase(context, config)`。元の設計と同じくphaseは順序だけを表し、各動作はfeaturesに置く。

1. `features/sumo_delivery_transition.py` の `build_move_delivery_to_sumo_straight`：デリバリー終了位置から②の直進開始位置まで。担当者が走行ツリーを実装。
2. `features/sumo_push_tree.py` の `build_sumo_push_tree`：実装済みの直線接近・正対・捕捉接近・押し出し・正対位置付近への復帰。詳細は `SUMO_STRAIGHT_HANDOFF_v1.md`。
3. `features/sumo_rally_transition.py` の `build_move_sumo_to_rally_start`：②の終了位置からETラリー開始位置まで。担当者が走行ツリーを実装。

①③は暫定で `StopNow` の後に `Failure` を返す。担当者は各featureの2ノードを実測済みの走行ツリーに置き換える。現在は動かず未実装のまま後続工程へ通過しない。②の復帰先は②の正対完了位置付近で、厳密な位置保証はない。緑への不侵入と物理的押し出し成功は未保証。

結合担当者はデリバリー終了後にこのphaseを組み込み、共有 `context` と `config` を渡す。既存の `build_et_sumo_phase` や `tree_builder.py` の旧順序には自動接続されていない。接続先の変更は他の通しミッションに影響するため、全体工程の担当者が行う。既存のボトルデリバリーphaseにはラリー開始位置へ動く処理が含まれるので、このphaseの①入口と物理的に一致するよう呼出し元を調整する。

②を構築する `config` は、`mission_mode` を単体の `sumo-push-route-test` 以外とし、`sumo_push_calibration=None`、`sumo_push_execute=True`、`sumo_relative_push.push_distance_mm` に明示値を設定する。未指定ならツリー構築時にエラーとする。押し出し距離に仮値を自動設定しない。機器初期化、gyro倍率、ResetDevice後の相撲方位登録、タッチ開始、制御周期は既存のalpha・全体ツリーが所有する。全体開始基準の初回直進目標はLEFT -90°、RIGHT +90°の実装だが、物理コースとの整合は実機確認が必要。

元の `build_et_sumo_phase` はラリー出口から相撲へ進む従来経路であり、この新phaseとは別物。単体 `sumo-push-route-test` は従来どおり使える。①③の配置・経路・左右差、②③の受渡し精度、結合走行は未検証。
