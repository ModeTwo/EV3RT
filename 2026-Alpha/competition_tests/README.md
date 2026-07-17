# 競技別実機テスト

`left_course_2026.py` が生成する本番サブツリーを抽出して実行します。
テスト側へ競技ロジックをコピーしていないため、本番修正とテスト内容がずれません。

すべて `2026-Alpha` ディレクトリで、Raspberry Pi上からモジュールとして起動します。

```bash
python -m competition_tests.lap_gate
python -m competition_tests.bottle_delivery --wireless-host 192.168.1.50
python -m competition_tests.et_rally --route-file competition_tests/example_rally_route.json
python -m competition_tests.et_sumo
python -m competition_tests.garage
```

## 開始位置

- `lap_gate`: スタート位置へ置く。
- `bottle_delivery`: LAPゲート通過後、デリバリーボトルをカメラ正面へ置く。
- `et_rally`: 最上段の赤ゾーン近くの青線上で、機体を上向きに置く。
- `et_sumo`: ET相撲開始位置で、探索対象ボトルをカメラの探索範囲へ置く。
- `garage`: ガレージへ続く黒ライン上で、進行方向へ向けて置く。

各テストはアーム上下端確認とデバイス初期化から開始し、対象工程終了後に左右モーターを停止します。
初期化後はタッチセンサーが押されるまで待機するため、開始位置と周囲の安全を確認してからテストを開始できます。
ボトルデリバリーテストでは、キャリブレーション中にヒントカード2用の4桁パスワードを入力します。

`example_rally_route.json` の距離・角度は構造確認用の仮値です。実コースでそのまま使用しないでください。
