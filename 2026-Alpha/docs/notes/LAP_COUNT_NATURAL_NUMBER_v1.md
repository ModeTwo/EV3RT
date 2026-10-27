# ETラリー周回数を正の整数へ拡張

## 変更

無線通信デバイスの `--et-rally-laps` と `ET_RALLY_LAPS` は、1以上の整数を受け付ける。0、負数、小数は拒否する。既定値は既存CLIの2周を維持する。

PC側の要求処理と経路計算ランナー、および走行体のStrategy受信検証も同じ条件にそろえた。PCは指定周回数分の全走行SEQを生成し、`selectedLaps` とともに返信する。走行体はSEQを重複展開せず一度だけ実行する。

## 留意点

形式上は上限なしだが、経路探索量・返信データ量・総走行時間は周回数に応じて増える。任意の大きな数が実機競技で実行可能という保証ではない。実用上限は経路計算時間、通信タイムアウト、走行時間と合わせて決める必要がある。通信ペイロードには既存の1 MiB上限がある。

## 対象ファイル

- `wireless_device/__main__.py`
- `wireless_device/transport.py`
- `wireless_device/et_rally_runner.py`
- `robot_program/services/strategy_exchange.py`
- `docs/notes/wireless_device/README.md`
