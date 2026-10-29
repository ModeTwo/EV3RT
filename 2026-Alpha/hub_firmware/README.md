# ハブのプログラム(asp.bin)の書き換え記録

2026-10-29。ジャイロのくせ(bias)を、ブレーキで止まっている間もハブ自身が測り直すようにした版を試すための記録。

## 元に戻す方法
- 元の asp.bin(2026-06-05、libraspike-art 付属): ロボットの `~/asp_backup/asp_orig_20260605.bin`
  (md5 `92975509297c641319fcb51368442ecd`)。
- ハブを書き込みモードにして(USBを抜いて電源を切り、Bluetoothボタンを押しながらUSBを挿し、LEDが点滅するまで押す)、
  ロボットで次を実行する。
  ```
  cd ~/libraspike-art/spike && sudo ../scripts/deploy-dfu.sh ~/asp_backup/asp_orig_20260605.bin
  ```

## 作り方
- ソース: https://github.com/ETrobocon/spike-rt-RasPike-ART の `904096d`(元の asp.bin と同じ日付)。
  サブモジュールは `git submodule update --init --recursive` のあと、
  `external/libpybricks/micropython/lib/btstack` を `git submodule deinit -f` で外す
  (micropython側のbtstackが残っていると、そちらを使ってビルドに失敗する)。
- `sdk/workspace/raspike/spike_version` を `0.1.1` にする(ロボット側のライブラリが0.1.1を期待している)。
- `cd sdk/workspace && make img=raspike`(WSL2 Ubuntu、gcc-arm-none-eabi 14.2.1)。

## 変更(brakecal_libpybricks.patch、external/libpybricks に当てる)
- `pbio_imu_is_stationary()`: 「全モーターが脱力」→「全モーターが脱力またはブレーキ(電圧0)」。
  元の `pbio_dcmotor_all_coasting()` は `dcmotors->` を見ており、実際は先頭のモーターしか見ていなかった。
- くせの測り直しの重み: 20回目以降0.05 → 4回目以降0.3(数秒止まれば新しいくせに追いつく)。
- ハブが測り直すのは、1秒(約833回の読み取り)続けて止まっているときだけ。

## できたファイル(作業PCのWSL `~/hubfw/out/`)
| ファイル | 内容 | md5 |
|---|---|---|
| asp_baseline_904096d_v011.bin | 変更なし | 5f8a92d466ad2122e0d6e1c34f21a9bb |
| asp_brakecal_904096d_v011.bin | 上の変更あり | b7370340f8d21a40a0986a96f073e614 |
