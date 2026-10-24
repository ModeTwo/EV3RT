"""ETラリー旋回の仕上げ制御(LQI・一発パルス仕上げ)を設計するための、PC側オフラインツール一式。

ここにあるものは、いずれも走行体の実行には関与しない(通信・経路計算のプロセスとは無関係)。
`robot_program/behaviours/et_rally_turn_calibration.py` が実機で記録したログ
(run_logs内、"TURN_CAL" タグの行)を読み込み、モデルの当てはめ・ゲインの計算・
オフラインでの動きの試算までを行う。このb, c, stiction_floorはLQI専用ではなく、
one_shot_pulse.py(一発パルス仕上げのパルス表作成)でも同じデータを使う。

    parse_log.py      ログから、較正用の記録を構造化された表へ取り出す。
    fit_model.py      応答特性の記録から、モデル(b, c)を最小二乗法で求める。
    design_gains.py   b, c と重み(Q, R)から、LQI(積分付きLQR)のゲインKを求める。
    one_shot_pulse.py b, c, stiction_floorから、一発パルス仕上げ用のPULSE_TABLEを作る。
    simulate.py       モデルとゲインで、閉ループの挙動をオフラインで試算する
                      (実機に載せる前の確認、および検証用ログとの答え合わせに使う)。
"""
