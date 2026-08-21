from py_trees.trees import BehaviourTree
from py_trees.composites import Sequence
from py_trees.common import Status

# sample2.py のクラス群を利用する前提
# RunByGyro, SpinAround, TraceLine, IsQRDecoded, IsColorDetected, IsDistanceEarned, StopNow, IsTimePassed, TheEnd
# g_course, EXEC_INTERVAL, SPIN_MAX_POWER, SPIN_MIN_POWER なども sample2.py から利用する

"""
=== tantou2.py - 走行挙動調整ガイド ===

このプログラムは黒ライン検出 → 青ライントレース → QR読み取り の流れで走行します。
走行挙動に問題がある場合、以下のポイントを確認・調整してください：

【速度調整】
  - RunByGyro の power=33 : 直進速度（0～100、大きいほど速い）
    問題: 遅い/速い → power値を増減
  - TraceLine の power=33 : ラインをたどる速度
    問題: ラインから外れやすい → power を下げる

【PID微調整】
  - pid_p（比例ゲイン）: 偏差に対する反応速度
    大きい → 反応が敏感で振動しやすい / 小さい → 反応が鈍い
    調整例: pid_p=1.1 → 1.0 (少し優しく) or 1.3 (敏感に)
  
  - pid_i（積分ゲイン）: 定常偏差の補正（小数値）
    問題: 一定の偏差が残る → pid_i を大きくする
    調整例: pid_i=0.00075 → 0.001 (修正強化)
  
  - pid_d（微分ゲイン）: 行き過ぎを防ぐ（振動抑制）
    問題: 振動する → pid_d を大きくする
    調整例: pid_d=0.04 → 0.05 (振動を抑える)

【距離調整】
  - IsDistanceEarned の delta_dist: 指定距離走行（mm単位）
  問題: 手前/奥で停止する → delta_dist を減減/増加
  例: delta_dist=150 (15cm走行)

【色検出調整】
  - IsColorDetected(color=Color.BLACK): 黒ライン検出の閾値
  - IsColorDetected(color=Color.BLUE): 青ライン検出の閾値
  問題: 色を検出できない → sample2.py の ColorSensor キャリブレーション確認

【ラインをたどる微調整】
  - TraceLine の target=TRACELINE_TARGET_V: 目標光反射値（中央に保つ基準）
  - TraceSide.NORMAL: ライン左側/右側のどちらをたどるか
    問題: ラインの逆側をたどる → TraceSide.REVERSE に変更

【回転速度調整】
  - SpinAround の max_power/min_power: 回転速度の上下限
  問題: 回転がガタつく/精度が悪い → max_power下げる or min_power上げる

【待機時間調整】
  - IsTimePassed の delta_time=0.5: 安定化待機時間（秒）
  問題: センサーが不安定 → delta_time を増やす（0.7など）

【全体フロー変更のポイント】
  root.add_children([...]) の順序で実行順序が決まります。
  段階の追加/削除/順序変更はここで行います。
"""

def build_tantou_tree() -> BehaviourTree:
    """タントウ区間（QR読み取り含む）の走行シーケンスを構築
    
    流れ: 左90° → 黒ライン検出 → 右90° → 青ラインをトレース → QR1読 → 
         15cm走行 → 左90° → 150cmトレース → 右90° → QR2読 → 
         左90° → 青ラインまで走行 → 終了
    """
    root = Sequence(name="Tantou Section", memory=True)

    # --- 左に90°回転 ---
    # 【調整ポイント】
    # ・回転が遅い/早い → max_power/min_power を調整
    # ・回転精度が悪い → pid_p を小さく（0.15）、待機時間を増やす（delta_time=0.7）
    # ・想定と逆方向に回転 → target=-90 に変更
    turn_left_90 = Sequence(name="turn_left_90", memory=True)
    turn_left_90.add_children([
        SpinAround(name="left 90", target=90,
                   max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                   pid_p=0.2, pid_i=0.00075, pid_d=0.03,
                   target_type=HeadingType.RELATIVE),
        StopNow(name="stop_after_left"),
        IsTimePassed(name="wait_left", delta_time=0.5),
    ])

    # --- 黒ラインを踏むまで直進（約60cm目安） ---
    # 【調整ポイント】
    # ・直進速度が遅い → power を 40～50 に増加
    # ・ゆらゆらと蛇行する → pid_p を 1.0 に減少、pid_d を 0.05 に増加
    # ・黒ラインを検出できない → ColorSensor の値が適切か確認（サンプル値：反射率 < 30%）
    # ・想定より手前で停止 → 壁や障害物の影響確認、黒い部分の位置再確認
    go_to_black = Sequence(name="go_to_black", memory=True)
    go_to_black.add_children([
        RunByGyro(name="run_to_black", target=0, power=33,
                  pid_p=1.1, pid_i=0.00075, pid_d=0.04,
                  target_type=HeadingType.ABSOLUTE),
        IsColorDetected(name="detect_black", color=Color.BLACK),
        StopNow(name="stop_black"),
    ])

    # --- 右に90°回転 ---
    # 【調整ポイント】
    # ・回転角度が不正確 → SpinAround の pid_p=0.2 を pid_p=0.15 に下げて精密に
    # ・回転後のカメラ向きを調整 → 回転の min_power を上げると精密性増
    turn_right_90 = Sequence(name="turn_right_90", memory=True)
    turn_right_90.add_children([
        SpinAround(name="right 90", target=-90,
                   max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                   pid_p=0.2, pid_i=0.00075, pid_d=0.03,
                   target_type=HeadingType.RELATIVE),
        StopNow(name="stop_after_right"),
        IsTimePassed(name="wait_right", delta_time=0.5),
    ])

    # --- ライントレース ---
    # 【調整ポイント】
    # ・ラインから外れる → power を 25～30 に下げて慎重に進む
    # ・左右に揺れ続ける → pid_p=0.55 を 0.45 に下げる
    # ・ラインの右側をたどる → trace_side を TraceSide.REVERSE に変更
    # ・青ラインを見つけられない → TRACELINE_TARGET_V の値を確認（通常 50～70）
    # ・power 値が小さすぎて進まない → IsColorDetected の タイムアウト回避のため、
    #   distance_limit パラメータで上限距離を設定（超過時点で中止）
    line_trace_1 = Sequence(name="line_trace_1", memory=True)
    line_trace_1.add_children([
        TraceLine(name="trace_to_qr1", target=TRACELINE_TARGET_V, power=33,
                  pid_p=0.55, pid_i=0.0000009, pid_d=0.015,
                  trace_side=TraceSide.NORMAL),
        IsColorDetected(name="detect_qr1_blue", color=Color.BLUE),
    ])

    # --- QRコード1読み取り ---
    # 【調整ポイント】
    # ・QRコードが読めない → 回転で北向き（target=0）にしてカメラの向きを調整
    # ・回転精度が悪い → min_power を上げる（20～25）、delta_time を 0.7 に増加
    # ・常に南を向いているわけではない場合 → target_type=HeadingType.RELATIVE に変更
    # ・複数回の読み込み試行 → IsQRDecoded 内のリトライ回数を増やす
    qr1_read = Sequence(name="qr1_read", memory=True)
    qr1_read.add_children([
        SpinAround(name="face_qr1", target=0,
                   max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                   pid_p=0.2, pid_i=0.00075, pid_d=0.03,
                   target_type=HeadingType.ABSOLUTE),
        StopNow(name="stop_face_qr1"),
        IsQRDecoded(name="read_qr1"),
        StopNow(name="stop_after_qr1"),
    ])

    # --- QR1後、青ラインを踏むまで直進（約15cm目安） ---
    # 【調整ポイント】
    # ・距離・色検出の２つ条件を同時に満たす（AND）
    # ・目標距離に達しても青を検出できない → delta_dist を 200 に増加（20cm走行）
    # ・150mm 手前で青を検出する → delta_dist を 120 に減少
    # ・センサー精度が低い → IsDistanceEarned と IsColorDetected の順序を入れ替えて
    #   距離優先判定に変更するか、OR条件にする検討
    go_to_blue_after_qr1 = Sequence(name="go_to_blue_after_qr1", memory=True)
    go_to_blue_after_qr1.add_children([
        RunByGyro(name="run_15cm", target=0, power=33,
                  pid_p=1.1, pid_i=0.00075, pid_d=0.04,
                  target_type=HeadingType.ABSOLUTE),
        IsDistanceEarned(name="dist_15cm", delta_dist=150),
        IsColorDetected(name="detect_blue", color=Color.BLUE),
        StopNow(name="stop_blue"),
    ])

    # --- 左に90°回転 ---
    # 【調整ポイント】
    # ・前回の turn_left_90 と同じパラメータ、同じ調整ポイント参照
    # ・異なる環境での回転精度低下 → pid_p=0.15, delta_time=0.7 に調整
    turn_left_90_b = Sequence(name="turn_left_90_b", memory=True)
    turn_left_90_b.add_children([
        SpinAround(name="left 90 again", target=90,
                   max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                   pid_p=0.2, pid_i=0.00075, pid_d=0.03,
                   target_type=HeadingType.RELATIVE),
        StopNow(name="stop_after_left_b"),
        IsTimePassed(name="wait_left_b", delta_time=0.5),
    ])

    # --- ライントレース150cm ---
    # 【調整ポイント】
    # ・長距離走行でドリフト発生 → power を 20～25 に低下させて安定性重視
    # ・1500mm は約150cm（10m走行体の大部分の距離） の距離超過で停止
    # ・ラインから外れると戻れない → pid_p を 0.4 に低下させて穏やかに
    # ・1500mm に達する前にラインを失う → distance_limit を 3000 に延長して
    #   さらに奥のコースも探索する検討
    # ・長距離で積分ドリフト → pid_i を 0.00001 に低下（積分項の蓄積制限）
    line_trace_150 = Sequence(name="line_trace_150", memory=True)
    line_trace_150.add_children([
        TraceLine(name="trace_150", target=TRACELINE_TARGET_V, power=33,
                  pid_p=0.55, pid_i=0.0000009, pid_d=0.015,
                  trace_side=TraceSide.NORMAL),
        IsDistanceEarned(name="dist_150", delta_dist=1500),
        StopNow(name="stop_trace_150"),
    ])

    # --- 停止して右に90°回転 ---
    # 【調整ポイント】
    # ・トレース中の回転なので最初に停止する必要がある
    # ・回転角度ずれ → min_power を 20 に上げて精密回転実現
    # ・予期しない向きになる → ジャイロセンサーのキャリブレーション確認
    turn_right_qr2 = Sequence(name="turn_right_qr2", memory=True)
    turn_right_qr2.add_children([
        StopNow(name="stop_before_qr2"),
        SpinAround(name="right 90 for qr2", target=-90,
                   max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                   pid_p=0.2, pid_i=0.00075, pid_d=0.03,
                   target_type=HeadingType.RELATIVE),
        StopNow(name="stop_after_qr2_turn"),
    ])

    # --- QRコード2読み取り ---
    # 【調整ポイント】
    # ・QR1と同じ読み取りロジック、同じ調整ポイント参照
    # ・環境による新しい問題が発生した場合：
    #   - QRの角度が異なる → target 値を手動指定（target=-90 など）で方向固定試行
    #   - 読み込み失敗 → IsQRDecoded のタイムアウト時間延長
    qr2_read = Sequence(name="qr2_read", memory=True)
    qr2_read.add_children([
        SpinAround(name="face_qr2", target=0,
                   max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                   pid_p=0.2, pid_i=0.00075, pid_d=0.03,
                   target_type=HeadingType.ABSOLUTE),
        StopNow(name="stop_face_qr2"),
        IsQRDecoded(name="read_qr2"),
        StopNow(name="stop_after_qr2"),
    ])

    # --- 左に90°旋回して元の向きへ ---
    # 【調整ポイント】
    # ・QR2後の帰路を青ラインまで走行するため、北向き（ジャイロ北）に戻す
    # ・回転精度が悪い → pid_p=0.15, min_power=20 で精密実現
    # ・不正確な向き → delta_time=0.7 で待機時間延長
    return_heading = Sequence(name="return_heading", memory=True)
    return_heading.add_children([
        SpinAround(name="left 90 return", target=90,
                   max_power=SPIN_MAX_POWER, min_power=SPIN_MIN_POWER,
                   pid_p=0.2, pid_i=0.00075, pid_d=0.03,
                   target_type=HeadingType.RELATIVE),
        StopNow(name="stop_after_return_heading"),
    ])

    # --- 青線を踏むまで走行 ---
    # 【調整ポイント】
    # ・ゴール（青ラインの位置）が想定より遠い → power を 40 に増加
    # ・センサーが不安定で終了できない → IsColorDetected にタイムアウト追加
    # ・ジャイロドリフト → 走行距離上限を設定（距離超過でも停止）検討
    # ・目前で停止する → コース内での青ラインの位置を確認、距離上限設定
    go_to_blue_final = Sequence(name="go_to_blue_final", memory=True)
    go_to_blue_final.add_children([
        RunByGyro(name="run_to_blue_final", target=0, power=33,
                  pid_p=1.1, pid_i=0.00075, pid_d=0.04,
                  target_type=HeadingType.ABSOLUTE),
        IsColorDetected(name="detect_blue_final", color=Color.BLUE),
        StopNow(name="stop_final"),
    ])

    # --- 全体の流れ ---
    # 【調整ポイント】
    # ・シーケンスの追加：現在のステップ間に中間処理を挿入
    #   例）line_trace_1 と qr1_read の間にカメラ角度調整ステップ
    # ・シーケンスの削除：不要な待機や停止を削減
    # ・順序変更：色検出条件の優先度変更（距離優先 vs 色優先）
    # ・条件分岐：Parallel や Select コンポーザを導入して複数経路対応
    #   例）ラインを失った時の代替処理
    # ・リトライ機構：失敗時に前のステップに戻す（memory=True 活用）
    root.add_children([
        turn_left_90,
        go_to_black,
        turn_right_90,
        line_trace_1,
        qr1_read,
        go_to_blue_after_qr1,
        turn_left_90_b,
        line_trace_150,
        turn_right_qr2,
        qr2_read,
        return_heading,
        go_to_blue_final,
        TheEnd(name="end"),
    ])

    return BehaviourTree(root)


# --- 実行部（sample2.py と同じ構造） ---
# 【全体実行フロー調整のポイント】
# 1. コース選択（right/left）でコース分岐が可能
#    例）python tantou2.py right
# 2. EXEC_INTERVAL はメインループの実行周期（sample2.py で定義、通常 10-20ms）
#    問題: 制御が不安定 → EXEC_INTERVAL を大きくして周期を遅くする検討
# 3. dispatch(interval=...) でメインループが実行
#    各ステップが memory=True で失敗時に前段階から再開
# 4. 緊急停止：try-finally で cleanup_thread() が必ず実行される
#    問題: 停止できない → StopNow() の強制度を上げる検討

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('course', choices=['right', 'left'])
    args = parser.parse_args()

    # コース分岐による走行体の動作を変える
    # 例：right なら反時計回り、left なら時計回りの回転体設定など
    if args.course == 'right':
        g_course = -1
    else:
        g_course = 1

    setup_thread()

    try:
        # ETRobo フレームワーク初期化（RasPike-ART バックエンド で走行体制御）
        # ジャイロセンサー、カラーセンサー、モーター、エンコーダーの初期化
        etrobo = initialize_etrobo(backend='raspike_art')
        # 構築した Behaviour Tree を実行
        tree = build_tantou_tree()
        # ツリーを ETRobo のハンドラとして登録
        etrobo.add_handler(TraverseBehaviourTree(tree))
        # メインループ実行開始（EXEC_INTERVAL 毎に状態更新と制御実行）
        # センサー入力→判定→モーター出力が繰り返される
        etrobo.dispatch(interval=EXEC_INTERVAL)

    finally:
        # スレッド安全に終了処理
        cleanup_thread()
        print(" -- exiting...")


# ============ 緊急時の微調整クイックリファレンス ============
#
# 【走行が停止してしまう場合】
#   → 最初の「左90°回転」が失敗している可能性
#   → target_type=HeadingType.RELATIVE に変更（相対角度優先）
#   → 初期ジャイロキャリブレーション（初期化時 5 秒待機確保）
#
# 【黒ラインが見つからない】
#   → ColorSensor のキャリブレーション実施
#   → Color.BLACK の閾値を手動調整（sample2.py の BLACK_THRESHOLD）
#   → 走行開始地点の照度確認（暗すぎないか）
#
# 【ラインから外れる】
#   → power を 25～30 に低下（遅く、慎重に）
#   → pid_p を 0.45 に低下（反応を優しく）
#   → trace_side を反転試行（左/右逆にたどる）
#
# 【QRコードが読めない】
#   → 回転後の待機時間を 1.0 秒に延長
#   → target=0 を target=-90 / 90 に変更（方向再試行）
#   → カメラのフォーカス、照度確認
#
# 【長距離走行でドリフト】
#   → ジャイロセンサーのドリフト補正（定期キャリブレーション）
#   → distance_limit を設定して無限走行防止
#   → pid_i を低下（0.00001）して積分蓄積を制限
# ======================================================
