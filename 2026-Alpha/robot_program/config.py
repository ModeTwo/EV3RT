"""Race feature switches used by the robot-side tree builder."""

from dataclasses import dataclass, replace
from typing import Optional

from .sumo_types import SumoSettings
from .integration_settings import IntegrationSettings


@dataclass(frozen=True)
class RaceConfig:
    # 試験時に工程単位で有効・無効を切り替えられるよう、設定値を一か所へ集約する。
    # configuredでは以下の工程フラグをそのまま使用する。
    # hint2系はRE→AT→TO接続試験を残すための専用モード。
    mission_mode: str = 'configured'
    lapgate : bool = True
    # profile: PDF距離-方位表。legacy: 従来の固定方位+ライントレース。
    start_lap_mode: str = 'profile'
    #start_lap_mode: str = 'legacy'
    start_lap_power: int = 85
    # 車軸中心から最初のカーブまで。アーム先端から500mm＋前方100mm。
    start_lap_first_straight_mm: float = 600.0
    # 最初のカーブ以降の距離倍率。別途実測するまでは旧表の長さを維持。
    start_lap_route_scale: float = 1.0
    # devRE完成版のスタート～LAP調整値。
    start_lap_pid_p: float = 1.8
    start_lap_pid_i: float = 0.0
    start_lap_pid_d: float = 0.03
    # 曲率から旋回出力を先行して与えるdevRE完成版の設定。
    start_lap_feedforward_gain: float = 1.0
    start_lap_wheel_tread_mm: float = 110.0
    # 横ずれはエンコーダ/IMUの推定値。0mmなら横補正を無効化できる。
    start_lap_cross_track_lookahead_mm: float = 300.0
    start_lap_max_heading_correction_deg: float = 8.0
    start_lap_log_interval_sec: float = 0.2

    # 青検知開始地点からLAPゲートまでのライン追従。白面ではライン方向へ強く旋回する。
    start_lap_line_target_v: int = 75
    # ボトルデリバリー終盤と同じカラーセンサー追従値。
    start_lap_line_power: int = 60
    start_lap_line_pid_p: float = 0.65
    start_lap_line_pid_i: float = 0.000001
    start_lap_line_pid_d: float = 0.045
    # 規定距離後は固定旋回せず、カメラでラインへ寄せる。
    start_lap_camera_before_blue_mm: float = 500.0
    start_lap_camera_power: int = 50
    # 2026base/camera_trace_demo.py の実走サンプル値。
    start_lap_camera_pid_p: float = 2.0
    start_lap_camera_pid_i: float = 0.0
    start_lap_camera_pid_d: float = 0.06
    # ラインへ短く進入するSEEKと、黒捕捉後に0度へ戻すALIGNを分ける。
    start_lap_camera_max_turn: int = 30
    start_lap_camera_align_power: int = 35
    # ALIGN後、通常PIDを始める前にライン端の目標明度へ穏やかに寄せる。
    start_lap_camera_handoff_power: int = 35
    start_lap_camera_handoff_pid_p: float = 0.3
    start_lap_camera_handoff_turn_cap: float = 10.0
    start_lap_camera_handoff_v_tolerance: int = 10
    start_lap_camera_handoff_stable_samples: int = 5
    start_lap_camera_tilt_ff_gain: float = 8.0
    start_lap_camera_ff_cap: float = 8.0
    start_lap_camera_stable_samples: int = 3
    start_lap_camera_gyro_kp: float = 0.8
    start_lap_camera_gyro_turn_cap: float = 25.0
    # カラーセンサーが黒側を3周期連続で読んだら通常追従へ渡す。
    start_lap_camera_rejoin_v: int = 65
    start_lap_camera_rejoin_samples: int = 3
    start_lap_heading_tolerance_deg: float = 5.0
    start_lap_blue_heading_stable_samples: int = 3
    # 青検知は距離と分離する。これは衝突防止の独立した非常停止時間。
    start_lap_blue_timeout_sec: float = 10.0
    enable_bottle_delivery: bool = True
    enable_et_rally: bool = True
    et_rally_laps: int = 3
    # received: PCから受信したSEQ、file: 従来の固定plan JSONを実行する。
    et_rally_strategy_source: str = "received"
    # ETラリー開始時にエラー(経路の受信失敗、Hint不足、経路の変換失敗)が出たとき、ミッションを失敗にせず、
    # 固定ルートでゴール(ガレージ)へ向かう(et_rally_fallback.py)。Falseなら、従来どおり失敗で止まる。
    et_rally_fallback_enabled: bool = True
    # Noneならtests/plan_seed9392783.json。相対パスは2026-Alpha直下を基準にする。
    et_rally_plan_path: Optional[str] = None
    # DD-19以降の較正やり直し用。spin_pid: 無補正PID旋回(gyro_drive.SpinAround、較正前のベースライン)、
    # spin_encoder: エンコーダ主導+ジャイロ仕上げ(現行既定、DD-13)、one_shot: 一発パルス仕上げ、
    # lqi: LQI(積分付き状態フィードバック)仕上げ。直進側はspin_*でも常にEtRallyRunByGyroを使う
    # (直進の実現方式はDD-13の対象外のため切替なし)。
    et_rally_turn_method: str = "spin_encoder"
    # DD-19較正用: spin_encoder/one_shot/lqiのフェーズ2(仕上げ)許容誤差を上書きする。
    # Noneなら各方式の素の既定値(0.5度)を使う。spin_pidとの純粋な方式比較時、
    # spin_pid側の許容誤差(2.0度固定)に揃えるために使う。
    et_rally_turn_fine_tolerance_deg: Optional[float] = None
    # DD-22〜23較正用: spin_encoderのフェーズ1で、残りの旋回角度がこの値未満になったら
    # decel_power(既定50)まで落として勢いを削ってから止める(既定0.0=無効)。
    # フェーズ1の慣性による行き過ぎ→フェーズ2(揺れ・バックラッシュの発生源)での
    # 補正量そのものを減らす狙い。
    et_rally_turn_decel_deg: Optional[float] = None
    # DD-22較正用: spin_encoderで、前の旋回と逆方向に切り替わる旋回だけ、フェーズ1の
    # 目標エンコーダ角度にこの分を上乗せする(機構内バックラッシュの先詰め)。既定0.0=無効。
    et_rally_turn_backlash_preload_deg: Optional[float] = None
    # DD-22較正用: spin_encoderで、フェーズ2(ジャイロ仕上げ)を無効化し、フェーズ1
    # (エンコーダのみ)だけで旋回を終える。フェーズ2自体が追加の方向反転(行き過ぎの
    # 戻し)を起こし、ズレの一因になっていないかを切り分けるための診断用。
    # Noneなら素の既定値(True、フェーズ2を使う)のまま。
    et_rally_turn_fine_trim_enabled: Optional[bool] = None
    # DD-22較正用: spin_encoderのフェーズ1の駆動パワーを上書きする。片輪エンコーダ
    # テスト(power=50既定)と同じ出力で比較するために使う。Noneなら素の既定値(70)。
    et_rally_turn_main_power: Optional[int] = None
    # DD-22較正用: spin_encoderのフェーズ1で、先に目標へ到達して止まる側(その旋回で
    # マイナス出力になる側)の目標値を、この分だけ低くする。まだ回っているもう片方の
    # 反力で「止まった後も同じ方向へ引き込まれて動いてしまう」分(実測5〜9度)を
    # 見越す補正。既定0.0=無効。
    et_rally_turn_lag_compensation_deg: Optional[float] = None
    # DD-22較正用: Trueなら上の固定値の代わりに、実測の引き込まれ量から毎回
    # 更新していく推定値(指数移動平均)を使う。固定値では追従できない旋回ごとの
    # ばらつき・power依存を、実行中に自動で合わせ込む狙い。既定False。
    et_rally_turn_adaptive_lag_compensation: Optional[bool] = None
    # DD-22較正用: 左輪の目標エンコーダ角度に掛ける係数。片輪エンコーダテストで実測した、
    # 右輪の方が同じ回転量に対して約0.9%多く実際の方位を変える(実効径・グリップの
    # 定常的な左右差)分を補正する。lag_compensationとは別の、物理的な車輪特性による
    # 差で、どちらが先に止まるかとは無関係。既定1.0=無効。
    et_rally_turn_left_target_scale: Optional[float] = None
    # 2026-10-28: one_shotのphase1の目標エンコーダ角度に掛ける係数
    # (headingが増える向きの旋回, 減る向きの旋回)。既定None=(1.0, 1.0)で無効。
    et_rally_turn_phase1_scale: Optional[tuple] = None
    # Falseでキャスター引きずり補正(DD-15)・横ズレ補正(DD-16)を両方無効化する
    # (較正値そのものは変えない)。2026-10-28: どちらも旧い旋回方式での仮説・実測に
    # もとづく補正で、現在の旋回(one_shot/spin_encoder)では効果を確認できないため、
    # 既定を無効にした(ユーザー判断)。有効にするには--rally-compensation on。
    et_rally_compensation_enabled: bool = False
    # Falseでジャイロ倍率補正(DD-17、GYRO_SCALE_FACTOR)とタイヤ径較正(ET_RALLY_TIRE_DIAMETER)を
    # 両方無効化する(ベースライン計測用。phases/et_rally.pyのEnableGyroScale/SetTireDiameterを省略する)。
    et_rally_gyro_tire_calibration_enabled: bool = True
    enable_et_sumo: bool = True
    enable_finish: bool = True
    # 8/20 goal reference: blue marker -> 800mm. Verify on the real course.
    garage_goal_distance_mm: float = 800.0
    garage_line_target_v: int = 75
    garage_blue_timeout_sec: float = 30.0
    garage_straight_timeout_sec: float = 20.0
    # 直接TCP接続用。SSHポート転送だけならhostを127.0.0.1へ変更する。
    strategy_host: str = "0.0.0.0"
    strategy_port: int = 50000
    strategy_timeout_s: float = 20.0
    sumo: SumoSettings = SumoSettings()
    integration: IntegrationSettings = IntegrationSettings()


INTEGRATION_MISSIONS = ('at-to', 'to-bottle', 'at-to-bottle',
                        'bottle-rally', 'rally-sumo', 'sumo-garage')
MANUAL_RALLY_MISSIONS = ('rally-drive', 'bottle-rally', 'rally-sumo')


MISSION_CHOICES = (
    *INTEGRATION_MISSIONS,
    'at',
    'to',
    'configured',
    'lap',
    'bottle',
    'bottle-final',
    'rally',
    'rally-drive',
    'sumo',
    'finish',
    'full',
    'hint2',
    'hint2-return',
    'et-rally-turn-calibration',
    'et-rally-marker-observe',
    'et-rally-turn-phase2-diagnostic',
    'et-rally-single-wheel-diagnostic',
    'et-rally-single-wheel-reversal-diagnostic',
    'et-rally-single-wheel-alternating-diagnostic',
    'et-rally-single-wheel-alternating-right-diagnostic',
    'et-rally-single-wheel-alternating-left-diagnostic',
)


def config_for_mission(mission: str, base: RaceConfig = None) -> RaceConfig:
    # コマンド指定時だけ対象工程を有効にし、設定ファイルを書き換えず単体実行できるようにする。
    config = RaceConfig() if base is None else base
    if mission not in MISSION_CHOICES:
        raise ValueError('Unknown mission: ' + mission)
    if mission == 'configured':
        return replace(config, mission_mode='configured')
    if mission in ('hint2', 'hint2-return'):
        return replace(config, mission_mode=mission)

    disabled = dict(
        lapgate=False,
        enable_bottle_delivery=False,
        enable_et_rally=False,
        et_rally_laps=0,
        enable_et_sumo=False,
        enable_finish=False,
    )
    if mission in INTEGRATION_MISSIONS:
        if mission in ('bottle-rally', 'rally-sumo'):
            disabled['enable_et_rally'] = True
            disabled['et_rally_laps'] = max(1, config.et_rally_laps)
        if mission in ('rally-sumo', 'sumo-garage'):
            disabled['enable_et_sumo'] = True
        if mission == 'sumo-garage':
            disabled['enable_finish'] = True
        return replace(config, mission_mode=mission, **disabled)
    if mission in ('at', 'to'):
        return replace(config, mission_mode=mission, **disabled)
    if mission == 'bottle-final':
        # Hint2後移動の終了位置から、Bottle Delivery後半だけを単体実行する。
        return replace(config, mission_mode=mission, **disabled)
    if mission == 'et-rally-turn-calibration':
        # ETラリー旋回の応答特性(LQI・一発パルス仕上げ共通)実測データ収集専用。
        # 準備工程・PC通信は一切実行しない。
        return replace(config, mission_mode=mission, **disabled)
    if mission == 'et-rally-marker-observe':
        # マーカー検出の実機確認専用(観測のみ、走行体は動かさない)。
        # EXPECTED_MARKER_ANGLE_DEG/ANGLE_TO_HEADING_GAINの較正前に、
        # まず検出そのものが安定するかを確認するためのミッション。
        return replace(config, mission_mode=mission, **disabled)
    if mission == 'et-rally-turn-phase2-diagnostic':
        # 旋回のB方向横ズレが、フェーズ2(仕上げ補正)由来かどうかを切り分ける専用。
        # plan_spin_alternating30.json(--rally-plan-path、通常設定)と比較して使う。
        return replace(config, mission_mode=mission, **disabled)
    if mission == 'et-rally-single-wheel-diagnostic':
        # 片輪だけをエンコーダで回し、同じ回転量に対する実際の方位変化量(=実効的な
        # 移動量)が左右で違うかを直接確認する専用(DD-22関連)。
        return replace(config, mission_mode=mission, **disabled)
    if mission == 'et-rally-single-wheel-reversal-diagnostic':
        # 片輪だけを順方向→逆方向と反転させ、反転直後の1周だけ実際の方位変化が
        # 小さくならないか(反転コスト)を、左右で比較する専用(DD-22関連)。
        return replace(config, mission_mode=mission, **disabled)
    if mission == 'et-rally-single-wheel-alternating-diagnostic':
        # 片輪だけを1周ごとに反転させ続け、反転コストのサンプル数を増やして
        # 左右差をより確実に見る専用(DD-22関連)。
        return replace(config, mission_mode=mission, **disabled)
    if mission in ('et-rally-single-wheel-alternating-right-diagnostic',
                   'et-rally-single-wheel-alternating-left-diagnostic'):
        # 上記の右輪だけ/左輪だけ単独版。2つの間に、実機の位置ズレを物理的に
        # 測る間を空けられるよう、別ミッションとして分けている(DD-22関連)。
        return replace(config, mission_mode=mission, **disabled)
    if mission == 'rally-drive':
        # 復号済みHintを外部入力し、準備工程なしでPC受信と周回走行だけを試す。
        disabled['enable_et_rally'] = True
        disabled['et_rally_laps'] = max(1, config.et_rally_laps)
        return replace(config, mission_mode=mission, **disabled)
    if mission == 'lap':
        disabled['lapgate'] = True
    elif mission == 'bottle':
        disabled['enable_bottle_delivery'] = True
    elif mission == 'rally':
        disabled['enable_et_rally'] = True
        # 明示的なrally指定では、設定が0でも最低1周はツリーへ含める。
        disabled['et_rally_laps'] = max(1, config.et_rally_laps)
    elif mission == 'sumo':
        disabled['enable_et_sumo'] = True
    elif mission == 'finish':
        disabled['enable_finish'] = True
    elif mission == 'full':
        disabled.update(
            lapgate=True,
            enable_bottle_delivery=True,
            enable_et_rally=True,
            et_rally_laps=max(1, config.et_rally_laps),
            enable_et_sumo=True,
            enable_finish=True,
        )
    return replace(config, mission_mode='configured', **disabled)


def mission_requires_qr(config: RaceConfig) -> bool:
    # Hint読取を含まない単体工程ではQRデコーダーを起動条件にしない。
    if config.mission_mode in ('bottle-rally', 'rally-sumo', 'sumo-garage'):
        return False
    if config.mission_mode in INTEGRATION_MISSIONS:
        return True
    if config.mission_mode == 'rally-drive':
        return False
    return (
        config.mission_mode in ('to', 'hint2', 'hint2-return')
        or config.enable_et_rally
        or config.enable_bottle_delivery
    )


def mission_requires_camera(config: RaceConfig) -> bool:
    # ET相撲は黒テープ付き力士ボトルの捕捉にカメラを使用する。
    if config.mission_mode == 'rally-drive':
        return False
    if config.mission_mode in INTEGRATION_MISSIONS:
        return True
    if config.mission_mode in ('at', 'to', 'hint2', 'hint2-return'):
        return True
    return any(
        (
            config.lapgate,
            config.enable_bottle_delivery,
            config.enable_et_rally,
            config.enable_et_sumo,
            config.enable_finish,
        )
    )
