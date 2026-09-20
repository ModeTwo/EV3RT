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
    start_lap_power: int = 80
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
    et_rally_strategy_source: str = "file"
    # Noneならtests/plan_seed9392783.json。相対パスは2026-Alpha直下を基準にする。
    et_rally_plan_path: Optional[str] = None
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
    strategy_timeout_s: float = 5.0
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
