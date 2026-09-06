"""Race feature switches used by the robot-side tree builder."""

from dataclasses import dataclass, replace

from .sumo_types import SumoSettings
from .integration_settings import IntegrationSettings


@dataclass(frozen=True)
class RaceConfig:
    # 試験時に工程単位で有効・無効を切り替えられるよう、設定値を一か所へ集約する。
    # configuredでは以下の工程フラグをそのまま使用する。
    # hint2系はRE→AT→TO接続試験を残すための専用モード。
    mission_mode: str = 'configured'
    lapgate : bool = True
    enable_bottle_delivery: bool = True
    enable_et_rally: bool = True
    et_rally_laps: int = 3
    enable_et_sumo: bool = True
    enable_finish: bool = True
    sumo: SumoSettings = SumoSettings()
    integration: IntegrationSettings = IntegrationSettings()


MISSION_CHOICES = (
    'configured',
    'lap',
    'bottle',
    'rally',
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
    return config.mission_mode in ('hint2', 'hint2-return') or config.enable_et_rally
