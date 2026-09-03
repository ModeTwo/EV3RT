"""Race feature switches used by the robot-side tree builder."""

from dataclasses import dataclass


@dataclass(frozen=True)
class RaceConfig:
    # 試験時に工程単位で有効・無効を切り替えられるよう、設定値を一か所へ集約する。
    lapgate : bool = False
    enable_bottle_delivery: bool = False
    enable_et_rally: bool = True
    et_rally_laps: int = 3
    enable_et_sumo: bool = False
    enable_finish: bool = False

