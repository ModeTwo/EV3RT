"""Shared state passed between robot-side features."""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from .services.race_timer import RaceTimer
from .sumo_types import SumoState
from .integration_settings import HandoffState


@dataclass
class RaceContext:
    # 各担当機能は他担当のモジュールを直接参照せず、この共有状態を介して情報を渡す。
    bottle_color: Optional[str] = None
    hint1: Optional[str] = None
    hint2: Optional[str] = None
    strategy: List[Dict[str, Any]] = field(default_factory=list)
    rally_lap: int = 0
    timer: RaceTimer = field(default_factory=RaceTimer)
    sumo: SumoState = field(default_factory=SumoState)
    at_to: HandoffState = field(default_factory=HandoffState)
