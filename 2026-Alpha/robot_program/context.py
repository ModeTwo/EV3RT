"""Shared state passed between robot-side features."""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from .services.race_timer import RaceTimer
from .sumo_types import SumoState
from .integration_settings import HandoffState
from .delivery_heading import DeliveryHeadingReference


@dataclass
class RaceContext:
    # 各担当機能は他担当のモジュールを直接参照せず、この共有状態を介して情報を渡す。
    bottle_color: Optional[str] = None
    # 配置先は認識色から一度だけ決定し、配置後の開始位置復帰にも同じ値を使う。
    selected_drop_zone: Optional[str] = None
    bottle_delivered: bool = False
    rally_ready: bool = False
    decryption_key: Optional[str] = None
    hint1: Optional[str] = None
    hint2: Optional[str] = None
    hint2_gate_info: Optional[str] = None
    strategy: List[Dict[str, Any]] = field(default_factory=list)
    # PCが決定した周回数。strategyにはこの周回数分の全走行指令が入る。
    selected_rally_laps: Optional[int] = None
    # 通信スレッドは直接書き換えず、BT周期側で受信キューから反映する。
    mission_id: Optional[int] = None
    strategy_status: str = "idle"
    strategy_error: Optional[str] = None
    rally_lap: int = 0
    timer: RaceTimer = field(default_factory=RaceTimer)
    sumo: SumoState = field(default_factory=SumoState)
    at_to: HandoffState = field(default_factory=HandoffState)

    delivery_heading: DeliveryHeadingReference = field(default_factory=DeliveryHeadingReference)
