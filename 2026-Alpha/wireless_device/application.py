"""PC-side object that owns strategy generation."""

from pathlib import Path
from typing import Any, Dict, List
from typing import Optional, Union

from .strategy_planner import StrategyPlanner


class WirelessDeviceApplication:
    # 復号済みゲート情報を受け取り、走行指示SEQ生成だけを担当する。
    def __init__(
            self, planner_runner: Optional[Union[str, Path]] = None) -> None:
        self.strategy_planner = StrategyPlanner(runner_path=planner_runner)

    def prepare_strategy(self, hints: Dict[str, Any]) -> List[Dict[str, Any]]:
        return self.strategy_planner.build(hints=hints)
