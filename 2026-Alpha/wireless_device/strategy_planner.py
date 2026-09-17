"""Stable PC-side facade for replaceable ET rally route planners."""

from pathlib import Path
from typing import Any, Dict, List, Optional, Union

from .planner_process import PlannerProcess


class StrategyPlanner:
    """Translate the communication payload into planner input and return steps."""

    def __init__(
            self, runner_path: Optional[Union[str, Path]] = None) -> None:
        # 通信側は計算本体の配置やimport構造を知らず、ランナー契約だけに依存する。
        self.process = PlannerProcess(runner_path)

    @property
    def runner_path(self) -> Path:
        return self.process.runner_path

    def build(self, hints: Dict[str, Any]) -> List[Dict[str, Any]]:
        return self.process.calculate(hints)
