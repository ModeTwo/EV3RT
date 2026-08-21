"""PC-side object that owns password input and strategy generation."""

from typing import Any, Dict, List

from .password_input import PasswordInput
from .strategy_planner import StrategyPlanner


class WirelessDeviceApplication:
    # パスワード入力と走行指示SEQ生成は、同じPC側物体の責務としてまとめる。
    # 走行体側Behavior Treeへパスワード入力ノードは配置しない。
    def __init__(self) -> None:
        self.password_input = PasswordInput()
        self.strategy_planner = StrategyPlanner()

    def prepare_strategy(self, hints: Dict[str, Any]) -> List[Dict[str, Any]]:
        # 入力とSEQ生成の呼出し順だけを示し、具体的な処理は各専用クラスへ委譲する。
        password = self.password_input.capture()
        return self.strategy_planner.build(password=password, hints=hints)

