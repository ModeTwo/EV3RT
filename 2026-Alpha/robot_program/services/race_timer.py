"""Feature 14 race timer interface."""

from typing import Optional


class RaceTimer:
    # No.14は直列工程ではないためBTノードにせず、全工程から参照できるサービスとする。
    def __init__(self) -> None:
        self.started_at: Optional[float] = None

    def start(self) -> None:
        # 実際の単調増加時計による計測は担当者が実装する。
        pass

    def elapsed_ms(self) -> int:
        # 未実装期間は0を返し、他機能の単体開発を止めない。
        return 0

