"""Feature 01 password input boundary."""

from typing import Optional


class PasswordInput:
    # No.1は走行体ではなく、PC側アプリケーションが所有する入力機能とする。
    def __init__(self) -> None:
        self.value: Optional[str] = None

    def capture(self) -> Optional[str]:
        # 画面、確認、再入力の具体処理は未実装とする。
        return self.value

