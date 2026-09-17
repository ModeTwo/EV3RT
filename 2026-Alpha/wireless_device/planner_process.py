"""Stable process boundary for replaceable route calculation programs."""

import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Union


RUNNER_ENV_NAME = "ET_RALLY_PLANNER_RUNNER"
PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_RUNNER = Path(__file__).resolve().with_name("et_rally_runner.py")


def resolve_runner_path(
        configured_path: Optional[Union[str, Path]] = None) -> Path:
    """Resolve a configured runner without depending on the launch directory."""
    # コマンドライン指定、環境変数、標準ランナーの順で採用する。
    raw_path = configured_path or os.environ.get(RUNNER_ENV_NAME)
    if raw_path:
        path = Path(raw_path).expanduser()
        # 相対パスは2026-Alphaを基準にし、起動ディレクトリの違いを排除する。
        if not path.is_absolute():
            path = PROJECT_ROOT / path
    else:
        path = DEFAULT_RUNNER
    path = path.resolve()
    if not path.is_file():
        raise FileNotFoundError("ET rally planner runner not found: " + str(path))
    return path


class PlannerProcess:
    """Execute one calculation through the stable JSON stdin/stdout contract."""

    def __init__(self, runner_path: Optional[Union[str, Path]] = None) -> None:
        self.runner_path = resolve_runner_path(runner_path)

    def calculate(self, payload: Dict[str, Any]) -> List[Dict[str, Any]]:
        # 計算ごとにランナーを起動するため、開発中にファイルを差し替えれば
        # PC通信プログラムを変更せず次回計算から新しい実装が使われる。
        completed = subprocess.run(
            [sys.executable, str(self.runner_path)],
            input=json.dumps(payload, ensure_ascii=True, allow_nan=False),
            capture_output=True,
            text=True,
            encoding="utf-8",
            env={**os.environ, "PYTHONIOENCODING": "utf-8"},
            check=False,
        )
        if completed.returncode != 0:
            detail = completed.stderr.strip().splitlines()
            message = detail[-1] if detail else "ET rally planner failed"
            raise RuntimeError(str(self.runner_path) + ": " + message)
        try:
            result = json.loads(completed.stdout)
        except json.JSONDecodeError as error:
            raise ValueError(
                "ET rally planner runner must write one JSON value to stdout: "
                + str(self.runner_path)
            ) from error
        if not isinstance(result, list):
            raise ValueError("ET rally planner must return a command list")
        return result
