"""Single control period for dispatch, all motion PID and control filters."""
import math

# 全工程共通。変更後はプログラムを再起動する。20ms = 50Hz。
CONTROL_INTERVAL_SEC = 0.02
if not math.isfinite(CONTROL_INTERVAL_SEC) or CONTROL_INTERVAL_SEC <= 0:
    raise ValueError('CONTROL_INTERVAL_SEC must be positive and finite')
