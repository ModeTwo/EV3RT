"""Pure data conversion for a labelled stationary ultrasonic measurement.

距離の生値分類・統計・CSV項目。正の数値分類と物理的な捕捉成功を区別する。"""

import math
import statistics

SAMPLE_FIELDS = (
    "repeat",
    "interactive_session_id",
    "sample_id",
    "read_started_monotonic_sec",
    "read_ended_monotonic_sec",
    "read_duration_ms",
    "utc",
    "label",
    "forward_mm",
    "lateral_mm",
    "notes",
    "raw_value",
    "raw_repr",
    "raw_status",
    "numeric_mm",
    "legacy_window_match",
    "same_as_previous_read",
    "same_value_run_reads",
    "gyro_deg",
    "odo_mm",
    "left_count",
    "right_count",
    "arm_count",
)
FRAME_FIELDS = (
    "probe_repeat",
    "probe_interactive_session_id",
    "probe_sample_id",
    "probe_read_started_monotonic_sec",
    "probe_read_ended_monotonic_sec",
    "probe_raw_value",
    "probe_raw_repr",
    "probe_raw_status",
    "probe_numeric_mm",
    "probe_label",
    "probe_forward_mm",
    "probe_lateral_mm",
    "probe_notes",
    "probe_sample_age_at_telemetry_sec",
)


def decode_distance(raw, scale, minimum, maximum, error=None):
    result = dict(
        raw_value=None,
        raw_repr=repr(raw),
        raw_status="none",
        numeric_mm=None,
        legacy_window_match=False,
    )
    if error is not None:
        result.update(raw_status="read_error", raw_repr=str(error))
        return result
    if raw is None:
        return result
    if isinstance(raw, bool) or not isinstance(raw, (int, float)):
        result["raw_status"] = "non_numeric"
        return result
    value = float(raw)
    if not math.isfinite(value):
        result["raw_status"] = "non_finite"
        return result
    result["raw_value"] = value
    mm = value * scale
    if not math.isfinite(mm):
        result["raw_status"] = "conversion_non_finite"
        return result
    result.update(
        numeric_mm=mm,
        raw_status="positive_numeric" if value > 0 else "non_positive",
        legacy_window_match=value > 0 and minimum <= mm <= maximum,
    )
    return result


def numeric_stats(values):
    values = list(values)
    if values:
        return dict(
            n=len(values),
            minimum=min(values),
            maximum=max(values),
            median=statistics.median(values),
            mean=statistics.mean(values),
        )
    else:
        return dict(n=0)
