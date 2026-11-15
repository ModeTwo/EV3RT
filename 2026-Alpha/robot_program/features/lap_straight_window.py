"""Derive the final zero-heading observation window from a distance profile."""
import math

ZERO_TOLERANCE_DEG = 1e-6
# Preserve the previous 4350/4450/4650 mm timings on the current profile.
OBSERVATION_OFFSET_MM = 45.21
DECISION_OFFSET_MM = 145.21
DECISION_WINDOW_MM = 200.0


def final_zero_window(target, limit_mm, *, decision_offset_mm=DECISION_OFFSET_MM):
    """Return (straight start, observation start, decision start, deadline).

    A bound HeadingProfile.heading_at method supplies calibrated points. Do not
    infer a straight from samples of an arbitrary callable or an isolated zero.
    """
    profile = getattr(target, '__self__', None)
    points = getattr(profile, 'points', None)
    if not points or not callable(getattr(profile, 'heading_at', None)):
        return None
    limit = float(limit_mm)
    if not math.isfinite(limit) or limit <= 0:
        return None
    clipped = [(float(s), float(h)) for s, h in points if 0 <= s < limit]
    clipped.append((limit, float(profile.heading_at(limit))))
    if len(clipped) < 2 or abs(clipped[-1][1]) > ZERO_TOLERANCE_DEG:
        return None
    i = len(clipped) - 1
    while i > 0 and abs(clipped[i - 1][1]) <= ZERO_TOLERANCE_DEG:
        i -= 1
    if i == len(clipped) - 1:
        return None
    straight_start = clipped[i][0]
    observation = straight_start + OBSERVATION_OFFSET_MM
    decision = straight_start + float(decision_offset_mm)
    # Keep the original deadline so an early trial does not shorten recovery.
    deadline = min(straight_start + DECISION_OFFSET_MM + DECISION_WINDOW_MM, limit)
    if not straight_start <= observation < decision < deadline <= limit:
        return None
    return straight_start, observation, decision, deadline
