"""One-sided floor correction for LAP straight-line recovery."""

def white_side_turn(h, s, v, course, normal_side):
    """Black: heading only; low-saturation bright floor: steer toward trace edge.

    V=50..80 is a provisional interpolation, not a calibrated floor boundary.
    Saturated colours are left to the gate detector, not treated as white.
    """
    if course not in (-1, 1) or not all(0 <= n <= 100 for n in (s, v)):
        raise ValueError('invalid floor observation or course')
    if s > 45:
        return 0.0
    strength = max(0.0, min(1.0, (v - 50.0) / 30.0))
    return (course if normal_side else -course) * 20.0 * strength
