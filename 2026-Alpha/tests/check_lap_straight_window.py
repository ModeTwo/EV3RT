"""Derivation follows the calibrated profile and refuses ambiguous endings."""
from robot_program.heading_profile import HeadingProfile
from robot_program.start_lap_calibration import calibrated_profile
from robot_program.start_lap_profile_v1 import POINTS, BLUE_START_MM, LAP_GATE_MM
from robot_program.features.lap_straight_window import final_zero_window

profile, _, _ = calibrated_profile(POINTS, BLUE_START_MM, LAP_GATE_MM,
                                   first_straight_mm=600.0, route_scale=1.0)
window = final_zero_window(profile.heading_at, 5290.540)
assert window is not None
assert abs(window[0] - 4304.79) < .01
assert abs(window[1] - 4350.) < .01
assert abs(window[2] - 4450.) < .01
assert abs(window[3] - 4650.) < .01
early = final_zero_window(profile.heading_at, 5290.540, decision_offset_mm=100.)
assert abs(early[2] - early[0] - 100.) < .01
assert early[0:2] == window[0:2] and early[3] == window[3]

shifted, _, _ = calibrated_profile(POINTS, BLUE_START_MM, LAP_GATE_MM,
                                   first_straight_mm=700.0, route_scale=1.0)
shifted_window = final_zero_window(shifted.heading_at, 5390.540)
assert all(abs(b - a - 100.) < .01 for a, b in zip(window, shifted_window))
assert final_zero_window(shifted.heading_at, 5290.540) == shifted_window

scaled, _, _ = calibrated_profile(POINTS, BLUE_START_MM, LAP_GATE_MM,
                                  first_straight_mm=600.0, route_scale=1.05)
scaled_window = final_zero_window(scaled.heading_at, 5590.)
assert scaled_window[0] > window[0]
assert abs(scaled_window[2] - scaled_window[0] - 145.21) < .01

nonzero = HeadingProfile([(0, 0), (100, 10), (500, 8)])
assert final_zero_window(nonzero.heading_at, 500) is None
isolated_zero = HeadingProfile([(0, 0), (100, 5), (200, 0)])
assert final_zero_window(isolated_zero.heading_at, 200) is None
too_short = HeadingProfile([(0, 0), (100, 5), (200, 0), (300, 0)])
assert final_zero_window(too_short.heading_at, 300) is None
assert final_zero_window(lambda _: 0., 1000.) is None
print('PASS current and changed profiles, final zero interval, bounded fallback')
