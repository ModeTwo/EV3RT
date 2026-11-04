HSV thresholds centralization patch (limited scope)

Files changed:
- robot_program/color_thresholds.py (new)
- robot_program/features/new_et_sumo.py
- robot_program/vision/sumo_black_bottle.py

Existing numeric thresholds preserved. Other line and bottle detection modules
are NOT modified, because their current code/HSV scaling needs verification.

Apply at repository root:
  unzip -o hsv_centralization_patch.zip
  pypy3 -m py_compile robot_program/color_thresholds.py robot_program/features/new_et_sumo.py robot_program/vision/sumo_black_bottle.py

Caution: modifying thresholds may affect detection. Validate on hardware safely.
