"""LAP tuning and experiment options, shared by production and trials."""
import math
import os

def decision_offset_mm():
    value = os.environ.get('LAP_DECISION_OFFSET_MM', '145.21')
    if value not in ('100', '145.21'):
        raise ValueError('LAP_DECISION_OFFSET_MM must be 100 or 145.21')
    return float(value)

def dark_entry_edge_mode():
    mode = os.environ.get('LAP_DARK_ENTRY_EDGE', 'off').lower()
    if mode not in ('on', 'off'):
        raise ValueError('LAP_DARK_ENTRY_EDGE must be on or off')
    return mode


def blue_unconfirmed_limit_mm():
    """Maximum projected distance after a blue candidate without confirmed black."""
    value = os.environ.get('LAP_BLUE_UNCONFIRMED_LIMIT_MM', '350')
    if value not in ('300', '350', '450'):
        raise ValueError('LAP_BLUE_UNCONFIRMED_LIMIT_MM must be 300, 350 or 450')
    return float(value)


def far_peak_deg():
    value = os.environ.get('LAP_FAR_PEAK_DEG', '10')
    if value not in ('10', '12'):
        raise ValueError('LAP_FAR_PEAK_DEG must be 10 or 12')
    return float(value)


def far_entry_policy():
    mode = os.environ.get('LAP_FAR_ENTRY_POLICY', 'evidence').lower()
    if mode not in ('fixed60', 'evidence'):
        raise ValueError('LAP_FAR_ENTRY_POLICY must be fixed60 or evidence')
    return mode


def lap_camera_reference_x():
    value = float(os.environ.get('LAP_CAMERA_REFERENCE_X', '160'))
    if not math.isfinite(value) or not 0 <= value < 320:
        raise ValueError('LAP_CAMERA_REFERENCE_X must be between 0 and 319')
    return value


TRIAL_VARIANTS={
    'A': {'reverse_side':False, 'line_scale':1.0, 'heading_gain':2.0},
    'B': {'reverse_side':True,  'line_scale':1.0, 'heading_gain':2.0},
    'C': {'reverse_side':False, 'line_scale':0.5, 'heading_gain':2.0},
    'D': {'reverse_side':False, 'line_scale':1.0, 'heading_gain':1.0},
    'E': {'reverse_side':False, 'line_scale':1.0, 'heading_gain':2.0, 'turn_limit':20.0},
    'F': {'reverse_side':False, 'line_scale':1.0, 'heading_gain':2.0, 'turn_limit':30.0},
    'G': {'reverse_side':True, 'line_scale':1.0, 'heading_gain':2.0, 'turn_limit':20.0},
}


def approach_mode():
    mode=os.environ.get('LAP_APPROACH_MODE','baseline')
    if mode not in ('baseline','heading_strong','line_half'):
        raise ValueError('LAP_APPROACH_MODE must be baseline, heading_strong or line_half')
    return mode


def post_gate_steer_mode():
    mode = os.environ.get('LAP_POST_GATE_STEER', 'baseline')
    if mode not in ('baseline', 'gyro'):
        raise ValueError('LAP_POST_GATE_STEER must be baseline or gyro')
    return mode


def line_entry_mode():
    mode = os.environ.get('LAP_LINE_ENTRY', 'baseline')
    if mode not in ('baseline', 'cross', 'cross_soft', 'cross_rejoin'):
        raise ValueError('LAP_LINE_ENTRY must be baseline, cross, cross_soft or cross_rejoin')
    return mode


def gate_recovery_mode():
    mode = os.environ.get('LAP_GATE_RECOVERY', 'baseline')
    if mode not in ('baseline', 'floor'):
        raise ValueError('LAP_GATE_RECOVERY must be baseline or floor')
    return mode


def gate_policy():
    policy = os.environ.get('LAP_GATE_POLICY', 'baseline')
    if policy not in ('baseline', 'slow', 'weak', 'combined'):
        raise ValueError('LAP_GATE_POLICY must be baseline, slow, weak or combined')
    return policy


def confirmed_slow_distance():
    value = os.environ.get('LAP_GATE_SLOW_AFTER_MM', '150')
    if value not in ('150', '200'):
        raise ValueError('LAP_GATE_SLOW_AFTER_MM must be 150 or 200')
    return float(value)


def gate_stop_measure_enabled():
    value = os.environ.get('LAP_GATE_STOP_MEASURE', '0')
    if value not in ('0', '1'):
        raise ValueError('LAP_GATE_STOP_MEASURE must be 0 or 1')
    return value == '1'


def trace_target_v():
    value=os.environ.get('LAP_TRACE_TARGET_V','40')
    if value not in ('75','50','40'):raise ValueError('LAP_TRACE_TARGET_V must be 75, 50 or 40')
    return int(value)


def entry_heading_limit():
    value=os.environ.get('LAP_ENTRY_HEADING_DEG','14')
    if value not in ('12','14'):raise ValueError('LAP_ENTRY_HEADING_DEG must be 12 or 14')
    return float(value)


def merge_peak():
    value=os.environ.get('LAP_MERGE_PEAK_DEG','10')
    if value not in ('8','10','12'):raise ValueError('LAP_MERGE_PEAK_DEG must be 8, 10 or 12')
    return float(value)


def merge_deadband():
    value=os.environ.get('LAP_MERGE_DEADBAND_PX','6')
    if value not in ('6','12','20'):raise ValueError('LAP_MERGE_DEADBAND_PX must be 6, 12 or 20')
    return float(value)


def merge_length():
    value=os.environ.get('LAP_MERGE_LENGTH_MM','600')
    if value not in ('600','800'):raise ValueError('LAP_MERGE_LENGTH_MM must be 600 or 800')
    return float(value)


def trial_variant():
    name=os.environ.get('LAP_TRIAL_VARIANT','E').upper()
    if name not in TRIAL_VARIANTS:raise ValueError('LAP_TRIAL_VARIANT must be A, B, C, D, E, F or G')
    return name,TRIAL_VARIANTS[name]


