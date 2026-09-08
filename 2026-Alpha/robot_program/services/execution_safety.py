"""Hardware-independent preflight and best-effort motor shutdown."""

from ..placeholder import PendingFeature


def pending_features(root):
    return [node.name for node in root.iterate() if isinstance(node, PendingFeature)]


def stop_motors(runtime):
    """Try every operation even if one motor or brake API fails."""
    errors = []
    for name in ('right_motor', 'left_motor', 'arm_motor'):
        motor = getattr(runtime, name, None)
        if motor is None:
            continue
        for method, value in (('set_power', 0), ('set_brake', True)):
            try:
                getattr(motor, method)(value)
            except Exception as error:
                errors.append(f'{name}.{method}: {error}')
    return errors
