"""Check logged odometry uses the counts already read for the real calculation."""
import contextlib
import io
import importlib.util
import math
from pathlib import Path
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('odometry_plotter', Path(__file__).resolve().parents[1] / 'py_etrobo_util' / 'plotter.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
Plotter = module.Plotter


class Motor:
    def __init__(self):
        self.value = 0
        self.reads = 0

    def reset_count(self):
        self.value = 0

    def get_count(self):
        self.reads += 1
        return self.value


class Gyro:
    def reset(self):
        pass

    def get_angle(self):
        return 0


def main():
    right, left, gyro = Motor(), Motor(), Gyro()
    plotter = Plotter()
    output = io.StringIO()
    with patch.object(module.time, 'monotonic', side_effect=[10.0, 11.0, 12.0]), contextlib.redirect_stdout(output):
        plotter.plot(None, None, right, left, None, None, None, gyro)
        right.value, left.value = 360, 180
        plotter.plot(None, None, right, left, None, None, None, gyro)
        plotter.plot(None, None, right, left, None, None, None, gyro)
    assert right.reads == left.reads == 3, 'diagnostics must not add motor reads'
    assert math.isclose(plotter.distance, math.pi * 57.33 * 0.75)
    lines = output.getvalue().splitlines()
    assert len(lines) == 3
    assert 'right_rpm=60.00 left_rpm=30.00' in lines[1]
    assert 'right_unchanged_sec=1.000 left_unchanged_sec=1.000' in lines[2]
    assert 'right_rpm=0.00 left_rpm=0.00' in lines[2]
    print('PASS existing encoder reads, odometry, RPM and frozen-value timing')


if __name__ == '__main__':
    main()
