import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / '41068_ignition_bringup_v1' / '41068_ignition_bringup' / 'scripts'))

import robot_status_gui as gui


def test_format_speed_rounds_cleanly():
    assert gui.format_speed(1.234) == '1.23 m/s'
    assert gui.format_speed(0.0) == '0.00 m/s'


def test_detect_obstacle_uses_threshold():
    assert gui.detect_obstacle([3.0, 3.0, 0.25, 3.0], threshold=0.5) is True
    assert gui.detect_obstacle([3.0, 2.0, 1.5, 3.0], threshold=0.5) is False
    assert gui.detect_obstacle([], threshold=0.5) is False
