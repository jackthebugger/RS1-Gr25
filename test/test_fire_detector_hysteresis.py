"""Unit tests for fire_detector consecutive-frame hysteresis (Entry 025)."""

from __future__ import annotations

import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

# Allow importing fire_detector without a sourced rs1_interfaces install.
if 'rs1_interfaces' not in sys.modules:
    _rs1 = types.ModuleType('rs1_interfaces')
    _rs1_msg = types.ModuleType('rs1_interfaces.msg')
    _rs1_msg.FireObservation = MagicMock(name='FireObservation')
    _rs1.msg = _rs1_msg
    sys.modules['rs1_interfaces'] = _rs1
    sys.modules['rs1_interfaces.msg'] = _rs1_msg

sys.path.insert(
    0,
    str(
        Path(__file__).resolve().parents[1]
        / 'src'
        / 'beer_fire_detection'
    ),
)

from beer_fire_detection.fire_detector import (  # noqa: E402
    hot_blob_bearing,
    update_fire_hysteresis,
)


def test_requires_positive_frames_before_true():
    state, pos, neg = False, 0, 0
    for _ in range(2):
        state, pos, neg = update_fire_hysteresis(True, state, pos, neg, 3, 5)
        assert state is False
    state, pos, neg = update_fire_hysteresis(True, state, pos, neg, 3, 5)
    assert state is True
    assert pos == 3


def test_requires_negative_frames_before_false():
    state, pos, neg = True, 0, 0
    for _ in range(4):
        state, pos, neg = update_fire_hysteresis(False, state, pos, neg, 3, 5)
        assert state is True
    state, pos, neg = update_fire_hysteresis(False, state, pos, neg, 3, 5)
    assert state is False
    assert neg == 5


def test_boundary_chatter_does_not_flap():
    """Alternating raw frames must not toggle published state."""
    state, pos, neg = False, 0, 0
    for i in range(20):
        state, pos, neg = update_fire_hysteresis(
            i % 2 == 0, state, pos, neg, 3, 5,
        )
    assert state is False


def test_hot_blob_bearing_centre_is_zero():
    bearing, half = hot_blob_bearing([160], 320, 1.047)
    assert abs(bearing) < 0.02
    assert half >= 0.0


def test_hot_blob_bearing_left_is_positive():
    bearing, _half = hot_blob_bearing([80], 320, 1.047)
    assert bearing > 0.2


def test_hot_blob_bearing_empty_is_none():
    assert hot_blob_bearing([], 320, 1.047) is None
