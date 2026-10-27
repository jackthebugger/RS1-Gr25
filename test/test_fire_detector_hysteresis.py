"""Unit tests for fire_detector consecutive-frame hysteresis (Entry 025)."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(
    0,
    str(
        Path(__file__).resolve().parents[1]
        / 'src'
        / 'beer_fire_detection'
    ),
)

from beer_fire_detection.fire_detector import update_fire_hysteresis  # noqa: E402


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
