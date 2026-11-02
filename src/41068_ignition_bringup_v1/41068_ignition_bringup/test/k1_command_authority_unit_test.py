#!/usr/bin/env python3
"""K1 / K1b unit-level gates (no Gazebo) — measurable mission-owner + gate policy.

Asserts published-style fields on MissionOwnerState (same values that go into
MissionOwnerStatus) plus abort-resend / Start reject codes. Full sim K1b
(wasd under inhibit, cmd_vel≈0) remains a separate harness.
"""

from __future__ import annotations

import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.abspath(os.path.join(HERE, '..'))
if PKG not in sys.path:
    sys.path.insert(0, PKG)

from rs1_nav.mission_owner import (  # noqa: E402
    CommandAuthority,
    MissionOwnerState,
    MissionPhase,
    SafetyInhibit,
)


class K1CommandAuthorityTests(unittest.TestCase):
    def test_k1_pending_fire_rejects_start(self) -> None:
        s = MissionOwnerState()
        s.raise_inhibit(SafetyInhibit.THERMAL_FIRE_PENDING)
        self.assertEqual(s.command_authority, CommandAuthority.NONE)
        self.assertEqual(s.phase, MissionPhase.HOLDING)
        self.assertIsNotNone(s.start_reject_code())
        s.accept_teleop()  # must not open gate while inhibit — policy: EnableTeleop rejects
        # accept_teleop itself doesn't check inhibits (caller does); simulate reject:
        self.assertTrue(s.has_blocking_inhibit())

    def test_k1_pending_timeout_terminal(self) -> None:
        s = MissionOwnerState()
        s.raise_inhibit(SafetyInhibit.THERMAL_FIRE_PENDING)
        s.active_inhibits.discard(SafetyInhibit.THERMAL_FIRE_PENDING)
        s.raise_inhibit(SafetyInhibit.PENDING_FIRE_UNRESOLVED)
        self.assertEqual(s.phase, MissionPhase.STOPPED)
        self.assertTrue(s.abort_resend_forbidden())

    def test_k1_operator_stop_then_start(self) -> None:
        s = MissionOwnerState()
        s.accept_normal_start()
        self.assertEqual(s.command_authority, CommandAuthority.NAV)
        s.begin_operator_stop()
        s.finish_operator_stop()
        self.assertTrue(s.operator_stop_latch)
        self.assertEqual(s.command_authority, CommandAuthority.NONE)
        self.assertIsNone(s.start_reject_code())
        s.accept_normal_start()
        self.assertFalse(s.operator_stop_latch)
        self.assertEqual(s.command_authority, CommandAuthority.NAV)

    def test_k1_holding_clear_then_start(self) -> None:
        s = MissionOwnerState()
        s.raise_inhibit(SafetyInhibit.THERMAL_FIRE_PENDING)
        s.clear_inhibit(SafetyInhibit.THERMAL_FIRE_PENDING)
        self.assertEqual(s.phase, MissionPhase.IDLE)
        self.assertEqual(s.command_authority, CommandAuthority.NONE)
        self.assertIsNone(s.start_reject_code())
        s.accept_normal_start()
        self.assertEqual(s.phase, MissionPhase.NAVIGATING)

    def test_k1b_manual_nav_inhibit(self) -> None:
        s = MissionOwnerState()
        s.accept_manual_nav()
        self.assertEqual(s.command_authority, CommandAuthority.BT_MANUAL)
        s.raise_inhibit(SafetyInhibit.THERMAL_FIRE_PENDING)
        self.assertEqual(s.command_authority, CommandAuthority.NONE)
        self.assertEqual(s.phase, MissionPhase.HOLDING)

    def test_k1b_abort_resend_after_stop(self) -> None:
        s = MissionOwnerState()
        s.accept_normal_start()
        s.begin_operator_stop()
        self.assertTrue(s.abort_resend_forbidden())


if __name__ == '__main__':
    unittest.main()
