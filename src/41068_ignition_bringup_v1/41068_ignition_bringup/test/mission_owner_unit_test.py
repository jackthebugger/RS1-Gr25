#!/usr/bin/env python3
"""Unit tests for rs1_nav.mission_owner (no Gazebo).

Covers Phase Two Wave 1b NO_SAFE_ROUTE rearm and Wave 0 hang-ladder helpers.
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
    home_stuck_action,
    mission_stuck_action,
    path_blocked_none_timed_out,
    seed_timed_out,
)


class MissionOwnerTests(unittest.TestCase):
    def test_holding_clear_to_idle(self) -> None:
        s = MissionOwnerState()
        s.raise_inhibit(SafetyInhibit.THERMAL_FIRE_PENDING)
        self.assertEqual(s.phase, MissionPhase.HOLDING)
        self.assertEqual(s.command_authority, CommandAuthority.NONE)
        s.clear_inhibit(SafetyInhibit.THERMAL_FIRE_PENDING)
        self.assertEqual(s.phase, MissionPhase.IDLE)
        self.assertEqual(s.command_authority, CommandAuthority.NONE)

    def test_holding_clear_preserves_latch(self) -> None:
        s = MissionOwnerState()
        s.begin_operator_stop()
        s.finish_operator_stop()
        s.raise_inhibit(SafetyInhibit.THERMAL_FIRE_PENDING)
        s.clear_inhibit(SafetyInhibit.THERMAL_FIRE_PENDING)
        self.assertEqual(s.phase, MissionPhase.STOPPED)
        self.assertTrue(s.operator_stop_latch)
        self.assertTrue(s.abort_resend_forbidden())

    def test_start_rejects_pending_fire(self) -> None:
        s = MissionOwnerState()
        s.raise_inhibit(SafetyInhibit.THERMAL_FIRE_PENDING)
        # HOLDING is rejected first (§C5); inhibit remains for status/GUI.
        self.assertEqual(s.start_reject_code(), 'PHASE_HOLDING')
        self.assertIn(SafetyInhibit.THERMAL_FIRE_PENDING, s.active_inhibits)

    def test_abort_resend_gate(self) -> None:
        s = MissionOwnerState()
        s.accept_normal_start()
        self.assertFalse(s.abort_resend_forbidden())
        s.begin_operator_stop()
        self.assertTrue(s.abort_resend_forbidden())

    def test_terminal_inhibit_stopped(self) -> None:
        s = MissionOwnerState()
        s.raise_inhibit(SafetyInhibit.PENDING_FIRE_UNRESOLVED)
        self.assertEqual(s.phase, MissionPhase.STOPPED)
        self.assertIn(SafetyInhibit.PENDING_FIRE_UNRESOLVED, s.active_inhibits)


class NoSafeRouteRearmTests(unittest.TestCase):
    """Wave 1b: raise → Start rejected → Stop clears → Start accepted."""

    def test_raise_start_rejected_stop_clears_start_accepted(self) -> None:
        s = MissionOwnerState()
        s.accept_normal_start()
        self.assertEqual(s.phase, MissionPhase.NAVIGATING)
        self.assertEqual(s.command_authority, CommandAuthority.NAV)

        # §3.2 raise ALL_BLOCKED_NO_HOME → STOPPED + NONE + UX NO_SAFE_ROUTE.
        s.raise_inhibit(SafetyInhibit.ALL_BLOCKED_NO_HOME)
        s.detail = 'NO_SAFE_ROUTE'
        self.assertEqual(s.phase, MissionPhase.STOPPED)
        self.assertEqual(s.command_authority, CommandAuthority.NONE)
        self.assertEqual(
            s.start_reject_code(), SafetyInhibit.ALL_BLOCKED_NO_HOME.value)

        # StopMission clears ALL_BLOCKED_NO_HOME (was missing → permanent lockout).
        s.begin_operator_stop()
        s.clear_all_blocked_on_stop()
        s.finish_operator_stop()
        s.detail = ''
        self.assertNotIn(
            SafetyInhibit.ALL_BLOCKED_NO_HOME, s.active_inhibits)
        self.assertEqual(s.phase, MissionPhase.STOPPED)
        self.assertEqual(s.command_authority, CommandAuthority.NONE)
        self.assertIsNone(s.start_reject_code())

        s.accept_normal_start()
        self.assertEqual(s.phase, MissionPhase.NAVIGATING)
        self.assertEqual(s.command_authority, CommandAuthority.NAV)
        self.assertFalse(s.operator_stop_latch)

    def test_start_reject_code_never_exempts_all_blocked(self) -> None:
        s = MissionOwnerState()
        s.phase = MissionPhase.STOPPED
        s.active_inhibits.add(SafetyInhibit.ALL_BLOCKED_NO_HOME)
        # Must reject even though phase is Start-eligible — no exemption.
        self.assertEqual(
            s.start_reject_code(), SafetyInhibit.ALL_BLOCKED_NO_HOME.value)

    def test_owner_home_sync_enter_and_success(self) -> None:
        """§3.4 sync: enter RETURNING+NAV; success IDLE+NONE (not STOPPED)."""
        s = MissionOwnerState()
        s.accept_normal_start()
        s.phase = MissionPhase.RETURNING_TO_START
        s.command_authority = CommandAuthority.NAV
        self.assertEqual(s.phase, MissionPhase.RETURNING_TO_START)
        self.assertEqual(s.command_authority, CommandAuthority.NAV)

        # Home success pinned to IDLE + NONE (APPROVE watch-out #2).
        s.phase = MissionPhase.IDLE
        s.command_authority = CommandAuthority.NONE
        s.detail = 'RETURNED_TO_START'
        self.assertEqual(s.phase, MissionPhase.IDLE)
        self.assertEqual(s.command_authority, CommandAuthority.NONE)
        self.assertIsNone(s.start_reject_code())


class HangBoundLadderTests(unittest.TestCase):
    """Wave 0 explicit ladder helpers — do not collapse T₁/T_seed with home."""

    def test_mission_stuck_ladder(self) -> None:
        self.assertEqual(
            mission_stuck_action(
                recoveries=3, budget_n=8,
                home_already_attempted=False, home_available=True),
            'recover',
        )
        self.assertEqual(
            mission_stuck_action(
                recoveries=8, budget_n=8,
                home_already_attempted=False, home_available=True),
            'attempt_home',
        )
        self.assertEqual(
            mission_stuck_action(
                recoveries=8, budget_n=8,
                home_already_attempted=True, home_available=True),
            'terminal',
        )
        self.assertEqual(
            mission_stuck_action(
                recoveries=8, budget_n=8,
                home_already_attempted=False, home_available=False),
            'terminal',
        )

    def test_home_stuck_ladder(self) -> None:
        self.assertEqual(
            home_stuck_action(
                recoveries=2, budget_n=8, elapsed_home_s=10.0, t2_s=60.0),
            'recover',
        )
        self.assertEqual(
            home_stuck_action(
                recoveries=8, budget_n=8, elapsed_home_s=10.0, t2_s=60.0),
            'terminal',
        )
        self.assertEqual(
            home_stuck_action(
                recoveries=0, budget_n=8, elapsed_home_s=60.0, t2_s=60.0),
            'terminal',
        )

    def test_t1_and_t_seed_direct_terminal(self) -> None:
        self.assertFalse(
            path_blocked_none_timed_out(elapsed_s=19.9, t1_s=20.0))
        self.assertTrue(
            path_blocked_none_timed_out(elapsed_s=20.0, t1_s=20.0))
        self.assertFalse(seed_timed_out(elapsed_s=29.9, t_seed_s=30.0))
        self.assertTrue(seed_timed_out(elapsed_s=30.0, t_seed_s=30.0))


if __name__ == '__main__':
    unittest.main()
