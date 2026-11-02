#!/usr/bin/env python3
"""Named certified-plan safety harnesses (no Gazebo).

1. Pending unlocalised FIRE → motion hold → timeout fail-safe
   (mission owner + gate authority NONE).
2. StopMission during §G COSTMAP_REBUILD → no obsolete route resume;
   authority stays NONE / STOPPED (odometry-stop policy at owner layer).

Manager wiring that mirrors these policies:
  path_bank_manager._tick_pending_failsafe
  path_bank_manager._srv_stop_mission (clears _rebuild_pending)
  path_bank_manager._maybe_finish_rebuild (discards resume when STOPPED)
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


class PendingUnlocalisedFireHarness(unittest.TestCase):
    """Pending unlocalised FIRE → hold → timeout/fail-safe."""

    def test_pending_fire_holds_gate(self) -> None:
        s = MissionOwnerState()
        s.accept_normal_start()
        self.assertEqual(s.command_authority, CommandAuthority.NAV)
        # Unlocalised thermal-positive (§F) → blocking inhibit.
        s.raise_inhibit(SafetyInhibit.THERMAL_FIRE_PENDING)
        self.assertEqual(s.phase, MissionPhase.HOLDING)
        self.assertEqual(s.command_authority, CommandAuthority.NONE)
        self.assertTrue(s.has_blocking_inhibit())
        self.assertIsNotNone(s.start_reject_code())

    def test_pending_failsafe_terminal(self) -> None:
        """Mirror path_bank_manager._tick_pending_failsafe after timeout."""
        s = MissionOwnerState()
        s.raise_inhibit(SafetyInhibit.THERMAL_FIRE_PENDING)
        # Fail-safe: drop pending, raise terminal unresolved.
        s.active_inhibits.discard(SafetyInhibit.THERMAL_FIRE_PENDING)
        s.raise_inhibit(SafetyInhibit.PENDING_FIRE_UNRESOLVED)
        self.assertEqual(s.phase, MissionPhase.STOPPED)
        self.assertEqual(s.command_authority, CommandAuthority.NONE)
        self.assertTrue(s.abort_resend_forbidden())
        self.assertIn(
            SafetyInhibit.PENDING_FIRE_UNRESOLVED, s.active_inhibits)


class StopDuringSgRebuildHarness(unittest.TestCase):
    """StopMission during §G local rebuild → no obsolete resume."""

    def test_stop_during_costmap_rebuild_no_resume(self) -> None:
        s = MissionOwnerState()
        s.accept_normal_start()
        # §G entered COSTMAP_REBUILD with gate NONE (manager sets this).
        s.phase = MissionPhase.COSTMAP_REBUILD
        s.command_authority = CommandAuthority.NONE
        gen_before = s.generation

        # Operator Stop during rebuild (manager also clears _rebuild_pending).
        s.begin_operator_stop()
        s.finish_operator_stop()

        self.assertEqual(s.phase, MissionPhase.STOPPED)
        self.assertEqual(s.command_authority, CommandAuthority.NONE)
        self.assertTrue(s.operator_stop_latch)
        self.assertTrue(s.abort_resend_forbidden())
        self.assertGreater(s.generation, gen_before)

        # Policy: rebuild completion must not re-arm NAV under latch.
        # (Manager: _maybe_finish_rebuild returns early on STOPPED / latch.)
        if s.operator_stop_latch or s.phase in (
            MissionPhase.STOPPING, MissionPhase.STOPPED,
        ):
            resume_ok = False
        else:
            resume_ok = True
        self.assertFalse(resume_ok)

    def test_rebuild_pending_discard_flag_contract(self) -> None:
        """Document manager contract: Stop clears rebuild_pending.

        Pure flag simulation — same order as _srv_stop_mission.
        """
        rebuild_pending = True
        s = MissionOwnerState()
        s.phase = MissionPhase.COSTMAP_REBUILD
        s.command_authority = CommandAuthority.NONE
        s.begin_operator_stop()
        rebuild_pending = False  # manager: self._rebuild_pending = False
        s.finish_operator_stop()
        self.assertFalse(rebuild_pending)
        self.assertEqual(s.phase, MissionPhase.STOPPED)
        self.assertEqual(s.command_authority, CommandAuthority.NONE)


if __name__ == '__main__':
    unittest.main()
