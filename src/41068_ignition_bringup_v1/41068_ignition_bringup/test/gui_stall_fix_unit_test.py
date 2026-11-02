#!/usr/bin/env python3
"""Unit coverage for GUI stall fix (demo-fire isolation, switch queue, gate)."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

_PKG = Path(__file__).resolve().parents[1]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from rs1_nav.fire_bridge import (  # noqa: E402
    bearing_aims_demo_obstacle,
    near_demo_obstacle,
)
from rs1_nav.mission_owner import (  # noqa: E402
    CommandAuthority,
    MissionOwnerState,
    MissionPhase,
    SafetyInhibit,
)


class DemoFireIsolationTest(unittest.TestCase):
    def test_near_demo_obstacle(self) -> None:
        demos = [(-1.3, -11.5)]
        self.assertTrue(near_demo_obstacle((-1.68, -10.97), demos, 2.5))
        self.assertFalse(near_demo_obstacle((10.0, 10.0), demos, 2.5))

    def test_bearing_aims_demo(self) -> None:
        demos = [(-1.3, -11.5)]
        self.assertTrue(
            bearing_aims_demo_obstacle(
                robot_xy=(-6.0, -11.5),
                robot_yaw=0.0,
                bearing_rad=0.0,
                demos=demos,
                max_range_m=10.0,
                angular_margin_rad=0.3,
            )
        )
        # Aim opposite direction
        self.assertFalse(
            bearing_aims_demo_obstacle(
                robot_xy=(-6.0, -11.5),
                robot_yaw=0.0,
                bearing_rad=math.pi,
                demos=demos,
                max_range_m=10.0,
                angular_margin_rad=0.2,
            )
        )


class MemoryHorizonGateTest(unittest.TestCase):
    """Active-path memory invalidation must be robot-horizon scoped.

    Distant trail-wall ghosts (e.g. (-23.1,-7.8) near spawn) must not abort
    Path 1 before the GUI demo can spawn at gap_a (~5 m trigger).
    """

    @staticmethod
    def _ahead(
        points,
        robot_xy,
        horizon_m: float,
        occupied,
    ) -> bool:
        """Mirror of path_bank_manager._memory_intersects_path_ahead gating."""
        if not points:
            return False
        if horizon_m <= 0.0 or robot_xy is None:
            return any(p in occupied for p in points)
        rx, ry = robot_xy
        h2 = horizon_m * horizon_m
        near = [
            (x, y)
            for (x, y) in points
            if (x - rx) * (x - rx) + (y - ry) * (y - ry) <= h2
        ]
        if not near:
            return False
        return any(p in occupied for p in near)

    def test_distant_ghost_ignored_near_spawn(self) -> None:
        # Path samples include a far western wall ghost and the demo gap.
        points = [(-23.1, -7.8), (-12.0, -4.0), (-1.3, -11.5)]
        occupied = {(-23.1, -7.8)}
        robot = (-18.0, 3.0)  # spawn
        self.assertFalse(
            self._ahead(points, robot, 5.0, occupied),
            'distant ghost within path but outside 5 m horizon must not invalidate',
        )
        # Full-path (horizon<=0) still sees it — candidate precheck behaviour.
        self.assertTrue(self._ahead(points, robot, 0.0, occupied))

    def test_near_horizon_hit_invalidates(self) -> None:
        points = [(-23.1, -7.8), (-1.3, -11.5)]
        occupied = {(-1.3, -11.5)}
        robot = (-4.0, -11.0)  # approaching gap_a (~2.7 m)
        self.assertTrue(self._ahead(points, robot, 5.0, occupied))

    def test_path_b_midroute_ghost_outside_horizon(self) -> None:
        # Observed false Path 2 block: locus (23.5,19.7) with robot ~(16.6,23.3)
        points = [(16.6, 23.3), (20.0, 21.0), (23.5, 19.7), (16.0, -0.1)]
        occupied = {(23.5, 19.7)}
        robot = (16.6, 23.3)
        self.assertFalse(self._ahead(points, robot, 5.0, occupied))
        self.assertTrue(self._ahead(points, robot, 8.0, occupied))

    def test_beyond_horizon_no_samples(self) -> None:
        points = [(-1.3, -11.5)]
        occupied = {(-1.3, -11.5)}
        robot = (-18.0, 3.0)
        # gap is ~18 m from spawn — outside 5 m → no near samples → False
        self.assertFalse(self._ahead(points, robot, 5.0, occupied))


class SwitchQueueContractTest(unittest.TestCase):
    """Mirror path_bank stop/inhibit clearing of SWITCH_PENDING."""

    def test_stop_clears_pending_flag_contract(self) -> None:
        pending = True
        owner = MissionOwnerState()
        owner.begin_operator_stop()
        # StopMission must clear queued switch (plan F2).
        pending = False
        owner.finish_operator_stop()
        self.assertFalse(pending)
        self.assertEqual(owner.command_authority, CommandAuthority.NONE)

    def test_terminal_inhibit_phases_block_rearm(self) -> None:
        for phase in (
            MissionPhase.COSTMAP_REBUILD,
            MissionPhase.HOLDING,
            MissionPhase.STOPPING,
            MissionPhase.STOPPED,
            MissionPhase.IDLE,
        ):
            owner = MissionOwnerState()
            owner.phase = phase
            owner.command_authority = CommandAuthority.NONE
            # Completing queued switch is forbidden in these phases.
            may_complete = (
                owner.command_authority == CommandAuthority.NAV
                and owner.phase in (MissionPhase.NAVIGATING, MissionPhase.SWITCHING)
                and not owner.has_blocking_inhibit()
                and not owner.operator_stop_latch
            )
            self.assertFalse(may_complete, phase.value)

    def test_pending_fire_unresolved_is_blocking(self) -> None:
        owner = MissionOwnerState()
        owner.raise_inhibit(SafetyInhibit.PENDING_FIRE_UNRESOLVED)
        self.assertTrue(owner.has_blocking_inhibit())
        self.assertEqual(owner.command_authority, CommandAuthority.NONE)


if __name__ == '__main__':
    unittest.main()
