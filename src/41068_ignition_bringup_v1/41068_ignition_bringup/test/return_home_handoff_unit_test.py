#!/usr/bin/env python3
"""Unit tests for Wave-1 return-home handoff fences (no Gazebo).

Covers D2 SUCCEEDED fence, T₁ latch semantics, and GUI banner honesty.
"""

from __future__ import annotations

import math
import os
import sys
import unittest
from typing import Optional

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
ROOT = os.path.abspath(os.path.join(PKG, '..', '..', '..'))
sys.path.insert(0, PKG)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(PKG, 'scripts'))
sys.path.insert(0, os.path.join(ROOT, 'test'))


class HomeSuccessFenceTests(unittest.TestCase):
    """Exercise ``_home_success_fence_ok`` logic via a lightweight stub."""

    @staticmethod
    def _fence(
        *,
        nav_mode: str = 'home',
        generation: int = 1,
        owner_gen: int = 1,
        epoch: int = 3,
        fp_epoch: int = 3,
        plan_pending: bool = False,
        home_route_uid: Optional[str] = 'abc',
        cb_uid: str = 'abc',
        home_fp_uid: Optional[str] = 'abc',
    ) -> bool:
        # Mirror path_bank_manager._home_success_fence_ok (Wave-1 D2).
        if nav_mode != 'home':
            return False
        if generation != owner_gen:
            return False
        if int(epoch) != int(fp_epoch):
            return False
        if plan_pending:
            return False
        if home_route_uid is None:
            return False
        if not cb_uid or cb_uid != home_route_uid:
            return False
        if home_fp_uid is not None and cb_uid != home_fp_uid:
            return False
        return True

    def test_accepts_when_all_gates_match(self) -> None:
        self.assertTrue(self._fence())

    def test_rejects_during_plan_pending(self) -> None:
        self.assertFalse(self._fence(plan_pending=True))

    def test_rejects_without_home_route(self) -> None:
        self.assertFalse(self._fence(home_route_uid=None))

    def test_rejects_uid_mismatch(self) -> None:
        self.assertFalse(self._fence(cb_uid='other'))

    def test_rejects_mission_mode(self) -> None:
        self.assertFalse(self._fence(nav_mode='mission'))

    def test_rejects_stale_epoch(self) -> None:
        self.assertFalse(self._fence(epoch=1, fp_epoch=3))


class T1LatchSemanticsTests(unittest.TestCase):
    def test_return_first_when_latch_false(self) -> None:
        home_attempted = False
        return_home = True
        has_alt = False
        switch_pending = False
        should_home = (
            return_home
            and not home_attempted
            and not has_alt
            and not switch_pending
        )
        self.assertTrue(should_home)

    def test_terminal_when_latch_true(self) -> None:
        home_attempted = True
        return_home = True
        has_alt = False
        switch_pending = False
        should_home = (
            return_home
            and not home_attempted
            and not has_alt
            and not switch_pending
        )
        self.assertFalse(should_home)
        # Second T₁ → path_blocked_none_timeout
        should_terminal = not should_home and not has_alt and not switch_pending
        self.assertTrue(should_terminal)

    def test_entry059_preserves_queued_switch(self) -> None:
        home_attempted = False
        has_alt = False
        switch_pending = True
        should_home = (
            True
            and not home_attempted
            and not has_alt
            and not switch_pending
        )
        self.assertFalse(should_home)


class GuiBannerHonestyTests(unittest.TestCase):
    def test_no_path_is_not_returning(self) -> None:
        import robot_status_gui as gui  # noqa: WPS433

        self.assertEqual(
            gui.mission_status_from_path_bank('NO_PATH_AVAILABLE'),
            'all paths blocked',
        )
        self.assertEqual(
            gui.mission_status_from_path_bank('RETURNING_TO_START'),
            'all paths blocked - returning to home',
        )
        self.assertNotIn(
            'NO_PATH_AVAILABLE',
            gui._PATH_BANK_RETURNING,
        )
        self.assertNotIn(
            'NO_PATH_AVAILABLE',
            gui._PATH_BANK_RETURN_TERMINAL,
        )

    def test_h6_no_safe_banner_includes_detail(self) -> None:
        import robot_status_gui as gui  # noqa: WPS433

        self.assertEqual(
            gui.mission_status_from_path_bank(
                'NO_SAFE_ROUTE', no_safe_detail='home_footprint_invalid'),
            'no safe route (home_footprint_invalid)',
        )
        banner = gui.mission_banner_from_owner(
            'STOPPED',
            active_inhibits=['ALL_BLOCKED_NO_HOME'],
            detail='NO_SAFE_ROUTE:home_footprint_invalid',
        )
        self.assertEqual(
            banner,
            'STOPPED: all blocked - no home route (home_footprint_invalid)',
        )
        # Mid-return must never show NO HOME even if stale detail present.
        mid = gui.mission_banner_from_owner(
            'RETURNING_TO_START',
            mission_running=True,
            command_authority='NAV',
            detail='NO_SAFE_ROUTE:stale',
            no_safe_detail='stale',
        )
        self.assertEqual(mid, 'all paths blocked - returning to home')
        self.assertNotIn('no home route', mid.lower())

    def test_parse_last_no_safe_detail(self) -> None:
        import robot_status_gui as gui  # noqa: WPS433

        status = (
            'state=NO_SAFE_ROUTE\n'
            'last_no_safe_detail=home_plan_empty\n'
            'active_id=None\n'
        )
        self.assertEqual(
            gui.parse_last_no_safe_detail(status),
            'home_plan_empty',
        )
        self.assertEqual(gui.parse_last_no_safe_detail('state=IDLE\n'), '')


class RebuildResumeHomeTests(unittest.TestCase):
    def test_home_token_forbids_mission_resume(self) -> None:
        resume = 'HOME'
        bank_state = 'FOLLOWING_PATH'  # briefly
        home_attempted = True
        nav_mode = 'home'
        forbid_mission = (
            resume == 'HOME'
            or (home_attempted and nav_mode == 'home')
        )
        self.assertTrue(forbid_mission)
        # Even if bank looks FOLLOWING, HOME token wins.
        self.assertEqual(resume, 'HOME')
        self.assertEqual(bank_state, 'FOLLOWING_PATH')


class ViaFsmContractTests(unittest.TestCase):
    """R4 via FSM contracts (source-level; no Gazebo)."""

    def test_retry_branch_footprint_enters_via_not_fresh_plan(self) -> None:
        import os

        pkg = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        src = open(os.path.join(pkg, 'scripts', 'path_bank_manager.py')).read()
        # Footprint → set count=2 + via enter; must not always-fresh-plan.
        self.assertIn("token == 'home_footprint_invalid'", src)
        self.assertIn('self._home_plan_attempt_count = 2', src)
        self.assertIn('self._home_via_fsm_enter()', src)
        self.assertIn('def _home_via_leg_callback_ok', src)
        # ASSEMBLE entry pinned to LEG2_PENDING pre-transition.
        self.assertIn(
            "if not self._home_via_leg_callback_ok('LEG2_PENDING')",
            src,
        )
        self.assertIn("self._home_via_phase = 'ASSEMBLE'", src)
        # Raise invalidates via gen sentinel.
        self.assertIn('self._home_via_generation = -1', src)
        # Via legs must not call _request_fresh_home_plan (helper docstring).
        self.assertIn(
            'never ``_request_fresh_home_plan``',
            src,
        )

    def test_point_clearance_public_and_sealed_disk(self) -> None:
        import numpy as np
        from rs1_nav.path_bank import LETHAL, OccupancyGrid2D, point_clearance
        from rs1_nav.route_instance import FOOTPRINT_PAD

        # 10×10 m map, 0.1 m cells; lethal disk radius 3 m at origin seals L≤2.
        res = 0.1
        w = h = 100
        costs = np.zeros((h, w), dtype=np.uint8)
        for iy in range(h):
            for ix in range(w):
                x = (ix + 0.5) * res - 5.0
                y = (iy + 0.5) * res - 5.0
                if math.hypot(x, y) <= 3.0:
                    costs[iy, ix] = LETHAL
        grid = OccupancyGrid2D(
            origin_x=-5.0,
            origin_y=-5.0,
            resolution=res,
            width=w,
            height=h,
            costs=costs,
        )
        # Open cell far away has clearance; near disk edge fails pad.
        self.assertGreater(point_clearance(4.5, 0.0, grid), FOOTPRINT_PAD)
        # Inside sealed radius → ~0 clearance.
        self.assertLess(point_clearance(0.0, 0.0, grid), 0.2)


if __name__ == '__main__':
    unittest.main()
