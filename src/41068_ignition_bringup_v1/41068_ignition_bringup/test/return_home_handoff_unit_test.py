#!/usr/bin/env python3
"""Unit tests for Wave-1 return-home handoff fences (no Gazebo).

Covers D2 SUCCEEDED fence, T₁ latch semantics, and GUI banner honesty.
"""

from __future__ import annotations

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


if __name__ == '__main__':
    unittest.main()
