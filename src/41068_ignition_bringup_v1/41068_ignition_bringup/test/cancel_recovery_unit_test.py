#!/usr/bin/env python3
"""Unit tests for bounded FollowPath cancellation and safe hold recovery."""

from __future__ import annotations

import os
import sys
import types
import unittest
from unittest.mock import patch

from builtin_interfaces.msg import Time
from action_msgs.msg import GoalStatus
from std_msgs.msg import Bool

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, PKG)
sys.path.insert(0, os.path.join(PKG, 'scripts'))

import path_bank_manager as pbm  # noqa: E402


class _Future:
    def __init__(self, response) -> None:
        self.response = response
        self.callbacks = []

    def add_done_callback(self, callback) -> None:
        self.callbacks.append(callback)

    def result(self):
        return self.response


class _Handle:
    def __init__(self) -> None:
        self.cancel_calls = 0
        self.futures = []

    def cancel_goal_async(self):
        self.cancel_calls += 1
        future = _Future(types.SimpleNamespace(return_code=0))
        self.futures.append(future)
        return future


class _Owner:
    def __init__(self) -> None:
        self.generation = 3
        self.phase = pbm.MissionPhase.NAVIGATING
        self.command_authority = pbm.CommandAuthority.NAV
        self.operator_stop_latch = False
        self.active_inhibits = set()
        self.detail = ''

    def has_blocking_inhibit(self):
        return False

    def raise_inhibit(self, code):
        self.active_inhibits.add(code)
        self.command_authority = pbm.CommandAuthority.NONE

    def clear_inhibit(self, code):
        self.active_inhibits.discard(code)
        return True


class CancelHarness:
    def __init__(self) -> None:
        self._owner = _Owner()
        self._state = pbm.MissionNavState.SWITCHING_PATH
        self._fp_goal_handle = _Handle()
        self._fp_goal_epoch = 7
        self._switch_action_token = 1
        self._fp_cancel_context = None
        self._fp_cancel_retry_used = False
        self._fp_cancel_started_at = 0.0
        self._fp_cancel_deadline = 0.0
        self._fp_cancel_reason = ''
        self._fp_cancelled_epochs = set()
        self._map_frame = 'husky1_map'
        self._last_submitted_path = None
        self._follow_path_ref_epoch = -1
        self._active_path_pub = types.SimpleNamespace(
            publish=lambda msg: self._published_active.append(msg))
        self._marker_pub = types.SimpleNamespace(publish=lambda msg: None)
        self._state_pub = types.SimpleNamespace(publish=lambda msg: None)
        self._published_active = []
        self._resumed = False
        self._failed = False
        self._logger = types.SimpleNamespace(
            info=lambda *args: None,
            warn=lambda *args: None,
            error=lambda *args: None,
        )
        for name in (
            '_cancel_response_acknowledged',
            '_begin_follow_path_cancellation',
            '_retry_follow_path_cancellation',
            '_on_follow_path_cancel_response',
            '_tick_follow_path_cancellation',
            '_enter_safe_hold',
            '_enter_nav2_cancel_failed',
            '_clear_active_path',
            '_set_state',
        ):
            method = getattr(pbm.PathBankManager, name)
            if name == '_cancel_response_acknowledged':
                setattr(self, name, method)
            else:
                setattr(self, name, types.MethodType(method, self))

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(
                to_msg=lambda: Time(sec=0, nanosec=0)))

    def _switch_follow_path(self, _live_grid):
        self._resumed = True

    def _publish_status(self):
        return None

    def _publish_authority_now(self):
        return None


class CancelRecoveryTests(unittest.TestCase):
    def test_timeout_enters_safe_hold_then_retries_exact_handle(self) -> None:
        h = CancelHarness()
        original = h._fp_goal_handle
        self.assertTrue(h._begin_follow_path_cancellation('grid', 3, 1))
        h._fp_cancel_deadline = 0.0
        h._tick_follow_path_cancellation()

        self.assertEqual(h._state, pbm.MissionNavState.SAFE_HOLD)
        self.assertEqual(h._owner.command_authority, pbm.CommandAuthority.NONE)
        self.assertIs(h._fp_cancel_context['handle'], original)
        self.assertEqual(original.cancel_calls, 2)

        h._fp_cancel_deadline = 0.0
        h._on_follow_path_cancel_response(
            original.futures[-1], 7, h._fp_cancel_context['switch_token'])

        self.assertTrue(h._resumed)
        self.assertIsNone(h._fp_cancel_context)

    def test_second_timeout_is_terminal_and_not_all_blocked(self) -> None:
        h = CancelHarness()
        h._begin_follow_path_cancellation('grid', 3, 1)
        h._fp_cancel_deadline = 0.0
        h._tick_follow_path_cancellation()
        h._fp_cancel_deadline = 0.0
        h._tick_follow_path_cancellation()

        self.assertEqual(h._state, pbm.MissionNavState.NAV2_CANCEL_FAILED)
        self.assertEqual(h._owner.command_authority, pbm.CommandAuthority.NONE)
        self.assertFalse(h._resumed)

    def test_stale_cancel_ack_after_stop_cannot_resume(self) -> None:
        h = CancelHarness()
        original = h._fp_goal_handle
        h._begin_follow_path_cancellation('grid', 3, 1)
        h._fp_cancel_context = None

        h._on_follow_path_cancel_response(original.futures[-1], 7, 1)

        self.assertFalse(h._resumed)
        self.assertIsNone(h._fp_cancel_context)

    def test_unlocalized_fire_timeout_is_terminal_hold(self) -> None:
        h = CancelHarness()
        h._pending_failsafe_s = 15.0
        h._pending_fire_since = 10.0
        h._cancel_nav = lambda: None
        h._publish_authority_now = lambda: None
        h.raise_blocking_inhibit = lambda code: h._owner.raise_inhibit(code)
        h._on_pending_fire = types.MethodType(
            pbm.PathBankManager._on_pending_fire, h)
        h._tick_pending_failsafe = types.MethodType(
            pbm.PathBankManager._tick_pending_failsafe, h)
        h._on_pending_fire(Bool(data=True))

        with patch.object(pbm.time, 'monotonic', return_value=26.0):
            h._tick_pending_failsafe()

        self.assertEqual(
            h._state, pbm.MissionNavState.FIRE_UNLOCALIZED_HOLD)
        self.assertEqual(h._owner.command_authority, pbm.CommandAuthority.NONE)
        self.assertIn(
            pbm.SafetyInhibit.PENDING_FIRE_UNRESOLVED,
            h._owner.active_inhibits)

        h._on_pending_fire(Bool(data=False))
        self.assertIn(
            pbm.SafetyInhibit.PENDING_FIRE_UNRESOLVED,
            h._owner.active_inhibits)

    def test_stale_spin_result_cannot_complete_new_spin(self) -> None:
        class Future:
            def result(self):
                return types.SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED)

        h = types.SimpleNamespace(
            _reorient_active=True,
            _spin_action_token=2,
            _owner=types.SimpleNamespace(generation=3),
            _spin_goal_handle=object(),
            _on_reorient_spin_complete=lambda: setattr(
                h, 'completed', True),
            _on_spin_failed_or_timeout=lambda: setattr(h, 'failed', True),
            _logger=types.SimpleNamespace(info=lambda *a: None),
            get_logger=lambda: h._logger,
        )
        h.completed = False
        h.failed = False
        method = types.MethodType(pbm.PathBankManager._spin_goal_result, h)

        method(Future(), 3, 1)

        self.assertFalse(h.completed)
        self.assertFalse(h.failed)
        self.assertIsNotNone(h._spin_goal_handle)


if __name__ == '__main__':
    unittest.main()
