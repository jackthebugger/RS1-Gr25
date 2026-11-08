#!/usr/bin/env python3
"""Unit coverage for Entry 062 GUI dual-block Wave-1 (DB-U1–U10).

Binds real ``PathBankManager`` methods onto a ROS-free harness so unlock /
pick / resume gates are exercised without Gazebo.
"""

from __future__ import annotations

import os
import sys
import types
import unittest
from typing import List, Optional, Tuple
from unittest.mock import MagicMock, patch

import numpy as np
from action_msgs.msg import GoalStatus

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
ROOT = os.path.abspath(os.path.join(PKG, '..', '..', '..'))
sys.path.insert(0, PKG)
sys.path.insert(0, os.path.join(PKG, 'scripts'))

from rs1_nav.path_bank import (  # noqa: E402
    CandidateState,
    OccupancyGrid2D,
    PathBank,
    PathBankConfig,
    PathCandidate,
)

# Import after path setup; requires rclpy (Humble) but not a spinning node.
import path_bank_manager as pbm  # noqa: E402


Point = Tuple[float, float]


class _FakeLogger:
    def __init__(self) -> None:
        self.infos: List[str] = []
        self.warns: List[str] = []
        self.errors: List[str] = []

    def info(self, msg: str, *args) -> None:
        self.infos.append(str(msg) % args if args else str(msg))

    def warn(self, msg: str, *args) -> None:
        self.warns.append(str(msg) % args if args else str(msg))

    def error(self, msg: str, *args) -> None:
        self.errors.append(str(msg) % args if args else str(msg))


class _FakeMaskInfo:
    def __init__(
        self, width: int, height: int, resolution: float, ox: float, oy: float
    ) -> None:
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin = types.SimpleNamespace(
            position=types.SimpleNamespace(x=ox, y=oy)
        )


class _FakeMask:
    def __init__(
        self,
        width: int,
        height: int,
        resolution: float,
        ox: float,
        oy: float,
        data: List[int],
    ) -> None:
        self.info = _FakeMaskInfo(width, height, resolution, ox, oy)
        self.data = data


class _FakeRoute:
    def __init__(self, path_id: int, points: List[Point]) -> None:
        self.path_id = path_id
        self.points = tuple(points)
        self.yaw_rad = tuple(0.0 for _ in points)
        self.route_uid = f'r{path_id}'


class _FakeRouteBank:
    def __init__(self, routes: dict) -> None:
        self._routes = routes
        self.active_uid = None

    def by_path_id(self, path_id: int):
        return self._routes.get(path_id)


def _free_grid(
    points: List[Point], *, pad_m: float = 5.0, res: float = 0.5
) -> OccupancyGrid2D:
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    ox = min(xs) - pad_m
    oy = min(ys) - pad_m
    w = int((max(xs) - ox + pad_m) / res) + 2
    h = int((max(ys) - oy + pad_m) / res) + 2
    return OccupancyGrid2D(
        origin_x=ox,
        origin_y=oy,
        resolution=res,
        width=w,
        height=h,
        costs=np.zeros((h, w), dtype=np.uint8),
    )


def _mask_with_hits(
    hits: List[Point],
    *,
    res: float = 0.5,
    pad_m: float = 5.0,
) -> _FakeMask:
    if not hits:
        ox = oy = -10.0
        w = h = 40
        return _FakeMask(w, h, res, ox, oy, [-1] * (w * h))
    xs = [p[0] for p in hits]
    ys = [p[1] for p in hits]
    ox = min(xs) - pad_m
    oy = min(ys) - pad_m
    w = int((max(xs) - ox + pad_m) / res) + 2
    h = int((max(ys) - oy + pad_m) / res) + 2
    data = [-1] * (w * h)
    for x, y in hits:
        ix = int((x - ox) / res)
        iy = int((y - oy) / res)
        if 0 <= ix < w and 0 <= iy < h:
            data[iy * w + ix] = 100
    return _FakeMask(w, h, res, ox, oy, data)


def _cand(
    path_id: int,
    points: List[Point],
    *,
    score: float,
    state: CandidateState = CandidateState.AVAILABLE,
    reason: str = '',
) -> PathCandidate:
    return PathCandidate(
        path_id=path_id,
        points=points,
        length_m=float(len(points)),
        min_clearance_m=1.0,
        mean_cost=0.0,
        turn_penalty=0.0,
        score=score,
        state=state,
        block_reason=reason,
    )


class DualBlockHarness:
    """ROS-free stand-in with PathBankManager Wave-1 methods bound."""

    def __init__(self) -> None:
        self._logger = _FakeLogger()
        self._cfg = PathBankConfig(sample_stride=1, lethal_threshold=254)
        self._block_locus: dict = {}
        self._block_locus_radius_m = 0.60
        self._ever_blocked_ids: set = set()
        self._operator_block_latch: dict = {}
        self._operator_block_seat_xy: dict = {}
        self._ever_demo_latched_ids: set = set()
        self._fire_memory_centres: List[Point] = []
        self._fire_unlock_corridor_radius_m = 0.80
        self._block_source: dict = {}
        self._demo_obstacle_poses: List[Point] = []
        self._operator_latch_assoc_radius_m = 3.0
        self._known_demo_seats: Tuple[Point, ...] = (
            (-1.3, -11.5),
            (-5.1, 21.1),
        )
        self._memory_mask = None
        self._prior_map = None
        self._bank: Optional[PathBank] = None
        self._bank_expansion_pending = False
        self._route_bank = None
        self._use_follow_path = False
        self._robot_xyyaw: Optional[Tuple[float, float, float]] = None
        self._memory_block_horizon_m = 5.0
        self._state = pbm.MissionNavState.FOLLOWING_PATH
        self._nav_mode: Optional[str] = 'mission'
        self._return_home_on_all_blocked = True
        self._begin_return_calls = 0
        self._map_frame = 'map'
        self._last_submitted_path = None
        self._follow_path_ref_epoch = -1
        self._published_active: List = []
        self._published_markers: List = []
        self._active_path_pub = MagicMock()
        self._marker_pub = MagicMock()
        self._active_path_pub.publish.side_effect = (
            lambda msg: self._published_active.append(msg)
        )
        self._marker_pub.publish.side_effect = (
            lambda msg: self._published_markers.append(msg)
        )

        bind = (
            '_memory_intersects_path',
            '_first_memory_hit_on_path',
            '_memory_intersects_path_near',
            '_memory_intersects_path_ahead',
            '_mask_occupied_near_xy',
            '_candidate_points_for_gate',
            '_record_block_locus',
            '_record_block_source',
            '_mark_path_blocked',
            '_nearest_demo_seat',
            '_maybe_set_operator_block_latch',
            '_set_operator_block_latch',
            '_operator_block_latch_active',
            '_clear_operator_block_latches',
            '_demo_pose_near_xy',
            '_maybe_clear_operator_latches_for_gone_gaps',
            '_route_forward_from_robot',
            '_route_validity_forward_from_robot',
            '_nearest_route_index_progress_aware',
            '_prior_block_locus_behind_forward_slice',
            '_candidate_route_valid',
            '_validate_pending_candidates',
            '_active_route_valid',
            '_try_evidence_unblock',
            '_pick_next_valid',
            '_sync_route_bank_active',
            '_on_demo_obstacle_poses',
            '_fire_cloud_intersects_corridor',
            '_publish_follow_path_ref',
            '_clear_active_path',
        )
        for name in bind:
            setattr(self, name, types.MethodType(getattr(pbm.PathBankManager, name), self))

    def get_clock(self):
        from builtin_interfaces.msg import Time
        clock = MagicMock()
        stamp = Time()
        stamp.sec = 0
        stamp.nanosec = 0
        clock.now.return_value.to_msg.return_value = stamp
        return clock

    def get_logger(self) -> _FakeLogger:
        return self._logger

    def _publish_bank_paths(self) -> None:
        return None

    def _set_state(self, state) -> None:
        self._state = state

    def _begin_return_to_start(self) -> None:
        self._begin_return_calls += 1
        self._state = pbm.MissionNavState.RETURNING_TO_START
        self._nav_mode = 'home'


GAP_A: Point = (-1.3, -11.5)
GAP_B: Point = (-5.1, 21.1)

# Simple A corridor through gap_a; B corridor through gap_b.
PATH_A_PTS: List[Point] = [
    (-18.0, 3.0),
    (-10.0, -5.0),
    GAP_A,
    (5.0, -6.0),
    (16.0, -0.1),
]
PATH_B_PTS: List[Point] = [
    (-18.0, 3.0),
    (-12.0, 12.0),
    GAP_B,
    (5.0, 10.0),
    (16.0, -0.1),
]


class GuiDualBlockUnitTest(unittest.TestCase):
    def _bank_ab(
        self,
        *,
        a_state: CandidateState = CandidateState.BLOCKED,
        b_state: CandidateState = CandidateState.ACTIVE,
        active_id: Optional[int] = 2,
    ) -> PathBank:
        return PathBank(
            start=(-18.0, 3.0),
            goal=(16.0, -0.1),
            candidates=[
                _cand(1, PATH_A_PTS, score=100.0, state=a_state, reason='unit'),
                _cand(2, PATH_B_PTS, score=90.0, state=b_state, reason=''),
            ],
            active_id=active_id,
        )

    def test_db_u1_full_path_refuse_for_demo_origin(self) -> None:
        """DB-U1: locus disk clear, residual centreline memory → no unlock."""
        h = DualBlockHarness()
        h._bank = self._bank_ab()
        h._block_locus[1] = GAP_A
        h._ever_blocked_ids.add(1)
        h._ever_demo_latched_ids.add(1)  # NB-M1 A: demo-originated depth
        # Residual keepout far from locus disk (on Path A centreline).
        far = (5.0, -6.0)
        h._memory_mask = _mask_with_hits([far])
        live = _free_grid(PATH_A_PTS + PATH_B_PTS)
        h._try_evidence_unblock(live)
        self.assertEqual(h._bank.candidates[0].state, CandidateState.BLOCKED)
        self.assertTrue(
            any('full-path memory hit' in s for s in h._logger.infos),
            h._logger.infos,
        )

    def test_db_u2_u7_unlock_allowed_non_demo(self) -> None:
        """DB-U2 / DB-U7: never-demo + locus clear + geom free → unlock."""
        h = DualBlockHarness()
        h._bank = self._bank_ab(active_id=2)
        h._block_locus[1] = GAP_A
        h._ever_blocked_ids.add(1)
        # No ever_demo_latched → Entry 035 locus-only.
        h._memory_mask = _mask_with_hits([])  # empty / unknown
        live = _free_grid(PATH_A_PTS + PATH_B_PTS)
        active_before = h._bank.active_id
        h._try_evidence_unblock(live)
        self.assertEqual(h._bank.candidates[0].state, CandidateState.AVAILABLE)
        self.assertEqual(h._bank.candidates[0].block_reason, 'evidence_clear')
        self.assertEqual(h._bank.active_id, active_before)

    def test_db_u3_exhaustion_pick_none(self) -> None:
        """DB-U3: A blocked + B invalid → pick None (caller → return-home)."""
        h = DualBlockHarness()
        h._bank = self._bank_ab(
            a_state=CandidateState.BLOCKED,
            b_state=CandidateState.AVAILABLE,
            active_id=None,
        )
        h._block_locus[1] = GAP_A
        h._ever_blocked_ids.add(1)
        h._operator_block_latch[1] = True
        h._memory_mask = _mask_with_hits([GAP_B])
        live = _free_grid(PATH_A_PTS + PATH_B_PTS)
        nxt = h._pick_next_valid(live)
        self.assertIsNone(nxt)
        # Simulate exhaustion caller (Entry 061 invoke, not rewrite).
        if nxt is None and h._return_home_on_all_blocked:
            h._begin_return_to_start()
        self.assertEqual(h._begin_return_calls, 1)
        self.assertEqual(h._state, pbm.MissionNavState.RETURNING_TO_START)

    def test_db_u11_pending_candidate_defers_exhaustion(self) -> None:
        """A merged-but-unvalidated route must not trigger return-home."""
        h = DualBlockHarness()
        h._bank = self._bank_ab(
            a_state=CandidateState.BLOCKED,
            b_state=CandidateState.PENDING,
            active_id=None,
        )
        h._memory_mask = _mask_with_hits([])
        live = _free_grid(PATH_A_PTS + PATH_B_PTS)

        nxt = h._pick_next_valid(live)

        self.assertIsNotNone(nxt)
        self.assertEqual(nxt.path_id, 2)
        self.assertEqual(h._bank.candidates[1].state, CandidateState.ACTIVE)
        self.assertEqual(h._begin_return_calls, 0)

    def test_db_u12_all_blocked_returns_home_despite_expansion_pending(self) -> None:
        """Entry 080/081: all_blocked → return-home even if expansion pending."""
        h = DualBlockHarness()
        h._bank = self._bank_ab(
            a_state=CandidateState.BLOCKED,
            b_state=CandidateState.BLOCKED,
            active_id=None,
        )
        h._bank_expansion_pending = True
        live = _free_grid(PATH_A_PTS + PATH_B_PTS)

        nxt = h._pick_next_valid(live)
        # Mirror _switch_follow_path all-blocked gate (Entry 080 normative).
        has_available_alternative = any(
            c.state == CandidateState.AVAILABLE
            and c.path_id != h._bank.active_id
            for c in h._bank.candidates
        )
        if nxt is None and (
            has_available_alternative
            or h._bank.all_blocked()
            or not h._bank_expansion_pending
        ):
            if nxt is None and h._return_home_on_all_blocked and h._bank.all_blocked():
                h._begin_return_to_start()

        self.assertIsNone(nxt)
        self.assertTrue(h._bank.all_blocked())
        self.assertEqual(h._begin_return_calls, 1)
        self.assertEqual(h._state, pbm.MissionNavState.RETURNING_TO_START)

    def test_db_u13_active_path_clears_during_switch_transition(self) -> None:
        """Authoritative active-path output must not retain blocked geometry."""
        h = DualBlockHarness()
        h._last_submitted_path = object()
        h._follow_path_ref_epoch = 4
        h._clear_active_path()

        self.assertEqual(len(h._published_active), 1)
        self.assertEqual(len(h._published_active[0].poses), 0)
        self.assertIsNone(h._last_submitted_path)
        self.assertEqual(h._follow_path_ref_epoch, -1)

    def test_db_u14_cancel_response_codes_are_explicit(self) -> None:
        """Cancellation cleanup accepts only terminal/non-running outcomes."""
        accepted = (0, 2, 3, 4)
        for code in accepted:
            response = types.SimpleNamespace(return_code=code)
            self.assertTrue(pbm.PathBankManager._cancel_response_acknowledged(
                response
            ))
        self.assertFalse(
            pbm.PathBankManager._cancel_response_acknowledged(
                types.SimpleNamespace(return_code=1)
            )
        )

    def test_db_u15_pending_switch_completes_when_available_despite_expansion(
        self,
    ) -> None:
        """Entry 080/081: AVAILABLE alt completes queued switch despite expansion."""
        bank = PathBank(
            start=(-18.0, 3.0),
            goal=(16.0, -0.1),
            candidates=[
                _cand(1, PATH_A_PTS, score=100.0, state=CandidateState.BLOCKED),
                _cand(2, PATH_B_PTS, score=90.0, state=CandidateState.AVAILABLE),
            ],
            active_id=1,
        )
        h = types.SimpleNamespace(
            _switch_pending=True,
            _use_follow_path=True,
            _owner=types.SimpleNamespace(
                operator_stop_latch=False,
                active_inhibits=set(),
                phase=pbm.MissionPhase.NAVIGATING,
                command_authority=pbm.CommandAuthority.NAV,
                has_blocking_inhibit=lambda: False,
            ),
            _live_costmap=object(),
            _bank=bank,
            _bank_expansion_pending=True,
            _fp_cancel_context=None,
            _switch_follow_path=MagicMock(),
            _set_state=MagicMock(),
            _logger=_FakeLogger(),
        )
        h.get_logger = lambda: h._logger
        method = types.MethodType(
            pbm.PathBankManager._maybe_complete_pending_switch, h)

        with patch.object(pbm, 'nav2_costmap_to_grid', return_value=object()):
            method()

        h._switch_follow_path.assert_called_once()

    def test_db_u15b_pending_switch_waits_without_available(self) -> None:
        """Defer queued switch only when no AVAILABLE and not all_blocked."""
        bank = PathBank(
            start=(-18.0, 3.0),
            goal=(16.0, -0.1),
            candidates=[
                _cand(1, PATH_A_PTS, score=100.0, state=CandidateState.BLOCKED),
                _cand(2, PATH_B_PTS, score=90.0, state=CandidateState.PENDING),
            ],
            active_id=1,
        )
        h = types.SimpleNamespace(
            _switch_pending=True,
            _use_follow_path=True,
            _owner=types.SimpleNamespace(
                operator_stop_latch=False,
                active_inhibits=set(),
                phase=pbm.MissionPhase.NAVIGATING,
                command_authority=pbm.CommandAuthority.NAV,
                has_blocking_inhibit=lambda: False,
            ),
            _live_costmap=object(),
            _bank=bank,
            _bank_expansion_pending=True,
            _fp_cancel_context=None,
            _switch_follow_path=MagicMock(),
            _set_state=MagicMock(),
            _logger=_FakeLogger(),
        )
        h.get_logger = lambda: h._logger
        method = types.MethodType(
            pbm.PathBankManager._maybe_complete_pending_switch, h)
        method()
        h._switch_follow_path.assert_not_called()

    def test_db_u4_locus_behind_slice_or_full_memory(self) -> None:
        """DB-U4: robot past gap_a; forward-slice skips locus; pick refuses A."""
        h = DualBlockHarness()
        h._use_follow_path = True
        # Past gap_a on densified A so forward-slice drops the block locus.
        h._robot_xyyaw = (5.0, -6.0, 0.0)
        densified_a = list(PATH_A_PTS)
        h._route_bank = _FakeRouteBank({1: _FakeRoute(1, densified_a)})
        h._bank = PathBank(
            start=(-18.0, 3.0),
            goal=(16.0, -0.1),
            candidates=[
                _cand(1, PATH_A_PTS, score=100.0, state=CandidateState.AVAILABLE),
            ],
            active_id=None,
        )
        h._block_locus[1] = GAP_A
        h._ever_blocked_ids.add(1)
        h._memory_mask = _mask_with_hits([GAP_A])
        live = _free_grid(PATH_A_PTS)
        ok, reason = h._candidate_route_valid(h._bank.candidates[0], live)
        self.assertFalse(ok)
        self.assertIn(
            reason,
            (
                'obstacle_memory_mask_full',
                'prior_block_locus_not_on_forward_slice',
                'obstacle_memory_mask',
            ),
            reason,
        )

    def test_db_u5_horizon_asymmetry(self) -> None:
        """DB-U5: 5 m active horizon ignores distant ghost; candidate full-path rejects."""
        h = DualBlockHarness()
        ghost = (-23.1, -7.8)
        pts = [ghost, (-12.0, -4.0), GAP_A]
        h._bank = PathBank(
            start=(-18.0, 3.0),
            goal=(16.0, -0.1),
            candidates=[_cand(1, pts, score=100.0, state=CandidateState.ACTIVE)],
            active_id=1,
        )
        h._robot_xyyaw = (-18.0, 3.0, 0.0)
        h._memory_mask = _mask_with_hits([ghost])
        live = _free_grid(pts)
        ok_active, _ = h._active_route_valid(h._bank.candidates[0], live)
        self.assertTrue(ok_active, 'distant ghost must not kill active under 5 m horizon')
        # Inactive candidate uses full-path memory (and ever-blocked harden).
        h._bank.candidates[0].state = CandidateState.AVAILABLE
        h._ever_blocked_ids.add(1)
        h._block_locus[1] = ghost
        ok_cand, reason = h._candidate_route_valid(h._bank.candidates[0], live)
        self.assertFalse(ok_cand)
        self.assertIn(reason, ('obstacle_memory_mask', 'obstacle_memory_mask_full'))

    def test_db_u6_no_bare_select_activates_invalid_a(self) -> None:
        """DB-U6: active_id None + AVAILABLE A with memory → pick must not activate A."""
        h = DualBlockHarness()
        h._bank = PathBank(
            start=(-18.0, 3.0),
            goal=(16.0, -0.1),
            candidates=[
                _cand(1, PATH_A_PTS, score=100.0, state=CandidateState.AVAILABLE),
                _cand(2, PATH_B_PTS, score=90.0, state=CandidateState.BLOCKED, reason='b'),
            ],
            active_id=None,
        )
        h._block_locus[1] = GAP_A
        h._ever_blocked_ids.add(1)
        h._memory_mask = _mask_with_hits([GAP_A])
        live = _free_grid(PATH_A_PTS + PATH_B_PTS)
        # Mirror timer branch: pick, not select_best_available.
        bare = h._bank.select_best_available
        chosen_bare = None
        # Prove bare select WOULD activate A (regression contrast).
        bank_copy_states = [c.state for c in h._bank.candidates]
        chosen_bare = bare()
        self.assertIsNotNone(chosen_bare)
        self.assertEqual(chosen_bare.path_id, 1)
        # Reset after bare contrast.
        h._bank.active_id = None
        for c, st in zip(h._bank.candidates, bank_copy_states):
            c.state = st
        chosen = h._pick_next_valid(live)
        self.assertIsNone(chosen)
        self.assertNotEqual(h._bank.active_id, 1)

    def test_db_u8_g5_latch_refuse_then_clear_fire(self) -> None:
        """DB-U8: demo still ACTIVE + all memory clear → refuse; after CLEAR → allow."""
        h = DualBlockHarness()
        h._bank = self._bank_ab()
        h._block_locus[1] = GAP_A
        h._ever_blocked_ids.add(1)
        h._demo_obstacle_poses = [GAP_A]
        h._set_operator_block_latch(1, GAP_A)
        h._memory_mask = _mask_with_hits([])  # locus + full clear
        live = _free_grid(PATH_A_PTS + PATH_B_PTS)
        h._try_evidence_unblock(live)
        self.assertEqual(h._bank.candidates[0].state, CandidateState.BLOCKED)
        self.assertTrue(
            any('operator_block_latch' in s for s in h._logger.infos),
            h._logger.infos,
        )
        # CLEAR FIRE: empty poses clear latch; memory still clear → unlock.
        msg = MagicMock()
        msg.poses = []
        h._on_demo_obstacle_poses(msg)
        self.assertFalse(h._operator_block_latch_active(1))
        h._try_evidence_unblock(live)
        self.assertEqual(h._bank.candidates[0].state, CandidateState.AVAILABLE)

    def test_db_u9_empty_mask_locus_behind_slice(self) -> None:
        """DB-U9: empty mask + locus behind slice → prior_block_locus refuse."""
        h = DualBlockHarness()
        h._use_follow_path = True
        h._robot_xyyaw = (5.0, -6.0, 0.0)  # past gap_a on densified A
        h._route_bank = _FakeRouteBank({1: _FakeRoute(1, list(PATH_A_PTS))})
        h._bank = PathBank(
            start=(-18.0, 3.0),
            goal=(16.0, -0.1),
            candidates=[
                _cand(1, PATH_A_PTS, score=100.0, state=CandidateState.AVAILABLE),
            ],
            active_id=None,
        )
        h._block_locus[1] = GAP_A
        h._ever_blocked_ids.add(1)
        h._memory_mask = _mask_with_hits([])  # empty
        live = _free_grid(PATH_A_PTS)
        # Lethal only on abandoned A samples (near gap_a) for (c) as well.
        gx, gy = GAP_A
        ix = live.world_to_grid(gx, gy)
        assert ix is not None
        live.costs[ix[1], ix[0]] = 254
        ok, reason = h._candidate_route_valid(h._bank.candidates[0], live)
        self.assertFalse(ok)
        self.assertIn(
            reason,
            ('prior_block_locus_not_on_forward_slice', 'lethal_full_centreline'),
            reason,
        )

    def test_db_u10_no_bare_nop_path_resume(self) -> None:
        """DB-U10: NO_PATH + unlock must not bare-flip FOLLOWING without pick."""
        h = DualBlockHarness()
        h._state = pbm.MissionNavState.NO_PATH_AVAILABLE
        h._bank = self._bank_ab(
            a_state=CandidateState.BLOCKED,
            b_state=CandidateState.BLOCKED,
            active_id=None,
        )
        h._block_locus[1] = GAP_A
        h._ever_blocked_ids.add(1)
        # Never-demo so unlock can succeed (locus clear).
        h._memory_mask = _mask_with_hits([])
        live = _free_grid(PATH_A_PTS + PATH_B_PTS)
        h._try_evidence_unblock(live)
        self.assertEqual(h._bank.candidates[0].state, CandidateState.AVAILABLE)
        # W1-RESUME: state must remain NO_PATH until validated pick.
        self.assertEqual(h._state, pbm.MissionNavState.NO_PATH_AVAILABLE)
        # Latched corridor unlock refused → still NO_PATH / not FOLLOWING.
        h2 = DualBlockHarness()
        h2._state = pbm.MissionNavState.NO_PATH_AVAILABLE
        h2._bank = self._bank_ab(
            a_state=CandidateState.BLOCKED,
            b_state=CandidateState.BLOCKED,
            active_id=None,
        )
        h2._block_locus[1] = GAP_A
        h2._set_operator_block_latch(1, GAP_A)
        h2._demo_obstacle_poses = [GAP_A]
        h2._memory_mask = _mask_with_hits([])
        h2._try_evidence_unblock(live)
        self.assertEqual(h2._bank.candidates[0].state, CandidateState.BLOCKED)
        self.assertEqual(h2._state, pbm.MissionNavState.NO_PATH_AVAILABLE)

    def test_latch_assoc_radius_pin(self) -> None:
        """R2 m1: association radius pinned at 3.0 m; live pose required."""
        h = DualBlockHarness()
        self.assertEqual(h._operator_latch_assoc_radius_m, 3.0)
        h._demo_obstacle_poses = []
        self.assertIsNone(h._nearest_demo_seat(GAP_A))
        h._demo_obstacle_poses = [GAP_A]
        self.assertEqual(h._nearest_demo_seat(GAP_A), GAP_A)
        # Far locus must not latch.
        self.assertIsNone(h._nearest_demo_seat((20.0, 20.0)))

    def test_g6_neg_fire_cloud_refuse_after_clear(self) -> None:
        """G6-NEG unit: FIRE on Path A corridor + latch gone → refuse unlock."""
        h = DualBlockHarness()
        h._bank = self._bank_ab()
        h._block_locus[1] = GAP_A
        h._ever_blocked_ids.add(1)
        h._ever_demo_latched_ids.add(1)
        h._block_source[1] = 'FIRE'
        # Off-centreline but within 0.80 m of Path A near gap_a.
        h._fire_memory_centres = [(-1.3, -11.5 + 0.55)]
        h._memory_mask = _mask_with_hits([])  # OccupancyGrid locus looks clear
        live = _free_grid(PATH_A_PTS + PATH_B_PTS)
        # CLEAR FIRE: empty poses drop latch only (typed FIRE retained).
        msg = MagicMock()
        msg.poses = []
        h._on_demo_obstacle_poses(msg)
        self.assertFalse(h._operator_block_latch_active(1))
        self.assertEqual(len(h._fire_memory_centres), 1)
        h._try_evidence_unblock(live)
        self.assertEqual(h._bank.candidates[0].state, CandidateState.BLOCKED)
        self.assertTrue(
            any('fire_memory_cloud near corridor' in s for s in h._logger.infos),
            h._logger.infos,
        )
        # B blocked + A FIRE-refused → pick None → return-home (Entry 061).
        h._bank.candidates[1].state = CandidateState.BLOCKED
        h._memory_mask = _mask_with_hits([GAP_B])
        nxt = h._pick_next_valid(live)
        self.assertIsNone(nxt)
        if nxt is None and h._return_home_on_all_blocked:
            h._begin_return_to_start()
        self.assertEqual(h._begin_return_calls, 1)

    def test_g6_pos_demo_only_unlock_empty_fire_cloud(self) -> None:
        """G6-POS / DB-U8: demo-only + empty fire cloud after CLEAR → unlock."""
        h = DualBlockHarness()
        h._bank = self._bank_ab()
        h._block_locus[1] = GAP_A
        h._ever_blocked_ids.add(1)
        h._demo_obstacle_poses = [GAP_A]
        h._set_operator_block_latch(1, GAP_A)
        h._block_source[1] = 'DEMO'
        h._fire_memory_centres = []
        h._memory_mask = _mask_with_hits([])
        live = _free_grid(PATH_A_PTS + PATH_B_PTS)
        msg = MagicMock()
        msg.poses = []
        h._on_demo_obstacle_poses(msg)
        h._try_evidence_unblock(live)
        self.assertEqual(h._bank.candidates[0].state, CandidateState.AVAILABLE)
        self.assertEqual(h._bank.candidates[0].block_reason, 'evidence_clear')

    def test_g6_off_centreline_fire_refuse(self) -> None:
        """Off-centreline FIRE within 0.80 m of unsliced Path A → refuse."""
        h = DualBlockHarness()
        h._bank = self._bank_ab()
        h._block_locus[1] = GAP_A
        h._ever_blocked_ids.add(1)
        # Never-demo: Entry 035 locus would unlock; FIRE cloud must still refuse.
        h._fire_memory_centres = [(-1.9, -11.5)]  # ~0.6 m lateral of gap_a
        h._memory_mask = _mask_with_hits([])
        live = _free_grid(PATH_A_PTS + PATH_B_PTS)
        hit = h._fire_cloud_intersects_corridor(PATH_A_PTS)
        self.assertIsNotNone(hit)
        h._try_evidence_unblock(live)
        self.assertEqual(h._bank.candidates[0].state, CandidateState.BLOCKED)

    def test_g6_lidar_t5_unlock_still_allowed(self) -> None:
        """Entry 035 T5: never-demo LIDAR residual far from locus → unlock OK."""
        h = DualBlockHarness()
        h._bank = self._bank_ab()
        h._block_locus[1] = GAP_A
        h._ever_blocked_ids.add(1)
        # No FIRE cloud; no ever-demo; locus disk clear; far residual ignored.
        h._fire_memory_centres = []
        h._memory_mask = _mask_with_hits([(5.0, -6.0)])  # far from locus
        live = _free_grid(PATH_A_PTS + PATH_B_PTS)
        h._try_evidence_unblock(live)
        self.assertEqual(h._bank.candidates[0].state, CandidateState.AVAILABLE)

    def test_g6_clear_does_not_wipe_fire_centres(self) -> None:
        """GUI CLEAR empties poses / latch but must not clear fire_memory cache."""
        h = DualBlockHarness()
        h._fire_memory_centres = [(-1.3, -12.2)]
        h._set_operator_block_latch(1, GAP_A)
        h._demo_obstacle_poses = [GAP_A]
        msg = MagicMock()
        msg.poses = []
        h._on_demo_obstacle_poses(msg)
        self.assertFalse(h._operator_block_latch_active(1))
        self.assertEqual(h._fire_memory_centres, [(-1.3, -12.2)])

    def test_a1_follow_path_ref_publish_and_clear(self) -> None:
        """A1/A3: accept-mirror publishes submitted Path; clear empties + DELETEALL."""
        from nav_msgs.msg import Path
        from geometry_msgs.msg import PoseStamped

        h = DualBlockHarness()
        path = Path()
        path.header.frame_id = 'map'
        for x, y in ((0.0, 0.0), (1.0, 0.0), (2.0, 0.5)):
            ps = PoseStamped()
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        h._last_submitted_path = path
        h._publish_follow_path_ref(path, epoch=7)
        self.assertEqual(h._follow_path_ref_epoch, 7)
        self.assertEqual(len(h._published_active), 1)
        self.assertEqual(len(h._published_active[0].poses), 3)
        h._clear_active_path()
        self.assertIsNone(h._last_submitted_path)
        self.assertEqual(h._follow_path_ref_epoch, -1)
        self.assertEqual(len(h._published_active[-1].poses), 0)
        self.assertTrue(h._published_markers)

    def test_db_u16_merge_keeps_failed_validation_blocked(self) -> None:
        """Entry 081: merge must not override validated BLOCKED → AVAILABLE."""
        h = DualBlockHarness()
        h._bank = PathBank(
            start=(-18.0, 3.0),
            goal=(16.0, -0.1),
            candidates=[
                _cand(1, PATH_A_PTS, score=100.0, state=CandidateState.ACTIVE),
            ],
            active_id=1,
        )
        h._route_bank = types.SimpleNamespace(
            routes=[_FakeRoute(1, PATH_A_PTS)],
            mission_salt='unit',
            by_path_id=lambda pid: (
                _FakeRoute(1, PATH_A_PTS) if pid == 1 else None
            ),
        )
        h._planning_grid = None
        h._live_costmap = object()
        h._candidate_route_valid = lambda cand, live: (False, 'unit_reject')
        h._publish_bank_paths = lambda: None
        expanded = PathBank(
            start=(-18.0, 3.0),
            goal=(16.0, -0.1),
            candidates=[
                _cand(1, PATH_A_PTS, score=100.0, state=CandidateState.AVAILABLE),
                _cand(2, PATH_B_PTS, score=90.0, state=CandidateState.AVAILABLE),
            ],
            active_id=1,
        )
        merge = types.MethodType(pbm.PathBankManager._merge_expanded_bank, h)
        with patch.object(pbm, 'nav2_costmap_to_grid', return_value=object()):
            merge(expanded)
        states = {c.path_id: c.state for c in h._bank.candidates}
        self.assertEqual(states[1], CandidateState.ACTIVE)
        self.assertEqual(states[2], CandidateState.BLOCKED)
        self.assertEqual(h._bank.active_id, 1)
        self.assertIn('merge_validation:', h._bank.candidates[1].block_reason)

    def test_stem_snap_helpers_shared_prefix_and_mid_corridor(self) -> None:
        """Stem-snap true mid-corridor on shared spawn prefix; false at spawn."""
        # Dense shared stem then diverge.
        path_a = [(-18.0, 3.0), (-14.0, 1.0), (-10.0, -2.0), (-1.3, -11.5), (16.0, -0.1)]
        path_b = [(-18.0, 3.0), (-14.0, 1.0), (-10.0, 6.0), (-5.1, 21.1), (16.0, -0.1)]
        # Densify lightly so nearest/arc helpers see many vertices.
        dens_a = []
        for a, b in zip(path_a, path_a[1:]):
            dens_a.append(a)
            dens_a.append(((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5))
        dens_a.append(path_a[-1])
        dens_b = []
        for a, b in zip(path_b, path_b[1:]):
            dens_b.append(a)
            dens_b.append(((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5))
        dens_b.append(path_b[-1])

        prefix_end = pbm.shared_prefix_end_index(dens_a, dens_b)
        self.assertGreaterEqual(prefix_end, 0)
        # Arc to prefix end must be comparable by arc, not raw resample index.
        prefix_arc = pbm.arc_length_to_index(dens_b, prefix_end)
        self.assertLessEqual(prefix_arc, pbm.STEM_PREFIX_MAX_ARC_M + 1e-6)

        # Robot still at spawn → not stem-snap.
        self.assertFalse(
            pbm.is_spawn_stem_snap(
                robot_xy=(-18.0, 3.0),
                bank_start=(-18.0, 3.0),
                target_points=dens_b,
                path_a_points=dens_a,
            )
        )
        # Robot mid Path A exclusive; Path B nearest is still on shared stem.
        self.assertTrue(
            pbm.is_spawn_stem_snap(
                robot_xy=(-1.3, -11.5),
                bank_start=(-18.0, 3.0),
                target_points=dens_b,
                path_a_points=dens_a,
            )
        )

    def test_homeward_progress_aware_nearest_prefers_mid_b(self) -> None:
        """Progress-aware nearest picks mid-B exclusive over geometrically closer stem."""
        # Stem vertices near spawn; exclusive mid-B farther along arc but near robot.
        stem = [(-18.0 + 0.5 * i, 3.0) for i in range(0, 9)]  # ~4 m stem
        exclusive = [(-14.0, 3.0 + 0.5 * i) for i in range(1, 21)]  # north exclusive
        pts = stem + exclusive
        # Robot near exclusive mid; stem start is farther than window but a stem
        # vertex may still be closer than a late exclusive if we used global nearest
        # incorrectly after travel — place robot at exclusive mid.
        rx, ry = -14.0, 8.0
        h = DualBlockHarness()
        nearest_fn = types.MethodType(
            pbm.PathBankManager._nearest_route_index_progress_aware, h)
        idx = nearest_fn(pts, rx, ry, window_m=2.0)
        # Must not land in the first few stem indices.
        self.assertGreater(idx, 8)
        self.assertLess(
            abs(pts[idx][0] - rx) + abs(pts[idx][1] - ry),
            3.0,
        )

    def test_db_u17_proximal_keepout_invalidates_active(self) -> None:
        """Entry 082: keepout under nearest sample invalidates despite +1 skip."""
        h = DualBlockHarness()
        h._use_follow_path = True
        # Dense route: robot on sample 2; keepout on sample 2 (under robot).
        dens = [(-20.0 - 0.5 * i, -1.0) for i in range(0, 12)]
        barrier = dens[2]
        h._robot_xyyaw = (barrier[0], barrier[1], 0.0)
        h._route_bank = _FakeRouteBank({1: _FakeRoute(1, dens)})
        h._bank = PathBank(
            start=dens[0],
            goal=dens[-1],
            candidates=[
                _cand(1, dens, score=100.0, state=CandidateState.ACTIVE),
            ],
            active_id=1,
        )
        h._memory_mask = _mask_with_hits([barrier])
        live = _free_grid(dens)
        ok, reason = h._active_route_valid(h._bank.candidates[0], live)
        self.assertFalse(ok, 'proximal keepout must invalidate active route')
        self.assertEqual(reason, 'obstacle_memory_mask')
        # FollowPath publish slice may still skip under-robot sample.
        fwd, _ = h._route_forward_from_robot(h._route_bank.by_path_id(1))
        self.assertNotEqual(fwd[0], dens[2])

    def test_db_u18_bank_centreline_lethal_near_robot(self) -> None:
        """Entry 082: bank centreline lethal within horizon invalidates densify dodge."""
        h = DualBlockHarness()
        h._use_follow_path = True
        bank_pts = [(-21.0, -2.0), (-20.5, -1.5), (-20.0, -1.0), (-18.0, 3.0)]
        # Densified dodge skips the lethal corridor cell.
        dens = [(-20.7, -0.5), (-20.2, 0.0), (-19.0, 1.5), (-18.0, 3.0)]
        h._robot_xyyaw = (-20.6, -1.4, 0.0)
        h._route_bank = _FakeRouteBank({1: _FakeRoute(1, dens)})
        h._bank = PathBank(
            start=bank_pts[0],
            goal=bank_pts[-1],
            candidates=[
                _cand(1, bank_pts, score=100.0, state=CandidateState.ACTIVE),
            ],
            active_id=1,
        )
        live = _free_grid(bank_pts + dens)
        # Paint lethal on bank centreline near robot.
        ix = live.world_to_grid(-20.5, -1.5)
        self.assertIsNotNone(ix)
        live.costs[ix[1], ix[0]] = 254  # harness lethal_threshold=254
        h._memory_mask = None
        ok, reason = h._active_route_valid(h._bank.candidates[0], live)
        self.assertFalse(ok, 'near-robot bank lethal must invalidate')
        self.assertIn('lethal', reason)

    def test_db_u19_early_horizon_invalidates_at_6m(self) -> None:
        """Entry 083 1A: keepout ~6 m ahead invalidates via EARLY_SWITCH_HORIZON_M.

        memory_block_horizon_m stays 5.0 (would miss); dedicated early horizon
        8.0 must mark active invalid so switch can arm before seating.
        """
        self.assertEqual(pbm.EARLY_SWITCH_HORIZON_M, 8.0)
        self.assertEqual(pbm.SWITCH_STANDOFF_MIN_M, 4.0)
        self.assertEqual(pbm.KEEPOUT_SEAT_DISK_M, 1.5)
        h = DualBlockHarness()
        h._use_follow_path = True
        h._memory_block_horizon_m = 5.0  # must remain; early horizon is dedicated
        # Straight corridor east; robot at origin; locus ~6 m ahead.
        pts = [(float(i), 0.0) for i in range(0, 12)]
        locus = (6.0, 0.0)
        h._robot_xyyaw = (0.0, 0.0, 0.0)
        h._route_bank = _FakeRouteBank({1: _FakeRoute(1, pts)})
        h._bank = PathBank(
            start=pts[0],
            goal=pts[-1],
            candidates=[
                _cand(1, pts, score=100.0, state=CandidateState.ACTIVE),
                _cand(2, [(0.0, 2.0), (11.0, 2.0)], score=50.0),
            ],
            active_id=1,
        )
        h._memory_mask = _mask_with_hits([locus])
        live = _free_grid(pts)
        # Baseline: 5 m ahead gate would miss a 6 m locus.
        self.assertFalse(
            h._memory_intersects_path_ahead(list(pts), horizon_m=5.0),
            '5 m horizon must not see 6 m locus (gui_stall contract)',
        )
        self.assertTrue(
            h._memory_intersects_path_ahead(
                list(pts), horizon_m=pbm.EARLY_SWITCH_HORIZON_M
            ),
            '8 m early horizon must see 6 m locus',
        )
        ok, reason = h._active_route_valid(h._bank.candidates[0], live)
        self.assertFalse(ok, 'active must invalidate at ~6 m keepout')
        self.assertEqual(reason, 'obstacle_memory_mask')

    @staticmethod
    def _densify_polyline(pts: List[Point], n_mid: int = 3) -> List[Point]:
        out: List[Point] = []
        for a, b in zip(pts, pts[1:]):
            out.append(a)
            for k in range(1, n_mid + 1):
                t = k / (n_mid + 1)
                out.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
        out.append(pts[-1])
        return out

    def _stem_mid_corridor_routes(self):
        """Dense A/B with shared spawn stem; robot mid-A exclusive → stem-snap."""
        path_a = [
            (-18.0, 3.0), (-14.0, 1.0), (-10.0, -2.0), (-1.3, -11.5), (16.0, -0.1),
        ]
        path_b = [
            (-18.0, 3.0), (-14.0, 1.0), (-10.0, 6.0), (-5.1, 21.1), (16.0, -0.1),
        ]
        dens_a = self._densify_polyline(path_a, n_mid=7)
        dens_b = self._densify_polyline(path_b, n_mid=7)
        return dens_a, dens_b

    def _stem_refuse_harness(self, dens_a, dens_b, *, robot_xy=(-1.3, -11.5)):
        h = DualBlockHarness()
        h._use_follow_path = True
        h._robot_xyyaw = (float(robot_xy[0]), float(robot_xy[1]), 0.0)
        h._bank = PathBank(
            start=(-18.0, 3.0),
            goal=(16.0, -0.1),
            candidates=[
                _cand(1, dens_a, score=100.0, state=CandidateState.BLOCKED),
                _cand(2, dens_b, score=90.0, state=CandidateState.ACTIVE),
            ],
            active_id=2,
        )
        h._route_bank = _FakeRouteBank(
            {1: _FakeRoute(1, dens_a), 2: _FakeRoute(2, dens_b)}
        )
        h._owner = types.SimpleNamespace(generation=1)
        h._goal_xyyaw = (16.0, -0.1, 0.0)
        h._stem_snap_bridge_fails = 0
        h._far_gap_rebuild_budget = 3
        h._far_gap_rebuilds = 0
        h._bridge_exhaust_drive_retries = 0
        h._drive_escape_used = False
        h._splice_pending = False
        h._switch_pending = False
        h._switch_join_queue = None
        h._switch_join_attempt = 0
        h._switch_goal_fallback_used = False
        h._switch_bridge_goal_only = False
        h._switch_escape_only = False
        h._fp_goal_handle = None
        h._stuck_budget_n = 8
        h._stuck_recoveries = 0
        h._stuck_timeout_s = 8.0
        h._stuck_recover_until = 0.0
        h._fp_client = MagicMock()
        h._fp_client.server_is_ready.return_value = True
        h._observation_hold = False
        h._state = pbm.MissionNavState.FOLLOWING_PATH
        # Bind Entry 084 helpers + async callbacks + exhaust/send gate.
        for name in (
            '_bank_start_xy',
            '_path_a_points_for_stem',
            '_route_is_spawn_stem_snap',
            '_refuse_bare_stem_follow_path',
            '_nearest_route_pose',
            '_join_candidates',
            '_switch_bridge_goal_response',
            '_switch_bridge_result',
            '_on_switch_bridge_exhausted',
            '_send_follow_path',
            '_try_next_switch_bridge',
        ):
            setattr(
                h, name, types.MethodType(getattr(pbm.PathBankManager, name), h)
            )
        h._arm_nav_authority = lambda: True
        h._route_forward_from_robot = MagicMock(
            return_value=(dens_b[:2], (0.0, 0.0))
        )
        h._send_follow_path_msg = MagicMock()
        h._request_switch_navfn_bridge = MagicMock(return_value=False)
        h._request_switch_navfn_goal_fallback = MagicMock(return_value=False)
        h._request_drive_on_heading_escape = MagicMock(return_value=False)
        h._maybe_owner_stuck_recover = MagicMock()
        h._send_follow_path_calls = []
        real_send = h._send_follow_path

        def _tracking_send(*args, **kwargs):
            h._send_follow_path_calls.append((args, kwargs))
            return real_send(*args, **kwargs)

        h._send_follow_path = _tracking_send
        return h

    def test_db_u20_async_reject_stem_no_densify(self) -> None:
        """Entry 084: async reject under stem → no densify send; Inc once."""
        dens_a, dens_b = self._stem_mid_corridor_routes()
        h = self._stem_refuse_harness(dens_a, dens_b)
        self.assertTrue(h._route_is_spawn_stem_snap(h._route_bank.by_path_id(2)))
        route = h._route_bank.by_path_id(2)
        fut = MagicMock()
        fut.result.return_value = None  # rejected / not accepted
        exhaust = MagicMock(wraps=h._on_switch_bridge_exhausted)
        h._on_switch_bridge_exhausted = exhaust
        h._send_follow_path_calls.clear()
        # Patch tracking around refuse path: async should not call send.
        send_spy = MagicMock()
        h._send_follow_path = send_spy
        h._switch_bridge_goal_response(fut, 1, route)
        send_spy.assert_not_called()
        self.assertEqual(h._stem_snap_bridge_fails, 1)
        exhaust.assert_called_once()
        kwargs = exhaust.call_args.kwargs
        self.assertTrue(kwargs.get('stem_snap'))
        self.assertTrue(
            any('STEM_SNAP_BARE_FP_REFUSED' in w for w in h._logger.warns),
            h._logger.warns,
        )

    def test_db_u21_async_status_fail_stem_gap_le5_no_densify(self) -> None:
        """Entry 084: stem + status fail + gap≤5 → no densify; exhaust stem."""
        dens_a, dens_b = self._stem_mid_corridor_routes()
        h = self._stem_refuse_harness(dens_a, dens_b)
        route = h._route_bank.by_path_id(2)
        # Force measured nearest-route gap ≤5 while stem remains True.
        h._nearest_route_pose = MagicMock(
            return_value=(h._robot_xyyaw[0] + 1.0, h._robot_xyyaw[1], 0.0, 0)
        )
        # Join queue exhausted immediately.
        h._switch_join_queue = []
        h._switch_join_attempt = 0
        h._switch_goal_fallback_used = True
        send_spy = MagicMock()
        h._send_follow_path = send_spy
        exhaust = MagicMock(wraps=types.MethodType(
            pbm.PathBankManager._on_switch_bridge_exhausted, h))
        h._on_switch_bridge_exhausted = exhaust
        fut = MagicMock()
        wrapped = types.SimpleNamespace(status=GoalStatus.STATUS_ABORTED, result=None)
        fut.result.return_value = wrapped
        # Bind result callback with patched helpers already on h.
        h._switch_bridge_result(fut, 1, route)
        send_spy.assert_not_called()
        self.assertEqual(h._stem_snap_bridge_fails, 1)
        exhaust.assert_called()
        self.assertTrue(exhaust.call_args.kwargs.get('stem_snap'))

    def test_db_u22_central_gate_never_orphan_following(self) -> None:
        """Entry 084: stem central gate leaves PATH_BLOCKED+pending or exhaust."""
        dens_a, dens_b = self._stem_mid_corridor_routes()
        h = self._stem_refuse_harness(dens_a, dens_b)
        route = h._route_bank.by_path_id(2)
        h._state = pbm.MissionNavState.FOLLOWING_PATH
        h._fp_goal_handle = None
        # No bridge flags → SYNC_SWITCH_ARM terminal via CENTRAL_GATE_OTHER.
        h._send_follow_path(
            route,
            refuse_context=pbm.RefuseContext.CENTRAL_GATE_OTHER,
        )
        self.assertNotEqual(
            (h._state, h._fp_goal_handle),
            (pbm.MissionNavState.FOLLOWING_PATH, None),
            'must not remain FOLLOWING_PATH without fp handle',
        )
        orphan = (
            h._state == pbm.MissionNavState.FOLLOWING_PATH
            and h._fp_goal_handle is None
        )
        self.assertFalse(orphan)
        self.assertEqual(h._state, pbm.MissionNavState.PATH_BLOCKED)
        self.assertTrue(h._switch_pending)
        self.assertEqual(h._stem_snap_bridge_fails, 1)
        h._send_follow_path_msg.assert_not_called()

    def test_db_u23_sync_r1_path_blocked_and_budget(self) -> None:
        """Entry 084 R1: sync refuse → PATH_BLOCKED+pending; budget marks path."""
        dens_a, dens_b = self._stem_mid_corridor_routes()
        h = self._stem_refuse_harness(dens_a, dens_b)
        route = h._route_bank.by_path_id(2)
        h._far_gap_rebuild_budget = 2
        h._refuse_bare_stem_follow_path(
            route, generation=1, context=pbm.RefuseContext.SYNC_SWITCH_ARM)
        self.assertEqual(h._state, pbm.MissionNavState.PATH_BLOCKED)
        self.assertTrue(h._switch_pending)
        self.assertEqual(h._stem_snap_bridge_fails, 1)
        h._refuse_bare_stem_follow_path(
            route, generation=1, context=pbm.RefuseContext.SYNC_SWITCH_ARM)
        # Budget hit → counter reset + stem_snap_bridge_exhausted mark.
        self.assertEqual(h._stem_snap_bridge_fails, 0)
        cand = h._bank.by_path_id(2) if hasattr(h._bank, 'by_path_id') else None
        if cand is None:
            cand = next(c for c in h._bank.candidates if c.path_id == 2)
        self.assertEqual(cand.state, CandidateState.BLOCKED)
        self.assertEqual(cand.block_reason, 'stem_snap_bridge_exhausted')

    def test_db_u24_near_spawn_bare_fp_allowed(self) -> None:
        """Entry 084: robot within STEM_ROBOT_AWAY_M → not stem; bare FP OK."""
        dens_a, dens_b = self._stem_mid_corridor_routes()
        h = self._stem_refuse_harness(
            dens_a, dens_b, robot_xy=(-18.0, 3.0)  # at bank start
        )
        self.assertFalse(h._route_is_spawn_stem_snap(h._route_bank.by_path_id(2)))
        route = h._route_bank.by_path_id(2)
        # Short forward slice near spawn — densify/gap path may still run;
        # ensure central gate does not refuse.
        h._route_forward_from_robot = MagicMock(
            return_value=(dens_b[:3], (0.0, 0.0, 0.0))
        )
        h._send_follow_path(route)
        self.assertEqual(h._stem_snap_bridge_fails, 0)
        self.assertNotEqual(h._state, pbm.MissionNavState.PATH_BLOCKED)
        h._send_follow_path_msg.assert_called()


if __name__ == '__main__':
    unittest.main()
