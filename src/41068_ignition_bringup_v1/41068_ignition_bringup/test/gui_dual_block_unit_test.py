#!/usr/bin/env python3
"""Unit coverage for Entry 062 GUI dual-block Wave-1 (DB-U1–U10).

Binds real ``PathBankManager`` methods onto a ROS-free harness so unlock /
pick / resume gates are exercised without Gazebo.
"""

from __future__ import annotations

import math
import os
import sys
import types
import unittest
from typing import List, Optional, Tuple
from unittest.mock import MagicMock

import numpy as np

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
from rs1_nav.route_instance import make_route_instance

# Import after path setup; requires rclpy (Humble) but not a spinning node.
import path_bank_manager as pbm  # noqa: E402


Point = Tuple[float, float]


def test_switch_join_candidates_do_not_select_points_behind_robot():
    """An alternate corridor must not make a switch command reverse travel."""
    manager = pbm.PathBankManager.__new__(pbm.PathBankManager)
    manager._robot_xyyaw = (5.0, 0.0, 0.0)
    manager._goal_xyyaw = (10.0, 0.0, 0.0)
    route = make_route_instance(
        2,
        [(0.0, 2.0), (5.0, 2.0), (10.0, 0.0)],
        densify_m=0.5,
    )

    joins = manager._join_candidates(route)

    assert joins
    assert all(
        math.hypot(x - 10.0, y) < math.hypot(5.0, 0.0) - 0.25
        for x, y, _yaw, _index in joins
    )
    assert all(x >= 4.5 for x, _y, _yaw, _index in joins)


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
            '_prior_block_locus_behind_forward_slice',
            '_candidate_route_valid',
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


if __name__ == '__main__':
    unittest.main()
