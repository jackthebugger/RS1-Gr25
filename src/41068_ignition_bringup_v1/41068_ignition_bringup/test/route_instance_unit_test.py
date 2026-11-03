#!/usr/bin/env python3
"""Unit tests for RouteInstance densify/yaw/oriented footprint (§E)."""

from __future__ import annotations

import math
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.abspath(os.path.join(HERE, '..'))
if PKG not in sys.path:
    sys.path.insert(0, PKG)

from rs1_nav.path_bank import (  # noqa: E402
    OccupancyGrid2D,
    nudge_polyline_away_from_lethal,
    path_is_valid,
)
from rs1_nav.route_instance import (  # noqa: E402
    along_track_yaw,
    centreline_clear_body_lethal_reject,
    densify_polyline,
    geometry_fingerprint,
    make_route_instance,
    oriented_footprint_path_valid,
)


def _grid_with_side_lethal() -> OccupancyGrid2D:
    """Corridor along x with lethal strip at y=+0.5 (body hits, centreline clear)."""
    res = 0.05
    width, height = 80, 40  # 4m x 2m
    costs = np.zeros((height, width), dtype=np.uint8)
    # Lethal band y in [0.35, 0.70]
    for ix in range(width):
        for iy in range(height):
            y = iy * res - 1.0
            if 0.35 <= y <= 0.70:
                costs[iy, ix] = 254
    return OccupancyGrid2D(
        origin_x=0.0, origin_y=-1.0, resolution=res,
        width=width, height=height, costs=costs,
    )


class RouteInstanceTests(unittest.TestCase):
    def test_densify_and_yaw(self) -> None:
        pts = [(0.0, 0.0), (1.0, 0.0)]
        dense = densify_polyline(pts, max_step_m=0.10)
        self.assertGreaterEqual(len(dense), 10)
        yaws = along_track_yaw(dense)
        self.assertEqual(len(yaws), len(dense))
        self.assertAlmostEqual(yaws[0], 0.0, places=5)

    def test_stable_fingerprint_and_uid(self) -> None:
        a = make_route_instance(1, [(0.0, 0.0), (2.0, 0.0)], mission_salt='m1')
        b = make_route_instance(1, [(0.0, 0.0), (2.0, 0.0)], mission_salt='m1')
        self.assertEqual(a.geometry_fingerprint, b.geometry_fingerprint)
        self.assertEqual(a.route_uid, b.route_uid)
        c = make_route_instance(1, [(0.0, 0.0), (2.0, 0.1)], mission_salt='m1')
        self.assertNotEqual(a.geometry_fingerprint, c.geometry_fingerprint)

    def test_oriented_rejects_body_lethal_centreline_clear(self) -> None:
        grid = _grid_with_side_lethal()
        # Centreline along y=0 — clear of lethal band at y>=0.35
        points = [(0.2 + 0.1 * i, 0.0) for i in range(20)]
        yaws = [0.0] * len(points)
        ok_c, _ = path_is_valid(points, grid, lethal_threshold=254)
        self.assertTrue(ok_c, 'centreline should be clear')
        reject, reason = centreline_clear_body_lethal_reject(
            points, yaws, grid, lethal_threshold=254)
        self.assertTrue(reject, reason)
        ok_o, reason_o = oriented_footprint_path_valid(
            points, yaws, grid, lethal_threshold=254)
        self.assertFalse(ok_o, reason_o)

    def test_fingerprint_helper(self) -> None:
        fp = geometry_fingerprint([(1.0, 2.0), (3.0, 4.0)])
        self.assertEqual(len(fp), 16)

    def test_c1_pose0_only_lethal_forward_slice_accepts(self) -> None:
        """C1 oracle: lethal only at pose-0; forward slice (skip under-robot) accepts.

        Use 1 m densify so pose-1 footprint does not still overlap the pose-0
        lethal cell (half_l+pad≈0.65 m).
        """
        res = 0.05
        width, height = 160, 80  # 8 m × 4 m
        costs = np.zeros((height, width), dtype=np.uint8)
        ox, oy = 0.0, -2.0
        # Single lethal cell at pose-0 centreline.
        ix0 = int((1.0 - ox) / res)
        iy0 = int((0.0 - oy) / res)
        costs[iy0, ix0] = 254
        grid = OccupancyGrid2D(
            origin_x=ox, origin_y=oy, resolution=res,
            width=width, height=height, costs=costs,
        )
        route = make_route_instance(
            0, [(1.0, 0.0), (5.0, 0.0)], label='home', densify_m=1.0)
        ok_full, reason_full = oriented_footprint_path_valid(
            route.points, route.yaw_rad, grid, lethal_threshold=254)
        # Mirror _route_forward_from_robot: robot at first pose → i0 = 1.
        pts_fwd = route.points[1:]
        yaws_fwd = route.yaw_rad[1:]
        ok_fwd, reason_fwd = oriented_footprint_path_valid(
            pts_fwd, yaws_fwd, grid, lethal_threshold=254)
        self.assertFalse(ok_full, f'full path should see pose-0 lethal ({reason_full})')
        self.assertTrue(
            ok_fwd,
            f'forward slice must accept pose-0-only lethal; got {reason_fwd}',
        )

    def test_c1_midpath_lethal_forward_slice_rejects(self) -> None:
        """C1 oracle: mid-path lethal on forward slice → reject."""
        res = 0.05
        width, height = 160, 80
        costs = np.zeros((height, width), dtype=np.uint8)
        ox, oy = 0.0, -2.0
        # Lethal band around x≈3.0 (mid corridor on forward slice).
        for ix in range(width):
            for iy in range(height):
                x = ox + (ix + 0.5) * res
                if 2.7 <= x <= 3.3:
                    costs[iy, ix] = 254
        grid = OccupancyGrid2D(
            origin_x=ox, origin_y=oy, resolution=res,
            width=width, height=height, costs=costs,
        )
        route = make_route_instance(
            0, [(1.0, 0.0), (5.0, 0.0)], label='home', densify_m=0.10)
        pts_fwd = route.points[1:]
        yaws_fwd = route.yaw_rad[1:]
        ok_fwd, reason_fwd = oriented_footprint_path_valid(
            pts_fwd, yaws_fwd, grid, lethal_threshold=254)
        self.assertFalse(ok_fwd, f'mid-path lethal must reject; got {reason_fwd}')
        self.assertIn('lethal', reason_fwd)

    def test_home_nudge_recovers_knife_edge_oriented(self) -> None:
        """GUI-DB home: NavFn knife-edge fails C1; Entry-034 nudge recovers.

        Dense parallel polyline offset toward a corridor wall so oriented body
        clips while centreline stays free. Production home build densifies →
        nudges densified verts → refreshes yaw **without** re-densify.
        """
        res = 0.05
        width, height = 280, 200  # 14 m × 10 m
        costs = np.zeros((height, width), dtype=np.uint8)
        ox, oy = -2.0, -5.0
        wall = 1.50
        for ix in range(width):
            for iy in range(height):
                y = oy + (iy + 0.5) * res
                if y >= wall or y <= -wall:
                    costs[iy, ix] = 254
        grid = OccupancyGrid2D(
            origin_x=ox, origin_y=oy, resolution=res,
            width=width, height=height, costs=costs,
        )
        # Dense samples; short endpoint ramps avoid steep-yaw chords.
        raw = []
        for i in range(55):
            x = 0.5 + 0.1 * i
            y = 0.0 if (i < 3 or i > 51) else 1.05
            raw.append((x, y))
        route_raw = make_route_instance(
            0, raw, label='home_raw', densify_m=0.10)
        ok_raw, reason_raw = oriented_footprint_path_valid(
            route_raw.points, route_raw.yaw_rad, grid, lethal_threshold=254)
        self.assertFalse(
            ok_raw,
            f'knife-edge must fail oriented before nudge; got {reason_raw}',
        )
        nudged = nudge_polyline_away_from_lethal(
            list(route_raw.points),
            grid,
            target_clearance_m=0.85,
            max_offset_m=1.5,
        )
        # Match production: yaw refresh only (no re-densify chords).
        route_n = make_route_instance(
            0, nudged, label='home_nudged', densify_m=1.0e6)
        ok_n, reason_n = oriented_footprint_path_valid(
            route_n.points, route_n.yaw_rad, grid, lethal_threshold=254)
        self.assertTrue(
            ok_n,
            f'nudge must recover oriented home path; got {reason_n}',
        )

    def test_home_nudge_does_not_bypass_sealed_midpath(self) -> None:
        """Sealed keepout across corridor still fails C1 after nudge."""
        res = 0.05
        width, height = 280, 200
        costs = np.zeros((height, width), dtype=np.uint8)
        ox, oy = -2.0, -5.0
        wall = 1.50
        for ix in range(width):
            for iy in range(height):
                x = ox + (ix + 0.5) * res
                y = oy + (iy + 0.5) * res
                if y >= wall or y <= -wall:
                    costs[iy, ix] = 254
                if 3.7 <= x <= 4.3 and -wall < y < wall:
                    costs[iy, ix] = 254
        grid = OccupancyGrid2D(
            origin_x=ox, origin_y=oy, resolution=res,
            width=width, height=height, costs=costs,
        )
        raw = [(0.5 + 0.1 * i, 0.0) for i in range(60)]
        route_raw = make_route_instance(
            0, raw, label='home_sealed_raw', densify_m=0.10)
        nudged = nudge_polyline_away_from_lethal(
            list(route_raw.points),
            grid,
            target_clearance_m=0.65,
            max_offset_m=1.0,
        )
        route = make_route_instance(
            0, nudged, label='home_sealed', densify_m=1.0e6)
        ok, reason = oriented_footprint_path_valid(
            route.points, route.yaw_rad, grid, lethal_threshold=254)
        self.assertFalse(ok, f'sealed mid-path must still reject; got {reason}')
        self.assertIn('lethal', reason)


if __name__ == '__main__':
    unittest.main()
