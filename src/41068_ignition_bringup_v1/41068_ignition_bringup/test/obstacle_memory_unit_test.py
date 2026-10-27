#!/usr/bin/env python3
"""Unit tests for sensor-only obstacle memory clear predicates (no Gazebo).

    python3 test/obstacle_memory_unit_test.py
"""

from __future__ import annotations

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, PKG)

from rs1_nav.obstacle_memory import (  # noqa: E402
    CellSource,
    ClearResult,
    MapMeta,
    ObstacleMemory,
    ObstacleMemoryConfig,
)
from rs1_nav.path_bank import CandidateState, PathBank, PathCandidate  # noqa: E402


def _mem(res: float = 0.05, w: int = 200, h: int = 200) -> ObstacleMemory:
    meta = MapMeta(origin_x=-5.0, origin_y=-5.0, resolution=res, width=w, height=h)
    cfg = ObstacleMemoryConfig(
        max_scan_age_s=0.5,
        clear_max_range_m=10.0,
        raytrace_max_m=10.0,
        range_epsilon_m=0.15,
        mark_neighbour_radius_cells=0,
        fire_clear_confirm_scans=3,
        thermal_cone_half_angle_rad=0.6,
        thermal_max_range_m=8.0,
    )
    return ObstacleMemory(meta=meta, cfg=cfg)


def _scan_toward(dist: float, n: int = 360) -> tuple:
    """360-beam scan; one finite return at angle 0 of length dist; rest inf."""
    ranges = [float('inf')] * n
    # Put finite return near angle 0 (index 0).
    ranges[0] = dist
    angle_min = -math.pi
    angle_inc = 2.0 * math.pi / n
    # Shift so index for bearing 0 is the finite beam: angle_min + i*inc ≈ 0
    # i = -angle_min / inc = n/2
    ranges = [float('inf')] * n
    mid = n // 2
    ranges[mid] = dist
    return ranges, angle_min, angle_inc, 0.05, 30.0


def test_mark_and_mask() -> list:
    fails = []
    m = _mem()
    m.mark_lidar_hit(0.0, 0.0, now=1.0)
    if m.occupied_count() < 1:
        fails.append('mark_lidar_hit did not add cell')
    data = m.build_occupancy_data()
    if not (data == 100).any():
        fails.append('mask missing occupied=100')
    if (data == 0).any():
        fails.append('mask must not publish free=0 for unknown')
    if not (data == -1).any():
        fails.append('mask should leave unknowns as -1')
    print('PASS  mark + mask semantics' if not fails else 'FAIL  mark + mask')
    return fails


def test_stale_scan_no_clear() -> list:
    fails = []
    m = _mem()
    m.mark_lidar_hit(2.0, 0.0, now=1.0)
    idx = m.meta.world_to_grid(2.0, 0.0)
    assert idx is not None
    ranges, amin, ainc, rmin, rmax = _scan_toward(10.0)  # free past cell
    result = m.try_clear_lidar_cell(
        *idx,
        ranges=ranges, angle_min=amin, angle_increment=ainc,
        range_min=rmin, range_max=rmax,
        origin_xy=(0.0, 0.0), origin_yaw=0.0,
        scan_age_s=2.0, tf_ok=True,
    )
    if result != ClearResult.STALE:
        fails.append(f'expected STALE, got {result}')
    if not m.has_cell(*idx):
        fails.append('stale scan cleared cell')
    result2 = m.try_clear_lidar_cell(
        *idx,
        ranges=ranges, angle_min=amin, angle_increment=ainc,
        range_min=rmin, range_max=rmax,
        origin_xy=(0.0, 0.0), origin_yaw=0.0,
        scan_age_s=0.1, tf_ok=False,
    )
    if result2 != ClearResult.STALE:
        fails.append(f'expected STALE on tf fail, got {result2}')
    print('PASS  stale/TF no clear' if not fails else 'FAIL  stale/TF')
    return fails


def test_occlusion_rejects() -> list:
    fails = []
    m = _mem()
    # Cell at 4 m; nearer hit at 2 m on same beam.
    m.mark_lidar_hit(4.0, 0.0, now=1.0)
    idx = m.meta.world_to_grid(4.0, 0.0)
    assert idx is not None
    ranges, amin, ainc, rmin, rmax = _scan_toward(2.0)
    result = m.try_clear_lidar_cell(
        *idx,
        ranges=ranges, angle_min=amin, angle_increment=ainc,
        range_min=rmin, range_max=rmax,
        origin_xy=(0.0, 0.0), origin_yaw=0.0,
        scan_age_s=0.0, tf_ok=True,
    )
    if result != ClearResult.REJECTED:
        fails.append(f'occlusion expected REJECTED, got {result}')
    if not m.has_cell(*idx):
        fails.append('occlusion cleared cell behind nearer hit')
    print('PASS  occlusion rejects' if not fails else 'FAIL  occlusion')
    return fails


def test_hit_at_cell_rejects() -> list:
    fails = []
    m = _mem()
    m.mark_lidar_hit(3.0, 0.0, now=1.0)
    idx = m.meta.world_to_grid(3.0, 0.0)
    assert idx is not None
    ranges, amin, ainc, rmin, rmax = _scan_toward(3.0)
    result = m.try_clear_lidar_cell(
        *idx,
        ranges=ranges, angle_min=amin, angle_increment=ainc,
        range_min=rmin, range_max=rmax,
        origin_xy=(0.0, 0.0), origin_yaw=0.0,
        scan_age_s=0.0, tf_ok=True,
    )
    if result != ClearResult.REJECTED:
        fails.append(f'hit-at-cell expected REJECTED, got {result}')
    if not m.has_cell(*idx):
        fails.append('hit-at-cell cleared occupied memory')
    print('PASS  hit-at-cell rejects' if not fails else 'FAIL  hit-at-cell')
    return fails


def test_free_ray_accepts() -> list:
    fails = []
    m = _mem()
    m.mark_lidar_hit(2.0, 0.0, now=1.0)
    idx = m.meta.world_to_grid(2.0, 0.0)
    assert idx is not None
    # Free ray (inf) through cell.
    ranges, amin, ainc, rmin, rmax = _scan_toward(float('inf'))
    ranges = [float('inf')] * len(ranges)
    result = m.try_clear_lidar_cell(
        *idx,
        ranges=ranges, angle_min=amin, angle_increment=ainc,
        range_min=rmin, range_max=rmax,
        origin_xy=(0.0, 0.0), origin_yaw=0.0,
        scan_age_s=0.0, tf_ok=True,
    )
    if result != ClearResult.ACCEPTED:
        fails.append(f'free ray expected ACCEPTED, got {result}')
    if m.has_cell(*idx):
        fails.append('free ray did not remove cell')
    print('PASS  free-ray clear' if not fails else 'FAIL  free-ray')
    return fails


def test_hit_behind_accepts() -> list:
    fails = []
    m = _mem()
    m.mark_lidar_hit(2.0, 0.0, now=1.0)
    idx = m.meta.world_to_grid(2.0, 0.0)
    assert idx is not None
    ranges, amin, ainc, rmin, rmax = _scan_toward(5.0)  # hit behind cell
    result = m.try_clear_lidar_cell(
        *idx,
        ranges=ranges, angle_min=amin, angle_increment=ainc,
        range_min=rmin, range_max=rmax,
        origin_xy=(0.0, 0.0), origin_yaw=0.0,
        scan_age_s=0.0, tf_ok=True,
    )
    if result != ClearResult.ACCEPTED:
        fails.append(f'hit-behind expected ACCEPTED, got {result}')
    print('PASS  hit-behind clear' if not fails else 'FAIL  hit-behind')
    return fails


def test_unrelated_beam() -> list:
    fails = []
    m = _mem()
    m.mark_lidar_hit(2.0, 0.0, now=1.0)
    idx = m.meta.world_to_grid(2.0, 0.0)
    assert idx is not None
    # Robot looking +Y; cell is on +X → out of useful clear (still in 360 FOV
    # but beam toward cell is +X with free ray — that WOULD clear). Use a
    # narrow FOV scan that does not cover +X.
    ranges = [1.0] * 10  # all short hits in a narrow rear FOV
    amin = 2.5
    ainc = 0.05
    result = m.try_clear_lidar_cell(
        *idx,
        ranges=ranges, angle_min=amin, angle_increment=ainc,
        range_min=0.05, range_max=30.0,
        origin_xy=(0.0, 0.0), origin_yaw=0.0,
        scan_age_s=0.0, tf_ok=True,
    )
    if result != ClearResult.OUT_OF_FOV:
        fails.append(f'unrelated FOV expected OUT_OF_FOV, got {result}')
    if not m.has_cell(*idx):
        fails.append('unrelated observation cleared cell X')
    print('PASS  unrelated FOV' if not fails else 'FAIL  unrelated FOV')
    return fails


def test_lidar_does_not_clear_fire() -> list:
    fails = []
    m = _mem()
    m.mark_fire_hit(2.0, 0.0, now=1.0)
    idx = m.meta.world_to_grid(2.0, 0.0)
    assert idx is not None
    ranges, amin, ainc, rmin, rmax = _scan_toward(float('inf'))
    ranges = [float('inf')] * len(ranges)
    result = m.try_clear_lidar_cell(
        *idx,
        ranges=ranges, angle_min=amin, angle_increment=ainc,
        range_min=rmin, range_max=rmax,
        origin_xy=(0.0, 0.0), origin_yaw=0.0,
        scan_age_s=0.0, tf_ok=True,
    )
    if result != ClearResult.WRONG_SOURCE:
        fails.append(f'lidar→fire expected WRONG_SOURCE, got {result}')
    if m.cell_source(*idx) != CellSource.FIRE:
        fails.append('FIRE cell source changed by lidar clear attempt')
    print('PASS  lidar does not clear FIRE' if not fails else 'FAIL  fire cross-clear')
    return fails


def test_fire_clear_policy() -> list:
    fails = []
    m = _mem()
    m.mark_fire_hit(2.0, 0.0, now=1.0)
    idx = m.meta.world_to_grid(2.0, 0.0)
    assert idx is not None

    # Hold active → no clear.
    r = m.try_clear_fire_cell(
        *idx, fire_detected=False, hold_active=True,
        origin_xy=(0.0, 0.0), origin_yaw=0.0, observation_fresh=True,
    )
    if r != ClearResult.HOLD_ACTIVE:
        fails.append(f'hold expected HOLD_ACTIVE, got {r}')

    # Need N consecutive in-FOV negatives.
    for i in range(m.cfg.fire_clear_confirm_scans - 1):
        r = m.try_clear_fire_cell(
            *idx, fire_detected=False, hold_active=False,
            origin_xy=(0.0, 0.0), origin_yaw=0.0, observation_fresh=True,
        )
        if r != ClearResult.NEED_MORE_NEGATIVES:
            fails.append(f'confirm {i} expected NEED_MORE, got {r}')
    r = m.try_clear_fire_cell(
        *idx, fire_detected=False, hold_active=False,
        origin_xy=(0.0, 0.0), origin_yaw=0.0, observation_fresh=True,
    )
    if r != ClearResult.ACCEPTED:
        fails.append(f'final fire clear expected ACCEPTED, got {r}')
    if m.has_cell(*idx):
        fails.append('fire cell remained after confirm')
    print('PASS  fire clear policy' if not fails else 'FAIL  fire clear')
    return fails


def test_fire_off_fov_no_clear() -> list:
    """T9: hold expired + off-FOV negatives must not clear FIRE."""
    fails = []
    m = _mem()
    m.mark_fire_hit(1.5, 0.0, now=1.0)
    idx = m.meta.world_to_grid(1.5, 0.0)
    assert idx is not None
    r = m.try_clear_fire_cell(
        *idx, fire_detected=False, hold_active=False,
        origin_xy=(10.0, 10.0), origin_yaw=0.0, observation_fresh=True,
    )
    if r != ClearResult.OUT_OF_FOV:
        fails.append(f'T9 off-FOV expected OUT_OF_FOV, got {r}')
    if not m.has_cell(*idx):
        fails.append('T9 off-FOV removed FIRE cell')
    print('PASS  fire off-FOV no clear' if not fails else 'FAIL  fire off-FOV')
    return fails


def test_lethal_region_report() -> list:
    """Spatial assertion helper: centre soft + neighbour lethal still ok."""
    fails = []
    from rs1_nav.geometry import lethal_region_report

    w, h, res = 40, 40, 0.05
    ox, oy = -1.0, -1.0
    data = [0] * (w * h)
    # Put lethal one cell east of world (0,0) → grid centre.
    cx = int((0.0 - ox) / res)
    cy = int((0.0 - oy) / res)
    data[cy * w + (cx + 1)] = 254
    rep = lethal_region_report(
        data, w, h, ox, oy, res, (0.0, 0.0),
        accept_radius_m=0.60, search_radius_m=1.0,
    )
    if not rep.ok:
        fails.append(f'neighbour lethal should pass: {rep.detail}')
    if rep.centre_cost not in (0, None):
        fails.append(f'centre expected free, got {rep.centre_cost}')
    if rep.nearest_lethal_m is None or rep.nearest_lethal_m > 0.10:
        fails.append(f'nearest_lethal_m unexpected: {rep.nearest_lethal_m}')

    empty = lethal_region_report(
        [0] * (w * h), w, h, ox, oy, res, (0.0, 0.0),
        accept_radius_m=0.60, search_radius_m=0.5,
    )
    if empty.ok:
        fails.append('empty region must not pass')
    print('PASS  lethal_region_report' if not fails else 'FAIL  lethal_region')
    return fails


def test_footprint_mark_exclusion() -> list:
    """Entry 023: marks under/near robot pose must be skipped."""
    fails = []
    m = _mem()
    m.cfg.mark_exclude_radius_m = 0.65
    robot = (0.0, 0.0)
    # Hit inside footprint halo — must not mark.
    near = m.mark_lidar_hit(0.3, 0.0, now=1.0, robot_xy=robot)
    if near:
        fails.append(f'near-robot hit marked cells {near}')
    if m.occupied_count() != 0:
        fails.append('footprint exclusion left occupied cells')
    # Hit outside halo — must mark.
    far = m.mark_lidar_hit(2.0, 0.0, now=1.0, robot_xy=robot)
    if not far:
        fails.append('far hit should mark')
    if m.occupied_count() < 1:
        fails.append('far hit did not persist')
    # Without robot_xy, near hit still marks (legacy / no TF).
    m2 = _mem()
    m2.cfg.mark_exclude_radius_m = 0.65
    bare = m2.mark_lidar_hit(0.3, 0.0, now=1.0)
    if not bare:
        fails.append('mark without robot_xy should still work')
    print('PASS  footprint mark exclusion' if not fails else 'FAIL  footprint exclude')
    return fails


def test_path_bank_evidence_unblock() -> list:
    fails = []
    bank = PathBank(start=(0.0, 0.0), goal=(5.0, 0.0), candidates=[
        PathCandidate(
            path_id=1, points=[(0.0, 0.0), (5.0, 0.0)],
            length_m=5.0, min_clearance_m=1.0, mean_cost=0.0,
            turn_penalty=0.0, score=90.0, state=CandidateState.BLOCKED,
            block_reason='unit',
        ),
        PathCandidate(
            path_id=2, points=[(0.0, 1.0), (5.0, 1.0)],
            length_m=5.0, min_clearance_m=1.0, mean_cost=0.0,
            turn_penalty=0.0, score=80.0, state=CandidateState.ACTIVE,
        ),
    ], active_id=2)
    if not bank.mark_available(1, 'evidence_clear'):
        fails.append('mark_available failed')
    if bank.candidates[0].state != CandidateState.AVAILABLE:
        fails.append('path 1 not AVAILABLE after evidence')
    if bank.active_id != 2:
        fails.append('unlock must not change active_id')
    # Second unlock on AVAILABLE is no-op.
    if bank.mark_available(1):
        fails.append('mark_available should be False when already AVAILABLE')
    print('PASS  bank evidence unblock' if not fails else 'FAIL  bank unblock')
    return fails


def main() -> int:
    failures: list = []
    for fn in (
        test_mark_and_mask,
        test_stale_scan_no_clear,
        test_occlusion_rejects,
        test_hit_at_cell_rejects,
        test_free_ray_accepts,
        test_hit_behind_accepts,
        test_unrelated_beam,
        test_lidar_does_not_clear_fire,
        test_fire_clear_policy,
        test_fire_off_fov_no_clear,
        test_lethal_region_report,
        test_footprint_mark_exclusion,
        test_path_bank_evidence_unblock,
    ):
        failures.extend(fn())
    if failures:
        print('FAIL')
        for f in failures:
            print(f'  - {f}')
        return 1
    print('ALL OBSTACLE-MEMORY UNIT TESTS PASSED')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
