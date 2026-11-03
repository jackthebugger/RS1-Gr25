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
        # Entry 027 camera ROI frustum defaults.
        thermal_hfov_rad=1.047,
        thermal_vfov_rad=0.810,
        thermal_roi_width_frac=0.6,
        thermal_roi_height_frac=0.7,
        thermal_near_m=0.1,
        thermal_camera_height_m=0.0,  # planar unit tests isolate horizontal gate
        thermal_cone_half_angle_rad=0.5235,
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


def test_hit_at_cell_wall_echo_clears() -> list:
    """Entry 035: HIT_AT_CELL on prior-free cell whose return is a prior wall."""
    import numpy as np

    fails = []
    m = _mem()
    # Cell at 3.0 m (prior free); adjacent prior wall at 3.2 m. Beam reports
    # HIT_AT_CELL (r=3.0); wall-echo gate clears because hit neighbourhood
    # contains prior occupied.
    prior = np.ones((m.meta.height, m.meta.width), dtype=bool)
    wall = m.meta.world_to_grid(3.2, 0.0)
    cell_xy = (3.0, 0.0)
    cell = m.meta.world_to_grid(*cell_xy)
    assert wall is not None and cell is not None
    prior[wall[1], wall[0]] = False
    m.mark_world(
        *cell_xy, CellSource.LIDAR, now=1.0,
        prior_free=prior, prior_edge_margin_cells=0,
    )
    if not m.has_cell(*cell):
        fails.append('setup: cell not marked')
        print('FAIL  wall-echo clear')
        return fails
    ranges, amin, ainc, rmin, rmax = _scan_toward(3.0)
    result = m.try_clear_lidar_cell(
        *cell,
        ranges=ranges, angle_min=amin, angle_increment=ainc,
        range_min=rmin, range_max=rmax,
        origin_xy=(0.0, 0.0), origin_yaw=0.0,
        scan_age_s=0.0, tf_ok=True,
        prior_free=prior,
    )
    if result != ClearResult.ACCEPTED:
        fails.append(f'wall-echo expected ACCEPTED, got {result}')
    if m.has_cell(*cell):
        fails.append('wall-echo did not remove prior-free memory cell')
    print('PASS  wall-echo HIT_AT_CELL clears' if not fails else 'FAIL  wall-echo')
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


def test_clear_lidar_scan_skips_fire() -> list:
    """Batch lidar free-ray clears LIDAR neighbours but leaves FIRE (Entry 026)."""
    fails = []
    m = _mem()
    m.mark_fire_hit(2.0, 0.0, now=1.0)
    m.mark_lidar_hit(2.0, 1.0, now=1.0)
    fire_idx = m.meta.world_to_grid(2.0, 0.0)
    lidar_idx = m.meta.world_to_grid(2.0, 1.0)
    assert fire_idx is not None and lidar_idx is not None
    n = 360
    ranges = [float('inf')] * n
    amin = -math.pi
    ainc = 2.0 * math.pi / n
    accepted, _rejected = m.clear_lidar_with_scan(
        ranges=ranges, angle_min=amin, angle_increment=ainc,
        range_min=0.05, range_max=30.0,
        origin_xy=(0.0, 0.0), origin_yaw=0.0,
        scan_age_s=0.0, tf_ok=True,
    )
    if accepted < 1:
        fails.append(f'expected lidar neighbour cleared, accepted={accepted}')
    if m.has_cell(*lidar_idx):
        fails.append('LIDAR neighbour remained after free-ray scan clear')
    if m.cell_source(*fire_idx) != CellSource.FIRE:
        fails.append('FIRE cell cleared or retagged by clear_lidar_with_scan')
    print('PASS  batch lidar clear skips FIRE' if not fails else 'FAIL  batch fire skip')
    return fails


def test_lidar_cannot_downgrade_fire() -> list:
    """mark_lidar_hit on an existing FIRE cell must not change source."""
    fails = []
    m = _mem()
    m.mark_fire_hit(1.5, 0.0, now=1.0)
    idx = m.meta.world_to_grid(1.5, 0.0)
    assert idx is not None
    m.mark_lidar_hit(1.5, 0.0, now=2.0)
    if m.cell_source(*idx) != CellSource.FIRE:
        fails.append(f'expected FIRE after lidar mark, got {m.cell_source(*idx)}')
    print('PASS  lidar cannot downgrade FIRE' if not fails else 'FAIL  fire downgrade')
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


def test_fire_frustum_outside_full_hfov() -> list:
    """Entry 027: bearing in (HFOV/2, 0.6] must be OUT_OF_FOV (legacy cone trap)."""
    fails = []
    m = _mem()
    # Legacy cone half was 0.6; camera half HFOV is ~0.5235.
    bearing = 0.55
    dist = 3.0
    fx = dist * math.cos(bearing)
    fy = dist * math.sin(bearing)
    m.mark_fire_hit(fx, fy, now=1.0)
    idx = m.meta.world_to_grid(fx, fy)
    assert idx is not None
    full_half = 0.5 * m.cfg.thermal_hfov_rad
    if not (full_half < bearing <= 0.6):
        fails.append(
            f'precondition: need full_half < bearing <= 0.6 '
            f'(full_half={full_half:.4f}, bearing={bearing})'
        )
    r = m.try_clear_fire_cell(
        *idx, fire_detected=False, hold_active=False,
        origin_xy=(0.0, 0.0), origin_yaw=0.0, observation_fresh=True,
        camera_z=0.0,
    )
    if r != ClearResult.OUT_OF_FOV:
        fails.append(f'outside full HFOV expected OUT_OF_FOV, got {r}')
    if not m.has_cell(*idx):
        fails.append('outside full HFOV removed FIRE cell')
    print('PASS  fire frustum outside HFOV' if not fails else 'FAIL  fire outside HFOV')
    return fails


def test_fire_frustum_outside_roi() -> list:
    """Entry 027: inside full HFOV but outside detector ROI → OUT_OF_FOV."""
    fails = []
    m = _mem()
    half_h, _ = m.thermal_roi_half_angles()
    full_half = 0.5 * m.cfg.thermal_hfov_rad
    # Midway between ROI half and full HFOV half.
    bearing = 0.5 * (half_h + full_half)
    dist = 3.0
    fx = dist * math.cos(bearing)
    fy = dist * math.sin(bearing)
    m.mark_fire_hit(fx, fy, now=1.0)
    idx = m.meta.world_to_grid(fx, fy)
    assert idx is not None
    if not (half_h < bearing < full_half):
        fails.append(
            f'precondition: half_h < bearing < full_half '
            f'(half_h={half_h:.4f}, bearing={bearing:.4f}, full={full_half:.4f})'
        )
    r = m.try_clear_fire_cell(
        *idx, fire_detected=False, hold_active=False,
        origin_xy=(0.0, 0.0), origin_yaw=0.0, observation_fresh=True,
        camera_z=0.0,
    )
    if r != ClearResult.OUT_OF_FOV:
        fails.append(f'outside ROI expected OUT_OF_FOV, got {r}')
    if not m.has_cell(*idx):
        fails.append('outside ROI removed FIRE cell')
    print('PASS  fire frustum outside ROI' if not fails else 'FAIL  fire outside ROI')
    return fails


def test_fire_frustum_inside_roi_clears() -> list:
    """Entry 027: bearing inside ROI half-angle + N negatives → clear."""
    fails = []
    m = _mem()
    half_h, _ = m.thermal_roi_half_angles()
    bearing = 0.5 * half_h  # safely inside ROI
    dist = 3.0
    fx = dist * math.cos(bearing)
    fy = dist * math.sin(bearing)
    m.mark_fire_hit(fx, fy, now=1.0)
    idx = m.meta.world_to_grid(fx, fy)
    assert idx is not None
    for i in range(m.cfg.fire_clear_confirm_scans - 1):
        r = m.try_clear_fire_cell(
            *idx, fire_detected=False, hold_active=False,
            origin_xy=(0.0, 0.0), origin_yaw=0.0, observation_fresh=True,
            camera_z=0.0,
        )
        if r != ClearResult.NEED_MORE_NEGATIVES:
            fails.append(f'in-ROI confirm {i} expected NEED_MORE, got {r}')
    r = m.try_clear_fire_cell(
        *idx, fire_detected=False, hold_active=False,
        origin_xy=(0.0, 0.0), origin_yaw=0.0, observation_fresh=True,
        camera_z=0.0,
    )
    if r != ClearResult.ACCEPTED:
        fails.append(f'in-ROI final clear expected ACCEPTED, got {r}')
    if m.has_cell(*idx):
        fails.append('in-ROI FIRE cell remained after confirm')
    print('PASS  fire frustum inside ROI clears' if not fails else 'FAIL  fire in-ROI clear')
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


def test_telemetry_stall_classifier() -> list:
    """Entry 028: stall = |v|<0.05 & |w|<0.10 ≥2 s while FOLLOWING_PATH, >0.5 m to goal."""
    from rs1_nav.telemetry import MotionSample, classify_motion, parse_launch_log_line

    fails = []
    samples = []
    t = 0.0
    # 0..5 s moving, 5..9 s stopped (stall 4 s), 9..10 s rotating only (1.5 s),
    # 10..13 s stopped but at goal (not a stall), 13..16 s stopped in
    # RETURNING_TO_START (filtered out).
    while t < 16.0:
        if t < 5.0:
            s = MotionSample(t=t, v=0.5, w=0.0, bank_state='FOLLOWING_PATH', goal_dist=10.0)
        elif t < 9.0:
            s = MotionSample(t=t, v=0.0, w=0.0, v_cmd=0.0, w_cmd=0.0,
                             bank_state='FOLLOWING_PATH', goal_dist=8.0)
        elif t < 10.5:
            s = MotionSample(t=t, v=0.0, w=0.9, bank_state='FOLLOWING_PATH', goal_dist=8.0)
        elif t < 13.0:
            s = MotionSample(t=t, v=0.0, w=0.0, bank_state='FOLLOWING_PATH', goal_dist=0.2)
        else:
            s = MotionSample(t=t, v=0.0, w=0.0, bank_state='RETURNING_TO_START', goal_dist=8.0)
        samples.append(s)
        t += 0.1
    cls = classify_motion(samples, t0=0.0, t1=16.0)
    if len(cls['stall']) != 1:
        fails.append(f'expected 1 stall, got {cls["stall"]}')
    elif not (3.5 <= cls['stall'][0].duration <= 4.1 and abs(cls['stall'][0].start - 5.0) < 0.15):
        fails.append(f'stall interval wrong: {cls["stall"][0]}')
    if len(cls['rotate']) != 1 or not (1.2 <= cls['rotate'][0].duration <= 1.6):
        fails.append(f'expected one ~1.5 s rotate, got {cls["rotate"]}')
    if len(cls['zero_cmd']) != 1:
        fails.append(f'expected one zero_cmd run, got {cls["zero_cmd"]}')
    # Short dip below threshold (<2 s) must not count.
    short = [MotionSample(t=i * 0.1, v=0.5 if (i < 10 or i > 25) else 0.0, w=0.0,
                          bank_state='FOLLOWING_PATH', goal_dist=5.0) for i in range(40)]
    if classify_motion(short, t0=0.0, t1=4.0)['stall']:
        fails.append('1.5 s dip counted as stall')
    # Launch-log parser.
    line = ('[controller_server-11] [ERROR] [1793093499.123456789] '
            '[husky1.controller_server]: FollowPath detected collision ahead!')
    parsed = parse_launch_log_line(line)
    if parsed is None or parsed[2] != 'RPP_COLLISION' or abs(parsed[0] - 1793093499.1235) > 1e-3:
        fails.append(f'log parse wrong: {parsed}')
    if parse_launch_log_line('[x] [INFO] [1.0] [n]: nothing interesting') is not None:
        fails.append('unmatched line should return None')
    print('PASS  telemetry stall classifier' if not fails else 'FAIL  telemetry classifier')
    return fails


def test_memory_introspection_points() -> list:
    """lidar/fire cloud helpers and cells_near report typed cells only."""
    mem = _mem()
    mem.mark_lidar_hit(1.0, 1.0, now=0.0)
    mem.mark_fire_hit(2.0, 2.0, now=0.0)
    fails = []
    if len(mem.lidar_cell_world_points()) != 1 or len(mem.fire_cell_world_points()) != 1:
        fails.append('typed cloud sizes wrong')
    if len(mem.cells_near(2.0, 2.0, 0.1, CellSource.FIRE)) != 1:
        fails.append('cells_near FIRE missed the fire cell')
    if mem.cells_near(2.0, 2.0, 0.1, CellSource.LIDAR):
        fails.append('cells_near LIDAR returned a FIRE cell')
    counts = mem.clear_fire_with_observation_detailed(
        fire_detected=False, hold_active=False, origin_xy=(0.0, 2.0),
        origin_yaw=0.0, observation_fresh=True, camera_z=0.0)
    if counts.get(ClearResult.NEED_MORE_NEGATIVES, 0) != 1:
        fails.append(f'detailed clear counts wrong: {counts}')
    print('PASS  memory introspection' if not fails else 'FAIL  memory introspection')
    return fails


def test_fire_bridge_helpers() -> list:
    """Entry 028: thermal mark requires detector True + bearing + LiDAR range."""
    from rs1_nav.fire_bridge import (
        HazardDeduper,
        ThermalBearing,
        hot_blob_bearing,
        thermal_hazard_point,
    )

    fails = []
    bb = hot_blob_bearing([160], 320, 1.047)
    if bb is None or abs(bb[0]) > 0.02:
        fails.append(f'centre-column bearing should be ~0, got {bb}')
    bb_left = hot_blob_bearing([80], 320, 1.047)
    if bb_left is None or bb_left[0] <= 0.0:
        fails.append(f'left-of-centre column should be +bearing, got {bb_left}')

    # 180-sample 360° scan, one return at 3 m on +X.
    n = 180
    inc = 2.0 * math.pi / n
    ranges = [float('inf')] * n
    ranges[0] = 3.0  # angle_min=0 → +X
    now = 10.0
    bearing = ThermalBearing(bearing_rad=0.0, half_width_rad=0.05, stamp_s=now)
    common = dict(
        bearing=bearing, now_s=now, scan_ranges=ranges,
        angle_min=0.0, angle_increment=inc, range_min=0.1, range_max=40.0,
        camera_forward_m=0.90,
    )
    if thermal_hazard_point(fire_detected=False, **common) is not None:
        fails.append('no mark when fire_detected is False')
    if thermal_hazard_point(fire_detected=True, bearing=None, now_s=now,
                            scan_ranges=ranges, angle_min=0.0,
                            angle_increment=inc, range_min=0.1, range_max=40.0) is not None:
        fails.append('no mark without bearing')
    empty = list(ranges)
    empty[0] = float('inf')
    if thermal_hazard_point(fire_detected=True, bearing=bearing, now_s=now,
                            scan_ranges=empty, angle_min=0.0,
                            angle_increment=inc, range_min=0.1, range_max=40.0,
                            camera_forward_m=0.90) is not None:
        fails.append('no mark without LiDAR return along bearing')
    stale = ThermalBearing(0.0, 0.05, stamp_s=now - 2.0)
    if thermal_hazard_point(fire_detected=True, bearing=stale, now_s=now,
                            scan_ranges=ranges, angle_min=0.0,
                            angle_increment=inc, range_min=0.1, range_max=40.0,
                            camera_forward_m=0.90) is not None:
        fails.append('stale bearing must not mark')
    pt = thermal_hazard_point(fire_detected=True, **common)
    if pt is None or abs(pt[0] - 3.0) > 0.15 or abs(pt[1]) > 0.15:
        fails.append(f'expected ~ (3,0) hazard, got {pt}')
    # Near off-bearing bank must not steal the on-bearing fire return.
    steal = list(ranges)
    steal[int(round(0.15 / inc))] = 1.8
    steal[0] = 5.0
    pt = thermal_hazard_point(
        fire_detected=True, bearing=bearing, now_s=now, scan_ranges=steal,
        angle_min=0.0, angle_increment=inc, range_min=0.1, range_max=40.0,
        camera_forward_m=0.90, min_range_m=1.5)
    if pt is None or abs(pt[0] - 5.0) > 0.3:
        fails.append(f'on-bearing return should win over nearer bank, got {pt}')
    # Return behind the camera (dx <= 0 vs 0.90 m optical) is ignored.
    behind = list(ranges)
    behind[0] = 0.4
    if thermal_hazard_point(fire_detected=True, bearing=bearing, now_s=now,
                            scan_ranges=behind, angle_min=0.0,
                            angle_increment=inc, range_min=0.1, range_max=40.0,
                            camera_forward_m=0.90) is not None:
        fails.append('return behind optical origin must not mark')
    d = HazardDeduper(min_separation_m=0.25, window_s=8.0)
    d.note_detected(True)
    if not d.accept((1.0, 1.0), 0.0) or d.accept((1.05, 1.0), 0.1):
        fails.append('deduper should accept once then reject near-duplicate')
    if d.accept((4.0, 4.0), 0.2):
        fails.append('same fire_detected episode must not emit a second disc')
    d.note_detected(False)
    d.note_detected(True)
    if not d.accept((4.0, 4.0), 6.0):
        fails.append('new detection episode should be allowed to mark')
    print('PASS  fire bridge helpers' if not fails else 'FAIL  fire bridge helpers')
    return fails


def test_fire_confirm_resets_on_out_of_fov() -> list:
    """Consecutive negatives must reset when the cell leaves the frustum."""
    fails = []
    m = _mem()
    m.mark_fire_hit(2.0, 0.0, now=1.0)
    idx = m.meta.world_to_grid(2.0, 0.0)
    assert idx is not None
    r = m.try_clear_fire_cell(
        *idx, fire_detected=False, hold_active=False,
        origin_xy=(0.0, 0.0), origin_yaw=0.0, observation_fresh=True,
        camera_z=0.0)
    if r != ClearResult.NEED_MORE_NEGATIVES:
        fails.append(f'first in-FOV expected NEED_MORE, got {r}')
    r = m.try_clear_fire_cell(
        *idx, fire_detected=False, hold_active=False,
        origin_xy=(10.0, 10.0), origin_yaw=0.0, observation_fresh=True,
        camera_z=0.0)
    if r != ClearResult.OUT_OF_FOV:
        fails.append(f'off-FOV expected OUT_OF_FOV, got {r}')
    cell = m.cells.get(idx)
    if cell is None or cell.fire_clear_confirm_count != 0:
        fails.append('OUT_OF_FOV must reset confirm count')
    r = m.try_clear_fire_cell(
        *idx, fire_detected=False, hold_active=False,
        origin_xy=(0.0, 0.0), origin_yaw=0.0, observation_fresh=False,
        camera_z=0.0)
    if r != ClearResult.STALE:
        fails.append(f'stale expected STALE, got {r}')
    cell = m.cells.get(idx)
    if cell is None or cell.fire_clear_confirm_count != 0:
        fails.append('STALE must reset confirm count')
    print('PASS  fire confirm reset' if not fails else 'FAIL  fire confirm reset')
    return fails


def test_fire_los_occluded_no_clear() -> list:
    """A nearer LiDAR hit between camera and FIRE cell vetoes the clear."""
    fails = []
    m = _mem()
    m.mark_fire_hit(4.0, 0.0, now=1.0)
    idx = m.meta.world_to_grid(4.0, 0.0)
    assert idx is not None
    n = 180
    inc = 2.0 * math.pi / n
    ranges = [float('inf')] * n
    ranges[0] = 1.5  # opaque at 1.5 m, cell at 4 m (beyond 1.0 m cluster margin)
    r = m.try_clear_fire_cell(
        *idx, fire_detected=False, hold_active=False,
        origin_xy=(0.0, 0.0), origin_yaw=0.0, observation_fresh=True,
        camera_z=0.0,
        ranges=ranges, angle_min=0.0, angle_increment=inc,
        range_min=0.1, range_max=40.0,
        scan_origin_xy=(0.0, 0.0), scan_origin_yaw=0.0,
    )
    if r != ClearResult.OCCLUDED:
        fails.append(f'expected OCCLUDED, got {r}')
    if not m.has_cell(*idx):
        fails.append('OCCLUDED must not delete FIRE cell')
    cell = m.cells.get(idx)
    if cell is None or cell.fire_clear_confirm_count != 0:
        fails.append('OCCLUDED must reset confirm count')
    # Hit within the 1.0 m cluster margin is the same object, not an occluder.
    ranges[0] = 3.6
    r = m.try_clear_fire_cell(
        *idx, fire_detected=False, hold_active=False,
        origin_xy=(0.0, 0.0), origin_yaw=0.0, observation_fresh=True,
        camera_z=0.0,
        ranges=ranges, angle_min=0.0, angle_increment=inc,
        range_min=0.1, range_max=40.0,
        scan_origin_xy=(0.0, 0.0), scan_origin_yaw=0.0,
    )
    if r == ClearResult.OCCLUDED:
        fails.append('same-cluster lidar hit must not OCCLUDE the FIRE cell')
    # Restore a nearer occluder for the free-ray check below.
    ranges[0] = float('inf')
    # Free ray (inf) along the bearing: not occluded, counts toward clear.
    ranges[0] = float('inf')
    r = m.try_clear_fire_cell(
        *idx, fire_detected=False, hold_active=False,
        origin_xy=(0.0, 0.0), origin_yaw=0.0, observation_fresh=True,
        camera_z=0.0,
        ranges=ranges, angle_min=0.0, angle_increment=inc,
        range_min=0.1, range_max=40.0,
        scan_origin_xy=(0.0, 0.0), scan_origin_yaw=0.0,
    )
    if r != ClearResult.NEED_MORE_NEGATIVES:
        fails.append(f'free-ray LOS expected NEED_MORE, got {r}')
    print('PASS  fire LOS occluded' if not fails else 'FAIL  fire LOS occluded')
    return fails


def test_optical_offset_moves_frustum() -> list:
    """A cell in front of camera_link but behind the optical origin is OUT_OF_FOV."""
    fails = []
    m = _mem()
    # 0.20 m ahead of camera_link; optical origin is +0.45 m.
    m.mark_fire_hit(0.20, 0.0, now=1.0)
    idx = m.meta.world_to_grid(0.20, 0.0)
    assert idx is not None
    r = m.try_clear_fire_cell(
        *idx, fire_detected=False, hold_active=False,
        origin_xy=(0.0, 0.0), origin_yaw=0.0, observation_fresh=True,
        camera_z=0.0, optical_forward_m=0.45, optical_up_m=0.0,
    )
    if r != ClearResult.OUT_OF_FOV:
        fails.append(f'behind optical origin expected OUT_OF_FOV, got {r}')
    if not m.has_cell(*idx):
        fails.append('optical-offset OUT_OF_FOV deleted the cell')
    print('PASS  optical offset frustum' if not fails else 'FAIL  optical offset')
    return fails


def test_fire_lidar_neighbour_coexistence() -> list:
    """FIRE and LIDAR cells at neighbouring coordinates stay independent."""
    fails = []
    m = _mem()
    m.mark_fire_hit(3.0, 0.0, now=1.0)
    m.mark_lidar_hit(3.0, 0.25, now=1.0)
    fidx = m.meta.world_to_grid(3.0, 0.0)
    lidx = m.meta.world_to_grid(3.0, 0.25)
    assert fidx is not None and lidx is not None
    if m.cell_source(*fidx) != CellSource.FIRE:
        fails.append('FIRE cell lost after neighbour LIDAR mark')
    if m.cell_source(*lidx) != CellSource.LIDAR:
        fails.append('LIDAR neighbour not stored')
    n = 180
    inc = 2.0 * math.pi / n
    ranges = [float('inf')] * n
    m.clear_lidar_with_scan(
        ranges=ranges, angle_min=0.0, angle_increment=inc,
        range_min=0.1, range_max=40.0,
        origin_xy=(0.0, 0.0), origin_yaw=0.0,
        scan_age_s=0.0, tf_ok=True,
    )
    if not m.has_cell(*fidx) or m.cell_source(*fidx) != CellSource.FIRE:
        fails.append('lidar free-ray cleared neighbouring FIRE')
    if m.has_cell(*lidx):
        fails.append('lidar free-ray should have cleared the LIDAR neighbour')
    # A lidar hit on the FIRE cell must not create/downgrade FIRE.
    m.mark_lidar_hit(3.0, 0.0, now=2.0)
    if m.cell_source(*fidx) != CellSource.FIRE:
        fails.append('lidar hit created/downgraded FIRE')
    print('PASS  fire/lidar neighbour coexistence' if not fails else
          'FAIL  fire/lidar coexistence')
    return fails


def main() -> int:
    failures: list = []
    for fn in (
        test_telemetry_stall_classifier,
        test_memory_introspection_points,
        test_fire_bridge_helpers,
        test_fire_confirm_resets_on_out_of_fov,
        test_fire_los_occluded_no_clear,
        test_optical_offset_moves_frustum,
        test_fire_lidar_neighbour_coexistence,
        test_mark_and_mask,
        test_stale_scan_no_clear,
        test_occlusion_rejects,
        test_hit_at_cell_rejects,
        test_hit_at_cell_wall_echo_clears,
        test_free_ray_accepts,
        test_hit_behind_accepts,
        test_unrelated_beam,
        test_lidar_does_not_clear_fire,
        test_clear_lidar_scan_skips_fire,
        test_lidar_cannot_downgrade_fire,
        test_fire_clear_policy,
        test_fire_off_fov_no_clear,
        test_fire_frustum_outside_full_hfov,
        test_fire_frustum_outside_roi,
        test_fire_frustum_inside_roi_clears,
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
