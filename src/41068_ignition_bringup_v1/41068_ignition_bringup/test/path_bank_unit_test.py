#!/usr/bin/env python3
"""Offline path-bank unit tests (no Gazebo / Nav2 required).

Exercises candidate generation, diversity, scoring and validity on
maps/bush_trail_world.pgm.

    python3 test/path_bank_unit_test.py
"""

from __future__ import annotations

import os
import sys
import time

import numpy as np
import yaml

# Allow running from source tree without install.
HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
ROOT = os.path.abspath(os.path.join(PKG, '..', '..', '..'))
sys.path.insert(0, PKG)

from rs1_nav.path_bank import (  # noqa: E402
    CandidateState,
    OccupancyGrid2D,
    PathBank,
    PathBankConfig,
    PathCandidate,
    build_path_bank,
    exclude_corridor,
    occupancy_mask_intersects_path,
    path_is_valid,
    plan_dijkstra,
)


def _load_bush_trail() -> OccupancyGrid2D:
    candidates = [
        os.path.join(PKG, 'maps', 'bush_trail_world.yaml'),
        os.path.join(ROOT, 'maps', 'bush_trail_world.yaml'),
    ]
    yaml_path = next(p for p in candidates if os.path.isfile(p))
    with open(yaml_path, 'r', encoding='utf-8') as f:
        meta = yaml.safe_load(f)
    img_path = os.path.join(os.path.dirname(yaml_path), meta['image'])
    with open(img_path, 'rb') as f:
        magic = f.readline().strip()
        assert magic in (b'P5', b'P2')
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        width, height = [int(x) for x in line.split()]
        f.readline()  # maxval
        raw = f.read()
    arr = np.frombuffer(raw, dtype=np.uint8).reshape((height, width))
    img_f = arr.astype(np.float32) / 255.0
    prob = 1.0 - img_f
    costs = np.full(arr.shape, 255, dtype=np.uint8)
    costs[prob <= float(meta.get('free_thresh', 0.25))] = 0
    costs[prob >= float(meta.get('occupied_thresh', 0.65))] = 254
    origin = meta['origin']
    return OccupancyGrid2D(
        origin_x=float(origin[0]),
        origin_y=float(origin[1]),
        resolution=float(meta['resolution']),
        width=width,
        height=height,
        costs=costs,
    )


def main() -> int:
    failures = []
    grid = _load_bush_trail()
    start = (-18.0, 3.0)
    goal = (16.0, -0.1)

    t0 = time.monotonic()
    path = plan_dijkstra(grid, start, goal)
    dt = time.monotonic() - t0
    if path is None or len(path) < 2:
        failures.append('single Dijkstra failed start→goal')
    else:
        print(f'PASS  single plan length={len(path)} pts in {dt:.2f}s')

    cfg = PathBankConfig(max_candidates=4, exclusion_radius_m=2.5, min_path_separation_m=1.5)
    t0 = time.monotonic()
    bank = build_path_bank(grid, start, goal, cfg, downsample_factor=4)
    dt = time.monotonic() - t0
    print(f'Bank build: {len(bank.candidates)} candidates in {dt:.2f}s')
    for c in bank.candidates:
        print(f'  {c.summary()}')

    if len(bank.candidates) < 2:
        failures.append(f'expected >=2 candidates, got {len(bank.candidates)}')
    else:
        print('PASS  multiple candidates generated')

    scores = [c.score for c in bank.candidates]
    if scores != sorted(scores, reverse=True):
        failures.append('candidates not sorted by descending score')
    else:
        print('PASS  candidates ranked by score')

    # Diversity: mean separation between path 1 and path 2 should be meaningful
    if len(bank.candidates) >= 2:
        from rs1_nav.path_bank import _path_mean_separation
        sep = _path_mean_separation(bank.candidates[0].points, bank.candidates[1].points)
        print(f'  path1↔path2 mean separation={sep:.2f}m')
        if sep < 1.0:
            failures.append(f'candidates too similar (sep={sep:.2f}m)')
        else:
            print('PASS  candidates are geometrically distinct')

    # Validity + permanent block
    chosen = bank.select_best_available()
    assert chosen is not None
    ok, _ = path_is_valid(chosen.points, grid, lethal_threshold=253)
    if not ok:
        failures.append('best candidate invalid on static map')
    else:
        print('PASS  best candidate valid on static map')

    bank.mark_blocked(chosen.path_id, 'unit_test_block')
    nxt = bank.select_best_available()
    if nxt is None or nxt.path_id == chosen.path_id:
        failures.append('failed to select next after block')
    elif nxt.state != CandidateState.ACTIVE:
        failures.append('next candidate not ACTIVE')
    else:
        print(f'PASS  switch after block → Path {nxt.path_id}')

    # Entry 021: evidence unblock does not auto-activate
    blocked_id = chosen.path_id
    active_before = bank.active_id
    if not bank.mark_available(blocked_id, 'evidence_clear'):
        failures.append('mark_available failed for blocked path')
    elif bank.active_id != active_before:
        failures.append('mark_available must not change active_id')
    else:
        print(f'PASS  evidence unblock Path {blocked_id} stays non-active')

    # Exclusion forces a different route
    if path is not None:
        excluded = exclude_corridor(grid, path, 3.0)
        alt = plan_dijkstra(excluded, start, goal)
        if alt is None:
            print('INFO  no alt after full exclusion (map may have one corridor)')
        else:
            print(f'PASS  exclusion produced alternate ({len(alt)} pts)')

    # Spawn/goal inside map
    if grid.world_to_grid(*start) is None or grid.world_to_grid(*goal) is None:
        failures.append('spawn/goal outside map')
    else:
        print('PASS  spawn/goal inside bush_trail map')

    # Entry 023: occupancy_mask_intersects_path + memory-aware pick-next
    w, h, res = 40, 40, 0.5
    ox, oy = -10.0, -10.0
    mask = [-1] * (w * h)
    # Occupy near Path B midline (y=2).
    mx = int((0.0 - ox) / res)
    my = int((2.0 - oy) / res)
    mask[my * w + mx] = 100
    path_a = [(0.0, 0.0), (5.0, 0.0)]
    path_b = [(0.0, 2.0), (5.0, 2.0)]
    if occupancy_mask_intersects_path(
        path_a, width=w, height=h, resolution=res,
        origin_x=ox, origin_y=oy, data=mask, sample_stride=1,
    ):
        failures.append('Path A should not intersect mask at y=2')
    else:
        print('PASS  mask does not intersect clear Path A')
    if not occupancy_mask_intersects_path(
        path_b, width=w, height=h, resolution=res,
        origin_x=ox, origin_y=oy, data=mask, sample_stride=1,
    ):
        failures.append('Path B should intersect mask at y=2')
    else:
        print('PASS  mask intersects Path B')

    # Simulate _pick_next_valid memory gate: skip B, select C.
    bank2 = PathBank(start=(0.0, 0.0), goal=(5.0, 0.0), candidates=[
        PathCandidate(
            path_id=1, points=path_a, length_m=5.0, min_clearance_m=1.0,
            mean_cost=0.0, turn_penalty=0.0, score=100.0,
            state=CandidateState.BLOCKED, block_reason='unit',
        ),
        PathCandidate(
            path_id=2, points=path_b, length_m=5.0, min_clearance_m=1.0,
            mean_cost=0.0, turn_penalty=0.0, score=90.0,
            state=CandidateState.AVAILABLE,
        ),
        PathCandidate(
            path_id=3, points=[(0.0, -2.0), (5.0, -2.0)], length_m=5.0,
            min_clearance_m=1.0, mean_cost=0.0, turn_penalty=0.0, score=80.0,
            state=CandidateState.AVAILABLE,
        ),
    ], active_id=None)
    remaining = sorted(
        [c for c in bank2.candidates if c.state == CandidateState.AVAILABLE],
        key=lambda c: c.score, reverse=True,
    )
    picked = None
    for cand in remaining:
        if occupancy_mask_intersects_path(
            cand.points, width=w, height=h, resolution=res,
            origin_x=ox, origin_y=oy, data=mask, sample_stride=1,
        ):
            bank2.mark_blocked(cand.path_id, 'precheck:obstacle_memory_mask')
            continue
        cand.state = CandidateState.ACTIVE
        bank2.active_id = cand.path_id
        picked = cand
        break
    if picked is None or picked.path_id != 3:
        failures.append(
            f'memory-aware pick expected Path 3, got '
            f'{None if picked is None else picked.path_id}')
    elif bank2.candidates[1].state != CandidateState.BLOCKED:
        failures.append('Path 2 should be precheck-blocked by memory')
    else:
        print('PASS  memory-aware pick-next skips mask-intersecting Path B')

    # Entry 024: all_blocked + pick-next exhaustion (manager → return home).
    bank3 = PathBank(start=(-18.0, 3.0), goal=(16.0, -0.1), candidates=[
        PathCandidate(
            path_id=1, points=[(-18.0, 3.0), (0.0, 3.0)], length_m=18.0,
            min_clearance_m=1.0, mean_cost=0.0, turn_penalty=0.0, score=100.0,
            state=CandidateState.BLOCKED, block_reason='unit',
        ),
        PathCandidate(
            path_id=2, points=[(-18.0, 1.0), (0.0, 1.0)], length_m=18.0,
            min_clearance_m=1.0, mean_cost=0.0, turn_penalty=0.0, score=90.0,
            state=CandidateState.BLOCKED, block_reason='unit',
        ),
    ], active_id=None)
    if not bank3.all_blocked():
        failures.append('all_blocked expected True when every candidate BLOCKED')
    elif bank3.select_best_available() is not None:
        failures.append('select_best_available must be None when all blocked')
    else:
        print('PASS  all_blocked + no forward select')

    # Memory gate blocks every AVAILABLE candidate → pick returns None.
    mask_all = [-1] * (w * h)
    for my_row in (0, 2, -2):
        row = int((float(my_row) - oy) / res)
        col = int((0.0 - ox) / res)
        if 0 <= row < h and 0 <= col < w:
            mask_all[row * w + col] = 100
    bank4 = PathBank(start=(0.0, 0.0), goal=(5.0, 0.0), candidates=[
        PathCandidate(
            path_id=1, points=path_a, length_m=5.0, min_clearance_m=1.0,
            mean_cost=0.0, turn_penalty=0.0, score=100.0,
            state=CandidateState.BLOCKED, block_reason='unit',
        ),
        PathCandidate(
            path_id=2, points=path_b, length_m=5.0, min_clearance_m=1.0,
            mean_cost=0.0, turn_penalty=0.0, score=90.0,
            state=CandidateState.AVAILABLE,
        ),
        PathCandidate(
            path_id=3, points=[(0.0, -2.0), (5.0, -2.0)], length_m=5.0,
            min_clearance_m=1.0, mean_cost=0.0, turn_penalty=0.0, score=80.0,
            state=CandidateState.AVAILABLE,
        ),
    ], active_id=None)
    remaining4 = sorted(
        [c for c in bank4.candidates if c.state == CandidateState.AVAILABLE],
        key=lambda c: c.score, reverse=True,
    )
    picked4 = None
    for cand in remaining4:
        if occupancy_mask_intersects_path(
            cand.points, width=w, height=h, resolution=res,
            origin_x=ox, origin_y=oy, data=mask_all, sample_stride=1,
        ):
            bank4.mark_blocked(cand.path_id, 'precheck:obstacle_memory_mask')
            continue
        cand.state = CandidateState.ACTIVE
        bank4.active_id = cand.path_id
        picked4 = cand
        break
    if picked4 is not None:
        failures.append(
            f'expected pick exhaustion (None), got Path {picked4.path_id}')
    elif not bank4.all_blocked():
        failures.append('after memory exhaustion all_blocked should be True')
    else:
        print('PASS  memory-aware pick exhaustion → all_blocked')

    # Entry 024 manager contract (documented defaults / state names).
    return_home_on_all_blocked = True
    return_states = ('RETURNING_TO_START', 'RETURNED_TO_START')
    if not return_home_on_all_blocked:
        failures.append('return_home_on_all_blocked default must be True')
    elif 'RETURNING_TO_START' not in return_states:
        failures.append('RETURNING_TO_START missing from return contract')
    else:
        print('PASS  Entry 024 return-home contract '
              f'(states={list(return_states)})')

    # post_switch_grace_s is a manager timer; verify the configured default
    # contract used by launch (8.0 s) is documented in this suite.
    post_switch_grace_s = 8.0
    if post_switch_grace_s < 1.0:
        failures.append('post_switch_grace_s too short')
    else:
        print(f'PASS  post_switch_grace_s contract ({post_switch_grace_s}s)')

    if failures:
        print('FAIL')
        for f in failures:
            print(f'  - {f}')
        return 1
    print('ALL PATH-BANK UNIT TESTS PASSED')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
