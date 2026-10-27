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
    PathBankConfig,
    build_path_bank,
    exclude_corridor,
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

    if failures:
        print('FAIL')
        for f in failures:
            print(f'  - {f}')
        return 1
    print('ALL PATH-BANK UNIT TESTS PASSED')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
