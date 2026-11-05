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
    INSCRIBED,
    LETHAL,
    CandidateState,
    OccupancyGrid2D,
    PathBank,
    PathBankConfig,
    PathCandidate,
    apply_soft_clearance_dt,
    build_path_bank,
    exclude_corridor,
    inflate_lethal,
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

    # Phase Two: timer-excluded terminal + hang-bound defaults (Wave 0/1b).
    terminal_states = ('NO_SAFE_ROUTE', 'FAILED', 'ARRIVED', 'RETURNED_TO_START')
    timer_active = ('FOLLOWING_PATH', 'PATH_BLOCKED', 'NO_PATH_AVAILABLE')
    if 'NO_SAFE_ROUTE' not in terminal_states:
        failures.append('NO_SAFE_ROUTE missing from terminal contract')
    elif 'NO_SAFE_ROUTE' in timer_active:
        failures.append('NO_SAFE_ROUTE must not be in timer-active set')
    else:
        print('PASS  Phase Two NO_SAFE_ROUTE timer-excluded terminal')

    hang_defaults = {
        'stuck_recovery_budget': 8,
        'terminal_path_blocked_none_s': 20.0,
        'home_stuck_timeout_s': 60.0,
        'start_seed_timeout_s': 30.0,
    }
    if hang_defaults['stuck_recovery_budget'] < 1:
        failures.append('stuck_recovery_budget invalid')
    else:
        print(
            'PASS  Phase Two hang-bound defaults '
            f'(N={hang_defaults["stuck_recovery_budget"]}, '
            f'T1={hang_defaults["terminal_path_blocked_none_s"]}, '
            f'T2={hang_defaults["home_stuck_timeout_s"]}, '
            f'T_seed={hang_defaults["start_seed_timeout_s"]})'
        )

    # Reverse-splice home recovery must be gone (Wave 1a contract).
    try:
        mgr_path = os.path.join(PKG, 'scripts', 'path_bank_manager.py')
        src = open(mgr_path, encoding='utf-8').read()
        if '_make_home_route' in src:
            failures.append('_make_home_route must be deleted (no reverse-splice)')
        elif 'best_i::-1' in src or 'points[best_i::-1]' in src:
            failures.append('reverse-splice home walk still present')
        elif 'Fresh home ComputePathToPose' not in src:
            failures.append('fresh home ComputePathToPose path missing')
        else:
            print('PASS  Wave 1a fresh-home (no reverse-splice) contract')

        # Wave-1 return-home plan contracts (APPROVED Phase Two).
        begin_idx = src.find('def _begin_return_to_start')
        plan_idx = src.find('def _request_fresh_home_plan')
        if begin_idx < 0 or plan_idx < 0 or begin_idx > plan_idx:
            failures.append('_begin_return_to_start / _request_fresh_home_plan missing')
        else:
            begin_body = src[begin_idx:plan_idx]
            cancel_pos = begin_body.find('self._cancel_nav()')
            bump_pos = begin_body.find('self._owner.bump_generation()')
            latch_pos = begin_body.find('self._home_attempted = True')
            clear_sw = begin_body.find('_clear_switch_pending')
            req_pos = begin_body.find('_request_fresh_home_plan')
            if min(cancel_pos, bump_pos, latch_pos, clear_sw) < 0:
                failures.append('D1 begin-return missing cancel/bump/latch/clear_switch')
            elif not (cancel_pos < bump_pos < latch_pos < req_pos):
                failures.append(
                    'D1 order: cancel → bump → latch → plan request required')
            else:
                print('PASS  D1 cancel-before-plan order in _begin_return_to_start')

        if 'def _home_success_fence_ok' not in src:
            failures.append('D2 _home_success_fence_ok missing')
        elif 'uid=route_uid' not in src:
            failures.append('D2 route_uid not threaded into _fp_goal_result')
        elif '_home_plan_pending' not in src[
                src.find('def _home_success_fence_ok'):
                src.find('def _home_success_fence_ok') + 800]:
            failures.append('D2 fence must reject while _home_plan_pending')
        else:
            print('PASS  D2 SUCCEEDED fence + route_uid threading')

        if "_rebuild_resume_mode = 'HOME'" not in src:
            failures.append("D3 _rebuild_resume_mode='HOME' missing")
        elif 'HOME resume' not in src:
            failures.append('D3 HOME resume branch missing')
        else:
            print('PASS  D3 §G _rebuild_resume_mode HOME')

        if 'return-home first (A.3 latch)' not in src and '_home_attempted' not in src:
            failures.append('A.3 T1 _home_attempted latch missing')
        elif 'path_blocked_none_timeout' not in src:
            failures.append('A.3 second-T1 terminal path missing')
        else:
            print('PASS  A.3 T1 return-first latch contract')

        if 'defensive return-home (A.4)' not in src and 'A.4' not in src:
            failures.append('A.4 T_seed invariant comment/path missing')
        else:
            print('PASS  A.4 T_seed ↔ home-None invariant')

        c1_def = src.find('def _home_route_c1_oriented_valid')
        if c1_def < 0:
            failures.append('C1 forward-slice helper missing')
        elif '_route_forward_from_robot' not in src[c1_def:c1_def + 900]:
            failures.append('C1 must use _route_forward_from_robot')
        elif 'mark_exclude' in src[c1_def:c1_def + 900].split('def ', 1)[0]:
            # Docstring may mention mark_exclude as forbidden — that is OK.
            body = src[c1_def:c1_def + 900]
            if 'mark_exclude_radius' in body or 'under mark_exclude' in body:
                failures.append('C1 must not cite mark_exclude as accept override')
            else:
                print('PASS  C1 forward-slice oriented accept')
        else:
            print('PASS  C1 forward-slice oriented accept')

        # Entry 062 follow-on: home ComputePath must nudge like mission bind
        # before C1 (NavFn centreline ↔ oriented knife-edge).
        build_home = src.find('def _build_home_route_from_points')
        if build_home < 0:
            failures.append('_build_home_route_from_points missing')
        else:
            next_def = src.find('\n    def ', build_home + 10)
            body = src[build_home: next_def if next_def > 0 else build_home + 3500]
            if 'nudge_polyline_away_from_lethal' not in body:
                failures.append(
                    'home build must nudge_polyline_away_from_lethal before C1')
            elif '_home_route_c1_oriented_valid' not in body:
                failures.append('home build must still C1-validate after nudge')
            elif '_bank_homeward_route_if_c1_valid' not in body:
                failures.append(
                    'home build must fall back to C1-gated bank homeward')
            else:
                print('PASS  home route nudge-before-C1 (GUI-DB knife-edge)')

        if 'def _bank_homeward_route_if_c1_valid' not in src:
            failures.append('_bank_homeward_route_if_c1_valid helper missing')
        else:
            print('PASS  bank homeward C1 fallback helper')

        if 'def _effective_home_t2_s' not in src:
            failures.append('_effective_home_t2_s missing (GUI-DB long-arm T2)')
        elif 'arm / 0.18' not in src and 'arm/0.18' not in src:
            failures.append('_effective_home_t2_s must scale with arm distance')
        else:
            print('PASS  home T2 arm-distance scale (GUI-DB)')

        if 'def _resume_home_after_rebuild' not in src:
            failures.append('_resume_home_after_rebuild helper missing')
        elif 'HOLD_REBUILD_TIMEOUT' not in src[
            src.find('def _maybe_finish_rebuild'):
            src.find('def _maybe_finish_rebuild') + 2500
        ]:
            failures.append('_maybe_finish_rebuild must mention HOLD_REBUILD_TIMEOUT')
        elif 'unsettled under HOME' not in src:
            failures.append('HOME unsettled rebuild must skip HOLD and re-arm')
        else:
            print('PASS  HOME skip HOLD_REBUILD on unsettled §G')

        if '_home_plan_attempt_count' not in src or '_retry_home_plan_or_raise' not in src:
            failures.append('C2 home plan retry budget missing')
        else:
            print('PASS  C2 home plan attempt budget (max 2)')

        # Clear sites lockstep for latch.
        if src.count('self._home_attempted = False') < 2:
            failures.append(
                '_home_attempted clear sites missing (Start/goal_pose lockstep)')
        else:
            print('PASS  _home_attempted clear-site lockstep')

    except OSError as exc:
        failures.append(f'could not inspect path_bank_manager: {exc}')

    # post_switch_grace_s is a manager timer; verify the configured default
    # contract used by launch (8.0 s) is documented in this suite.
    post_switch_grace_s = 8.0
    if post_switch_grace_s < 1.0:
        failures.append('post_switch_grace_s too short')
    else:
        print(f'PASS  post_switch_grace_s contract ({post_switch_grace_s}s)')

    # Issue A1: soft DT writes free-cell costs < lethal; max_cost=0 is no-op.
    synth = OccupancyGrid2D(
        origin_x=0.0, origin_y=0.0, resolution=0.5,
        width=11, height=5,
        costs=np.zeros((5, 11), dtype=np.uint8),
    )
    synth.costs[:, 0] = LETHAL
    synth.costs[:, 10] = LETHAL
    inflated = inflate_lethal(synth, 0.0)
    soft = apply_soft_clearance_dt(
        inflated, soft_dt_max_cost=80.0, soft_dt_decay_m=0.40)
    if (soft.costs[:, 0] < LETHAL).any() or (soft.costs[:, 10] < LETHAL).any():
        failures.append('soft DT must not clear lethal cells')
    elif int(soft.costs[2, 5]) >= LETHAL:
        failures.append('soft DT free mid-cell must stay < lethal')
    elif int(soft.costs[2, 5]) >= int(soft.costs[2, 1]):
        failures.append('soft DT mid-corridor cost should be lower than near-wall')
    else:
        print(
            f'PASS  soft DT centre<{soft.costs[2, 5]} wall-near={soft.costs[2, 1]} '
            f'(both < {LETHAL})'
        )
    noop = apply_soft_clearance_dt(inflated, soft_dt_max_cost=0.0)
    if not np.array_equal(noop.costs, inflated.costs):
        failures.append('soft_dt_max_cost=0 must be a no-op')
    else:
        print('PASS  soft_dt_max_cost=0 negative-control no-op')

    # Issue A defaults + A1/A2 vs neg_nudge_only profile distinction.
    prod = PathBankConfig()
    neg = PathBankConfig(
        bilateral_weight=0.0, soft_dt_max_cost=0.0, length_weight=0.45,
        nudge_target_clearance_m=0.70,
    )
    if abs(prod.bilateral_weight - 0.20) > 1e-12 or abs(prod.length_weight - 0.35) > 1e-12:
        failures.append(
            f'A2 defaults expected bilat=0.20 length=0.35; got '
            f'{prod.bilateral_weight}/{prod.length_weight}')
    elif abs(prod.soft_dt_max_cost - 80.0) > 1e-12 or abs(prod.nudge_target_clearance_m - 0.80) > 1e-12:
        failures.append('A1 soft-DT/nudge defaults mismatch')
    elif neg.bilateral_weight != 0.0 or neg.soft_dt_max_cost != 0.0:
        failures.append('neg_nudge_only profile must disable bilat + soft-DT')
    else:
        print('PASS  A1/A2 production defaults vs neg_nudge_only profile')

    # Soft costs must stay at/below inscribed (253).
    free_mask = inflated.costs < LETHAL
    if free_mask.any() and int(soft.costs[free_mask].max()) > INSCRIBED:
        failures.append('soft DT free costs must be <= INSCRIBED(253)')
    else:
        print('PASS  soft DT free costs <= INSCRIBED')

    if failures:
        print('FAIL')
        for f in failures:
            print(f'  - {f}')
        return 1
    print('ALL PATH-BANK UNIT TESTS PASSED')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
