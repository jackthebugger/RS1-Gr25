#!/usr/bin/env python3
"""Unit tests for Issue B far-gap join ranking + bridge prefix reject.

Run:

    python3 test/switch_join_unit_test.py
"""

from __future__ import annotations

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PKG, 'scripts'))

from path_bank_manager import (  # noqa: E402
    bridge_arc_length_m,
    bridge_detour_excessive,
    bridge_prefix_blocked,
    lidar_cone_clear,
    path_b_tangent_yaw,
    pick_drive_escape_heading,
    pick_escape_hop_pose,
    rank_switch_join_candidates,
    should_arm_sealed_start_unseal,
    should_restore_sealed_start_unseal,
)


def check(name, condition, detail=''):
    print(f'  {"PASS" if condition else "FAIL"}  {name}'
          + (f'  [{detail}]' if detail else ''))
    return condition


def main() -> int:
    results = []

    # Synthetic Path B: eastbound corridor north of a Path-A barrier pocket.
    # Robot at exclusive barrier facing east into keepout.
    robot = (4.3, -2.5)
    yaw = 0.0  # east
    goal = (16.0, -0.1)
    # Ahead-through-pocket join (classic R5 cand1) vs lateral escape join.
    route_pts = [
        (15.1, 0.83),   # ahead, little lateral — historically tried first
        (15.1, 3.50),
        (12.0, 6.00),   # more lateral / escape
        (10.0, -8.00),  # south escape
        (16.0, -0.05),  # near-goal snap (should be penalised)
    ]
    route_yaws = [0.0] * len(route_pts)

    ranked = rank_switch_join_candidates(
        robot, yaw, goal, route_pts, route_yaws, limit=5, dedup_m=1.0,
    )
    results.append(check(
        'rank returns candidates',
        len(ranked) >= 3,
        f'n={len(ranked)} first={ranked[0][:2] if ranked else None}',
    ))
    if ranked:
        # Lateral / progress joins should beat pure ahead-through-pocket.
        first_xy = (ranked[0][0], ranked[0][1])
        results.append(check(
            'first join is not the near-goal snap',
            math.hypot(first_xy[0] - 16.0, first_xy[1] + 0.05) >= 0.5,
            f'first={first_xy}',
        ))
        # Escape preference: lateral join should outrank (15.1, 0.83) ahead pocket.
        ahead = (15.1, 0.83)
        idx_ahead = next(
            (i for i, t in enumerate(ranked)
             if abs(t[0] - ahead[0]) < 0.05 and abs(t[1] - ahead[1]) < 0.05),
            None,
        )
        idx_lat = next(
            (i for i, t in enumerate(ranked)
             if abs(t[0] - 12.0) < 0.05 and abs(t[1] - 6.0) < 0.05),
            None,
        )
        if idx_lat is not None and idx_ahead is not None:
            results.append(check(
                'lateral escape join ranks before ahead-pocket join',
                idx_lat < idx_ahead,
                f'lat_i={idx_lat} ahead_i={idx_ahead}',
            ))
        else:
            results.append(check(
                'lateral escape join present in ranked set',
                idx_lat is not None,
                f'ranked={[(round(x,1), round(y,1)) for x,y,_,__ in ranked]}',
            ))

    # Empty route
    results.append(check(
        'empty route → empty joins',
        rank_switch_join_candidates(robot, yaw, goal, [], []) == [],
    ))

    # Behind-robot join must rank after forward joins
    with_behind = rank_switch_join_candidates(
        robot, yaw, goal,
        route_pts + [(-14.0, 5.7)],
        route_yaws + [3.14],
        limit=6, dedup_m=1.0,
    )
    behind_i = next(
        (i for i, t in enumerate(with_behind)
         if t[0] < 0.0),
        None,
    )
    results.append(check(
        'behind-robot join dropped from queue',
        behind_i is None,
        f'behind_i={behind_i} n={len(with_behind)}',
    ))

    # bridge_prefix_blocked: free prefix OK
    free_path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0), (4.0, 0.0), (5.0, 0.0)]

    def cost_free(_x, _y):
        return 0

    blocked, why = bridge_prefix_blocked(free_path, cost_free)
    results.append(check(
        'free prefix not blocked',
        not blocked and why == '',
        why,
    ))

    def cost_lethal(x, _y):
        # Lethal after 2.0 m along +x (past default skip_m=1.5)
        return 254 if x >= 2.0 else 0

    blocked, why = bridge_prefix_blocked(
        free_path, cost_lethal, prefix_m=5.0, skip_m=1.5, min_lethal_hits=2)
    results.append(check(
        'lethal prefix blocked after skip (2 hits)',
        blocked and 'prefix_lethal' in why,
        why,
    ))

    # Single lethal sample is not enough
    sparse = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0), (4.0, 0.0)]

    def cost_one_hit(x, _y):
        return 254 if abs(x - 2.5) < 0.2 else 0

    blocked, why = bridge_prefix_blocked(
        sparse, cost_one_hit, prefix_m=5.0, skip_m=1.5, min_lethal_hits=2)
    results.append(check(
        'single lethal hit not enough',
        not blocked,
        why,
    ))

    # Inscribed (253) alone must NOT block (R6 false reject class)
    def cost_inscribed(x, _y):
        return 253 if x >= 2.0 else 0

    blocked, why = bridge_prefix_blocked(
        free_path, cost_inscribed, prefix_m=5.0, skip_m=1.5)
    results.append(check(
        'inscribed-only prefix allowed',
        not blocked,
        why,
    ))

    # Skip window hides lethal at robot start
    def cost_at_start(x, _y):
        return 254 if x < 1.0 else 0

    blocked, why = bridge_prefix_blocked(
        free_path, cost_at_start, prefix_m=5.0, skip_m=1.5, min_lethal_hits=2)
    results.append(check(
        'lethal under skip_m ignored',
        not blocked,
        why,
    ))

    # Unknown (255) ignored
    def cost_unknown(_x, _y):
        return 255

    blocked, why = bridge_prefix_blocked(free_path, cost_unknown)
    results.append(check(
        'unknown cost ignored',
        not blocked,
        why,
    ))

    short = [(0.0, 0.0), (3.0, 0.0), (6.0, 0.0), (10.0, 0.0)]
    results.append(check(
        'arc length of short bridge',
        abs(bridge_arc_length_m(short) - 10.0) < 1e-9,
        f'{bridge_arc_length_m(short)}',
    ))
    # Zigzag long detour for a 10 m gap
    zigzag = [(0.0, 0.0)]
    for i in range(1, 60):
        zigzag.append((i * 0.5, 8.0 if i % 2 else -8.0))
    zigzag.append((10.0, 0.0))
    bad, why = bridge_detour_excessive(zigzag, gap_m=10.0)
    results.append(check(
        'absurd detour rejected',
        bad and 'detour_arc' in why,
        why,
    ))
    ok_d, why = bridge_detour_excessive(short, gap_m=10.0)
    results.append(check(
        'reasonable bridge allowed',
        not ok_d,
        why,
    ))

    # Escape hop: free cells lateral from eastbound robot
    def cost_corridor(x, y):
        # Lethal wall ahead (x>5, |y|<1); free elsewhere
        if x > 5.0 and abs(y) < 1.0:
            return 254
        return 0

    hop = pick_escape_hop_pose(
        (4.0, 0.0), 0.0, cost_corridor, hop_m=3.5,
        require_attract_progress=False,
    )
    results.append(check(
        'escape hop finds free lateral pose',
        hop is not None and abs(hop[1]) > 1.0,
        f'hop={hop}',
    ))
    results.append(check(
        'escape hop not reverse of heading',
        hop is not None and (hop[0] - 4.0) > -0.5,
        f'hop={hop}',
    ))

    # Entry 072: attract toward Path B (north) + repel from barrier locus.
    def cost_open(_x, _y):
        return 0

    barrier = (4.3, -2.5)
    hop_attr = pick_escape_hop_pose(
        (5.0, -1.8), 0.0, cost_open,
        attract_xy=(15.1, 3.5),
        repel_xy=barrier,
        hop_radii=(3.5, 5.0, 6.5),
        free_cost_max=80,
        require_attract_progress=True,
        min_repel_gain_m=1.25,
    )
    results.append(check(
        'attracted+repelled hop finds pose',
        hop_attr is not None,
        f'hop={hop_attr}',
    ))
    if hop_attr is not None:
        d0 = math.hypot(5.0 - 15.1, -1.8 - 3.5)
        d1 = math.hypot(hop_attr[0] - 15.1, hop_attr[1] - 3.5)
        r0 = math.hypot(5.0 - barrier[0], -1.8 - barrier[1])
        r1 = math.hypot(hop_attr[0] - barrier[0], hop_attr[1] - barrier[1])
        results.append(check(
            'hop reduces Path-B distance',
            d1 < d0 - 0.3,
            f'd0={d0:.2f} d1={d1:.2f} hop=({hop_attr[0]:.2f},{hop_attr[1]:.2f})',
        ))
        results.append(check(
            'hop increases barrier standoff',
            r1 >= r0 + 1.2,
            f'r0={r0:.2f} r1={r1:.2f}',
        ))
        results.append(check(
            'hop not deep-south of barrier (R10/R11 class)',
            hop_attr[1] > -2.0,
            f'hop_y={hop_attr[1]:.2f}',
        ))

    # DriveOnHeading escape: prefer heading that leaves barrier.
    drive_yaw = pick_drive_escape_heading(
        (5.0, -1.8), 0.0, cost_open,
        repel_xy=barrier, attract_xy=(15.1, 3.5), drive_m=2.5,
    )
    results.append(check(
        'drive escape heading found',
        drive_yaw is not None,
        f'yaw={drive_yaw}',
    ))
    if drive_yaw is not None:
        ex = 5.0 + 2.5 * math.cos(drive_yaw)
        ey = -1.8 + 2.5 * math.sin(drive_yaw)
        results.append(check(
            'drive escape increases barrier standoff',
            math.hypot(ex - barrier[0], ey - barrier[1])
            > math.hypot(5.0 - barrier[0], -1.8 - barrier[1]) + 0.7,
            f'end=({ex:.2f},{ey:.2f})',
        ))

    # Soft-band (cost 180) must not be accepted as hop target (R10 free_cost=200).
    def cost_soft_ring(x, y):
        # Entire 3.5 m ring around robot is soft inflation except unknown.
        d = math.hypot(x - 4.0, y - 0.0)
        if 3.0 <= d <= 4.0:
            return 180
        if x > 5.0 and abs(y) < 1.0:
            return 254
        return 0

    hop_soft = pick_escape_hop_pose(
        (4.0, 0.0), 0.0, cost_soft_ring,
        attract_xy=(15.0, 3.0),
        hop_radii=(3.5,),
        free_cost_max=80,
        require_attract_progress=True,
    )
    results.append(check(
        'soft-band hop ring rejected (free_cost_max=80)',
        hop_soft is None,
        f'hop={hop_soft}',
    ))

    # Entry 073: Path-B tangent + sealed-start unseal helpers (not hop).
    tyaw = path_b_tangent_yaw((5.3, -1.6), (15.1, 3.5))
    expected = math.atan2(3.5 - (-1.6), 15.1 - 5.3)
    results.append(check(
        'path_b_tangent_yaw toward attractor',
        abs(math.atan2(math.sin(tyaw - expected), math.cos(tyaw - expected)))
        < 1e-9,
        f'tyaw={tyaw:.3f} expected={expected:.3f}',
    ))
    # Clear forward cone: all ranges far.
    clear_ranges = [5.0] * 36
    ainc = (2.0 * math.pi) / 36.0
    results.append(check(
        'lidar_cone_clear with far returns',
        lidar_cone_clear(
            clear_ranges, -math.pi, ainc,
            cone_center_rad=0.0, half_width_rad=0.45, min_clear_m=1.25,
        ),
    ))
    blocked_ranges = [5.0] * 36
    # Index near angle 0.
    mid = int(round((0.0 - (-math.pi)) / ainc)) % 36
    blocked_ranges[mid] = 0.4
    results.append(check(
        'lidar_cone_clear rejects near return ahead',
        not lidar_cone_clear(
            blocked_ranges, -math.pi, ainc,
            cone_center_rad=0.0, half_width_rad=0.45, min_clear_m=1.25,
        ),
    ))
    results.append(check(
        'lidar_cone_clear rejects empty scan (no evidence)',
        not lidar_cone_clear([], -math.pi, 0.1),
    ))
    results.append(check(
        'arm unseal on barrier NE corner far-gap',
        should_arm_sealed_start_unseal(10.8, 1.2),
    ))
    results.append(check(
        'no arm when already on Path B',
        not should_arm_sealed_start_unseal(1.5, 1.2),
    ))
    results.append(check(
        'no restore before min_hold even if far',
        not should_restore_sealed_start_unseal(6.0, 0.0, 1.0)[0],
    ))
    results.append(check(
        'restore when cleared locus after min_hold',
        should_restore_sealed_start_unseal(5.1, 0.0, 8.5)[0],
    ))
    results.append(check(
        'restore on max hold when not deferred',
        should_restore_sealed_start_unseal(1.0, 0.0, 45.1, defer=False)[0],
    ))
    results.append(check(
        'defer blocks soft max_hold while reorienting',
        not should_restore_sealed_start_unseal(1.0, 0.0, 45.1, defer=True)[0],
    ))
    results.append(check(
        'hard_max restores even when deferred',
        should_restore_sealed_start_unseal(1.0, 0.0, 60.1, defer=True)[0],
    ))
    results.append(check(
        'hold while still sealed and under max hold',
        not should_restore_sealed_start_unseal(1.0, 0.0, 20.0)[0],
    ))

    n_ok = sum(1 for r in results if r)
    print(f'\n{n_ok}/{len(results)} PASS')
    return 0 if n_ok == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
