#!/usr/bin/env python3
"""Fast unit tests for Wave 2 clearance metrics (C_min, C_body, bilateral).

Synthetic costmap only — no Gazebo. Run:

    python3 test/clearance_metrics_unit_test.py
"""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rs1_nav.geometry import (
    FOOTPRINT_HALF_WIDTH_M,
    bilateral_balance,
    c_body_m,
    c_min_m,
    costmap_obstacle_cells,
    path_clearance_metrics,
)
from rs1_nav.path_bank import PathBankConfig, score_candidate


def check(name, condition, detail=''):
    print(f'  {"PASS" if condition else "FAIL"}  {name}'
          + (f'  [{detail}]' if detail else ''))
    return condition


def main() -> int:
    results = []

    # Corridor: lethals at y=0 and y=4 along x; path on y=2 (centred).
    # resolution 1.0, origin (0,0); cells are unit squares centred at i+0.5.
    # Build 5×5 grid: lethal on rows y=0 and y=4 for x=0..4.
    w, h = 5, 5
    data = [0] * (w * h)
    for x in range(w):
        data[0 * w + x] = 254
        data[4 * w + x] = 254
    obstacles = costmap_obstacle_cells(data, w, h, 0.0, 0.0, 1.0)
    path = [(0.5, 2.5), (2.5, 2.5), (4.5, 2.5)]

    metrics = path_clearance_metrics(path, obstacles, sample_stride=1)
    # Nearest lethal centres are at y=0.5 and y=4.5 → distance 2.0 from y=2.5.
    results.append(check(
        'C_min centred corridor ≈ 2.0 m',
        metrics.c_min_m is not None and abs(metrics.c_min_m - 2.0) < 1e-6,
        f'c_min={metrics.c_min_m}',
    ))
    expected_body = 2.0 - FOOTPRINT_HALF_WIDTH_M
    results.append(check(
        'C_body = C_min − 0.38',
        metrics.c_body_m is not None
        and abs(metrics.c_body_m - expected_body) < 1e-6,
        f'c_body={metrics.c_body_m}',
    ))
    results.append(check(
        'c_min_m / c_body_m helpers agree',
        abs((c_min_m(path, obstacles) or -1) - 2.0) < 1e-6
        and abs((c_body_m(2.0) or -1) - expected_body) < 1e-6,
    ))
    results.append(check(
        'bilateral balance near 1.0 when L≈R',
        metrics.bilateral_balance is not None
        and metrics.bilateral_balance > 0.95,
        f'bal={metrics.bilateral_balance} '
        f'L={metrics.left_clearance_m} R={metrics.right_clearance_m}',
    ))

    # Knife-edge: path at y=0.5 (through lethal row centres) → C_min≈0.
    knife = [(0.5, 0.5), (2.5, 0.5), (4.5, 0.5)]
    knife_m = path_clearance_metrics(knife, obstacles)
    results.append(check(
        'knife-edge C_min ≈ 0 and C_body negative',
        knife_m.c_min_m is not None and knife_m.c_min_m < 0.05
        and knife_m.c_body_m is not None and knife_m.c_body_m < 0.0,
        f'c_min={knife_m.c_min_m} c_body={knife_m.c_body_m}',
    ))

    # Asymmetric: only bottom wall → right-biased path should unbalance.
    data_one = [0] * (w * h)
    for x in range(w):
        data_one[0 * w + x] = 254
    obs_one = costmap_obstacle_cells(data_one, w, h, 0.0, 0.0, 1.0)
    asy = path_clearance_metrics([(0.5, 1.5), (4.5, 1.5)], obs_one)
    results.append(check(
        'asymmetric corridor bilateral < 1.0',
        asy.bilateral_balance is not None and asy.bilateral_balance < 0.99,
        f'bal={asy.bilateral_balance}',
    ))
    b_eq = bilateral_balance(1.0, 1.0)
    b_one = bilateral_balance(2.0, 0.0)
    results.append(check(
        'bilateral_balance helper 1.0 / 0.0 edges',
        b_eq is not None and abs(b_eq - 1.0) < 1e-9
        and b_one is not None and abs(b_one - 0.0) < 1e-9,
        f'eq={b_eq} one_sided={b_one}',
    ))

    # score_candidate: bilateral_weight=0 (neg-control) ignores bilateral term.
    cfg_neg = PathBankConfig(
        bilateral_weight=0.0, soft_dt_max_cost=0.0, length_weight=0.45,
        nudge_target_clearance_m=0.70,
    )
    cfg0 = PathBankConfig()
    cfg_b = PathBankConfig(bilateral_weight=0.20, clearance_weight=0.15)
    base = score_candidate(
        10.0, 1.0, 0.0, 0.0, ref_length_m=10.0, cfg=cfg_neg,
        bilateral_balance_01=0.5,
    )
    same = score_candidate(
        10.0, 1.0, 0.0, 0.0, ref_length_m=10.0, cfg=cfg_neg,
        bilateral_balance_01=1.0,
    )
    results.append(check(
        'bilateral_weight=0 ignores bilateral term',
        abs(base - same) < 1e-9, f'{base} vs {same}',
    ))
    low = score_candidate(
        10.0, 1.0, 0.0, 0.0, ref_length_m=10.0, cfg=cfg_b,
        bilateral_balance_01=0.0,
    )
    high = score_candidate(
        10.0, 1.0, 0.0, 0.0, ref_length_m=10.0, cfg=cfg_b,
        bilateral_balance_01=1.0,
    )
    results.append(check(
        'bilateral_weight>0 raises score when balanced',
        high > low + 1.0, f'low={low:.2f} high={high:.2f}',
    ))
    results.append(check(
        'production A1/A2 defaults (soft-DT + bilateral + centre nudge)',
        abs(cfg0.clearance_weight - 0.35) < 1e-12
        and abs(cfg0.bilateral_weight - 0.20) < 1e-12
        and abs(cfg0.length_weight - 0.35) < 1e-12
        and abs(cfg0.nudge_target_clearance_m - 0.80) < 1e-12
        and abs(cfg0.soft_dt_max_cost - 80.0) < 1e-12
        and abs(cfg0.soft_dt_decay_m - 0.40) < 1e-12,
    ))
    # Negative control: nudge-only / no soft-DT / bilateral off must remain a
    # distinct profile from full A1+A2 (centreline gates use this inequality).
    results.append(check(
        'neg_nudge_only profile disables soft-DT + bilateral',
        abs(cfg_neg.bilateral_weight - 0.0) < 1e-12
        and abs(cfg_neg.soft_dt_max_cost - 0.0) < 1e-12
        and cfg_neg.nudge_target_clearance_m >= 0.70,
        f'bilat={cfg_neg.bilateral_weight} dt={cfg_neg.soft_dt_max_cost}',
    ))

    # Empty / no obstacles → None metrics.
    empty = path_clearance_metrics(path, [])
    results.append(check(
        'empty obstacles → None C_min',
        empty.c_min_m is None and empty.c_body_m is None,
    ))

    ok = all(results)
    print(f'CLEARANCE METRICS UNIT TEST {"PASSED" if ok else "FAILED"} '
          f'({sum(results)}/{len(results)})')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
