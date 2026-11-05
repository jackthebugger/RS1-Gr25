#!/usr/bin/env python3
"""Unit tests for Issue B yaw-gate helpers (no Gazebo / Nav2).

Run:

    python3 test/reorient_yaw_unit_test.py
"""

from __future__ import annotations

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PKG, 'scripts'))

from path_bank_manager import (  # noqa: E402
    PATH_YAW_LOOKAHEAD_M,
    SPIN_TIMEOUT_S,
    THETA_EXIT_RAD,
    THETA_REORIENT_RAD,
    wrap_angle,
    yaw_aligned,
    yaw_error,
)


def check(name, condition, detail=''):
    print(f'  {"PASS" if condition else "FAIL"}  {name}'
          + (f'  [{detail}]' if detail else ''))
    return condition


def main() -> int:
    results = []

    results.append(check(
        'wrap_angle π+ε → −π+ε',
        abs(wrap_angle(math.pi + 0.1) + (math.pi - 0.1)) < 1e-9,
        f'{wrap_angle(math.pi + 0.1)}',
    ))
    results.append(check(
        'yaw_error 0→π/2 is +π/2',
        abs(yaw_error(0.0, math.pi / 2) - math.pi / 2) < 1e-9,
    ))
    results.append(check(
        'yaw_error wraps across ±π',
        abs(yaw_error(3.0, -3.0) - wrap_angle(-3.0 - 3.0)) < 1e-9,
    ))

    results.append(check(
        'θ_exit / θ_reorient locked',
        abs(THETA_EXIT_RAD - 0.45) < 1e-12
        and abs(THETA_REORIENT_RAD - 0.60) < 1e-12,
    ))
    results.append(check(
        '|err| < θ_exit → aligned (may skip Spin)',
        yaw_aligned(0.44) and yaw_aligned(-0.44),
    ))
    results.append(check(
        '|err| ≥ θ_exit → NOT aligned (must Spin/fallback; no FP)',
        not yaw_aligned(0.45) and not yaw_aligned(0.60) and not yaw_aligned(-1.0),
        f'aligned(0.45)={yaw_aligned(0.45)}',
    ))
    results.append(check(
        'θ_exit is stricter than θ_reorient entry marker',
        THETA_EXIT_RAD < THETA_REORIENT_RAD,
    ))

    # Authority invariant: timeout alone must not waive the gate predicate.
    timeout_err = 0.50
    results.append(check(
        'timeout-without-heading still fails yaw_aligned',
        not yaw_aligned(timeout_err, THETA_EXIT_RAD),
        f'err={timeout_err}',
    ))
    results.append(check(
        'path yaw look-ahead locked (> first-segment vacuous gate)',
        PATH_YAW_LOOKAHEAD_M >= 4.0,
        f'lookahead={PATH_YAW_LOOKAHEAD_M}',
    ))
    results.append(check(
        'Spin timeout allows ~π at min_rotational_vel',
        SPIN_TIMEOUT_S >= 12.0,
        f'spin_timeout={SPIN_TIMEOUT_S}',
    ))

    ok = all(results)
    print(f'REORIENT YAW UNIT TEST {"PASSED" if ok else "FAILED"} '
          f'({sum(results)}/{len(results)})')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
