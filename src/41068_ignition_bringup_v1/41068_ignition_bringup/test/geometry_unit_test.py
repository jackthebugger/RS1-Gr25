#!/usr/bin/env python3
"""Fast unit tests for path geometry used by the replan injector. No simulation."""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rs1_nav.geometry import path_closest_approach, point_ahead_on_path


def check(name, condition, detail=''):
    print(f'  {"PASS" if condition else "FAIL"}  {name}' + (f'  [{detail}]' if detail else ''))
    return condition


def main() -> int:
    path = [(0.0, 0.0), (4.0, 0.0), (4.0, 3.0)]
    results = []

    dist = path_closest_approach(path, (2.0, 1.0))
    results.append(check('closest approach to a point beside the first segment',
                         dist is not None and abs(dist - 1.0) < 1e-6, f'{dist}'))

    ahead, heading = point_ahead_on_path(path, (0.0, 0.0), 2.0)
    results.append(check('2 m ahead of origin along +X',
                         abs(ahead[0] - 2.0) < 1e-6 and abs(ahead[1]) < 1e-6
                         and abs(heading) < 1e-6, f'{ahead} h={heading}'))

    ahead, heading = point_ahead_on_path(path, (3.5, 0.0), 1.0)
    # 0.5 m left on first segment + 0.5 m up the second → (4.0, 0.5), heading +Y
    results.append(check('look-ahead crosses a corner',
                         abs(ahead[0] - 4.0) < 1e-6 and abs(ahead[1] - 0.5) < 1e-6
                         and abs(heading - math.pi / 2) < 1e-6, f'{ahead} h={heading}'))

    ok = all(results)
    print(f'GEOMETRY UNIT TEST {"PASSED" if ok else "FAILED"}')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
