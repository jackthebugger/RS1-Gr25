#!/usr/bin/env python3
"""Run the navigation test suite in order, stopping after the first failure.

Each test launches and tears down its own simulation. This runner only sequences
them and prints a summary. Individual tests still have their own timeouts.

    python3 test/run_nav_tests.py
    python3 test/run_nav_tests.py --only fast movement
"""

import argparse
import os
import subprocess
import sys
import time

TESTS = (
    ('geometry', 'geometry_unit_test.py', 30),
    ('fast', 'fast_test.py', 300),
    ('movement', 'movement_test.py', 300),
    ('obstacle', 'obstacle_injection_test.py', 240),
    ('tf_slam', 'tf_slam_test.py', 420),
    ('navigation', 'navigation_test.py', 900),
    ('replan', 'replan_test.py', 480),
)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--only', nargs='+', default=None,
                        help='Subset of test names to run')
    args = parser.parse_args(argv)

    here = os.path.dirname(os.path.abspath(__file__))
    selected = TESTS
    if args.only:
        wanted = set(args.only)
        selected = tuple(item for item in TESTS if item[0] in wanted)
        unknown = wanted - {item[0] for item in TESTS}
        if unknown:
            print(f'unknown tests: {sorted(unknown)}', file=sys.stderr)
            return 2

    workspace = os.path.abspath(os.path.join(here, '..', '..', '..', '..'))
    install_setup = os.path.join(workspace, 'install', 'setup.bash')
    ros_setup = '/opt/ros/humble/setup.bash'

    results = []
    overall_started = time.monotonic()
    for name, filename, timeout in selected:
        path = os.path.join(here, filename)
        print(f'\n{"=" * 68}\nRUN  {name}  ({filename}, cap {timeout}s)\n{"=" * 68}',
              flush=True)
        started = time.monotonic()
        sourced = (
            'set -o pipefail; '
            f'source {ros_setup} >/dev/null 2>&1; '
            f'source {install_setup} >/dev/null 2>&1; '
            'export ROS_LOCALHOST_ONLY=1; '
            f'exec {sys.executable} {path}'
        )
        try:
            done = subprocess.run(
                ['bash', '-lc', sourced],
                cwd=here,
                timeout=timeout,
                check=False,
            )
            code = done.returncode
        except subprocess.TimeoutExpired:
            print(f'FAIL  {name} exceeded the runner cap of {timeout}s', flush=True)
            code = 124
        elapsed = time.monotonic() - started
        results.append((name, code == 0, elapsed, code))
        print(f'{"PASS" if code == 0 else "FAIL"}  {name} in {elapsed:.1f}s (exit {code})',
              flush=True)
        if code != 0:
            break

    print('\n' + '=' * 68)
    for name, passed, elapsed, code in results:
        print(f'  {"PASS" if passed else "FAIL"}  {name:12}  {elapsed:7.1f}s  exit {code}')
    print(f'TOTAL {time.monotonic() - overall_started:.1f}s')
    print('=' * 68)
    return 0 if results and all(p for _n, p, _e, _c in results) and len(results) == len(selected) else 1


if __name__ == '__main__':
    sys.exit(main())
