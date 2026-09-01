#!/usr/bin/env python3
"""FAST TEST -- does the stack come up at all?

Brings up Gazebo + Husky + sensors + EKF + SLAM + Nav2 headless, verifies each
startup stage with a bounded wait, then stops immediately. Nothing here waits
for a fixed duration: as soon as the last stage passes, the simulation is torn
down.

    python3 test/fast_test.py [--no-nav2]
"""

import argparse
import sys
import time

from nav_test_lib import (
    await_simulation,
    bringup,
    log,
    print_diagnostics,
    ros2,
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--no-nav2', action='store_true', help='simulation + sensors only')
    parser.add_argument('--world', default='simple_trees')
    args = parser.parse_args()

    require_nav2 = not args.no_nav2
    started = time.monotonic()

    sup = bringup(
        world=args.world,
        nav2=require_nav2,
        rviz=False,
        gui=False,
        max_runtime=300.0,
        log_path='/tmp/fast_test_sim.log',
    )

    with sup:
        ok, stages = await_simulation(sup, require_nav2=require_nav2)

        if ok:
            # One extra piece of evidence: the scan must contain real ranges,
            # not just exist as a topic.
            code, out = ros2('topic echo /husky1/scan --once --field ranges', timeout=20.0)
            has_ranges = code == 0 and any(ch.isdigit() for ch in out)
            log(f'  {"OK   " if has_ranges else "FAIL "} lidar ranges populated')
            ok = ok and has_ranges

        if not ok:
            print_diagnostics(sup, topics=['/husky1/scan', '/husky1/cmd_vel'])

    elapsed = time.monotonic() - started
    log('=' * 68)
    for stage in stages:
        log(f'{"PASS" if stage.result.ok else "FAIL"}  {stage.name} ({stage.result.elapsed:.1f}s)')
    log(f'FAST TEST {"PASSED" if ok else "FAILED"} in {elapsed:.1f}s')
    log('=' * 68)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
