#!/usr/bin/env python3
"""NAVIGATION TEST: configurable start, several goals, robot actually arrives.

Proves the plan-and-follow half of the autonomy loop:

    goal -> NavFn plans on the SLAM-backed global costmap
         -> DWB follows it on the rolling local costmap
         -> cmd_vel -> DiffDrive -> Husky moves -> goal reached

Three goals are driven from one simulation, in different directions and past
different trees, so a route that only works for one bearing cannot pass. The
start pose is supplied as a launch argument, which is also how a user changes
it, so this exercises the documented mechanism rather than a test-only path.

Success is not taken from Nav2's own "SUCCEEDED": the robot's TF pose must also
be within ARRIVAL_TOLERANCE of the goal. Nav2 reports success against its
configured goal checker, so checking geometry independently is what makes the
result trustworthy.

    python3 test/navigation_test.py
    python3 test/navigation_test.py --start 3 -3 1.57

Each goal stops the moment it is decided; nothing waits out a timeout.
"""

import argparse
import sys

import rclpy

from nav_test_lib import await_simulation, bringup, log, print_diagnostics
from rs1_nav import MissionRunner, NavObserver, init_ros

# Goals are in husky1_map. SLAM initialises that frame on the robot, so these
# coordinates are relative to the spawn pose, not the Gazebo world origin.
# simple_trees grass is only 15 x 15 m; a spawn far from the origin plus a
# southbound goal of 5 m can drive the robot off the plane (Gazebo then
# crashes in ODE). Keep start poses within about 2 m of the origin when using
# these goals. Pine at world (5, 0), oak at world (0, 3).
DEFAULT_START = (0.0, 0.0, 0.0)
GOALS = (
    (0.0, -5.0, 0.0),     # south, clear of both trees
    (-4.5, 1.5, 1.57),    # north-west, around the oak
    (3.5, -3.5, -0.7),    # south-east, around the pine
)

# 0.6 m/s top speed, but planning, turning and obstacle avoidance all cost
# time, so budget on an effective 0.25 m/s plus a fixed startup allowance.
SPEED_BUDGET = 0.25
FIXED_ALLOWANCE = 45.0


def goal_timeout(straight_line: float) -> float:
    return FIXED_ALLOWANCE + straight_line / SPEED_BUDGET


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--start', nargs=3, type=float, metavar=('X', 'Y', 'YAW'),
                        default=list(DEFAULT_START),
                        help='Husky start pose, passed to the launch file')
    parser.add_argument('--world', default='simple_trees', choices=['large_demo', 'simple_trees'])
    args = parser.parse_args(argv)
    start = tuple(args.start)

    sup = bringup(
        world=args.world,
        nav2=True,
        rviz=False,
        gui=False,
        husky_x=start[0],
        husky_y=start[1],
        husky_yaw=start[2],
        max_runtime=900.0,
        log_path='/tmp/navigation_test.log',
    )

    reports = []

    with sup:
        ok, _stages = await_simulation(sup, require_nav2=True)
        if not ok:
            print_diagnostics(sup, topics=['/husky1/scan', '/husky1/map'])
            log('NAVIGATION TEST FAILED (stack did not start)')
            return 1

        init_ros()
        observer = NavObserver()
        mission = MissionRunner(observer, logger=log)
        try:
            if not mission.wait_until_ready(timeout=180.0):
                print_diagnostics(sup, topics=['/husky1/plan'])
                return 1

            pose = observer.robot_pose()
            log(f'start pose requested ({start[0]:.2f}, {start[1]:.2f}, {start[2]:.2f}), '
                f'robot reports ({pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f})')

            for index, goal in enumerate(GOALS, start=1):
                log(f'--- goal {index} of {len(GOALS)} ---')
                here = observer.robot_pose()
                straight = ((goal[0] - here[0]) ** 2 + (goal[1] - here[1]) ** 2) ** 0.5
                report = mission.run(goal, timeout=goal_timeout(straight))
                reports.append(report)
                if not report.reached:
                    log(f'  goal {index} failed; stopping so the failure can be '
                        'diagnosed rather than masked by later goals')
                    print_diagnostics(sup, topics=['/husky1/plan', '/husky1/cmd_vel'])
                    break
        finally:
            observer.destroy_node()
            rclpy.shutdown()

    log('=' * 68)
    for index, report in enumerate(reports, start=1):
        log(f'  {"PASS" if report.reached else "FAIL"}  goal {index}: {report.summary()}')
        if report.reached and report.first_plan_length:
            efficiency = report.distance_travelled / max(report.straight_line, 0.01)
            log(f'          path quality: straight line {report.straight_line:.2f} m, '
                f'first plan {report.first_plan_length:.2f} m, '
                f'drove {report.distance_travelled:.2f} m ({efficiency:.2f}x direct)')
    passed = len(reports) == len(GOALS) and all(r.reached for r in reports)
    log(f'NAVIGATION TEST {"PASSED" if passed else "FAILED"} '
        f'({sum(1 for r in reports if r.reached)}/{len(GOALS)} goals reached)')
    log('=' * 68)
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
