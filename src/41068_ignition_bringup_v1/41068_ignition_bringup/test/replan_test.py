#!/usr/bin/env python3
"""DYNAMIC REPLANNING TEST — live sensor data must force a new route.

    start → goal
         ↓
    robot begins following the first plan
         ↓
    a real Gazebo barrier is inserted on that plan
         ↓
    lidar range to the site collapses
         ↓
    Nav2 costmap marks the cells
         ↓
    current path becomes invalid / Nav2 publishes a diverging plan
         ↓
    robot follows the new route around the barrier
         ↓
    goal reached

Nothing about Nav2 is told that an obstacle was added. The barrier is a genuine
model, so a "replan" that happened without a lidar change would fail this test.

    python3 test/replan_test.py
"""

import math
import sys

import rclpy

from nav_test_lib import await_simulation, bringup, log, print_diagnostics
from rs1_nav import (
    GazeboWorld,
    MissionRunner,
    NavObserver,
    PathBlocker,
    init_ros,
    path_closest_approach,
)

START = (0.0, 0.0, 0.0)
# Far enough south that a mid-path wall still leaves ~2 m of open ground
# between the barrier and the goal, so the detour can finish.
GOAL = (0.0, -6.0, 0.0)
WORLD = 'simple_trees'
# Inscribed (253) or lethal (254). 255 is NO_INFORMATION and must not count:
# a rolling global costmap is mostly unknown, so max() would always be 255.
OCCUPIED_COST = 252
MIN_RANGE_DROP = 0.8


def main() -> int:
    sup = bringup(
        world=WORLD,
        nav2=True,
        rviz=False,
        gui=False,
        husky_x=START[0],
        husky_y=START[1],
        husky_yaw=START[2],
        max_runtime=420.0,
        log_path='/tmp/replan_test.log',
    )

    results = []
    report = None

    with sup:
        ok, _stages = await_simulation(sup, require_nav2=True)
        if not ok:
            print_diagnostics(sup, topics=['/husky1/scan', '/husky1/plan'])
            log('REPLAN TEST FAILED (stack did not start)')
            return 1

        init_ros()
        observer = NavObserver()
        mission = MissionRunner(observer, logger=log)
        try:
            if not mission.wait_until_ready(timeout=180.0):
                print_diagnostics(sup, topics=['/husky1/plan'])
                return 1

            world = GazeboWorld(WORLD, logger=lambda m: log(f'  {m}'))
            if not world.wait_until_available(max_wait=30.0):
                log('FAIL  Gazebo world services unavailable')
                return 1

            blocker = PathBlocker(observer, world, logger=lambda m: log(f'  {m}'))
            range_before = None
            range_after = None
            cost_after = None
            clearance_after = None

            def range_toward(world_xy):
                pose = observer.robot_pose()
                if pose is None or world_xy is None:
                    return float('nan')
                bearing_world = math.atan2(world_xy[1] - pose[1], world_xy[0] - pose[0])
                bearing_robot = math.atan2(
                    math.sin(bearing_world - pose[2]),
                    math.cos(bearing_world - pose[2]),
                )
                return observer.min_range_in_sector(bearing_robot, 0.35)

            def on_tick(_elapsed, current_report):
                nonlocal range_before, range_after, cost_after, clearance_after
                if not blocker.injected:
                    if range_before is None:
                        range_before = range_toward(GOAL[:2])
                    blocker.maybe_inject(_elapsed, current_report)
                    return
                xy = blocker.injection_xy
                if xy is None:
                    return
                # Do not nest spin_for here: MissionRunner already spins.
                if range_after is None:
                    measured = range_toward(xy)
                    if measured == measured:  # not NaN
                        range_after = measured
                cost = observer.max_cost_near(
                    xy[0], xy[1], radius=1.0, local=False, ignore_unknown=True)
                if cost is not None and (cost_after is None or cost > cost_after):
                    cost_after = cost
                path = observer.latest_path()
                if path is not None:
                    clearance_after = path_closest_approach(path.points, xy)

            report = mission.run(GOAL, timeout=90.0, on_tick=on_tick)

            results.append(('obstacle inserted as a Gazebo model',
                            blocker.injected,
                            '' if blocker.injected else 'PathBlocker never fired'))

            lidar_ok = (
                range_after is not None
                and range_before is not None
                and (range_before - range_after) >= MIN_RANGE_DROP
            ) or (
                range_after is not None and range_after < 3.5
            )
            results.append(
                ('lidar range collapsed after insertion',
                 lidar_ok,
                 f'before={range_before} after={range_after}'))

            costmap_ok = cost_after is not None and cost_after >= OCCUPIED_COST
            results.append(
                ('global costmap marked the barrier',
                 costmap_ok,
                 f'max cost near insertion={cost_after}'))

            replan_ok = bool(report.replans)
            results.append(
                ('Nav2 published a diverging plan',
                 replan_ok,
                 f'{len(report.replans)} replan(s) from {report.plans_received} plans'))

            if clearance_after is not None:
                results.append(
                    ('new plan keeps clear of the barrier',
                     clearance_after >= 0.6,
                     f'closest approach {clearance_after:.2f} m'))

            results.append(
                ('robot reached the goal after the replan',
                 report.reached,
                 report.summary()))

            if blocker.spec is not None:
                world.remove_model(blocker.spec.name)
        finally:
            observer.destroy_node()
            rclpy.shutdown()

    log('=' * 68)
    for name, passed, detail in results:
        log(f'  {"PASS" if passed else "FAIL"}  {name}' + (f'  [{detail}]' if detail else ''))
    all_passed = bool(results) and all(p for _n, p, _d in results)
    log(f'REPLAN TEST {"PASSED" if all_passed else "FAILED"}')
    log('=' * 68)
    return 0 if all_passed else 1


if __name__ == '__main__':
    sys.exit(main())
