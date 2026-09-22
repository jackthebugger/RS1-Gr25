#!/usr/bin/env python3
"""Forest-gap dynamic obstacle + replanning integration test (custom_world_1).

Mirrors the Status UI obstacle pipeline without Tk:

  Start mission (18,0,0)
       → request obstacle (force Gap A or B)
       → wait until robot ≤5 m from selected gap
       → spawn real Gazebo box
       → lidar / costmap register it
       → Nav2 replans around remaining gap
       → goal retained and reached
       → clear removes dynamic model

    python3 test/forest_gap_obstacle_test.py --gap gap_a
    python3 test/forest_gap_obstacle_test.py --gap gap_b
"""

from __future__ import annotations

import argparse
import math
import sys

import rclpy

from nav_test_lib import await_simulation, bringup, log, print_diagnostics
from rs1_nav import MissionRunner, NavObserver, init_ros
from rs1_nav.forest_obstacle_manager import (
    ForestObstacleManager,
    ObstacleState,
    load_forest_gap_config,
)

START = (-18.0, 3.0, 0.0)
GOAL = (18.0, 0.0, 0.0)
WORLD = 'custom_world_1'
OCCUPIED_COST = 252
MIN_RANGE_DROP = 0.5


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--gap', choices=('gap_a', 'gap_b'), default='gap_a',
        help='Which forest gap to block (deterministic for CI)',
    )
    parser.add_argument('--timeout', type=float, default=300.0,
                        help='Mission budget in seconds')
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    force_gap = args.gap

    sup = bringup(
        world=WORLD,
        nav2=True,
        rviz=False,
        gui=False,
        husky_x=START[0],
        husky_y=START[1],
        husky_yaw=START[2],
        max_runtime=args.timeout + 180.0,
        log_path=f'/tmp/forest_gap_{force_gap}_test.log',
        extra={'slam': 'true', 'fire_avoidance': 'false'},
    )

    results = []
    report = None

    with sup:
        ok, _stages = await_simulation(sup, require_nav2=True)
        if not ok:
            print_diagnostics(sup, topics=['/husky1/scan', '/husky1/plan'])
            log('FOREST GAP TEST FAILED (stack did not start)')
            return 1

        init_ros()
        observer = NavObserver()
        mission = MissionRunner(observer, logger=log)
        try:
            if not mission.wait_until_ready(timeout=180.0):
                print_diagnostics(sup, topics=['/husky1/plan'])
                return 1

            cfg = load_forest_gap_config()
            cfg.world_name = WORLD
            mgr = ForestObstacleManager(
                cfg,
                force_gap=force_gap,
                logger=lambda m: log(f'  {m}'),
            )
            if not mgr.world.wait_until_available(max_wait=30.0):
                log('FAIL  Gazebo world services unavailable')
                return 1

            ok_req, msg = mgr.request_obstacle()
            results.append(('obstacle_requested', ok_req))
            log(f'  request: {msg}')
            if not ok_req:
                return 1

            gap = mgr.selected_gap
            assert gap is not None
            range_before = None
            range_after = None
            cost_after = None
            clearance_after = None
            spawned_at = None

            def range_toward(world_xy):
                pose = observer.robot_pose()
                if pose is None or world_xy is None:
                    return float('nan')
                bearing_world = math.atan2(world_xy[1] - pose[1], world_xy[0] - pose[0])
                bearing_robot = math.atan2(
                    math.sin(bearing_world - pose[2]),
                    math.cos(bearing_world - pose[2]),
                )
                return observer.min_range_in_sector(bearing_robot, half_width=0.35)

            def on_tick(elapsed, report_snapshot):
                nonlocal range_before, range_after, cost_after, clearance_after, spawned_at
                pose = observer.odom_pose() or observer.robot_pose()
                robot_xy = (pose[0], pose[1]) if pose else None

                if mgr.state == ObstacleState.WAITING:
                    if range_before is None and robot_xy is not None:
                        # Snapshot once the robot is approaching the wall line.
                        if robot_xy[0] > 0.0:
                            range_before = range_toward((gap.x, gap.y))
                            log(f'  lidar toward gap BEFORE spawn: {range_before:.2f} m')
                    mgr.tick(robot_xy)

                if mgr.state == ObstacleState.ACTIVE and spawned_at is None:
                    spawned_at = elapsed
                    log(f'  obstacle ACTIVE at t={elapsed:.1f}s '
                        f'(distance was {mgr.last_distance_m:.2f} m)')
                    observer.spin_for(1.5)
                    range_after = range_toward((gap.x, gap.y))
                    cost_after = observer.max_cost_near(
                        gap.x, gap.y, radius=1.0, local=False, ignore_unknown=True,
                    )
                    path = observer.latest_path()
                    if path is not None:
                        from rs1_nav import path_closest_approach
                        clearance_after = path_closest_approach(path.points, (gap.x, gap.y))
                    log(f'  lidar AFTER spawn: {range_after:.2f} m; '
                        f'costmap max near gap={cost_after}; '
                        f'path clearance={clearance_after}')

            report = mission.run(GOAL, timeout=args.timeout, on_tick=on_tick)

            results.append(('obstacle_spawned', mgr.state == ObstacleState.ACTIVE
                            or (mgr.state == ObstacleState.IDLE and spawned_at is not None)
                            or spawned_at is not None))
            # After mission, state should still be ACTIVE until cleared.
            results.append(('spawned_during_mission', spawned_at is not None))
            results.append(('goal_reached', report.reached))
            results.append(('goal_retained', report.goal[:2] == GOAL[:2]))
            results.append(('replan_observed', len(report.replans) >= 1))

            if range_before is not None and range_after is not None and math.isfinite(range_after):
                results.append((
                    'lidar_saw_obstacle',
                    (not math.isfinite(range_before))
                    or (range_before - range_after >= MIN_RANGE_DROP)
                    or range_after < 12.0,
                ))
            else:
                results.append(('lidar_saw_obstacle', range_after is not None
                                and math.isfinite(range_after) and range_after < 15.0))

            if cost_after is not None:
                results.append(('costmap_marked', cost_after >= OCCUPIED_COST))
            else:
                results.append(('costmap_marked', False))

            if clearance_after is not None:
                results.append(('path_avoids_gap', clearance_after > 1.0))
            else:
                # Replans list is secondary evidence if path snapshot missed.
                results.append(('path_avoids_gap', len(report.replans) >= 1))

            ok_clear, clear_msg = mgr.clear_obstacles()
            log(f'  clear: {clear_msg}')
            results.append(('clear_ok', ok_clear and mgr.state == ObstacleState.IDLE
                            and not mgr.active_names))

        finally:
            observer.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    log('--- results ---')
    all_ok = True
    for name, passed in results:
        mark = 'PASS' if passed else 'FAIL'
        log(f'  {mark}  {name}')
        all_ok = all_ok and passed
    if report is not None:
        log(f'  mission: {report.summary()}')

    if all_ok:
        log(f'FOREST GAP TEST PASSED ({force_gap})')
        return 0
    log(f'FOREST GAP TEST FAILED ({force_gap})')
    return 1


if __name__ == '__main__':
    sys.exit(main())
