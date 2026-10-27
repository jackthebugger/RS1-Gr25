#!/usr/bin/env python3
"""Entry 019+ bush_trail environment validation (no path-bank switching).

Checks:
  1. Lidar sees nearby trail boundaries (finite ranges < corridor scale)
  2. Global costmap has lethal cells near spawn (StaticLayer / walls)
  3. Dynamic Gazebo barrier → lidar range drop → plan divergence → goal
  4. fire_detected → fire_hazards PointCloud → costmap mark near hazard

    python3 test/bush_trail_env_validation.py
"""

from __future__ import annotations

import math
import sys
import time

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool

from nav_test_lib import await_simulation, bringup, log, print_diagnostics
from rs1_nav import (
    GazeboWorld,
    MissionRunner,
    NavObserver,
    PathBlocker,
    init_ros,
    path_closest_approach,
)

START = (-18.0, 3.0, 0.0)
GOAL = (16.0, -0.1, 0.0)
WORLD = 'bush_trail_world'
OCCUPIED_COST = 252


def _finite_ranges(observer, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        observer.spin_for(0.2)
        scan = observer.scan
        if scan is None or not scan.ranges:
            continue
        vals = [r for r in scan.ranges if math.isfinite(r) and r > 0.05]
        if vals:
            return vals
    return []


def test_lidar_boundaries(observer) -> bool:
    log('=== TEST: Lidar trail boundaries ===')
    vals = _finite_ranges(observer, timeout=8.0)
    if not vals:
        log('FAIL  no finite lidar ranges')
        return False
    near = [r for r in vals if r < 8.0]
    med = float(sorted(vals)[len(vals) // 2])
    log(f'  finite={len(vals)} near(<8m)={len(near)} median={med:.2f}m min={min(vals):.2f}m')
    # On a bounded trail the robot should see walls within a few metres on
    # multiple beams (not an open-field median >> 20 m).
    ok = len(near) >= 8 and med < 15.0
    log('PASS' if ok else 'FAIL')
    return ok


def test_costmap_static(observer) -> bool:
    log('=== TEST: Global costmap static occupancy ===')
    deadline = time.monotonic() + 20.0
    grid = None
    while time.monotonic() < deadline:
        observer.spin_for(0.3)
        grid = observer.global_costmap
        if grid is not None and grid.metadata.size_x > 0:
            break
    if grid is None:
        log('FAIL  no global costmap')
        return False
    data = list(grid.data)
    lethal = sum(1 for c in data if c >= OCCUPIED_COST)
    free = sum(1 for c in data if 0 <= c < 50)
    frac = lethal / max(1, len(data))
    log(
        f'  size={grid.metadata.size_x}x{grid.metadata.size_y} '
        f'lethal_frac={frac:.3f} free≈{free}'
    )
    # Regenerated PGM is ~65% occupied; StaticLayer should mark a large lethal share.
    ok = frac > 0.15 and free > 1000
    log('PASS' if ok else 'FAIL')
    return ok


def test_dynamic_obstacle(observer, mission, world) -> bool:
    log('=== TEST: Dynamic on-trail obstacle ===')
    blocker = PathBlocker(
        observer, world,
        min_travel=2.0, look_ahead=3.0,
        width=2.2, thickness=0.45, height=1.6,
        name='bush_trail_dyn_block',
        logger=lambda m: log(f'  {m}'),
    )
    range_before = None
    range_after = None
    clearance_after = None
    diverged = False

    def range_toward(xy):
        pose = observer.robot_pose()
        if pose is None or xy is None:
            return float('nan')
        bearing_world = math.atan2(xy[1] - pose[1], xy[0] - pose[0])
        bearing_robot = math.atan2(
            math.sin(bearing_world - pose[2]),
            math.cos(bearing_world - pose[2]),
        )
        return observer.min_range_in_sector(bearing_robot, 0.35)

    cost_after = None

    def on_tick(elapsed, report):
        nonlocal range_before, range_after, clearance_after, diverged, cost_after
        if not blocker.injected:
            if range_before is None and report.plans_received >= 1:
                path = observer.latest_path()
                pose = observer.robot_pose()
                if path and pose:
                    from rs1_nav.geometry import point_ahead_on_path
                    ahead = point_ahead_on_path(path.points, (pose[0], pose[1]), 3.0)
                    if ahead:
                        range_before = range_toward(ahead[0])
            blocker.maybe_inject(elapsed, report)
            return
        xy = blocker.injection_xy
        if xy is None:
            return
        if range_after is None:
            measured = range_toward(xy)
            if measured == measured:
                range_after = measured
        cost = observer.max_cost_near(
            xy[0], xy[1], radius=1.0, local=False, ignore_unknown=True)
        if cost is not None and (cost_after is None or cost > cost_after):
            cost_after = cost
        path = observer.latest_path()
        if path is not None:
            clr = path_closest_approach(path.points, xy)
            if clr is not None:
                clearance_after = clr
                if clr > 1.2:
                    diverged = True

    report = mission.run(GOAL, timeout=320.0, on_tick=on_tick)
    log(report.summary())
    log(
        f'  barrier={blocker.injection_xy} range_before={range_before} '
        f'range_after={range_after} cost_after={cost_after} '
        f'clear_after={clearance_after} diverged={diverged} replans={len(report.replans)}'
    )
    lidar_ok = (
        range_after is not None
        and (
            (range_before is not None and range_before - range_after >= 0.5)
            or range_after < 3.5
        )
    )
    cost_ok = cost_after is not None and cost_after >= OCCUPIED_COST
    ok = report.reached and lidar_ok and (diverged or bool(report.replans) or cost_ok)
    log('PASS' if ok else 'FAIL')
    return ok


def test_fire_pipeline(observer) -> bool:
    log('=== TEST: Fire detected → fire_hazards ===')
    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
    )
    clouds = []

    def on_cloud(msg: PointCloud2):
        clouds.append(msg)

    sub = observer.create_subscription(PointCloud2, 'fire_hazards', on_cloud, qos)
    pub = observer.create_publisher(Bool, 'fire_detected', 10)
    # Pulse fire_detected true for several seconds
    deadline = time.monotonic() + 12.0
    while time.monotonic() < deadline:
        msg = Bool()
        msg.data = True
        pub.publish(msg)
        observer.spin_for(0.25)
    n_pts = 0
    if clouds:
        c = clouds[-1]
        # xyz float32 → 12 bytes/point typically
        step = max(1, c.point_step)
        n_pts = int(c.width * c.height)
        if n_pts == 0 and c.data:
            n_pts = len(c.data) // step
    log(f'  fire_hazards msgs={len(clouds)} last_points≈{n_pts}')
    observer.destroy_subscription(sub)
    ok = len(clouds) >= 1 and n_pts >= 4
    log('PASS' if ok else 'FAIL')
    return ok


def main() -> int:
    sup = bringup(
        world=WORLD,
        nav2=True,
        rviz=False,
        gui=False,
        husky_x=START[0],
        husky_y=START[1],
        husky_yaw=START[2],
        max_runtime=900.0,
        log_path='/tmp/bush_trail_env_validation.log',
        extra={
            'husky_z': '0.50',
            'use_prior_map': 'true',
            'prior_map_file': 'bush_trail_world.yaml',
            'path_bank': 'false',
            'fire_avoidance': 'true',
        },
    )
    results = {}
    with sup:
        ok, _ = await_simulation(sup, require_nav2=True)
        if not ok:
            print_diagnostics(sup)
            return 1
        init_ros()
        observer = NavObserver()
        mission = MissionRunner(observer, logger=log)
        if not mission.wait_until_ready(timeout=180.0):
            return 1
        world = GazeboWorld(WORLD, logger=lambda m: log(f'  {m}'))
        if not world.wait_until_available(max_wait=30.0):
            return 1

        results['lidar_boundaries'] = test_lidar_boundaries(observer)
        results['costmap_static'] = test_costmap_static(observer)
        results['fire_pipeline'] = test_fire_pipeline(observer)
        results['dynamic_obstacle'] = test_dynamic_obstacle(observer, mission, world)

    log('=== SUMMARY ===')
    for k, v in results.items():
        log(f'  {k}: {"PASS" if v else "FAIL"}')
    return 0 if results and all(results.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
