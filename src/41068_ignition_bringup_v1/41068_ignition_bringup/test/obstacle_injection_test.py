#!/usr/bin/env python3
"""Verify that an injected obstacle is genuinely SEEN BY THE LIDAR.

This underpins the dynamic-replanning test, so it is proven on its own: if the
obstacle were invisible to the sensor, a later "replan" would prove nothing.

Method: leave the robot parked, record the lidar range straight ahead, insert a
barrier at a known distance, and confirm the measured range collapses to that
distance. Ranges come from subscribed LaserScan messages, not from parsing
`ros2 topic echo`, which truncates long arrays.

    python3 test/obstacle_injection_test.py
"""

import sys

import rclpy

from nav_test_lib import await_simulation, bringup, log, print_diagnostics
from rs1_nav import GazeboWorld, NavObserver, ObstacleSpec, init_ros

# The robot spawns at the origin facing +X. The barrier must be nearer than
# anything already in the world, or the lidar would simply keep reporting the
# closer object: simple_trees has a pine at (5, 0) whose foliage is measured at
# about 3.6 m, so 2 m of clear ground gives an unambiguous before/after.
OBSTACLE_X = 2.0
OBSTACLE_Y = 0.0
THICKNESS = 0.4
TOLERANCE = 0.4
MIN_CLEARANCE = 0.8  # the barrier must shorten the range by at least this much


def main() -> int:
    sup = bringup(
        world='simple_trees',
        nav2=False,          # only the sensor pipeline is under test
        rviz=False,
        gui=False,
        max_runtime=240.0,
        log_path='/tmp/obstacle_injection_test.log',
    )

    passed = False
    detail = 'did not run'

    with sup:
        ok, _stages = await_simulation(sup, require_nav2=False)
        if not ok:
            print_diagnostics(sup, topics=['/husky1/scan'])
            log('OBSTACLE INJECTION TEST FAILED (simulation did not start)')
            return 1

        init_ros()
        probe = NavObserver()
        try:
            if not probe.wait_for('lidar messages received',
                                  lambda: probe.scan is not None, timeout=30.0):
                return 1

            before = probe.min_range_in_sector(centre_angle=0.0, half_width=0.20)
            log(f'lidar range straight ahead BEFORE injection: {before:.2f} m')
            if before < OBSTACLE_X + MIN_CLEARANCE:
                log(f'FAIL  the obstacle site is not clear ({before:.2f} m), so the '
                    'measurement could not distinguish it from existing geometry')
                return 1

            world = GazeboWorld('simple_trees', logger=lambda m: log(f'  {m}'))
            if not world.wait_until_available(max_wait=30.0):
                log('FAIL  Gazebo world services unavailable')
                return 1

            spec = ObstacleSpec(
                name='injection_probe',
                x=OBSTACLE_X, y=OBSTACLE_Y,
                size_x=THICKNESS, size_y=3.0, size_z=1.5,
            )
            if not world.spawn_obstacle(spec):
                log('FAIL  could not insert obstacle')
                return 1

            expected = OBSTACLE_X - 0.5 * THICKNESS
            seen = probe.wait_for(
                f'obstacle detected at ~{expected:.2f} m',
                lambda: abs(probe.min_range_in_sector(0.0, 0.20) - expected) <= TOLERANCE,
                timeout=25.0,
                on_timeout_detail=lambda: f'range is {probe.min_range_in_sector(0.0, 0.20):.2f} m',
            )
            after = probe.min_range_in_sector(0.0, 0.20)
            log(f'lidar range straight ahead AFTER injection : {after:.2f} m '
                f'(expected ~{expected:.2f} m)')

            # A genuine detection must also be a CHANGE: the range has to have
            # shortened, otherwise we are just measuring pre-existing geometry.
            shortened = before - after >= MIN_CLEARANCE
            log(f'range shortened by {before - after:.2f} m')
            passed = seen and shortened
            detail = (f'before={before:.2f} after={after:.2f} expected={expected:.2f}')

            world.remove_model(spec.name)
        finally:
            probe.destroy_node()
            rclpy.shutdown()

    log('=' * 68)
    log(f'OBSTACLE INJECTION TEST {"PASSED" if passed else "FAILED"}  [{detail}]')
    log('=' * 68)
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
