#!/usr/bin/env python3
"""Verify the TF tree and that SLAM builds a usable map from live lidar data.

The environment representation Nav2 plans in is only as good as this layer, so
the test checks the things that silently break it.

TF authority. Three nodes publish transforms and each owns a different edge:

    husky1_map  -> husky1_odom        slam_toolbox
    husky1_odom -> husky1_base_link   robot_localization (EKF)
    husky1_base_link -> sensors/wheels robot_state_publisher

The repository audit flagged a possible AMCL/SLAM fight over map -> odom. In
fact nav2_bringup's navigation_launch.py contains no AMCL at all (localization
is a separate launch file), so no conflict exists -- but a regression here would
be nasty to debug, so the test asserts the publisher set explicitly rather than
trusting the reading.

Mapping. Checked by motion: the map must GROW as the robot drives to new
vantage points, which can only happen if cells are coming from live scans. A
static snapshot would also be produced by a stale or hard-coded map, so growth
is the property worth testing. Known trees must also appear as occupied cells.

Run in simple_trees: the pine at (5, 0) is in lidar range from the origin, so
occupied cells appear without needing the heavier large_demo world.

    python3 test/tf_slam_test.py
"""

import sys

import rclpy

from nav_test_lib import await_simulation, bringup, log, print_diagnostics, ros2
from rs1_nav import NavObserver, init_ros

# A tree in simple_trees that is close enough to be seen from the start pose.
# Pine at (5, 0); its foliage is typically measured around 3.6 m ahead.
TREE_XY = (5.0, 0.0)
TREE_SEARCH_RADIUS = 1.5

# Nodes legitimately allowed to broadcast on /husky1/tf.
EXPECTED_TF_PUBLISHERS = ('robot_localization', 'slam_toolbox', 'robot_state_publisher')
# A node that must never appear: it would fight SLAM for map -> odom.
FORBIDDEN_TF_PUBLISHERS = ('amcl',)

EXPLORE_DISTANCE = 3.0   # m of driving used to reveal new ground
MIN_MAP_GROWTH = 1.20    # the map must gain at least 20% more observed cells


def tf_publisher_nodes() -> list:
    """Node names publishing on /husky1/tf, from the ROS graph itself."""
    code, out = ros2('topic info /husky1/tf --verbose', timeout=20.0)
    if code != 0:
        return []
    names = []
    in_publishers = False
    for line in out.splitlines():
        stripped = line.strip()
        if stripped.startswith('Publisher count:'):
            in_publishers = True
            continue
        if stripped.startswith('Subscription count:'):
            in_publishers = False
            continue
        if in_publishers and stripped.startswith('Node name:'):
            names.append(stripped.split(':', 1)[1].strip())
    return names


def tree_is_occupied(probe: NavObserver) -> bool:
    """True when the SLAM grid marks cells near the known tree as occupied."""
    grid = probe.map
    if grid is None:
        return False
    res = grid.info.resolution
    ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
    steps = int(TREE_SEARCH_RADIUS / res)
    cx = int((TREE_XY[0] - ox) / res)
    cy = int((TREE_XY[1] - oy) / res)
    for dy in range(-steps, steps + 1):
        for dx in range(-steps, steps + 1):
            ix, iy = cx + dx, cy + dy
            if not (0 <= ix < grid.info.width and 0 <= iy < grid.info.height):
                continue
            if grid.data[iy * grid.info.width + ix] >= 65:
                return True
    return False


def main() -> int:
    sup = bringup(
        world='simple_trees',
        nav2=True,           # nav2:=true also starts SLAM Toolbox
        rviz=False,
        gui=False,
        max_runtime=420.0,
        log_path='/tmp/tf_slam_test.log',
    )

    results = []

    with sup:
        ok, _stages = await_simulation(sup, require_nav2=True)
        if not ok:
            print_diagnostics(sup, topics=['/husky1/map', '/husky1/scan'])
            log('TF/SLAM TEST FAILED (stack did not start)')
            return 1

        init_ros()
        probe = NavObserver()
        try:
            # -- TF chain -----------------------------------------------------
            chain_ok = probe.wait_for(
                'TF husky1_map -> husky1_base_scan resolves',
                lambda: probe.tf_buffer.can_transform(
                    probe.map_frame, f'{probe.robot}_base_scan', rclpy.time.Time()),
                timeout=60.0)
            results.append(('full TF chain map -> base_scan', chain_ok, ''))

            # -- TF authority -------------------------------------------------
            publishers = tf_publisher_nodes()
            log(f'/husky1/tf publishers: {publishers}')
            unexpected = [n for n in publishers
                          if not any(e in n for e in EXPECTED_TF_PUBLISHERS)]
            forbidden = [n for n in publishers
                         if any(f in n for f in FORBIDDEN_TF_PUBLISHERS)]
            results.append(('only the expected nodes broadcast TF',
                            bool(publishers) and not unexpected and not forbidden,
                            f'unexpected={unexpected} forbidden={forbidden}'))

            # -- mapping is live ----------------------------------------------
            if not probe.wait_for('SLAM map received',
                                  lambda: probe.map is not None, timeout=60.0):
                return 1
            before = probe.map_known_cell_count()
            info = probe.map.info
            log(f'map before driving: {before} observed cells, '
                f'{info.width}x{info.height} at {info.resolution:.3f} m/cell')

            # Driving here is deliberate: it is the stimulus that reveals new
            # ground. This test covers mapping, not autonomy.
            probe.drive_for(0.4, 0.0, EXPLORE_DISTANCE / 0.4)
            probe.spin_for(4.0)

            target = int(before * MIN_MAP_GROWTH)
            grew = probe.wait_for(
                f'map grew past {target} observed cells after driving '
                f'{EXPLORE_DISTANCE:.0f} m',
                lambda: probe.map_known_cell_count() >= target,
                timeout=60.0,
                on_timeout_detail=lambda: f'{probe.map_known_cell_count()} cells')
            after = probe.map_known_cell_count()
            log(f'map after driving: {after} observed cells '
                f'({after / max(before, 1):.2f}x)')
            results.append(('map grows from live lidar as the robot moves', grew,
                            f'{before} -> {after} cells'))

            occupied = probe.wait_for(
                f'tree at {TREE_XY} appears as occupied cells',
                lambda: tree_is_occupied(probe), timeout=45.0)
            results.append(('world obstacles appear in the SLAM map', occupied, ''))
        finally:
            probe.destroy_node()
            rclpy.shutdown()

    log('=' * 68)
    for name, passed, detail in results:
        log(f'  {"PASS" if passed else "FAIL"}  {name}' + (f'  [{detail}]' if detail else ''))
    all_passed = bool(results) and all(p for _n, p, _d in results)
    log(f'TF/SLAM TEST {"PASSED" if all_passed else "FAILED"}')
    log('=' * 68)
    return 0 if all_passed else 1


if __name__ == '__main__':
    sys.exit(main())
