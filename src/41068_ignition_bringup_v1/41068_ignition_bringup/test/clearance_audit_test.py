#!/usr/bin/env python3
"""Measure path–obstacle clearance on the primary custom_world_1 mission.

Baselines and validates inflation/clearance tuning (Entry 017+):

    (-18, 3) → (18, 0) with live lidar costmaps

Reports:
  - goal reached / duration / distance travelled / replan count
  - first & final plan length
  - min / mean path clearance to lethal costmap cells (254)
  - max / mean cost under the path
  - min lidar range observed while navigating (proxy for live proximity)

    python3 test/clearance_audit_test.py
    python3 test/clearance_audit_test.py --label baseline
"""

from __future__ import annotations

import argparse
import math
import sys

import rclpy

from nav_test_lib import await_simulation, bringup, log, print_diagnostics
from rs1_nav import MissionRunner, NavObserver, init_ros
from rs1_nav.geometry import (
    clearance_to_points,
    costmap_obstacle_cells,
    path_cost_stats,
    path_obstacle_clearances,
)

START = (-18.0, 3.0, 0.0)
GOAL = (18.0, 0.0, 0.0)
WORLD = 'custom_world_1'
# Half-width of configured footprint [[±0.55, ±0.38], ...]
FOOTPRINT_HALF_WIDTH = 0.38


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--label', default='audit', help='Tag for log lines')
    parser.add_argument('--timeout', type=float, default=240.0)
    args = parser.parse_args(argv)

    sup = bringup(
        world=WORLD,
        nav2=True,
        rviz=False,
        gui=False,
        husky_x=START[0],
        husky_y=START[1],
        husky_yaw=START[2],
        max_runtime=args.timeout + 180.0,
        log_path=f'/tmp/clearance_audit_{args.label}.log',
    )

    with sup:
        ok, _ = await_simulation(sup, require_nav2=True)
        if not ok:
            print_diagnostics(sup, topics=['/husky1/scan', '/husky1/plan'])
            log(f'CLEARANCE AUDIT [{args.label}] FAILED (stack did not start)')
            return 1

        init_ros()
        observer = NavObserver()
        mission = MissionRunner(observer, logger=log)
        pose_clearances = []
        lidar_mins = []

        poses_near_gap = []

        def on_tick(_elapsed, _report):
            # Use the local costmap (10×10 m) for live pose clearance — scanning
            # the 120×120 m global grid every tick stalls the mission loop.
            pose = observer.robot_pose()
            if pose is not None:
                if 6.0 <= pose[0] <= 10.0:
                    poses_near_gap.append((pose[0], pose[1]))
                if observer.local_costmap is not None:
                    grid = observer.local_costmap
                    meta = grid.metadata
                    obstacles = costmap_obstacle_cells(
                        grid.data, meta.size_x, meta.size_y,
                        meta.origin.position.x, meta.origin.position.y,
                        meta.resolution, lethal_threshold=254,
                    )
                    d = clearance_to_points((pose[0], pose[1]), obstacles)
                    if d is not None:
                        pose_clearances.append(d)

            scan = observer.scan
            if scan is not None and scan.ranges:
                valid = [
                    r for r in scan.ranges
                    if math.isfinite(r) and scan.range_min < r < scan.range_max
                ]
                if valid:
                    lidar_mins.append(min(valid))

        try:
            if not mission.wait_until_ready(timeout=180.0):
                print_diagnostics(sup, topics=['/husky1/plan'])
                return 1

            report = mission.run(
                GOAL, timeout=args.timeout, on_tick=on_tick, tick_interval=1.0)

            # Prefer the first mission plan for clearance (final plans are often
            # truncated remaining paths in already-cleared space).
            mission_paths = observer.paths
            first_path = mission_paths[0] if mission_paths else None
            final_path = mission_paths[-1] if mission_paths else None
            analysis_path = first_path or final_path

            min_c = mean_c = max_c = None
            max_cost = mean_cost = None
            high_cost_samples = 0
            gap_y = None
            if analysis_path is not None and observer.global_costmap is not None:
                meta = observer.global_costmap.metadata
                obstacles = costmap_obstacle_cells(
                    observer.global_costmap.data,
                    meta.size_x, meta.size_y,
                    meta.origin.position.x, meta.origin.position.y,
                    meta.resolution, lethal_threshold=254,
                )
                min_c, mean_c, max_c = path_obstacle_clearances(
                    analysis_path.points, obstacles, sample_stride=2)
                max_cost, mean_cost, high_cost_samples = path_cost_stats(
                    analysis_path.points,
                    lambda x, y: observer.costmap_cost_at(x, y),
                    sample_stride=2,
                )
                # Crossing y at the forest wall line (x≈8) shows wall-skim vs
                # gap-centre routing (Gap A centre y=5.5, wall_5 top y≈2.66).
                crossings = [
                    y for x, y in analysis_path.points if 7.5 <= x <= 8.5
                ]
                if crossings:
                    gap_y = sum(crossings) / len(crossings)

            body_min = None
            if min_c is not None:
                body_min = max(0.0, min_c - FOOTPRINT_HALF_WIDTH)

            log('=== CLEARANCE AUDIT RESULTS ===')
            log(f'label: {args.label}')
            log(report.summary())
            if report.first_plan_length is not None:
                log(f'first_plan_length_m: {report.first_plan_length:.2f}')
            if report.final_plan_length is not None:
                log(f'final_plan_length_m: {report.final_plan_length:.2f}')
            if analysis_path is first_path and first_path is not None:
                log('clearance_measured_on: first_plan')
            elif analysis_path is not None:
                log('clearance_measured_on: final_plan')
            if min_c is not None:
                log(f'path_clearance_to_lethal_m: min={min_c:.3f} '
                    f'mean={mean_c:.3f} max={max_c:.3f}')
                log(f'approx_body_side_clearance_m: min={body_min:.3f} '
                    f'(centerline_min - footprint_half_width)')
            else:
                log('path_clearance_to_lethal_m: unavailable')
            if gap_y is not None:
                log(f'path_y_at_x8_m: {gap_y:.3f} '
                    f'(Gap A centre=5.5, wall_5 edge≈2.66)')
            if poses_near_gap:
                ys = [y for _, y in poses_near_gap]
                log(f'robot_y_near_x8_m: min={min(ys):.3f} mean={sum(ys)/len(ys):.3f} '
                    f'max={max(ys):.3f} n={len(ys)}')
            if max_cost is not None:
                log(f'path_cost: max={max_cost} mean={mean_cost:.1f} '
                    f'high_cost_samples(>200)={high_cost_samples}')
            if pose_clearances:
                log(f'pose_clearance_to_lethal_m: '
                    f'min={min(pose_clearances):.3f} '
                    f'mean={sum(pose_clearances)/len(pose_clearances):.3f} '
                    f'n={len(pose_clearances)}')
            if lidar_mins:
                log(f'lidar_min_range_m: min={min(lidar_mins):.3f} '
                    f'mean={sum(lidar_mins)/len(lidar_mins):.3f} '
                    f'n={len(lidar_mins)}')
            log('=== END CLEARANCE AUDIT ===')

            return 0 if report.reached else 1
        finally:
            observer.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
