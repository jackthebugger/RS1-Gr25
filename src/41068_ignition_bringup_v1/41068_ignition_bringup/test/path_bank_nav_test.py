#!/usr/bin/env python3
"""Integration tests: static costmap + path bank with live Nav2.

Scenarios (subset runnable headless):

  1. Clean start — bank builds, best route selected, goal reached
  2. Block active path — lidar→costmap→bank switch→continue
  8. Irrelevant obstacle — no switch

    python3 test/path_bank_nav_test.py
    python3 test/path_bank_nav_test.py --scenario clean
    python3 test/path_bank_nav_test.py --scenario block
    python3 test/path_bank_nav_test.py --scenario irrelevant
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from std_msgs.msg import String

from nav_test_lib import await_simulation, bringup, log, print_diagnostics
from rs1_nav import (
    GazeboWorld,
    MissionRunner,
    NavObserver,
    ObstacleSpec,
    init_ros,
    path_closest_approach,
)

START = (-18.0, 3.0, 0.0)
GOAL = (16.0, -0.1, 0.0)  # on-trail (Entry 019); (18,0) was off-trail
WORLD = 'bush_trail_world'


class PathBankWatcher:
    """Lightweight subscriber for path_bank status/state topics."""

    def __init__(self, observer: NavObserver):
        self.node = observer
        self.state = ''
        self.status = ''
        self.node.create_subscription(String, 'path_bank/state', self._on_state, 10)
        self.node.create_subscription(String, 'path_bank/status', self._on_status, 10)

    def _on_state(self, msg: String) -> None:
        self.state = msg.data

    def _on_status(self, msg: String) -> None:
        self.status = msg.data

    def wait_for_bank(self, timeout: float = 60.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.node.spin_for(0.2)
            if 'score=' in self.status and 'Path ' in self.status:
                return True
        return False

    def candidate_count(self) -> int:
        return sum(1 for line in self.status.splitlines() if line.startswith('Path '))

    def switch_count(self) -> int:
        for line in self.status.splitlines():
            if line.startswith('switches='):
                try:
                    return int(line.split('=', 1)[1])
                except ValueError:
                    return 0
        return 0


def _spawn_box(world: GazeboWorld, name: str, x: float, y: float,
               yaw: float = 0.0) -> bool:
    # Thin wall across the trail (same family as PathBlocker). A 1.5×3.0 box
    # sealed both bank corridors and left the Husky stuck at the choke.
    from rs1_nav.gazebo_world import barrier_across_heading
    spec = barrier_across_heading(
        name=name,
        at=(x, y),
        heading=yaw,
        width=2.2,
        thickness=0.45,
        height=1.6,
    )
    return world.spawn_obstacle(spec)


def run_clean(observer, mission, bank) -> bool:
    log('=== TEST 1: Clean start ===')
    report = mission.run(GOAL, timeout=240.0)
    log(report.summary())
    bank.node.spin_for(0.5)
    n = bank.candidate_count()
    log(f'path_bank candidates seen: {n}; state={bank.state}')
    ok = report.reached and n >= 1
    log('PASS' if ok else 'FAIL')
    return ok


def _exclusive_block_xy(observer, timeout: float = 20.0):
    """Prefer a point on path_bank/candidate_1 far from candidate_2."""
    from nav_msgs.msg import Path as RosPath
    from rclpy.qos import (
        DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
    )

    latched = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )
    paths = {1: None, 2: None}

    def _make(i):
        def _cb(msg: RosPath):
            pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            if pts:
                paths[i] = pts
        return _cb

    # Topics are candidate_1 / candidate_2 (path_id), not 0-based.
    subs = [
        observer.create_subscription(
            RosPath, f'path_bank/candidate_{i}', _make(i), latched)
        for i in (1, 2)
    ]
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and (paths[1] is None or paths[2] is None):
        observer.spin_for(0.2)
    for s in subs:
        observer.destroy_subscription(s)
    if paths[1] and paths[2]:
        # Prefer max-separation exclusive point with y≥-3 (eastbound corridor).
        # Deep southern exclusives (y≈-11) force a long Path-2 handoff.
        best = None  # (sep, pt)
        for pt in paths[1][::12]:
            if pt[1] < -3.0:
                continue
            sep = min(
                math.hypot(pt[0] - q[0], pt[1] - q[1]) for q in paths[2][::8]
            )
            if sep < 4.0:
                continue
            if best is None or sep > best[0]:
                best = (sep, pt)
        if best is not None:
            return best[1], best[0]
        # Fallback: global max-sep (may be deep Path A).
        best = None
        for pt in paths[1][::12]:
            sep = min(
                math.hypot(pt[0] - q[0], pt[1] - q[1]) for q in paths[2][::8]
            )
            if best is None or sep > best[0]:
                best = (sep, pt)
        if best is not None and best[0] > 2.0:
            return best[1], best[0]
    return None, 0.0


def run_block(observer, mission, bank, world) -> bool:
    log('=== TEST 2: Block active path ===')
    # Start mission asynchronously by sending goal then inserting obstacle.
    if not mission.wait_until_ready(timeout=30.0):
        return False
    observer.send_goal(GOAL[0], GOAL[1], GOAL[2])
    # FollowPath mode has no /plan chatter — wait for path_bank candidates.
    if not bank.wait_for_bank(60.0):
        log('FAIL  path_bank status never published candidates')
        return False
    switches_before = bank.switch_count()

    # Block a Path-1-exclusive corridor point when available so Path 2 remains open.
    exclusive, sep = _exclusive_block_xy(observer)
    plan_pts = None
    if observer.paths:
        plan_pts = observer.paths[-1].points
    if exclusive is not None:
        mid = exclusive
        log(f'Inserting barrier on Path-1-exclusive point ({mid[0]:.1f}, {mid[1]:.1f}) sep={sep:.1f}m')
    elif plan_pts is not None and len(plan_pts) >= 3:
        mid = plan_pts[len(plan_pts) // 2]
        log(f'Inserting barrier near plan midpoint ({mid[0]:.1f}, {mid[1]:.1f})')
    else:
        # FollowPath: no /plan — fall back to latched active_path midpoint.
        from nav_msgs.msg import Path as RosPath
        from rclpy.qos import (
            DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
        )
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        held = {'pts': None}

        def _active_cb(msg: RosPath):
            pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            if pts:
                held['pts'] = pts

        sub = observer.create_subscription(
            RosPath, 'path_bank/active_path', _active_cb, latched)
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and held['pts'] is None:
            observer.spin_for(0.2)
        observer.destroy_subscription(sub)
        if held['pts'] is None or len(held['pts']) < 3:
            log('FAIL  no Path-1 exclusive point and no active_path for barrier')
            return False
        mid = held['pts'][len(held['pts']) // 2]
        plan_pts = held['pts']
        log(f'Inserting barrier on active_path midpoint ({mid[0]:.1f}, {mid[1]:.1f})')
    # Orient barrier across local path tangent when we have a polyline.
    yaw = 0.0
    pts_for_yaw = plan_pts
    if pts_for_yaw is None:
        # Approximate eastbound trail heading at exclusive point.
        yaw = 0.0
    else:
        idx = max(1, min(len(pts_for_yaw) - 2, len(pts_for_yaw) // 2))
        a, b = pts_for_yaw[idx - 1], pts_for_yaw[idx + 1]
        yaw = math.atan2(b[1] - a[1], b[0] - a[0])
    if not _spawn_box(world, 'path_bank_block_1', mid[0], mid[1], yaw=yaw):
        log('FAIL  spawn barrier')
        return False

    # Wait for switch/replan, then keep waiting for goal — do not treat a
    # transient NavigateToPose abort during bank resend as mission failure.
    report_deadline = time.monotonic() + 360.0
    switched = False
    while time.monotonic() < report_deadline:
        observer.spin_for(0.3)
        if bank.switch_count() > switches_before:
            switched = True
            log(f'Detected path-bank switch (switches={bank.switch_count()})')
            break
        if len(observer.paths) >= 2:
            div = path_closest_approach(observer.paths[-1].points, mid)
            if div is not None and div > 1.5:
                log(f'Nav2 plan diverged from barrier (clear={div:.2f}m)')
                switched = True
                break
        pose = observer.robot_pose()
        if pose and math.hypot(pose[0] - GOAL[0], pose[1] - GOAL[1]) < 0.5:
            break

    # MissionRunner-style wait: pose only. Observer's own NavigateToPose may
    # be preempted by path_bank_manager; ignore that client's terminal status.
    last_pose = observer.robot_pose()
    idle_since = time.monotonic()
    while time.monotonic() < report_deadline:
        observer.spin_for(0.4)
        pose = observer.robot_pose()
        if pose and math.hypot(pose[0] - GOAL[0], pose[1] - GOAL[1]) < 0.6:
            log('Goal reached after blockage')
            ok = switched
            log('PASS' if ok else 'FAIL (reached but no switch/replan evidence)')
            return ok
        if pose is not None and last_pose is not None:
            moved = math.hypot(pose[0] - last_pose[0], pose[1] - last_pose[1])
            if moved > 0.12:
                idle_since = time.monotonic()
                last_pose = pose
        # Entry 049: far-gap NavFn bridge + join rotation can exceed 150 s
        # before progress resumes; do not abort while bank still FOLLOWING.
        idle_limit = 240.0 if switched else 150.0
        if switched and (time.monotonic() - idle_since) > idle_limit:
            log(f'Idle >{idle_limit:.0f}s after switch/replan; stopping wait')
            break

    pose = observer.robot_pose()
    dist = None if pose is None else math.hypot(pose[0] - GOAL[0], pose[1] - GOAL[1])
    log(f'End dist={dist}; switches={bank.switch_count()}; state={bank.state}')
    ok = switched and dist is not None and dist < 1.0
    log('PASS' if ok else 'FAIL')
    return ok


def run_irrelevant(observer, mission, bank, world) -> bool:
    log('=== TEST 8: Irrelevant obstacle ===')
    observer.send_goal(GOAL[0], GOAL[1], GOAL[2])
    deadline = time.monotonic() + 45.0
    while time.monotonic() < deadline:
        observer.spin_for(0.2)
        if observer.paths:
            break
    bank.wait_for_bank(30.0)
    switches_before = bank.switch_count()
    # Far off the expected eastbound corridor
    _spawn_box(world, 'path_bank_irrelevant', 0.0, -20.0)
    time.sleep(8.0)
    observer.spin_for(10.0)
    switches_after = bank.switch_count()
    ok = switches_after == switches_before
    log(f'switches before={switches_before} after={switches_after}')
    log('PASS' if ok else 'FAIL')
    # Cancel so we do not leave a long mission running
    observer.cancel_goal()
    return ok


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--scenario',
        choices=('all', 'clean', 'block', 'irrelevant'),
        default='all',
    )
    args = parser.parse_args()

    sup = bringup(
        world=WORLD,
        nav2=True,
        rviz=False,
        gui=False,
        husky_x=START[0],
        husky_y=START[1],
        husky_yaw=START[2],
        max_runtime=900.0,
        log_path='/tmp/path_bank_nav_test.log',
        extra={
            'husky_z': '0.50',
            'use_prior_map': 'true',
            'prior_map_file': 'bush_trail_world.yaml',
            'path_bank': 'true',
            # Block scenario needs forced resend onto next banked route.
            'path_bank_switch_resend': 'true' if args.scenario in ('all', 'block') else 'false',
            'fire_avoidance': 'false',
            # Harness speed (plan §I); production YAML stays 0.5 / ±0.6.
            'test_speed': 'true',
        },
    )

    results = {}
    with sup:
        ok, _ = await_simulation(sup, require_nav2=True)
        if not ok:
            print_diagnostics(sup, topics=['/husky1/prior_map', '/husky1/plan'])
            log('FAIL stack start')
            return 1

        init_ros()
        observer = NavObserver()
        mission = MissionRunner(observer, logger=log)
        bank = PathBankWatcher(observer)
        if not mission.wait_until_ready(timeout=180.0):
            print_diagnostics(sup, topics=['/husky1/prior_map', '/husky1/global_costmap/costmap'])
            return 1

        log('Nav2 ready; proceeding')

        world = GazeboWorld(WORLD, logger=lambda m: log(f'  {m}'))
        if not world.wait_until_available(max_wait=30.0):
            log('FAIL Gazebo services')
            return 1

        scenarios = []
        if args.scenario in ('all', 'clean'):
            scenarios.append(('clean', lambda: run_clean(observer, mission, bank)))
        if args.scenario in ('all', 'block'):
            scenarios.append(('block', lambda: run_block(observer, mission, bank, world)))
        if args.scenario in ('all', 'irrelevant'):
            scenarios.append(('irrelevant', lambda: run_irrelevant(observer, mission, bank, world)))

        for name, fn in scenarios:
            try:
                results[name] = fn()
            except Exception as exc:  # noqa: BLE001
                log(f'EXCEPTION in {name}: {exc}')
                results[name] = False
            # Entry 035: StopMission between scenarios so ARRIVED/NAVIGATING
            # cannot reject the next StartMission (service race after clean).
            try:
                observer.cancel_goal()
            except Exception:  # noqa: BLE001
                pass
            observer.spin_for(0.5)

    log('=== SUMMARY ===')
    for name, passed in results.items():
        log(f'  {name}: {"PASS" if passed else "FAIL"}')
    return 0 if results and all(results.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
