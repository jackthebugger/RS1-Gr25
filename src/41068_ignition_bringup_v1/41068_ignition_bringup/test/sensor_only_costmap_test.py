#!/usr/bin/env python3
"""Integration regression: sensor-only keepout memory (Entries 021–024).

Primary scenario T3 — Path A blocked → switch to B → remove obstacle off-FOV
→ Path A must remain lethal in keepout/costmap and bank stays BLOCKED; no
corridor return to A.

T5 — off-FOV remove → open-loop approach (test transport) → real lidar clear →
Path A AVAILABLE without auto-preempt. Also asserts T14-clear corridor free.

T11 — block A → switch B → block B → RETURNING_TO_START toward home (~spawn).

    python3 test/sensor_only_costmap_test.py
    python3 test/sensor_only_costmap_test.py --scenario priority
    python3 test/sensor_only_costmap_test.py --scenario t5
    python3 test/sensor_only_costmap_test.py --scenario t11
    python3 test/sensor_only_costmap_test.py --scenario extended
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import List, Optional, Tuple

from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid, Path as RosPath
from std_msgs.msg import String
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from nav_test_lib import await_simulation, bringup, log, print_diagnostics
from rs1_nav import (
    GazeboWorld,
    NavObserver,
    init_ros,
    lethal_region_report,
    point_ahead_on_path,
)
from rs1_nav.gazebo_world import barrier_across_heading

START = (-18.0, 3.0, 0.0)
GOAL = (16.0, -0.1, 0.0)
WORLD = 'bush_trail_world'
LETHAL = 254

LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


class MemoryWatcher:
    def __init__(self, observer: NavObserver):
        self.node = observer
        self.mask: Optional[OccupancyGrid] = None
        self.status = ''
        self.bank_status = ''
        self.bank_state = ''
        self.costmap: Optional[Costmap] = None
        self.plan: Optional[RosPath] = None
        self.node.create_subscription(
            OccupancyGrid, 'obstacle_memory/mask', self._on_mask, LATCHED)
        self.node.create_subscription(
            String, 'obstacle_memory/status', self._on_mem_status, 10)
        self.node.create_subscription(
            String, 'path_bank/status', self._on_bank_status, 10)
        self.node.create_subscription(
            String, 'path_bank/state', self._on_bank_state, 10)
        self.node.create_subscription(
            Costmap, 'global_costmap/costmap_raw', self._on_costmap, 2)
        self.node.create_subscription(RosPath, 'plan', self._on_plan, 5)

    def _on_mask(self, msg: OccupancyGrid) -> None:
        self.mask = msg

    def _on_mem_status(self, msg: String) -> None:
        self.status = msg.data

    def _on_bank_status(self, msg: String) -> None:
        self.bank_status = msg.data

    def _on_bank_state(self, msg: String) -> None:
        self.bank_state = msg.data

    def _on_costmap(self, msg: Costmap) -> None:
        self.costmap = msg

    def _on_plan(self, msg: RosPath) -> None:
        self.plan = msg

    def spin(self, dt: float = 0.2) -> None:
        self.node.spin_for(dt)

    def wait_mask(self, timeout: float = 60.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.spin(0.2)
            if self.mask is not None:
                return True
        return False

    def wait_bank(self, timeout: float = 60.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.spin(0.2)
            if 'score=' in self.bank_status and 'Path ' in self.bank_status:
                return True
        return False

    def mask_occupied_count(self) -> int:
        if self.mask is None:
            return 0
        return sum(1 for v in self.mask.data if int(v) >= 50)

    def cost_at(self, x: float, y: float) -> Optional[int]:
        cm = self.costmap
        if cm is None:
            return None
        meta = cm.metadata
        w, h = int(meta.size_x), int(meta.size_y)
        res = float(meta.resolution)
        ox = float(meta.origin.position.x)
        oy = float(meta.origin.position.y)
        ix = int((x - ox) / res)
        iy = int((y - oy) / res)
        if not (0 <= ix < w and 0 <= iy < h):
            return None
        return int(cm.data[iy * w + ix])

    def costmap_lethal_report(
        self,
        x: float,
        y: float,
        *,
        search_radius_m: float = 1.5,
        accept_radius_m: float = 0.60,
    ):
        """Structured lethal neighbourhood report for costmap_raw."""
        cm = self.costmap
        if cm is None:
            return None
        meta = cm.metadata
        return lethal_region_report(
            cm.data,
            int(meta.size_x),
            int(meta.size_y),
            float(meta.origin.position.x),
            float(meta.origin.position.y),
            float(meta.resolution),
            (x, y),
            frame_id=getattr(meta, 'layer', '') or 'global_costmap',
            search_radius_m=search_radius_m,
            accept_radius_m=accept_radius_m,
        )

    def mask_occupancy_report(
        self,
        x: float,
        y: float,
        *,
        search_radius_m: float = 1.5,
        accept_radius_m: float = 0.60,
    ):
        """Treat mask occupied (≥50) like lethal for spatial assertions."""
        m = self.mask
        if m is None:
            return None
        info = m.info
        # Map OccupancyGrid 100 → synthetic 254 so lethal_region_report applies.
        data = [254 if int(v) >= 50 else (255 if int(v) < 0 else 0) for v in m.data]
        return lethal_region_report(
            data,
            int(info.width),
            int(info.height),
            float(info.origin.position.x),
            float(info.origin.position.y),
            float(info.resolution),
            (x, y),
            frame_id=m.header.frame_id,
            search_radius_m=search_radius_m,
            accept_radius_m=accept_radius_m,
        )

    def mask_occupied_near(self, x: float, y: float, radius_m: float = 0.6) -> bool:
        m = self.mask
        if m is None:
            return False
        info = m.info
        w, h = int(info.width), int(info.height)
        res = float(info.resolution)
        ox = float(info.origin.position.x)
        oy = float(info.origin.position.y)
        r_cells = max(1, int(radius_m / res))
        cx = int((x - ox) / res)
        cy = int((y - oy) / res)
        for dx in range(-r_cells, r_cells + 1):
            for dy in range(-r_cells, r_cells + 1):
                ix, iy = cx + dx, cy + dy
                if 0 <= ix < w and 0 <= iy < h and int(m.data[iy * w + ix]) >= 50:
                    return True
        return False

    def lethal_near(self, x: float, y: float, radius_m: float = 0.6) -> bool:
        cm = self.costmap
        if cm is None:
            return False
        meta = cm.metadata
        w, h = int(meta.size_x), int(meta.size_y)
        res = float(meta.resolution)
        ox = float(meta.origin.position.x)
        oy = float(meta.origin.position.y)
        r_cells = max(1, int(radius_m / res))
        cx = int((x - ox) / res)
        cy = int((y - oy) / res)
        for dx in range(-r_cells, r_cells + 1):
            for dy in range(-r_cells, r_cells + 1):
                ix, iy = cx + dx, cy + dy
                if 0 <= ix < w and 0 <= iy < h:
                    c = int(cm.data[iy * w + ix])
                    if c >= LETHAL and c != 255:
                        return True
        return False

    def nearest_mask_occupied_xy(
        self, x: float, y: float, radius_m: float = 1.5
    ) -> Optional[Tuple[float, float]]:
        """World XY of nearest mask occupied cell near (x,y), for T14 queries."""
        m = self.mask
        if m is None:
            return None
        info = m.info
        w, h = int(info.width), int(info.height)
        res = float(info.resolution)
        ox = float(info.origin.position.x)
        oy = float(info.origin.position.y)
        r_cells = max(1, int(radius_m / res))
        cx = int((x - ox) / res)
        cy = int((y - oy) / res)
        best = None
        for dx in range(-r_cells, r_cells + 1):
            for dy in range(-r_cells, r_cells + 1):
                ix, iy = cx + dx, cy + dy
                if 0 <= ix < w and 0 <= iy < h and int(m.data[iy * w + ix]) >= 50:
                    wx = ox + (ix + 0.5) * res
                    wy = oy + (iy + 0.5) * res
                    d = math.hypot(wx - x, wy - y)
                    if best is None or d < best[0]:
                        best = (d, wx, wy)
        if best is None:
            return None
        return best[1], best[2]

    def wait_lethal_near(
        self, x: float, y: float, timeout: float = 20.0, radius_m: float = 0.8
    ) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.spin(0.25)
            if self.lethal_near(x, y, radius_m=radius_m):
                return True
        return False

    def bank_active_id(self) -> Optional[int]:
        for line in self.bank_status.splitlines():
            if line.startswith('active_id='):
                try:
                    v = line.split('=', 1)[1].strip()
                    return int(v) if v not in ('', 'None') else None
                except ValueError:
                    return None
        return None

    def bank_path_state(self, path_id: int) -> str:
        for line in self.bank_status.splitlines():
            if line.startswith(f'Path {path_id}:'):
                if 'state=blocked' in line:
                    return 'blocked'
                if 'state=active' in line:
                    return 'active'
                if 'state=available' in line:
                    return 'available'
        return ''

    def switch_count(self) -> int:
        for line in self.bank_status.splitlines():
            if line.startswith('switches='):
                try:
                    return int(line.split('=', 1)[1])
                except ValueError:
                    return 0
        return 0

    def plan_near_xy(self, x: float, y: float, radius: float = 1.5) -> bool:
        if self.plan is None or not self.plan.poses:
            return False
        for p in self.plan.poses:
            if math.hypot(p.pose.position.x - x, p.pose.position.y - y) < radius:
                return True
        return False


def _exclusive_block_xy(observer: NavObserver, timeout: float = 25.0):
    paths = {1: None, 2: None}

    def _make(i):
        def _cb(msg: RosPath):
            pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            if pts:
                paths[i] = pts
        return _cb

    subs = [
        observer.create_subscription(RosPath, f'path_bank/candidate_{i}', _make(i), 5)
        for i in (1, 2)
    ]
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and (paths[1] is None or paths[2] is None):
        observer.spin_for(0.2)
    for s in subs:
        observer.destroy_subscription(s)
    if paths[1] and paths[2]:
        best = None
        for pt in paths[1][::10]:
            d = min(math.hypot(pt[0] - q[0], pt[1] - q[1]) for q in paths[2][::8])
            if best is None or d > best[0]:
                best = (d, pt)
        if best is not None and best[0] > 1.5:
            return best[1], best[0], paths[1]
    if paths[1]:
        mid = paths[1][len(paths[1]) // 2]
        return mid, 0.0, paths[1]
    return None, 0.0, None


def run_t1_clean(watcher: MemoryWatcher) -> bool:
    log('=== T1: Clean startup ===')
    if not watcher.wait_mask(90.0):
        log('FAIL  no obstacle_memory/mask')
        return False
    t0 = time.monotonic()
    while time.monotonic() - t0 < 8.0:
        watcher.spin(0.3)
    n = watcher.mask_occupied_count()
    log(f'  mask occupied cells={n}; status={watcher.status}')
    if n > 8000:
        log(f'FAIL  mask unexpectedly dense ({n})')
        return False
    log('PASS  T1 clean startup')
    return True


def _barrier_xy_ahead_on_path(
    path_pts: List[Tuple[float, float]],
    pose: Tuple[float, float, float],
    *,
    prefer_xy: Optional[Tuple[float, float]] = None,
    ahead_m: float = 6.0,
    min_ahead_m: float = 3.5,
    max_ahead_m: float = 9.0,
) -> Tuple[float, float]:
    """Place barrier on Path-1 ahead of the robot so lidar can mark it.

    With test_speed the robot can race past a far exclusive midpoint before
    memory marks (mask_near=False). Prefer a standoff ~ahead_m along path1.
    """
    if not path_pts:
        if prefer_xy is not None:
            return prefer_xy
        return (pose[0] + ahead_m * math.cos(pose[2]),
                pose[1] + ahead_m * math.sin(pose[2]))

    # Closest path index to robot.
    ci = min(
        range(len(path_pts)),
        key=lambda i: math.hypot(path_pts[i][0] - pose[0], path_pts[i][1] - pose[1]),
    )
    # Walk forward until cumulative arc length reaches ahead_m.
    acc = 0.0
    bi = ci
    for i in range(ci, len(path_pts) - 1):
        step = math.hypot(
            path_pts[i + 1][0] - path_pts[i][0],
            path_pts[i + 1][1] - path_pts[i][1],
        )
        acc += step
        bi = i + 1
        if acc >= ahead_m:
            break
    bx, by = path_pts[bi]
    dist = math.hypot(bx - pose[0], by - pose[1])
    # If prefer exclusive is already in the good window, use it.
    if prefer_xy is not None:
        pd = math.hypot(prefer_xy[0] - pose[0], prefer_xy[1] - pose[1])
        if min_ahead_m <= pd <= max_ahead_m:
            return prefer_xy
    if dist < min_ahead_m and bi + 1 < len(path_pts):
        return path_pts[min(len(path_pts) - 1, bi + max(1, int(2.0 / 0.05)))]
    return (bx, by)


def run_t2_block(
    watcher: MemoryWatcher,
    observer: NavObserver,
    world: GazeboWorld,
) -> Optional[Tuple[float, float]]:
    log('=== T2: Block A, switch B ===')
    # Wait until Nav2 action server is responsive (lifecycle settle).
    ready_deadline = time.monotonic() + 60.0
    while time.monotonic() < ready_deadline:
        watcher.spin(0.2)
        if observer.nav2_ready(timeout=1.0):
            break
    else:
        log('FAIL  Nav2 navigate_to_pose not ready')
        return None
    if not observer.send_goal(GOAL[0], GOAL[1], GOAL[2]):
        # One retry after brief settle — intermittent action-server stalls.
        watcher.spin(3.0)
        if not observer.send_goal(GOAL[0], GOAL[1], GOAL[2]):
            log('FAIL  NavigateToPose rejected/timeout')
            return None
    if not watcher.wait_bank(90.0):
        log('FAIL  path bank never published candidates')
        return None
    switches_before = watcher.switch_count()

    # Give bank candidates a moment to publish under test_speed.
    exclusive, sep, path1 = _exclusive_block_xy(observer, timeout=40.0)
    if exclusive is None:
        plan = observer.latest_path()
        if plan is None or len(plan.points) < 4:
            log('FAIL  no exclusive point and no plan')
            return None
        exclusive = plan.points[len(plan.points) // 2]
        path1 = plan.points
        log(f'  Using plan midpoint ({exclusive[0]:.1f},{exclusive[1]:.1f})')
    else:
        log(f'  Path-1 exclusive ({exclusive[0]:.1f},{exclusive[1]:.1f}) sep={sep:.1f}m')

    # Wait briefly for a pose, then place barrier ahead of the robot on Path 1
    # so lidar mark range (~12 m) is reached while still on Path A.
    pose = None
    pose_deadline = time.monotonic() + 20.0
    while time.monotonic() < pose_deadline:
        watcher.spin(0.2)
        pose = observer.robot_pose()
        if pose is not None:
            break
    if pose is None:
        log('FAIL  no robot pose before barrier spawn')
        return None

    pts = path1 or [exclusive]
    block_xy = _barrier_xy_ahead_on_path(
        pts, pose, prefer_xy=exclusive, ahead_m=6.0)
    log(f'  Barrier target ({block_xy[0]:.2f},{block_xy[1]:.2f}) '
        f'pose=({pose[0]:.2f},{pose[1]:.2f}) '
        f'dist={math.hypot(block_xy[0] - pose[0], block_xy[1] - pose[1]):.2f}m')

    # Orient barrier across local tangent on the Path-1 corridor.
    mid_i = min(
        range(len(pts)),
        key=lambda i: math.hypot(
            pts[i][0] - block_xy[0], pts[i][1] - block_xy[1]))
    i0 = max(0, mid_i - 2)
    i1 = min(len(pts) - 1, mid_i + 2)
    yaw = math.atan2(pts[i1][1] - pts[i0][1], pts[i1][0] - pts[i0][0])
    spec = barrier_across_heading(
        name='sensor_only_block_a',
        at=(block_xy[0], block_xy[1]),
        heading=yaw,
        width=2.2,
        thickness=0.45,
        height=1.6,
    )
    if not world.spawn_obstacle(spec):
        log('FAIL  spawn barrier')
        return None

    deadline = time.monotonic() + 150.0
    switched = False
    dist_at_switch = None
    saw_mask = False
    while time.monotonic() < deadline:
        watcher.spin(0.3)
        pose = observer.robot_pose()
        if pose is not None:
            dist_at_switch = math.hypot(
                pose[0] - block_xy[0], pose[1] - block_xy[1])
        if watcher.mask_occupied_near(*block_xy, radius_m=1.5):
            saw_mask = True
        # Prefer an explicit bank switch onto Path 2+.
        if watcher.switch_count() > switches_before and watcher.bank_active_id() not in (None, 1):
            switched = True
            break
        # Path 1 blocked and another candidate active (active_id != 1).
        if (
            watcher.bank_path_state(1) == 'blocked'
            and watcher.bank_active_id() not in (None, 1)
        ):
            switched = True
            break
        # Path 1 blocked with no active yet — keep waiting for Path 2 select.
        if (
            watcher.bank_path_state(1) == 'blocked'
            and watcher.bank_active_id() is None
            and watcher.switch_count() > switches_before
        ):
            # Switch counted but active not published yet.
            continue

    cost = watcher.cost_at(*block_xy)
    mask = watcher.mask_occupied_near(*block_xy)
    log(f'  switches={watcher.switch_count()} active={watcher.bank_active_id()} '
        f'path1={watcher.bank_path_state(1)} cost={cost} mask_near={mask} '
        f'saw_mask={saw_mask} dist_at_switch={dist_at_switch}')
    if not switched:
        # Last-chance: Path1 blocked is necessary but not sufficient without
        # an alternate active route for the off-FOV T3 sequence.
        if (
            watcher.bank_path_state(1) == 'blocked'
            and watcher.bank_active_id() not in (None, 1)
        ):
            switched = True
        else:
            log('FAIL  T2 no bank switch after barrier '
                f'(path1={watcher.bank_path_state(1)} '
                f'active={watcher.bank_active_id()} saw_mask={saw_mask})')
            return None
    # Entry 023: after switch, require continued progress on Path B so a
    # post-reroute freeze (RPP zero / bank re-block) fails the regression.
    pose_at_switch = observer.robot_pose()
    if pose_at_switch is None:
        log('FAIL  T2 no pose at switch for progress check')
        return None
    progress_deadline = time.monotonic() + 45.0
    progressed = False
    best_delta = 0.0
    while time.monotonic() < progress_deadline:
        watcher.spin(0.3)
        pose = observer.robot_pose()
        if pose is None:
            continue
        best_delta = max(
            best_delta,
            math.hypot(pose[0] - pose_at_switch[0], pose[1] - pose_at_switch[1]),
        )
        if best_delta >= 1.0:
            progressed = True
            break
        # If bank exhausted immediately after switch, fail early.
        if (
            watcher.bank_state == 'NO_PATH_AVAILABLE'
            and watcher.bank_active_id() is None
        ):
            log(f'FAIL  T2 bank exhausted after switch (delta={best_delta:.2f}m)')
            return None
    log(f'  post-switch progress delta={best_delta:.2f}m '
        f'active={watcher.bank_active_id()} state={watcher.bank_state}')
    if not progressed:
        log(f'FAIL  T2 freeze after switch (moved only {best_delta:.2f}m in 45s)')
        return None
    log('PASS  T2 block A → switch B (+ post-switch progress)')
    return (block_xy[0], block_xy[1])


def run_t3_primary(
    watcher: MemoryWatcher,
    observer: NavObserver,
    world: GazeboWorld,
    barrier_xy: Tuple[float, float],
) -> bool:
    log('=== T3: PRIMARY — remove A off-FOV ===')
    bx, by = barrier_xy

    log('  Waiting until barrier is off-FOV (dist > 12 m or 100 s)...')
    deadline = time.monotonic() + 100.0
    far = False
    while time.monotonic() < deadline:
        watcher.spin(0.3)
        pose = observer.robot_pose()
        if pose is not None:
            dist = math.hypot(pose[0] - bx, pose[1] - by)
            if dist > 12.0:
                far = True
                log(f'  Robot at ({pose[0]:.1f},{pose[1]:.1f}) dist={dist:.1f} m')
                break

    if not far:
        log('  INFO  off-FOV by pose not confirmed; removing anyway')

    cost_before = watcher.cost_at(bx, by)
    mask_before = watcher.mask_occupied_near(bx, by)
    path1_before = watcher.bank_path_state(1)
    active_before = watcher.bank_active_id()
    log(f'  before remove: cost={cost_before} mask={mask_before} '
        f'path1={path1_before} active={active_before} mem={watcher.status}')

    if not world.remove_model('sensor_only_block_a'):
        log('FAIL  could not remove barrier from Gazebo')
        return False
    log('  Barrier removed from Gazebo')

    t0 = time.monotonic()
    returned_to_a = False
    keepout_retained = False
    while time.monotonic() - t0 < 30.0:
        watcher.spin(0.3)
        if watcher.lethal_near(bx, by) or watcher.mask_occupied_near(bx, by):
            keepout_retained = True
        if watcher.bank_active_id() == 1:
            returned_to_a = True
        if watcher.plan_near_xy(bx, by, radius=1.2) and watcher.bank_path_state(1) != 'blocked':
            # Plan through A corridor after unblock would indicate regression.
            if watcher.bank_active_id() in (None, 1):
                returned_to_a = True

    cost_after = watcher.cost_at(bx, by)
    mask_after = watcher.mask_occupied_near(bx, by)
    path1_after = watcher.bank_path_state(1)
    log(f'  after remove: cost={cost_after} mask={mask_after} '
        f'path1={path1_after} active={watcher.bank_active_id()} '
        f'keepout_retained={keepout_retained} returned_to_a={returned_to_a}')

    ok = True
    # Prefer end-of-window evidence: mask or costmap still occupied near A.
    end_retained = bool(mask_after) or watcher.lethal_near(bx, by, radius_m=1.5)
    if not end_retained and not keepout_retained:
        log('FAIL  T3 neither keepout mask nor costmap lethal at barrier after remove')
        ok = False
    elif not end_retained:
        log('FAIL  T3 keepout was transient during wait but gone at end '
            '(likely long-range free-ray clear)')
        ok = False
    if path1_after != 'blocked' and watcher.switch_count() >= 1:
        log('FAIL  T3 Path 1 not BLOCKED after off-FOV remove')
        ok = False
    if returned_to_a:
        log('FAIL  T3 returned to Path A after unobserved remove')
        ok = False

    if ok:
        log('PASS  T3 primary off-FOV remove retains Path A memory')
    return ok


def run_t14_costmap_raw(
    watcher: MemoryWatcher,
    barrier_xy: Tuple[float, float],
) -> bool:
    """T14-block: remembered keepout must produce lethal costmap_raw coverage.

    Uses lethal_region_report (map meta + neighbourhood), not a single centre
    cell — neighbour marking / discretisation can leave the geometric centre
    soft (0) while an adjacent keepout cell is 254. Do not widen accept radius.
    """
    log('=== T14-block: costmap_raw lethal near remembered cell ===')
    bx, by = barrier_xy
    deadline = time.monotonic() + 25.0
    last_detail = ''
    while time.monotonic() < deadline:
        watcher.spin(0.3)
        mask_rep = watcher.mask_occupancy_report(bx, by)
        cost_rep = watcher.costmap_lethal_report(bx, by)
        if mask_rep is None or cost_rep is None:
            continue
        # Diagnose centre=0 vs neighbour=254 without relaxing acceptance.
        last_detail = (
            f'mask[{mask_rep.detail}] costmap[{cost_rep.detail}] '
            f'centre_cost={cost_rep.centre_cost} '
            f'nearest_lethal_m={cost_rep.nearest_lethal_m} '
            f'nearest_xy={cost_rep.nearest_lethal_xy} '
            f'origin=({cost_rep.origin_x:.2f},{cost_rep.origin_y:.2f}) '
            f'res={cost_rep.resolution:.3f}'
        )
        if mask_rep.ok and cost_rep.ok:
            if cost_rep.centre_cost == 0 and cost_rep.nearest_lethal_cost == 254:
                log('  INFO  T14 centre soft/0 while neighbour lethal=254 '
                    '(discretisation / mark_neighbour); nearest-mask accept OK')
            log(f'  PASS  T14-block {last_detail}')
            return True
    log(f'FAIL  T14-block {last_detail or "no costmap/mask yet"}')
    return False


def run_t14_clear_corridor(
    watcher: MemoryWatcher,
    barrier_xy: Tuple[float, float],
    *,
    safe_offset_m: float = 2.5,
) -> bool:
    """T14-clear: after accepted lidar clear, intended corridor must be free.

    Queries the barrier/corridor cell and a documented nearby safe offset.
    Does not widen accept radius; soft centre alone is not a PASS if neighbours
    remain lethal on the intended corridor.
    """
    log('=== T14-clear: intended corridor free after clear ===')
    bx, by = barrier_xy
    # Safe corridor sample: offset perpendicular-ish toward start side.
    dx = START[0] - bx
    dy = START[1] - by
    dist = math.hypot(dx, dy)
    if dist < 1e-3:
        sx, sy = bx - safe_offset_m, by
    else:
        # Offset along approach (start side) so we stay on-trail if possible.
        ux, uy = dx / dist, dy / dist
        sx = bx + safe_offset_m * ux
        sy = by + safe_offset_m * uy

    deadline = time.monotonic() + 30.0
    last_detail = ''
    while time.monotonic() < deadline:
        watcher.spin(0.3)
        mask_rep = watcher.mask_occupancy_report(bx, by, accept_radius_m=0.60)
        cost_rep = watcher.costmap_lethal_report(bx, by, accept_radius_m=0.60)
        safe_cost = watcher.cost_at(sx, sy)
        safe_mask = watcher.mask_occupied_near(sx, sy, radius_m=0.6)
        if mask_rep is None or cost_rep is None:
            continue
        corridor_mask_free = not mask_rep.ok
        corridor_cost_free = not cost_rep.ok
        last_detail = (
            f'corridor mask_ok(lethal)={mask_rep.ok}[{mask_rep.detail}] '
            f'cost_ok(lethal)={cost_rep.ok}[{cost_rep.detail}] '
            f'centre_cost={cost_rep.centre_cost} '
            f'safe=({sx:.2f},{sy:.2f}) cost={safe_cost} mask={safe_mask}'
        )
        if corridor_mask_free and corridor_cost_free and not safe_mask:
            # Safe corridor should remain free (not newly sealed).
            if safe_cost is not None and safe_cost >= LETHAL and safe_cost != 255:
                log(f'FAIL  T14-clear safe corridor lethal {last_detail}')
                return False
            log(f'  PASS  T14-clear {last_detail}')
            return True
    log(f'FAIL  T14-clear corridor still lethal/occupied {last_detail}')
    return False


def run_t15_clear_rehydrate(
    watcher: MemoryWatcher,
    observer: NavObserver,
    barrier_xy: Tuple[float, float],
) -> bool:
    """T15: clear_entirely_global_costmap must not permanently erase keepouts."""
    log('=== T15: TransientLocal rehydrate after global clear ===')
    from nav2_msgs.srv import ClearEntireCostmap

    bx, by = barrier_xy
    cell = None
    deadline = time.monotonic() + 12.0
    while time.monotonic() < deadline:
        watcher.spin(0.3)
        cell = watcher.nearest_mask_occupied_xy(bx, by, radius_m=2.5)
        if cell is not None:
            break
    if cell is None:
        log('FAIL  T15 precondition: no mask occupied cell near barrier '
            f'(radius 2.5 m); occupied={watcher.mask_occupied_count()}')
        return False
    log(f'  precondition cell=({cell[0]:.2f},{cell[1]:.2f})')

    svc = '/husky1/global_costmap/clear_entirely_global_costmap'
    client = observer.create_client(ClearEntireCostmap, svc)
    if not client.wait_for_service(timeout_sec=15.0):
        log(f'FAIL  T15 {svc} unavailable')
        return False
    req = ClearEntireCostmap.Request()
    fut = client.call_async(req)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 5.0 and not fut.done():
        watcher.spin(0.1)
    log(f'  Called {svc}')

    # Rehydrate via TransientLocal mask + filter_info refresh (≤12 s default).
    # No periodic mask force-republish (Entry 023).
    mask_ok = False
    lethal_ok = False
    t_rehydrate = None
    deadline = time.monotonic() + 45.0
    t_clear = time.monotonic()
    while time.monotonic() < deadline:
        watcher.spin(0.3)
        cost_rep = watcher.costmap_lethal_report(bx, by, accept_radius_m=0.80)
        mask_rep = watcher.mask_occupancy_report(bx, by, accept_radius_m=0.80)
        if mask_rep is not None and mask_rep.ok:
            mask_ok = True
        if cost_rep is not None and cost_rep.ok:
            lethal_ok = True
            if t_rehydrate is None:
                t_rehydrate = time.monotonic() - t_clear
        if mask_ok and lethal_ok:
            break

    log(f'  after wipe: mask_ok={mask_ok} lethal_ok={lethal_ok} '
        f'rehydrate_s={t_rehydrate} path1={watcher.bank_path_state(1)} '
        f'active={watcher.bank_active_id()}')
    if not mask_ok:
        log('FAIL  T15 mask lost after clear_entirely')
        return False
    if not lethal_ok:
        log('FAIL  T15 costmap_raw keepout did not rehydrate')
        return False
    if watcher.bank_path_state(1) == 'available' and watcher.bank_active_id() == 1:
        log('FAIL  T15 returned to Path A after wipe')
        return False
    log('PASS  T15 keepout rehydrates after global clear')
    return True


def run_t4_suppress_scan(
    watcher: MemoryWatcher,
    barrier_xy: Tuple[float, float],
) -> bool:
    """T4: memory retained when scan goes stale (no messages ≠ clear).

    Unit tests cover SENSOR_STALE. Here we assert off-FOV retention after
    physical removal already happened in T3 — mask must still be occupied
    without requiring new scan hits at the barrier.
    """
    log('=== T4: memory retained without new observations at A ===')
    bx, by = barrier_xy
    n0 = watcher.mask_occupied_count()
    watcher.spin(3.0)
    n1 = watcher.mask_occupied_count()
    retained = watcher.mask_occupied_near(bx, by, radius_m=1.5)
    log(f'  occupied before={n0} after={n1} mask_near={retained}')
    if not retained:
        log('FAIL  T4 mask cleared without covering observations')
        return False
    log('PASS  T4 memory retained without new A observations')
    return True


def _approach_pose(
    barrier_xy: Tuple[float, float],
    standoff_m: float = 5.0,
) -> Tuple[float, float, float]:
    """Pose on the start side of the barrier, facing it for clear rays."""
    bx, by = barrier_xy
    dx = START[0] - bx
    dy = START[1] - by
    dist = math.hypot(dx, dy)
    if dist < 1e-3:
        return (bx - standoff_m, by, 0.0)
    ux, uy = dx / dist, dy / dist
    ax = bx + standoff_m * ux
    ay = by + standoff_m * uy
    yaw = math.atan2(by - ay, bx - ax)
    return (ax, ay, yaw)


def _lifecycle_change(
    observer: NavObserver,
    node_name: str,
    transition_id: int,
    label: str,
) -> bool:
    """Change a Nav2 lifecycle node state (e.g. deactivate controller)."""
    try:
        from lifecycle_msgs.srv import ChangeState
        from lifecycle_msgs.msg import Transition

        client = observer.create_client(ChangeState, f'{node_name}/change_state')
        if not client.wait_for_service(timeout_sec=5.0):
            log(f'  WARN  {node_name}/change_state unavailable')
            return False
        req = ChangeState.Request()
        req.transition.id = int(transition_id)
        req.transition.label = label
        fut = client.call_async(req)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 8.0 and not fut.done():
            observer.spin_for(0.1)
        ok = bool(fut.done() and fut.result() is not None and fut.result().success)
        log(f'  lifecycle {node_name} → {label} (id={transition_id}) ok={ok}')
        return ok
    except Exception as exc:
        log(f'  WARN  lifecycle {node_name} {label} failed: {exc}')
        return False


def _pause_nav_for_transport(observer: NavObserver) -> None:
    """Stop Nav2 from fighting open-loop / set_pose on cmd_vel.

    Path-bank switch_resend and the velocity smoother otherwise keep driving
    Path B after cancel_goal (seen as open-loop peak_cmd=0.08 while robot
    continues toward the goal).
    """
    from lifecycle_msgs.msg import Transition

    try:
        observer.cancel_goal()
    except Exception:
        pass
    # Deactivate bt_navigator first so it cannot re-send FollowPath.
    _lifecycle_change(
        observer, '/husky1/bt_navigator',
        Transition.TRANSITION_DEACTIVATE, 'deactivate')
    _lifecycle_change(
        observer, '/husky1/controller_server',
        Transition.TRANSITION_DEACTIVATE, 'deactivate')
    # Zero cmd_vel briefly.
    from geometry_msgs.msg import Twist
    for _ in range(10):
        observer.cmd_vel_pub.publish(Twist())
        observer.spin_for(0.05)


def _resume_nav_after_transport(observer: NavObserver) -> None:
    from lifecycle_msgs.msg import Transition

    _lifecycle_change(
        observer, '/husky1/controller_server',
        Transition.TRANSITION_ACTIVATE, 'activate')
    _lifecycle_change(
        observer, '/husky1/bt_navigator',
        Transition.TRANSITION_ACTIVATE, 'activate')


def _open_loop_toward(
    observer: NavObserver,
    watcher: MemoryWatcher,
    target_xy: Tuple[float, float],
    *,
    stop_dist_m: float = 5.0,
    timeout_s: float = 90.0,
    linear_mps: float = 0.45,
) -> Optional[float]:
    """Open-loop cmd_vel toward target (bypasses keepout-blocked NavFn).

    TEST TRANSPORT ONLY — remembered keepouts prevent NavigateToPose into the
    observation pose. Does not count as autonomous nav success; clearing proof
    is still live lidar → memory → mask → costmap → bank. Returns final dist
    to target. Caller must pause Nav2 controller/bt first or smoother wins.
    """
    from geometry_msgs.msg import Twist

    pub = observer.cmd_vel_pub
    deadline = time.monotonic() + timeout_s
    last_dist: Optional[float] = None
    peak_cmd = 0.0
    while time.monotonic() < deadline:
        watcher.spin(0.05)
        pose = observer.robot_pose()
        if pose is None:
            continue
        dx = target_xy[0] - pose[0]
        dy = target_xy[1] - pose[1]
        last_dist = math.hypot(dx, dy)
        if last_dist <= stop_dist_m:
            break
        desired = math.atan2(dy, dx)
        err = math.atan2(math.sin(desired - pose[2]), math.cos(desired - pose[2]))
        cmd = Twist()
        # Rotate in place until roughly aligned — creeping while misaligned
        # previously drove farther off-FOV along Path B heading.
        if abs(err) > 0.30:
            cmd.angular.z = 0.8 if err > 0.0 else -0.8
            cmd.linear.x = 0.0
        else:
            cmd.angular.z = max(-0.5, min(0.5, 1.5 * err))
            cmd.linear.x = linear_mps
        peak_cmd = max(peak_cmd, abs(float(cmd.linear.x)))
        pub.publish(cmd)
    for _ in range(5):
        pub.publish(Twist())
        watcher.spin(0.05)
    log(f'  open_loop done dist={last_dist} peak_cmd_linear={peak_cmd:.2f} '
        f'(transport only; not NavigateToPose proof)')
    return last_dist


def _set_memory_marking(observer: NavObserver, enabled: bool) -> bool:
    """Attempt 2 helper: suppress new marks during open-loop approach."""
    try:
        from rclpy.parameter import Parameter
        from rcl_interfaces.srv import SetParameters

        node_name = '/husky1/obstacle_memory_manager'
        client = observer.create_client(SetParameters, f'{node_name}/set_parameters')
        if not client.wait_for_service(timeout_sec=5.0):
            log(f'  WARN  {node_name}/set_parameters unavailable')
            return False
        req = SetParameters.Request()
        req.parameters = [
            Parameter('marking_enabled', Parameter.Type.BOOL, enabled).to_parameter_msg()
        ]
        fut = client.call_async(req)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 5.0 and not fut.done():
            observer.spin_for(0.1)
        ok = fut.done() and fut.result() is not None
        log(f'  marking_enabled:={enabled} ok={ok}')
        return bool(ok)
    except Exception as exc:
        log(f'  WARN  marking_enabled set failed: {exc}')
        return False


def _log_speed_params(observer: NavObserver) -> None:
    """One-shot proof that test_speed rewrite applied (or production defaults)."""
    try:
        from rcl_interfaces.srv import GetParameters

        for node, names in (
            ('/husky1/controller_server', ['FollowPath.desired_linear_vel']),
            ('/husky1/velocity_smoother', ['max_velocity', 'min_velocity']),
        ):
            client = observer.create_client(GetParameters, f'{node}/get_parameters')
            if not client.wait_for_service(timeout_sec=3.0):
                log(f'  WARN  {node}/get_parameters unavailable')
                continue
            req = GetParameters.Request()
            req.names = list(names)
            fut = client.call_async(req)
            t0 = time.monotonic()
            while time.monotonic() - t0 < 3.0 and not fut.done():
                observer.spin_for(0.05)
            if not fut.done() or fut.result() is None:
                log(f'  WARN  param get timeout for {node}')
                continue
            vals = []
            for p in fut.result().values:
                if p.type == p.TYPE_DOUBLE:
                    vals.append(f'{p.double_value}')
                elif p.type == p.TYPE_DOUBLE_ARRAY:
                    vals.append(str(list(p.double_array_value)))
                else:
                    vals.append(str(p))
            log(f'  PARAM {node}: ' + ', '.join(
                f'{n}={v}' for n, v in zip(names, vals)))
    except Exception as exc:
        log(f'  WARN  speed param probe failed: {exc}')


def run_t5_drive_back_clear(
    watcher: MemoryWatcher,
    observer: NavObserver,
    world: GazeboWorld,
    barrier_xy: Tuple[float, float],
) -> bool:
    """T5: off-FOV remove → open-loop approach → real lidar clear → AVAILABLE.

    Sequence (Entry 023):
      1. After T2, wait until barrier is off-FOV (dist>12)
      2. remove_model (Gazebo only — not memory clear)
      3. Retain window: mask/costmap still lethal; path1 blocked; active≠1
      4. cancel_goal + _open_loop_toward(_approach_pose) — transport only
      5. Wait live lidar free-ray clear (never fake free cells / memory APIs)
      6. T14-clear corridor + path1 AVAILABLE; active stays B (no auto-preempt)

    Max 2 open-loop approach attempts with distinct diagnoses.
    """
    log('=== T5: off-FOV remove → drive-back → lidar clear → AVAILABLE ===')
    log('  NOTE: open-loop approach is TEST TRANSPORT only (keepout seals '
        'NavFn). PASS requires real sensor→memory→mask→costmap→bank evidence.')
    bx, by = barrier_xy
    _log_speed_params(observer)

    path1_before = watcher.bank_path_state(1)
    active_before = watcher.bank_active_id()
    switches_before = watcher.switch_count()
    pose = observer.robot_pose()
    dist = None if pose is None else math.hypot(pose[0] - bx, pose[1] - by)
    log(f'  start: path1={path1_before} active={active_before} '
        f'switches={switches_before} dist={dist} pose={pose} '
        f'mask_near={watcher.mask_occupied_near(bx, by, radius_m=1.5)} '
        f'mem={watcher.status}')
    if path1_before != 'blocked':
        log(f'FAIL  T5 precondition path1={path1_before} (expected blocked)')
        return False

    # --- Step: wait off-FOV ---
    log('  Waiting until barrier is off-FOV (dist > 12 m or 100 s)...')
    deadline = time.monotonic() + 100.0
    far = False
    while time.monotonic() < deadline:
        watcher.spin(0.3)
        pose = observer.robot_pose()
        if pose is not None:
            dist = math.hypot(pose[0] - bx, pose[1] - by)
            if dist > 12.0:
                far = True
                log(f'  off-FOV pose=({pose[0]:.2f},{pose[1]:.2f},{pose[2]:.2f}) '
                    f'dist={dist:.2f}')
                break
    if not far:
        log(f'  INFO  off-FOV by pose not confirmed (dist={dist}); '
            'removing anyway for retain check')

    # Pause Nav2 immediately so Path B does not flee farther during retain /
    # approach (prior FAIL: retain left robot ~20 m away).
    _pause_nav_for_transport(observer)

    # --- Step: remove barrier in Gazebo ---
    if not world.remove_model('sensor_only_block_a'):
        _resume_nav_after_transport(observer)
        log('FAIL  T5 could not remove barrier from Gazebo')
        return False
    log('  Barrier removed from Gazebo (memory must retain until lidar clear)')

    # --- Step: retain window (stationary; Nav2 paused) ---
    retain_ok = False
    t_retain = time.monotonic()
    while time.monotonic() - t_retain < 20.0:
        watcher.spin(0.3)
        mask_hit = watcher.mask_occupied_near(bx, by, radius_m=1.5)
        lethal_hit = watcher.lethal_near(bx, by, radius_m=1.5)
        path1 = watcher.bank_path_state(1)
        active = watcher.bank_active_id()
        if (mask_hit or lethal_hit) and path1 == 'blocked' and active not in (None, 1):
            retain_ok = True
            if time.monotonic() - t_retain > 6.0:
                break
    pose = observer.robot_pose()
    dist = None if pose is None else math.hypot(pose[0] - bx, pose[1] - by)
    cost_rep = watcher.costmap_lethal_report(bx, by)
    mask_rep = watcher.mask_occupancy_report(bx, by)
    log(f'  retain: ok={retain_ok} dist={dist} '
        f'path1={watcher.bank_path_state(1)} active={watcher.bank_active_id()} '
        f'mask[{None if mask_rep is None else mask_rep.detail}] '
        f'cost[{None if cost_rep is None else cost_rep.detail}]')
    if not retain_ok:
        _resume_nav_after_transport(observer)
        log('FAIL  T5 memory/bank did not retain Path A block after off-FOV remove')
        return False

    # --- Step: transport into FOV (max 2 attempts; Nav2 already paused) ---
    approach = _approach_pose(barrier_xy, standoff_m=5.0)
    log(f'  approach_pose=({approach[0]:.2f},{approach[1]:.2f},{approach[2]:.2f})')
    approach_ok = False
    final_approach_dist: Optional[float] = None
    # Suppress new marks for the whole approach+clear window so remaking cannot
    # re-seal the corridor while we gather free-ray clear evidence.
    _set_memory_marking(observer, False)
    for attempt in (1, 2):
        log(f'  --- approach attempt {attempt}/2 ---')
        watcher.spin(0.3)

        if attempt == 1:
            # Rotate-then-drive open-loop (Nav2 paused).
            to_approach = _open_loop_toward(
                observer,
                watcher,
                (approach[0], approach[1]),
                stop_dist_m=1.0,
                timeout_s=150.0,
                linear_mps=0.85,
            )
        else:
            # Diagnosis change: short open-loop segments toward barrier itself
            # (not only approach pose) after a pure rotate.
            to_approach = _open_loop_toward(
                observer,
                watcher,
                (bx, by),
                stop_dist_m=5.0,
                timeout_s=150.0,
                linear_mps=0.70,
            )

        pose = observer.robot_pose()
        if pose is None:
            log(f'  attempt {attempt}: no pose after transport')
            continue
        final_approach_dist = math.hypot(pose[0] - bx, pose[1] - by)
        log(f'  attempt {attempt}: pose=({pose[0]:.2f},{pose[1]:.2f},{pose[2]:.2f}) '
            f'dist_barrier={final_approach_dist:.2f} dist_approach={to_approach}')
        if final_approach_dist <= 8.0:
            approach_ok = True
            break
        log(f'  attempt {attempt}: still outside clear FOV '
            f'(dist={final_approach_dist:.2f} > 8)')

    if not approach_ok:
        _set_memory_marking(observer, True)
        _resume_nav_after_transport(observer)
        log(f'BLOCKED  T5 could not reach clear FOV after 2 transport attempts '
            f'(final_dist={final_approach_dist}). Keepout/SLAM may seal approach. '
            'Not inventing clear evidence.')
        return False

    # Face barrier for free-ray clears.
    pose = observer.robot_pose()
    if pose is not None:
        desired_yaw = math.atan2(by - pose[1], bx - pose[0])
        from geometry_msgs.msg import Twist
        t_yaw = time.monotonic()
        while time.monotonic() - t_yaw < 12.0:
            watcher.spin(0.05)
            pose = observer.robot_pose()
            if pose is None:
                continue
            err = math.atan2(
                math.sin(desired_yaw - pose[2]),
                math.cos(desired_yaw - pose[2]))
            if abs(err) < 0.2:
                break
            cmd = Twist()
            cmd.angular.z = 0.6 if err > 0.0 else -0.6
            observer.cmd_vel_pub.publish(cmd)
        for _ in range(3):
            observer.cmd_vel_pub.publish(Twist())
            watcher.spin(0.05)

    # --- Step: wait real lidar clear (marking still off) ---
    # Slowly yaw-sweep so beams traverse the keepout blob; static heading can
    # leave barrier cells as HIT_AT_CELL / OFF-beam forever.
    log('  Waiting for live lidar clear evidence (≤150 s); marking_enabled=False; '
        'yaw-sweeping toward barrier...')
    mask_cleared = False
    path1_available = False
    flipped_to_a = False
    t_clear = time.monotonic()
    deadline = t_clear + 150.0
    sweep_sign = 1.0
    last_progress_log = t_clear
    from geometry_msgs.msg import Twist
    while time.monotonic() < deadline:
        watcher.spin(0.1)
        pose = observer.robot_pose()
        if pose is not None:
            dist = math.hypot(pose[0] - bx, pose[1] - by)
            desired_yaw = math.atan2(by - pose[1], bx - pose[0])
            # Sweep ±0.55 rad about the barrier bearing.
            phase = math.sin((time.monotonic() - t_clear) * 0.7)
            target_yaw = desired_yaw + 0.55 * phase * sweep_sign
            err = math.atan2(
                math.sin(target_yaw - pose[2]), math.cos(target_yaw - pose[2]))
            cmd = Twist()
            cmd.angular.z = max(-0.55, min(0.55, 1.8 * err))
            # Creep closer if still > 5 m so clear_max / beam density improve.
            if dist > 5.0 and abs(err) < 0.35:
                cmd.linear.x = 0.25
            observer.cmd_vel_pub.publish(cmd)
        # Corridor clear = no mask occupancy within accept radius of barrier.
        mask_rep = watcher.mask_occupancy_report(bx, by, accept_radius_m=0.60)
        if mask_rep is not None and not mask_rep.ok:
            mask_cleared = True
        if watcher.bank_path_state(1) == 'available':
            path1_available = True
        if watcher.bank_active_id() == 1 and active_before not in (None, 1):
            flipped_to_a = True
            break
        if mask_cleared and path1_available and not flipped_to_a:
            break
        if time.monotonic() - last_progress_log > 20.0:
            last_progress_log = time.monotonic()
            log(f'  clear progress: dist={dist} mask_cleared={mask_cleared} '
                f'path1={watcher.bank_path_state(1)} '
                f'mask[{None if mask_rep is None else mask_rep.detail}] '
                f'mem={watcher.status}')
    for _ in range(3):
        observer.cmd_vel_pub.publish(Twist())
        watcher.spin(0.05)

    path1_after = watcher.bank_path_state(1)
    active_after = watcher.bank_active_id()
    switches_after = watcher.switch_count()
    cost_rep = watcher.costmap_lethal_report(bx, by, accept_radius_m=0.60)
    mask_rep = watcher.mask_occupancy_report(bx, by, accept_radius_m=0.60)
    mask_corridor_blocked = bool(mask_rep is not None and mask_rep.ok)
    log(f'  after clear wait: dist={dist} '
        f'mask_rep[{None if mask_rep is None else mask_rep.detail}] '
        f'cost_rep[{None if cost_rep is None else cost_rep.detail}] '
        f'path1={path1_after} active={active_after} '
        f'switches {switches_before}→{switches_after} '
        f'mask_cleared={mask_cleared} path1_available={path1_available} '
        f'flipped_to_a={flipped_to_a} mem={watcher.status}')

    ok = True
    if flipped_to_a or (active_after == 1 and active_before != 1):
        log('FAIL  T5 auto-preempted back to Path A on unlock')
        ok = False
    if mask_corridor_blocked:
        log('FAIL  T5 mask still occupied on intended corridor after clear')
        ok = False
    if cost_rep is not None and cost_rep.ok:
        log('FAIL  T5 costmap_raw still lethal on intended corridor')
        ok = False
    if path1_after != 'available':
        log(f'FAIL  T5 path1={path1_after} (expected available after clear)')
        ok = False
    # Switch count must not bump solely from unlock (active stays B).
    if switches_after > switches_before + 1:
        log(f'FAIL  T5 unexpected switch churn ({switches_before}→{switches_after})')
        ok = False

    t14_clear = False
    if ok:
        t14_clear = run_t14_clear_corridor(watcher, barrier_xy)
        if not t14_clear:
            ok = False
    else:
        log('  SKIP  T14-clear (T5 clear assertions already failed)')

    _set_memory_marking(observer, True)
    # Resume Nav2 after observation; Path B should remain active (no auto-preempt).
    _resume_nav_after_transport(observer)

    if ok and t14_clear:
        log('PASS  T5 drive-back clear → AVAILABLE without auto-preempt '
            '(+ T14-clear corridor free)')
    return ok


# Back-compat alias for any external callers.
run_t5_fov_clear = run_t5_drive_back_clear


def _respawn_barrier_for_t3(
    watcher: MemoryWatcher,
    observer: NavObserver,
    world: GazeboWorld,
    barrier_xy: Tuple[float, float],
) -> bool:
    """Re-block Path A after T5 so the off-FOV T3 sequence can run."""
    bx, by = barrier_xy
    log('=== Re-block Path A for T3 off-FOV sequence ===')
    # Ensure we are still navigating so the bank watches the live path.
    observer.send_goal(GOAL[0], GOAL[1], GOAL[2])
    watcher.spin(1.0)
    spec = barrier_across_heading(
        name='sensor_only_block_a',
        at=(bx, by),
        heading=0.0,
        width=2.2,
        thickness=0.45,
        height=1.6,
    )
    if not world.spawn_obstacle(spec):
        log('FAIL  re-block spawn')
        return False
    deadline = time.monotonic() + 90.0
    while time.monotonic() < deadline:
        watcher.spin(0.3)
        if watcher.bank_path_state(1) == 'blocked' and watcher.mask_occupied_near(
                bx, by, radius_m=1.5):
            log(f'  re-blocked path1={watcher.bank_path_state(1)} '
                f'active={watcher.bank_active_id()}')
            return True
    log('FAIL  re-block did not mark Path A blocked')
    return False


def run_t8_new_obstacle(
    watcher: MemoryWatcher,
    observer: NavObserver,
    world: GazeboWorld,
    barrier_xy: Tuple[float, float],
) -> bool:
    """T8: new obstacle marks mask and blocks the relevant path; prior memory
    behaviour preserved for unrelated cells (here: rematch Path A after T5)."""
    log('=== T8/T17: re-block cleared Path A corridor ===')
    bx, by = barrier_xy
    # Orient barrier roughly across trail (eastbound).
    spec = barrier_across_heading(
        name='sensor_only_reblock_a',
        at=(bx, by),
        heading=0.0,
        width=2.2,
        thickness=0.45,
        height=1.6,
    )
    if not world.spawn_obstacle(spec):
        log('FAIL  T8 spawn reblock barrier')
        return False

    deadline = time.monotonic() + 90.0
    marked = False
    blocked = False
    while time.monotonic() < deadline:
        watcher.spin(0.3)
        if watcher.mask_occupied_near(bx, by, radius_m=1.5):
            marked = True
        if watcher.bank_path_state(1) == 'blocked':
            blocked = True
        if marked and blocked:
            break

    world.remove_model('sensor_only_reblock_a')
    log(f'  marked={marked} blocked={blocked} active={watcher.bank_active_id()} '
        f'mem={watcher.status}')
    if not marked:
        log('FAIL  T8 new obstacle not in mask')
        return False
    if not blocked:
        log('FAIL  T8 Path 1 not blocked after rematch')
        return False
    log('PASS  T8/T17 new obstacle rematches Path A')
    return True


def _candidate_path_pts(
    observer: NavObserver, path_id: int, timeout: float = 15.0
) -> Optional[List[Tuple[float, float]]]:
    """Fetch path_bank/candidate_{id} polyline points."""
    holder: List[Optional[List[Tuple[float, float]]]] = [None]

    def _cb(msg: RosPath) -> None:
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if pts:
            holder[0] = pts

    sub = observer.create_subscription(
        RosPath, f'path_bank/candidate_{path_id}', _cb, 5)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and holder[0] is None:
        observer.spin_for(0.2)
    observer.destroy_subscription(sub)
    return holder[0]


def run_t11_return_home(
    watcher: MemoryWatcher,
    observer: NavObserver,
    world: GazeboWorld,
    barrier_a_xy: Tuple[float, float],
) -> bool:
    """T11: after Path A then Path B blocked, return toward recorded home.

    Preconditions: T2 already switched to Path B with barrier A in place.
    Spawns a second barrier on the active Path B corridor, then asserts
    RETURNING_TO_START / RETURNED_TO_START and westward motion toward START.
    """
    log('=== T11: ALL_BLOCKED → return to start ===')
    del barrier_a_xy  # retained for call-site symmetry with other scenarios
    active = watcher.bank_active_id()
    if active in (None, 1):
        log(f'FAIL  T11 expected active Path B-ish, got active={active}')
        return False

    pose = observer.robot_pose()
    if pose is None:
        log('FAIL  T11 no robot pose before Path B block')
        return False
    dist_home0 = math.hypot(pose[0] - START[0], pose[1] - START[1])
    east0 = pose[0]

    path_b = _candidate_path_pts(observer, int(active), timeout=20.0)
    if not path_b:
        # Fallback: use live plan if candidate topic not refreshed.
        plan = observer.latest_path()
        if plan is None or len(plan.points) < 4:
            log('FAIL  T11 no Path B candidate / plan')
            return False
        path_b = plan.points
        log('  Using live plan as Path B proxy')

    block_xy = _barrier_xy_ahead_on_path(path_b, pose, ahead_m=5.0)
    mid_i = min(
        range(len(path_b)),
        key=lambda i: math.hypot(
            path_b[i][0] - block_xy[0], path_b[i][1] - block_xy[1]))
    i0 = max(0, mid_i - 2)
    i1 = min(len(path_b) - 1, mid_i + 2)
    yaw = math.atan2(
        path_b[i1][1] - path_b[i0][1], path_b[i1][0] - path_b[i0][0])
    spec = barrier_across_heading(
        name='sensor_only_block_b',
        at=(block_xy[0], block_xy[1]),
        heading=yaw,
        width=2.2,
        thickness=0.45,
        height=1.6,
    )
    log(f'  Path B barrier at ({block_xy[0]:.2f},{block_xy[1]:.2f}) '
        f'active={active} dist_home0={dist_home0:.2f}m')
    if not world.spawn_obstacle(spec):
        log('FAIL  T11 spawn Path B barrier')
        return False

    # Wait for return-home state (NO_PATH_AVAILABLE may flash first).
    returning = False
    returned = False
    deadline = time.monotonic() + 90.0
    while time.monotonic() < deadline:
        watcher.spin(0.3)
        st = watcher.bank_state
        if st == 'RETURNING_TO_START':
            returning = True
            break
        if st == 'RETURNED_TO_START':
            returning = True
            returned = True
            break
        # Status may lag state topic; also accept nav_mode=home.
        if 'nav_mode=home' in watcher.bank_status:
            returning = True
            break
    if not returning:
        log(f'FAIL  T11 never entered RETURNING_TO_START '
            f'(state={watcher.bank_state!r} status=\n{watcher.bank_status})')
        return False
    log(f'  return armed state={watcher.bank_state} '
        f'nav_mode_line={[ln for ln in watcher.bank_status.splitlines() if "nav_mode" in ln or ln.startswith("home=")]}')

    # Motion toward home (west toward spawn), not re-commit east to GOAL.
    progress_deadline = time.monotonic() + 75.0
    best_home_gain = 0.0
    saw_east_regress = False
    while time.monotonic() < progress_deadline:
        watcher.spin(0.3)
        if watcher.bank_state == 'RETURNED_TO_START':
            returned = True
            break
        p = observer.robot_pose()
        if p is None:
            continue
        dist_home = math.hypot(p[0] - START[0], p[1] - START[1])
        best_home_gain = max(best_home_gain, dist_home0 - dist_home)
        if p[0] > east0 + 2.0:
            saw_east_regress = True
        if best_home_gain >= 1.5:
            break

    final = observer.robot_pose()
    final_dist = (
        math.hypot(final[0] - START[0], final[1] - START[1])
        if final is not None else float('nan')
    )
    log(f'  home_gain={best_home_gain:.2f}m final_dist_home={final_dist:.2f}m '
        f'returned={returned} east_regress={saw_east_regress} '
        f'state={watcher.bank_state}')

    if saw_east_regress and best_home_gain < 1.0:
        log('FAIL  T11 resumed eastward mission after return armed')
        return False
    if not returned and best_home_gain < 1.5:
        log(f'FAIL  T11 insufficient motion toward home '
            f'(gain={best_home_gain:.2f}m)')
        return False
    log('PASS  T11 ALL_BLOCKED → return toward start')
    return True


def run_t10_no_oscillation(
    watcher: MemoryWatcher,
    settle_s: float = 20.0,
) -> bool:
    """T10: no A↔B oscillation while keepout retains Path A."""
    log('=== T10: no Path A↔B oscillation ===')
    active0 = watcher.bank_active_id()
    switches0 = watcher.switch_count()
    flips = 0
    last = active0
    t0 = time.monotonic()
    while time.monotonic() - t0 < settle_s:
        watcher.spin(0.3)
        cur = watcher.bank_active_id()
        if cur != last and cur is not None and last is not None:
            flips += 1
        last = cur if cur is not None else last
    switches1 = watcher.switch_count()
    log(f'  active0={active0} active1={watcher.bank_active_id()} '
        f'switch_delta={switches1 - switches0} flips={flips}')
    if flips >= 2 or (switches1 - switches0) >= 2:
        log('FAIL  T10 oscillation detected')
        return False
    log('PASS  T10 no oscillation')
    return True


def run_stuck_recovery_local(
    watcher: MemoryWatcher,
    observer: NavObserver,
    world: GazeboWorld,
    barrier_xy: Tuple[float, float],
) -> bool:
    """Stuck recovery: temporary local block clears; remembered Path A keepout
    is not erased by local ClearEntireCostmap recoveries."""
    log('=== Stuck recovery: local clear + keepout persistence ===')
    bx, by = barrier_xy
    pose = observer.robot_pose()
    if pose is None:
        log('FAIL  stuck recovery: no robot pose')
        return False
    # Drop a temporary wall just ahead of the robot.
    yaw = pose[2] if len(pose) > 2 else 0.0
    ahead = (pose[0] + 2.0 * math.cos(yaw), pose[1] + 2.0 * math.sin(yaw))
    spec = barrier_across_heading(
        name='sensor_only_stuck_tmp',
        at=ahead,
        heading=yaw,
        width=2.0,
        thickness=0.40,
        height=1.6,
    )
    mask_before = watcher.mask_occupied_near(bx, by, radius_m=1.5)
    path1_before = watcher.bank_path_state(1)
    if not world.spawn_obstacle(spec):
        log('FAIL  stuck: spawn temp barrier')
        return False

    # Resume goal so BT recovery can run if FollowPath fails.
    observer.send_goal(GOAL[0], GOAL[1], GOAL[2])
    t_block = time.monotonic()
    watcher.spin(8.0)
    world.remove_model('sensor_only_stuck_tmp')
    recovered = False
    deadline = time.monotonic() + 60.0
    while time.monotonic() < deadline:
        watcher.spin(0.3)
        p = observer.robot_pose()
        if p is None:
            continue
        # Moved away from the temp barrier site after removal.
        if math.hypot(p[0] - ahead[0], p[1] - ahead[1]) > 1.5:
            recovered = True
            break
        # Or progressed toward goal.
        if math.hypot(p[0] - GOAL[0], p[1] - GOAL[1]) < math.hypot(
                pose[0] - GOAL[0], pose[1] - GOAL[1]) - 1.0:
            recovered = True
            break
    t_recover = time.monotonic() - t_block
    mask_after = watcher.mask_occupied_near(bx, by, radius_m=1.5)
    path1_after = watcher.bank_path_state(1)
    log(f'  recovered={recovered} t={t_recover:.1f}s '
        f'mask_A_before={mask_before} after={mask_after} '
        f'path1 {path1_before}→{path1_after}')

    # Keepout for Path A: if it was occupied before stuck test, must remain
    # unless T5 already cleared it. If path1 was blocked, stay blocked.
    ok = True
    if path1_before == 'blocked' and mask_before and not mask_after:
        log('FAIL  stuck recovery erased Path A keepout mask')
        ok = False
    if not recovered:
        log('FAIL  stuck recovery: robot did not resume after temp clear')
        ok = False
    if ok:
        log('PASS  stuck recovery (local) + keepout retained')
    return ok


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--scenario',
        default='priority',
        choices=(
            't1', 't2', 't3', 't4', 't5', 't8', 't10', 't11', 't14', 't15',
            'stuck', 'priority', 'extended', 'all',
        ),
        help=(
            'priority=T1–T4+T14-block+T15; extended=+T8+T10+stuck; '
            't11=block A+B → return home '
            '(not T5 — use --scenario t5 for drive-back clear)'
        ),
    )
    args = parser.parse_args()

    # test_speed for scenarios that need faster open-trail / approach motion.
    use_test_speed = args.scenario in (
        't5', 't11', 'extended', 'stuck', 'all',
    )
    sup = bringup(
        world=WORLD,
        nav2=True,
        rviz=False,
        gui=False,
        husky_x=START[0],
        husky_y=START[1],
        husky_yaw=START[2],
        max_runtime=1200.0,
        log_path='/tmp/sensor_only_costmap_test.log',
        extra={
            'husky_z': '0.50',
            'use_prior_map': 'true',
            'prior_map_file': 'bush_trail_world.yaml',
            'path_bank': 'true',
            'path_bank_switch_resend': 'true',
            'fire_avoidance': 'false',
            'test_speed': 'true' if use_test_speed else 'false',
        },
    )

    results = {}
    with sup:
        ok, _ = await_simulation(sup, require_nav2=True)
        if not ok:
            print_diagnostics(sup)
            log('SENSOR_ONLY COSTMAP TEST FAILED (sim did not start)')
            return 1

        init_ros()
        observer = NavObserver()
        watcher = MemoryWatcher(observer)
        world = GazeboWorld(WORLD, logger=lambda m: log(f'  {m}'))
        if not world.wait_until_available(max_wait=60.0):
            log('FAIL  Gazebo world unavailable')
            return 1

        try:
            chain = {
                't2', 't3', 't4', 't5', 't8', 't10', 't11', 't14', 't15',
                'stuck', 'priority', 'extended', 'all',
            }
            need_chain = args.scenario in chain
            if args.scenario in ('t1', 'priority', 'extended', 'all'):
                results['T1'] = run_t1_clean(watcher)
            else:
                watcher.wait_mask(90.0)

            barrier_xy = None
            if need_chain and args.scenario != 't1':
                barrier_xy = run_t2_block(watcher, observer, world)
                results['T2'] = barrier_xy is not None

            # T5 is a dedicated scenario: off-FOV remove → open-loop approach →
            # real lidar clear. Not chained into priority/extended (would destroy
            # the T3 barrier). Run: --scenario t5
            if args.scenario == 't5':
                if barrier_xy is None:
                    results['T5'] = False
                else:
                    results['T5'] = run_t5_drive_back_clear(
                        watcher, observer, world, barrier_xy)

            # T11: block Path B after T2 → return home. Dedicated (mutates bank).
            if args.scenario == 't11':
                if barrier_xy is None:
                    results['T11'] = False
                else:
                    results['T11'] = run_t11_return_home(
                        watcher, observer, world, barrier_xy)

            if args.scenario in ('t3', 'priority', 'extended', 'all'):
                if barrier_xy is None:
                    results['T3'] = False
                    log('FAIL  T3 skipped (no barrier from T2)')
                else:
                    results['T3'] = run_t3_primary(
                        watcher, observer, world, barrier_xy)

            if args.scenario in ('t4', 't3', 'priority', 'extended', 'all'):
                if barrier_xy is not None and results.get(
                        'T3', args.scenario == 't4'):
                    results['T4'] = run_t4_suppress_scan(watcher, barrier_xy)

            if args.scenario in ('t14', 'priority', 'extended', 'all'):
                if barrier_xy is None:
                    results['T14'] = False
                else:
                    results['T14'] = run_t14_costmap_raw(watcher, barrier_xy)

            if args.scenario in ('t15', 'priority', 'extended', 'all'):
                if barrier_xy is None:
                    results['T15'] = False
                else:
                    results['T15'] = run_t15_clear_rehydrate(
                        watcher, observer, barrier_xy)

            if args.scenario in ('t10', 'extended', 'all'):
                results['T10'] = run_t10_no_oscillation(watcher)

            if args.scenario in ('t8', 'extended', 'all'):
                if barrier_xy is None:
                    results['T8'] = False
                else:
                    results['T8'] = run_t8_new_obstacle(
                        watcher, observer, world, barrier_xy)

            if args.scenario in ('stuck', 'extended', 'all'):
                if barrier_xy is None:
                    results['stuck'] = False
                else:
                    results['stuck'] = run_stuck_recovery_local(
                        watcher, observer, world, barrier_xy)
        finally:
            try:
                observer.cancel_goal()
            except Exception:
                pass

    log('=== Summary ===')
    for k, v in results.items():
        log(f'  {k}: {"PASS" if v else "FAIL"}')
    if not results:
        return 1
    return 0 if all(results.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
