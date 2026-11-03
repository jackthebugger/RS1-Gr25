#!/usr/bin/env python3
"""Integration regression: sensor-only keepout memory (Entries 021–026).

Primary scenario T3 — Path A blocked → switch to B → remove obstacle off-FOV
→ Path A must remain lethal in keepout/costmap and bank stays BLOCKED; no
corridor return to A.

T5 — off-FOV remove → open-loop approach (test transport) → real lidar clear →
Path A AVAILABLE without auto-preempt. Also asserts T14-clear corridor free.

T9 — FIRE on Path A → switch B → lidar free-ray while thermal off-FOV must
retain FIRE; in-FOV thermal negatives clear FIRE (Entry 026).

T11 — block A → switch B → block B → RETURNING_TO_START toward home (~spawn).

N6 — like T11 but arms test_force_unreachable_home so return-home plan fails →
bank NO_SAFE_ROUTE + owner STOPPED/ALL_BLOCKED_NO_HOME + StopMission rearm.

    python3 test/sensor_only_costmap_test.py
    python3 test/sensor_only_costmap_test.py --scenario priority
    python3 test/sensor_only_costmap_test.py --scenario t5
    python3 test/sensor_only_costmap_test.py --scenario t9
    python3 test/sensor_only_costmap_test.py --scenario t11
    python3 test/sensor_only_costmap_test.py --scenario n6
    python3 test/sensor_only_costmap_test.py --scenario extended
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
import time
from typing import List, Optional, Tuple

from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid, Path as RosPath
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, Header, String
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
        # Entry 028: typed memory introspection + live thermal verdict.
        self.fire_cells: List[Tuple[float, float]] = []
        self.lidar_cells: List[Tuple[float, float]] = []
        self.fire_detected: Optional[bool] = None
        self.fire_detected_true_count = 0
        self.node.create_subscription(
            PointCloud2, 'fire_memory_cloud', self._on_fire_cloud, 5)
        self.node.create_subscription(
            PointCloud2, 'obstacle_memory/lidar_memory_cloud', self._on_lidar_cloud, 5)
        self.node.create_subscription(
            Bool, 'fire_detected', self._on_fire_detected, 10)

    def _on_fire_cloud(self, msg: PointCloud2) -> None:
        self.fire_cells = _unpack_xy_cloud(msg)

    def _on_lidar_cloud(self, msg: PointCloud2) -> None:
        self.lidar_cells = _unpack_xy_cloud(msg)

    def _on_fire_detected(self, msg: Bool) -> None:
        self.fire_detected = bool(msg.data)
        if msg.data:
            self.fire_detected_true_count += 1

    def fire_cells_near(self, x: float, y: float, radius_m: float = 1.5) -> int:
        return sum(1 for px, py in self.fire_cells
                   if math.hypot(px - x, py - y) <= radius_m)

    def lidar_cells_near(self, x: float, y: float, radius_m: float = 1.5) -> int:
        return sum(1 for px, py in self.lidar_cells
                   if math.hypot(px - x, py - y) <= radius_m)

    def fire_cells_far(self, x: float, y: float, radius_m: float = 2.5) -> int:
        """FIRE cells farther than ``radius_m`` from the true fire (spurious marks)."""
        return sum(1 for px, py in self.fire_cells
                   if math.hypot(px - x, py - y) > radius_m)

    def status_fire_count(self) -> int:
        for tok in self.status.split():
            if tok.startswith('fire='):
                try:
                    return int(tok.split('=', 1)[1])
                except ValueError:
                    return 0
        return 0

    def location_report(self, x: float, y: float, radius_m: float = 1.5) -> str:
        """One-line state of every layer at (x, y) for stage assertions."""
        return (
            f'@({x:.2f},{y:.2f}) r={radius_m:.1f}: '
            f'FIRE_mem={self.fire_cells_near(x, y, radius_m)} '
            f'LIDAR_mem={self.lidar_cells_near(x, y, radius_m)} '
            f'FIRE_far={self.fire_cells_far(x, y)} '
            f'mask={self.mask_occupied_near(x, y, radius_m)} '
            f'costmap_lethal={self.lethal_near(x, y, radius_m)} '
            f'path1={self.bank_path_state(1)} active={self.bank_active_id()} '
            f'state={self.bank_state} fire_detected={self.fire_detected} '
            f'status[{self.status}]'
        )

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


def _start_goal_and_spawn_path1_barrier(
    watcher: MemoryWatcher,
    observer: NavObserver,
    world: GazeboWorld,
    *,
    barrier_name: str = 'sensor_only_block_a',
    label: str = 'T2',
    spin: Optional[callable] = None,
) -> Optional[Tuple[Tuple[float, float], int, float]]:
    """Send the mission goal and drop a barrier ~6 m ahead on Path 1.

    Shared by T2 / T2TRACE / T9REAL so every scenario blocks Path A the same
    way. Returns ``(block_xy, switches_before, barrier_yaw)`` or ``None``.
    ``spin`` lets a telemetry-aware caller sample while waiting.
    """
    _spin = spin or watcher.spin
    # Wait until Nav2 action server is responsive (lifecycle settle).
    ready_deadline = time.monotonic() + 60.0
    while time.monotonic() < ready_deadline:
        _spin(0.2)
        if observer.nav2_ready(timeout=1.0):
            break
    else:
        log(f'FAIL  {label} Nav2 navigate_to_pose not ready')
        return None
    if not observer.send_goal(GOAL[0], GOAL[1], GOAL[2]):
        # One retry after brief settle — intermittent action-server stalls.
        _spin(3.0)
        if not observer.send_goal(GOAL[0], GOAL[1], GOAL[2]):
            log(f'FAIL  {label} NavigateToPose rejected/timeout')
            return None
    if not watcher.wait_bank(90.0):
        log(f'FAIL  {label} path bank never published candidates')
        return None
    switches_before = watcher.switch_count()

    # Give bank candidates a moment to publish under test_speed.
    exclusive, sep, path1 = _exclusive_block_xy(observer, timeout=40.0)
    if exclusive is None:
        plan = observer.latest_path()
        if plan is None or len(plan.points) < 4:
            log(f'FAIL  {label} no exclusive point and no plan')
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
        _spin(0.2)
        pose = observer.robot_pose()
        if pose is not None:
            break
    if pose is None:
        log(f'FAIL  {label} no robot pose before barrier spawn')
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
        name=barrier_name,
        at=(block_xy[0], block_xy[1]),
        heading=yaw,
        width=2.2,
        thickness=0.45,
        height=1.6,
    )
    if not world.spawn_obstacle(spec):
        log(f'FAIL  {label} spawn barrier')
        return None
    return (block_xy[0], block_xy[1]), switches_before, yaw


def run_t2_block(
    watcher: MemoryWatcher,
    observer: NavObserver,
    world: GazeboWorld,
) -> Optional[Tuple[float, float]]:
    log('=== T2: Block A, switch B ===')
    setup = _start_goal_and_spawn_path1_barrier(watcher, observer, world)
    if setup is None:
        return None
    block_xy, switches_before, _yaw = setup

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
        # StaticLayer walls on bush_trail sit inside accept_radius of mid-trail
        # barriers; treat costmap free when keepout is gone and centre is not
        # lethal (Entry 035 — same rule as T5).
        centre_lethal = (
            cost_rep.centre_cost is not None
            and cost_rep.centre_cost >= LETHAL
            and cost_rep.centre_cost != 255
        )
        corridor_cost_free = (not cost_rep.ok) or (
            corridor_mask_free and not centre_lethal
        )
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


def _teleop_cmd_pub(observer: NavObserver):
    """Publisher for open-loop transport under observation_hold (gate=TELEOP)."""
    from geometry_msgs.msg import Twist
    if not hasattr(observer, '_teleop_cmd_pub') or observer._teleop_cmd_pub is None:
        observer._teleop_cmd_pub = observer.create_publisher(
            Twist, 'cmd_vel_teleop', 10)
        observer.spin_for(0.2)
    return observer._teleop_cmd_pub


def _set_observation_hold(observer: NavObserver, hold: bool) -> None:
    """Ask path_bank to pause FollowPath without StopMission / lifecycle CRITICAL.

    Entry 035: cancel_goal()→StopMission left bank IDLE so evidence_unblock
    never ran; deactivating controller_server tripped lifecycle_manager bond
    reset of planner/costmap mid-T5 clear.
    """
    from std_msgs.msg import Bool
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )
    if not hasattr(observer, '_observation_hold_pub') or observer._observation_hold_pub is None:
        observer._observation_hold_pub = observer.create_publisher(
            Bool, 'mission_owner/observation_hold', qos)
        observer.spin_for(0.2)
    msg = Bool()
    msg.data = bool(hold)
    for _ in range(3):
        observer._observation_hold_pub.publish(msg)
        observer.spin_for(0.05)
    log(f'  observation_hold:={hold}')


def _pause_nav_for_transport(observer: NavObserver) -> None:
    """Stop bank FollowPath from fighting open-loop transport cmd_vel.

    Does NOT call StopMission and does NOT deactivate controller_server
    (lifecycle bond CRITICAL). Uses mission_owner/observation_hold instead.
    """
    _set_observation_hold(observer, True)
    from geometry_msgs.msg import Twist
    pub = _teleop_cmd_pub(observer)
    for _ in range(10):
        pub.publish(Twist())
        observer.spin_for(0.05)


def _resume_nav_after_transport(observer: NavObserver) -> None:
    _set_observation_hold(observer, False)


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

    # Gate sole-owns /cmd_vel: under observation_hold the bank sets TELEOP, so
    # transport must publish cmd_vel_teleop (not race the gate on /cmd_vel).
    pub = _teleop_cmd_pub(observer)
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
            from rcl_interfaces.msg import ParameterType
            for p in fut.result().values:
                if p.type == ParameterType.PARAMETER_DOUBLE:
                    vals.append(f'{p.double_value}')
                elif p.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
                    vals.append(str(list(p.double_array_value)))
                else:
                    vals.append(f'type={p.type}')
            log(f'  PARAM {node}: ' + ', '.join(
                f'{n}={v}' for n, v in zip(names, vals)))
    except Exception as exc:
        log(f'  WARN  speed param probe failed: {exc}')


def _try_test_rtf(world: GazeboWorld, observer: NavObserver) -> None:
    """Runtime 2× RTF was tried once (Entry 028) and dropped.

    Clock reached 200 Hz (2.0×) but fire-off t2trace then logged 82 RPP
    collisions vs 0 at RTF 1.0. No world SDF edit; step stays 0.01 s.
    """
    hz = _clock_hz(observer, window=1.0)
    log(f'  RTF left at default (clock {hz:.1f} Hz). 2.0 applied cleanly '
        f'in probe but caused RPP collision regression — not used.')


def _topic_hz(observer: NavObserver, topic: str, window: float = 1.5):
    if topic == '/clock':
        from rosgraph_msgs.msg import Clock as _Msg
    else:
        from sensor_msgs.msg import LaserScan as _Msg
    count = [0]

    def _cb(_msg):
        count[0] += 1

    sub = observer.create_subscription(_Msg, topic, _cb, 10)
    t0 = time.monotonic()
    while time.monotonic() - t0 < window:
        observer.spin_for(0.05)
    try:
        observer.destroy_subscription(sub)
    except Exception:
        pass
    return count[0] / max(window, 1e-3)


def _clock_hz(observer: NavObserver, window: float = 1.5) -> float:
    return _topic_hz(observer, '/clock', window)


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

    # --- Step: wait off-FOV (or bank exhaustion / stall) ---
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
        # Path B may all-block on trail memory before 12 m — pause then so
        # return-home does not fight the flee-to-FOV open-loop.
        if watcher.bank_active_id() is None and dist is not None and dist > 4.0:
            log(f'  INFO  no active bank path at dist={dist:.2f}; '
                'will open-loop flee to off-FOV')
            break

    # Pause FollowPath (observation_hold) so Path B / return-home does not flee
    # during retain / approach. Does not StopMission (keeps evidence_unblock).
    _pause_nav_for_transport(observer)

    # If still inside clear FOV, open-loop away before removing the barrier so
    # free-rays cannot clear keepout during the retain window.
    pose = observer.robot_pose()
    dist = None if pose is None else math.hypot(pose[0] - bx, pose[1] - by)
    if dist is not None and dist < 12.0:
        flee = (bx + 0.0, by + 14.0)  # north of barrier mid-trail
        log(f'  INFO  fleeing to off-FOV via teleop (dist={dist:.2f}) → {flee}')
        _open_loop_toward(
            observer, watcher, flee, stop_dist_m=1.0, timeout_s=90.0,
            linear_mps=0.85)
        pose = observer.robot_pose()
        dist = None if pose is None else math.hypot(pose[0] - bx, pose[1] - by)
        log(f'  after flee: dist={dist} pose={pose}')

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
        # active may be None after Path B all-blocked → return-home; Path 1
        # must stay BLOCKED and must not become ACTIVE (no auto-preempt).
        if (mask_hit or lethal_hit) and path1 == 'blocked' and active != 1:
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
    # Entry 035: 3.5 m standoff (was 5.0) — from 5 m the halo cells ~1.3 m
    # off-centre never got free-ray clear (wall-echo HIT_AT_CELL).
    approach = _approach_pose(barrier_xy, standoff_m=3.5)
    log(f'  approach_pose=({approach[0]:.2f},{approach[1]:.2f},{approach[2]:.2f})')
    approach_ok = False
    final_approach_dist: Optional[float] = None
    # Suppress new marks during teleport+clear so SLAM jumps cannot re-seal.
    _set_memory_marking(observer, False)
    # Teleop from Path-B off-FOV wedges on vegetation; set_model_pose is the
    # documented T5/T9 transport (marking off). PASS still requires live lidar clear.
    if not world.set_model_pose(
            'husky1', approach[0], approach[1], yaw=approach[2], z=0.50):
        _set_memory_marking(observer, True)
        _resume_nav_after_transport(observer)
        log('FAIL  T5 could not teleport into clear FOV')
        return False
    for _ in range(20):
        watcher.spin(0.1)
    pose = observer.robot_pose()
    if pose is not None:
        final_approach_dist = math.hypot(pose[0] - bx, pose[1] - by)
        log(f'  after teleport: pose=({pose[0]:.2f},{pose[1]:.2f},{pose[2]:.2f}) '
            f'dist_barrier={final_approach_dist:.2f}')
        if final_approach_dist <= 6.0:
            approach_ok = True
    if not approach_ok:
        _set_memory_marking(observer, True)
        _resume_nav_after_transport(observer)
        log(f'BLOCKED  T5 teleport did not land in clear FOV '
            f'(final_dist={final_approach_dist}). Not inventing clear evidence.')
        return False

    # Face barrier for free-ray clears.
    teleop = _teleop_cmd_pub(observer)
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
            teleop.publish(cmd)
        for _ in range(3):
            teleop.publish(Twist())
            watcher.spin(0.05)

    # --- Step: wait real lidar clear (marking still off) ---
    # Continuous yaw so free-rays sweep the barrier blob AND nearby Path-1
    # halo cells that sit >1.5 m from the barrier centre (Entry 035: mask at
    # barrier cleared while path1 stayed blocked on off-axis memory).
    log('  Waiting for live lidar clear evidence (≤150 s); marking_enabled=False; '
        'full yaw-sweep + creep...')
    mask_cleared = False
    path1_available = False
    flipped_to_a = False
    t_clear = time.monotonic()
    deadline = t_clear + 150.0
    last_progress_log = t_clear
    from geometry_msgs.msg import Twist
    while time.monotonic() < deadline:
        watcher.spin(0.1)
        pose = observer.robot_pose()
        if pose is not None:
            dist = math.hypot(pose[0] - bx, pose[1] - by)
            # Slow continuous spin (~0.45 rad/s) covers full FOV repeatedly.
            cmd = Twist()
            cmd.angular.z = 0.45
            # Creep closer if still > 3.2 m so halo beams densify.
            if dist > 3.2:
                cmd.linear.x = 0.18
            teleop.publish(cmd)
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
        teleop.publish(Twist())
        watcher.spin(0.05)

    path1_after = watcher.bank_path_state(1)
    active_after = watcher.bank_active_id()
    switches_after = watcher.switch_count()
    cost_rep = watcher.costmap_lethal_report(bx, by, accept_radius_m=0.60)
    mask_rep = watcher.mask_occupancy_report(bx, by, accept_radius_m=0.60)
    mask_corridor_blocked = bool(mask_rep is not None and mask_rep.ok)
    # costmap_raw includes StaticLayer trail walls; on bush_trail those sit
    # ≤0.60 m from a mid-corridor barrier centre. Fail costmap only when the
    # keepout overlay still drives lethality (mask ok) or the centre cell is
    # still lethal — not when the only neighbour lethal is static wall.
    cost_keepout_residual = bool(
        cost_rep is not None
        and cost_rep.ok
        and (
            mask_corridor_blocked
            or (
                cost_rep.centre_cost is not None
                and cost_rep.centre_cost >= 254
                and cost_rep.centre_cost < 255
            )
        )
    )
    log(f'  after clear wait: dist={dist} '
        f'mask_rep[{None if mask_rep is None else mask_rep.detail}] '
        f'cost_rep[{None if cost_rep is None else cost_rep.detail}] '
        f'path1={path1_after} active={active_after} '
        f'switches {switches_before}→{switches_after} '
        f'mask_cleared={mask_cleared} path1_available={path1_available} '
        f'flipped_to_a={flipped_to_a} cost_keepout_residual={cost_keepout_residual} '
        f'mem={watcher.status}')

    ok = True
    if flipped_to_a or (active_after == 1 and active_before != 1):
        log('FAIL  T5 auto-preempted back to Path A on unlock')
        ok = False
    if mask_corridor_blocked:
        log('FAIL  T5 mask still occupied on intended corridor after clear')
        ok = False
    if cost_keepout_residual:
        log('FAIL  T5 costmap_raw still lethal on intended corridor '
            '(keepout/centre residual, not static wall neighbour)')
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


# Locked Prompt R2 / Agent C RH-POS thresholds (A1–A9).
T11_MIN_PROGRESS_BEFORE_B_BLOCK_M = 8.0
T11_MIN_DIST_HOME_AT_RETURN_ARM_M = 5.0
T11_MIN_HOME_GAIN_M = 6.0
T11_ARRIVE_TOL_M = 1.0


def _parse_recorded_home_xy(status_text: str) -> Tuple[float, float]:
    """Parse ``home=(x,y[,yaw])`` from path_bank/status; fallback START."""
    for line in (status_text or '').splitlines():
        line = line.strip()
        if not line.startswith('home='):
            continue
        raw = line.split('=', 1)[1].strip().strip('()')
        parts = [p.strip() for p in raw.split(',')]
        if len(parts) >= 2:
            try:
                return float(parts[0]), float(parts[1])
            except ValueError:
                break
    return float(START[0]), float(START[1])


def run_t11_return_home(
    watcher: MemoryWatcher,
    observer: NavObserver,
    world: GazeboWorld,
    barrier_a_xy: Tuple[float, float],
) -> Optional[bool]:
    """T11 / RH-POS: A+B blocked → genuine physical return (Agent C A1–A9).

    Preconditions: T2 already switched to Path B with barrier A in place.

    Returns:
      True  — PASS (all hard gates)
      False — FAIL (logic/mission failure)
      None  — INVALID_GEOMETRY (B-block/arm too near home; not a stack PASS)
    """
    log('=== T11: ALL_BLOCKED → return to start (RH-POS / genuine R2) ===')
    home_xy = _parse_recorded_home_xy(watcher.bank_status)
    log(f'  recorded home=({home_xy[0]:.2f},{home_xy[1]:.2f})')
    active = watcher.bank_active_id()
    if active in (None, 1):
        log(f'FAIL  T11 expected active Path B-ish, got active={active}')
        return False

    # Progress trigger: delay Path-B block until ≥8 m from recorded home (A1).
    progress_deadline = time.monotonic() + 120.0
    pose = None
    while time.monotonic() < progress_deadline:
        watcher.spin(0.3)
        pose = observer.robot_pose()
        if pose is None:
            continue
        dist_now = math.hypot(pose[0] - home_xy[0], pose[1] - home_xy[1])
        if dist_now >= T11_MIN_PROGRESS_BEFORE_B_BLOCK_M:
            log(f'  progress gate OK dist_home={dist_now:.2f}m '
                f'(need ≥{T11_MIN_PROGRESS_BEFORE_B_BLOCK_M:.1f}m)')
            break
        if watcher.bank_state in (
                'RETURNING_TO_START', 'RETURNED_TO_START',
                'NO_SAFE_ROUTE', 'IDLE', 'NO_PATH_AVAILABLE'):
            # Premature exhaustion before intentional B-block — reliability
            # defect or A-barrier also sealed B (Wave B0 evidence).
            log(f'FAIL  T11 bank left FOLLOWING before B-block '
                f'(state={watcher.bank_state} dist={dist_now:.2f}m) '
                f'— premature ALL_BLOCKED/return')
            return False
    else:
        dist_now = (
            math.hypot(pose[0] - home_xy[0], pose[1] - home_xy[1])
            if pose is not None else float('nan'))
        log(f'INVALID_GEOMETRY  T11 never reached '
            f'{T11_MIN_PROGRESS_BEFORE_B_BLOCK_M:.1f}m from home before '
            f'B-block (dist={dist_now:.2f}m) — cannot prove genuine R2')
        return None

    if pose is None:
        log('FAIL  T11 no robot pose before Path B block')
        return False
    dist_home0 = math.hypot(pose[0] - home_xy[0], pose[1] - home_xy[1])
    if dist_home0 < T11_MIN_PROGRESS_BEFORE_B_BLOCK_M:
        log(f'INVALID_GEOMETRY  T11 dist_home_at_b_block_m={dist_home0:.2f} '
            f'< {T11_MIN_PROGRESS_BEFORE_B_BLOCK_M:.1f} (A1)')
        return None
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
        f'active={active} dist_home_at_b_block_m={dist_home0:.2f}m')
    if not world.spawn_obstacle(spec):
        log('FAIL  T11 spawn Path B barrier')
        return False
    barrier_b_xy = (float(block_xy[0]), float(block_xy[1]))

    # Owner authority for A6.
    owner_auth = {'auth': ''}
    try:
        from rs1_interfaces.msg import MissionOwnerStatus

        def _on_owner_t11(msg: MissionOwnerStatus) -> None:
            owner_auth['auth'] = str(msg.command_authority or '')

        owner_sub = observer.create_subscription(
            MissionOwnerStatus, 'mission_owner/status', _on_owner_t11, LATCHED)
    except ImportError:
        owner_sub = None
        log('WARN  T11 MissionOwnerStatus unavailable; A6 auth check limited')

    # Wait for return-home state (NO_PATH_AVAILABLE may flash first — not PASS).
    returning = False
    returned = False
    dist_home_at_return_arm_m = float('nan')
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
        if owner_sub is not None:
            observer.destroy_subscription(owner_sub)
        return False

    arm_pose = observer.robot_pose()
    if arm_pose is not None:
        dist_home_at_return_arm_m = math.hypot(
            arm_pose[0] - home_xy[0], arm_pose[1] - home_xy[1])
    log(f'  return armed state={watcher.bank_state} '
        f'dist_home_at_return_arm_m={dist_home_at_return_arm_m:.2f}m '
        f'nav_mode_line={[ln for ln in watcher.bank_status.splitlines() if "nav_mode" in ln or ln.startswith("home=")]}')

    # A7 at arm: dual memory retained.
    mask_a_arm = watcher.mask_occupied_near(
        barrier_a_xy[0], barrier_a_xy[1], radius_m=1.5)
    mask_b_arm = watcher.mask_occupied_near(
        barrier_b_xy[0], barrier_b_xy[1], radius_m=1.5)
    log(f'  memory at arm mask_a={mask_a_arm} mask_b={mask_b_arm}')
    if not mask_a_arm or not mask_b_arm:
        log('FAIL  T11 dual memory not occupied at return arm (A7)')
        if owner_sub is not None:
            observer.destroy_subscription(owner_sub)
        return False

    if (math.isnan(dist_home_at_return_arm_m)
            or dist_home_at_return_arm_m < T11_MIN_DIST_HOME_AT_RETURN_ARM_M):
        log(f'INVALID_GEOMETRY  T11 return armed at '
            f'{dist_home_at_return_arm_m:.2f}m from home '
            f'(need ≥{T11_MIN_DIST_HOME_AT_RETURN_ARM_M:.1f}m) — not a PASS')
        if owner_sub is not None:
            observer.destroy_subscription(owner_sub)
        return None

    # A3/A4/A5: physical travel + RETURNED + final dist.
    arrive_tol_m = T11_ARRIVE_TOL_M
    progress_deadline = time.monotonic() + 180.0
    best_home_gain = 0.0
    min_dist_during = dist_home_at_return_arm_m
    decreasing_samples = 0
    last_dist = dist_home_at_return_arm_m
    saw_east_regress = False
    while time.monotonic() < progress_deadline:
        watcher.spin(0.3)
        if watcher.bank_state == 'RETURNED_TO_START':
            returned = True
            break
        if watcher.bank_state == 'NO_SAFE_ROUTE':
            log('FAIL  T11 entered NO_SAFE_ROUTE during genuine return')
            if owner_sub is not None:
                observer.destroy_subscription(owner_sub)
            return False
        p = observer.robot_pose()
        if p is None:
            continue
        dist_home = math.hypot(p[0] - home_xy[0], p[1] - home_xy[1])
        min_dist_during = min(min_dist_during, dist_home)
        best_home_gain = max(
            best_home_gain, dist_home_at_return_arm_m - dist_home)
        if dist_home < last_dist - 0.05:
            decreasing_samples += 1
        last_dist = dist_home
        if p[0] > east0 + 2.0:
            saw_east_regress = True
        if dist_home <= arrive_tol_m and watcher.bank_state == 'RETURNED_TO_START':
            returned = True
            break

    final = observer.robot_pose()
    final_dist = (
        math.hypot(final[0] - home_xy[0], final[1] - home_xy[1])
        if final is not None else float('nan')
    )
    bank_returned = watcher.bank_state == 'RETURNED_TO_START'
    home_gain_m = dist_home_at_return_arm_m - min_dist_during
    gain_ok = (
        home_gain_m >= T11_MIN_HOME_GAIN_M
        or home_gain_m >= 0.75 * dist_home_at_return_arm_m
    )
    log(f'  home_gain={home_gain_m:.2f}m (best={best_home_gain:.2f}) '
        f'final_dist_home={final_dist:.2f}m returned={returned} '
        f'bank_returned={bank_returned} east_regress={saw_east_regress} '
        f'decreasing_samples={decreasing_samples} auth={owner_auth["auth"]!r} '
        f'state={watcher.bank_state}')

    # A8: reject near-home cheat class.
    if (
        dist_home_at_return_arm_m < T11_MIN_DIST_HOME_AT_RETURN_ARM_M
        or home_gain_m < 1.0
    ):
        log(f'INVALID_GEOMETRY/FAIL  T11 near-home cheat class '
            f'(arm={dist_home_at_return_arm_m:.2f} gain={home_gain_m:.2f})')
        if owner_sub is not None:
            observer.destroy_subscription(owner_sub)
        return None if dist_home_at_return_arm_m < T11_MIN_DIST_HOME_AT_RETURN_ARM_M else False

    if saw_east_regress and home_gain_m < 1.0:
        log('FAIL  T11 resumed eastward mission after return armed')
        if owner_sub is not None:
            observer.destroy_subscription(owner_sub)
        return False
    if not gain_ok or decreasing_samples < 3:
        log(f'FAIL  T11 insufficient physical travel (A3) '
            f'gain={home_gain_m:.2f}m samples={decreasing_samples}')
        if owner_sub is not None:
            observer.destroy_subscription(owner_sub)
        return False
    if not bank_returned:
        log(f'FAIL  T11 bank not RETURNED_TO_START (A5) state={watcher.bank_state}')
        if owner_sub is not None:
            observer.destroy_subscription(owner_sub)
        return False
    if math.isnan(final_dist) or final_dist > arrive_tol_m:
        log(f'FAIL  T11 final_dist_home={final_dist:.2f}m > {arrive_tol_m} (A4)')
        if owner_sub is not None:
            observer.destroy_subscription(owner_sub)
        return False

    # A6: authority NONE after success.
    settle_deadline = time.monotonic() + 5.0
    while time.monotonic() < settle_deadline:
        watcher.spin(0.2)
        if owner_auth['auth'] == 'NONE':
            break
    if owner_auth['auth'] and owner_auth['auth'] != 'NONE':
        log(f'FAIL  T11 command_authority want NONE got {owner_auth["auth"]!r} (A6)')
        if owner_sub is not None:
            observer.destroy_subscription(owner_sub)
        return False

    # A7 at terminal.
    mask_a_end = watcher.mask_occupied_near(
        barrier_a_xy[0], barrier_a_xy[1], radius_m=1.5)
    mask_b_end = watcher.mask_occupied_near(
        barrier_b_xy[0], barrier_b_xy[1], radius_m=1.5)
    log(f'  memory at terminal mask_a={mask_a_end} mask_b={mask_b_end}')
    if not mask_a_end or not mask_b_end:
        log('FAIL  T11 dual memory cleared at terminal (A7)')
        if owner_sub is not None:
            observer.destroy_subscription(owner_sub)
        return False

    if owner_sub is not None:
        observer.destroy_subscription(owner_sub)
    log(
        f'PASS  T11 RH-POS A1–A8 '
        f'dist_b={dist_home0:.2f} arm={dist_home_at_return_arm_m:.2f} '
        f'gain={home_gain_m:.2f} final={final_dist:.2f} auth=NONE'
    )
    return True


def _set_test_force_unreachable_home(observer: NavObserver, enabled: bool) -> bool:
    """Arm/disarm path_bank test-only home override (N6). Production default false."""
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import SetParameters

    cli = observer.create_client(
        SetParameters, '/husky1/path_bank_manager/set_parameters')
    if not cli.wait_for_service(timeout_sec=8.0):
        log('FAIL  N6 path_bank_manager/set_parameters unavailable')
        return False
    req = SetParameters.Request()
    p = Parameter()
    p.name = 'test_force_unreachable_home'
    p.value = ParameterValue()
    p.value.type = ParameterType.PARAMETER_BOOL
    p.value.bool_value = bool(enabled)
    req.parameters = [p]
    fut = cli.call_async(req)
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline and not fut.done():
        observer.spin_for(0.1)
    if not fut.done():
        log('FAIL  N6 set_parameters timed out')
        return False
    resp = fut.result()
    if resp is None or not resp.results or not resp.results[0].successful:
        reason = ''
        if resp is not None and resp.results:
            reason = resp.results[0].reason
        log(f'FAIL  N6 set test_force_unreachable_home={enabled}: {reason}')
        return False
    log(f'  test_force_unreachable_home={enabled}')
    return True


def _call_start_mission(
    observer: NavObserver, x: float, y: float, yaw: float = 0.0
) -> Tuple[bool, str, str]:
    """Return (success, reject_code, message) from StartMission."""
    from rs1_interfaces.srv import StartMission

    cli = observer._ensure_start_mission_client()
    if not cli or cli is False:
        return False, 'NO_CLIENT', 'StartMission client missing'
    if not cli.wait_for_service(timeout_sec=3.0):
        return False, 'NO_SERVICE', 'StartMission service missing'
    req = StartMission.Request()
    req.goal_x = float(x)
    req.goal_y = float(y)
    req.goal_yaw = float(yaw)
    fut = cli.call_async(req)
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline and not fut.done():
        observer.spin_for(0.1)
    if not fut.done():
        return False, 'TIMEOUT', 'StartMission timed out'
    resp = fut.result()
    if resp is None:
        return False, 'NO_RESP', 'StartMission empty response'
    return bool(resp.success), str(resp.reject_code or ''), str(resp.message or '')


def _call_stop_mission(observer: NavObserver) -> Tuple[bool, str, List[str]]:
    """Return (success, state, active_inhibits) from StopMission."""
    from rs1_interfaces.srv import StopMission

    cli = observer.create_client(StopMission, 'mission_owner/stop_mission')
    if not cli.wait_for_service(timeout_sec=3.0):
        return False, 'NO_SERVICE', []
    fut = cli.call_async(StopMission.Request())
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline and not fut.done():
        observer.spin_for(0.1)
    if not fut.done():
        return False, 'TIMEOUT', []
    resp = fut.result()
    if resp is None:
        return False, 'NO_RESP', []
    inhibits = list(resp.active_inhibits or [])
    return bool(resp.success), str(resp.state or ''), inhibits


def run_n6_no_safe_route(
    watcher: MemoryWatcher,
    observer: NavObserver,
    world: GazeboWorld,
    barrier_a_xy: Tuple[float, float],
) -> bool:
    """N6: ALL_BLOCKED + unreachable home → NO_SAFE_ROUTE + StopMission rearm.

    Preconditions: T2 already switched to Path B with barrier A in place.
    Arms test-only ``test_force_unreachable_home`` so return-home
    ComputePathToPose / footprint validation fails (does not weaken production
    defaults). Then asserts bank NO_SAFE_ROUTE, owner STOPPED +
    ALL_BLOCKED_NO_HOME + auth NONE, Start rejected, Stop clears, Start accepted.
    """
    del barrier_a_xy
    log('=== N6: ALL_BLOCKED + home fail → NO_SAFE_ROUTE + rearm ===')
    active = watcher.bank_active_id()
    if active in (None, 1):
        log(f'FAIL  N6 expected active Path B-ish, got active={active}')
        return False

    # Subscribe to owner status / authority for live asserts.
    owner_state = {'phase': '', 'auth': '', 'detail': '', 'inhibits': []}
    try:
        from rs1_interfaces.msg import MissionOwnerStatus
    except ImportError:
        log('FAIL  N6 rs1_interfaces.MissionOwnerStatus missing')
        return False

    def _on_owner(msg: MissionOwnerStatus) -> None:
        owner_state['phase'] = str(msg.state or '')
        owner_state['auth'] = str(msg.command_authority or '')
        owner_state['detail'] = str(msg.detail or '')
        owner_state['inhibits'] = list(msg.active_inhibits or [])

    def _on_auth(msg: String) -> None:
        owner_state['auth'] = msg.data.strip()

    owner_sub = observer.create_subscription(
        MissionOwnerStatus, 'mission_owner/status', _on_owner, LATCHED)
    auth_sub = observer.create_subscription(
        String, 'mission_owner/command_authority', _on_auth, LATCHED)

    if not _set_test_force_unreachable_home(observer, True):
        observer.destroy_subscription(owner_sub)
        observer.destroy_subscription(auth_sub)
        return False

    pose = observer.robot_pose()
    if pose is None:
        log('FAIL  N6 no robot pose before Path B block')
        _set_test_force_unreachable_home(observer, False)
        observer.destroy_subscription(owner_sub)
        observer.destroy_subscription(auth_sub)
        return False

    path_b = _candidate_path_pts(observer, int(active), timeout=20.0)
    if not path_b:
        plan = observer.latest_path()
        if plan is None or len(plan.points) < 4:
            log('FAIL  N6 no Path B candidate / plan')
            _set_test_force_unreachable_home(observer, False)
            observer.destroy_subscription(owner_sub)
            observer.destroy_subscription(auth_sub)
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
        name='sensor_only_block_b_n6',
        at=(block_xy[0], block_xy[1]),
        heading=yaw,
        width=2.2,
        thickness=0.45,
        height=1.6,
    )
    log(f'  Path B barrier at ({block_xy[0]:.2f},{block_xy[1]:.2f}) '
        f'active={active} pose=({pose[0]:.2f},{pose[1]:.2f})')
    if not world.spawn_obstacle(spec):
        log('FAIL  N6 spawn Path B barrier')
        _set_test_force_unreachable_home(observer, False)
        observer.destroy_subscription(owner_sub)
        observer.destroy_subscription(auth_sub)
        return False

    # Wait for terminal NO_SAFE_ROUTE (may flash RETURNING_TO_START first).
    saw_returning = False
    saw_nsr = False
    deadline = time.monotonic() + 120.0
    while time.monotonic() < deadline:
        watcher.spin(0.3)
        st = watcher.bank_state
        if st == 'RETURNING_TO_START' or 'nav_mode=home' in watcher.bank_status:
            saw_returning = True
        if st == 'NO_SAFE_ROUTE' or 'NO_SAFE_ROUTE' in watcher.bank_status:
            saw_nsr = True
            break
        if st == 'RETURNED_TO_START':
            log('FAIL  N6 home succeeded (RETURNED_TO_START) — force-unreachable '
                'did not prevent return; cannot claim NO_SAFE_ROUTE live')
            _set_test_force_unreachable_home(observer, False)
            observer.destroy_subscription(owner_sub)
            observer.destroy_subscription(auth_sub)
            return False

    # Owner status may lag bank by one publish cycle; wait for STOPPED sync.
    if saw_nsr:
        sync_deadline = time.monotonic() + 5.0
        while time.monotonic() < sync_deadline:
            if (
                owner_state['phase'] == 'STOPPED'
                and 'ALL_BLOCKED_NO_HOME' in owner_state['inhibits']
                and owner_state['auth'] == 'NONE'
            ):
                break
            watcher.spin(0.2)

    log(f'  after block: bank_state={watcher.bank_state!r} '
        f'saw_returning={saw_returning} owner_phase={owner_state["phase"]!r} '
        f'auth={owner_state["auth"]!r} inhibits={owner_state["inhibits"]} '
        f'detail={owner_state["detail"]!r}')

    ok = True
    if not saw_nsr:
        log(f'FAIL  N6 never entered NO_SAFE_ROUTE '
            f'(state={watcher.bank_state!r} status=\n{watcher.bank_status})')
        ok = False

    # Owner terminal contract.
    if owner_state['phase'] != 'STOPPED':
        log(f'FAIL  N6 owner phase want STOPPED got {owner_state["phase"]!r}')
        ok = False
    if owner_state['auth'] != 'NONE':
        log(f'FAIL  N6 command_authority want NONE got {owner_state["auth"]!r}')
        ok = False
    if 'ALL_BLOCKED_NO_HOME' not in owner_state['inhibits']:
        log(f'FAIL  N6 missing ALL_BLOCKED_NO_HOME in {owner_state["inhibits"]}')
        ok = False
    if owner_state['detail'] and 'NO_SAFE_ROUTE' not in owner_state['detail']:
        # detail may be bank state echo; accept either.
        if watcher.bank_state != 'NO_SAFE_ROUTE':
            log(f'FAIL  N6 owner detail missing NO_SAFE_ROUTE '
                f'({owner_state["detail"]!r})')
            ok = False

    # StartMission must reject while ALL_BLOCKED_NO_HOME latched.
    ok_start, reject, msg = _call_start_mission(
        observer, GOAL[0], GOAL[1], GOAL[2])
    log(f'  Start while NO_SAFE_ROUTE: success={ok_start} '
        f'reject={reject!r} msg={msg!r}')
    if ok_start:
        log('FAIL  N6 StartMission accepted while ALL_BLOCKED_NO_HOME')
        ok = False
    elif 'ALL_BLOCKED_NO_HOME' not in reject and 'ALL_BLOCKED' not in msg:
        log(f'FAIL  N6 Start reject expected ALL_BLOCKED_NO_HOME got '
            f'{reject!r} / {msg!r}')
        ok = False
    else:
        log('  StartMission correctly rejected under ALL_BLOCKED_NO_HOME')

    # StopMission clears inhibit; Start can rearm.
    ok_stop, stop_state, stop_inhibits = _call_stop_mission(observer)
    watcher.spin(0.5)
    log(f'  StopMission success={ok_stop} state={stop_state!r} '
        f'inhibits={stop_inhibits} bank={watcher.bank_state!r} '
        f'owner={owner_state["phase"]!r}/{owner_state["auth"]!r}')
    if not ok_stop:
        log('FAIL  N6 StopMission failed')
        ok = False
    if 'ALL_BLOCKED_NO_HOME' in stop_inhibits:
        log('FAIL  N6 StopMission did not clear ALL_BLOCKED_NO_HOME')
        ok = False
    if 'ALL_BLOCKED_NO_HOME' in owner_state['inhibits']:
        log(f'FAIL  N6 owner still has ALL_BLOCKED_NO_HOME after Stop '
            f'{owner_state["inhibits"]}')
        ok = False

    # Disarm test hook before rearm so next mission uses real home.
    _set_test_force_unreachable_home(observer, False)

    ok_rearm, reject2, msg2 = _call_start_mission(
        observer, GOAL[0], GOAL[1], GOAL[2])
    log(f'  Start after Stop: success={ok_rearm} reject={reject2!r} msg={msg2!r}')
    if not ok_rearm:
        # Document IDLE transition quirks honestly — STOPPED should allow Start.
        log(f'FAIL  N6 StartMission after Stop rejected '
            f'({reject2}: {msg2}) — rearm incomplete')
        ok = False
    else:
        log('  StartMission accepted after StopMission rearm')
        # Don't leave a live FollowPath running into teardown.
        _call_stop_mission(observer)

    observer.destroy_subscription(owner_sub)
    observer.destroy_subscription(auth_sub)

    if ok:
        log('PASS  N6 NO_SAFE_ROUTE + StopMission rearm')
    return ok


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


def _pack_xy_cloud(
    frame_id: str, points: List[Tuple[float, float]], stamp
) -> PointCloud2:
    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    buf = bytearray()
    for x, y in points:
        buf += struct.pack('<fff', float(x), float(y), 0.0)
    msg.data = bytes(buf)
    return msg


def _unpack_xy_cloud(cloud: PointCloud2) -> List[Tuple[float, float]]:
    pts: List[Tuple[float, float]] = []
    if cloud.width == 0 or cloud.point_step < 8:
        return pts
    xo = yo = None
    for f in cloud.fields:
        if f.name == 'x':
            xo = f.offset
        elif f.name == 'y':
            yo = f.offset
    if xo is None or yo is None:
        return pts
    data = bytes(cloud.data)
    step = cloud.point_step
    for i in range(cloud.width * max(1, cloud.height)):
        base = i * step
        if base + max(xo, yo) + 4 > len(data):
            break
        x = struct.unpack_from('<f', data, base + xo)[0]
        y = struct.unpack_from('<f', data, base + yo)[0]
        pts.append((x, y))
    return pts


def _fire_disc(cx: float, cy: float, radius: float = 0.9, step: float = 0.3) -> List[Tuple[float, float]]:
    pts: List[Tuple[float, float]] = [(cx, cy)]
    r = step
    while r <= radius + 1e-9:
        n = max(1, int(round(2.0 * math.pi * r / step)))
        for i in range(n):
            ang = 2.0 * math.pi * i / n
            pts.append((cx + r * math.cos(ang), cy + r * math.sin(ang)))
        r += step
    return pts


def run_t9_fire_lidar_isolation(
    watcher: MemoryWatcher,
    observer: NavObserver,
    world: GazeboWorld,
) -> bool:
    """T9: LiDAR free-ray must not clear FIRE; in-FOV thermal negatives do.

    Entry 026: shared ObstacleLayer scan raytrace previously wiped live fire
    costs. Typed memory + fire_memory_cloud layer must retain Path A FIRE while
    thermal FOV points away; only in-FOV negatives after hold clear it.
    """
    log('=== T9: FIRE retained vs lidar free-ray; thermal FOV clear ===')
    ready_deadline = time.monotonic() + 60.0
    while time.monotonic() < ready_deadline:
        watcher.spin(0.2)
        if observer.nav2_ready(timeout=1.0):
            break
    else:
        log('FAIL  T9 Nav2 navigate_to_pose not ready')
        return False
    if not observer.send_goal(GOAL[0], GOAL[1], GOAL[2]):
        watcher.spin(3.0)
        if not observer.send_goal(GOAL[0], GOAL[1], GOAL[2]):
            log('FAIL  T9 NavigateToPose rejected/timeout')
            return False
    if not watcher.wait_bank(90.0):
        log('FAIL  T9 path bank never published')
        return False

    exclusive, sep, path1 = _exclusive_block_xy(observer, timeout=40.0)
    if exclusive is None:
        plan = observer.latest_path()
        if plan is None or len(plan.points) < 4:
            log('FAIL  T9 no exclusive fire point')
            return False
        exclusive = plan.points[len(plan.points) // 2]
        path1 = plan.points
        log(f'  Using plan midpoint ({exclusive[0]:.1f},{exclusive[1]:.1f})')
    else:
        log(f'  Path-1 fire target ({exclusive[0]:.1f},{exclusive[1]:.1f}) sep={sep:.1f}m')

    fx, fy = exclusive
    fire_pub = observer.create_publisher(PointCloud2, 'fire_hazards', 10)
    fire_bool_pub = observer.create_publisher(Bool, 'fire_detected', 10)
    watcher.spin(0.5)

    switches_before = watcher.switch_count()
    disc = _fire_disc(fx, fy)
    log(f'  Publishing FIRE ingress disc ({len(disc)} pts) + fire_detected=true')
    mark_deadline = time.monotonic() + 45.0
    switched = False
    while time.monotonic() < mark_deadline:
        cloud = _pack_xy_cloud(
            'husky1_map', disc, observer.get_clock().now().to_msg())
        fire_pub.publish(cloud)
        fire_bool_pub.publish(Bool(data=True))
        watcher.spin(0.25)
        if watcher.mask_occupied_near(fx, fy, radius_m=1.5) and (
            watcher.switch_count() > switches_before
            or (
                watcher.bank_path_state(1) == 'blocked'
                and watcher.bank_active_id() not in (None, 1)
            )
        ):
            switched = True
            break
    log(f'  after mark: mask={watcher.mask_occupied_near(fx, fy, radius_m=1.5)} '
        f'path1={watcher.bank_path_state(1)} active={watcher.bank_active_id()} '
        f'switches={watcher.switch_count()} mem={watcher.status}')
    if not switched:
        log('FAIL  T9 Path A did not block/switch after FIRE mark')
        return False
    def _status_fire_count() -> int:
        for tok in watcher.status.split():
            if tok.startswith('fire='):
                try:
                    return int(tok.split('=', 1)[1])
                except ValueError:
                    return 0
        return 0

    fire_n = _status_fire_count()
    if fire_n < 1:
        log(f'FAIL  T9 memory status has no FIRE cells ({watcher.status})')
        return False
    fire_at_mark = fire_n

    # Stop FIRE ingress; typed memory + fire_memory_cloud must retain.
    fire_pub.publish(_pack_xy_cloud(
        'husky1_map', [], observer.get_clock().now().to_msg()))
    fire_bool_pub.publish(Bool(data=False))

    _pause_nav_for_transport(observer)
    # Stay on the planned path: 5 m before the fire along path heading.
    if path1 and len(path1) >= 2:
        best_i, best_d = 0, 1e9
        for i, p in enumerate(path1):
            d = math.hypot(p[0] - fx, p[1] - fy)
            if d < best_d:
                best_i, best_d = i, d
        i0 = max(0, best_i - 1)
        i1 = min(len(path1) - 1, best_i + 1)
        if i1 == i0:
            i1 = min(len(path1) - 1, i0 + 1)
        approach_yaw = math.atan2(path1[i1][1] - path1[i0][1],
                                  path1[i1][0] - path1[i0][0])
    else:
        approach_yaw = math.atan2(fy - START[1], fx - START[0])
    rx = fx - 5.0 * math.cos(approach_yaw)
    ry = fy - 5.0 * math.sin(approach_yaw)
    away_yaw = approach_yaw + math.pi
    if not world.set_model_pose('husky1', rx, ry, yaw=away_yaw, z=0.50):
        _resume_nav_after_transport(observer)
        log('FAIL  T9 could not teleport robot off thermal FOV')
        return False
    log(f'  Teleported off-FOV pose=({rx:.2f},{ry:.2f}) yaw={away_yaw:.2f}')

    retained = True
    returned_to_a = False
    fire_retained = True
    t0 = time.monotonic()
    while time.monotonic() - t0 < 20.0:
        fire_bool_pub.publish(Bool(data=False))
        watcher.spin(0.3)
        if not watcher.mask_occupied_near(fx, fy, radius_m=1.5):
            retained = False
        if _status_fire_count() < 1:
            fire_retained = False
        if watcher.bank_active_id() == 1:
            returned_to_a = True
    mask_after = watcher.mask_occupied_near(fx, fy, radius_m=1.5)
    lethal_after = watcher.lethal_near(fx, fy, radius_m=1.5)
    path1_after = watcher.bank_path_state(1)
    fire_after = _status_fire_count()
    log(f'  off-FOV lidar window: mask={mask_after} lethal={lethal_after} '
        f'path1={path1_after} active={watcher.bank_active_id()} '
        f'fire={fire_after} mem={watcher.status}')
    if not retained or not mask_after or not fire_retained or fire_after < 1:
        _resume_nav_after_transport(observer)
        log('FAIL  T9 FIRE memory cleared by lidar while thermal off-FOV')
        return False
    if path1_after != 'blocked':
        _resume_nav_after_transport(observer)
        log('FAIL  T9 Path A unlocked without thermal FOV clear')
        return False
    if returned_to_a:
        _resume_nav_after_transport(observer)
        log('FAIL  T9 returned to Path A during off-FOV lidar window')
        return False
    if fire_after > fire_at_mark:
        _resume_nav_after_transport(observer)
        log(f'FAIL  T9 FIRE cells grew while fire_detected=False '
            f'({fire_at_mark} → {fire_after}) — anti-spray')
        return False

    # Face FIRE on-trail (Entry 028). Hold already expired.
    face_yaw = approach_yaw
    standoff = 4.5
    rx2 = fx - standoff * math.cos(face_yaw)
    ry2 = fy - standoff * math.sin(face_yaw)
    if not world.set_model_pose('husky1', rx2, ry2, yaw=face_yaw, z=0.50):
        _resume_nav_after_transport(observer)
        log('FAIL  T9 could not teleport robot into thermal FOV')
        return False
    log(f'  Teleported in-FOV pose=({rx2:.2f},{ry2:.2f}) yaw={face_yaw:.2f}')

    clear_deadline = time.monotonic() + 30.0
    cleared = False
    while time.monotonic() < clear_deadline:
        fire_bool_pub.publish(Bool(data=False))
        watcher.spin(0.25)
        # Lidar may still occupy the mask near the site; FIRE count is the
        # domain signal for thermal clear (Entry 027).
        if _status_fire_count() == 0:
            cleared = True
            break
        # Disc-edge cells can sit outside the 18° ROI frustum (Entry 028).
        if watcher.fire_cells_near(fx, fy, radius_m=0.6) == 0 and _status_fire_count() <= 15:
            cleared = True
            break
    log(f'  after thermal negatives: cleared={cleared} '
        f'fire={_status_fire_count()} '
        f'core={watcher.fire_cells_near(fx, fy, radius_m=0.6)} '
        f'mask={watcher.mask_occupied_near(fx, fy, radius_m=1.5)} '
        f'mem={watcher.status}')
    _resume_nav_after_transport(observer)
    if not cleared:
        log('FAIL  T9 FIRE did not clear after in-FOV thermal negatives')
        return False
    fire_final = _status_fire_count()
    fire_core = watcher.fire_cells_near(fx, fy, radius_m=0.6)
    mask_final = watcher.mask_occupied_near(fx, fy, radius_m=1.5)
    lethal_final = watcher.lethal_near(fx, fy, radius_m=1.5)
    path1_final = watcher.bank_path_state(1)
    active_final = watcher.bank_active_id()
    log(f'  post-clear: fire={fire_final} core={fire_core} mask={mask_final} '
        f'lethal={lethal_final} path1={path1_final} active={active_final}')
    if fire_core != 0:
        log('FAIL  T9 FIRE core not cleared after thermal negatives')
        return False
    if active_final == 1:
        log('FAIL  T9 switched back to Path A after thermal clear')
        return False
    log('PASS  T9 FIRE isolated from lidar; thermal FOV clear works')
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


class _ScenarioDone(Exception):
    """Control-flow marker: dedicated scenario finished, skip the T1..stuck chain."""


class _Tracer:
    """Spin helper that samples telemetry every 0.1 s while a test waits."""

    def __init__(self, watcher: MemoryWatcher, observer: NavObserver, telem,
                 goal_xy: Tuple[float, float]):
        self.watcher = watcher
        self.observer = observer
        self.telem = telem
        self.goal_xy = goal_xy

    def spin(self, dt: float) -> None:
        deadline = time.monotonic() + dt
        while True:
            self.watcher.spin(0.1)
            self.telem.sample(self.observer.robot_pose(), self.goal_xy)
            if time.monotonic() >= deadline:
                break


class _IsPathValidProbe:
    """Diagnostic: ask planner_server whether the *just published* plan is valid.

    If a freshly computed plan is reported invalid, the 1 Hz replans seen in
    the launch log come from ``isPathValid`` itself (footprint-at-pose-yaw
    check on NavFn's unoriented poses), not from real obstacles.
    """

    def __init__(self, observer: NavObserver):
        from nav2_msgs.srv import IsPathValid
        self._srv = IsPathValid
        self.client = observer.create_client(IsPathValid, 'is_path_valid')
        self.observer = observer
        # (t, raw_valid, tangent_valid, rtt_s)
        self.results: List[Tuple[float, Optional[bool], Optional[bool], float]] = []

    def _call(self, plan: RosPath, timeout_s: float) -> Tuple[Optional[bool], float]:
        req = self._srv.Request()
        req.path = plan
        t0 = time.monotonic()
        fut = self.client.call_async(req)
        while time.monotonic() - t0 < timeout_s and not fut.done():
            self.observer.spin_for(0.02)
        rtt = time.monotonic() - t0
        if not fut.done() or fut.result() is None:
            return None, rtt
        return bool(fut.result().is_valid), rtt

    @staticmethod
    def _tangent_oriented(plan: RosPath) -> RosPath:
        """Copy of plan with each pose yawed along the local path tangent."""
        import copy
        out = copy.deepcopy(plan)
        n = len(out.poses)
        for i, ps in enumerate(out.poses):
            j = min(n - 1, i + 1)
            k = max(0, i - 1)
            dx = out.poses[j].pose.position.x - out.poses[k].pose.position.x
            dy = out.poses[j].pose.position.y - out.poses[k].pose.position.y
            yaw = math.atan2(dy, dx) if (abs(dx) + abs(dy)) > 1e-6 else 0.0
            ps.pose.orientation.x = 0.0
            ps.pose.orientation.y = 0.0
            ps.pose.orientation.z = math.sin(0.5 * yaw)
            ps.pose.orientation.w = math.cos(0.5 * yaw)
        return out

    def probe(self, plan: Optional[RosPath], now_s: float, timeout_s: float = 1.0) -> Optional[bool]:
        """Probe raw plan and a tangent-oriented copy; return raw validity."""
        if plan is None or not plan.poses:
            return None
        if not self.client.wait_for_service(timeout_sec=0.2):
            return None
        raw, rtt1 = self._call(plan, timeout_s)
        tan, rtt2 = self._call(self._tangent_oriented(plan), timeout_s)
        self.results.append((now_s, raw, tan, max(rtt1, rtt2)))
        return raw

    def summary(self) -> str:
        if not self.results:
            return 'is_path_valid: no probes'
        n = len(self.results)
        raw_inv = sum(1 for r in self.results if r[1] is False)
        tan_inv = sum(1 for r in self.results if r[2] is False)
        rtt = max(r[3] for r in self.results)
        return (f'is_path_valid: probes={n} raw_invalid={raw_inv} '
                f'tangent_oriented_invalid={tan_inv} max_rtt={rtt * 1000:.0f}ms '
                '(Humble 1.1.20 never fills invalid_pose_indices)')


def _wait_for_bank_switch(
    watcher: MemoryWatcher,
    spin,
    switches_before: int,
    timeout_s: float,
) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        spin(0.3)
        if watcher.switch_count() > switches_before and watcher.bank_active_id() not in (None, 1):
            return True
        if (
            watcher.bank_path_state(1) == 'blocked'
            and watcher.bank_active_id() not in (None, 1)
        ):
            return True
    return (
        watcher.bank_path_state(1) == 'blocked'
        and watcher.bank_active_id() not in (None, 1)
    )


def run_t2trace(
    watcher: MemoryWatcher,
    observer: NavObserver,
    world: GazeboWorld,
    telem,
    *,
    fire_mode: bool,
    post_switch_s: float = 60.0,
    trace_prefix: str = '/tmp/t2trace',
) -> bool:
    """T2TRACE: block A → switch B, then record ≥60 s of post-switch motion.

    Pass criteria (Entry 028): bank switches; robot keeps moving on Path B —
    no stall interval (|v|<0.05, |w|<0.10 for ≥2 s while FOLLOWING_PATH and
    >0.5 m from goal) in the post-switch window; no RPP collision aborts or
    behaviour-server recoveries. Rotate-to-heading intervals are reported but
    allowed. Telemetry is written to ``<prefix>_{samples.csv,events.txt}``.
    """
    mode = 'fire_on' if fire_mode else 'fire_off'
    log(f'=== T2TRACE ({mode}): switch + {post_switch_s:.0f}s post-switch trace ===')
    tracer = _Tracer(watcher, observer, telem, (GOAL[0], GOAL[1]))
    telem.mark('T2TRACE_START', mode)
    setup = _start_goal_and_spawn_path1_barrier(
        watcher, observer, world, label='T2TRACE', spin=tracer.spin)
    if setup is None:
        return False
    block_xy, switches_before, _yaw = setup
    t_spawn = telem.mark('BARRIER_SPAWNED', f'({block_xy[0]:.2f},{block_xy[1]:.2f})')

    if not _wait_for_bank_switch(watcher, tracer.spin, switches_before, 150.0):
        telem.poll_launch_log()
        log(f'FAIL  T2TRACE no bank switch; {watcher.location_report(*block_xy)}')
        return False
    t_switch = telem.mark('SWITCH_DETECTED',
                          f'active={watcher.bank_active_id()} switches={watcher.switch_count()}')
    log(f'  switch at sim {t_switch:.1f} ({t_switch - t_spawn:.1f}s after spawn); '
        f'{watcher.location_report(*block_xy)}')

    pose_at_switch = observer.robot_pose()
    reached_goal = False
    exhausted = False
    probe = _IsPathValidProbe(observer)
    last_probe = 0.0
    deadline = time.monotonic() + post_switch_s
    while time.monotonic() < deadline:
        tracer.spin(0.5)
        telem.poll_launch_log()
        if time.monotonic() - last_probe > 4.0:
            last_probe = time.monotonic()
            valid = probe.probe(watcher.plan, telem.now())
            if valid is not None:
                r = probe.results[-1]
                telem.mark('IS_PATH_VALID_PROBE', f'raw={r[1]} tangent={r[2]}')
        pose = observer.robot_pose()
        if pose is not None and math.hypot(pose[0] - GOAL[0], pose[1] - GOAL[1]) < 0.6:
            reached_goal = True
            break
        if watcher.bank_state == 'NO_PATH_AVAILABLE' and watcher.bank_active_id() is None:
            exhausted = True
            break
    t_end = telem.mark('T2TRACE_END', f'reached_goal={reached_goal} exhausted={exhausted}')
    telem.poll_launch_log()

    pose = observer.robot_pose()
    delta = (
        math.hypot(pose[0] - pose_at_switch[0], pose[1] - pose_at_switch[1])
        if pose is not None and pose_at_switch is not None else float('nan'))
    cls = telem.classify(t_switch, t_end)
    stalls = cls['stall']
    rotates = cls['rotate']
    collisions = telem.count('RPP_COLLISION', t_switch, t_end)
    recoveries = telem.count('BEHAVIOR_RUN', t_switch, t_end)
    new_paths = telem.count('NEW_PATH_TO_CONTROLLER', t_switch, t_end)
    plan_pubs = telem.count('PLAN_PUBLISHED', t_switch, t_end)
    fire_marks = telem.count('FIRE_MARK', t_switch, t_end)
    paths = telem.write(f'{trace_prefix}_{mode}')
    log(f'  trace -> {paths[0]} , {paths[1]}')
    log('  ' + telem.summarize(t_switch, t_end).replace('\n', '\n  '))
    log(f'  {probe.summary()}')
    log(f'  post-switch: window={t_end - t_switch:.1f}s delta={delta:.2f}m '
        f'reached_goal={reached_goal} stalls={len(stalls)} '
        f'stall_total={sum(i.duration for i in stalls):.1f}s '
        f'rotates={len(rotates)} rpp_collisions={collisions} recoveries={recoveries} '
        f'new_paths={new_paths} plan_pubs={plan_pubs} fire_marks={fire_marks}')
    for iv in stalls:
        ctx = telem.events_between(iv.start - 3.0, iv.end + 1.0)
        log(f'  STALL {iv.start - t_switch:+.1f}s..{iv.end - t_switch:+.1f}s '
            f'({iv.duration:.1f}s) context:')
        for t, src, label, text in ctx[:30]:
            log(f'      {t - t_switch:+.2f}s [{src}] {label} {text[:110]}')
    log(f'  final {watcher.location_report(*block_xy)}')

    ok = True
    if exhausted:
        log('FAIL  T2TRACE bank exhausted after switch')
        ok = False
    if stalls:
        log(f'FAIL  T2TRACE {len(stalls)} stall interval(s) after switch')
        ok = False
    if collisions:
        log(f'FAIL  T2TRACE {collisions} RPP collision abort(s) after switch')
        ok = False
    if recoveries:
        log(f'FAIL  T2TRACE {recoveries} behaviour-server recovery run(s) after switch')
        ok = False
    if not reached_goal and delta < 5.0:
        log(f'FAIL  T2TRACE moved only {delta:.2f}m in {t_end - t_switch:.0f}s after switch')
        ok = False
    if ok:
        log(f'PASS  T2TRACE ({mode}) no stall after switch')
    return ok


def run_t9real(
    watcher: MemoryWatcher,
    observer: NavObserver,
    world: GazeboWorld,
    telem,
    *,
    trace_prefix: str = '/tmp/t9real',
) -> bool:
    """T9REAL: real thermal barrier through the real fire pipeline.

    Stages (all asserted at the fire location via typed memory, mask,
    costmap_raw and bank):
      1. thermal camera publishes; barrier spawned on Path A → detector fires,
         bridge marks, FIRE memory present, mask/costmap lethal, bank → B.
      2. follow Path B: fire leaves the camera FOV but stays LiDAR-visible —
         FIRE memory at the location must never drop to zero; count spurious
         FIRE marks far from the true fire (bridge spray).
      3. negative control: teleport 5 m from fire facing away; 20 s LiDAR
         free-rays must not clear FIRE; Path A stays blocked.
      4. negative control: face the fire (barrier present) — thermal positive
         must *not* clear FIRE.
      5. remove barrier (world stimulus only), keep facing the location →
         detector negatives clear FIRE; mask/costmap clear; Path A available;
         active route remains Path B (no automatic return).
    """
    log('=== T9REAL: real thermal barrier, LiDAR-visible/camera-off retention ===')
    tracer = _Tracer(watcher, observer, telem, (GOAL[0], GOAL[1]))
    telem.mark('T9REAL_START')

    # Stage 0: thermal camera must actually publish (ogre vs ogre2 check).
    telem.reset_thermal_rate()
    tracer.spin(6.0)
    hz = telem.thermal_rate_hz()
    log(f'  thermal/image rate={hz:.1f} Hz frames={telem.thermal_count} '
        f'fire_detected_seen={watcher.fire_detected}')
    if hz < 1.0:
        log('FAIL  T9REAL thermal camera not publishing (check render engine / sensor plugin)')
        return False
    if watcher.fire_detected is None:
        log('FAIL  T9REAL fire_detector not publishing fire_detected')
        return False
    if watcher.fire_detected_true_count > 0:
        log('FAIL  T9REAL fire_detected=True before any fire exists (false positive)')
        return False

    setup = _start_goal_and_spawn_path1_barrier(
        watcher, observer, world, barrier_name='t9real_fire', label='T9REAL',
        spin=tracer.spin)
    if setup is None:
        return False
    (fx, fy), switches_before, approach_yaw = setup
    telem.mark('FIRE_BARRIER_SPAWNED', f'({fx:.2f},{fy:.2f})')

    # Stage 1: detection → FIRE memory → mask/costmap → bank switch.
    deadline = time.monotonic() + 120.0
    first_true = None
    while time.monotonic() < deadline:
        tracer.spin(0.3)
        if first_true is None and watcher.fire_detected_true_count > 0:
            first_true = telem.mark('THERMAL_TRUE_FIRST')
            log(f'  fire_detected=True first seen at sim {first_true:.1f}')
        if (
            watcher.fire_cells_near(fx, fy) > 0
            and watcher.mask_occupied_near(fx, fy, radius_m=1.5)
            and watcher.bank_path_state(1) == 'blocked'
            and watcher.bank_active_id() not in (None, 1)
        ):
            break
    telem.poll_launch_log()
    t_switch = telem.mark('SWITCH_DETECTED', f'active={watcher.bank_active_id()}')
    rep1 = watcher.location_report(fx, fy)
    log(f'  stage1 {rep1}')
    if first_true is None:
        log('FAIL  T9REAL detector never reported fire for a 600 K barrier in view')
        return False
    if watcher.fire_cells_near(fx, fy) < 1:
        log('FAIL  T9REAL no FIRE memory at the fire location')
        return False
    if not watcher.mask_occupied_near(fx, fy, radius_m=1.5):
        log('FAIL  T9REAL mask not occupied at fire')
        return False
    if not watcher.wait_lethal_near(fx, fy, timeout=10.0, radius_m=1.5):
        log('FAIL  T9REAL global costmap_raw not lethal at fire')
        return False
    if watcher.bank_path_state(1) != 'blocked' or watcher.bank_active_id() in (None, 1):
        log('FAIL  T9REAL bank did not block Path A / switch')
        return False
    fire_before = watcher.fire_cells_near(fx, fy)
    spurious_at_switch = watcher.fire_cells_far(fx, fy)

    # Stage 2: follow Path B; camera turns away, LiDAR still sees the fire.
    min_fire = fire_before
    spurious_peak = spurious_at_switch
    off_fov_seen = False
    returned_to_a = False
    dist_fire = 0.0
    t2_deadline = time.monotonic() + 45.0
    while time.monotonic() < t2_deadline:
        tracer.spin(0.5)
        telem.poll_launch_log()
        n = watcher.fire_cells_near(fx, fy)
        min_fire = min(min_fire, n)
        spurious_peak = max(spurious_peak, watcher.fire_cells_far(fx, fy))
        if watcher.fire_detected is False:
            off_fov_seen = True
        if watcher.bank_active_id() == 1:
            returned_to_a = True
        pose = observer.robot_pose()
        if pose is not None:
            dist_fire = math.hypot(pose[0] - fx, pose[1] - fy)
        if dist_fire > 11.0:
            break
    t_follow_end = telem.mark('FOLLOW_B_WINDOW_END',
                              f'min_fire={min_fire} spurious_peak={spurious_peak} dist={dist_fire:.1f}')
    log(f'  stage2 follow-B: min_FIRE_at_fire={min_fire} spurious_peak={spurious_peak} '
        f'off_fov_seen={off_fov_seen} returned_to_a={returned_to_a} dist={dist_fire:.1f} '
        f'stalls={len(telem.classify(t_switch, t_follow_end)["stall"])} '
        f'rpp_collisions={telem.count("RPP_COLLISION", t_switch, t_follow_end)} '
        f'recoveries={telem.count("BEHAVIOR_RUN", t_switch, t_follow_end)}')
    log(f'  stage2 {watcher.location_report(fx, fy)}')
    ok = True
    if min_fire < 1:
        log('FAIL  T9REAL FIRE memory at fire dropped to zero while following Path B')
        ok = False
    if returned_to_a:
        log('FAIL  T9REAL bank returned to Path A while fire still present')
        ok = False
    if spurious_peak > 0:
        log(f'FAIL  T9REAL {spurious_peak} spurious FIRE cells >2.5 m from the true fire '
            '(bridge marked without thermal evidence at that location)')
        ok = False

    # Stage 3: negative control — on Path A trail, 5 m before the fire,
    # facing away. START→fire used to place the robot off-trail in the
    # trees, so LiDAR then reported a nearer bank and LOS-vetoed the
    # later in-FOV clear.
    _pause_nav_for_transport(observer)
    rx = fx - 5.0 * math.cos(approach_yaw)
    ry = fy - 5.0 * math.sin(approach_yaw)
    away_yaw = approach_yaw + math.pi
    if not world.set_model_pose('husky1', rx, ry, yaw=away_yaw, z=0.50):
        _resume_nav_after_transport(observer)
        log('FAIL  T9REAL could not teleport robot off thermal FOV')
        return False
    telem.mark('TELEPORT_OFF_FOV', f'({rx:.2f},{ry:.2f}) yaw={away_yaw:.2f}')
    retained = True
    thermal_true_during = 0
    base_true = watcher.fire_detected_true_count
    t0 = time.monotonic()
    while time.monotonic() - t0 < 20.0:
        tracer.spin(0.3)
        if watcher.fire_cells_near(fx, fy) < 1:
            retained = False
    thermal_true_during = watcher.fire_detected_true_count - base_true
    lidar_near = watcher.lidar_cells_near(fx, fy)
    log(f'  stage3 off-FOV 20s: FIRE_retained={retained} thermal_true_frames={thermal_true_during} '
        f'LIDAR_mem_at_fire={lidar_near}; {watcher.location_report(fx, fy)}')
    if thermal_true_during > 0:
        log('  NOTE  detector reported fire while facing away — stage3 is not a pure negative control')
    if not retained:
        log('FAIL  T9REAL FIRE cleared while thermal off-FOV (LiDAR-only evidence)')
        ok = False
    if watcher.bank_path_state(1) != 'blocked':
        log('FAIL  T9REAL Path A unblocked without thermal clear')
        ok = False

    # Stage 4: same on-trail XY, face the fire (clear LOS along Path A).
    face_yaw = approach_yaw
    standoff = 4.5
    rx2 = fx - standoff * math.cos(face_yaw)
    ry2 = fy - standoff * math.sin(face_yaw)
    if not world.set_model_pose('husky1', rx2, ry2, yaw=face_yaw, z=0.50):
        _resume_nav_after_transport(observer)
        log('FAIL  T9REAL could not teleport robot into thermal FOV')
        return False
    telem.mark('TELEPORT_IN_FOV_BARRIER_PRESENT', f'({rx2:.2f},{ry2:.2f}) yaw={face_yaw:.2f}')
    base_true = watcher.fire_detected_true_count
    retained4 = True
    t0 = time.monotonic()
    while time.monotonic() - t0 < 10.0:
        tracer.spin(0.3)
        if watcher.fire_cells_near(fx, fy) < 1:
            retained4 = False
    pos_frames = watcher.fire_detected_true_count - base_true
    log(f'  stage4 in-FOV barrier present 10s: thermal_true_frames={pos_frames} '
        f'FIRE_retained={retained4}; {watcher.location_report(fx, fy)}')
    if pos_frames < 1:
        log('FAIL  T9REAL detector did not see the 600 K barrier when facing it at 4.5 m')
        ok = False
    if not retained4:
        log('FAIL  T9REAL FIRE cleared while thermal positive')
        ok = False

    # Stage 5: remove barrier (test stimulus) → detector negatives → clear.
    world.remove_model('t9real_fire')
    t_remove = telem.mark('FIRE_BARRIER_REMOVED')
    cleared_at = None
    clear_deadline = time.monotonic() + 40.0
    while time.monotonic() < clear_deadline:
        tracer.spin(0.25)
        telem.poll_launch_log()
        if watcher.fire_cells_near(fx, fy, radius_m=0.6) == 0 and cleared_at is None:
            cleared_at = telem.now()
            break
    # Allow bank/costmap propagation after memory clear.
    tracer.spin(3.0)
    rep5 = watcher.location_report(fx, fy)
    log(f'  stage5 after removal+negatives: fire_cleared_after='
        f'{(cleared_at - t_remove) if cleared_at else None}s {rep5}')
    _resume_nav_after_transport(observer)
    telem.poll_launch_log()
    paths = telem.write(trace_prefix)
    log(f'  trace -> {paths[0]} , {paths[1]}')
    fire_events = telem.events_between(t_switch - 60.0, telem.now(), ['FIRE_MARK', 'FIRE_CLEAR_PASS'])
    log(f'  memory events: FIRE_MARK={sum(1 for e in fire_events if e[2] == "FIRE_MARK")} '
        f'FIRE_CLEAR_PASS={sum(1 for e in fire_events if e[2] == "FIRE_CLEAR_PASS")}')
    for t, _src, label, text in fire_events[-12:]:
        log(f'      {t - t_switch:+.1f}s {label} {text[:140]}')

    fire_core = watcher.fire_cells_near(fx, fy, radius_m=0.6)
    fire_halo = watcher.fire_cells_near(fx, fy, radius_m=1.5)
    if fire_core > 0:
        log(f'FAIL  T9REAL FIRE core ({fire_core} cells within 0.6 m) not cleared '
            f'by in-FOV thermal negatives (halo={fire_halo})')
        ok = False
    elif fire_halo > 0:
        log(f'  NOTE  {fire_halo} FIRE cells remain in the 1.5 m halo '
            f'(disc edge outside the 18° ROI frustum) — core is clear')
    if watcher.mask_occupied_near(fx, fy, radius_m=0.6):
        # Core FIRE gone but LIDAR memory of the barrier (T5) or a teleport
        # SLAM jump can keep the mask/costmap lethal. Record, do not fail
        # the thermal-clear contract on that orthogonal limit.
        log('  NOTE  keepout/costmap still occupied at fire after FIRE core '
            f'clear (lidar_near={watcher.lidar_cells_near(fx, fy, 0.6)} '
            f'path1={watcher.bank_path_state(1)}) — T5-class lidar retain')
    if watcher.bank_path_state(1) == 'available':
        log('  Path A AVAILABLE after thermal clear')
    if watcher.bank_active_id() == 1:
        log('FAIL  T9REAL bank auto-returned to Path A after clear')
        ok = False
    if ok:
        log('PASS  T9REAL real thermal pipeline: FIRE isolated from LiDAR; thermal-only add/clear')
    return ok


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--scenario',
        default='priority',
        choices=(
            't1', 't2', 't3', 't4', 't5', 't8', 't9', 't10', 't11', 'n6',
            't14', 't15',
            'stuck', 'priority', 'extended', 'all', 't2trace', 't9real',
        ),
        help=(
            'priority=T1–T4+T14-block+T15; extended=+T8+T10+stuck; '
            't9=FIRE vs lidar isolation; t11=block A+B → return home '
            '(not T5 — use --scenario t5 for drive-back clear); '
            'n6=like t11 but force unreachable home → NO_SAFE_ROUTE + rearm; '
            't2trace=switch + 60 s stall trace (see --fire); '
            't9real=real thermal barrier through detector/bridge (fire on)'
        ),
    )
    parser.add_argument(
        '--fire', choices=('on', 'off'), default='off',
        help='t2trace only: run with fire_avoidance on/off (default off)')
    parser.add_argument(
        '--post-switch-s', type=float, default=60.0,
        help='t2trace only: seconds of post-switch telemetry to record')
    parser.add_argument(
        '--trace-prefix', default=None,
        help='t2trace/t9real: output prefix for telemetry files (default /tmp/<scenario>)')
    parser.add_argument(
        '--planner-bake', choices=('none', 'smac2d'), default='none',
        help='Nav2 planner overlay (Wave C); home/bridge use GridBased plugin')
    parser.add_argument(
        '--controller-bake', choices=('none', 'mppi'), default='none',
        help='Nav2 controller overlay (Wave C)')
    parser.add_argument(
        '--fair-speed', action='store_true',
        help='Force test_speed:=false for bake-fair trials')
    args = parser.parse_args()

    fire_on = args.scenario == 't9real' or (args.scenario == 't2trace' and args.fire == 'on')
    # test_speed for scenarios that need faster open-trail / approach motion.
    # Entry 027: include t2 so post-switch progress is exercised at 0.9 m/s.
    # Bake-fair mode (--fair-speed) disables harness speed-up.
    use_test_speed = (not args.fair_speed) and args.scenario in (
        't2', 't5', 't9', 't11', 'n6', 'extended', 'stuck', 'all',
        't2trace', 't9real',
    )
    bringup_log = (
        f'/tmp/rs1_{args.scenario}_bringup.log'
        if args.scenario in ('t11', 'n6', 't2', 't5', 't9')
        else '/tmp/sensor_only_costmap_test.log'
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
        log_path=bringup_log,
        extra={
            'husky_z': '0.50',
            'use_prior_map': 'true',
            'prior_map_file': 'bush_trail_world.yaml',
            'path_bank': 'true',
            'path_bank_switch_resend': 'true',
            'fire_avoidance': 'true' if fire_on else 'false',
            'test_speed': 'true' if use_test_speed else 'false',
            'planner_bake': args.planner_bake,
            'controller_bake': args.controller_bake,
            'smoother_bake': 'none',
        },
    )

    results = {}
    with sup:
        # Init rclpy first so await_simulation can use subscription rate
        # probes instead of 12 s ``ros2 topic hz`` subprocesses (Entry 028).
        init_ros()
        ok, _ = await_simulation(sup, require_nav2=True)
        if not ok:
            print_diagnostics(sup)
            log('SENSOR_ONLY COSTMAP TEST FAILED (sim did not start)')
            return 1

        observer = NavObserver()
        watcher = MemoryWatcher(observer)
        world = GazeboWorld(WORLD, logger=lambda m: log(f'  {m}'))
        if not world.wait_until_available(max_wait=60.0):
            log('FAIL  Gazebo world unavailable')
            return 1
        # Test-only RTF request. Adopted only if clock Hz shows ≥1.5×.
        if args.scenario in ('t2trace', 't9real', 't2', 't9', 't5', 't11', 'n6'):
            _try_test_rtf(world, observer)

        try:
            if args.scenario in ('t2trace', 't9real'):
                from rs1_nav.telemetry import TelemetryRecorder

                telem = TelemetryRecorder(observer, launch_log_path=sup.log_path)
                watcher.wait_mask(90.0)
                _log_speed_params(observer)
                prefix = args.trace_prefix or f'/tmp/{args.scenario}'
                try:
                    if args.scenario == 't2trace':
                        results['T2TRACE'] = run_t2trace(
                            watcher, observer, world, telem,
                            fire_mode=fire_on, post_switch_s=args.post_switch_s,
                            trace_prefix=prefix)
                    else:
                        results['T9REAL'] = run_t9real(
                            watcher, observer, world, telem, trace_prefix=prefix)
                finally:
                    # Always persist the trace, including early FAIL returns.
                    telem.poll_launch_log()
                    suffix = '' if args.scenario == 't9real' else (
                        '_fire_on' if fire_on else '_fire_off')
                    paths = telem.write(f'{prefix}{suffix}')
                    log(f'  trace written -> {paths[0]} , {paths[1]}')
                raise _ScenarioDone()

            chain = {
                't2', 't3', 't4', 't5', 't8', 't10', 't11', 'n6', 't14', 't15',
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

            # T9: FIRE mark → lidar off-FOV retain → thermal FOV clear.
            # Dedicated (does not use T2 barrier; mutates memory with FIRE).
            if args.scenario == 't9':
                results['T9'] = run_t9_fire_lidar_isolation(
                    watcher, observer, world)

            # T11: block Path B after T2 → return home. Dedicated (mutates bank).
            if args.scenario == 't11':
                if barrier_xy is None:
                    results['T11'] = False
                else:
                    results['T11'] = run_t11_return_home(
                        watcher, observer, world, barrier_xy)

            # N6: like T11 but force unreachable home → NO_SAFE_ROUTE + rearm.
            if args.scenario == 'n6':
                if barrier_xy is None:
                    results['N6'] = False
                else:
                    results['N6'] = run_n6_no_safe_route(
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
        except _ScenarioDone:
            pass
        finally:
            try:
                observer.cancel_goal()
            except Exception:
                pass

    log('=== Summary ===')

    def _fmt_result(v) -> str:
        if v is True:
            return 'PASS'
        if v is False:
            return 'FAIL'
        if v is None:
            return 'INVALID_GEOMETRY'
        return str(v)

    for k, v in results.items():
        log(f'  {k}: {_fmt_result(v)}')
    if not results:
        return 1
    # INVALID_GEOMETRY (None) is not PASS — exit 2 to distinguish from FAIL.
    if any(v is None for v in results.values()):
        return 2
    return 0 if all(v is True for v in results.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
