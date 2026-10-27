#!/usr/bin/env python3
"""Path-bank manager — mission-scoped alternative routes for Husky Nav2.

Authority model
---------------
* Nav2 (NavFn + IsPathValid BT + RPP) still owns follow/replan *timing*.
* This node owns the *candidate set*: generate once, rank, mark blocked,
  select next valid candidate when the active route is collision-invalid on
  the live global costmap (plugins + KeepoutFilter obstacle memory).

It does NOT replan every second. Validity is checked at ``check_hz`` (default
2 Hz). A switch only happens when the active bank path intersects lethal
cost on the live costmap.

On switch:
  1. Mark current candidate BLOCKED (ROUTE_MARKED_BLOCKED)
  2. Pick next highest-scored AVAILABLE candidate that is still valid live
  3. Preempt NavigateToPose onto the same goal (NavFn sees keepout + live)
  If no candidate remains valid → NO_PATH_AVAILABLE; Entry 024 optionally
  preempts NavigateToPose back to the recorded mission-start home pose
  (RETURNING_TO_START → RETURNED_TO_START).

Entry 021 — evidence unblock:
  BLOCKED→AVAILABLE only when ``obstacle_memory/mask`` no longer occupies the
  corridor AND ``costmap_raw`` reports the path valid. Never use Gazebo GT or
  "a new plan exists". Unlock does **not** auto-preempt back to that path.
  Entry 024: once RETURNING_TO_START / RETURNED_TO_START, unlock does not
  resume the forward mission.

Static vs live
--------------
Planning bank at mission start prefers ``prior_map`` (static baseline).
Validity monitoring always uses the live ``global_costmap/costmap_raw``.
"""

from __future__ import annotations

import math
import time
from enum import Enum
from typing import List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from rs1_nav.path_bank import (
    CandidateState,
    PathBank,
    PathBankConfig,
    PathCandidate,
    build_path_bank,
    nav2_costmap_to_grid,
    occupancy_mask_intersects_path,
    occupancy_msg_to_grid,
    path_is_valid,
)


LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


class MissionNavState(str, Enum):
    IDLE = 'IDLE'
    INITIALISING = 'INITIALISING'
    PLANNING = 'PLANNING'
    FOLLOWING_PATH = 'FOLLOWING_PATH'
    PATH_BLOCKED = 'PATH_BLOCKED'
    SWITCHING_PATH = 'SWITCHING_PATH'
    NO_PATH_AVAILABLE = 'NO_PATH_AVAILABLE'
    # Entry 024: all banked routes exhausted → navigate back to mission start.
    RETURNING_TO_START = 'RETURNING_TO_START'
    RETURNED_TO_START = 'RETURNED_TO_START'
    ARRIVED = 'ARRIVED'
    FAILED = 'FAILED'


class PathBankManager(Node):
    def __init__(self) -> None:
        super().__init__('path_bank_manager')
        self._cbg = ReentrantCallbackGroup()

        self.declare_parameter('robot_name', 'husky1')
        self.declare_parameter('enabled', True)
        self.declare_parameter('max_candidates', 4)
        self.declare_parameter('exclusion_radius_m', 2.5)
        self.declare_parameter('min_path_separation_m', 2.0)
        self.declare_parameter('min_clearance_m', 0.35)
        self.declare_parameter('length_weight', 0.45)
        self.declare_parameter('clearance_weight', 0.35)
        self.declare_parameter('cost_weight', 0.15)
        self.declare_parameter('smoothness_weight', 0.05)
        self.declare_parameter('clearance_ref_m', 2.0)
        self.declare_parameter('check_hz', 2.0)
        self.declare_parameter('footprint_inflate_m', 0.38)
        # Entry 019+: after PGM↔Gazebo alignment, default launch enables
        # switch_resend (preempt onto next banked route). Parameter still
        # defaults False here so unit/manual node starts stay observe-only
        # unless launch sets path_bank_switch_resend:=true.
        self.declare_parameter('auto_switch', True)
        self.declare_parameter('switch_resend_goal', False)
        self.declare_parameter('validity_grace_s', 15.0)
        # Entry 023: after A→B switch, pause validity so Keepout/local costmap
        # and NavFn settle before the new active path can be re-blocked.
        self.declare_parameter('post_switch_grace_s', 8.0)
        self.declare_parameter('cancel_on_no_path', False)
        # Entry 024: when the bank is exhausted, preempt NavigateToPose home.
        self.declare_parameter('return_home_on_all_blocked', True)
        self.declare_parameter('home_yaw', 0.0)
        # Keep 254 (LETHAL). Using 253 (inscribed) false-blocks alternate
        # bank routes on soft inflation and yields active_id=None (T2 flake).
        self.declare_parameter('lethal_threshold', 254)

        def _as_bool(value) -> bool:
            if isinstance(value, str):
                return value.strip().lower() in ('1', 'true', 'yes', 'on')
            return bool(value)

        robot = str(self.get_parameter('robot_name').value).strip('/') or 'husky1'
        self._robot = robot
        self._map_frame = f'{robot}_map'
        self._enabled = _as_bool(self.get_parameter('enabled').value)
        self._auto_switch = _as_bool(self.get_parameter('auto_switch').value)
        self._switch_resend = _as_bool(self.get_parameter('switch_resend_goal').value)
        self._grace_s = float(self.get_parameter('validity_grace_s').value)
        self._post_switch_grace_s = float(
            self.get_parameter('post_switch_grace_s').value)
        self._cancel_on_no_path = _as_bool(self.get_parameter('cancel_on_no_path').value)
        self._return_home_on_all_blocked = _as_bool(
            self.get_parameter('return_home_on_all_blocked').value)
        self._home_yaw = float(self.get_parameter('home_yaw').value)
        self._bank_ready_at: Optional[float] = None
        # Entry 023: absolute monotonic deadline after a bank switch.
        self._switch_grace_until: float = 0.0
        # Ignore NavigateToPose result callbacks from goals we intentionally
        # superseded during a bank switch (preempt/abort is expected).
        self._ignore_goal_results = 0
        # Entry 023: one abort→resend per mission goal to recover ownership
        # when the post-switch NavigateToPose aborts while FOLLOWING_PATH.
        # Entry 024: also used once while RETURNING_TO_START (resend home).
        self._abort_resend_used = False

        self._cfg = PathBankConfig(
            max_candidates=int(self.get_parameter('max_candidates').value),
            exclusion_radius_m=float(self.get_parameter('exclusion_radius_m').value),
            min_path_separation_m=float(self.get_parameter('min_path_separation_m').value),
            min_clearance_m=float(self.get_parameter('min_clearance_m').value),
            lethal_threshold=int(self.get_parameter('lethal_threshold').value),
            length_weight=float(self.get_parameter('length_weight').value),
            clearance_weight=float(self.get_parameter('clearance_weight').value),
            cost_weight=float(self.get_parameter('cost_weight').value),
            smoothness_weight=float(self.get_parameter('smoothness_weight').value),
            clearance_ref_m=float(self.get_parameter('clearance_ref_m').value),
        )
        self._footprint_inflate = float(self.get_parameter('footprint_inflate_m').value)

        self._prior_map: Optional[OccupancyGrid] = None
        self._live_costmap: Optional[Costmap] = None
        self._memory_mask: Optional[OccupancyGrid] = None
        self._bank: Optional[PathBank] = None
        self._state = MissionNavState.IDLE
        self._goal_xyyaw: Optional[Tuple[float, float, float]] = None
        # Entry 024: mission-start home; nav_mode 'mission' | 'home' | None.
        self._home_xyyaw: Optional[Tuple[float, float, float]] = None
        self._nav_mode: Optional[str] = None
        self._goal_handle = None
        self._switch_count = 0
        self._building = False

        self.create_subscription(OccupancyGrid, 'prior_map', self._on_prior, LATCHED)
        self.create_subscription(
            Costmap, 'global_costmap/costmap_raw', self._on_costmap, 2)
        self.create_subscription(
            OccupancyGrid, 'obstacle_memory/mask', self._on_memory_mask, LATCHED)
        self.create_subscription(Path, 'plan', self._on_plan, 5)
        self.create_subscription(PoseStamped, 'goal_pose', self._on_goal_pose, 10)

        self._state_pub = self.create_publisher(String, 'path_bank/state', 10)
        self._status_pub = self.create_publisher(String, 'path_bank/status', LATCHED)
        self._active_path_pub = self.create_publisher(Path, 'path_bank/active_path', 10)
        self._alt_path_pubs = [
            self.create_publisher(Path, f'path_bank/candidate_{i}', 10)
            for i in range(1, self._cfg.max_candidates + 1)
        ]
        self._marker_pub = self.create_publisher(MarkerArray, 'path_bank/markers', 10)

        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose', callback_group=self._cbg)

        hz = float(self.get_parameter('check_hz').value)
        self.create_timer(1.0 / max(hz, 0.1), self._on_timer, callback_group=self._cbg)
        self.create_timer(1.0, self._publish_status, callback_group=self._cbg)

        self.get_logger().info(
            f'path_bank_manager ready (robot={robot}, enabled={self._enabled}, '
            f'max_candidates={self._cfg.max_candidates})'
        )

    # -- callbacks ---------------------------------------------------------

    def _on_prior(self, msg: OccupancyGrid) -> None:
        self._prior_map = msg

    def _on_costmap(self, msg: Costmap) -> None:
        self._live_costmap = msg

    def _on_memory_mask(self, msg: OccupancyGrid) -> None:
        self._memory_mask = msg

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        if not self._enabled:
            return
        yaw = _yaw_from_quat(msg.pose.orientation)
        self._goal_xyyaw = (msg.pose.position.x, msg.pose.position.y, yaw)
        self._set_state(MissionNavState.PLANNING)
        self._bank = None
        self._home_xyyaw = None
        self._nav_mode = None
        self._switch_count = 0
        self._abort_resend_used = False
        self.get_logger().info(
            f'Goal pose received ({self._goal_xyyaw[0]:.2f}, {self._goal_xyyaw[1]:.2f}); '
            'will build path bank on next plan/costmap'
        )

    def _on_plan(self, msg: Path) -> None:
        if not self._enabled or self._building:
            return
        # Entry 024: ignore plan chatter while returning / returned home.
        if self._state in (
            MissionNavState.RETURNING_TO_START,
            MissionNavState.RETURNED_TO_START,
        ):
            return
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(points) < 2:
            return
        # First plan of a mission: build bank if we do not have one yet.
        if self._bank is None:
            start = points[0]
            goal = points[-1]
            if self._goal_xyyaw is not None:
                goal = (self._goal_xyyaw[0], self._goal_xyyaw[1])
            else:
                # NavigateToPose clients do not publish goal_pose; recover the
                # mission goal from the first Nav2 plan endpoint.
                self._goal_xyyaw = (goal[0], goal[1], 0.0)
            self._build_bank(start, goal)
        if self._state in (MissionNavState.PLANNING, MissionNavState.SWITCHING_PATH, MissionNavState.IDLE):
            self._set_state(MissionNavState.FOLLOWING_PATH)
            self._nav_mode = 'mission'

    def _on_timer(self) -> None:
        if not self._enabled:
            return
        if self._bank is None or self._live_costmap is None:
            return
        # Entry 024: do not run bank switch logic while returning / returned.
        if self._state not in (
            MissionNavState.FOLLOWING_PATH,
            MissionNavState.PATH_BLOCKED,
            MissionNavState.NO_PATH_AVAILABLE,
        ):
            return
        # Allow StaticLayer + ObstacleLayer + inflation to settle before the
        # bank starts invalidating candidates (clean-start false positives).
        now = time.monotonic()
        if self._bank_ready_at is not None:
            if (now - self._bank_ready_at) < self._grace_s:
                return
        # Entry 023: post-switch settle — do not re-block the newly active path
        # while NavFn / KeepoutFilter / local costmap catch up.
        if now < self._switch_grace_until:
            return

        live = nav2_costmap_to_grid(self._live_costmap)
        # Entry 021: evidence-based BLOCKED→AVAILABLE (no auto path flip).
        self._try_evidence_unblock(live)

        active = self._bank.active()
        if active is None:
            chosen = self._bank.select_best_available()
            if chosen is not None:
                self._publish_bank_paths()
            return

        ok, reason = path_is_valid(
            active.points,
            live,
            lethal_threshold=self._cfg.lethal_threshold,
            sample_stride=self._cfg.sample_stride,
        )
        # Keepout mask may mark cells one sample off the polyline while
        # costmap_raw centre reads soft/253 — treat memory intersection as
        # blockage so bank switches reliably (Entry 023 / T2).
        if ok and self._memory_intersects_path(active.points):
            ok = False
            reason = 'obstacle_memory_mask'
        if ok:
            return

        self.get_logger().warn(
            f'Active Path {active.path_id} invalid on live costmap: {reason}'
        )
        self._set_state(MissionNavState.PATH_BLOCKED)
        self._bank.mark_blocked(active.path_id, reason)
        self.get_logger().info(
            f'ROUTE_MARKED_BLOCKED Path {active.path_id}: {reason}'
        )
        self._publish_bank_paths()

        if not self._auto_switch:
            return

        self._set_state(MissionNavState.SWITCHING_PATH)
        nxt = self._pick_next_valid(live)
        if nxt is None:
            self.get_logger().error(
                'No valid banked path remains'
                + (' (all_blocked)' if self._bank.all_blocked() else '')
            )
            self._set_state(MissionNavState.NO_PATH_AVAILABLE)
            if self._return_home_on_all_blocked:
                self._begin_return_to_start()
                return
            # Legacy advisory mode: leave IsPathValid BT in control.
            self.get_logger().error(
                'return_home_on_all_blocked=false; leaving Nav2 IsPathValid BT '
                'in control'
            )
            if self._cancel_on_no_path:
                self._cancel_nav()
            return

        self._switch_count += 1
        self.get_logger().info(
            f'Switching to Path {nxt.path_id} (score={nxt.score:.1f}, '
            f'switch #{self._switch_count})'
        )
        self._publish_bank_paths()
        if self._switch_resend and self._goal_xyyaw is not None:
            # Preempt with a fresh NavigateToPose (do not cancel first). A
            # cancel→resend race was marking FAILED when the aborted goal's
            # result arrived after state returned to FOLLOWING_PATH.
            # Live costmap already contains the blockage so NavFn avoids it.
            self._nav_mode = 'mission'
            if self._goal_handle is not None:
                self._ignore_goal_results += 1
            self._send_goal(*self._goal_xyyaw)
        # Entry 023: settle window before re-validating the newly active path.
        if self._post_switch_grace_s > 0.0:
            self._switch_grace_until = time.monotonic() + self._post_switch_grace_s
        self._set_state(MissionNavState.FOLLOWING_PATH)

    def _memory_intersects_path(self, points: List[Tuple[float, float]]) -> bool:
        """True if obstacle_memory mask marks occupied near the path.

        Checks a 1-cell neighbourhood around each sample so thin keepout marks
        one cell off the polyline still count (Entry 023 T2 reliability).
        """
        mask = self._memory_mask
        if mask is None or not mask.data:
            return False
        info = mask.info
        return occupancy_mask_intersects_path(
            points,
            width=int(info.width),
            height=int(info.height),
            resolution=float(info.resolution),
            origin_x=float(info.origin.position.x),
            origin_y=float(info.origin.position.y),
            data=mask.data,
            sample_stride=self._cfg.sample_stride,
            occupied_threshold=50,
            neighbourhood=1,
        )

    def _try_evidence_unblock(self, live_grid) -> None:
        """BLOCKED→AVAILABLE only when memory + costmap_raw both clear.

        Does not change active_id or preempt NavigateToPose (no auto return).
        """
        if self._bank is None:
            return
        unlocked = False
        for cand in self._bank.candidates:
            if cand.state != CandidateState.BLOCKED:
                continue
            if self._memory_intersects_path(cand.points):
                continue
            ok, _reason = path_is_valid(
                cand.points,
                live_grid,
                lethal_threshold=self._cfg.lethal_threshold,
                sample_stride=self._cfg.sample_stride,
            )
            if not ok:
                continue
            if self._bank.mark_available(cand.path_id, 'evidence_clear'):
                unlocked = True
                self.get_logger().info(
                    f'ROUTE_MARKED_AVAILABLE Path {cand.path_id} '
                    '(memory+costmap_raw agree clear; not auto-selected)'
                )
        if unlocked:
            self._publish_bank_paths()
            # If we were NO_PATH_AVAILABLE and something unlocked, resume follow.
            # Entry 024: never resume forward mission once return-to-start armed.
            if self._state == MissionNavState.NO_PATH_AVAILABLE:
                if self._nav_mode == 'home':
                    return
                if self._bank.active() is not None or self._bank.available():
                    self._set_state(MissionNavState.FOLLOWING_PATH)

    # -- bank construction -------------------------------------------------

    def _build_bank(self, start: Tuple[float, float], goal: Tuple[float, float]) -> None:
        self._building = True
        self._set_state(MissionNavState.PLANNING)
        try:
            # Entry 024: capture mission-start home from first plan start.
            self._home_xyyaw = (float(start[0]), float(start[1]), self._home_yaw)
            self.get_logger().info(
                f'Home pose recorded ({self._home_xyyaw[0]:.2f}, '
                f'{self._home_xyyaw[1]:.2f}, yaw={self._home_xyyaw[2]:.2f})'
            )

            grid = None
            if self._prior_map is not None:
                grid = occupancy_msg_to_grid(self._prior_map)
                self.get_logger().info(
                    f'Building path bank from prior_map '
                    f'{grid.width}x{grid.height} @ {grid.resolution}m'
                )
            elif self._live_costmap is not None:
                grid = nav2_costmap_to_grid(self._live_costmap)
                self.get_logger().info('Building path bank from live global costmap')
            else:
                self.get_logger().warn('No prior_map or costmap yet; bank deferred')
                return

            bank = build_path_bank(
                grid, start, goal, self._cfg,
                footprint_inflate_m=self._footprint_inflate,
                downsample_factor=4,
            )
            if not bank.candidates:
                self.get_logger().warn('Path bank generated 0 candidates')
                self._bank = bank
                self._set_state(MissionNavState.NO_PATH_AVAILABLE)
                if self._return_home_on_all_blocked:
                    self._begin_return_to_start()
                return

            chosen = bank.select_best_available()
            self._bank = bank
            assert chosen is not None
            for line in (c.summary() for c in bank.candidates):
                self.get_logger().info(f'  {line}')
            self.get_logger().info(
                f'Path bank ready: {len(bank.candidates)} candidates; '
                f'active=Path {chosen.path_id}'
            )
            self._publish_bank_paths()
            self._bank_ready_at = time.monotonic()
            self._abort_resend_used = False
            self._switch_grace_until = 0.0
            self._nav_mode = 'mission'
            self._set_state(MissionNavState.FOLLOWING_PATH)
        finally:
            self._building = False

    def _begin_return_to_start(self) -> None:
        """Entry 024: preempt NavigateToPose to recorded home after bank exhaustion."""
        if self._home_xyyaw is None:
            self.get_logger().error(
                'ALL_BLOCKED but no home pose recorded; cannot return to start'
            )
            if self._cancel_on_no_path:
                self._cancel_nav()
            return
        hx, hy, hyaw = self._home_xyyaw
        self.get_logger().warn(
            f'ALL_BLOCKED → RETURNING_TO_START home=({hx:.2f}, {hy:.2f})'
        )
        self._nav_mode = 'home'
        self._abort_resend_used = False
        self._set_state(MissionNavState.RETURNING_TO_START)
        # Same preempt pattern as path switch (Entry 023): do not cancel first.
        if self._goal_handle is not None:
            self._ignore_goal_results += 1
        self._send_goal(hx, hy, hyaw)

    def _pick_next_valid(self, live_grid) -> Optional[PathCandidate]:
        # Re-validate remaining AVAILABLE candidates against live costmap AND
        # obstacle_memory mask (Entry 023: same gate as active-path checks).
        remaining = sorted(
            [c for c in self._bank.candidates if c.state == CandidateState.AVAILABLE],
            key=lambda c: c.score,
            reverse=True,
        )
        for cand in remaining:
            ok, reason = path_is_valid(
                cand.points,
                live_grid,
                lethal_threshold=self._cfg.lethal_threshold,
                sample_stride=self._cfg.sample_stride,
            )
            if ok and self._memory_intersects_path(cand.points):
                ok = False
                reason = 'obstacle_memory_mask'
            if not ok:
                self._bank.mark_blocked(cand.path_id, f'precheck:{reason}')
                self.get_logger().info(
                    f'Path {cand.path_id} also invalid ({reason}); skipping'
                )
                continue
            cand.state = CandidateState.ACTIVE
            self._bank.active_id = cand.path_id
            return cand
        return None

    # -- nav helpers -------------------------------------------------------

    def _send_goal(self, x: float, y: float, yaw: float) -> None:
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('navigate_to_pose server not available')
            return
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = self._map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        half = 0.5 * yaw
        pose.pose.orientation.z = math.sin(half)
        pose.pose.orientation.w = math.cos(half)
        goal.pose = pose
        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response)

    def _goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'goal send failed: {exc}')
            return
        if not handle.accepted:
            self.get_logger().warn('NavigateToPose goal rejected')
            return
        self._goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._goal_result)

    def _goal_result(self, future) -> None:
        try:
            result = future.result()
            status = result.status
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'goal result failed: {exc}')
            if self._ignore_goal_results > 0:
                self._ignore_goal_results -= 1
                return
            self._set_state(MissionNavState.FAILED)
            return
        if self._ignore_goal_results > 0 and status != GoalStatus.STATUS_SUCCEEDED:
            self._ignore_goal_results -= 1
            self.get_logger().info(
                f'Ignoring superseded NavigateToPose result (status={status})'
            )
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            if self._nav_mode == 'home' or self._state == MissionNavState.RETURNING_TO_START:
                self._set_state(MissionNavState.RETURNED_TO_START)
                self._goal_handle = None
                home = self._home_xyyaw
                self.get_logger().info(
                    'RETURNED_TO_START'
                    + (
                        f' home=({home[0]:.2f}, {home[1]:.2f})'
                        if home is not None else ''
                    )
                    + f'; switches={self._switch_count}'
                )
            else:
                self._set_state(MissionNavState.ARRIVED)
                self.get_logger().info('Goal reached (NavigateToPose succeeded)')
        elif self._state in (
            MissionNavState.SWITCHING_PATH,
            MissionNavState.PATH_BLOCKED,
        ):
            return
        elif status in (
            GoalStatus.STATUS_CANCELED,
            GoalStatus.STATUS_ABORTED,
        ):
            # Entry 024: abort while returning home → resend home once.
            if (
                self._state == MissionNavState.RETURNING_TO_START
                and self._home_xyyaw is not None
                and self._ignore_goal_results == 0
                and not self._abort_resend_used
            ):
                self._abort_resend_used = True
                self.get_logger().warn(
                    f'NavigateToPose ended with status={status} while '
                    f'RETURNING_TO_START; resending home once '
                    '(Entry 024 ownership recovery)'
                )
                self._send_goal(*self._home_xyyaw)
                return
            if (
                self._state == MissionNavState.RETURNING_TO_START
                and self._abort_resend_used
            ):
                self.get_logger().error(
                    f'Return-to-start NavigateToPose status={status} after '
                    'one resend; FAILED'
                )
                self._set_state(MissionNavState.FAILED)
                return
            # Entry 023: if we own the mission goal and are FOLLOWING_PATH,
            # resend once so an unexpected abort does not leave the robot
            # frozen with no NavigateToPose client.
            if (
                self._state == MissionNavState.FOLLOWING_PATH
                and self._nav_mode != 'home'
                and self._goal_xyyaw is not None
                and self._ignore_goal_results == 0
                and not self._abort_resend_used
            ):
                self._abort_resend_used = True
                self.get_logger().warn(
                    f'NavigateToPose ended with status={status} while '
                    f'{self._state.value}; resending goal once '
                    '(Entry 023 ownership recovery)'
                )
                self._send_goal(*self._goal_xyyaw)
                return
            # External client (test/UI) may own the original goal; bank
            # switch preempt is not a mission failure by itself.
            self.get_logger().warn(
                f'NavigateToPose ended with status={status}; '
                f'leaving state={self._state.value}'
            )
        else:
            self._set_state(MissionNavState.FAILED)

    def _cancel_nav(self) -> None:
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass
            self._goal_handle = None

    # -- publishing --------------------------------------------------------

    def _set_state(self, state: MissionNavState) -> None:
        if state == self._state:
            return
        self._state = state
        msg = String()
        msg.data = state.value
        self._state_pub.publish(msg)

    def _publish_status(self) -> None:
        lines: List[str] = [
            f'state={self._state.value}',
            f'switches={self._switch_count}',
            f'nav_mode={self._nav_mode or "none"}',
        ]
        if self._home_xyyaw is not None:
            lines.append(
                f'home=({self._home_xyyaw[0]:.2f},{self._home_xyyaw[1]:.2f})'
            )
        if self._bank is not None:
            lines.append(f'active_id={self._bank.active_id}')
            lines.append(f'all_blocked={self._bank.all_blocked()}')
            for c in self._bank.candidates:
                lines.append(c.summary())
        msg = String()
        msg.data = '\n'.join(lines)
        self._status_pub.publish(msg)

    def _publish_bank_paths(self) -> None:
        if self._bank is None:
            return
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        # Clear previous
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for c in self._bank.candidates:
            path_msg = Path()
            path_msg.header.frame_id = self._map_frame
            path_msg.header.stamp = stamp
            for x, y in c.points:
                ps = PoseStamped()
                ps.header = path_msg.header
                ps.pose.position.x = x
                ps.pose.position.y = y
                ps.pose.orientation.w = 1.0
                path_msg.poses.append(ps)
            if 1 <= c.path_id <= len(self._alt_path_pubs):
                self._alt_path_pubs[c.path_id - 1].publish(path_msg)
            if c.state == CandidateState.ACTIVE:
                self._active_path_pub.publish(path_msg)

            m = Marker()
            m.header.frame_id = self._map_frame
            m.header.stamp = stamp
            m.ns = 'path_bank'
            m.id = c.path_id
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.12 if c.state == CandidateState.ACTIVE else 0.06
            if c.state == CandidateState.ACTIVE:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.9, 0.2, 1.0
            elif c.state == CandidateState.BLOCKED:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.9, 0.1, 0.1, 0.6
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.5, 0.9, 0.7
            m.pose.orientation.w = 1.0
            from geometry_msgs.msg import Point as GeoPoint
            for x, y in c.points[:: max(1, len(c.points) // 200)]:
                pt = GeoPoint()
                pt.x, pt.y, pt.z = x, y, 0.15
                m.points.append(pt)
            markers.markers.append(m)
        self._marker_pub.publish(markers)


def _yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathBankManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
