#!/usr/bin/env python3
"""Path-bank manager — mission-scoped alternative routes for Husky Nav2.

Authority model
---------------
* Nav2 (NavFn + IsPathValid BT + RPP) still owns follow/replan *timing*.
* This node owns the *candidate set*: generate once, rank, mark blocked,
  select next valid candidate when the active route is collision-invalid on
  the live global costmap.

It does NOT replan every second. Validity is checked at ``check_hz`` (default
2 Hz). A switch only happens when the active bank path intersects lethal /
inscribed cost on the live costmap.

On switch:
  1. Mark current candidate BLOCKED for this mission
  2. Pick next highest-scored AVAILABLE candidate that is still valid live
  3. Cancel NavigateToPose and re-send the same goal (NavFn then plans around
     the already-sensed obstacle; bank state prevents treating a blocked
     corridor as a preferred option in status/logging)
  If no candidate remains valid → publish NO_PATH_AVAILABLE and cancel nav.

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
        self.declare_parameter('cancel_on_no_path', False)

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
        self._cancel_on_no_path = _as_bool(self.get_parameter('cancel_on_no_path').value)
        self._bank_ready_at: Optional[float] = None
        # Ignore NavigateToPose result callbacks from goals we intentionally
        # superseded during a bank switch (preempt/abort is expected).
        self._ignore_goal_results = 0

        self._cfg = PathBankConfig(
            max_candidates=int(self.get_parameter('max_candidates').value),
            exclusion_radius_m=float(self.get_parameter('exclusion_radius_m').value),
            min_path_separation_m=float(self.get_parameter('min_path_separation_m').value),
            min_clearance_m=float(self.get_parameter('min_clearance_m').value),
            length_weight=float(self.get_parameter('length_weight').value),
            clearance_weight=float(self.get_parameter('clearance_weight').value),
            cost_weight=float(self.get_parameter('cost_weight').value),
            smoothness_weight=float(self.get_parameter('smoothness_weight').value),
            clearance_ref_m=float(self.get_parameter('clearance_ref_m').value),
        )
        self._footprint_inflate = float(self.get_parameter('footprint_inflate_m').value)

        self._prior_map: Optional[OccupancyGrid] = None
        self._live_costmap: Optional[Costmap] = None
        self._bank: Optional[PathBank] = None
        self._state = MissionNavState.IDLE
        self._goal_xyyaw: Optional[Tuple[float, float, float]] = None
        self._goal_handle = None
        self._switch_count = 0
        self._building = False

        self.create_subscription(OccupancyGrid, 'prior_map', self._on_prior, LATCHED)
        self.create_subscription(
            Costmap, 'global_costmap/costmap_raw', self._on_costmap, 2)
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

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        if not self._enabled:
            return
        yaw = _yaw_from_quat(msg.pose.orientation)
        self._goal_xyyaw = (msg.pose.position.x, msg.pose.position.y, yaw)
        self._set_state(MissionNavState.PLANNING)
        self._bank = None
        self._switch_count = 0
        self.get_logger().info(
            f'Goal pose received ({self._goal_xyyaw[0]:.2f}, {self._goal_xyyaw[1]:.2f}); '
            'will build path bank on next plan/costmap'
        )

    def _on_plan(self, msg: Path) -> None:
        if not self._enabled or self._building:
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

    def _on_timer(self) -> None:
        if not self._enabled:
            return
        if self._bank is None or self._live_costmap is None:
            return
        if self._state not in (
            MissionNavState.FOLLOWING_PATH,
            MissionNavState.PATH_BLOCKED,
        ):
            return
        # Allow StaticLayer + ObstacleLayer + inflation to settle before the
        # bank starts invalidating candidates (clean-start false positives).
        if self._bank_ready_at is not None:
            if (time.monotonic() - self._bank_ready_at) < self._grace_s:
                return

        active = self._bank.active()
        if active is None:
            chosen = self._bank.select_best_available()
            if chosen is not None:
                self._publish_bank_paths()
            return

        live = nav2_costmap_to_grid(self._live_costmap)
        ok, reason = path_is_valid(
            active.points,
            live,
            lethal_threshold=self._cfg.lethal_threshold,
            sample_stride=self._cfg.sample_stride,
        )
        if ok:
            return

        self.get_logger().warn(
            f'Active Path {active.path_id} invalid on live costmap: {reason}'
        )
        self._set_state(MissionNavState.PATH_BLOCKED)
        self._bank.mark_blocked(active.path_id, reason)
        self._publish_bank_paths()

        if not self._auto_switch:
            return

        self._set_state(MissionNavState.SWITCHING_PATH)
        nxt = self._pick_next_valid(live)
        if nxt is None:
            # Do NOT cancel Nav2 by default: IsPathValid BT still owns recovery
            # / replanning on the live costmap. Bank exhaustion is advisory.
            self.get_logger().error(
                'No valid banked path remains; leaving Nav2 IsPathValid BT in control'
            )
            self._set_state(MissionNavState.NO_PATH_AVAILABLE)
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
            if self._goal_handle is not None:
                self._ignore_goal_results += 1
            self._send_goal(*self._goal_xyyaw)
        self._set_state(MissionNavState.FOLLOWING_PATH)

    # -- bank construction -------------------------------------------------

    def _build_bank(self, start: Tuple[float, float], goal: Tuple[float, float]) -> None:
        self._building = True
        self._set_state(MissionNavState.PLANNING)
        try:
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
            self._set_state(MissionNavState.FOLLOWING_PATH)
        finally:
            self._building = False

    def _pick_next_valid(self, live_grid) -> Optional[PathCandidate]:
        # Re-validate remaining AVAILABLE candidates against live costmap.
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
        lines: List[str] = [f'state={self._state.value}', f'switches={self._switch_count}']
        if self._bank is not None:
            lines.append(f'active_id={self._bank.active_id}')
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
