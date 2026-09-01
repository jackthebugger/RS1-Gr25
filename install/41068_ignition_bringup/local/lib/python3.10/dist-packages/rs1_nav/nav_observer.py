#!/usr/bin/env python3
"""Observation layer: one rclpy node that watches the whole navigation stack.

`NavObserver` subscribes to the lidar, the SLAM map, both Nav2 costmaps, the
global plan and the odometry, and can send NavigateToPose goals. It is the
single place where this project talks to the navigation stack, used by both the
autonomous mission (rs1_nav.mission) and the test suite -- so the tests
exercise the same code that ships.

Two deliberate choices:

* Real subscriptions, not `ros2 topic echo`. Echo truncates long arrays, which
  silently turns a 360-sample laser scan into a wrong answer.
* Every wait is bounded and returns a value instead of blocking, so neither a
  test nor the demo can ever hang waiting on the simulation.
"""

import math
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
import tf2_ros

LATCHED = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
)


@dataclass
class PathSnapshot:
    """One received global plan, with the wall time it arrived."""

    stamp: float
    points: List[Tuple[float, float]] = field(default_factory=list)

    def length(self) -> float:
        return sum(
            math.hypot(b[0] - a[0], b[1] - a[1])
            for a, b in zip(self.points, self.points[1:])
        )


def init_ros(robot: str = 'husky1') -> None:
    """Initialise rclpy with the namespaced TF remaps this stack uses.

    tf2_ros always subscribes to the absolute topics /tf and /tf_static, but
    this package gives every robot its own TF tree under /<robot>/tf. Without
    these remaps no transform is ever received.
    """
    rclpy.init(args=[
        '--ros-args',
        '-r', f'/tf:=/{robot}/tf',
        '-r', f'/tf_static:=/{robot}/tf_static',
    ])


class NavObserver(Node):
    """Observes sensors, costmaps and plans; sends NavigateToPose goals."""

    def __init__(self, robot: str = 'husky1', node_name: str = 'nav_observer'):
        super().__init__(node_name, namespace=f'/{robot}')
        self.robot = robot
        self.map_frame = f'{robot}_map'
        self.odom_frame = f'{robot}_odom'
        self.base_frame = f'{robot}_base_link'

        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.scan: Optional[LaserScan] = None
        self.map: Optional[OccupancyGrid] = None
        self.global_costmap: Optional[Costmap] = None
        self.local_costmap: Optional[Costmap] = None
        self.paths: List[PathSnapshot] = []
        self.odom: Optional[Odometry] = None
        self.cmd_vel_count = 0
        self.peak_commanded_speed = 0.0

        self.create_subscription(LaserScan, 'scan', self._on_scan, 5)
        self.create_subscription(OccupancyGrid, 'map', self._on_map, LATCHED)
        self.create_subscription(Path, 'plan', self._on_plan, 5)
        self.create_subscription(Odometry, 'odom', self._on_odom, 5)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(
            Costmap, 'global_costmap/costmap_raw', self._on_global_costmap, 2)
        self.create_subscription(
            Costmap, 'local_costmap/costmap_raw', self._on_local_costmap, 2)

        # Only used by the movement-chain test, which deliberately drives the
        # robot to prove the actuation path works. Autonomous tests never use it.
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.goal_handle = None
        self.goal_status: Optional[int] = None
        self.last_feedback = None
        self._result_future = None

    # -- callbacks --------------------------------------------------------

    def _on_scan(self, msg: LaserScan) -> None:
        self.scan = msg

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map = msg

    def _on_plan(self, msg: Path) -> None:
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if not points:
            return
        self.paths.append(PathSnapshot(stamp=time.monotonic(), points=points))
        # Keep memory bounded on long missions; the tests only ever compare the
        # newest plan against a snapshot they took themselves.
        if len(self.paths) > 400:
            del self.paths[:200]

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg

    def _on_cmd_vel(self, msg: Twist) -> None:
        self.cmd_vel_count += 1
        speed = math.hypot(msg.linear.x, msg.linear.y) + abs(msg.angular.z)
        self.peak_commanded_speed = max(self.peak_commanded_speed, speed)

    def _on_global_costmap(self, msg: Costmap) -> None:
        self.global_costmap = msg

    def _on_local_costmap(self, msg: Costmap) -> None:
        self.local_costmap = msg

    # -- bounded spinning -------------------------------------------------

    def spin_for(self, seconds: float) -> None:
        """Process callbacks for a fixed duration."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_for(
        self,
        description: str,
        predicate: Callable[[], bool],
        timeout: float,
        on_timeout_detail: Optional[Callable[[], str]] = None,
    ) -> bool:
        """Spin until `predicate` is true or `timeout` seconds pass."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                elapsed = timeout - (deadline - time.monotonic())
                self.log(f'OK    {description} after {elapsed:.1f}s')
                return True
        detail = f' ({on_timeout_detail()})' if on_timeout_detail else ''
        self.log(f'FAIL  {description} -- timeout after {timeout:.1f}s{detail}')
        return False

    def log(self, message: str) -> None:
        print(f'[{time.strftime("%H:%M:%S")}]   {message}', flush=True)

    # -- observations -----------------------------------------------------

    def robot_pose(self, frame: Optional[str] = None) -> Optional[Tuple[float, float, float]]:
        """Robot (x, y, yaw) in `frame` (default: the map frame)."""
        target = frame or self.map_frame
        try:
            tf = self.tf_buffer.lookup_transform(
                target, self.base_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.3))
        except Exception:
            return None
        t, r = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2.0 * (r.w * r.z + r.x * r.y),
                         1.0 - 2.0 * (r.y * r.y + r.z * r.z))
        return (t.x, t.y, yaw)

    def min_range_in_sector(self, centre_angle: float, half_width: float) -> float:
        """Smallest valid lidar range within a bearing sector (robot frame)."""
        scan = self.scan
        if scan is None or not scan.ranges:
            return float('nan')
        best = float('inf')
        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance):
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue
            angle = scan.angle_min + index * scan.angle_increment
            delta = math.atan2(math.sin(angle - centre_angle),
                               math.cos(angle - centre_angle))
            if abs(delta) <= half_width:
                best = min(best, distance)
        return best

    def latest_path(self) -> Optional[PathSnapshot]:
        return self.paths[-1] if self.paths else None

    def odom_pose(self) -> Optional[Tuple[float, float, float]]:
        """Robot pose from the EKF-filtered /odom message."""
        if self.odom is None:
            return None
        p = self.odom.pose.pose
        yaw = math.atan2(2.0 * (p.orientation.w * p.orientation.z),
                         1.0 - 2.0 * (p.orientation.z ** 2))
        return (p.position.x, p.position.y, yaw)

    def drive(self, linear: float, angular: float) -> None:
        """Publish one velocity command (movement-chain test only)."""
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_pub.publish(msg)

    def drive_for(self, linear: float, angular: float, seconds: float,
                  rate_hz: float = 20.0) -> None:
        """Hold a velocity command for a bounded duration, then stop."""
        deadline = time.monotonic() + seconds
        period = 1.0 / rate_hz
        while time.monotonic() < deadline:
            self.drive(linear, angular)
            rclpy.spin_once(self, timeout_sec=period)
        for _ in range(5):
            self.drive(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.05)

    def costmap_cost_at(self, x: float, y: float, *, local: bool = False) -> Optional[int]:
        """Cost (0-255) of the cell containing the world point, or None."""
        grid = self.local_costmap if local else self.global_costmap
        if grid is None:
            return None
        meta = grid.metadata
        ox = meta.origin.position.x
        oy = meta.origin.position.y
        res = meta.resolution
        ix = int((x - ox) / res)
        iy = int((y - oy) / res)
        if not (0 <= ix < meta.size_x and 0 <= iy < meta.size_y):
            return None
        return int(grid.data[iy * meta.size_x + ix])

    def max_cost_near(self, x: float, y: float, radius: float,
                      *, local: bool = False,
                      ignore_unknown: bool = False) -> Optional[int]:
        """Highest cost within `radius` of a world point.

        Used as the evidence that the navigation stack's own environment
        representation registered a new obstacle. Nav2 uses 255 for
        NO_INFORMATION, which is *not* an obstacle; pass ignore_unknown=True
        when the question is "did a real hit get marked?".
        """
        grid = self.local_costmap if local else self.global_costmap
        if grid is None:
            return None
        res = grid.metadata.resolution
        steps = max(1, int(radius / res))
        best = None
        for dx in range(-steps, steps + 1):
            for dy in range(-steps, steps + 1):
                cost = self.costmap_cost_at(x + dx * res, y + dy * res, local=local)
                if cost is None:
                    continue
                if ignore_unknown and cost == 255:
                    continue
                if best is None or cost > best:
                    best = cost
        return best

    def map_known_cell_count(self) -> int:
        """Number of cells SLAM has actually observed (-1 means unknown).

        Counted via the array's own count(), because polling this on a
        500x500 grid with a Python loop is slow enough to distort the timing of
        the very waits that call it.
        """
        if self.map is None:
            return 0
        return len(self.map.data) - self.map.data.count(-1)

    # -- Nav2 goals -------------------------------------------------------

    def nav2_ready(self, timeout: float = 1.0) -> bool:
        return self.nav_client.wait_for_server(timeout_sec=timeout)

    def send_goal(self, x: float, y: float, yaw: float = 0.0) -> bool:
        """Send one NavigateToPose goal. Returns True once Nav2 accepts it."""
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(0.5 * yaw)
        goal.pose.pose.orientation.w = math.cos(0.5 * yaw)

        self.goal_status = None
        self.goal_handle = None
        send_future = self.nav_client.send_goal_async(
            goal, feedback_callback=self._on_feedback)

        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline and not send_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        if not send_future.done():
            self.log('FAIL  Nav2 did not respond to the goal request')
            return False

        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.log('FAIL  Nav2 rejected the goal')
            return False

        self.goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_result)
        self._result_future = result_future
        return True

    def _on_feedback(self, msg) -> None:
        self.last_feedback = msg.feedback

    def _on_result(self, future) -> None:
        wrapped = future.result()
        self.goal_status = wrapped.status if wrapped is not None else GoalStatus.STATUS_UNKNOWN

    def goal_finished(self) -> bool:
        return self.goal_status is not None

    def goal_succeeded(self) -> bool:
        return self.goal_status == GoalStatus.STATUS_SUCCEEDED

    def goal_status_text(self) -> str:
        names = {
            GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
            GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
            GoalStatus.STATUS_EXECUTING: 'EXECUTING',
            GoalStatus.STATUS_CANCELING: 'CANCELING',
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }
        if self.goal_status is None:
            return 'IN_PROGRESS'
        return names.get(self.goal_status, f'STATUS_{self.goal_status}')

    def cancel_goal(self) -> None:
        """Stop the current mission immediately (used as soon as a test passes)."""
        if self.goal_handle is None or self.goal_finished():
            return
        future = self.goal_handle.cancel_goal_async()
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

    def distance_remaining(self) -> Optional[float]:
        if self.last_feedback is None:
            return None
        return getattr(self.last_feedback, 'distance_remaining', None)
