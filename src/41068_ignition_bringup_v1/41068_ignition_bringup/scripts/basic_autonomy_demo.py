#!/usr/bin/env python3
"""Autonomous Husky navigation demonstration.

One command starts the simulation, waits for sensors / SLAM / Nav2, sends a
goal, and reports whether the robot arrived. Every wait is bounded; the
simulation is torn down as soon as the mission is decided.

    # start at the origin, drive south in simple_trees (default)
    python3 scripts/basic_autonomy_demo.py

    # configure start and goal
    python3 scripts/basic_autonomy_demo.py \
        --start 0 0 0 --goal 0 -5 0 --world simple_trees

    # inject a real obstacle mid-route and require a replan
    python3 scripts/basic_autonomy_demo.py --replan

    # attach to a simulation that is already running
    python3 scripts/basic_autonomy_demo.py --attach --goal 8 6 0

    # original course random-walk (requires sim + Nav2 already up)
    python3 scripts/basic_autonomy_demo.py --attach --mode random_walk

RViz "Nav2 Goal" still works independently of this script: click a pose on the
map in frame husky1_map.
"""

from __future__ import annotations

import argparse
import math
import os
import random
import sys
import time
from typing import Optional, Tuple

import numpy as np

# Allow `python3 scripts/basic_autonomy_demo.py` from a source tree that has
# not been sourced yet: the package root (parent of scripts/) is on sys.path.
_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Image
import tf2_ros

from rs1_nav import (
    GazeboWorld,
    MissionRunner,
    NavObserver,
    PathBlocker,
    init_ros,
)
from rs1_nav.sim import bringup, log


DEFAULT_GOALS = {
    'simple_trees': (0.0, -5.0, 0.0),
    'large_demo': (8.0, 6.0, 0.0),
}
# Replan needs a longer southbound run so the mid-path wall still leaves
# open ground between the barrier and the goal.
DEFAULT_REPLAN_GOALS = {
    'simple_trees': (0.0, -6.0, 0.0),
    'large_demo': (8.0, 6.0, 0.0),
}


# ---------------------------------------------------------------------------
# Original course random-walk node (preserved as mission_mode:=random_walk)
# ---------------------------------------------------------------------------

class BasicAutonomyDemo(Node):
    """A tiny map-aware random-walk autonomy example."""

    def __init__(self):
        super().__init__('basic_autonomy_demo')

        self.declare_parameter('robot_name', '')
        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('image_topic', 'camera/image')
        self.declare_parameter('navigate_action', 'navigate_to_pose')

        self.declare_parameter('close_goal_min_distance', 4.0)
        self.declare_parameter('close_goal_max_distance', 7.0)
        self.declare_parameter('far_goal_min_distance', 7.0)
        self.declare_parameter('far_goal_max_distance', 15.0)
        self.declare_parameter('bright_image_threshold', 0.25)

        self.declare_parameter('free_cell_threshold', 20)
        self.declare_parameter('occupied_margin_cells', 4)
        self.declare_parameter('goal_pause_seconds', 1.0)

        self.robot_name = self._resolve_robot_name()
        self.map_frame = f'{self.robot_name}_map'
        self.base_frame = f'{self.robot_name}_base_link'

        self.map_msg: Optional[OccupancyGrid] = None
        self.map_array: Optional[np.ndarray] = None
        self.latest_brightness: Optional[float] = None
        self.last_image_process_time = 0.0

        self.goal_active = False
        self.goal_handle = None
        self.next_goal_time = 0.0
        self.last_feedback_log_time = 0.0
        self.last_waiting_log_time = 0.0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.get_parameter('map_topic').value,
            self._map_callback,
            1,
        )
        self.image_sub = self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self._image_callback,
            1,
        )
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            self.get_parameter('navigate_action').value,
        )

        self.timer = self.create_timer(1.0, self._tick)

        self.get_logger().info(
            f'Autonomy demo started for robot "{self.robot_name}". '
            f'Using map frame "{self.map_frame}" and base frame "{self.base_frame}".'
        )

    def _resolve_robot_name(self) -> str:
        configured_name = str(self.get_parameter('robot_name').value).strip().strip('/')
        if configured_name:
            return configured_name
        namespace = self.get_namespace().strip('/')
        if namespace:
            return namespace
        return 'husky1'

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg
        self.map_array = np.asarray(msg.data, dtype=np.int16).reshape(
            (msg.info.height, msg.info.width)
        )

    def _image_callback(self, msg: Image) -> None:
        now = time.monotonic()
        if now - self.last_image_process_time < 1.0:
            return
        self.last_image_process_time = now
        brightness = self._estimate_image_brightness(msg)
        if brightness is not None:
            self.latest_brightness = brightness

    def _estimate_image_brightness(self, msg: Image) -> Optional[float]:
        encoding = msg.encoding.lower()
        channels = None
        channel_order = None
        if encoding in ('rgb8', 'bgr8'):
            channels = 3
            channel_order = encoding
        elif encoding in ('rgba8', 'bgra8'):
            channels = 4
            channel_order = encoding
        elif encoding in ('mono8', '8uc1'):
            channels = 1
            channel_order = encoding
        else:
            self.get_logger().warn(
                f'Unsupported image encoding "{msg.encoding}". Brightness will not be updated.',
                throttle_duration_sec=10.0,
            )
            return None
        try:
            data = np.frombuffer(msg.data, dtype=np.uint8)
            rows = data.reshape((msg.height, msg.step))
        except ValueError:
            self.get_logger().warn('Could not reshape image data. Brightness will not be updated.')
            return None
        stride = max(1, min(msg.width, msg.height) // 80)
        if channels == 1:
            sampled = rows[::stride, :msg.width:stride]
            return float(np.mean(sampled)) / 255.0
        useful_cols = msg.width * channels
        sampled = rows[::stride, :useful_cols].reshape((-1, msg.width, channels))[:, ::stride, :]
        if channel_order.startswith('rgb'):
            rgb = sampled[..., :3]
        elif channel_order.startswith('bgr'):
            rgb = sampled[..., :3][..., ::-1]
        else:
            rgb = sampled[..., :3]
        return float(np.mean(rgb)) / 255.0

    def _tick(self) -> None:
        if self.goal_active:
            return
        if time.monotonic() < self.next_goal_time:
            return
        if not self.nav_client.server_is_ready():
            if not self.nav_client.wait_for_server(timeout_sec=0.1):
                self._log_waiting('Waiting for Nav2 NavigateToPose action server...')
                return
        if self.map_msg is None or self.map_array is None:
            self._log_waiting('Waiting for an OccupancyGrid map...')
            return
        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            self._log_waiting(f'Waiting for TF from {self.map_frame} to {self.base_frame}...')
            return
        goal_xy = self._choose_goal(robot_pose)
        if goal_xy is None:
            self._log_waiting('No suitable free-space waypoint found in the current map yet...')
            return
        self._send_goal(goal_xy)

    def _log_waiting(self, message: str) -> None:
        now = time.monotonic()
        if now - self.last_waiting_log_time > 5.0:
            self.last_waiting_log_time = now
            self.get_logger().info(message)

    def _lookup_robot_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except Exception as exc:
            self.get_logger().debug(f'TF lookup failed: {exc}')
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = self._yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
        return (translation.x, translation.y, yaw)

    def _choose_goal(self, robot_pose: Tuple[float, float, float]) -> Optional[Tuple[float, float, float]]:
        assert self.map_msg is not None
        assert self.map_array is not None
        brightness = self.latest_brightness if self.latest_brightness is not None else 0.5
        bright_threshold = float(self.get_parameter('bright_image_threshold').value)
        if brightness >= bright_threshold:
            min_distance = float(self.get_parameter('far_goal_min_distance').value)
            max_distance = float(self.get_parameter('far_goal_max_distance').value)
            mode = 'farther'
        else:
            min_distance = float(self.get_parameter('close_goal_min_distance').value)
            max_distance = float(self.get_parameter('close_goal_max_distance').value)
            mode = 'closer'
        robot_x, robot_y, _ = robot_pose
        result = self._sample_free_goal(robot_x, robot_y, min_distance, max_distance, max_tries=1500)
        if result is None:
            result = self._sample_free_goal(robot_x, robot_y, 1.0, max(max_distance, 8.0), max_tries=1500)
            mode = 'fallback'
        if result is not None:
            goal_x, goal_y = result
            goal_yaw = random.uniform(-math.pi, math.pi)
            self.get_logger().info(
                f'Brightness={brightness:.2f}; choosing a {mode} free-space goal '
                f'at ({goal_x:.2f}, {goal_y:.2f}).'
            )
            return (goal_x, goal_y, goal_yaw)
        return None

    def _sample_free_goal(
        self,
        robot_x: float,
        robot_y: float,
        min_distance: float,
        max_distance: float,
        max_tries: int,
    ) -> Optional[Tuple[float, float]]:
        assert self.map_msg is not None
        assert self.map_array is not None
        width = self.map_msg.info.width
        height = self.map_msg.info.height
        margin = int(self.get_parameter('occupied_margin_cells').value)
        if width <= 2 * margin or height <= 2 * margin:
            return None
        for _ in range(max_tries):
            ix = random.randint(margin, width - margin - 1)
            iy = random.randint(margin, height - margin - 1)
            if not self._is_free_with_margin(ix, iy, margin):
                continue
            wx, wy = self._map_cell_to_world(ix, iy)
            distance = math.hypot(wx - robot_x, wy - robot_y)
            if min_distance <= distance <= max_distance:
                return (wx, wy)
        return None

    def _is_free_with_margin(self, ix: int, iy: int, margin: int) -> bool:
        assert self.map_array is not None
        free_threshold = int(self.get_parameter('free_cell_threshold').value)
        window = self.map_array[iy - margin:iy + margin + 1, ix - margin:ix + margin + 1]
        if np.any(window < 0):
            return False
        if np.any(window > free_threshold):
            return False
        return True

    def _map_cell_to_world(self, ix: int, iy: int) -> Tuple[float, float]:
        assert self.map_msg is not None
        info = self.map_msg.info
        resolution = info.resolution
        origin = info.origin
        local_x = (ix + 0.5) * resolution
        local_y = (iy + 0.5) * resolution
        origin_yaw = self._yaw_from_quaternion(
            origin.orientation.x,
            origin.orientation.y,
            origin.orientation.z,
            origin.orientation.w,
        )
        cos_yaw = math.cos(origin_yaw)
        sin_yaw = math.sin(origin_yaw)
        world_x = origin.position.x + cos_yaw * local_x - sin_yaw * local_y
        world_y = origin.position.y + sin_yaw * local_x + cos_yaw * local_y
        return (world_x, world_y)

    def _send_goal(self, goal_xy_yaw: Tuple[float, float, float]) -> None:
        x, y, yaw = goal_xy_yaw
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self.map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        qx, qy, qz, qw = self._quaternion_from_yaw(yaw)
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw
        self.goal_active = True
        self.last_feedback_log_time = 0.0
        self.get_logger().info(f'Sending Nav2 goal: ({x:.2f}, {y:.2f}) in frame {self.map_frame}.')
        send_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback,
        )
        send_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future) -> None:
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn('Nav2 goal was rejected.')
            self.goal_active = False
            self.next_goal_time = time.monotonic() + float(self.get_parameter('goal_pause_seconds').value)
            return
        self.get_logger().info('Nav2 goal accepted. Waiting for result...')
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg) -> None:
        now = time.monotonic()
        if now - self.last_feedback_log_time < 5.0:
            return
        self.last_feedback_log_time = now
        feedback = feedback_msg.feedback
        distance = getattr(feedback, 'distance_remaining', None)
        if distance is not None:
            self.get_logger().info(f'Nav2 feedback: distance remaining {distance:.2f} m.')

    def _result_callback(self, future) -> None:
        wrapped_result = future.result()
        status = wrapped_result.status
        status_text = self._goal_status_to_text(status)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Nav2 goal finished: {status_text}.')
        else:
            self.get_logger().warn(f'Nav2 goal finished: {status_text}. Choosing another goal soon.')
        self.goal_active = False
        self.goal_handle = None
        self.next_goal_time = time.monotonic() + float(self.get_parameter('goal_pause_seconds').value)

    @staticmethod
    def _goal_status_to_text(status: int) -> str:
        names = {
            GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
            GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
            GoalStatus.STATUS_EXECUTING: 'EXECUTING',
            GoalStatus.STATUS_CANCELING: 'CANCELING',
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }
        return names.get(status, f'UNRECOGNISED_STATUS_{status}')

    @staticmethod
    def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
        half_yaw = 0.5 * yaw
        return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


# ---------------------------------------------------------------------------
# Structured start → goal (and optional replan) mission
# ---------------------------------------------------------------------------

def _goal_timeout(straight_line: float) -> float:
    return 45.0 + straight_line / 0.25


def _run_structured_mission(args: argparse.Namespace) -> int:
    robot = args.robot
    goal = (args.goal[0], args.goal[1], args.goal[2])
    replan = args.mode == 'replan' or args.replan

    init_ros(robot)
    observer = NavObserver(robot=robot, node_name='basic_autonomy_demo')
    mission = MissionRunner(observer, logger=log)
    blocker = None
    world = None
    try:
        if not mission.wait_until_ready(timeout=args.startup_timeout):
            log('FAIL  navigation stack did not become ready')
            return 1

        if replan:
            world = GazeboWorld(args.world, logger=lambda m: log(f'  {m}'))
            if not world.wait_until_available(max_wait=30.0):
                log('FAIL  Gazebo world services unavailable; cannot inject an obstacle')
                return 1
            blocker = PathBlocker(
                observer, world,
                width=4.0,        # longer wall
                thickness=0.6,
                height=2.0,
                min_travel=1.0,   # wait until robot has moved 2 m
                look_ahead=3.0,   # place wall 3 m ahead on path
                logger=lambda m: log(f'  {m}'),
            )

        def on_tick(elapsed: float, report) -> None:
            remaining = observer.distance_remaining()
            extra = f', Nav2 remaining {remaining:.2f} m' if remaining is not None else ''
            log(f'  t={elapsed:.0f}s travelled {report.distance_travelled:.2f} m'
                f', {report.plans_received} plan(s){extra}')
            if blocker is not None:
                blocker.maybe_inject(elapsed, report)

        here = observer.robot_pose()
        straight = 5.0 if here is None else math.hypot(goal[0] - here[0], goal[1] - here[1])
        timeout = args.timeout if args.timeout > 0 else _goal_timeout(straight)
        if replan and args.timeout <= 0:
            timeout = max(timeout, 90.0)
        report = mission.run(
            goal,
            timeout=timeout,
            on_tick=on_tick,
        )

        log('=' * 68)
        log(report.summary())
        if replan:
            if blocker is None or not blocker.injected:
                log('FAIL  replan demo: obstacle was never inserted')
                return 1
            if not report.replans:
                log('FAIL  replan demo: obstacle was inserted but Nav2 never rerouted')
                return 1
            log(f'OK    {len(report.replans)} genuine replan(s) after the barrier appeared')
        if not report.reached:
            log('DEMO FAILED: robot did not reach the goal')
            return 1
        log('DEMO PASSED: robot reached the goal')
        return 0
    finally:
        if world is not None and blocker is not None and blocker.spec is not None:
            world.remove_model(blocker.spec.name)
        observer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _run_random_walk() -> int:
    rclpy.init()
    node = BasicAutonomyDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def _parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Autonomous Husky navigation demonstration.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument('--mode', choices=('single_goal', 'replan', 'random_walk'),
                        default='single_goal',
                        help='single_goal (default), replan, or the original random_walk')
    parser.add_argument('--replan', action='store_true',
                        help='Shorthand for --mode replan: inject an obstacle mid-route')
    parser.add_argument('--world', default='simple_trees',
                        choices=('simple_trees', 'large_demo'))
    parser.add_argument('--robot', default='husky1')
    parser.add_argument('--start', nargs=3, type=float, metavar=('X', 'Y', 'YAW'),
                        default=[0.0, 0.0, 0.0],
                        help='Husky spawn pose (metres, radians). Ignored with --attach.')
    parser.add_argument('--goal', nargs=3, type=float, metavar=('X', 'Y', 'YAW'),
                        default=None,
                        help='Goal pose in husky1_map. Default depends on --world.')
    parser.add_argument('--rviz', action='store_true', help='Open RViz with the launch')
    parser.add_argument('--gui', action='store_true',
                        help='Open the Gazebo GUI (off by default for reliability)')
    parser.add_argument('--attach', action='store_true',
                        help='Do not launch simulation; attach to one that is already running')
    parser.add_argument('--timeout', type=float, default=0.0,
                        help='Mission timeout in seconds (0 = derived from distance)')
    parser.add_argument('--startup-timeout', type=float, default=180.0,
                        help='Seconds to wait for the navigation stack to come up')
    args, _unknown = parser.parse_known_args(argv)
    if args.replan:
        args.mode = 'replan'
    if args.goal is None:
        table = DEFAULT_REPLAN_GOALS if args.mode == 'replan' else DEFAULT_GOALS
        args.goal = list(table[args.world])
    return args


def main(args=None) -> int:
    parsed = _parse_args(args)

    if parsed.mode == 'random_walk':
        log('random_walk mode: attaching as a ROS node (simulation must already be running)')
        return _run_random_walk()

    if parsed.attach:
        log(f'Attaching to running simulation, mode={parsed.mode}, '
            f'goal=({parsed.goal[0]:.2f}, {parsed.goal[1]:.2f}, {parsed.goal[2]:.2f})')
        return _run_structured_mission(parsed)

    log(f'Starting simulation: world={parsed.world} start='
        f'({parsed.start[0]:.2f}, {parsed.start[1]:.2f}, {parsed.start[2]:.2f}) '
        f'goal=({parsed.goal[0]:.2f}, {parsed.goal[1]:.2f}, {parsed.goal[2]:.2f}) '
        f'mode={parsed.mode}')

    sup = bringup(
        world=parsed.world,
        nav2=True,
        rviz=parsed.rviz,
        gui=parsed.gui,
        husky_x=parsed.start[0],
        husky_y=parsed.start[1],
        husky_yaw=parsed.start[2],
        max_runtime=max(parsed.startup_timeout + 300.0, 600.0),
        log_path='/tmp/basic_autonomy_demo.log',
    )
    with sup:
        if not sup.is_alive():
            log('FAIL  simulation launch exited immediately')
            return 1
        return _run_structured_mission(parsed)


if __name__ == '__main__':
    sys.exit(main())
