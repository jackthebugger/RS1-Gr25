"""Saved-map navigation with thermal fire obstacles."""

from __future__ import annotations

import math
import struct
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, TransformStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, PointCloud2, PointField
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster


class DecisionMaking(Node):
    """Send a goal through Nav2 and mark detected fires in its costmaps."""

    def __init__(self) -> None:
        super().__init__('decision_making')
        self.declare_parameter('goal_x', -4.5)
        self.declare_parameter('goal_y', -4.5)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('initial_x', -18.0)
        self.declare_parameter('initial_y', 3.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('fire_threshold_kelvin', 400.0)
        self.declare_parameter('thermal_resolution', 0.01)
        self.declare_parameter('thermal_hfov_degrees', 60.0)
        self.declare_parameter('fire_distance_fallback', 2.0)
        self.declare_parameter('fire_topic', 'fire_detected')
        self.declare_parameter('thermal_topic', 'thermal/image')
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('fire_obstacle_topic', 'fire_obstacles')

        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.goal_yaw = float(self.get_parameter('goal_yaw').value)
        self.initial_x = float(self.get_parameter('initial_x').value)
        self.initial_y = float(self.get_parameter('initial_y').value)
        self.initial_yaw = float(self.get_parameter('initial_yaw').value)
        self.threshold = float(self.get_parameter('fire_threshold_kelvin').value)
        self.resolution = float(self.get_parameter('thermal_resolution').value)
        self.hfov = math.radians(float(self.get_parameter('thermal_hfov_degrees').value))
        self.fallback_distance = float(self.get_parameter('fire_distance_fallback').value)
        self.fire_topic = str(self.get_parameter('fire_topic').value)
        self.thermal_topic = str(self.get_parameter('thermal_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        obstacle_topic = str(self.get_parameter('fire_obstacle_topic').value)

        self.latest_scan: Optional[LaserScan] = None
        self.initial_odom = None
        self.tf_broadcaster = TransformBroadcaster(self)
        self.fire_active = False
        self.goal_sent = False
        self.startup_ticks = 0
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.fire_pub = self.create_publisher(Bool, self.fire_topic, 10)
        self.obstacle_pub = self.create_publisher(PointCloud2, obstacle_topic, 10)
        self.create_subscription(Image, self.thermal_topic, self._thermal_callback, 10)
        self.create_subscription(LaserScan, self.scan_topic, self._scan_callback, 10)
        self.create_subscription(Odometry, 'odometry', self._odometry_callback, 10)
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_timer(0.1, self._publish_startup_tf)
        self.create_timer(1.0, self._startup_tick)
        self.get_logger().info(
            f'Decision mode active: goal=({self.goal_x:.2f}, {self.goal_y:.2f}) '
            f'using thermal={self.thermal_topic}, obstacles={obstacle_topic}'
        )

    def _startup_tick(self) -> None:
        if self.goal_sent:
            return
        self.startup_ticks += 1
        self._publish_initial_pose()
        if self.startup_ticks < 25:
            return
        if not self.action_client.wait_for_server(timeout_sec=0.1):
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'husky1_map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.goal_x
        goal.pose.pose.position.y = self.goal_y
        goal.pose.pose.orientation = self._quaternion(self.goal_yaw)
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response)
        self.goal_sent = True
        self.get_logger().info('Sent saved-map navigation goal to Nav2')

    def _publish_initial_pose(self) -> None:
        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'husky1_map'
        message.pose.pose.position.x = self.initial_x
        message.pose.pose.position.y = self.initial_y
        message.pose.pose.orientation = self._quaternion(self.initial_yaw)
        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = 0.0685
        self.initial_pose_pub.publish(message)

    def _goal_response(self, future) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('Nav2 rejected the decision-making goal')
            return
        self.get_logger().info('Nav2 accepted the decision-making goal')

    def _scan_callback(self, message: LaserScan) -> None:
        self.latest_scan = message

    def _odometry_callback(self, message: Odometry) -> None:
        pose = message.pose.pose
        odom_yaw = self._yaw(pose.orientation)
        if self.initial_odom is None:
            self.initial_odom = (pose.position.x, pose.position.y, odom_yaw)
        initial_x, initial_y, initial_yaw = self.initial_odom
        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = 'husky1_map'
        transform.child_frame_id = 'husky1_odom'
        transform.transform.translation.x = self.initial_x - initial_x
        transform.transform.translation.y = self.initial_y - initial_y
        transform.transform.rotation = self._quaternion(self.initial_yaw - initial_yaw)
        self.tf_broadcaster.sendTransform(transform)

        odom_transform = TransformStamped()
        odom_transform.header.stamp = message.header.stamp
        odom_transform.header.frame_id = 'husky1_odom'
        odom_transform.child_frame_id = 'husky1_base_link'
        odom_transform.transform.translation.x = pose.position.x
        odom_transform.transform.translation.y = pose.position.y
        odom_transform.transform.translation.z = pose.position.z
        odom_transform.transform.rotation = pose.orientation
        self.tf_broadcaster.sendTransform(odom_transform)

    def _publish_startup_tf(self) -> None:
        """Keep Nav2's TF buffer connected while Gazebo odometry starts."""
        stamp = self.get_clock().now().to_msg()
        map_odom = TransformStamped()
        map_odom.header.stamp = stamp
        map_odom.header.frame_id = 'husky1_map'
        map_odom.child_frame_id = 'husky1_odom'
        self.tf_broadcaster.sendTransform(map_odom)

        odom_base = TransformStamped()
        odom_base.header.stamp = stamp
        odom_base.header.frame_id = 'husky1_odom'
        odom_base.child_frame_id = 'husky1_base_link'
        odom_base.transform.translation.x = self.initial_x
        odom_base.transform.translation.y = self.initial_y
        odom_base.transform.rotation = self._quaternion(self.initial_yaw)
        self.tf_broadcaster.sendTransform(odom_base)

    def _thermal_callback(self, message: Image) -> None:
        temperatures = self._decode_image(message)
        if temperatures is None:
            return
        hot = temperatures >= self.threshold
        active = bool(np.count_nonzero(hot))
        self.fire_pub.publish(Bool(data=active))
        if not active:
            if self.fire_active:
                self.get_logger().info('Fire cleared; removing fire obstacle')
            self.fire_active = False
            self._publish_obstacles(message.header.stamp.sec, message.header.stamp.nanosec, [])
            return

        rows, columns = np.nonzero(hot)
        center_x = float(np.mean(columns))
        bearing = (center_x / max(1, message.width - 1) - 0.5) * self.hfov
        distance = self._distance_at_bearing(bearing)
        self.fire_active = True
        self._publish_obstacles(
            message.header.stamp.sec,
            message.header.stamp.nanosec,
            self._fire_points(distance, bearing),
        )
        self.get_logger().info(
            f'Fire obstacle at range {distance:.2f} m, bearing {math.degrees(bearing):.1f} deg',
            throttle_duration_sec=5.0,
        )

    def _distance_at_bearing(self, bearing: float) -> float:
        scan = self.latest_scan
        if scan is None or not scan.ranges:
            return self.fallback_distance
        candidates = []
        for index, distance in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment
            delta = math.atan2(math.sin(angle - bearing), math.cos(angle - bearing))
            if abs(delta) <= math.radians(3.0) and math.isfinite(distance):
                if scan.range_min <= distance <= scan.range_max:
                    candidates.append(distance)
        return min(candidates) if candidates else self.fallback_distance

    @staticmethod
    def _fire_points(distance: float, bearing: float):
        center_x = distance * math.cos(bearing)
        center_y = distance * math.sin(bearing)
        points = []
        for offset_x, offset_y in ((0.0, 0.0), (0.45, 0.0), (-0.45, 0.0), (0.0, 0.45), (0.0, -0.45)):
            points.append((center_x + offset_x, center_y + offset_y, 0.1))
        return points

    def _publish_obstacles(self, seconds: int, nanoseconds: int, points) -> None:
        message = PointCloud2()
        message.header.stamp.sec = seconds
        message.header.stamp.nanosec = nanoseconds
        message.header.frame_id = 'husky1_base_link'
        message.height = 1
        message.width = len(points)
        message.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = message.point_step * message.width
        message.is_dense = True
        message.data = b''.join(struct.pack('<fff', *point) for point in points)
        self.obstacle_pub.publish(message)

    def _decode_image(self, message: Image) -> Optional[np.ndarray]:
        try:
            encoding = message.encoding.lower()
            if '16' in encoding or encoding == 'l16':
                values = np.frombuffer(message.data, dtype=np.uint16)
                if message.is_bigendian:
                    values = values.byteswap()
                return values[:message.height * message.width].reshape((message.height, message.width)) * self.resolution
            if '8' in encoding:
                values = np.frombuffer(message.data, dtype=np.uint8)
                return values[:message.height * message.width].reshape((message.height, message.width)) * self.resolution
        except (ValueError, TypeError):
            self.get_logger().warning('Could not decode thermal image')
        return None

    @staticmethod
    def _quaternion(yaw: float) -> Quaternion:
        return Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))

    @staticmethod
    def _yaw(quaternion: Quaternion) -> float:
        return math.atan2(
            2.0 * quaternion.w * quaternion.z,
            1.0 - 2.0 * quaternion.z * quaternion.z,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DecisionMaking()
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
