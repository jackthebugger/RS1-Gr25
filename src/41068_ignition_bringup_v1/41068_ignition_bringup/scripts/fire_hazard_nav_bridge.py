#!/usr/bin/env python3
"""Publish configured / thermally inferred fire hazards as a PointCloud2.

Nav2 ObstacleLayers subscribe to ``fire_hazards`` and treat the points as
lethal obstacles. This bridges B.E.E.R. fire detection into the costmaps
without inventing a second planner.

Sources of hazard points (union):
1. ``known_fire_poses`` - world/map-frame XY positions of simulated fires
   (always marked, so thin flames still create a keep-out even if lidar
   intermittently misses them).
2. Thermal ``/fire_detected`` - when true, marks a disc ahead of the robot
   using the nearest lidar return in a forward cone (live detection path).
"""

from __future__ import annotations

import math
import struct
from typing import List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Bool, Header
import tf2_ros


def _pack_xyz_cloud(header: Header, points: Sequence[Tuple[float, float, float]]) -> PointCloud2:
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
    for x, y, z in points:
        buf += struct.pack('<fff', float(x), float(y), float(z))
    msg.data = bytes(buf)
    return msg


def _disc_points(cx: float, cy: float, radius: float, step: float) -> List[Tuple[float, float, float]]:
    pts: List[Tuple[float, float, float]] = []
    if radius <= 0.0:
        return [(cx, cy, 0.0)]
    r = 0.0
    while r <= radius + 1e-9:
        circumference = max(2.0 * math.pi * r, step)
        n = max(1, int(round(circumference / step)))
        for i in range(n):
            ang = 2.0 * math.pi * i / n
            pts.append((cx + r * math.cos(ang), cy + r * math.sin(ang), 0.0))
        r += step
    return pts


class FireHazardNavBridge(Node):
    def __init__(self) -> None:
        super().__init__('fire_hazard_nav_bridge')

        self.declare_parameter('robot_name', 'husky1')
        self.declare_parameter('map_frame', '')
        self.declare_parameter('base_frame', '')
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('fire_detected_topic', '/fire_detected')
        self.declare_parameter('hazard_topic', 'fire_hazards')
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('known_fire_poses', [0.0])  # length<2 → no known fires
        self.declare_parameter('hazard_radius', 0.9)
        self.declare_parameter('hazard_step', 0.25)
        self.declare_parameter('thermal_cone_half_angle', 0.6)  # rad
        self.declare_parameter('thermal_max_range', 8.0)
        self.declare_parameter('thermal_default_range', 3.0)

        robot = str(self.get_parameter('robot_name').value).strip().strip('/') or 'husky1'
        map_frame = str(self.get_parameter('map_frame').value).strip() or f'{robot}_map'
        base_frame = str(self.get_parameter('base_frame').value).strip() or f'{robot}_base_link'
        self.map_frame = map_frame
        self.base_frame = base_frame
        self.hazard_radius = float(self.get_parameter('hazard_radius').value)
        self.hazard_step = float(self.get_parameter('hazard_step').value)
        self.cone_half = float(self.get_parameter('thermal_cone_half_angle').value)
        self.thermal_max_range = float(self.get_parameter('thermal_max_range').value)
        self.thermal_default_range = float(self.get_parameter('thermal_default_range').value)

        raw_poses = list(self.get_parameter('known_fire_poses').value)
        self.known_fires: List[Tuple[float, float]] = []
        for i in range(0, len(raw_poses) - 1, 2):
            self.known_fires.append((float(raw_poses[i]), float(raw_poses[i + 1])))

        self.fire_detected = False
        self.latest_scan: Optional[LaserScan] = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').value,
            self._on_scan,
            sensor_qos,
        )
        self.create_subscription(
            Bool,
            self.get_parameter('fire_detected_topic').value,
            self._on_fire,
            10,
        )
        self.pub = self.create_publisher(
            PointCloud2,
            self.get_parameter('hazard_topic').value,
            10,
        )

        period = 1.0 / max(0.5, float(self.get_parameter('publish_rate_hz').value))
        self.create_timer(period, self._tick)
        self.get_logger().info(
            f'Fire hazard bridge ready: {len(self.known_fires)} known fire(s), '
            f'frame={self.map_frame}, topic={self.get_parameter("hazard_topic").value}'
        )

    def _on_scan(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def _on_fire(self, msg: Bool) -> None:
        self.fire_detected = bool(msg.data)

    def _lookup_base(self) -> Optional[TransformStamped]:
        try:
            return self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()
            )
        except Exception:
            return None

    def _yaw_from_quat(self, q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _thermal_hazard_xy(self, bx: float, by: float, yaw: float) -> Optional[Tuple[float, float]]:
        if not self.fire_detected:
            return None
        rng = self.thermal_default_range
        scan = self.latest_scan
        if scan is not None and scan.ranges:
            best = None
            angle = scan.angle_min
            for r in scan.ranges:
                if scan.range_min < r < min(scan.range_max, self.thermal_max_range):
                    if abs(angle) <= self.cone_half:
                        if best is None or r < best:
                            best = r
                angle += scan.angle_increment
            if best is not None:
                rng = best
        return (bx + rng * math.cos(yaw), by + rng * math.sin(yaw))

    def _tick(self) -> None:
        points: List[Tuple[float, float, float]] = []
        for fx, fy in self.known_fires:
            points.extend(_disc_points(fx, fy, self.hazard_radius, self.hazard_step))

        tf = self._lookup_base()
        if tf is not None:
            bx = tf.transform.translation.x
            by = tf.transform.translation.y
            yaw = self._yaw_from_quat(tf.transform.rotation)
            thermal_xy = self._thermal_hazard_xy(bx, by, yaw)
            if thermal_xy is not None:
                points.extend(
                    _disc_points(thermal_xy[0], thermal_xy[1], self.hazard_radius, self.hazard_step)
                )

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.map_frame
        self.pub.publish(_pack_xyz_cloud(header, points))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FireHazardNavBridge()
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
