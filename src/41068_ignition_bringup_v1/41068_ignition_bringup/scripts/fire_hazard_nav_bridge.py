#!/usr/bin/env python3
"""Publish configured / thermally inferred fire hazards as a PointCloud2.

Mark ingress only (Entry 026)
-----------------------------
``obstacle_memory_manager`` consumes ``fire_hazards`` to tag typed
``CellSource.FIRE`` keepout cells. LiDAR ObstacleLayer must never observe
these points (shared-layer scan raytrace previously wiped fire marks off
thermal FOV).

This bridge does not clear FIRE memory. Clear requires consecutive camera-frame
ROI-frustum thermal negatives in ``obstacle_memory`` (Entries 027/028).

Thermal-evidence rule (Entry 028)
---------------------------------
A thermal hazard disc is emitted only when **all** of:

* the detector's debounced ``fire_detected`` is True *right now* (no bridge
  side hold — the 8 s hold made the bridge keep marking at a guessed range
  while the robot U-turned onto Path B, spraying FIRE discs behind itself
  and sealing the corridor it was driving);
* a fresh ``fire_bearing`` (hot-blob bearing from the detector) exists;
* a LiDAR return lies along that bearing within ``thermal_max_range``.

No default range: without a LiDAR return along the blob bearing the fire's
position is unknown and nothing is marked (logged, throttled).

Sources of hazard points (union):
1. ``known_fire_poses`` - world/map-frame XY positions of simulated fires.
2. Thermal detection as above.
"""

from __future__ import annotations

import math
import struct
from typing import List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseArray, TransformStamped, Vector3Stamped
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Bool, Header, String
import tf2_ros

try:
    from rs1_interfaces.msg import FireObservation
except ImportError:  # pragma: no cover — build order
    FireObservation = None  # type: ignore

from rs1_nav.fire_bridge import (
    THERMAL_OPTICAL_FORWARD_M,
    HazardDeduper,
    ThermalBearing,
    base_to_map,
    bearing_aims_demo_obstacle,
    near_demo_obstacle,
    thermal_hazard_point,
)


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
        self.declare_parameter('fire_bearing_topic', 'fire_bearing')
        self.declare_parameter('fire_observation_topic', 'fire_observation')
        self.declare_parameter('hazard_topic', 'fire_hazards')
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('known_fire_poses', [0.0])  # length<2 → no known fires
        self.declare_parameter('hazard_radius', 0.9)
        self.declare_parameter('hazard_step', 0.25)
        self.declare_parameter('thermal_max_range', 8.0)
        self.declare_parameter('thermal_min_range', 1.5)
        # Thermal optical origin forward of base_link (camera_joint + sensor pose).
        self.declare_parameter('camera_forward_m', THERMAL_OPTICAL_FORWARD_M)
        # Accept LiDAR returns this far outside the hot-blob angular span.
        self.declare_parameter('bearing_margin_rad', 0.06)
        self.declare_parameter('bearing_max_age_s', 0.5)
        self.declare_parameter('scan_max_age_s', 0.5)
        # Dedupe re-emission of the same disc location within a short window.
        self.declare_parameter('dedupe_separation_m', 1.5)
        self.declare_parameter('dedupe_window_s', 120.0)
        # Status GUI BLOCK FIRE boxes (demo) — ignore thermal mark/pending.
        self.declare_parameter('demo_obstacle_topic', 'demo_obstacle/poses')
        # GUI gap_b thermal marks ~2.8 m from seat (-5.1,21.1) were escaping
        # 2.5 m and thrashing §G rebuild during home (GUI-DB HOLD_REBUILD).
        self.declare_parameter('demo_ignore_radius_m', 4.0)

        robot = str(self.get_parameter('robot_name').value).strip().strip('/') or 'husky1'
        map_frame = str(self.get_parameter('map_frame').value).strip() or f'{robot}_map'
        base_frame = str(self.get_parameter('base_frame').value).strip() or f'{robot}_base_link'
        self.map_frame = map_frame
        self.base_frame = base_frame
        self.hazard_radius = float(self.get_parameter('hazard_radius').value)
        self.hazard_step = float(self.get_parameter('hazard_step').value)
        self.thermal_max_range = float(self.get_parameter('thermal_max_range').value)
        self.thermal_min_range = float(self.get_parameter('thermal_min_range').value)
        self.camera_forward_m = float(self.get_parameter('camera_forward_m').value)
        self.bearing_margin = float(self.get_parameter('bearing_margin_rad').value)
        self.bearing_max_age = float(self.get_parameter('bearing_max_age_s').value)
        self.scan_max_age = float(self.get_parameter('scan_max_age_s').value)
        self._dedupe = HazardDeduper(
            float(self.get_parameter('dedupe_separation_m').value),
            float(self.get_parameter('dedupe_window_s').value),
        )

        raw_poses = list(self.get_parameter('known_fire_poses').value)
        self.known_fires: List[Tuple[float, float]] = []
        for i in range(0, len(raw_poses) - 1, 2):
            self.known_fires.append((float(raw_poses[i]), float(raw_poses[i + 1])))

        self.fire_detected = False
        self.latest_scan: Optional[LaserScan] = None
        self._scan_rx_s = 0.0
        self.latest_bearing: Optional[ThermalBearing] = None
        self._last_no_range_log = 0.0
        self.marks_emitted = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self._on_scan, sensor_qos)
        self.create_subscription(
            Bool, self.get_parameter('fire_detected_topic').value, self._on_fire, 10)
        self.create_subscription(
            Vector3Stamped, self.get_parameter('fire_bearing_topic').value,
            self._on_bearing, 10)
        self._use_fire_observation = False
        self._latest_obs = None
        if FireObservation is not None:
            self.create_subscription(
                FireObservation,
                self.get_parameter('fire_observation_topic').value,
                self._on_fire_observation,
                10,
            )
        self.pub = self.create_publisher(
            PointCloud2, self.get_parameter('hazard_topic').value, 10)
        self.status_pub = self.create_publisher(String, 'fire_hazard_bridge/status', 10)
        # Latched pending-FIRE signal for mission owner (§F).
        self._pending_pub = self.create_publisher(
            Bool, 'mission_owner/pending_fire', QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ))
        self._pending_latched = False
        self._demo_poses: List[Tuple[float, float]] = []
        self._demo_ignore_radius = float(
            self.get_parameter('demo_ignore_radius_m').value)
        demo_topic = str(self.get_parameter('demo_obstacle_topic').value).strip()
        self.create_subscription(
            PoseArray, demo_topic or 'demo_obstacle/poses',
            self._on_demo_poses, QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ))

        period = 1.0 / max(0.5, float(self.get_parameter('publish_rate_hz').value))
        self.create_timer(period, self._tick)
        self.get_logger().info(
            f'Fire hazard bridge ready: {len(self.known_fires)} known fire(s), '
            f'frame={self.map_frame}, topic={self.get_parameter("hazard_topic").value}, '
            'thermal marks require detector positive + bearing + LiDAR range (no default range); '
            f'demo obstacles on {demo_topic or "demo_obstacle/poses"} ignored '
            f'(r={self._demo_ignore_radius:.1f} m)'
        )

    def _on_demo_poses(self, msg: PoseArray) -> None:
        self._demo_poses = [
            (float(p.position.x), float(p.position.y)) for p in msg.poses
        ]

    # -- time helpers ---------------------------------------------------------

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _stamp_s(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    # -- callbacks ------------------------------------------------------------

    def _on_scan(self, msg: LaserScan) -> None:
        self.latest_scan = msg
        self._scan_rx_s = self._now_s()

    def _on_fire(self, msg: Bool) -> None:
        # Detector output is already debounced (positive_required /
        # negative_required frames). No additional hold here.
        self.fire_detected = bool(msg.data)

    def _on_bearing(self, msg: Vector3Stamped) -> None:
        if self._use_fire_observation:
            return  # FireObservation is sole bearing authority when live
        stamp = self._stamp_s(msg.header.stamp)
        if stamp <= 0.0:
            stamp = self._now_s()
        self.latest_bearing = ThermalBearing(
            bearing_rad=float(msg.vector.x),
            half_width_rad=max(0.0, float(msg.vector.y)),
            stamp_s=stamp,
        )

    def _on_fire_observation(self, msg) -> None:
        """Same-frame FireObservation — sole mark/pending thermal authority."""
        self._use_fire_observation = True
        self._latest_obs = msg
        # Debounced flag drives clear/hold; raw+bearing_valid required to mark.
        self.fire_detected = bool(msg.fire_debounced)
        stamp = self._stamp_s(msg.header.stamp)
        if stamp <= 0.0:
            stamp = self._now_s()
        if msg.bearing_valid and msg.raw_hot_blob and msg.fire_debounced:
            self.latest_bearing = ThermalBearing(
                bearing_rad=float(msg.bearing_rad),
                half_width_rad=max(0.0, float(msg.blob_half_width_rad)),
                stamp_s=stamp,
            )
        elif not msg.bearing_valid:
            # Debounce hold / no blob this frame — do not reuse old bearing.
            self.latest_bearing = None

    def _lookup_base(self) -> Optional[TransformStamped]:
        try:
            return self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()
            )
        except Exception:
            return None

    @staticmethod
    def _yaw_from_quat(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _publish_pending_fire(self, pending: bool) -> None:
        if pending == self._pending_latched:
            return
        self._pending_latched = pending
        msg = Bool()
        msg.data = bool(pending)
        self._pending_pub.publish(msg)

    def _thermal_hazard_xy(self, bx: float, by: float, yaw: float) -> Optional[Tuple[float, float]]:
        """Map-frame hazard centre, or None when thermal evidence is incomplete."""
        # FireObservation authority: require raw_hot_blob && fire_debounced &&
        # bearing_valid on the same stamp (plan §B4). Bool alone is ignored.
        if self._use_fire_observation:
            obs = self._latest_obs
            if obs is None or not (
                obs.raw_hot_blob and obs.fire_debounced and obs.bearing_valid
            ):
                return None
        elif not self.fire_detected:
            return None
        now = self._now_s()
        scan = self.latest_scan
        if scan is None or (now - self._scan_rx_s) > self.scan_max_age:
            return None
        pt_base = thermal_hazard_point(
            fire_detected=True,
            bearing=self.latest_bearing,
            now_s=now,
            scan_ranges=scan.ranges,
            angle_min=scan.angle_min,
            angle_increment=scan.angle_increment,
            range_min=scan.range_min,
            range_max=scan.range_max,
            max_range_m=self.thermal_max_range,
            min_range_m=self.thermal_min_range,
            bearing_max_age_s=self.bearing_max_age,
            angular_margin_rad=self.bearing_margin,
            camera_forward_m=self.camera_forward_m,
        )
        if pt_base is None:
            # Unlocalized thermal-positive + bearing → pending FIRE signal for
            # mission owner (§F / THERMAL_FIRE_PENDING). No default range disc.
            # GUI demo BLOCK FIRE boxes are exempt (path_bank uses lidar/costmap).
            b = self.latest_bearing
            if (
                self.fire_detected
                and b is not None
                and (now - b.stamp_s) <= self.bearing_max_age
            ):
                if bearing_aims_demo_obstacle(
                    robot_xy=(bx, by),
                    robot_yaw=yaw,
                    bearing_rad=b.bearing_rad,
                    demos=self._demo_poses,
                    max_range_m=self.thermal_max_range + self._demo_ignore_radius,
                    angular_margin_rad=max(0.20, self.bearing_margin + 0.12),
                ):
                    self._publish_pending_fire(False)
                    if now - self._last_no_range_log > 2.0:
                        self._last_no_range_log = now
                        self.get_logger().info(
                            'Thermal positive aimed at GUI demo obstacle — '
                            'pending FIRE suppressed'
                        )
                    return None
                self._publish_pending_fire(True)
            if now - self._last_no_range_log > 2.0:
                self._last_no_range_log = now
                age = None if b is None else round(now - b.stamp_s, 2)
                self.get_logger().info(
                    'Thermal positive without usable LiDAR range along bearing '
                    f'(bearing={None if b is None else round(b.bearing_rad, 3)} '
                    f'age={age}s) — not marking; pending FIRE signaled')
            return None
        map_xy = base_to_map(pt_base, (bx, by), yaw)
        if near_demo_obstacle(map_xy, self._demo_poses, self._demo_ignore_radius):
            self._publish_pending_fire(False)
            self.get_logger().info(
                f'THERMAL mark suppressed near demo obstacle '
                f'centre=({map_xy[0]:.2f},{map_xy[1]:.2f})'
            )
            return None
        self._publish_pending_fire(False)
        return map_xy

    def _tick(self) -> None:
        points: List[Tuple[float, float, float]] = []
        for fx, fy in self.known_fires:
            points.extend(_disc_points(fx, fy, self.hazard_radius, self.hazard_step))

        thermal_xy = None
        tf = self._lookup_base()
        if tf is not None:
            bx = tf.transform.translation.x
            by = tf.transform.translation.y
            yaw = self._yaw_from_quat(tf.transform.rotation)
            self._dedupe.note_detected(self.fire_detected)
            thermal_xy = self._thermal_hazard_xy(bx, by, yaw)
            if thermal_xy is not None and self._dedupe.accept(thermal_xy, self._now_s()):
                points.extend(
                    _disc_points(thermal_xy[0], thermal_xy[1], self.hazard_radius, self.hazard_step)
                )
                self.marks_emitted += 1
                self.get_logger().info(
                    f'THERMAL_FIRE_MARK centre=({thermal_xy[0]:.2f},{thermal_xy[1]:.2f}) '
                    f'robot=({bx:.2f},{by:.2f}) bearing='
                    f'{self.latest_bearing.bearing_rad if self.latest_bearing else float("nan"):.3f}')

        if points:
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.map_frame
            self.pub.publish(_pack_xyz_cloud(header, points))

        status = String()
        b = self.latest_bearing
        status.data = (
            f'fire_detected={self.fire_detected} '
            f'bearing={"none" if b is None else f"{b.bearing_rad:.3f}"} '
            f'thermal_xy={"none" if thermal_xy is None else f"({thermal_xy[0]:.2f},{thermal_xy[1]:.2f})"} '
            f'marks_emitted={self.marks_emitted}'
        )
        self.status_pub.publish(status)


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
