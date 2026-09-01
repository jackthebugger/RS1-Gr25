"""Synthetic thermal image publisher for B.E.E.R. stakeholder demo.

Publishes:
 - /husky1/thermal/demo_image  (sensor_msgs/msg/Image, mono16)
 - /husky1/thermal/demo_visual (sensor_msgs/msg/Image, mono8) for RViz

Subscribes:
 - /husky1/scan (sensor_msgs/msg/LaserScan)

Logic:
 - Detect if an object exists within +/- roi_angle_deg around front and between
   min_range and max_range. If so, publish a frame with central hotspot (~600 K),
   otherwise publish ambient (~288-300 K).
 - Uses resolution param (K per raw unit) consistent with fire_detector (0.01).
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
import numpy as np
import math


class SyntheticThermalDemo(Node):
    def __init__(self):
        super().__init__('synthetic_thermal_demo')

        # Parameters
        self.declare_parameter('demo_topic', '/husky1/thermal/demo_image')
        self.declare_parameter('visual_topic', '/husky1/thermal/demo_visual')
        self.declare_parameter('scan_topic', '/husky1/scan')
        self.declare_parameter('resolution', 0.01)  # K per raw unit
        self.declare_parameter('frame_id', 'husky1/husky1_base_link/beer_thermal_camera')
        self.declare_parameter('width', 320)
        self.declare_parameter('height', 240)
        self.declare_parameter('roi_angle_deg', 20.0)
        self.declare_parameter('min_range', 0.5)
        self.declare_parameter('max_range', 4.0)

        self.demo_topic = self.get_parameter('demo_topic').get_parameter_value().string_value
        self.visual_topic = self.get_parameter('visual_topic').get_parameter_value().string_value
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.resolution = float(self.get_parameter('resolution').get_parameter_value().double_value)
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.width = int(self.get_parameter('width').get_parameter_value().integer_value)
        self.height = int(self.get_parameter('height').get_parameter_value().integer_value)
        self.roi_angle_deg = float(self.get_parameter('roi_angle_deg').get_parameter_value().double_value)
        self.min_range = float(self.get_parameter('min_range').get_parameter_value().double_value)
        self.max_range = float(self.get_parameter('max_range').get_parameter_value().double_value)

        # Debounce / hysteresis configuration
        self.declare_parameter('positive_required', 3)
        self.declare_parameter('negative_required', 5)
        self.declare_parameter('min_valid_rays', 3)
        self.positive_required = int(self.get_parameter('positive_required').get_parameter_value().integer_value)
        self.negative_required = int(self.get_parameter('negative_required').get_parameter_value().integer_value)
        self.min_valid_rays = int(self.get_parameter('min_valid_rays').get_parameter_value().integer_value)

        # Publishers
        self.demo_pub = self.create_publisher(Image, self.demo_topic, 10)
        self.visual_pub = self.create_publisher(Image, self.visual_topic, 10)

        # Subscribe to LiDAR
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, 10)

        # Current detection state and counters
        self.object_ahead = False
        self.fire_state = False
        self._pos_count = 0
        self._neg_count = 0

        # Publish at a modest rate when no scans arriving
        self.timer = self.create_timer(0.5, self._publish_from_state)

        self.get_logger().info('Synthetic thermal demo started')
        self.get_logger().info(f'Publishing demo thermal: {self.demo_topic} (mono16)')
        self.get_logger().info(f'Publishing visual: {self.visual_topic} (mono8)')
        self.get_logger().info(f'ROI angle: ±{self.roi_angle_deg} deg, min_range: {self.min_range}, max_range: {self.max_range}')
        self.get_logger().info(f'Debounce pos/neg: {self.positive_required}/{self.negative_required}, min_valid_rays: {self.min_valid_rays}')

    def scan_cb(self, msg: LaserScan):
        # Determine if any range in central +/- roi_angle is within thresholds
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = np.array(msg.ranges, dtype=np.float32)

        # Compute index range for roi
        roi_rad = math.radians(self.roi_angle_deg)
        # Angles array
        n = ranges.size
        angles = angle_min + np.arange(n) * angle_increment
        # Select indices within -roi..+roi
        mask = (np.abs(angles) <= roi_rad) & np.isfinite(ranges)
        selected = ranges[mask]

        # Count valid rays within forward cone that satisfy distance conditions
        valid_mask = (np.abs(angles) <= roi_rad) & np.isfinite(ranges)
        # rays that are finite and within min/max range
        in_range_mask = valid_mask & (ranges >= self.min_range) & (ranges <= self.max_range)
        valid_rays = int(np.count_nonzero(in_range_mask))

        # Per-scan immediate result (before debounce)
        found = (valid_rays >= self.min_valid_rays)

        # Update immediate flag for diagnostics
        if found != self.object_ahead:
            self.get_logger().info('Synthetic thermal: object ahead = %s (valid_rays=%d)' % (found, valid_rays))
        self.object_ahead = bool(found)

        # Debounce logic: require N consecutive positive scans to set fire_state True
        if found:
            self._pos_count += 1
            self._neg_count = 0
        else:
            self._neg_count += 1
            self._pos_count = 0

        # Emit per-scan debug info so we can inspect why the state persists
        self.get_logger().info(f'Synthetic scan: valid_rays={valid_rays}, pos_count={self._pos_count}, neg_count={self._neg_count}, fire_state={self.fire_state}')

        prev_state = self.fire_state
        # Transition to True only after positive_required consecutive positives
        if (not self.fire_state) and (self._pos_count >= self.positive_required):
            self.fire_state = True
            self.get_logger().warn('Synthetic thermal: FIRE state debounced -> TRUE')
        # Transition to False only after negative_required consecutive negatives
        if self.fire_state and (self._neg_count >= self.negative_required):
            self.fire_state = False
            self.get_logger().warn('Synthetic thermal: FIRE state debounced -> FALSE')

        # Immediately publish a synthetic frame reflecting debounced fire_state
        self._publish_from_state()

    def _publish_from_state(self):
        # Build mono16 image (uint16) with resolution mapping
        # When debounced fire_state is True, publish a clear 600 K hotspot
        # Otherwise publish ambient (~288 K) with no hotspot
        if self.fire_state:
            hot_K = 600.0
            ambient_K = 288.0
        else:
            hot_K = 288.0
            ambient_K = 288.0

        # Raw units (uint16) using resolution
        ambient_raw = int(round(ambient_K / self.resolution))
        hot_raw = int(round(hot_K / self.resolution))

        # Create image array filled with ambient
        img = np.full((self.height, self.width), ambient_raw, dtype=np.uint16)

        if self.fire_state:
            # Draw central hot ellipse/rectangle (obvious flame-like hotspot)
            cx = self.width // 2
            cy = self.height // 2
            rx = int(self.width * 0.15)
            ry = int(self.height * 0.15)
            y, x = np.ogrid[-cy:self.height-cy, -cx:self.width-cx]
            mask = (x*x)/(rx*rx) + (y*y)/(ry*ry) <= 1.0
            img[mask] = hot_raw
            # Add gradient around hotspot
            dist = np.sqrt((x*x)/(rx*rx) + (y*y)/(ry*ry))
            outer = (dist > 1.0) & (dist <= 2.0)
            img[outer] = np.uint16((hot_raw + ambient_raw)//2)

        # Create sensor_msgs/Image for mono16
        out = Image()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.height = self.height
        out.width = self.width
        out.encoding = 'mono16'
        out.is_bigendian = 0
        out.step = self.width * 2
        out.data = img.tobytes()

        self.demo_pub.publish(out)

        # Also publish visual mono8 for RViz (scale between ambient~280 and hot~650)
        vis = np.clip(((img.astype(np.float32) * self.resolution) - 280.0) / (650.0 - 280.0) * 255.0, 0, 255).astype(np.uint8)
        vmsg = Image()
        vmsg.header = out.header
        vmsg.height = self.height
        vmsg.width = self.width
        vmsg.encoding = 'mono8'
        vmsg.is_bigendian = 0
        vmsg.step = self.width
        vmsg.data = vis.tobytes()
        self.visual_pub.publish(vmsg)


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticThermalDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
