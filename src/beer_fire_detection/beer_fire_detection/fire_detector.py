"""B.E.E.R. thermal fire detector node

Subscribes to a thermal Image topic (default /husky1/thermal/image) and
publishes /fire_observation (rs1_interfaces/FireObservation) every frame,
plus /fire_detected (std_msgs/Bool, GUI mirror of fire_debounced) and
optional /fire_temperature (Float32).

This node uses a simple threshold and minimum-hot-pixels check inside a central
region of interest to avoid edge false positives. Parameters are exposed for
thresholds and topic names.

Entry 028: on every raw-positive frame it also publishes ``/fire_bearing``
(geometry_msgs/Vector3Stamped, header = image header): ``x`` = bearing of the
hot-pixel centroid (rad, +left, pinhole model from ``camera_hfov_rad``),
``y`` = angular half-width of the hot blob, ``z`` = max ROI temperature (K).
Kept for backward compatibility while consumers migrate to FireObservation.
The navigation bridge uses this bearing to pick the LiDAR return that gives
the fire its range; without it no FIRE keep-out is created.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from rs1_interfaces.msg import FireObservation
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32

import numpy as np


def hot_blob_bearing(hot_cols, image_width: int, hfov_rad: float):
    """(bearing, half_width) in radians, +left, of hot pixel columns.

    Mirrors ``rs1_nav.fire_bridge.hot_blob_bearing`` (kept local so this
    package has no dependency on the navigation package).
    """
    if hot_cols is None or len(hot_cols) == 0 or image_width <= 0 or hfov_rad <= 0.0:
        return None
    fx = (image_width / 2.0) / math.tan(hfov_rad / 2.0)
    cx = image_width / 2.0
    cols = np.asarray(hot_cols, dtype=np.float64)
    mean_col = float(cols.mean())
    bearing = -math.atan((mean_col - cx) / fx)
    left = -math.atan((float(cols.min()) - cx) / fx)
    right = -math.atan((float(cols.max()) - cx) / fx)
    half_width = max(abs(left - bearing), abs(right - bearing))
    return bearing, half_width


def update_fire_hysteresis(
    fire_raw: bool,
    fire_state: bool,
    pos_count: int,
    neg_count: int,
    positive_required: int,
    negative_required: int,
) -> tuple[bool, int, int]:
    """Advance consecutive-frame hysteresis; return (state, pos_count, neg_count)."""
    if fire_raw:
        pos_count += 1
        neg_count = 0
    else:
        neg_count += 1
        pos_count = 0

    if (not fire_state) and pos_count >= positive_required:
        fire_state = True
    elif fire_state and neg_count >= negative_required:
        fire_state = False
    return fire_state, pos_count, neg_count


class FireDetector(Node):
    def __init__(self):
        super().__init__('beer_fire_detector')

        # Parameters (configurable)
        self.declare_parameter('thermal_topic', '/husky1/thermal/image')
        self.declare_parameter('fire_threshold_kelvin', 400.0)
        self.declare_parameter('minimum_hot_pixels', 20)
        self.declare_parameter('thermal_resolution', 0.01)  # K per raw unit
        self.declare_parameter('roi_width_frac', 0.6)
        self.declare_parameter('roi_height_frac', 0.7)
        # Consecutive-frame hysteresis so FOV-edge chatter does not flap the Bool.
        self.declare_parameter('positive_required', 3)
        self.declare_parameter('negative_required', 5)
        # Horizontal FOV of the thermal camera (husky.gazebo.xacro: 1.047 rad).
        self.declare_parameter('camera_hfov_rad', 1.047)

        self.thermal_topic = self.get_parameter('thermal_topic').get_parameter_value().string_value
        self.threshold = self.get_parameter('fire_threshold_kelvin').get_parameter_value().double_value
        self.min_hot_pixels = int(self.get_parameter('minimum_hot_pixels').get_parameter_value().integer_value)
        self.resolution = float(self.get_parameter('thermal_resolution').get_parameter_value().double_value)
        self.roi_w = float(self.get_parameter('roi_width_frac').get_parameter_value().double_value)
        self.roi_h = float(self.get_parameter('roi_height_frac').get_parameter_value().double_value)
        self.positive_required = max(
            1, int(self.get_parameter('positive_required').get_parameter_value().integer_value)
        )
        self.negative_required = max(
            1, int(self.get_parameter('negative_required').get_parameter_value().integer_value)
        )

        self.camera_hfov = float(
            self.get_parameter('camera_hfov_rad').get_parameter_value().double_value)

        # Publishers
        self.obs_pub = self.create_publisher(FireObservation, '/fire_observation', 10)
        self.fire_pub = self.create_publisher(Bool, '/fire_detected', 10)
        self.temp_pub = self.create_publisher(Float32, '/fire_temperature', 10)
        self.bearing_pub = self.create_publisher(Vector3Stamped, '/fire_bearing', 10)

        # Debounced publish state (raw frame hits only update counters).
        self.fire_state = False
        self.previous_fire_state = False
        self._pos_count = 0
        self._neg_count = 0

        # Subscribe to thermal images
        self.get_logger().info('B.E.E.R. thermal detector started')
        self.get_logger().info(f'Threshold: {self.threshold:.1f} K')
        self.get_logger().info(
            f'Debounce pos/neg frames: {self.positive_required}/{self.negative_required}'
        )
        self.get_logger().info(f'Monitoring {self.thermal_topic}')

        self.sub = self.create_subscription(Image, self.thermal_topic, self.image_callback, 10)

        # On startup report no fire until data proves otherwise
        self.get_logger().info('No fire detected')

    def image_callback(self, msg: Image):
        try:
            height = msg.height
            width = msg.width
            encoding = msg.encoding.lower()
            # Determine bytes per pixel expectation from encoding
            if '16' in encoding:
                bytes_per_pixel = 2
            elif '8' in encoding or 'mono8' in encoding:
                bytes_per_pixel = 1
            else:
                # Unknown encoding — attempt to process generically
                bytes_per_pixel = None

            # Handle common encodings explicitly (mono16 / L16 and mono8)
            raw = None
            try:
                if '16' in encoding or 'mono16' in encoding or 'l16' in encoding:
                    # Data provided as byte sequence (list of uint8) or bytes; interpret as uint16
                    if isinstance(msg.data, (bytes, bytearray)):
                        arr_uint8 = np.frombuffer(msg.data, dtype=np.uint8)
                    else:
                        arr_uint8 = np.asarray(msg.data, dtype=np.uint8)

                    if arr_uint8.size < height * width * 2:
                        raise RuntimeError(f'Unexpected data length {arr_uint8.size} for {height}x{width} mono16')

                    # View as uint16 and respect endianness
                    arr16 = arr_uint8.view(np.uint16)
                    if msg.is_bigendian:
                        arr16 = arr16.byteswap()
                    raw = arr16.reshape((height, width))
                    temp_frame = raw.astype(np.float32) * self.resolution

                elif '8' in encoding or 'mono8' in encoding:
                    # 8-bit single-channel
                    if isinstance(msg.data, (bytes, bytearray)):
                        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width))
                    else:
                        raw = np.asarray(msg.data, dtype=np.uint8).reshape((height, width))
                    temp_frame = raw.astype(np.float32) * self.resolution

                else:
                    # Fallback generic handling for other encodings and data types
                    arr = np.asarray(msg.data)
                    if arr.size == (height * width):
                        raw = arr.reshape((height, width))
                    elif bytes_per_pixel == 2 and arr.size == (height * width * 2):
                        arr_uint8 = arr.astype(np.uint8)
                        arr16 = arr_uint8.view(np.uint16)
                        if msg.is_bigendian:
                            arr16 = arr16.byteswap()
                        raw = arr16.reshape((height, width))
                    else:
                        bpp = bytes_per_pixel if bytes_per_pixel is not None else 1
                        row_stride = msg.step // bpp if msg.step and bpp else width
                        raw = arr.reshape((height, row_stride))[:, :width]

                    if np.issubdtype(raw.dtype, np.floating):
                        if np.nanmax(raw) > 100.0:
                            temp_frame = raw.astype(np.float32)
                        else:
                            temp_frame = raw.astype(np.float32) * self.resolution
                    else:
                        temp_frame = raw.astype(np.float32) * self.resolution

            except Exception as e:
                self.get_logger().error(f'Error processing thermal image: {e}')
                return

            # Compute ROI
            x0 = int(((1.0 - self.roi_w) / 2.0) * width)
            x1 = width - x0
            y0 = int(((1.0 - self.roi_h) / 2.0) * height)
            y1 = height - y0
            roi = temp_frame[y0:y1, x0:x1]

            max_temp = float(np.max(roi))
            hot_mask = roi >= self.threshold
            hot_pixels = int(np.count_nonzero(hot_mask))

            fire = (hot_pixels >= self.min_hot_pixels) or (max_temp >= (self.threshold + 50.0))

            # Entry 028: bearing of the hot blob on raw-positive frames only.
            bearing_rad = 0.0
            blob_half_width_rad = 0.0
            bearing_valid = False
            if fire:
                if hot_pixels > 0:
                    hot_cols = np.nonzero(hot_mask)[1] + x0
                else:
                    # Single very hot pixel path: use the ROI max location.
                    hot_cols = np.asarray([int(np.argmax(roi) % roi.shape[1]) + x0])
                bearing = hot_blob_bearing(hot_cols, width, self.camera_hfov)
                if bearing is not None:
                    bearing_rad = float(bearing[0])
                    blob_half_width_rad = float(bearing[1])
                    bearing_valid = True
                    b_msg = Vector3Stamped()
                    b_msg.header = msg.header
                    b_msg.vector.x = bearing_rad
                    b_msg.vector.y = blob_half_width_rad
                    b_msg.vector.z = max_temp
                    self.bearing_pub.publish(b_msg)

            self.fire_state, self._pos_count, self._neg_count = update_fire_hysteresis(
                bool(fire),
                self.fire_state,
                self._pos_count,
                self._neg_count,
                self.positive_required,
                self.negative_required,
            )

            # Primary same-frame observation (header.stamp matches thermal Image).
            obs = FireObservation()
            obs.header = msg.header
            obs.raw_hot_blob = bool(fire)
            obs.fire_debounced = bool(self.fire_state)
            obs.bearing_valid = bearing_valid
            obs.bearing_rad = bearing_rad if bearing_valid else 0.0
            obs.blob_half_width_rad = blob_half_width_rad if bearing_valid else 0.0
            obs.max_roi_temperature_k = float(max_temp)
            self.obs_pub.publish(obs)

            # GUI mirror of fire_debounced; temperature every frame
            fire_msg = Bool()
            fire_msg.data = bool(self.fire_state)
            self.fire_pub.publish(fire_msg)

            temp_msg = Float32()
            temp_msg.data = float(max_temp)
            self.temp_pub.publish(temp_msg)

            # Log only on debounced state changes
            if (not self.previous_fire_state) and self.fire_state:
                self.get_logger().warning('=====================================')
                self.get_logger().warning('🔥 FIRE DETECTED AHEAD')
                self.get_logger().warning(f'Maximum temperature: {max_temp:.1f} K')
                self.get_logger().warning('=====================================')
            elif self.previous_fire_state and (not self.fire_state):
                self.get_logger().info('Fire cleared')
                self.get_logger().info(f'Maximum temperature: {max_temp:.1f} K')

            self.previous_fire_state = bool(self.fire_state)

        except Exception as e:
            self.get_logger().error(f'Error processing thermal image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = FireDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
