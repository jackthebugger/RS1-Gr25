"""B.E.E.R. thermal fire detector node

Subscribes to a thermal Image topic (default /husky1/thermal/image) and
publishes /fire_detected (std_msgs/Bool) and optional /fire_temperature (Float32).

This node uses a simple threshold and minimum-hot-pixels check inside a central
region of interest to avoid edge false positives. Parameters are exposed for
thresholds and topic names.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32

import numpy as np


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

        self.thermal_topic = self.get_parameter('thermal_topic').get_parameter_value().string_value
        self.threshold = self.get_parameter('fire_threshold_kelvin').get_parameter_value().double_value
        self.min_hot_pixels = int(self.get_parameter('minimum_hot_pixels').get_parameter_value().integer_value)
        self.resolution = float(self.get_parameter('thermal_resolution').get_parameter_value().double_value)
        self.roi_w = float(self.get_parameter('roi_width_frac').get_parameter_value().double_value)
        self.roi_h = float(self.get_parameter('roi_height_frac').get_parameter_value().double_value)

        # Publishers
        self.fire_pub = self.create_publisher(Bool, '/fire_detected', 10)
        self.temp_pub = self.create_publisher(Float32, '/fire_temperature', 10)

        # State
        self.previous_fire_state = False

        # Subscribe to thermal images
        self.get_logger().info('B.E.E.R. thermal detector started')
        self.get_logger().info(f'Threshold: {self.threshold:.1f} K')
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
            hot_pixels = int(np.count_nonzero(roi >= self.threshold))

            fire = (hot_pixels >= self.min_hot_pixels) or (max_temp >= (self.threshold + 50.0))

            # Publish fire flag and temperature every frame
            fire_msg = Bool()
            fire_msg.data = bool(fire)
            self.fire_pub.publish(fire_msg)

            temp_msg = Float32()
            temp_msg.data = float(max_temp)
            self.temp_pub.publish(temp_msg)

            # Log only on state changes
            if (not self.previous_fire_state) and fire:
                self.get_logger().warning('=====================================')
                self.get_logger().warning('🔥 FIRE DETECTED AHEAD')
                self.get_logger().warning(f'Maximum temperature: {max_temp:.1f} K')
                self.get_logger().warning('=====================================')
            elif self.previous_fire_state and (not fire):
                self.get_logger().info('Fire cleared')
                self.get_logger().info(f'Maximum temperature: {max_temp:.1f} K')

            self.previous_fire_state = bool(fire)

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
