#!/usr/bin/env python3
"""Standalone observation GUI for the B.E.E.R. thermal camera."""

from __future__ import annotations

import math
import time
from typing import Optional, Tuple

import numpy as np
import rclpy
import tkinter as tk
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32


THERMAL_MIN_K = 273.15
THERMAL_MAX_K = 673.15
STALE_AFTER_SEC = 1.0
DISPLAY_WIDTH = 800
DISPLAY_HEIGHT = 600


def _thermal_colour(value: float) -> Tuple[int, int, int]:
    """Map a normalized temperature value to a blue-cyan-yellow-red ramp."""
    value = max(0.0, min(1.0, value))
    stops = (
        (0.0, (8, 20, 120)),
        (0.25, (0, 170, 255)),
        (0.5, (0, 220, 80)),
        (0.75, (255, 220, 0)),
        (1.0, (220, 20, 20)),
    )
    for (start, start_rgb), (end, end_rgb) in zip(stops, stops[1:]):
        if value <= end:
            fraction = (value - start) / (end - start)
            return tuple(
                int(first + (second - first) * fraction)
                for first, second in zip(start_rgb, end_rgb)
            )
    return stops[-1][1]


def thermal_frame_to_ppm(
    data: bytes,
    width: int,
    height: int,
    step: int,
    encoding: str,
    is_bigendian: bool,
) -> Optional[bytes]:
    """Convert the supported thermal image encodings to a Tk-compatible PPM."""
    if width <= 0 or height <= 0:
        return None

    encoding = encoding.lower()
    if encoding in ('l16', 'mono16', '16uc1', '16sc1'):
        row_bytes = step or width * 2
        if len(data) < row_bytes * height:
            return None
        raw = np.frombuffer(data, dtype='>u2' if is_bigendian else '<u2')
        raw = raw.reshape((height, row_bytes // 2))[:, :width]
        temperatures = raw.astype(np.float32) * 0.01
    elif encoding in ('mono8', '8uc1', '8sc1'):
        row_bytes = step or width
        if len(data) < row_bytes * height:
            return None
        raw = np.frombuffer(data, dtype=np.uint8)
        raw = raw.reshape((height, row_bytes))[:, :width]
        temperatures = THERMAL_MIN_K + (
            raw.astype(np.float32) / 255.0 * (THERMAL_MAX_K - THERMAL_MIN_K)
        )
    else:
        return None

    normalized = np.clip(
        (temperatures - THERMAL_MIN_K) / (THERMAL_MAX_K - THERMAL_MIN_K),
        0.0,
        1.0,
    )
    rgb = np.empty((height, width, 3), dtype=np.uint8)
    for index in range(5):
        start = index / 4.0
        end = (index + 1) / 4.0
        mask = (normalized >= start) & (normalized <= end if index == 4 else normalized < end)
        if not np.any(mask):
            continue
        colours = np.array([_thermal_colour(start), _thermal_colour(end)], dtype=np.float32)
        fraction = ((normalized[mask] - start) / (end - start))[:, None]
        rgb[mask] = (colours[0] + (colours[1] - colours[0]) * fraction).astype(np.uint8)

    scale = min(DISPLAY_WIDTH / width, DISPLAY_HEIGHT / height)
    output_width = max(1, int(round(width * scale)))
    output_height = max(1, int(round(height * scale)))
    x_indices = np.minimum(
        (np.arange(output_width) * width // output_width), width - 1,
    )
    y_indices = np.minimum(
        (np.arange(output_height) * height // output_height), height - 1,
    )
    rgb = rgb[y_indices[:, None], x_indices[None, :]]
    return f'P6\n{output_width} {output_height}\n255\n'.encode() + rgb.tobytes()


class ThermalCameraWindow(tk.Tk):
    """Tk window that renders the latest state supplied by the ROS node."""

    def __init__(self) -> None:
        super().__init__()
        self.title('B.E.E.R. Thermal Camera')
        self.geometry('900x820')
        self.minsize(700, 650)
        self.configure(bg='#f3f4f6')

        tk.Label(
            self,
            text='B.E.E.R. THERMAL CAMERA',
            font=('Arial', 20, 'bold'),
            bg='#f3f4f6',
            fg='#1f2937',
            pady=14,
        ).pack(fill='x')

        self.image_frame = tk.Frame(self, bg='#111827', width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT)
        self.image_frame.pack(fill='both', expand=True, padx=20, pady=(0, 14))
        self.image_frame.pack_propagate(False)
        self.image_label = tk.Label(
            self.image_frame,
            text='LIVE THERMAL IMAGE\nWaiting for data...',
            font=('Arial', 16),
            bg='#111827',
            fg='#ffffff',
        )
        self.image_label.pack(fill='both', expand=True)
        self.image_photo = None

        status = tk.Frame(self, bg='#ffffff', bd=1, relief='solid')
        status.pack(fill='x', padx=20, pady=(0, 20))
        self.fire_label = tk.Label(
            status, text='FIRE STATUS: NO FIRE DETECTED',
            font=('Arial', 14, 'bold'), bg='#ffffff', fg='#047857',
            anchor='w', padx=16, pady=8,
        )
        self.fire_label.pack(fill='x')
        self.temperature_label = tk.Label(
            status, text='TEMPERATURE: -- K',
            font=('Arial', 13), bg='#ffffff', fg='#111827',
            anchor='w', padx=16, pady=5,
        )
        self.temperature_label.pack(fill='x')
        self.feed_label = tk.Label(
            status, text='THERMAL FEED: NO DATA',
            font=('Arial', 13, 'bold'), bg='#ffffff', fg='#b45309',
            anchor='w', padx=16, pady=8,
        )
        self.feed_label.pack(fill='x')

        self.latest_image: Optional[Tuple[bytes, int, int, int, str, bool]] = None
        self.latest_fire = False
        self.latest_temperature: Optional[float] = None
        self.last_image_time = 0.0
        self.rendered_image = None

    def accept_image(self, msg: Image) -> None:
        self.latest_image = (
            bytes(msg.data), int(msg.width), int(msg.height), int(msg.step),
            str(msg.encoding), bool(msg.is_bigendian),
        )
        self.last_image_time = time.monotonic()

    def accept_fire(self, msg: Bool) -> None:
        self.latest_fire = bool(msg.data)

    def accept_temperature(self, msg: Float32) -> None:
        self.latest_temperature = float(msg.data)

    def render(self) -> None:
        if self.latest_image is not None and self.latest_image is not self.rendered_image:
            ppm = thermal_frame_to_ppm(*self.latest_image)
            if ppm is not None:
                try:
                    self.image_photo = tk.PhotoImage(data=ppm, format='PPM')
                    self.image_label.configure(image=self.image_photo, text='')
                    self.rendered_image = self.latest_image
                except tk.TclError:
                    self.image_label.configure(image='', text='LIVE THERMAL IMAGE\nInvalid thermal frame')

        connected = time.monotonic() - self.last_image_time <= STALE_AFTER_SEC
        self.feed_label.configure(
            text='THERMAL FEED: CONNECTED' if connected else 'THERMAL FEED: NO DATA',
            fg='#047857' if connected else '#b45309',
        )
        self.fire_label.configure(
            text='FIRE STATUS: FIRE DETECTED' if self.latest_fire else 'FIRE STATUS: NO FIRE DETECTED',
            fg='#b91c1c' if self.latest_fire else '#047857',
            bg='#fee2e2' if self.latest_fire else '#ffffff',
        )
        if self.latest_temperature is None or not math.isfinite(self.latest_temperature):
            text = 'TEMPERATURE: -- K'
        else:
            text = f'TEMPERATURE: {self.latest_temperature:.1f} K'
        self.temperature_label.configure(text=text)


class ThermalCameraNode(Node):
    def __init__(self, window: ThermalCameraWindow, image_topic: str, fire_topic: str, temperature_topic: str):
        super().__init__('beer_thermal_camera_gui')
        self.window = window
        self.create_subscription(Image, image_topic, self._image_callback, 10)
        self.create_subscription(Bool, fire_topic, self._fire_callback, 10)
        self.create_subscription(Float32, temperature_topic, self._temperature_callback, 10)
        self.get_logger().info(
            f'Thermal GUI listening to {image_topic}, {fire_topic}, and {temperature_topic}'
        )

    def _image_callback(self, msg: Image) -> None:
        self.window.accept_image(msg)

    def _fire_callback(self, msg: Bool) -> None:
        self.window.accept_fire(msg)

    def _temperature_callback(self, msg: Float32) -> None:
        self.window.accept_temperature(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    window = ThermalCameraWindow()
    parameter_node = Node('beer_thermal_camera_gui_parameters')
    image_topic = str(parameter_node.declare_parameter(
        'image_topic', '/husky1/thermal/image',
    ).value)
    fire_topic = str(parameter_node.declare_parameter(
        'fire_topic', '/husky1/fire_detected',
    ).value)
    temperature_topic = str(parameter_node.declare_parameter(
        'temperature_topic', '/husky1/fire_temperature',
    ).value)
    parameter_node.destroy_node()
    node = ThermalCameraNode(
        window,
        image_topic,
        fire_topic,
        temperature_topic,
    )
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            window.render()
            window.update_idletasks()
            window.update()
    except KeyboardInterrupt:
        pass
    finally:
        window.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
