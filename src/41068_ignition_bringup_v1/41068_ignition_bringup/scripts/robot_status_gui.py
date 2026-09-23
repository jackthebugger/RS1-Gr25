#!/usr/bin/env python3
"""Small modular GUI for displaying robot status during autonomy demos.

Supports:
  * Unified START/STOP MISSION using X/Y/Yaw fields → NavigateToPose
  * Dynamic Path A / Path B / Random block (5 m spatial trigger) + Clear
"""

from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass
from math import cos, sin
from typing import Iterable, List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan

import tkinter as tk

# Allow running from source tree without install.
_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from rs1_nav.forest_obstacle_manager import (  # noqa: E402
    PATH_A,
    PATH_B,
    ForestObstacleManager,
    ObstacleState,
    load_forest_gap_config,
)
from rs1_nav.geometry import yaw_from_quaternion  # noqa: E402


# Defaults for custom_world_1 primary scenario (also used as Start Mission goal).
DEFAULT_GOAL_X = 18.0
DEFAULT_GOAL_Y = 0.0
DEFAULT_GOAL_YAW = 0.0

# Soft bounds for coordinate validation (60×30 m ground, X±30 Y±15).
MAP_X_MIN, MAP_X_MAX = -29.0, 29.0
MAP_Y_MIN, MAP_Y_MAX = -14.0, 14.0


@dataclass
class RobotStatus:
    """Simple in-memory snapshot of values shown in the GUI."""

    robot_name: str = 'Rescue Bot'
    speed: float = 0.0
    destination: str = 'Safehouse'
    distance_to_destination: float = 0.0
    time_to_destination: str = 'Pending'
    battery: float = 100.0
    obstacle_detected: bool = False

    def to_display_lines(self) -> List[str]:
        return [
            f'Robot Name: {self.robot_name}',
            f'Destination: {self.destination}',
            f'Speed: {format_speed(self.speed)}',
            f'Distance to Destination: {self.distance_to_destination:.1f} m',
            f'Time to Destination: {self.time_to_destination}',
            f'Battery: {self.battery:.0f}%',
            f'Obstacle detected: {"yes" if self.obstacle_detected else "no"}',
        ]


def format_speed(speed: float) -> str:
    return f'{float(speed):.2f} m/s'


def format_remaining_time(seconds: float) -> str:
    total_seconds = max(0, int(round(float(seconds))))
    minutes, remaining_seconds = divmod(total_seconds, 60)
    if minutes:
        return f'{minutes}m {remaining_seconds:02d}s'
    return f'{remaining_seconds}s'


def detect_obstacle(ranges: Iterable[float], threshold: float = 1.0) -> bool:
    """Return True when a nearby range measurement suggests an obstacle."""
    if ranges is None:
        return False

    valid_ranges = [float(value) for value in ranges if value is not None]
    if not valid_ranges:
        return False

    nearest = min(valid_ranges)
    return nearest < threshold


def parse_coordinate(raw: str, name: str) -> Tuple[Optional[float], Optional[str]]:
    """Parse a numeric field. Returns (value, error_message)."""
    text = (raw or '').strip()
    if not text:
        return None, f'Invalid {name}: empty'
    try:
        value = float(text)
    except ValueError:
        return None, f'Invalid {name}: not a number'
    if not math.isfinite(value):
        return None, f'Invalid {name}: not finite'
    return value, None


def validate_goal(x: float, y: float, yaw: float) -> Optional[str]:
    """Return an error string if the goal is unreasonable for this world."""
    if not (MAP_X_MIN <= x <= MAP_X_MAX):
        return f'Invalid X coordinate (expected {MAP_X_MIN}…{MAP_X_MAX})'
    if not (MAP_Y_MIN <= y <= MAP_Y_MAX):
        return f'Invalid Y coordinate (expected {MAP_Y_MIN}…{MAP_Y_MAX})'
    if abs(yaw) > 2.0 * math.pi + 0.01:
        return 'Invalid yaw (expected radians within ±2π)'
    return None


class RobotStatusWindow(tk.Tk):
    """A clean GUI window that displays a robot status summary."""

    def __init__(self, robot_name: str):
        super().__init__()
        self.robot_name = robot_name
        self.title(f'{robot_name} status')
        # Compact default so windowed desktops show controls; content scrolls.
        self.geometry('560x640')
        self.minsize(420, 420)

        self.configure(bg='#f3f4f6')

        self.header = tk.Label(
            self,
            text='Robot Status',
            font=('Arial', 16, 'bold'),
            bg='#f3f4f6',
            fg='#1f2937',
            anchor='w',
            padx=20,
            pady=8,
        )
        self.header.pack(fill='x')

        # Scrollable body: status + mission + goals + dynamic obstacle buttons.
        scroll_host = tk.Frame(self, bg='#f3f4f6')
        scroll_host.pack(fill='both', expand=True)

        self._scrollbar = tk.Scrollbar(scroll_host, orient='vertical')
        self._scrollbar.pack(side='right', fill='y')
        self._canvas = tk.Canvas(
            scroll_host,
            bg='#f3f4f6',
            highlightthickness=0,
            yscrollcommand=self._scrollbar.set,
        )
        self._canvas.pack(side='left', fill='both', expand=True)
        self._scrollbar.configure(command=self._canvas.yview)

        self._content = tk.Frame(self._canvas, bg='#f3f4f6')
        self._content_window = self._canvas.create_window(
            (0, 0), window=self._content, anchor='nw',
        )
        self._content.bind('<Configure>', self._on_content_configure)
        self._canvas.bind('<Configure>', self._on_canvas_configure)
        self._bind_mousewheel(self._canvas)
        self._bind_mousewheel(self._content)

        body = self._content

        self.camera_frame = tk.Frame(
            body,
            width=320,
            height=160,
            bg='#111827',
        )
        self.camera_frame.pack(padx=20, pady=(0, 10))
        self.camera_frame.pack_propagate(False)
        self.camera_label = tk.Label(
            self.camera_frame,
            text='Camera: waiting for image',
            bg='#111827',
            fg='#ffffff',
            anchor='center',
        )
        self.camera_label.pack(fill='both', expand=True)
        self.camera_photo = None

        self.info_frame = tk.Frame(body, bg='#ffffff', bd=1, relief='solid')
        self.info_frame.pack(fill='x', padx=20, pady=(0, 10))

        self.robot_name_var = tk.StringVar(value='Robot Name: Rescue Bot')
        self.destination_var = tk.StringVar(value='Destination: Safehouse')
        self.speed_var = tk.StringVar(value='Speed: 0.00 m/s')
        self.distance_var = tk.StringVar(value='Distance to Destination: 0.0 m')
        self.time_var = tk.StringVar(value='Time to Destination: Pending')
        self.battery_var = tk.StringVar(value='Battery: 100%')
        self.obstacle_var = tk.StringVar(value='Obstacle detected: no')
        self.mission_var = tk.StringVar(value='Mission: ready')
        self.goal_status_var = tk.StringVar(value='Goal: none')
        self.dyn_obstacle_var = tk.StringVar(value='Obstacle: READY')
        self.feedback_var = tk.StringVar(value='')

        self.labels = [
            tk.Label(self.info_frame, textvariable=self.robot_name_var, font=('Arial', 12), bg='#ffffff', fg='#111827', anchor='w', padx=16, pady=3),
            tk.Label(self.info_frame, textvariable=self.destination_var, font=('Arial', 12), bg='#ffffff', fg='#111827', anchor='w', padx=16, pady=3),
            tk.Label(self.info_frame, textvariable=self.speed_var, font=('Arial', 12), bg='#ffffff', fg='#111827', anchor='w', padx=16, pady=3),
            tk.Label(self.info_frame, textvariable=self.distance_var, font=('Arial', 12), bg='#ffffff', fg='#111827', anchor='w', padx=16, pady=3),
            tk.Label(self.info_frame, textvariable=self.time_var, font=('Arial', 12), bg='#ffffff', fg='#111827', anchor='w', padx=16, pady=3),
            tk.Label(self.info_frame, textvariable=self.battery_var, font=('Arial', 12), bg='#ffffff', fg='#111827', anchor='w', padx=16, pady=3),
            tk.Label(self.info_frame, textvariable=self.obstacle_var, font=('Arial', 12), bg='#ffffff', fg='#111827', anchor='w', padx=16, pady=3),
            tk.Label(self.info_frame, textvariable=self.goal_status_var, font=('Arial', 12), bg='#ffffff', fg='#111827', anchor='w', padx=16, pady=3),
            tk.Label(self.info_frame, textvariable=self.mission_var, font=('Arial', 12, 'bold'), bg='#ffffff', fg='#111827', anchor='w', padx=16, pady=3),
            tk.Label(self.info_frame, textvariable=self.dyn_obstacle_var, font=('Arial', 12, 'bold'), bg='#ffffff', fg='#111827', anchor='w', padx=16, pady=3),
        ]
        self.feedback_label = tk.Label(
            self.info_frame, textvariable=self.feedback_var, font=('Arial', 11),
            bg='#ffffff', fg='#b45309', anchor='w', padx=16, pady=3,
        )
        self.labels.append(self.feedback_label)

        for label in self.labels:
            label.pack(fill='x')

        # --- Navigation Goal (X/Y/Yaw + unified Start/Stop) ---
        self.goal_frame = tk.LabelFrame(
            body,
            text='Navigation Goal',
            bg='#f3f4f6',
            fg='#1f2937',
            font=('Arial', 11, 'bold'),
            padx=12,
            pady=8,
        )
        self.goal_frame.pack(fill='x', padx=20, pady=(0, 8))

        self.goal_x_var = tk.StringVar(value=f'{DEFAULT_GOAL_X:g}')
        self.goal_y_var = tk.StringVar(value=f'{DEFAULT_GOAL_Y:g}')
        self.goal_yaw_var = tk.StringVar(value=f'{DEFAULT_GOAL_YAW:g}')

        for row, (label, var) in enumerate((
            ('X', self.goal_x_var),
            ('Y', self.goal_y_var),
            ('Yaw', self.goal_yaw_var),
        )):
            tk.Label(
                self.goal_frame, text=f'{label}:', bg='#f3f4f6', fg='#111827',
                font=('Arial', 11), width=5, anchor='w',
            ).grid(row=row, column=0, sticky='w', pady=2)
            tk.Entry(
                self.goal_frame, textvariable=var, font=('Arial', 11), width=16,
            ).grid(row=row, column=1, sticky='ew', pady=2)

        self.goal_frame.columnconfigure(1, weight=1)

        self.mission_button = tk.Button(
            self.goal_frame,
            text='START MISSION',
            state='disabled',
            command=lambda: None,
            height=2,
        )
        self.mission_button.grid(
            row=3, column=0, columnspan=2, sticky='ew', pady=(10, 0), ipady=2,
        )
        self._mission_running = False

        # --- Dynamic Obstacles: Path A / Path B / Random / Clear ---
        self.obstacle_frame = tk.LabelFrame(
            body,
            text='Dynamic Obstacles',
            bg='#f3f4f6',
            fg='#1f2937',
            font=('Arial', 11, 'bold'),
            padx=12,
            pady=10,
        )
        self.obstacle_frame.pack(fill='x', padx=20, pady=(0, 20))
        self.obstacle_frame.columnconfigure(0, weight=1)
        self.obstacle_frame.columnconfigure(1, weight=1)

        self.block_path_a_button = tk.Button(
            self.obstacle_frame,
            text='BLOCK PATH A',
            state='disabled',
            command=lambda: None,
            height=2,
        )
        self.block_path_a_button.grid(
            row=0, column=0, sticky='ew', padx=(0, 6), pady=4, ipady=4,
        )
        self.block_path_b_button = tk.Button(
            self.obstacle_frame,
            text='BLOCK PATH B',
            state='disabled',
            command=lambda: None,
            height=2,
        )
        self.block_path_b_button.grid(
            row=0, column=1, sticky='ew', padx=(6, 0), pady=4, ipady=4,
        )
        self.random_block_button = tk.Button(
            self.obstacle_frame,
            text='RANDOM BLOCK',
            state='disabled',
            command=lambda: None,
            height=2,
        )
        self.random_block_button.grid(
            row=1, column=0, sticky='ew', padx=(0, 6), pady=4, ipady=4,
        )
        self.clear_obstacle_button = tk.Button(
            self.obstacle_frame,
            text='CLEAR OBSTACLES',
            state='disabled',
            command=lambda: None,
            height=2,
        )
        self.clear_obstacle_button.grid(
            row=1, column=1, sticky='ew', padx=(6, 0), pady=4, ipady=4,
        )

        self.update_status()
        self._bind_mousewheel_tree(body)
        self.after_idle(self._refresh_scroll_region)

    def _on_content_configure(self, _event=None) -> None:
        self._refresh_scroll_region()

    def _on_canvas_configure(self, event) -> None:
        self._canvas.itemconfigure(self._content_window, width=event.width)
        self._refresh_scroll_region()

    def _refresh_scroll_region(self) -> None:
        self._canvas.update_idletasks()
        bbox = self._canvas.bbox('all')
        if bbox is not None:
            self._canvas.configure(scrollregion=bbox)

    def _bind_mousewheel(self, widget) -> None:
        widget.bind('<MouseWheel>', self._on_mousewheel, add='+')
        widget.bind('<Button-4>', self._on_mousewheel, add='+')
        widget.bind('<Button-5>', self._on_mousewheel, add='+')

    def _bind_mousewheel_tree(self, widget) -> None:
        self._bind_mousewheel(widget)
        for child in widget.winfo_children():
            self._bind_mousewheel_tree(child)

    def _on_mousewheel(self, event) -> str:
        if getattr(event, 'num', None) == 4 or getattr(event, 'delta', 0) > 0:
            self._canvas.yview_scroll(-1, 'units')
        elif getattr(event, 'num', None) == 5 or getattr(event, 'delta', 0) < 0:
            self._canvas.yview_scroll(1, 'units')
        return 'break'

    def set_mission_callback(self, toggle_callback) -> None:
        self.mission_button.configure(command=toggle_callback, state='normal')

    def set_obstacle_callbacks(
        self,
        path_a_callback,
        path_b_callback,
        random_callback,
        clear_callback,
    ) -> None:
        self.block_path_a_button.configure(command=path_a_callback, state='normal')
        self.block_path_b_button.configure(command=path_b_callback, state='normal')
        self.random_block_button.configure(command=random_callback, state='normal')
        self.clear_obstacle_button.configure(command=clear_callback, state='normal')

    def set_mission_running(self, running: bool) -> None:
        self._mission_running = bool(running)
        if running:
            self.mission_button.configure(text='STOP MISSION', state='normal')
        else:
            self.mission_button.configure(text='START MISSION', state='normal')

    def set_mission_status(self, status: str) -> None:
        self.mission_var.set(f'Mission: {status}')

    def set_goal_status(self, status: str) -> None:
        self.goal_status_var.set(f'Goal: {status}')

    def set_dyn_obstacle_status(self, status: str) -> None:
        if status.startswith('Obstacle:'):
            self.dyn_obstacle_var.set(status)
        else:
            self.dyn_obstacle_var.set(f'Obstacle: {status}')

    def set_feedback(self, message: str, *, error: bool = False) -> None:
        self.feedback_var.set(message)
        self.feedback_label.configure(fg='#b91c1c' if error else '#047857')

    def read_goal_fields(self) -> Tuple[str, str, str]:
        return (
            self.goal_x_var.get(),
            self.goal_y_var.get(),
            self.goal_yaw_var.get(),
        )

    def set_goal_fields(self, x: float, y: float, yaw: float) -> None:
        self.goal_x_var.set(f'{x:g}')
        self.goal_y_var.set(f'{y:g}')
        self.goal_yaw_var.set(f'{yaw:g}')

    def update_camera(self, image: Image) -> None:
        if image.encoding not in ('rgb8', 'bgr8', 'rgba8', 'bgra8', 'mono8'):
            self.camera_label.configure(text=f'Camera: unsupported {image.encoding}', image='')
            return

        source_channels = 1 if image.encoding == 'mono8' else 4 if image.encoding in ('rgba8', 'bgra8') else 3
        row_width = image.width * source_channels
        pixels = bytearray()
        for row in range(image.height):
            start = row * image.step
            current = image.data[start:start + row_width]
            if image.encoding in ('bgr8', 'bgra8'):
                current = bytes(current[index] for pixel in range(image.width) for index in (
                    pixel * (4 if image.encoding == 'bgra8' else 3) + 2,
                    pixel * (4 if image.encoding == 'bgra8' else 3) + 1,
                    pixel * (4 if image.encoding == 'bgra8' else 3),
                ))
            elif image.encoding in ('rgba8', 'bgra8'):
                current = bytes(current[index] for pixel in range(image.width) for index in (
                    pixel * 4,
                    pixel * 4 + 1,
                    pixel * 4 + 2,
                ))
            elif image.encoding == 'mono8':
                current = bytes(value for value in current for _ in range(3))
            pixels.extend(current)

        ppm = f'P6\n{image.width} {image.height}\n255\n'.encode() + pixels
        try:
            self.camera_photo = tk.PhotoImage(
                data=ppm,
                format='PPM',
            )
        except tk.TclError:
            self.camera_label.configure(text='Camera: invalid image frame', image='')
            return
        self.camera_label.configure(image=self.camera_photo, text='')

    def update_status(
        self,
        speed: float = 0.0,
        destination: str = 'Safehouse',
        distance_to_destination: float = 0.0,
        time_to_destination: str = 'Pending',
        battery: float = 100.0,
        obstacle_detected: bool = False,
        robot_name: str = 'Rescue Bot',
    ) -> None:
        self.robot_name_var.set(f'Robot Name: {robot_name}')
        self.destination_var.set(f'Destination: {destination}')
        self.speed_var.set(f'Speed: {format_speed(speed)}')
        self.distance_var.set(f'Distance to Destination: {distance_to_destination:.1f} m')
        self.time_var.set(f'Time to Destination: {time_to_destination}')
        self.battery_var.set(f'Battery: {battery:.0f}%')
        self.obstacle_var.set(f'Obstacle detected: {"yes" if obstacle_detected else "no"}')

    def set_status(self, status: RobotStatus) -> None:
        self.update_status(
            speed=status.speed,
            destination=status.destination,
            distance_to_destination=status.distance_to_destination,
            time_to_destination=status.time_to_destination,
            battery=status.battery,
            obstacle_detected=status.obstacle_detected,
            robot_name=status.robot_name,
        )


class RobotStatusNode(Node):
    """ROS node that gathers robot data and updates the Tkinter panel."""

    def __init__(self, gui: RobotStatusWindow, robot_name: str):
        super().__init__('robot_status_gui')
        self.gui = gui
        self.robot_name = robot_name
        self.status = RobotStatus(robot_name=robot_name)
        self.latest_speed = 0.0
        self.latest_obstacle_detected = False
        self.latest_distance = 0.0
        self.latest_time = 'Pending'
        self.latest_pose: Optional[Tuple[float, float, float]] = None
        self.goal_handle = None
        self.start_request_pending = False
        self.start_wait_timer = None
        self._pending_goal: Optional[Tuple[float, float, float]] = None
        self.active_goal: Optional[Tuple[float, float, float]] = None

        goal_frame = self.declare_parameter('goal_frame', 'husky1_map').value
        goal_x = self.declare_parameter('goal_x', DEFAULT_GOAL_X).value
        goal_y = self.declare_parameter('goal_y', DEFAULT_GOAL_Y).value
        goal_yaw = self.declare_parameter('goal_yaw', DEFAULT_GOAL_YAW).value
        navigate_action = self.declare_parameter(
            'navigate_action', 'navigate_to_pose'
        ).value
        world_name = self.declare_parameter('world_name', 'custom_world_1').value
        gaps_config = self.declare_parameter('forest_gaps_config', '').value
        obstacle_seed = self.declare_parameter('obstacle_seed', -1).value
        force_gap = self.declare_parameter('force_gap', '').value

        self.goal_frame = str(goal_frame)
        self.default_goal_x = float(goal_x)
        self.default_goal_y = float(goal_y)
        self.default_goal_yaw = float(goal_yaw)
        self.navigate_client = ActionClient(self, NavigateToPose, navigate_action)
        self.status.distance_to_destination = 0.0
        self.status.time_to_destination = 'Pending'
        self.status.battery = 100.0

        self.gui.set_goal_fields(
            self.default_goal_x, self.default_goal_y, self.default_goal_yaw
        )
        self.gui.set_goal_status(
            f'default ({self.default_goal_x:g}, {self.default_goal_y:g}, '
            f'{self.default_goal_yaw:g})'
        )

        odom_topic = self.declare_parameter('odom_topic', 'odom').value
        scan_topic = self.declare_parameter('scan_topic', 'scan').value
        camera_topic = self.declare_parameter('camera_topic', 'camera/image').value
        obstacle_threshold = self.declare_parameter('obstacle_threshold', 1.0).value
        self.obstacle_threshold = float(obstacle_threshold)

        config_path = str(gaps_config).strip() or None
        gap_config = load_forest_gap_config(config_path)
        if world_name:
            gap_config.world_name = str(world_name)
        seed = int(obstacle_seed)
        self.obstacle_manager = ForestObstacleManager(
            gap_config,
            seed=None if seed < 0 else seed,
            force_gap=str(force_gap).strip() or None,
            logger=lambda m: self.get_logger().info(m),
        )
        self.gui.set_dyn_obstacle_status(self.obstacle_manager.status_message)

        self.create_subscription(Odometry, odom_topic, self._odom_callback, 10)
        self.create_subscription(LaserScan, scan_topic, self._scan_callback, qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'base_scan', self._scan_callback, qos_profile_sensor_data)
        self.create_subscription(Image, camera_topic, self._camera_callback, qos_profile_sensor_data)
        self.create_timer(0.1, self._update_gui)
        self.create_timer(0.2, self._obstacle_tick)

        self.get_logger().info(
            f'Robot status GUI started for {self.robot_name}. '
            f'Default goal ({self.default_goal_x}, {self.default_goal_y}, '
            f'{self.default_goal_yaw}). Listening to {odom_topic} and '
            f'{scan_topic}/base_scan. World={gap_config.world_name}.'
        )

    # -- shared goal submission -------------------------------------------

    def submit_goal(self, x: float, y: float, yaw: float, *, source: str) -> None:
        """Single NavigateToPose pathway used by START MISSION."""
        if self.goal_handle is not None:
            self.gui.set_feedback('Goal rejected: mission already running', error=True)
            self.gui.set_mission_status('running')
            self.gui.set_mission_running(True)
            return

        if not self.navigate_client.wait_for_server(timeout_sec=0.1):
            self._pending_goal = (x, y, yaw)
            self.start_request_pending = True
            self.gui.set_mission_status('waiting for Nav2')
            self.gui.set_feedback('Navigation unavailable — waiting for Nav2', error=True)
            if self.start_wait_timer is None:
                self.start_wait_timer = self.create_timer(1.0, self._retry_pending_goal)
            return

        self.start_request_pending = False
        self._pending_goal = None
        if self.start_wait_timer is not None:
            self.start_wait_timer.cancel()
            self.start_wait_timer = None

        goal_message = NavigateToPose.Goal()
        goal_message.pose = PoseStamped()
        goal_message.pose.header.frame_id = self.goal_frame
        goal_message.pose.header.stamp = self.get_clock().now().to_msg()
        goal_message.pose.pose.position.x = float(x)
        goal_message.pose.pose.position.y = float(y)
        goal_message.pose.pose.position.z = 0.0
        goal_message.pose.pose.orientation.z = sin(yaw / 2.0)
        goal_message.pose.pose.orientation.w = cos(yaw / 2.0)

        self.active_goal = (float(x), float(y), float(yaw))
        self.latest_distance = 0.0
        self.latest_time = 'Pending'
        self.gui.set_goal_fields(x, y, yaw)
        self.gui.set_goal_status(f'({x:g}, {y:g}, {yaw:g})')
        self.gui.set_mission_status('starting')
        self.gui.set_mission_running(True)
        self.gui.set_feedback(
            f'Mission started — navigating to X={x:g} Y={y:g} Yaw={yaw:g}'
        )
        self.get_logger().info(
            f'Submitting goal ({x:.2f}, {y:.2f}, {yaw:.2f}) from {source}'
        )

        future = self.navigate_client.send_goal_async(
            goal_message,
            feedback_callback=self._feedback_callback,
        )
        future.add_done_callback(self._goal_response_callback)

    def toggle_mission(self) -> None:
        """Unified START MISSION / STOP MISSION control."""
        if self.gui._mission_running or self.goal_handle is not None or self.start_request_pending:
            self.stop_mission()
            return
        self.start_mission_from_fields()

    def start_mission_from_fields(self) -> None:
        """Validate X/Y/Yaw entry fields and submit via the shared goal handler."""
        raw_x, raw_y, raw_yaw = self.gui.read_goal_fields()
        x, err = parse_coordinate(raw_x, 'X coordinate')
        if err:
            self.gui.set_feedback('Invalid navigation coordinates', error=True)
            self.get_logger().warn(err)
            return
        y, err = parse_coordinate(raw_y, 'Y coordinate')
        if err:
            self.gui.set_feedback('Invalid navigation coordinates', error=True)
            self.get_logger().warn(err)
            return
        yaw, err = parse_coordinate(raw_yaw, 'yaw')
        if err:
            self.gui.set_feedback('Invalid navigation coordinates', error=True)
            self.get_logger().warn(err)
            return

        bound_err = validate_goal(x, y, yaw)
        if bound_err:
            self.gui.set_feedback('Invalid navigation coordinates', error=True)
            self.get_logger().warn(bound_err)
            return

        self.submit_goal(x, y, yaw, source='start_mission')

    def _retry_pending_goal(self) -> None:
        if not self.start_request_pending or self._pending_goal is None:
            return
        if self.navigate_client.server_is_ready():
            x, y, yaw = self._pending_goal
            self.submit_goal(x, y, yaw, source='retry')

    def stop_mission(self) -> None:
        self.start_request_pending = False
        self._pending_goal = None
        if self.start_wait_timer is not None:
            self.start_wait_timer.cancel()
            self.start_wait_timer = None

        if self.goal_handle is None:
            self.gui.set_mission_status('ready')
            self.gui.set_mission_running(False)
            self.gui.set_feedback('Mission stopped')
            return

        self.gui.set_mission_status('stopping')
        future = self.goal_handle.cancel_goal_async()
        future.add_done_callback(self._cancel_response_callback)

    # -- obstacle UI ------------------------------------------------------

    def _request_path(self, path_key: Optional[str], label: str) -> None:
        if path_key is None:
            ok, message = self.obstacle_manager.request_random()
        elif path_key == PATH_A:
            ok, message = self.obstacle_manager.request_path_a()
        elif path_key == PATH_B:
            ok, message = self.obstacle_manager.request_path_b()
        else:
            ok, message = self.obstacle_manager.request_obstacle(path_key)
        self.gui.set_dyn_obstacle_status(message)
        self.gui.set_feedback(message if ok else message, error=not ok)
        if ok:
            self.get_logger().info(f'{label}: {message}')

    def block_path_a(self) -> None:
        self._request_path(PATH_A, 'BLOCK PATH A')

    def block_path_b(self) -> None:
        self._request_path(PATH_B, 'BLOCK PATH B')

    def random_block(self) -> None:
        self._request_path(None, 'RANDOM BLOCK')

    def clear_obstacles(self) -> None:
        ok, message = self.obstacle_manager.clear_obstacles()
        self.gui.set_dyn_obstacle_status(message)
        self.gui.set_feedback('Obstacles cleared' if ok else message, error=not ok)
        self.obstacle_manager.status_message = 'Obstacle: READY'

    def _obstacle_tick(self) -> None:
        pose = self.latest_pose
        robot_xy = (pose[0], pose[1]) if pose is not None else None
        status = self.obstacle_manager.tick(robot_xy)
        if status:
            self.gui.set_dyn_obstacle_status(status)
            if self.obstacle_manager.state == ObstacleState.ACTIVE:
                self.gui.set_feedback('Dynamic obstacle spawned')
        elif self.obstacle_manager.state == ObstacleState.WAITING:
            self.gui.set_dyn_obstacle_status(self.obstacle_manager.status_message)

    # -- Nav2 callbacks ---------------------------------------------------

    def _feedback_callback(self, feedback_message) -> None:
        feedback = feedback_message.feedback
        self.latest_distance = float(feedback.distance_remaining)
        duration = feedback.estimated_time_remaining
        self.latest_time = format_remaining_time(
            duration.sec + duration.nanosec / 1e9
        )

    def _goal_response_callback(self, future) -> None:
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.goal_handle = None
            self.active_goal = None
            self.gui.set_mission_status('rejected')
            self.gui.set_feedback('Goal rejected', error=True)
            self.gui.set_mission_running(False)
            return

        self.gui.set_mission_status('running')
        self.gui.set_mission_running(True)
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _cancel_response_callback(self, future) -> None:
        if future.result().goals_canceling:
            self.goal_handle = None
            self.gui.set_mission_status('ready')
            self.gui.set_mission_running(False)
            self.gui.set_feedback('Mission stopped')
        else:
            self.gui.set_mission_status('cancel failed')
            self.gui.set_feedback('Cancel failed', error=True)

    def _result_callback(self, future) -> None:
        status = future.result().status
        self.goal_handle = None
        self.gui.set_mission_running(False)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.gui.set_mission_status('complete')
            self.gui.set_feedback('Goal reached')
        elif status == GoalStatus.STATUS_CANCELED:
            self.gui.set_mission_status('ready')
            self.gui.set_feedback('Mission stopped')
        else:
            self.gui.set_mission_status('failed')
            self.gui.set_feedback('Goal failed', error=True)

    # -- sensors ----------------------------------------------------------

    def _odom_callback(self, msg: Odometry) -> None:
        self.latest_speed = abs(float(msg.twist.twist.linear.x))
        self.status.speed = self.latest_speed
        p = msg.pose.pose
        yaw = yaw_from_quaternion(
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w,
        )
        # Gazebo OdometryPublisher reports world-frame pose — matches gap coords.
        self.latest_pose = (float(p.position.x), float(p.position.y), yaw)

    def _scan_callback(self, msg: LaserScan) -> None:
        if not hasattr(msg, 'ranges'):
            return
        self.latest_obstacle_detected = detect_obstacle(msg.ranges, threshold=self.obstacle_threshold)
        self.status.obstacle_detected = self.latest_obstacle_detected

    def _camera_callback(self, msg: Image) -> None:
        self.gui.update_camera(msg)

    def _update_gui(self) -> None:
        self.status.speed = self.latest_speed
        self.status.obstacle_detected = self.latest_obstacle_detected
        if self.active_goal is not None:
            x, y, yaw = self.active_goal
            self.status.destination = f'({x:g}, {y:g}, {yaw:g})'
        else:
            self.status.destination = 'Safehouse'
        self.status.distance_to_destination = self.latest_distance
        self.status.time_to_destination = self.latest_time
        self.status.battery = 100.0
        self.gui.set_status(self.status)


def main(args=None):
    rclpy.init(args=args)

    # Temporary node so we can read launch-file parameters before creating the GUI.
    param_node = Node('robot_status_gui_param_reader')
    robot_name = str(param_node.declare_parameter('robot_name', 'husky1').value)
    param_node.destroy_node()

    gui = RobotStatusWindow(robot_name)
    node = RobotStatusNode(gui, robot_name)
    gui.set_mission_callback(node.toggle_mission)
    gui.set_obstacle_callbacks(
        node.block_path_a,
        node.block_path_b,
        node.random_block,
        node.clear_obstacles,
    )

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            gui.update_idletasks()
            gui.update()
    except KeyboardInterrupt:
        pass
    finally:
        gui.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
