#!/usr/bin/env python3
"""Small modular GUI for displaying robot status during autonomy demos.

Supports:
  * Unified START/STOP MISSION via mission_owner/start_mission (default)
  * Optional NavigateToPose fallback only when allow_ntp_fallback:=true
  * Dynamic Path A / Path B / Random block (5 m spatial trigger) + Clear
"""

from __future__ import annotations

import math
import os
import sys
import threading
import time
from dataclasses import dataclass
from math import cos, sin
from typing import Iterable, List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

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
from rs1_nav.gazebo_world import GoalVisualSpec  # noqa: E402
from rs1_nav.geometry import yaw_from_quaternion  # noqa: E402


# Defaults for bush_trail_world (current husky launch default). On-trail goal
# from Entry 019 — (18, 0) is occupied on the regenerated PGM.
DEFAULT_GOAL_X = 16.0
DEFAULT_GOAL_Y = -0.1
DEFAULT_GOAL_YAW = 0.0
DEFAULT_WORLD_NAME = 'bush_trail_world'
GOAL_VISUAL_NAME = 'navigation_goal_visual'
# Faiyad UI: treat mission arrival within 1 m of the goal visual.
ARRIVAL_DISTANCE_THRESHOLD_M = 1.0
CAMERA_DISPLAY_SIZE = (500, 300)
MAP_DISPLAY_SIZE = (500, 300)


def distance_to_goal(
    robot_pose: Tuple[float, float, float],
    goal_pose: Optional[Tuple[float, float, float]],
) -> float:
    """Return planar distance between a robot pose and navigation goal."""
    if goal_pose is None:
        return 0.0
    return math.hypot(robot_pose[0] - goal_pose[0], robot_pose[1] - goal_pose[1])


def distance_for_display(
    feedback_distance: float,
    robot_pose: Optional[Tuple[float, float, float]],
    goal_pose: Optional[Tuple[float, float, float]],
) -> float:
    """Choose Nav2's route distance, falling back to live odometry distance."""
    if math.isfinite(feedback_distance) and feedback_distance > 0.0:
        return feedback_distance
    if robot_pose is not None:
        return distance_to_goal(robot_pose, goal_pose)
    return max(0.0, feedback_distance) if math.isfinite(feedback_distance) else 0.0


def _thermal_colour(temperature_k: float) -> Tuple[int, int, int]:
    """Map absolute Kelvin values to thermal hue families (Faiyad GUI)."""
    stops = (
        (273.15, (8, 12, 72)),
        (288.0, (20, 32, 125)),
        (300.0, (48, 55, 165)),
        (330.0, (105, 48, 175)),
        (400.0, (130, 18, 28)),
        (500.0, (205, 35, 0)),
        (560.0, (235, 65, 0)),
        (590.0, (255, 95, 0)),
        (600.0, (255, 125, 0)),
    )
    temperature_k = max(stops[0][0], temperature_k)
    for (start, start_rgb), (end, end_rgb) in zip(stops, stops[1:]):
        if temperature_k <= end:
            fraction = (temperature_k - start) / (end - start)
            return tuple(
                int(first + (second - first) * fraction)
                for first, second in zip(start_rgb, end_rgb)
            )
    return stops[-1][1]


def resize_rgb(
    pixels: bytes,
    source_width: int,
    source_height: int,
    target_width: int,
    target_height: int,
) -> Tuple[bytes, int, int]:
    """Scale RGB pixels to fit a display area without distorting the image."""
    if source_width <= 0 or source_height <= 0:
        return pixels, source_width, source_height
    scale = min(target_width / source_width, target_height / source_height)
    width = max(1, int(source_width * scale))
    height = max(1, int(source_height * scale))
    if width == source_width and height == source_height:
        return pixels, width, height
    resized = bytearray(width * height * 3)
    for y in range(height):
        source_y = min(source_height - 1, int(y * source_height / height))
        for x in range(width):
            source_x = min(source_width - 1, int(x * source_width / width))
            source_index = (source_y * source_width + source_x) * 3
            target_index = (y * width + x) * 3
            resized[target_index:target_index + 3] = pixels[
                source_index:source_index + 3
            ]
    return bytes(resized), width, height

# Soft bounds for coordinate validation (60×60 m bush_trail / forest arena).
MAP_X_MIN, MAP_X_MAX = -29.0, 29.0
MAP_Y_MIN, MAP_Y_MAX = -29.0, 29.0


@dataclass
class RobotStatus:
    """Simple in-memory snapshot of values shown in the GUI."""

    robot_name: str = 'Rescue Bot'
    speed: float = 0.0
    destination: str = 'Safehouse'
    distance_to_destination: float = 0.0
    battery: float = 100.0
    obstacle_detected: bool = False
    fire_detected: bool = False

    def to_display_lines(self) -> List[str]:
        return [
            f'Robot Name: {self.robot_name}',
            f'Speed: {format_speed(self.speed)}',
            f'Distance to Destination: {self.distance_to_destination:.1f} m',
            f'Obstacle detected: {"yes" if self.obstacle_detected else "no"}',
            f'Fire detected: {"yes" if self.fire_detected else "no"}',
        ]


def format_speed(speed: float) -> str:
    return f'{float(speed):.2f} m/s'


# Entry 063 Wave-1 honest GUI speed presets (labels == applied MPPI envelope).
# Fast/MAXIMUM = 0.50 m/s / 1.2 rad/s — not historical RPP test_speed 1.0 / 0.60.
SPEED_PRESETS = {'slow': 0.15, 'medium': 0.30, 'fast': 0.50}
SPEED_ANGULAR_MAX = 1.2


def speed_param_writes(speed: str) -> dict:
    """Map GUI speed key → controller/smoother SetParameters payloads.

    Primary writes: FollowPath.vx_max / wz_max + velocity_smoother
    max_velocity / min_velocity (lockstep). Does not use desired_linear_vel.
    """
    if speed not in SPEED_PRESETS:
        raise KeyError(speed)
    vx = float(SPEED_PRESETS[speed])
    wz = float(SPEED_ANGULAR_MAX)
    return {
        'FollowPath.vx_max': vx,
        'FollowPath.wz_max': wz,
        'max_velocity': [vx, 0.0, wz],
        'min_velocity': [-vx, 0.0, -wz],
    }


# Path-bank MissionNavState values that should drive the Mission Status label.
_PATH_BANK_NAVIGATING = frozenset({
    'FOLLOWING_PATH',
    'SWITCHING_PATH',
    'PLANNING',
    'PATH_BLOCKED',
})
# Bare NO_PATH_AVAILABLE is prelude / all-blocked advisory — NOT "returning".
_PATH_BANK_RETURNING = frozenset({
    'RETURNING_TO_START',
})
_PATH_BANK_ALL_BLOCKED_PRELUDE = frozenset({
    'NO_PATH_AVAILABLE',
})
_PATH_BANK_RETURN_TERMINAL = frozenset({
    'RETURNING_TO_START',
    'RETURNED_TO_START',
})
# Terminal / blocked states (path_bank or owner inhibit) → banner copy.
_NO_SAFE_ROUTE_STATES = frozenset({
    'NO_SAFE_ROUTE',
    'ALL_BLOCKED_NO_HOME',
})
_INHIBIT_BANNER = {
    'ALL_BLOCKED_NO_HOME': 'all blocked - no home route',
    'NO_SAFE_ROUTE': 'no safe route',
    'THERMAL_FIRE_PENDING': 'thermal fire pending - holding',
    'PENDING_FIRE_UNRESOLVED': 'pending fire unresolved - stopped',
}

# Display caps: throttle ≤ typical sensor rates (thermal 10 Hz, RGB camera_update_rate=3).
# Latest-frame drop is the acceptance bar — do not claim fixed FPS under Gazebo load.
DEFAULT_CAMERA_DISPLAY_HZ = 3.0
DEFAULT_THERMAL_DISPLAY_HZ = 10.0


def parse_path_bank_active_id(status_text: str) -> Optional[int]:
    """Extract active_id=N from a multiline path_bank/status payload."""
    for line in (status_text or '').splitlines():
        line = line.strip()
        if line.startswith('active_id='):
            raw = line.split('=', 1)[1].strip()
            if raw in ('', 'None', 'none'):
                return None
            try:
                return int(raw)
            except ValueError:
                return None
    return None


def parse_last_no_safe_detail(status_text: str) -> str:
    """Extract last_no_safe_detail=TOKEN from path_bank/status (H6)."""
    for line in (status_text or '').splitlines():
        line = line.strip()
        if line.startswith('last_no_safe_detail='):
            return line.split('=', 1)[1].strip()
    return ''


def _no_safe_detail_token(
    *,
    owner_detail: str = '',
    path_bank_detail: str = '',
) -> str:
    """Prefer owner.detail suffix NO_SAFE_ROUTE:<token>, else path_bank field."""
    od = (owner_detail or '').strip()
    if od.upper().startswith('NO_SAFE_ROUTE:'):
        token = od.split(':', 1)[1].strip()
        if token:
            return token
    pb = (path_bank_detail or '').strip()
    if pb and pb.lower() not in ('none', 'null'):
        return pb
    return ''


def _append_no_home_detail(base: str, detail_token: str) -> str:
    token = (detail_token or '').strip()
    if not token:
        return base
    if f'({token})' in base:
        return base
    return f'{base} ({token})'


def mission_status_from_path_bank(
    state: str,
    active_id: Optional[int] = None,
    *,
    no_safe_detail: str = '',
) -> Optional[str]:
    """Map path_bank/state to a Mission Status display string.

    Returns None when the state should not override the Nav2-client label
    (e.g. IDLE before a mission starts).

    H6: when state is NO_SAFE_ROUTE / ALL_BLOCKED_NO_HOME, append ``(detail)``
    when ``no_safe_detail`` is non-empty.
    """
    key = (state or '').strip().upper()
    if not key:
        return None
    if key in _NO_SAFE_ROUTE_STATES:
        base = _INHIBIT_BANNER.get(key, 'no safe route')
        return _append_no_home_detail(base, no_safe_detail)
    if key in _PATH_BANK_RETURNING:
        return 'all paths blocked - returning to home'
    if key in _PATH_BANK_ALL_BLOCKED_PRELUDE:
        return 'all paths blocked'
    if key == 'RETURNED_TO_START':
        return 'returned to start'
    if key == 'ARRIVED':
        return 'complete'
    if key == 'FAILED':
        return 'failed'
    if key in _PATH_BANK_NAVIGATING:
        if active_id is not None:
            return f'navigating to goal (path {active_id} active)'
        return 'navigating to goal'
    return None


def _format_inhibit_label(inhibit: str, detail_token: str = '') -> str:
    key = (inhibit or '').strip().upper()
    base = _INHIBIT_BANNER.get(key, inhibit)
    if key == 'ALL_BLOCKED_NO_HOME':
        return _append_no_home_detail(base, detail_token)
    return base


def mission_banner_from_owner(
    state: str,
    *,
    operator_stop_latch: bool = False,
    active_inhibits: Optional[Iterable[str]] = None,
    command_authority: str = 'NONE',
    stop_reason: str = '',
    mission_running: bool = False,
    detail: str = '',
    no_safe_detail: str = '',
) -> str:
    """Map MissionOwnerStatus fields to a Mission banner label (no 'Mission:' prefix).

    H6: STOPPED + ALL_BLOCKED_NO_HOME appends ``(detail)`` from owner.detail
    suffix or path_bank ``last_no_safe_detail``. Mid-return never shows NO HOME.
    """
    phase = (state or 'IDLE').strip().upper() or 'IDLE'
    inhibits = [str(i).strip() for i in (active_inhibits or []) if str(i).strip()]
    auth = (command_authority or 'NONE').strip().upper() or 'NONE'
    reason = (stop_reason or '').strip()
    detail_token = _no_safe_detail_token(
        owner_detail=detail, path_bank_detail=no_safe_detail)
    if phase in ('HOLDING', 'COSTMAP_REBUILD') and inhibits:
        return f'{phase}: {_format_inhibit_label(inhibits[0], detail_token)}'
    if phase == 'STOPPED':
        if inhibits:
            return (
                f'STOPPED: {_format_inhibit_label(inhibits[0], detail_token)}'
            )
        if operator_stop_latch or reason == 'OPERATOR_STOP':
            return 'STOPPED'
        if reason:
            friendly = _INHIBIT_BANNER.get(reason.upper())
            label = friendly or reason
            if reason.upper() == 'ALL_BLOCKED_NO_HOME':
                label = _append_no_home_detail(label, detail_token)
            return f'STOPPED: {label}'
        return 'STOPPED'
    if phase == 'IDLE':
        return 'IDLE'
    if phase == 'COSTMAP_REBUILD':
        return 'costmap rebuild - holding'
    if mission_running or phase in (
        'NAVIGATING', 'SWITCHING', 'RETURNING_TO_START', 'MANUAL_NAV',
    ):
        if phase == 'RETURNING_TO_START':
            return 'all paths blocked - returning to home'
        if auth not in ('NAV', 'BT_MANUAL', 'TELEOP'):
            if inhibits:
                return (
                    f'{phase.lower()}: '
                    f'{_format_inhibit_label(inhibits[0], detail_token)}'
                )
            return f'{phase.lower()} (no command authority)'
        return 'running'
    if phase == 'STOPPING':
        return 'stopping'
    return phase.lower()


def rgb_image_to_ppm(image: Image) -> Tuple[Optional[bytes], Optional[str]]:
    """Convert a camera Image to PPM bytes. Returns (ppm, error)."""
    if image.encoding not in ('rgb8', 'bgr8', 'rgba8', 'bgra8', 'mono8'):
        return None, f'unsupported {image.encoding}'
    if image.width <= 0 or image.height <= 0:
        return None, 'invalid image frame'

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

    return f'P6\n{image.width} {image.height}\n255\n'.encode() + pixels, None


def thermal_image_to_ppm(image: Image) -> Tuple[Optional[bytes], Optional[str]]:
    """Convert mono16 thermal Image to Kelvin false-color PPM. Returns (ppm, error)."""
    if image.encoding != 'mono16' or image.width <= 0 or image.height <= 0:
        return None, f'unsupported {image.encoding}'

    values = []
    for row in range(image.height):
        start = row * image.step
        row_bytes = image.data[start:start + image.width * 2]
        values.extend(
            row_bytes[index] | (row_bytes[index + 1] << 8)
            for index in range(0, len(row_bytes) - 1, 2)
        )
    if not values:
        return None, 'invalid frame'

    pixels = bytearray()
    for value in values:
        pixels.extend(_thermal_colour(value * 0.01))

    return f'P6\n{image.width} {image.height}\n255\n'.encode() + pixels, None


def start_mission_action(
    *,
    client_available: bool,
    service_ready: bool,
    allow_ntp_fallback: bool,
) -> str:
    """Decide Start Mission transport. Pure helper for unit tests.

    Returns one of: 'start_mission', 'wait_mission_owner', 'ntp_fallback', 'fail_closed'.
    """
    if client_available and service_ready:
        return 'start_mission'
    if client_available and not service_ready:
        return 'wait_mission_owner'
    if allow_ntp_fallback:
        return 'ntp_fallback'
    return 'fail_closed'


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
        # Taller default so Start/BLOCK/CLEAR stay on-screen with pinned Speed
        # (Entry 063 Phase Four REAL_GUI); content still scrolls if needed.
        self.geometry('560x780')
        self.minsize(420, 520)

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

        # Entry 063 Phase Four: pin Speed control outside the scroll canvas so
        # REAL_GUI xdotool can always hit Fast/MAXIMUM (scroll-canvas radios
        # reported off-window root coords and clicks never applied).
        self.speed_frame = tk.LabelFrame(
            self, text='Speed control', bg='#f3f4f6', fg='#1f2937',
            font=('Arial', 11, 'bold'), padx=8, pady=4,
        )
        self.speed_frame.pack(fill='x', padx=12, pady=(0, 6))
        self.speed_var = tk.StringVar(value='Speed: 0.00 m/s')
        tk.Label(
            self.speed_frame, textvariable=self.speed_var, font=('Arial', 16, 'bold'),
            bg='#f3f4f6', fg='#0f172a', anchor='w', padx=8, pady=4,
        ).pack(fill='x')
        self.speed_choice_var = tk.StringVar(value='slow')
        self.speed_buttons = {}
        # Push-buttons (not Radiobutton): same click class as BLOCK FIRE / proven
        # with Entry 062 xdotool absolute clicks. Labels == applied limits.
        for value, label in (
            ('slow', 'Slow 0.15'),
            ('medium', 'Medium 0.30'),
            ('fast', 'Fast 0.50'),
        ):
            button = tk.Button(
                self.speed_frame, text=label, state='disabled',
                bg='#e5e7eb', fg='#111827', font=('Arial', 12, 'bold'),
                height=2, relief='raised',
            )
            button.pack(side='left', expand=True, fill='x', padx=2)
            self.speed_buttons[value] = button

        # Scrollable body keeps both operator sections reachable on small displays.
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

        self.robot_section = tk.LabelFrame(
            body, text='Robot GUI', bg='#dbeafe', fg='#1e3a8a',
            font=('Arial', 12, 'bold'), padx=6, pady=6,
        )
        self.robot_section.pack(fill='x', padx=12, pady=(0, 10))

        self.sim_section = tk.LabelFrame(
            body, text='Sim GUI', bg='#fee2e2', fg='#991b1b',
            font=('Arial', 12, 'bold'), padx=6, pady=6,
        )
        self.sim_section.pack(fill='x', padx=12, pady=(0, 20))

        self.views_frame = tk.Frame(self.robot_section, bg='#dbeafe')
        self.views_frame.pack(fill='x', padx=8, pady=(0, 10))
        for column in range(3):
            self.views_frame.columnconfigure(column, weight=1)

        self.map_frame = tk.Frame(self.views_frame, width=520, height=320, bg='#111827')
        self.map_frame.grid(row=0, column=0, padx=4, sticky='nsew')
        self.map_frame.grid_propagate(False)
        self.map_label = tk.Label(
            self.map_frame,
            text='Nav2 cost map: waiting for global costmap',
            bg='#111827',
            fg='#ffffff',
            anchor='center',
        )
        self.map_label.pack(fill='both', expand=True)
        self.map_photo = None

        self.camera_frame = tk.Frame(self.views_frame, width=520, height=320, bg='#111827')
        self.camera_frame.grid(row=0, column=1, padx=4, sticky='nsew')
        self.camera_frame.grid_propagate(False)
        self.camera_label = tk.Label(
            self.camera_frame,
            text='Camera: waiting for image',
            bg='#111827',
            fg='#ffffff',
            anchor='center',
        )
        self.camera_label.pack(fill='both', expand=True)
        self.camera_photo = None
        self.camera_luminance = None

        self.thermal_frame = tk.Frame(
            self.views_frame, width=520, height=320, bg='#111827',
        )
        self.thermal_frame.grid(row=0, column=2, padx=4, sticky='nsew')
        self.thermal_frame.grid_propagate(False)
        self.thermal_label = tk.Label(
            self.thermal_frame,
            text='Thermal camera: waiting for image',
            bg='#111827',
            fg='#ffffff',
            anchor='center',
        )
        self.thermal_label.pack(fill='both', expand=True)
        self.thermal_photo = None
        self.thermal_mode = 'FUSED THERMAL'
        self.thermal_mode_button = tk.Button(
            self.thermal_frame,
            text=self.thermal_mode,
            command=self.toggle_thermal_mode,
            bg='#374151',
            fg='#ffffff',
            activebackground='#4b5563',
            activeforeground='#ffffff',
            relief='flat',
        )
        self.thermal_mode_button.pack(fill='x')

        self.info_frame = tk.Frame(self.robot_section, bg='#ffffff', bd=1, relief='solid')
        self.info_frame.pack(fill='x', padx=20, pady=(0, 10))

        self.distance_var = tk.StringVar(value='Distance to Destination: 0.0 m')
        self.battery_var = tk.StringVar(value='Battery: 100%')
        self.obstacle_var = tk.StringVar(value='Obstacle detected: no')
        self.fire_var = tk.StringVar(value='Fire detected: no')
        # Healthy owner boot is IDLE — do not imply operator-stop / latch.
        self.mission_var = tk.StringVar(value='Mission: IDLE')
        self.recent_status_var = tk.StringVar(value='Recent Status: system ready')

        self.mission_label = tk.Label(
            self.info_frame, textvariable=self.mission_var,
            font=('Arial', 22, 'bold'), bg='#64748b', fg='#ffffff',
            anchor='w', padx=16, pady=10,
        )
        self.labels = [
            self.mission_label,
            tk.Label(self.info_frame, textvariable=self.distance_var, font=('Arial', 18, 'bold'), bg='#ffffff', fg='#0f172a', anchor='w', padx=16, pady=8),
            tk.Label(self.info_frame, textvariable=self.obstacle_var, font=('Arial', 12), bg='#ffffff', fg='#111827', anchor='w', padx=16, pady=3),
            tk.Label(self.info_frame, textvariable=self.fire_var, font=('Arial', 12), bg='#ffffff', fg='#111827', anchor='w', padx=16, pady=3),
            tk.Label(self.info_frame, textvariable=self.recent_status_var, font=('Arial', 12), bg='#ffffff', fg='#2563eb', anchor='w', padx=16, pady=6),
        ]

        for label in self.labels:
            label.pack(fill='x')

        # --- Navigation Goal (X/Y/Yaw + unified Start/Stop) ---
        self.goal_frame = tk.LabelFrame(
            self.robot_section,
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
            bg='#16a34a',
            activebackground='#15803d',
            fg='#ffffff',
            activeforeground='#ffffff',
            font=('Arial', 12, 'bold'),
            relief='raised',
            bd=3,
        )
        self.mission_button.grid(
            row=3, column=0, columnspan=2, sticky='ew', pady=(10, 0), ipady=2,
        )
        self._mission_running = False

        # --- Dynamic Obstacles: Path A / Path B / Random / Clear ---
        self.obstacle_frame = tk.LabelFrame(
            self.sim_section,
            text='DYNAMIC FIRE',
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
            text='BLOCK FIRE A',
            state='disabled',
            command=lambda: None,
            height=2,
        )
        self.block_path_a_button.grid(
            row=0, column=0, sticky='ew', padx=(0, 6), pady=4, ipady=4,
        )
        self.block_path_b_button = tk.Button(
            self.obstacle_frame,
            text='BLOCK FIRE B',
            state='disabled',
            command=lambda: None,
            height=2,
        )
        self.block_path_b_button.grid(
            row=0, column=1, sticky='ew', padx=(6, 0), pady=4, ipady=4,
        )
        self.random_block_button = tk.Button(
            self.obstacle_frame,
            text='RANDOM FIRE',
            state='disabled',
            command=lambda: None,
            height=2,
        )
        self.random_block_button.grid(
            row=1, column=0, sticky='ew', padx=(0, 6), pady=4, ipady=4,
        )
        self.block_both_paths_button = tk.Button(
            self.obstacle_frame,
            text='BLOCK BOTH PATHS',
            state='disabled',
            command=lambda: None,
            height=2,
        )
        self.block_both_paths_button.grid(
            row=1, column=1, sticky='ew', padx=(6, 0), pady=4, ipady=4,
        )
        self.clear_obstacle_button = tk.Button(
            self.obstacle_frame,
            text='CLEAR FIRE',
            state='disabled',
            command=lambda: None,
            height=2,
        )
        self.clear_obstacle_button.grid(
            row=2, column=0, columnspan=2, sticky='ew', padx=0, pady=4, ipady=4,
        )

        self.sim_updates = []
        self.sim_update_var = tk.StringVar(value='No simulation updates yet')
        self.sim_update_frame = tk.LabelFrame(
            self.sim_section,
            text='SIM UPDATE',
            bg='#fee2e2',
            fg='#991b1b',
            font=('Arial', 11, 'bold'),
            padx=10,
            pady=8,
        )
        self.sim_update_frame.pack(fill='x', padx=20, pady=(0, 20))
        tk.Label(
            self.sim_update_frame,
            textvariable=self.sim_update_var,
            justify='left',
            anchor='w',
            font=('Courier New', 10),
            bg='#111827',
            fg='#e2e8f0',
            padx=10,
            pady=8,
        ).pack(fill='x')

        self.update_status()
        self._bind_mousewheel_tree(body)
        self.after_idle(self._refresh_scroll_region)
        # Optional REAL_GUI calibration: dump on-screen button centres for xdotool.
        if os.environ.get('RS1_GUI_DUMP_GEOMETRY', '').strip() in ('1', 'true', 'TRUE'):
            self.after(800, self._dump_button_geometry_loop)

    def _dump_button_geometry_loop(self) -> None:
        """Write visible button centres to /tmp for desktop automation."""
        try:
            import json
            import time as _time
            from pathlib import Path

            self.update_idletasks()
            payload = {
                'ts': _time.time(),
                'title': str(self.title()) if hasattr(self, 'title') else 'husky1 status',
            }
            for key, widget in (
                ('START MISSION', self.mission_button),
                ('BLOCK FIRE A', self.block_path_a_button),
                ('BLOCK FIRE B', self.block_path_b_button),
                ('RANDOM FIRE', self.random_block_button),
                ('CLEAR FIRE', self.clear_obstacle_button),
                # Entry 063 Phase Four: REAL_GUI max-speed click targets.
                ('Slow 0.15', self.speed_buttons.get('slow')),
                ('Medium 0.30', self.speed_buttons.get('medium')),
                ('Fast 0.50', self.speed_buttons.get('fast')),
            ):
                if widget is None:
                    continue
                try:
                    w = max(1, int(widget.winfo_width()))
                    h = max(1, int(widget.winfo_height()))
                    rx = int(widget.winfo_rootx())
                    ry = int(widget.winfo_rooty())
                    payload[key] = {
                        'root_x': rx,
                        'root_y': ry,
                        'width': w,
                        'height': h,
                        'cx': rx + w // 2,
                        'cy': ry + h // 2,
                        'viewable': bool(widget.winfo_viewable()),
                        'text': str(widget.cget('text')),
                        'state': str(widget.cget('state')),
                    }
                except Exception as exc:  # noqa: BLE001
                    payload[key] = {'error': str(exc)}
            Path('/tmp/rs1_gui_button_geom.json').write_text(
                json.dumps(payload, indent=2)
            )
        except Exception:
            pass
        if os.environ.get('RS1_GUI_DUMP_GEOMETRY', '').strip() in ('1', 'true', 'TRUE'):
            # Faster refresh so automation can wait on post-scroll updates.
            self.after(250, self._dump_button_geometry_loop)

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

    def set_speed_callback(self, callback) -> None:
        for value, button in self.speed_buttons.items():
            button.configure(
                command=lambda selected=value: callback(selected),
                state='normal',
            )

    def set_speed(self, value: str) -> None:
        self.speed_choice_var.set(value)
        # Visual selected state for pinned push-button speed presets.
        for key, button in self.speed_buttons.items():
            if key == value:
                button.configure(bg='#86efac', relief='sunken')
            else:
                button.configure(bg='#e5e7eb', relief='raised')

    def set_obstacle_callbacks(
        self,
        path_a_callback,
        path_b_callback,
        random_callback,
        both_callback,
        clear_callback,
    ) -> None:
        self.block_path_a_button.configure(command=path_a_callback, state='normal')
        self.block_path_b_button.configure(command=path_b_callback, state='normal')
        self.random_block_button.configure(command=random_callback, state='normal')
        self.block_both_paths_button.configure(command=both_callback, state='normal')
        self.clear_obstacle_button.configure(command=clear_callback, state='normal')

    def set_mission_running(self, running: bool) -> None:
        self._mission_running = bool(running)
        if running:
            self.mission_button.configure(
                text='STOP MISSION', bg='#dc2626', activebackground='#b91c1c',
                state='normal',
            )
        else:
            self.mission_button.configure(
                text='START MISSION', bg='#16a34a', activebackground='#15803d',
                state='normal',
            )

    def set_mission_status(self, status: str) -> None:
        normalized = status.strip().lower()
        if normalized in (
            'running',
            'starting',
            'complete',
            'returned to start',
        ) or normalized.startswith('navigating to goal'):
            color = '#16a34a'
        elif normalized in (
            'waiting for nav2',
            'waiting for mission owner',
            'stopping',
            'all paths blocked - returning to home',
        ):
            color = '#d97706'
        elif normalized in ('idle',):
            color = '#64748b'
        else:
            # stopped / rejected / failed / inhibit codes
            color = '#dc2626'
        self.mission_var.set(f'Mission: {status.upper()}')
        self.mission_label.configure(bg=color)

    def set_goal_status(self, status: str) -> None:
        return

    def set_dyn_obstacle_status(self, status: str) -> None:
        if status:
            self.sim_updates.append(str(status).replace('Obstacle', 'Fire'))
            self.sim_updates = self.sim_updates[-5:]
            self.sim_update_var.set('\n'.join(self.sim_updates))

    def set_feedback(self, message: str, *, error: bool = False) -> None:
        self.recent_status_var.set(f'Recent Status: {message}')

    def set_fire_status(self, detected: bool, location: Optional[str] = None) -> None:
        if detected and location:
            self.fire_var.set(f'Fire detected: yes ({location})')
        else:
            self.fire_var.set(f'Fire detected: {"yes" if detected else "no"}')

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
        """Render one RGB frame and cache luminance for fused thermal (Faiyad)."""
        if image.encoding not in ('rgb8', 'bgr8', 'rgba8', 'bgra8', 'mono8'):
            self.camera_label.configure(
                text=f'Camera: unsupported {image.encoding}', image='',
            )
            return

        source_channels = (
            1 if image.encoding == 'mono8'
            else 4 if image.encoding in ('rgba8', 'bgra8')
            else 3
        )
        row_width = image.width * source_channels
        pixels = bytearray()
        luminance: List[int] = []
        for row in range(image.height):
            start = row * image.step
            current = image.data[start:start + row_width]
            row_luminance: List[int] = []
            if image.encoding == 'mono8':
                row_luminance = list(current[:image.width])
            else:
                channel_count = 4 if image.encoding in ('rgba8', 'bgra8') else 3
                for pixel in range(image.width):
                    offset = pixel * channel_count
                    if image.encoding in ('bgr8', 'bgra8'):
                        r, g, b = (
                            current[offset + 2],
                            current[offset + 1],
                            current[offset],
                        )
                    else:
                        r, g, b = (
                            current[offset],
                            current[offset + 1],
                            current[offset + 2],
                        )
                    row_luminance.append(int(0.299 * r + 0.587 * g + 0.114 * b))
            luminance.extend(row_luminance)
            if image.encoding in ('bgr8', 'bgra8'):
                current = bytes(
                    current[index]
                    for pixel in range(image.width)
                    for index in (
                        pixel * (4 if image.encoding == 'bgra8' else 3) + 2,
                        pixel * (4 if image.encoding == 'bgra8' else 3) + 1,
                        pixel * (4 if image.encoding == 'bgra8' else 3),
                    )
                )
            elif image.encoding in ('rgba8', 'bgra8'):
                current = bytes(
                    current[index]
                    for pixel in range(image.width)
                    for index in (pixel * 4, pixel * 4 + 1, pixel * 4 + 2)
                )
            elif image.encoding == 'mono8':
                current = bytes(value for value in current for _ in range(3))
            pixels.extend(current)

        display_pixels, display_width, display_height = resize_rgb(
            pixels, image.width, image.height, *CAMERA_DISPLAY_SIZE,
        )
        ppm = (
            f'P6\n{display_width} {display_height}\n255\n'.encode()
            + display_pixels
        )
        try:
            self.camera_photo = tk.PhotoImage(data=ppm, format='PPM')
        except tk.TclError:
            self.camera_label.configure(text='Camera: invalid image frame', image='')
            return
        self.camera_label.configure(image=self.camera_photo, text='')
        self.camera_luminance = (image.width, image.height, luminance)

    def toggle_thermal_mode(self) -> None:
        self.thermal_mode = (
            'RAW THERMAL' if self.thermal_mode == 'FUSED THERMAL'
            else 'FUSED THERMAL'
        )
        self.thermal_mode_button.configure(text=self.thermal_mode)

    def _thermal_brightness(self, width: int, height: int) -> Optional[List[float]]:
        if self.thermal_mode != 'FUSED THERMAL' or self.camera_luminance is None:
            return None
        camera_width, camera_height, luminance = self.camera_luminance
        if camera_width <= 0 or camera_height <= 0:
            return None
        brightness: List[float] = []
        for y in range(height):
            source_y = min(camera_height - 1, int(y * camera_height / height))
            for x in range(width):
                # RGB FOV is 2x wider than thermal FOV; use its centred half.
                source_x = min(
                    camera_width - 1,
                    int((camera_width * 0.25) + x * camera_width * 0.5 / width),
                )
                value = luminance[source_y * camera_width + source_x] / 255.0
                tone = value ** 0.60
                brightness.append(0.50 + 0.48 * tone)
        return brightness

    def update_thermal(self, image: Image) -> None:
        """Render Kelvin thermal, optionally fused with RGB luminance (Faiyad)."""
        if image.encoding != 'mono16' or image.width <= 0 or image.height <= 0:
            self.thermal_label.configure(
                text=f'Thermal camera: unsupported {image.encoding}', image='',
            )
            return

        values = []
        for row in range(image.height):
            start = row * image.step
            row_bytes = image.data[start:start + image.width * 2]
            values.extend(
                row_bytes[index] | (row_bytes[index + 1] << 8)
                for index in range(0, len(row_bytes) - 1, 2)
            )
        if not values:
            return

        brightness = self._thermal_brightness(image.width, image.height)
        pixels = bytearray()
        for value in values:
            temperature_k = value * 0.01
            red, green, blue = _thermal_colour(temperature_k)
            if brightness is not None:
                factor = brightness[len(pixels) // 3]
                if temperature_k >= 400.0:
                    fire_brightness = min(
                        1.0, 0.80 + (temperature_k - 400.0) / 1000.0,
                    )
                    factor = max(factor, fire_brightness)
                red, green, blue = (
                    min(255, int(red * factor)),
                    min(255, int(green * factor)),
                    min(255, int(blue * factor)),
                )
            pixels.extend((red, green, blue))

        display_pixels, display_width, display_height = resize_rgb(
            pixels, image.width, image.height, *CAMERA_DISPLAY_SIZE,
        )
        ppm = (
            f'P6\n{display_width} {display_height}\n255\n'.encode()
            + display_pixels
        )
        try:
            self.thermal_photo = tk.PhotoImage(data=ppm, format='PPM')
        except tk.TclError:
            self.thermal_label.configure(text='Thermal camera: invalid frame', image='')
            return
        self.thermal_label.configure(image=self.thermal_photo, text='')

    def update_map(
        self,
        grid: OccupancyGrid,
        robot_pose: Optional[Tuple[float, float, float]] = None,
        goal_pose: Optional[Tuple[float, float, float]] = None,
        path: Optional[List[Tuple[float, float]]] = None,
    ) -> None:
        width = int(grid.info.width)
        height = int(grid.info.height)
        if width <= 0 or height <= 0 or len(grid.data) < width * height:
            return

        display_width = min(MAP_DISPLAY_SIZE[0], width)
        display_height = min(MAP_DISPLAY_SIZE[1], height)
        pixels = bytearray()
        for display_y in range(display_height):
            source_y = height - 1 - (display_y * height // display_height)
            for display_x in range(display_width):
                source_x = display_x * width // display_width
                occupancy = grid.data[source_y * width + source_x]
                if occupancy < 0:
                    color = (100, 116, 139)  # unknown
                elif occupancy >= 50:
                    color = (127, 29, 29)  # occupied / hazard red
                else:
                    color = (241, 245, 249)  # traversable space
                pixels.extend(color)

        def map_pixel(pose: Optional[Tuple[float, float, float]]) -> Optional[Tuple[int, int]]:
            if pose is None or grid.info.resolution <= 0.0:
                return None
            pixel_x = int(
                (pose[0] - grid.info.origin.position.x)
                / (grid.info.resolution * width)
                * display_width
            )
            pixel_y = display_height - 1 - int(
                (pose[1] - grid.info.origin.position.y)
                / (grid.info.resolution * height)
                * display_height
            )
            if 0 <= pixel_x < display_width and 0 <= pixel_y < display_height:
                return pixel_x, pixel_y
            return None

        def draw_marker(point: Optional[Tuple[int, int]], color: Tuple[int, int, int]) -> None:
            if point is None:
                return
            center_x, center_y = point
            radius = 6
            for offset_y in range(-radius, radius + 1):
                for offset_x in range(-radius, radius + 1):
                    if offset_x * offset_x + offset_y * offset_y > radius * radius:
                        continue
                    pixel_x = center_x + offset_x
                    pixel_y = center_y + offset_y
                    if 0 <= pixel_x < display_width and 0 <= pixel_y < display_height:
                        index = (pixel_y * display_width + pixel_x) * 3
                        pixels[index:index + 3] = bytes(color)

        def draw_path(points: List[Tuple[int, int]], color: Tuple[int, int, int]) -> None:
            for start, end in zip(points, points[1:]):
                x0, y0 = start
                x1, y1 = end
                steps = max(abs(x1 - x0), abs(y1 - y0), 1)
                for step in range(steps + 1):
                    x = round(x0 + (x1 - x0) * step / steps)
                    y = round(y0 + (y1 - y0) * step / steps)
                    for offset_x, offset_y in (
                        (0, 0), (1, 0), (-1, 0), (0, 1), (0, -1),
                    ):
                        pixel_x = x + offset_x
                        pixel_y = y + offset_y
                        if 0 <= pixel_x < display_width and 0 <= pixel_y < display_height:
                            index = (pixel_y * display_width + pixel_x) * 3
                            pixels[index:index + 3] = bytes(color)

        path_pixels = [
            point for point in (
                map_pixel((x, y, 0.0)) for x, y in (path or [])
            )
            if point is not None
        ]
        draw_path(path_pixels, (236, 72, 153))  # pink planned route
        draw_marker(map_pixel(goal_pose), (22, 163, 74))
        draw_marker(map_pixel(robot_pose), (37, 99, 235))

        display_pixels, display_width, display_height = resize_rgb(
            pixels, display_width, display_height, *MAP_DISPLAY_SIZE,
        )
        ppm = (
            f'P6\n{display_width} {display_height}\n255\n'.encode()
            + display_pixels
        )
        try:
            self.map_photo = tk.PhotoImage(data=ppm, format='PPM')
        except tk.TclError:
            self.map_label.configure(
                text='Nav2 cost map: invalid occupancy grid', image='',
            )
            return
        self.map_label.configure(image=self.map_photo, text='')

    def update_status(
        self,
        speed: float = 0.0,
        destination: str = 'Safehouse',
        distance_to_destination: float = 0.0,
        battery: float = 100.0,
        obstacle_detected: bool = False,
        fire_detected: bool = False,
        robot_name: str = 'Rescue Bot',
    ) -> None:
        self.speed_var.set(f'Speed: {format_speed(speed)}')
        self.distance_var.set(f'Distance to Destination: {distance_to_destination:.1f} m')
        self.obstacle_var.set(f'Obstacle detected: {"yes" if obstacle_detected else "no"}')
        self.fire_var.set(f'Fire detected: {"yes" if fire_detected else "no"}')

    def set_status(self, status: RobotStatus) -> None:
        self.update_status(
            speed=status.speed,
            destination=status.destination,
            distance_to_destination=status.distance_to_destination,
            battery=status.battery,
            obstacle_detected=status.obstacle_detected,
            fire_detected=status.fire_detected,
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
        self.latest_fire_detected = False
        self.latest_distance = 0.0
        self.latest_pose: Optional[Tuple[float, float, float]] = None
        self.latest_path: List[Tuple[float, float]] = []
        self.latest_map: Optional[OccupancyGrid] = None
        # Latest-frame drop: callbacks overwrite; convert/render on timer only.
        self._latest_camera_image: Optional[Image] = None
        self._latest_thermal_image: Optional[Image] = None
        self._camera_dirty = False
        self._thermal_dirty = False
        self._last_camera_render = 0.0
        self._last_thermal_render = 0.0
        self.goal_handle = None
        self.start_request_pending = False
        self.start_wait_timer = None
        self._pending_goal: Optional[Tuple[float, float, float]] = None
        self.active_goal: Optional[Tuple[float, float, float]] = None
        # When path_bank (or another client) preempts our NavigateToPose, keep
        # the mission alive and watch pose until arrival instead of flashing FAIL.
        self._user_stop = False
        self._monitor_arrival = False
        self._monitor_idle_since: Optional[float] = None
        self._monitor_last_pose: Optional[Tuple[float, float]] = None
        self._path_bank_status = ''
        self._path_bank_state = ''
        self._path_bank_active_id: Optional[int] = None
        self._last_no_safe_detail = ''  # H6 from path_bank/status
        # StartMission readiness wait (separate from NavigateToPose pending).
        self._pending_start_mission: Optional[Tuple[float, float, float]] = None
        self._start_mission_wait_since: Optional[float] = None
        self._start_mission_wait_timer = None
        self._start_mission_call_inflight = False
        self._rs1_interfaces_error: Optional[str] = None
        self._local_mission_override = False
        self._owner_state = 'IDLE'
        self._owner_inhibits: List[str] = []
        self._owner_latch = False
        self._owner_authority = 'NONE'
        self._owner_detail = ''

        goal_frame = self.declare_parameter('goal_frame', 'husky1_map').value
        goal_x = self.declare_parameter('goal_x', DEFAULT_GOAL_X).value
        goal_y = self.declare_parameter('goal_y', DEFAULT_GOAL_Y).value
        goal_yaw = self.declare_parameter('goal_yaw', DEFAULT_GOAL_YAW).value
        navigate_action = self.declare_parameter(
            'navigate_action', 'navigate_to_pose'
        ).value
        world_name = self.declare_parameter('world_name', DEFAULT_WORLD_NAME).value
        gaps_config = self.declare_parameter('forest_gaps_config', '').value
        obstacle_seed = self.declare_parameter('obstacle_seed', -1).value
        force_gap = self.declare_parameter('force_gap', '').value
        allow_ntp = self.declare_parameter('allow_ntp_fallback', False).value
        start_wait = self.declare_parameter('start_mission_wait_s', 30.0).value

        self.goal_frame = str(goal_frame)
        self.default_goal_x = float(goal_x)
        self.default_goal_y = float(goal_y)
        self.default_goal_yaw = float(goal_yaw)
        self.world_name = str(world_name).strip() or DEFAULT_WORLD_NAME
        self._allow_ntp_fallback = bool(allow_ntp)
        self._start_mission_wait_s = max(1.0, float(start_wait))
        self.navigate_client = ActionClient(self, NavigateToPose, navigate_action)
        try:
            from rs1_interfaces.srv import StartMission, StopMission
            from rs1_interfaces.msg import MissionOwnerStatus
            self._start_mission_cli = self.create_client(
                StartMission, 'mission_owner/start_mission')
            self._stop_mission_cli = self.create_client(
                StopMission, 'mission_owner/stop_mission')
            self.create_subscription(
                MissionOwnerStatus,
                'mission_owner/status',
                self._owner_status_callback,
                10,
            )
        except Exception as exc:  # noqa: BLE001
            self._start_mission_cli = None
            self._stop_mission_cli = None
            self._rs1_interfaces_error = f'{type(exc).__name__}: {exc}'
            self.get_logger().error(
                'rs1_interfaces unavailable — StartMission disabled '
                f'({self._rs1_interfaces_error}). '
                'source <workspace>/install/setup.bash so install/rs1_interfaces '
                f'is on AMENT_PREFIX_PATH (allow_ntp_fallback={self._allow_ntp_fallback})'
            )
        self.speed_clients = {
            'controller': self.create_client(
                SetParameters, '/husky1/controller_server/set_parameters'
            ),
            'smoother': self.create_client(
                SetParameters, '/husky1/velocity_smoother/set_parameters'
            ),
        }
        # Entry 063 Wave-1: honest MAXIMUM = validated MPPI envelope 0.50/1.2.
        # Do not label Fast as 0.60 while MPPI vx_max binds at 0.50.
        self.speed_limits = dict(SPEED_PRESETS)
        self.speed_angular_limit = float(SPEED_ANGULAR_MAX)
        self.selected_speed = 'fast'
        self.status.distance_to_destination = 0.0
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
        # Relative topic under the robot namespace (launch PushRosNamespace).
        fire_topic = self.declare_parameter('fire_topic', 'fire_detected').value
        camera_topic = self.declare_parameter('camera_topic', 'camera/image').value
        thermal_topic = self.declare_parameter('thermal_topic', 'thermal/image').value
        # Prefer Nav2 global costmap for the GUI map panel (Faiyad / status_gui).
        map_topic = self.declare_parameter(
            'map_topic', 'global_costmap/costmap',
        ).value
        obstacle_threshold = self.declare_parameter('obstacle_threshold', 1.0).value
        self.obstacle_threshold = float(obstacle_threshold)
        # Display throttle ≤ sensor rates (RGB default 3 Hz when enable_camera:=true;
        # thermal ~10 Hz). Launch keeps enable_camera default false.
        cam_hz = float(self.declare_parameter(
            'camera_display_hz', DEFAULT_CAMERA_DISPLAY_HZ,
        ).value)
        therm_hz = float(self.declare_parameter(
            'thermal_display_hz', DEFAULT_THERMAL_DISPLAY_HZ,
        ).value)
        self._camera_min_interval = 1.0 / max(0.1, cam_hz)
        self._thermal_min_interval = 1.0 / max(0.1, therm_hz)

        config_path = str(gaps_config).strip() or None
        gap_config = load_forest_gap_config(
            config_path, world_name=self.world_name,
        )
        seed = int(obstacle_seed)
        self.obstacle_manager = ForestObstacleManager(
            gap_config,
            seed=None if seed < 0 else seed,
            force_gap=str(force_gap).strip() or None,
            logger=lambda m: self.get_logger().info(m),
        )
        self.gui.set_dyn_obstacle_status(self.obstacle_manager.status_message)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._demo_pose_pub = self.create_publisher(
            PoseArray, 'demo_obstacle/poses', latched)
        self._publish_demo_poses()  # empty until spawn
        self.path_bank_cancel_pub = self.create_publisher(
            Bool, 'path_bank/cancel', 10)
        self.create_service(Trigger, 'gui/block_path_a', self._srv_block_path_a)
        self.create_service(Trigger, 'gui/block_path_b', self._srv_block_path_b)
        self.create_service(Trigger, 'gui/clear_fire', self._srv_clear_fire)
        self.create_service(Trigger, 'gui/start_mission', self._srv_start_mission_btn)
        self.create_service(Trigger, 'gui/stop_mission', self._srv_stop_mission_btn)
        self.create_service(Trigger, 'gui/set_speed_slow', self._srv_set_speed_slow)
        self.create_service(Trigger, 'gui/set_speed_medium', self._srv_set_speed_medium)
        self.create_service(Trigger, 'gui/set_speed_fast', self._srv_set_speed_fast)

        self.create_subscription(Odometry, odom_topic, self._odom_callback, 10)
        self.create_subscription(LaserScan, scan_topic, self._scan_callback, qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'base_scan', self._scan_callback, qos_profile_sensor_data)
        self.create_subscription(Bool, fire_topic, self._fire_callback, 10)
        self.create_subscription(Image, camera_topic, self._camera_callback, qos_profile_sensor_data)
        self.create_subscription(Image, thermal_topic, self._thermal_callback, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, map_topic, self._map_callback, 10)
        # Prefer path-bank active path for pink overlay; fall back to Nav2 plan.
        self.create_subscription(
            Path, 'path_bank/active_path', self._active_path_callback, 10)
        self.create_subscription(Path, 'plan', self._plan_callback, 10)
        self.create_subscription(String, 'path_bank/status', self._path_bank_status_callback, 10)
        self.create_subscription(String, 'path_bank/state', self._path_bank_state_callback, 10)
        self.create_timer(0.1, self._update_gui)
        # Camera/thermal render tick at 50 ms — still gated by display_hz throttle.
        self.create_timer(0.05, self._camera_render_tick)
        self.create_timer(0.2, self._obstacle_tick)
        self.create_timer(0.5, self._arrival_monitor_tick)
        self._readiness_announce_every_s = 2.0
        self._readiness_announce_last = 0.0

        self.get_logger().info(
            f'Robot status GUI started for {self.robot_name}. '
            f'Default goal ({self.default_goal_x}, {self.default_goal_y}, '
            f'{self.default_goal_yaw}). Listening to {odom_topic} and '
            f'{scan_topic}/base_scan. World={gap_config.world_name}.'
        )
        self._announce_start_mission_readiness()

    def set_speed(self, speed: str) -> None:
        """Apply GUI speed preset to MPPI vx_max/wz_max + velocity_smoother.

        Entry 063: stop writing obsolete RPP ``desired_linear_vel`` as primary.
        Lockstep: MPPI ``FollowPath.vx_max`` == smoother ``max_velocity[0]``
        == GUI preset; ``wz_max`` / angular max = 1.2; ``min_velocity[0]=-vx``.
        """
        if speed not in self.speed_limits:
            return
        self.selected_speed = speed
        self.gui.set_speed(speed)
        writes = speed_param_writes(speed)
        linear_limit = float(writes['FollowPath.vx_max'])
        angular_limit = float(writes['FollowPath.wz_max'])
        parameter_sets = (
            (
                self.speed_clients['smoother'],
                [
                    Parameter(
                        'max_velocity', Parameter.Type.DOUBLE_ARRAY,
                        list(writes['max_velocity']),
                    ),
                    Parameter(
                        'min_velocity', Parameter.Type.DOUBLE_ARRAY,
                        list(writes['min_velocity']),
                    ),
                ],
            ),
            (
                self.speed_clients['controller'],
                [
                    Parameter(
                        'FollowPath.vx_max', Parameter.Type.DOUBLE,
                        linear_limit,
                    ),
                    Parameter(
                        'FollowPath.wz_max', Parameter.Type.DOUBLE,
                        angular_limit,
                    ),
                ],
            ),
        )
        for client, values in parameter_sets:
            if client.service_is_ready():
                request = SetParameters.Request()
                request.parameters = [value.to_parameter_msg() for value in values]
                client.call_async(request)
        self.gui.set_feedback(
            f'Speed set to {speed} '
            f'(vx_max={linear_limit:.2f} m/s, wz_max={angular_limit:.1f} rad/s)'
        )

    def _srv_set_speed_slow(self, _request, response):
        self.set_speed('slow')
        response.success = True
        response.message = 'set_speed slow'
        return response

    def _srv_set_speed_medium(self, _request, response):
        self.set_speed('medium')
        response.success = True
        response.message = 'set_speed medium'
        return response

    def _srv_set_speed_fast(self, _request, response):
        self.set_speed('fast')
        response.success = True
        response.message = 'set_speed fast'
        return response

    # -- Gazebo goal visual (Faiyad) --------------------------------------

    def _publish_path_bank_cancel(self) -> None:
        cancel = Bool()
        cancel.data = True
        self.path_bank_cancel_pub.publish(cancel)

    def _update_gazebo_goal(self, x: float, y: float, yaw: float) -> None:
        spec = GoalVisualSpec(
            name=GOAL_VISUAL_NAME, x=float(x), y=float(y), yaw=float(yaw),
        )
        threading.Thread(
            target=self._spawn_gazebo_goal, args=(spec,), daemon=True,
        ).start()

    def _remove_gazebo_goal(self) -> None:
        threading.Thread(target=self._delete_gazebo_goal, daemon=True).start()

    def _spawn_gazebo_goal(self, spec: GoalVisualSpec) -> None:
        world = getattr(self.obstacle_manager, 'world', None)
        if world is None or not world.spawn_goal_visual(spec):
            self.get_logger().warning(
                f'Unable to display Gazebo goal visual at '
                f'({spec.x:.2f}, {spec.y:.2f})'
            )

    def _delete_gazebo_goal(self) -> None:
        world = getattr(self.obstacle_manager, 'world', None)
        if world is None or not world.remove_model(GOAL_VISUAL_NAME):
            self.get_logger().warning('Unable to remove Gazebo goal visual')

    # -- shared goal submission -------------------------------------------

    def submit_goal(self, x: float, y: float, yaw: float, *, source: str) -> None:
        """Single NavigateToPose pathway used by START MISSION."""
        if self.goal_handle is not None or self._monitor_arrival:
            self.gui.set_feedback('Goal rejected: mission already running', error=True)
            self.gui.set_mission_status('running')
            self.gui.set_mission_running(True)
            return

        if not self.navigate_client.wait_for_server(timeout_sec=0.1):
            self._pending_goal = (x, y, yaw)
            self.start_request_pending = True
            self.gui.set_mission_status('waiting for Nav2')
            self.gui.set_feedback('Navigation unavailable - waiting for Nav2', error=True)
            if self.start_wait_timer is None:
                self.start_wait_timer = self.create_timer(1.0, self._retry_pending_goal)
            return

        self.start_request_pending = False
        self._pending_goal = None
        self._user_stop = False
        self._monitor_arrival = False
        self._monitor_idle_since = None
        self._monitor_last_pose = None
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
        self._update_gazebo_goal(x, y, yaw)
        self.latest_distance = 0.0
        self.gui.set_goal_fields(x, y, yaw)
        self.gui.set_goal_status(f'({x:g}, {y:g}, {yaw:g})')
        self.gui.set_mission_status('starting')
        self.gui.set_mission_running(True)
        self.gui.set_feedback(
            f'Mission started - navigating to X={x:g} Y={y:g} Yaw={yaw:g}'
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
        if (
            self.gui._mission_running
            or self.goal_handle is not None
            or self.start_request_pending
            or self._monitor_arrival
        ):
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

        # Prefer mission-owner StartMission. Never silently fall through to
        # NavigateToPose under bank_follow_path (BT PAUSE → Mission: REJECTED).
        action = start_mission_action(
            client_available=self._start_mission_cli is not None,
            service_ready=(
                self._start_mission_cli is not None
                and self._start_mission_cli.service_is_ready()
            ),
            allow_ntp_fallback=self._allow_ntp_fallback,
        )
        if action == 'start_mission':
            self._call_start_mission_service(x, y, yaw)
            return
        if action == 'wait_mission_owner':
            self._begin_start_mission_wait(x, y, yaw)
            return
        if action == 'ntp_fallback':
            self.get_logger().warn(
                'StartMission unavailable — NavigateToPose fallback enabled'
            )
            self.gui.set_feedback(
                'Mission owner unavailable — using NavigateToPose', error=True
            )
            self.submit_goal(x, y, yaw, source='start_mission_ntp_fallback')
            return
        self.gui.set_mission_running(False)
        self.gui.set_mission_status('rejected: STARTMISSION_UNAVAILABLE')
        detail = self._rs1_interfaces_error or 'rs1_interfaces / StartMission client missing'
        self.gui.set_feedback(
            f'StartMission unavailable ({detail}). '
            'source <workspace>/install/setup.bash so install/rs1_interfaces is '
            'on AMENT_PREFIX_PATH. Enable allow_ntp_fallback only for legacy '
            'NavigateToPose bringup.',
            error=True,
        )
        self._local_mission_override = True

    def _announce_start_mission_readiness(self) -> None:
        """Boot/periodic UX: never imply READY before StartMission is usable."""
        if self._local_mission_override or self._pending_start_mission is not None:
            return
        if self._start_mission_cli is None:
            self.gui.set_mission_status('rejected: STARTMISSION_UNAVAILABLE')
            detail = self._rs1_interfaces_error or 'rs1_interfaces import failed'
            self.gui.set_feedback(
                f'StartMission UNAVAILABLE ({detail}). '
                'source <workspace>/install/setup.bash (need install/rs1_interfaces).',
                error=True,
            )
            return
        if self._start_mission_cli.service_is_ready():
            return
        # Client exists but owner not discovered yet — waiting, not READY.
        if self.gui.mission_var.get() in ('Mission: IDLE', 'Mission: WAITING FOR MISSION OWNER'):
            self.gui.set_mission_status('waiting for mission owner')
            self.gui.set_feedback(
                'Waiting for /husky1/mission_owner/start_mission — '
                'keep Terminal 1 husky launch alive (path_bank_manager).',
                error=True,
            )

    def _call_start_mission_service(self, x: float, y: float, yaw: float) -> None:
        if self._start_mission_call_inflight:
            self.gui.set_feedback('StartMission already in flight', error=True)
            return
        from rs1_interfaces.srv import StartMission as StartMissionSrv
        req = StartMissionSrv.Request()
        req.goal_x = float(x)
        req.goal_y = float(y)
        req.goal_yaw = float(yaw)
        self._local_mission_override = True
        self._start_mission_call_inflight = True
        self.gui.set_mission_status('starting')
        self.gui.set_mission_running(True)
        future = self._start_mission_cli.call_async(req)
        future.add_done_callback(
            lambda f, gx=x, gy=y, gyaw=yaw: self._start_mission_service_callback(
                f, gx, gy, gyaw))

    def _begin_start_mission_wait(self, x: float, y: float, yaw: float) -> None:
        self._pending_start_mission = (float(x), float(y), float(yaw))
        self._start_mission_wait_since = time.monotonic()
        self._local_mission_override = True
        self.gui.set_mission_status('waiting for mission owner')
        self.gui.set_mission_running(False)
        self.gui.set_feedback(
            f'Waiting for mission_owner/start_mission '
            f'(timeout {self._start_mission_wait_s:.0f}s wall). '
            'Terminal 1 husky launch / path_bank_manager must be alive.',
            error=True,
        )
        # rclpy create_timer is sim-time gated when use_sim_time:=true; orphan
        # Gazebo with a dead /clock would hang WAITING forever. Primary poll is
        # poll_wall_clock_tasks() from the Tk/spin main loop (wall time).
        if self._start_mission_wait_timer is None:
            self._start_mission_wait_timer = self.create_timer(
                0.5, self._retry_pending_start_mission
            )

    def poll_wall_clock_tasks(self) -> None:
        """Wall-clock work for the main loop — runs even when /clock is dead."""
        now = time.monotonic()
        if self._pending_start_mission is not None:
            self._retry_pending_start_mission()
        if now - self._readiness_announce_last >= self._readiness_announce_every_s:
            self._readiness_announce_last = now
            self._announce_start_mission_readiness()

    def _clear_start_mission_wait(self) -> None:
        self._pending_start_mission = None
        self._start_mission_wait_since = None
        if self._start_mission_wait_timer is not None:
            self._start_mission_wait_timer.cancel()
            self._start_mission_wait_timer = None

    def _retry_pending_start_mission(self) -> None:
        """Poll StartMission readiness — must NOT call submit_goal/NTP."""
        if self._pending_start_mission is None or self._start_mission_wait_since is None:
            return
        if self._start_mission_call_inflight:
            return
        x, y, yaw = self._pending_start_mission
        if (
            self._start_mission_cli is not None
            and self._start_mission_cli.service_is_ready()
        ):
            self._clear_start_mission_wait()
            self._call_start_mission_service(x, y, yaw)
            return
        elapsed = time.monotonic() - self._start_mission_wait_since
        if elapsed < self._start_mission_wait_s:
            return
        self._clear_start_mission_wait()
        if self._allow_ntp_fallback:
            self.get_logger().warn(
                'StartMission wait timed out — NavigateToPose fallback'
            )
            self.gui.set_feedback(
                'Mission owner timeout — using NavigateToPose', error=True
            )
            self.submit_goal(x, y, yaw, source='start_mission_timeout_fallback')
            return
        self._local_mission_override = True
        self.gui.set_mission_running(False)
        self.gui.set_mission_status('rejected: TIMEOUT_WAITING_MISSION_OWNER')
        self.gui.set_feedback(
            'StartMission timed out waiting for mission owner. '
            'Keep Terminal 1 husky launch alive (path_bank_manager publishes '
            '/husky1/mission_owner/start_mission).',
            error=True,
        )

    def _start_mission_service_callback(self, future, x, y, yaw) -> None:
        self._start_mission_call_inflight = False
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001
            self.gui.set_mission_running(False)
            self._local_mission_override = True
            self.gui.set_mission_status('rejected: STARTMISSION_EXCEPTION')
            self.gui.set_feedback(f'StartMission failed: {exc}', error=True)
            return
        if result is None or not result.success:
            self.gui.set_mission_running(False)
            code = getattr(result, 'reject_code', '') if result else ''
            msg = getattr(result, 'message', 'rejected') if result else 'rejected'
            banner = f'rejected: {code}' if code else 'rejected'
            self._local_mission_override = True
            self.gui.set_mission_status(banner)
            inhibit_hint = ''
            if self._owner_inhibits:
                inhibit_hint = f'; inhibits={",".join(self._owner_inhibits)}'
            self.gui.set_feedback(
                f'Start rejected ({code}): {msg} '
                f'[phase={self._owner_state}{inhibit_hint}]. '
                f'Clear safety inhibit / wait for IDLE or STOPPED, then retry.',
                error=True,
            )
            return
        self.active_goal = (float(x), float(y), float(yaw))
        self._update_gazebo_goal(x, y, yaw)
        self.gui.set_goal_fields(x, y, yaw)
        self.gui.set_goal_status(f'({x:g}, {y:g}, {yaw:g})')
        self._local_mission_override = False
        self.gui.set_mission_status('running')
        self.gui.set_feedback(
            f'Mission started (id={result.mission_id}) → X={x:g} Y={y:g}'
        )

    def _retry_pending_goal(self) -> None:
        if not self.start_request_pending or self._pending_goal is None:
            return
        if self.navigate_client.server_is_ready():
            x, y, yaw = self._pending_goal
            self.submit_goal(x, y, yaw, source='retry')

    def stop_mission(self) -> None:
        self._user_stop = True
        self._monitor_arrival = False
        self._monitor_idle_since = None
        self.start_request_pending = False
        self._pending_goal = None
        self._clear_start_mission_wait()
        if self.start_wait_timer is not None:
            self.start_wait_timer.cancel()
            self.start_wait_timer = None
        self._publish_path_bank_cancel()
        self._remove_gazebo_goal()

        # Prefer mission-owner StopMission (closes cmd_vel gate + latch).
        if (
            self._stop_mission_cli is not None
            and self._stop_mission_cli.service_is_ready()
        ):
            from rs1_interfaces.srv import StopMission as StopMissionSrv
            self._local_mission_override = True
            self.gui.set_mission_status('stopping')
            future = self._stop_mission_cli.call_async(StopMissionSrv.Request())
            future.add_done_callback(self._stop_mission_service_callback)
            # Still cancel local NTP handle if we own one.
            if self.goal_handle is not None:
                try:
                    self.goal_handle.cancel_goal_async()
                except Exception:  # noqa: BLE001
                    pass
            return

        if self.goal_handle is None:
            self.active_goal = None
            self.gui.set_mission_running(False)
            self._local_mission_override = False
            self._apply_owner_mission_banner()
            self.gui.set_feedback('Mission stopped')
            return

        self._local_mission_override = True
        self.gui.set_mission_status('stopping')
        future = self.goal_handle.cancel_goal_async()
        future.add_done_callback(self._cancel_response_callback)

    def _stop_mission_service_callback(self, future) -> None:
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001
            self.gui.set_feedback(f'StopMission failed: {exc}', error=True)
            self.gui.set_mission_status('stop failed')
            return
        self.goal_handle = None
        self.active_goal = None
        self._remove_gazebo_goal()
        self.gui.set_mission_running(False)
        self._local_mission_override = False
        if result is not None and getattr(result, 'success', False):
            inhibits = list(getattr(result, 'active_inhibits', []) or [])
            if inhibits:
                self.gui.set_mission_status(f'Stopped — blocked: {inhibits[0]}')
                self._local_mission_override = True
            else:
                self._apply_owner_mission_banner()
            self.gui.set_feedback('Mission stopped (gate NONE)')
        else:
            self._apply_owner_mission_banner()
            self.gui.set_feedback(
                getattr(result, 'message', 'StopMission completed')
                if result is not None else 'StopMission completed'
            )

    def _owner_status_callback(self, msg) -> None:
        """Drive Mission banner from mission_owner/status when not locally overriding."""
        self._owner_state = str(getattr(msg, 'state', 'IDLE') or 'IDLE')
        self._owner_latch = bool(getattr(msg, 'operator_stop_latch', False))
        self._owner_inhibits = [
            str(i) for i in (getattr(msg, 'active_inhibits', []) or [])
        ]
        self._owner_authority = str(
            getattr(msg, 'command_authority', 'NONE') or 'NONE'
        )
        self._owner_detail = str(getattr(msg, 'detail', '') or '')
        running = bool(getattr(msg, 'mission_running', False))
        if running and not self.gui._mission_running:
            # Owner accepted a mission (possibly outside this GUI).
            self.gui.set_mission_running(True)
        elif (
            not running
            and self.gui._mission_running
            and not self._local_mission_override
            and self.goal_handle is None
            and not self._monitor_arrival
            and self._pending_start_mission is None
        ):
            self.gui.set_mission_running(False)
        if self._local_mission_override or self._pending_start_mission is not None:
            return
        # Owner terminal / inhibit states always win over stale bank FOLLOWING_PATH.
        owner_phase = self._owner_state.strip().upper()
        if owner_phase in (
            'STOPPED', 'STOPPING', 'HOLDING', 'COSTMAP_REBUILD',
        ) or self._owner_inhibits:
            self._apply_owner_mission_banner(msg)
            return
        # Path-bank navigating / blocked labels take precedence while bank is active.
        if self._path_bank_state in (
            _PATH_BANK_NAVIGATING
            | _PATH_BANK_RETURNING
            | _PATH_BANK_ALL_BLOCKED_PRELUDE
            | _PATH_BANK_RETURN_TERMINAL
            | _NO_SAFE_ROUTE_STATES
        ):
            return
        self._apply_owner_mission_banner(msg)

    def _apply_owner_mission_banner(self, msg=None) -> None:
        if msg is None:
            banner = mission_banner_from_owner(
                self._owner_state,
                operator_stop_latch=self._owner_latch,
                active_inhibits=self._owner_inhibits,
                command_authority=self._owner_authority,
                mission_running=self.gui._mission_running,
                detail=self._owner_detail,
                no_safe_detail=self._last_no_safe_detail,
            )
        else:
            banner = mission_banner_from_owner(
                getattr(msg, 'state', 'IDLE'),
                operator_stop_latch=bool(
                    getattr(msg, 'operator_stop_latch', False)
                ),
                active_inhibits=getattr(msg, 'active_inhibits', []) or [],
                command_authority=str(
                    getattr(msg, 'command_authority', 'NONE') or 'NONE'
                ),
                stop_reason=str(getattr(msg, 'stop_reason', '') or ''),
                mission_running=bool(getattr(msg, 'mission_running', False)),
                detail=str(getattr(msg, 'detail', '') or ''),
                no_safe_detail=self._last_no_safe_detail,
            )
        self.gui.set_mission_status(banner)

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
        self._publish_demo_poses()

    def block_path_b(self) -> None:
        self._request_path(PATH_B, 'BLOCK PATH B')
        self._publish_demo_poses()

    def block_both_paths(self) -> None:
        ok, message = self.obstacle_manager.request_both()
        self.gui.set_dyn_obstacle_status(message)
        self.gui.set_feedback(message, error=not ok)
        self._publish_demo_poses()
        if ok:
            self.get_logger().info(f'BLOCK BOTH PATHS: {message}')

    def random_block(self) -> None:
        self._request_path(None, 'RANDOM BLOCK')
        self._publish_demo_poses()

    def clear_obstacles(self) -> None:
        ok, message = self.obstacle_manager.clear_obstacles()
        self.gui.set_dyn_obstacle_status(message)
        self.gui.set_feedback('Obstacles cleared' if ok else message, error=not ok)
        self.obstacle_manager.status_message = 'Obstacle: READY'
        self._publish_demo_poses()

    def _publish_demo_poses(self) -> None:
        """Latched PoseArray of active/armed GUI demo obstacles for fire bridge."""
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'husky1_map'
        gaps = []
        # Dual-block: publish every pending + active gap (G5 Path A+B).
        for gap in list(getattr(self.obstacle_manager, 'pending_gaps', []) or []):
            gaps.append(gap)
        for gap in list(getattr(self.obstacle_manager, 'active_gaps', []) or []):
            if gap not in gaps:
                gaps.append(gap)
        if not gaps:
            gap = self.obstacle_manager.selected_gap
            if (
                gap is not None
                and self.obstacle_manager.state in (
                    ObstacleState.WAITING, ObstacleState.ACTIVE, ObstacleState.REQUESTED,
                )
            ):
                gaps = [gap]
        for gap in gaps:
            p = Pose()
            p.position.x = float(gap.x)
            p.position.y = float(gap.y)
            msg.poses.append(p)
        self._demo_pose_pub.publish(msg)

    def _srv_block_path_a(self, _req, response):
        self.block_path_a()
        self._publish_demo_poses()
        response.success = self.obstacle_manager.state != ObstacleState.IDLE
        response.message = self.obstacle_manager.status_message
        return response

    def _srv_block_path_b(self, _req, response):
        self.block_path_b()
        self._publish_demo_poses()
        response.success = self.obstacle_manager.state != ObstacleState.IDLE
        response.message = self.obstacle_manager.status_message
        return response

    def _srv_clear_fire(self, _req, response):
        self.clear_obstacles()
        response.success = True
        response.message = self.obstacle_manager.status_message
        return response

    def _srv_start_mission_btn(self, _req, response):
        if self.gui._mission_running:
            response.success = False
            response.message = 'mission already running'
            return response
        self.start_mission_from_fields()
        response.success = True
        response.message = 'START MISSION invoked'
        return response

    def _srv_stop_mission_btn(self, _req, response):
        self.stop_mission()
        response.success = True
        response.message = 'STOP MISSION invoked'
        return response

    def _obstacle_tick(self) -> None:
        pose = self.latest_pose
        robot_xy = (pose[0], pose[1]) if pose is not None else None
        prev = self.obstacle_manager.state
        status = self.obstacle_manager.tick(robot_xy)
        if status:
            self.gui.set_dyn_obstacle_status(status)
            if self.obstacle_manager.state == ObstacleState.ACTIVE:
                self.gui.set_feedback('Dynamic obstacle spawned')
                if prev != ObstacleState.ACTIVE:
                    self._publish_demo_poses()
        elif self.obstacle_manager.state == ObstacleState.WAITING:
            self.gui.set_dyn_obstacle_status(self.obstacle_manager.status_message)

    # -- Nav2 callbacks ---------------------------------------------------

    def _feedback_callback(self, feedback_message) -> None:
        feedback = feedback_message.feedback
        self.latest_distance = float(feedback.distance_remaining)

    def _goal_response_callback(self, future) -> None:
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.goal_handle = None
            self.active_goal = None
            self._local_mission_override = True
            self.gui.set_mission_status('rejected: NAV2_GOAL_REJECTED')
            self.gui.set_feedback(
                'NavigateToPose rejected (BT may be paused under '
                'bank_follow_path — use mission_owner/start_mission)',
                error=True,
            )
            self.gui.set_mission_running(False)
            return

        self._local_mission_override = False
        self.gui.set_mission_status('running')
        self.gui.set_mission_running(True)
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _cancel_response_callback(self, future) -> None:
        if future.result().goals_canceling:
            self.goal_handle = None
            self.gui.set_mission_running(False)
            self._local_mission_override = False
            self._apply_owner_mission_banner()
            self.gui.set_feedback('Mission stopped')
        else:
            self.gui.set_mission_status('cancel failed')
            self.gui.set_feedback('Cancel failed', error=True)

    def _result_callback(self, future) -> None:
        status = future.result().status
        self.goal_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._monitor_arrival = False
            self.active_goal = None
            self.gui.set_mission_running(False)
            self.gui.set_mission_status('complete')
            self.gui.set_feedback('Goal reached')
            return
        if status == GoalStatus.STATUS_CANCELED and self._user_stop:
            self._monitor_arrival = False
            self.active_goal = None
            self.gui.set_mission_running(False)
            self.gui.set_mission_status('ready')
            self.gui.set_feedback('Mission stopped')
            return
        # Preempted by path_bank_manager or another NavigateToPose client —
        # keep the mission panel live; bank state drives Mission Status text.
        if self.active_goal is not None and not self._user_stop:
            returning = self._path_bank_state in _PATH_BANK_RETURN_TERMINAL
            self._monitor_arrival = not returning
            self._monitor_idle_since = None
            self._monitor_last_pose = None
            self.gui.set_mission_running(True)
            mapped = mission_status_from_path_bank(
                self._path_bank_state,
                self._path_bank_active_id,
                no_safe_detail=self._last_no_safe_detail,
            )
            if mapped is not None:
                self.gui.set_mission_status(mapped)
            elif not returning:
                self.gui.set_mission_status('running')
            detail = 'path-bank/Nav2 preempt'
            if self._path_bank_status:
                first = self._path_bank_status.splitlines()[0][:60]
                detail = first
            self.gui.set_feedback(f'Mission continuing ({detail})')
            self.get_logger().info(
                f'NavigateToPose ended status={status}; monitoring arrival'
                f'={self._monitor_arrival}'
            )
            return
        self._monitor_arrival = False
        self.active_goal = None
        self.gui.set_mission_running(False)
        self.gui.set_mission_status('failed')
        self.gui.set_feedback('Goal failed', error=True)

    def _apply_path_bank_mission_status(self) -> None:
        mapped = mission_status_from_path_bank(
            self._path_bank_state,
            self._path_bank_active_id,
            no_safe_detail=self._last_no_safe_detail,
        )
        if mapped is None:
            return
        self.gui.set_mission_status(mapped)
        if self._path_bank_state == 'RETURNING_TO_START':
            self._monitor_arrival = False
            self.gui.set_mission_running(True)
            self.gui.set_feedback('All paths blocked — returning to start')
        elif self._path_bank_state == 'RETURNED_TO_START':
            self._monitor_arrival = False
            self.active_goal = None
            self.gui.set_mission_running(False)
            self.gui.set_feedback('Returned to start')
        elif self._path_bank_state == 'NO_SAFE_ROUTE':
            # Owner STOPPED path usually wins; refresh with detail if bank leads.
            self._monitor_arrival = False
            self.active_goal = None
            self.gui.set_mission_running(False)
            if self._owner_inhibits or self._owner_state.strip().upper() == 'STOPPED':
                self._apply_owner_mission_banner()
        elif self._path_bank_state in _PATH_BANK_NAVIGATING and self.active_goal is not None:
            self.gui.set_mission_running(True)

    def _path_bank_status_callback(self, msg: String) -> None:
        self._path_bank_status = msg.data or ''
        self._path_bank_active_id = parse_path_bank_active_id(self._path_bank_status)
        self._last_no_safe_detail = parse_last_no_safe_detail(self._path_bank_status)
        # Refresh Path-N label while navigating if only status (not state) changed.
        if self._path_bank_state in _PATH_BANK_NAVIGATING:
            self._apply_path_bank_mission_status()
        elif self._path_bank_state in _NO_SAFE_ROUTE_STATES:
            self._apply_path_bank_mission_status()
        elif (
            self._owner_state.strip().upper() == 'STOPPED'
            and 'ALL_BLOCKED_NO_HOME' in [
                i.strip().upper() for i in self._owner_inhibits
            ]
        ):
            self._apply_owner_mission_banner()

    def _path_bank_state_callback(self, msg: String) -> None:
        self._path_bank_state = (msg.data or '').strip().upper()
        self._apply_path_bank_mission_status()

    def _arrival_monitor_tick(self) -> None:
        """Complete/fail missions after another client took NavigateToPose."""
        if self._path_bank_state in _PATH_BANK_RETURN_TERMINAL:
            self._monitor_arrival = False
            return
        if not self._monitor_arrival or self.active_goal is None:
            return
        pose = self.latest_pose
        if pose is None:
            return
        gx, gy, _gyaw = self.active_goal
        dist = math.hypot(pose[0] - gx, pose[1] - gy)
        self.latest_distance = dist
        if dist <= ARRIVAL_DISTANCE_THRESHOLD_M:
            self._monitor_arrival = False
            self.active_goal = None
            self._publish_path_bank_cancel()
            self._remove_gazebo_goal()
            self.gui.set_mission_running(False)
            self.gui.set_mission_status('successful')
            self.gui.set_feedback(
                f'Goal reached (within {ARRIVAL_DISTANCE_THRESHOLD_M:g} m)'
            )
            return
        now = time.monotonic()
        if self._monitor_last_pose is None:
            self._monitor_last_pose = (pose[0], pose[1])
            self._monitor_idle_since = now
            return
        moved = math.hypot(
            pose[0] - self._monitor_last_pose[0],
            pose[1] - self._monitor_last_pose[1],
        )
        if moved > 0.15:
            self._monitor_last_pose = (pose[0], pose[1])
            self._monitor_idle_since = now
            return
        if self._monitor_idle_since is not None and (now - self._monitor_idle_since) > 90.0:
            self._monitor_arrival = False
            self.gui.set_mission_running(False)
            self.gui.set_mission_status('failed')
            self.gui.set_feedback(
                f'Goal stalled {dist:.1f} m from target', error=True
            )

    # -- sensors ----------------------------------------------------------

    def _odom_callback(self, msg: Odometry) -> None:
        self.latest_speed = abs(float(msg.twist.twist.linear.x))
        self.status.speed = self.latest_speed
        p = msg.pose.pose
        yaw = yaw_from_quaternion(
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w,
        )
        # Gazebo OdometryPublisher reports world-frame pose - matches gap coords.
        self.latest_pose = (float(p.position.x), float(p.position.y), yaw)

    def _scan_callback(self, msg: LaserScan) -> None:
        if not hasattr(msg, 'ranges'):
            return
        self.latest_obstacle_detected = detect_obstacle(msg.ranges, threshold=self.obstacle_threshold)
        self.status.obstacle_detected = self.latest_obstacle_detected

    def _camera_callback(self, msg: Image) -> None:
        # Latest-frame drop: overwrite only; never convert on the spin path.
        self._latest_camera_image = msg
        self._camera_dirty = True

    def _thermal_callback(self, msg: Image) -> None:
        self._latest_thermal_image = msg
        self._thermal_dirty = True

    def _camera_render_tick(self) -> None:
        """Convert/render pending camera frames off the subscription hot path.

        Profile-first: drop stale frames; display rate ≤ configured sensor caps.
        Gazebo load can still starve the GUI — do not treat this as FPS acceptance.
        """
        now = time.monotonic()
        if (
            self._camera_dirty
            and self._latest_camera_image is not None
            and (now - self._last_camera_render) >= self._camera_min_interval
        ):
            self._camera_dirty = False
            self._last_camera_render = now
            self.gui.update_camera(self._latest_camera_image)
        if (
            self._thermal_dirty
            and self._latest_thermal_image is not None
            and (now - self._last_thermal_render) >= self._thermal_min_interval
        ):
            self._thermal_dirty = False
            self._last_thermal_render = now
            self.gui.update_thermal(self._latest_thermal_image)

    def _fire_callback(self, msg: Bool) -> None:
        # Live FOV thermal flag only (debounced upstream in fire_detector).
        self.latest_fire_detected = bool(msg.data)

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg
        self.gui.update_map(
            msg,
            robot_pose=self.latest_pose,
            goal_pose=self.active_goal,
            path=self.latest_path,
        )

    def _active_path_callback(self, msg: Path) -> None:
        self.latest_path = [
            (float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in msg.poses
        ]
        if self.latest_map is not None:
            self.gui.update_map(
                self.latest_map,
                robot_pose=self.latest_pose,
                goal_pose=self.active_goal,
                path=self.latest_path,
            )

    def _plan_callback(self, msg: Path) -> None:
        # Only use Nav2 /plan when path-bank has not published an active path.
        if self.latest_path:
            return
        self.latest_path = [
            (float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in msg.poses
        ]
        if self.latest_map is not None:
            self.gui.update_map(
                self.latest_map,
                robot_pose=self.latest_pose,
                goal_pose=self.active_goal,
                path=self.latest_path,
            )

    def _update_gui(self) -> None:
        self.status.speed = self.latest_speed
        self.status.obstacle_detected = self.latest_obstacle_detected
        self.status.fire_detected = self.latest_fire_detected
        if self.active_goal is not None:
            x, y, yaw = self.active_goal
            self.status.destination = f'({x:g}, {y:g}, {yaw:g})'
        else:
            self.status.destination = 'Safehouse'
        self.status.distance_to_destination = distance_for_display(
            self.latest_distance,
            self.latest_pose,
            self.active_goal,
        )
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
    gui.set_speed_callback(node.set_speed)
    # Entry 063: boot to highest validated (Fast/MAXIMUM=0.50), not Slow 0.15.
    node.set_speed('fast')
    gui.set_obstacle_callbacks(
        node.block_path_a,
        node.block_path_b,
        node.random_block,
        node.block_both_paths,
        node.clear_obstacles,
    )

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            node.poll_wall_clock_tasks()
            gui.update_idletasks()
            gui.update()
    except KeyboardInterrupt:
        pass
    finally:
        node._clear_start_mission_wait()
        gui.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
