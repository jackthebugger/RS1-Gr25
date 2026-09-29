#!/usr/bin/env python3
"""Small modular GUI for displaying robot status during autonomy demos."""

import tkinter as tk
from dataclasses import dataclass
from math import cos, sin
from typing import Iterable, List, Optional

import rclpy
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


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
    fire_detected: bool = False

    def to_display_lines(self) -> List[str]:
        return [
            f'Robot Name: {self.robot_name}',
            f'Speed: {format_speed(self.speed)}',
            f'Distance to Destination: {self.distance_to_destination:.1f} m',
            f'Time to Destination: {self.time_to_destination}',
            f'Obstacle detected: {"yes" if self.obstacle_detected else "no"}',
            f'Fire detected: {"yes" if self.fire_detected else "no"}',
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


class RobotStatusWindow(tk.Tk):
    """A clean GUI window that displays a robot status summary."""

    def __init__(self, robot_name: str):
        super().__init__()
        self.robot_name = robot_name
        self.title(f'{robot_name} status')
        self.geometry('560x980')
        self.minsize(480, 900)

        self.configure(bg='#f3f4f6')

        self.header = tk.Label(
            self,
            text=f'Robot Status',
            font=('Arial', 16, 'bold'),
            bg='#f3f4f6',
            fg='#1f2937',
            anchor='w',
            padx=20,
            pady=12,
        )
        self.header.pack(fill='x')

        self.camera_frame = tk.Frame(
            self,
            width=320,
            height=240,
            bg='#111827',
        )
        self.camera_frame.pack(padx=20, pady=(0, 12))
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

        self.map_frame = tk.Frame(
            self,
            width=320,
            height=240,
            bg='#111827',
        )
        self.map_frame.pack(padx=20, pady=(0, 12))
        self.map_frame.pack_propagate(False)
        self.map_label = tk.Label(
            self.map_frame,
            text='Map: waiting for SLAM',
            bg='#111827',
            fg='#ffffff',
            anchor='center',
        )
        self.map_label.pack(fill='both', expand=True)
        self.map_photo = None

        self.info_frame = tk.Frame(self, bg='#ffffff', bd=1, relief='solid')
        self.info_frame.pack(fill='both', expand=True, padx=20, pady=(0, 20))

        self.robot_name_var = tk.StringVar(value='Robot Name: Rescue Bot')
        self.speed_var = tk.StringVar(value='Speed: 0.00 m/s')
        self.distance_var = tk.StringVar(value='Distance to Destination: 0.0 m')
        self.time_var = tk.StringVar(value='Time to Destination: Pending')
        self.obstacle_var = tk.StringVar(value='Obstacle detected: no')
        self.fire_var = tk.StringVar(value='Fire detected: no')
        self.mission_var = tk.StringVar(value='Mission: ready')

        self.labels = [
            tk.Label(self.info_frame, textvariable=self.robot_name_var, font=('Arial', 12), bg='#ffffff', fg='#111827', anchor='w', padx=16, pady=6),
            tk.Label(self.info_frame, textvariable=self.speed_var, font=('Arial', 12), bg='#ffffff', fg='#111827', anchor='w', padx=16, pady=6),
            tk.Label(self.info_frame, textvariable=self.distance_var, font=('Arial', 12), bg='#ffffff', fg='#111827', anchor='w', padx=16, pady=6),
            tk.Label(self.info_frame, textvariable=self.time_var, font=('Arial', 12), bg='#ffffff', fg='#111827', anchor='w', padx=16, pady=6),
            tk.Label(self.info_frame, textvariable=self.obstacle_var, font=('Arial', 12), bg='#ffffff', fg='#111827', anchor='w', padx=16, pady=6),
            tk.Label(self.info_frame, textvariable=self.fire_var, font=('Arial', 12), bg='#ffffff', fg='#111827', anchor='w', padx=16, pady=6),
            tk.Label(self.info_frame, textvariable=self.mission_var, font=('Arial', 12, 'bold'), bg='#ffffff', fg='#111827', anchor='w', padx=16, pady=6),
        ]

        for label in self.labels:
            label.pack(fill='x')

        self.controls = tk.Frame(self, bg='#f3f4f6')
        self.controls.pack(fill='x', padx=20, pady=(0, 12))
        self.start_button = tk.Button(
            self.controls,
            text='Start mission',
            state='disabled',
            command=lambda: None,
            bg='#16a34a',
            activebackground='#15803d',
            fg='#ffffff',
            activeforeground='#ffffff',
            disabledforeground='#d1d5db',
        )
        self.start_button.pack(side='left', expand=True, fill='x', padx=(0, 6))
        self.stop_button = tk.Button(
            self.controls,
            text='Stop mission',
            state='disabled',
            command=lambda: None,
            bg='#dc2626',
            activebackground='#b91c1c',
            fg='#ffffff',
            activeforeground='#ffffff',
            disabledforeground='#d1d5db',
        )
        self.stop_button.pack(side='left', expand=True, fill='x', padx=(6, 0))

        self.speed_frame = tk.LabelFrame(
            self,
            text='Driving speed',
            bg='#f3f4f6',
            fg='#1f2937',
            padx=8,
            pady=4,
        )
        self.speed_frame.pack(fill='x', padx=20, pady=(0, 12))
        self.speed_choice_var = tk.StringVar(value='slow')
        self.speed_buttons = {}
        for value, label in (('slow', 'Slow'), ('medium', 'Medium'), ('fast', 'Fast')):
            button = tk.Radiobutton(
                self.speed_frame,
                text=label,
                variable=self.speed_choice_var,
                value=value,
                state='disabled',
                bg='#f3f4f6',
                fg='#111827',
                anchor='w',
            )
            button.pack(side='left', expand=True, fill='x')
            self.speed_buttons[value] = button

        self.update_status()

    def set_mission_callbacks(self, start_callback, stop_callback) -> None:
        self.start_button.configure(command=start_callback, state='normal')
        self.stop_button.configure(command=stop_callback)

    def set_mission_running(self, running: bool) -> None:
        self.start_button.configure(state='disabled' if running else 'normal')
        self.stop_button.configure(state='normal' if running else 'disabled')

    def set_mission_status(self, status: str) -> None:
        self.mission_var.set(f'Mission: {status}')

    def set_speed_callback(self, callback) -> None:
        for value, button in self.speed_buttons.items():
            button.configure(command=lambda selected=value: callback(selected), state='normal')

    def set_speed(self, value: str) -> None:
        self.speed_choice_var.set(value)

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

    def update_map(self, grid: OccupancyGrid) -> None:
        width = int(grid.info.width)
        height = int(grid.info.height)
        if width <= 0 or height <= 0 or len(grid.data) < width * height:
            return

        display_width = min(320, width)
        display_height = min(240, height)
        pixels = bytearray()
        for display_y in range(display_height):
            source_y = height - 1 - (display_y * height // display_height)
            for display_x in range(display_width):
                source_x = display_x * width // display_width
                occupancy = grid.data[source_y * width + source_x]
                if occupancy < 0:
                    value = 150
                elif occupancy >= 50:
                    value = 0
                else:
                    value = 255
                pixels.extend((value, value, value))

        ppm = (
            f'P6\n{display_width} {display_height}\n255\n'.encode() + pixels
        )
        try:
            self.map_photo = tk.PhotoImage(data=ppm, format='PPM')
        except tk.TclError:
            self.map_label.configure(text='Map: invalid occupancy grid', image='')
            return
        self.map_label.configure(image=self.map_photo, text='')

    def update_status(
        self,
        speed: float = 0.0,
        destination: str = 'Safehouse',
        distance_to_destination: float = 0.0,
        time_to_destination: str = 'Pending',
        battery: float = 100.0,
        obstacle_detected: bool = False,
        fire_detected: bool = False,
        robot_name: str = 'Rescue Bot',
    ) -> None:
        self.robot_name_var.set(f'Robot Name: {robot_name}')
        self.speed_var.set(f'Speed: {format_speed(speed)}')
        self.distance_var.set(f'Distance to Destination: {distance_to_destination:.1f} m')
        self.time_var.set(f'Time to Destination: {time_to_destination}')
        self.obstacle_var.set(f'Obstacle detected: {"yes" if obstacle_detected else "no"}')
        self.fire_var.set(f'Fire detected: {"yes" if fire_detected else "no"}')

    def set_status(self, status: RobotStatus) -> None:
        self.update_status(
            speed=status.speed,
            destination=status.destination,
            distance_to_destination=status.distance_to_destination,
            time_to_destination=status.time_to_destination,
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
        self.latest_time = 'Pending'
        self.goal_handle = None
        self.goal_message = None
        self.start_request_pending = False
        self.start_wait_timer = None

        goal_frame = self.declare_parameter('goal_frame', 'husky1_map').value
        goal_x = self.declare_parameter('goal_x', -4.5).value
        goal_y = self.declare_parameter('goal_y', -4.5).value
        goal_yaw = self.declare_parameter('goal_yaw', 0.0).value
        navigate_action = self.declare_parameter(
            'navigate_action', 'navigate_to_pose'
        ).value
        self.goal_frame = str(goal_frame)
        self.goal_x = float(goal_x)
        self.goal_y = float(goal_y)
        self.goal_yaw = float(goal_yaw)
        self.navigate_client = ActionClient(self, NavigateToPose, navigate_action)
        self.speed_clients = {
            'controller': self.create_client(
                SetParameters, '/husky1/controller_server/set_parameters'
            ),
            'smoother': self.create_client(
                SetParameters, '/husky1/velocity_smoother/set_parameters'
            ),
        }
        self.speed_limits = {'slow': 0.15, 'medium': 0.30, 'fast': 0.60}
        self.selected_speed = 'slow'
        self.status.distance_to_destination = 0.0
        self.status.time_to_destination = 'Pending'
        self.status.battery = 100.0

        odom_topic = self.declare_parameter('odom_topic', 'odom').value
        scan_topic = self.declare_parameter('scan_topic', 'scan').value
        fire_topic = self.declare_parameter('fire_topic', '/fire_detected').value
        camera_topic = self.declare_parameter('camera_topic', 'camera/image').value
        map_topic = self.declare_parameter('map_topic', 'map').value
        obstacle_threshold = self.declare_parameter('obstacle_threshold', 1.0).value
        self.obstacle_threshold = float(obstacle_threshold)

        self.create_subscription(Odometry, odom_topic, self._odom_callback, 10)
        self.create_subscription(LaserScan, scan_topic, self._scan_callback, qos_profile_sensor_data)

        self.create_subscription(LaserScan, 'base_scan', self._scan_callback, qos_profile_sensor_data)
        self.create_subscription(Bool, fire_topic, self._fire_callback, 10)
        self.create_subscription(Image, camera_topic, self._camera_callback, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, map_topic, self._map_callback, 10)
        self.create_timer(0.1, self._update_gui)

        self.get_logger().info(
            f'Robot status GUI started for {self.robot_name}. '
            f'Listening to {odom_topic} and {scan_topic}/base_scan.'
        )

    def set_speed(self, speed: str) -> None:
        if speed not in self.speed_limits:
            return
        self.selected_speed = speed
        self.gui.set_speed(speed)
        linear_limit = self.speed_limits[speed]
        parameters = [
            Parameter('max_velocity', Parameter.Type.DOUBLE_ARRAY,
                      [linear_limit, 0.0, 1.2]),
        ]
        controller_parameter = Parameter(
            'FollowPath.desired_linear_vel',
            Parameter.Type.DOUBLE,
            linear_limit,
        )
        for client, values in (
            (self.speed_clients['smoother'], parameters),
            (self.speed_clients['controller'], [controller_parameter]),
        ):
            if client.service_is_ready():
                request = SetParameters.Request()
                request.parameters = [value.to_parameter_msg() for value in values]
                client.call_async(request)
        self.gui.set_mission_status(f'{self.gui.mission_var.get().removeprefix("Mission: ")} ({speed})')

    def start_mission(self) -> None:
        if self.goal_handle is not None:
            self.gui.set_mission_status('running')
            self.gui.set_mission_running(True)
            return

        if not self.navigate_client.wait_for_server(timeout_sec=0.1):
            self.start_request_pending = True
            self.gui.set_mission_status('waiting for Nav2')
            if self.start_wait_timer is None:
                self.start_wait_timer = self.create_timer(1.0, self._retry_start_mission)
            return

        self.start_request_pending = False
        if self.start_wait_timer is not None:
            self.start_wait_timer.cancel()
            self.start_wait_timer = None
        self.set_speed(self.selected_speed)

        if self.goal_message is None:
            self.goal_message = NavigateToPose.Goal()
            self.goal_message.pose = PoseStamped()
            self.goal_message.pose.header.frame_id = self.goal_frame
            self.goal_message.pose.pose.position.x = self.goal_x
            self.goal_message.pose.pose.position.y = self.goal_y
            self.goal_message.pose.pose.orientation.z = sin(self.goal_yaw / 2.0)
            self.goal_message.pose.pose.orientation.w = cos(self.goal_yaw / 2.0)

        self.goal_message.pose.header.stamp = self.get_clock().now().to_msg()
        self.latest_distance = 0.0
        self.latest_time = 'Pending'
        self.gui.set_mission_status('starting')
        self.gui.set_mission_running(True)
        future = self.navigate_client.send_goal_async(
            self.goal_message,
            feedback_callback=self._feedback_callback,
        )
        future.add_done_callback(self._goal_response_callback)

    def _feedback_callback(self, feedback_message) -> None:
        feedback = feedback_message.feedback
        self.latest_distance = float(feedback.distance_remaining)
        duration = feedback.estimated_time_remaining
        self.latest_time = format_remaining_time(
            duration.sec + duration.nanosec / 1e9
        )

    def _retry_start_mission(self) -> None:
        if self.start_request_pending and self.navigate_client.server_is_ready():
            self.start_mission()

    def stop_mission(self) -> None:
        if self.goal_handle is None:
            self.gui.set_mission_status('paused')
            self.gui.set_mission_running(False)
            return

        self.gui.set_mission_status('stopping')
        future = self.goal_handle.cancel_goal_async()
        future.add_done_callback(self._cancel_response_callback)

    def _goal_response_callback(self, future) -> None:
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.goal_handle = None
            self.gui.set_mission_status('rejected')
            self.gui.set_mission_running(False)
            return

        self.gui.set_mission_status('running')
        self.gui.set_mission_running(True)
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _cancel_response_callback(self, future) -> None:
        if future.result().goals_canceling:
            self.goal_handle = None
            self.gui.set_mission_status('paused')
            self.gui.set_mission_running(False)
        else:
            self.gui.set_mission_status('cancel failed')

    def _result_callback(self, future) -> None:
        status = future.result().status
        self.goal_handle = None
        self.gui.set_mission_running(False)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.gui.set_mission_status('complete')
        elif status == GoalStatus.STATUS_CANCELED:
            self.gui.set_mission_status('paused')
        else:
            self.gui.set_mission_status('failed')

    def _odom_callback(self, msg: Odometry) -> None:
        self.latest_speed = abs(float(msg.twist.twist.linear.x))
        self.status.speed = self.latest_speed

    def _scan_callback(self, msg: LaserScan) -> None:
        if not hasattr(msg, 'ranges'):
            return
        self.latest_obstacle_detected = detect_obstacle(msg.ranges, threshold=self.obstacle_threshold)
        self.status.obstacle_detected = self.latest_obstacle_detected

    def _camera_callback(self, msg: Image) -> None:
        self.gui.update_camera(msg)

    def _fire_callback(self, msg: Bool) -> None:
        self.latest_fire_detected = bool(msg.data)

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self.gui.update_map(msg)

    def _update_gui(self) -> None:
        self.status.speed = self.latest_speed
        self.status.obstacle_detected = self.latest_obstacle_detected
        self.status.fire_detected = self.latest_fire_detected
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
    gui.set_mission_callbacks(node.start_mission, node.stop_mission)
    gui.set_speed_callback(node.set_speed)
    node.set_speed('slow')

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
