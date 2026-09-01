#!/usr/bin/env python3
"""Small modular GUI for displaying robot status during autonomy demos."""

import tkinter as tk
from dataclasses import dataclass
from typing import Iterable, List, Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


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
        self.geometry('480x260')
        self.minsize(420, 230)

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

        self.info_frame = tk.Frame(self, bg='#ffffff', bd=1, relief='solid')
        self.info_frame.pack(fill='both', expand=True, padx=20, pady=(0, 20))

        self.robot_name_var = tk.StringVar(value='Robot Name: Rescue Bot')
        self.destination_var = tk.StringVar(value='Destination: Safehouse')
        self.speed_var = tk.StringVar(value='Speed: 0.00 m/s')
        self.distance_var = tk.StringVar(value='Distance to Destination: 0.0 m')
        self.time_var = tk.StringVar(value='Time to Destination: Pending')
        self.battery_var = tk.StringVar(value='Battery: 100%')
        self.obstacle_var = tk.StringVar(value='Obstacle detected: no')

        self.labels = [
            tk.Label(self.info_frame, textvariable=self.robot_name_var, font=('Arial', 12), bg='#ffffff', fg='#111827', anchor='w', padx=16, pady=6),
            tk.Label(self.info_frame, textvariable=self.destination_var, font=('Arial', 12), bg='#ffffff', fg='#111827', anchor='w', padx=16, pady=6),
            tk.Label(self.info_frame, textvariable=self.speed_var, font=('Arial', 12), bg='#ffffff', fg='#111827', anchor='w', padx=16, pady=6),
            tk.Label(self.info_frame, textvariable=self.distance_var, font=('Arial', 12), bg='#ffffff', fg='#111827', anchor='w', padx=16, pady=6),
            tk.Label(self.info_frame, textvariable=self.time_var, font=('Arial', 12), bg='#ffffff', fg='#111827', anchor='w', padx=16, pady=6),
            tk.Label(self.info_frame, textvariable=self.battery_var, font=('Arial', 12), bg='#ffffff', fg='#111827', anchor='w', padx=16, pady=6),
            tk.Label(self.info_frame, textvariable=self.obstacle_var, font=('Arial', 12), bg='#ffffff', fg='#111827', anchor='w', padx=16, pady=6),
        ]

        for label in self.labels:
            label.pack(fill='x')

        self.update_status()

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
        self.status.distance_to_destination = 0.0
        self.status.time_to_destination = 'Pending'
        self.status.battery = 100.0

        odom_topic = self.declare_parameter('odom_topic', 'odom').value
        scan_topic = self.declare_parameter('scan_topic', 'scan').value
        obstacle_threshold = self.declare_parameter('obstacle_threshold', 1.0).value
        self.obstacle_threshold = float(obstacle_threshold)

        self.create_subscription(Odometry, odom_topic, self._odom_callback, 10)
        self.create_subscription(LaserScan, scan_topic, self._scan_callback, qos_profile_sensor_data)

        self.create_subscription(LaserScan, 'base_scan', self._scan_callback, qos_profile_sensor_data)
        self.create_timer(0.1, self._update_gui)

        self.get_logger().info(
            f'Robot status GUI started for {self.robot_name}. '
            f'Listening to {odom_topic} and {scan_topic}/base_scan.'
        )

    def _odom_callback(self, msg: Odometry) -> None:
        self.latest_speed = abs(float(msg.twist.twist.linear.x))
        self.status.speed = self.latest_speed

    def _scan_callback(self, msg: LaserScan) -> None:
        if not hasattr(msg, 'ranges'):
            return
        self.latest_obstacle_detected = detect_obstacle(msg.ranges, threshold=self.obstacle_threshold)
        self.status.obstacle_detected = self.latest_obstacle_detected

    def _update_gui(self) -> None:
        self.status.speed = self.latest_speed
        self.status.obstacle_detected = self.latest_obstacle_detected
        self.status.destination = 'Safehouse'
        self.status.distance_to_destination = 0.0
        self.status.time_to_destination = 'Pending'
        self.status.battery = 100.0
        self.gui.set_status(self.status)


def main(args=None):
    rclpy.init(args=args)

    robot_name = 'husky1'
    argv = args or []
    for idx, arg in enumerate(argv):
        if arg.startswith('__name:='):
            continue
        if arg.startswith('--robot'):
            robot_name = arg.split('=', 1)[1] if '=' in arg else argv[idx + 1]
            break

    gui = RobotStatusWindow(robot_name)
    node = RobotStatusNode(gui, robot_name)

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
