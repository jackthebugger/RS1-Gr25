#!/usr/bin/env python3
"""Diagnose why an angular cmd_vel produces almost no rotation.

Commands a fixed yaw rate and reports, side by side:
  * the commanded rate;
  * the IMU's measured yaw rate (raw sensor);
  * the Gazebo OdometryPublisher's yaw rate (raw plugin);
  * the EKF's yaw rate (/husky1/odom);
  * Gazebo's ground-truth model pose.

If the IMU and the raw plugin agree with each other but not with the command,
the robot really is not turning and the cause is physics, not estimation.
"""

import math
import subprocess
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu

from nav_test_lib import await_simulation, bringup, log
from rs1_nav import NavObserver, init_ros

COMMANDED_RATE = 0.5
DURATION = 4.0


class RawWatcher(Node):
    """Subscribes to the pre-EKF sources so estimation can be ruled in or out."""

    def __init__(self):
        super().__init__('raw_watcher', namespace='/husky1')
        self.imu_rates = []
        self.plugin_rates = []
        self.create_subscription(Imu, 'imu', self._on_imu, 20)
        self.create_subscription(Odometry, 'odometry', self._on_odom, 20)
        self.collecting = False

    def _on_imu(self, msg):
        if self.collecting:
            self.imu_rates.append(msg.angular_velocity.z)

    def _on_odom(self, msg):
        if self.collecting:
            self.plugin_rates.append(msg.twist.twist.angular.z)


def gazebo_pose() -> str:
    try:
        done = subprocess.run(
            ['ign', 'model', '-m', 'husky1', '-p'],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, timeout=15.0, check=False)
        lines = [l.strip() for l in (done.stdout or '').splitlines() if l.strip()]
        return ' | '.join(lines[:6]) or '(no output)'
    except (subprocess.TimeoutExpired, FileNotFoundError) as exc:
        return f'(unavailable: {exc})'


def mean(values):
    return sum(values) / len(values) if values else float('nan')


def main() -> int:
    sup = bringup(world='simple_trees', nav2=False, rviz=False, gui=False,
                  max_runtime=240.0, log_path='/tmp/diag_rotation.log')
    with sup:
        if not await_simulation(sup, require_nav2=False)[0]:
            return 1

        init_ros()
        probe = NavObserver()
        raw = RawWatcher()
        try:
            probe.wait_for('odometry', lambda: probe.odom is not None, timeout=30.0)

            log(f'ground truth before: {gazebo_pose()}')
            start_yaw = probe.odom_pose()[2]
            raw.collecting = True

            deadline = time.monotonic() + DURATION
            while time.monotonic() < deadline:
                probe.drive(0.0, COMMANDED_RATE)
                rclpy.spin_once(probe, timeout_sec=0.02)
                rclpy.spin_once(raw, timeout_sec=0.02)
            raw.collecting = False
            probe.drive_for(0.0, 0.0, 0.5)
            probe.spin_for(1.0)

            end_yaw = probe.odom_pose()[2]
            ekf_rate = math.atan2(math.sin(end_yaw - start_yaw),
                                  math.cos(end_yaw - start_yaw)) / DURATION

            log(f'commanded yaw rate        : {COMMANDED_RATE:+.3f} rad/s')
            log(f'IMU measured yaw rate     : {mean(raw.imu_rates):+.3f} rad/s '
                f'({len(raw.imu_rates)} samples)')
            log(f'Gazebo plugin yaw rate    : {mean(raw.plugin_rates):+.3f} rad/s '
                f'({len(raw.plugin_rates)} samples)')
            log(f'EKF integrated yaw rate   : {ekf_rate:+.3f} rad/s')
            log(f'ground truth after : {gazebo_pose()}')
        finally:
            raw.destroy_node()
            probe.destroy_node()
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
