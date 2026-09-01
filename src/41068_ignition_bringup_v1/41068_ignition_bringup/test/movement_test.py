#!/usr/bin/env python3
"""Verify the actuation chain: cmd_vel -> bridge -> VelocityControl -> Husky.

    /husky1/cmd_vel  (geometry_msgs/Twist, ROS)
        -> ros_ign_bridge
        -> /model/husky1/cmd_vel  (ignition.msgs.Twist)
        -> ignition::gazebo::systems::VelocityControl
        -> Husky moves
        -> OdometryPublisher + IMU -> EKF -> /husky1/odom and TF

This is the one test that publishes velocity commands itself; it exists to prove
the actuation interface works before anything autonomous is attempted. The
autonomous tests never command the robot directly.

Checks, in order:
  1. a forward command is tracked accurately, in both odom and TF;
  2. yaw commands are tracked accurately at two different rates, without
     drifting off the spot;
  3. a zero command actually stops the robot.

Tracking accuracy matters beyond "it moved": DWB predicts its candidate
trajectories from the velocities it commands, so a robot that only achieves 60%
of a commanded yaw rate systematically under-turns and follows paths badly. The
tolerance below is therefore tight, and is what calibrates the DiffDrive
effective_wheel_separation launch argument.

    python3 test/movement_test.py
"""

import argparse
import math
import sys

import rclpy

from nav_test_lib import await_simulation, bringup, log, print_diagnostics
from rs1_nav import NavObserver, init_ros

DRIVE_SPEED = 0.4        # m/s
DRIVE_TIME = 4.0         # s
LINEAR_TOLERANCE = 0.20  # fraction of the commanded distance

TURN_RATES = (0.4, 0.8)  # rad/s; two rates confirm the mapping is linear
TURN_TIME = 3.0          # s
ANGULAR_TOLERANCE = 0.25  # fraction of the commanded yaw change
MAX_TURN_DRIFT = 0.35    # m of translation allowed while turning in place

MAX_STOPPED_DRIFT = 0.10  # m of movement allowed after commanding zero


def angle_difference(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--wheel-separation', type=float, default=None,
        help='Override effective_wheel_separation, to recalibrate yaw tracking')
    parser.add_argument(
        '--drive-plugin', default=None, choices=['diff_drive', 'velocity_control'],
        help='Override the Gazebo drive system, to compare the two')
    args = parser.parse_args(argv)

    extra = {}
    if args.wheel_separation is not None:
        extra['effective_wheel_separation'] = f'{args.wheel_separation}'
    if args.drive_plugin is not None:
        extra['drive_plugin'] = args.drive_plugin

    sup = bringup(
        world='simple_trees',
        nav2=False,          # the actuation chain does not involve Nav2
        rviz=False,
        gui=False,
        max_runtime=300.0,
        log_path='/tmp/movement_test.log',
        extra=extra or None,
    )

    results = []

    with sup:
        ok, _stages = await_simulation(sup, require_nav2=False)
        if not ok:
            print_diagnostics(sup, topics=['/husky1/cmd_vel', '/husky1/odom'])
            log('MOVEMENT TEST FAILED (simulation did not start)')
            return 1

        init_ros()
        probe = NavObserver()
        try:
            if not probe.wait_for(
                    'EKF odometry and TF available',
                    lambda: probe.odom is not None and probe.robot_pose(
                        frame=probe.odom_frame) is not None,
                    timeout=40.0):
                print_diagnostics(sup, topics=['/husky1/odom'])
                return 1

            # -- 1. forward ---------------------------------------------------
            start_odom = probe.odom_pose()
            start_tf = probe.robot_pose(frame=probe.odom_frame)
            probe.drive_for(DRIVE_SPEED, 0.0, DRIVE_TIME)
            probe.spin_for(1.5)
            end_odom = probe.odom_pose()
            end_tf = probe.robot_pose(frame=probe.odom_frame)

            odom_moved = math.hypot(end_odom[0] - start_odom[0],
                                    end_odom[1] - start_odom[1])
            tf_moved = math.hypot(end_tf[0] - start_tf[0], end_tf[1] - start_tf[1])
            expected = DRIVE_SPEED * DRIVE_TIME
            error = abs(odom_moved - expected) / expected
            log(f'forward {DRIVE_SPEED} m/s for {DRIVE_TIME}s: moved {odom_moved:.2f} m '
                f'(expected {expected:.2f} m, error {error * 100:.0f}%), '
                f'TF agrees to {abs(tf_moved - odom_moved):.3f} m')
            results.append(('forward velocity tracking',
                            error <= LINEAR_TOLERANCE and abs(tf_moved - odom_moved) < 0.05,
                            f'{odom_moved:.2f}/{expected:.2f} m, {error * 100:.0f}% error'))

            # -- 2. rotation at two rates -------------------------------------
            for rate in TURN_RATES:
                start_odom = probe.odom_pose()
                probe.drive_for(0.0, rate, TURN_TIME)
                probe.spin_for(1.5)
                end_odom = probe.odom_pose()

                turned = angle_difference(end_odom[2], start_odom[2])
                drifted = math.hypot(end_odom[0] - start_odom[0],
                                     end_odom[1] - start_odom[1])
                achieved = turned / TURN_TIME
                error = abs(achieved - rate) / rate
                log(f'rotate {rate} rad/s for {TURN_TIME}s: achieved {achieved:.3f} rad/s '
                    f'({achieved / rate * 100:.0f}% of command), drifted {drifted:.2f} m')
                results.append((f'yaw rate tracking at {rate} rad/s',
                                error <= ANGULAR_TOLERANCE and drifted <= MAX_TURN_DRIFT,
                                f'{achieved:.3f}/{rate} rad/s, {error * 100:.0f}% error, '
                                f'{drifted:.2f} m drift'))

            # -- 3. stop ------------------------------------------------------
            probe.drive_for(0.0, 0.0, 1.0)
            before_stop = probe.odom_pose()
            probe.spin_for(3.0)
            after_stop = probe.odom_pose()
            residual = math.hypot(after_stop[0] - before_stop[0],
                                  after_stop[1] - before_stop[1])
            log(f'after commanding zero, robot drifted {residual:.3f} m in 3 s')
            results.append(('zero command stops the robot',
                            residual <= MAX_STOPPED_DRIFT,
                            f'{residual:.3f} m'))
        finally:
            probe.destroy_node()
            rclpy.shutdown()

    log('=' * 68)
    for name, passed, detail in results:
        log(f'  {"PASS" if passed else "FAIL"}  {name}  [{detail}]')
    all_passed = bool(results) and all(passed for _n, passed, _d in results)
    log(f'MOVEMENT TEST {"PASSED" if all_passed else "FAILED"}')
    log('=' * 68)
    return 0 if all_passed else 1


if __name__ == '__main__':
    sys.exit(main())
