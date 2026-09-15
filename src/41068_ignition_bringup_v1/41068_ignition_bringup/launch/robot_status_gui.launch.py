from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    robot_name = LaunchConfiguration('robot_name')
    odom_topic = LaunchConfiguration('odom_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    obstacle_threshold = LaunchConfiguration('obstacle_threshold')

    ld.add_action(DeclareLaunchArgument('robot_name', default_value='husky1'))
    ld.add_action(DeclareLaunchArgument('odom_topic', default_value='odom'))
    ld.add_action(DeclareLaunchArgument('scan_topic', default_value='scan'))
    ld.add_action(DeclareLaunchArgument('obstacle_threshold', default_value='1.0'))

    ld.add_action(Node(
        package='41068_ignition_bringup',
        executable='robot_status_gui.py',
        name='robot_status_gui',
        output='screen',
        parameters=[{
            'robot_name': robot_name,
            'odom_topic': odom_topic,
            'scan_topic': scan_topic,
            'obstacle_threshold': obstacle_threshold,
            'use_sim_time': True,
        }],
    ))

    return ld
