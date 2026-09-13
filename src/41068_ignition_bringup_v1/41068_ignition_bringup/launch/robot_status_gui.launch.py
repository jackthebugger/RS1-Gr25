from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot = LaunchConfiguration('robot')
    use_sim_time = LaunchConfiguration('use_sim_time')
    obstacle_threshold = LaunchConfiguration('obstacle_threshold')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            default_value='husky1',
            description='Robot namespace (topics /{robot}/odom and /{robot}/scan)',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation clock when attached to Gazebo',
        ),
        DeclareLaunchArgument(
            'obstacle_threshold',
            default_value='1.0',
            description='Laser range (m) below which an obstacle is reported',
        ),
        Node(
            package='41068_ignition_bringup',
            executable='robot_status_gui.py',
            namespace=robot,
            name='robot_status_gui',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_name': robot,
                'odom_topic': 'odom',
                'scan_topic': 'scan',
                'obstacle_threshold': obstacle_threshold,
            }],
        ),
    ])
