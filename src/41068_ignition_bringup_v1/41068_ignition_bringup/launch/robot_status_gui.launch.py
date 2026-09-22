from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    robot = LaunchConfiguration('robot')
    use_sim_time = LaunchConfiguration('use_sim_time')
    obstacle_threshold = LaunchConfiguration('obstacle_threshold')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_yaw = LaunchConfiguration('goal_yaw')
    world_name = LaunchConfiguration('world_name')

    share = get_package_share_directory('41068_ignition_bringup')
    gaps_config = os.path.join(share, 'config', 'forest_gaps_custom_world_1.yaml')

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
        DeclareLaunchArgument(
            'goal_x',
            default_value='18.0',
            description='Default Start Mission goal X (map frame, metres)',
        ),
        DeclareLaunchArgument(
            'goal_y',
            default_value='0.0',
            description='Default Start Mission goal Y (map frame, metres)',
        ),
        DeclareLaunchArgument(
            'goal_yaw',
            default_value='0.0',
            description='Default Start Mission goal yaw (radians)',
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value='custom_world_1',
            description='Gazebo world name for dynamic obstacle spawn/remove',
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
                'goal_frame': 'husky1_map',
                'goal_x': goal_x,
                'goal_y': goal_y,
                'goal_yaw': goal_yaw,
                'world_name': world_name,
                'forest_gaps_config': gaps_config,
            }],
        ),
    ])
