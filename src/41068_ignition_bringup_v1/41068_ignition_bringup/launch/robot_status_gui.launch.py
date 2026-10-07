from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def _gap_config_for_world(world_name: str, share: str) -> str:
    """Pick Path A/B YAML that matches the running Gazebo world."""
    mapping = {
        'bush_trail_world': 'trail_blocks_bush_trail_world.yaml',
        'custom_world_1': 'forest_gaps_custom_world_1.yaml',
    }
    filename = mapping.get(world_name, mapping['bush_trail_world'])
    path = os.path.join(share, 'config', filename)
    if os.path.isfile(path):
        return path
    return os.path.join(share, 'config', 'forest_gaps_custom_world_1.yaml')


def _launch_setup(context, *args, **kwargs):
    robot = LaunchConfiguration('robot')
    use_sim_time = LaunchConfiguration('use_sim_time')
    obstacle_threshold = LaunchConfiguration('obstacle_threshold')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_yaw = LaunchConfiguration('goal_yaw')
    world_name = LaunchConfiguration('world_name')
    forest_gaps_config = LaunchConfiguration('forest_gaps_config')

    share = get_package_share_directory('41068_ignition_bringup')
    world = world_name.perform(context).strip() or 'bush_trail_world'
    gaps_override = forest_gaps_config.perform(context).strip()
    gaps_config = gaps_override or _gap_config_for_world(world, share)

    return [
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
                'fire_topic': 'fire_detected',
            }],
        ),
        Node(
            package='beer_fire_detection',
            executable='fire_detector',
            name='beer_fire_detector',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'thermal_topic': 'thermal/image',
                'fire_threshold_kelvin': 400.0,
                'minimum_hot_pixels': 20,
                'thermal_resolution': 0.01,
            }],
            remappings=[
                ('/fire_detected', 'fire_detected'),
                ('/fire_temperature', 'fire_temperature'),
            ],
        ),
    ]


def generate_launch_description():
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
            default_value='16.0',
            description='Default Start Mission goal X (on-trail for bush_trail_world)',
        ),
        DeclareLaunchArgument(
            'goal_y',
            default_value='-0.1',
            description='Default Start Mission goal Y (on-trail for bush_trail_world)',
        ),
        DeclareLaunchArgument(
            'goal_yaw',
            default_value='0.0',
            description='Default Start Mission goal yaw (radians)',
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value='bush_trail_world',
            description='Gazebo world name for dynamic obstacle spawn/remove',
        ),
        DeclareLaunchArgument(
            'forest_gaps_config',
            default_value='',
            description=(
                'Optional Path A/B YAML override. Empty → auto-select from world_name '
                '(bush_trail_world or custom_world_1).'
            ),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
