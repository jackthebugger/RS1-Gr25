import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    pkg = FindPackageShare('41068_ignition_bringup')
    config_path = PathJoinSubstitution([pkg, 'config'])
    use_sim_time = LaunchConfiguration('use_sim_time')

    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='True'))

    # Resolve package models (grass_plane, tree_burning, etc.).
    pkg_share = get_package_share_directory('41068_ignition_bringup')
    models_path = os.path.join(pkg_share, 'models')
    resource_path_parts = [models_path]
    for env_name in ('IGN_GAZEBO_RESOURCE_PATH', 'GZ_SIM_RESOURCE_PATH'):
        existing = os.environ.get(env_name, '')
        if existing:
            resource_path_parts.extend(p for p in existing.split(os.pathsep) if p)
    seen = set()
    resource_path = os.pathsep.join(
        p for p in resource_path_parts if not (p in seen or seen.add(p))
    )
    ld.add_action(SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=resource_path))
    ld.add_action(SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=resource_path))

    server_config_file = PathJoinSubstitution([config_path, 'ignition_server.config'])
    ld.add_action(SetEnvironmentVariable(name='IGN_GAZEBO_SERVER_CONFIG_PATH', value=server_config_file))
    ld.add_action(SetEnvironmentVariable(name='GZ_SIM_SERVER_CONFIG_PATH', value=server_config_file))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ros_ign_gazebo'),
            'launch',
            'ign_gazebo.launch.py',
        ])),
        launch_arguments={
            'ign_args': [PathJoinSubstitution([pkg, 'worlds', 'beer_demo_world.sdf']), ' -r'],
        }.items(),
    ))

    ld.add_action(Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge_clock',
        output='screen',
        parameters=[{
            'config_file': PathJoinSubstitution([config_path, 'gazebo_bridge_clock.yaml']),
            'use_sim_time': use_sim_time,
        }],
    ))

    robot_description_content = ParameterValue(
        Command([
            'xacro ', PathJoinSubstitution([pkg, 'urdf_husky', 'husky.urdf.xacro']),
            ' prefix:=', 'husky1_', ' gz_model_name:=husky1',
        ]),
        value_type=str,
    )

    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='husky1',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
        }],
    ))

    spawn_husky = Node(
        package='ros_ign_gazebo',
        executable='create',
        name='spawn_husky1',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-topic', '/husky1/robot_description',
            '-name', 'husky1',
            '-x', '0.0', '-y', '0.0', '-z', '0.4', '-Y', '0.0',
        ],
    )
    ld.add_action(TimerAction(period=3.0, actions=[spawn_husky]))

    ld.add_action(Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        namespace='husky1',
        name='gazebo_bridge',
        output='screen',
        parameters=[{
            'config_file': PathJoinSubstitution([config_path, 'gazebo_bridge_husky1.yaml']),
            'use_sim_time': use_sim_time,
        }],
    ))

    ld.add_action(Node(
        package='beer_fire_detection',
        executable='synthetic_thermal_demo',
        name='synthetic_thermal_demo',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'demo_topic': '/husky1/thermal/demo_image',
            'visual_topic': '/husky1/thermal/demo_visual',
            'scan_topic': '/husky1/scan',
            'resolution': 0.01,
        }],
    ))

    ld.add_action(Node(
        package='beer_fire_detection',
        executable='fire_detector',
        name='beer_fire_detector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'thermal_topic': '/husky1/thermal/demo_image',
            'fire_threshold_kelvin': 400.0,
            'minimum_hot_pixels': 20,
            'thermal_resolution': 0.01,
        }],
    ))

    return ld
