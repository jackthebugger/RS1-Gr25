from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ld = LaunchDescription()

    pkg = FindPackageShare('41068_ignition_bringup')
    config_path = PathJoinSubstitution([pkg, 'config'])

    # Argument to control sim time
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='True'))
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Ensure server config is used (same as original launch)
    server_config_file = PathJoinSubstitution([config_path, 'ignition_server.config'])
    ld.add_action(SetEnvironmentVariable(name='IGN_GAZEBO_SERVER_CONFIG_PATH', value=server_config_file))
    ld.add_action(SetEnvironmentVariable(name='GZ_SIM_SERVER_CONFIG_PATH', value=server_config_file))

    # Launch Ignition/Gazebo with the beer demo world
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ros_ign_gazebo'),
            'launch',
            'ign_gazebo.launch.py',
        ])),
        launch_arguments={'ign_args': [PathJoinSubstitution([pkg, 'worlds', 'beer_demo_world.sdf']), ' -r']}.items(),
    ))

    # Start ROS<->Ignition clock bridge
    ld.add_action(Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge_clock',
        output='screen',
        parameters=[{'config_file': PathJoinSubstitution([config_path, 'gazebo_bridge_clock.yaml']), 'use_sim_time': use_sim_time}],
    ))

    # Publish robot_description via xacro and start robot_state_publisher for husky1
    pkg_path = FindPackageShare('41068_ignition_bringup')
    robot_description_content = ParameterValue(
        Command([
            'xacro ', PathJoinSubstitution([pkg_path, 'urdf_husky', 'husky.urdf.xacro']),
            ' prefix:=', 'husky1_', ' gz_model_name:=husky1'
        ]),
        value_type=str,
    )

    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='husky1',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': use_sim_time}],
    ))

    # Spawn the Husky using ros_ign_gazebo create (topic-based robot_description)
    spawn_husky = Node(
        package='ros_ign_gazebo',
        executable='create',
        name='spawn_husky1',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-topic', '/husky1/robot_description', '-name', 'husky1', '-x', '0.0', '-y', '0.0', '-z', '0.4', '-Y', '0.0'],
    )
    ld.add_action(TimerAction(period=3.0, actions=[spawn_husky]))

    # Start the bridge for the husky topics using existing bridge config
    ld.add_action(Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        namespace='husky1',
        name='gazebo_bridge',
        output='screen',
        parameters=[{'config_file': PathJoinSubstitution([config_path, 'gazebo_bridge_husky1.yaml']), 'use_sim_time': use_sim_time}],
    ))

    # Start synthetic thermal demo script directly (avoid package lookup issues)
    from launch.actions import ExecuteProcess
    synthetic_script = '/home/faiyad/RS1-Gr25/install/beer_fire_detection/bin/synthetic_thermal_demo'
    ld.add_action(ExecuteProcess(
        cmd=[synthetic_script, '--ros-args', '-p', 'demo_topic:=/husky1/thermal/demo_image', '-p', 'visual_topic:=/husky1/thermal/demo_visual', '-p', 'scan_topic:=/husky1/scan', '-p', 'resolution:=0.01'],
        output='screen',
    ))

    # Start the fire detector script directly
    fire_script = '/home/faiyad/RS1-Gr25/install/beer_fire_detection/bin/fire_detector'
    ld.add_action(ExecuteProcess(
        cmd=[fire_script, '--ros-args', '-p', 'thermal_topic:=/husky1/thermal/demo_image', '-p', 'fire_threshold_kelvin:=400.0', '-p', 'minimum_hot_pixels:=20', '-p', 'thermal_resolution:=0.01'],
        output='screen',
    ))

    return ld
