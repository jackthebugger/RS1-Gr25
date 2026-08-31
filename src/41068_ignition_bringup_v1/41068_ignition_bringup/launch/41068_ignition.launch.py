import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


_TRUE_STRINGS = "['true', '1', 'yes', 'on']"


def _is_true_expression(name):
    return ["'", LaunchConfiguration(name), "'.lower() in ", _TRUE_STRINGS]


def _if_true(name):
    return IfCondition(PythonExpression(_is_true_expression(name)))


def _if_all_true(*names):
    pieces = []
    for i, name in enumerate(names):
        if i > 0:
            pieces.append(' and ')
        pieces.extend(_is_true_expression(name))
    return IfCondition(PythonExpression(pieces))


def add_robot(
    ld,
    *,
    pkg_path,
    config_path,
    use_sim_time,
    enabled_arg,
    namespace,
    frame_prefix,
    gz_model_name,
    xacro_parts,
    bridge_config,
    localization_config,
    x,
    y,
    z,
    yaw='0.0',
    spawn_delay=0.0,
    xacro_extra_args=(),
):
    """Add one robot instance to the launch description.

    Convention used here:
      - ROS nodes/topics are under /<namespace>/...
      - Gazebo entity/model is named <gz_model_name>
      - Gazebo plugin topics are under /model/<gz_model_name>/...
      - TF topics are under /<namespace>/tf and /<namespace>/tf_static
      - TF frame ids use <frame_prefix>, e.g. husky1_base_link
    """

    xacro_command = [
        'xacro ',
        PathJoinSubstitution([pkg_path] + xacro_parts),
        ' ',
        'prefix:=', frame_prefix,
        ' ',
        'gz_model_name:=', gz_model_name,
    ]
    for arg_name in xacro_extra_args:
        xacro_command += [' ', f'{arg_name}:=', LaunchConfiguration(arg_name)]

    robot_description_content = ParameterValue(
        Command(xacro_command),
        value_type=str,
    )

    robot_enabled = _if_true(enabled_arg)

    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        condition=robot_enabled,
    ))

    ld.add_action(Node(
        package='41068_ignition_bringup',
        executable='odometry_tf_broadcaster.py',
        namespace=namespace,
        name='odometry_tf_broadcaster',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_topic': 'odometry',
            'output_odom_topic': 'odom',
            'odom_frame': frame_prefix + 'odom',
            'base_frame': frame_prefix + 'base_link',
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        condition=robot_enabled,
    ))

    robot_spawner = Node(
        package='ros_ign_gazebo',
        executable='create',
        name='spawn_' + namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-topic', '/' + namespace + '/robot_description',
            '-name', gz_model_name,
            '-x', x,
            '-y', y,
            '-z', z,
            '-Y', yaw,
        ],
    )

    # Start Gazebo first, then insert robot entities in a predictable order.
    ld.add_action(TimerAction(
        period=spawn_delay,
        actions=[robot_spawner],
        condition=robot_enabled,
    ))

    ld.add_action(Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        name='gazebo_bridge',
        output='screen',
        parameters=[{
            'config_file': PathJoinSubstitution([config_path, bridge_config]),
            'use_sim_time': use_sim_time,
        }],
        condition=robot_enabled,
    ))


def add_navigation_instance(
    ld,
    *,
    pkg_path,
    use_sim_time,
    robot_arg,
    robot_namespace,
    start_delay,
):
    nav_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_path,
                'launch',
                '41068_navigation.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': robot_namespace,
            'config_filename_suffix': '_' + robot_namespace,
            'slam': LaunchConfiguration('slam'),
            'nav2': LaunchConfiguration('nav2'),
        }.items(),
    )

    # Scope is important: without it, repeated IncludeLaunchDescription calls can
    # leak/overwrite launch configurations such as namespace and config suffix.
    ld.add_action(TimerAction(
        period=start_delay,
        actions=[GroupAction(
            scoped=True,
            actions=[nav_include],
        )],
        condition=_if_true(robot_arg),
    ))


def add_rviz_instance(
    ld,
    *,
    config_path,
    use_sim_time,
    robot_arg,
    robot_namespace,
    rviz_config,
    start_delay,
):
    # Each RViz process is itself placed in the robot namespace so the Nav2 RViz
    # panel resolves relative action/service names such as navigate_to_pose and
    # lifecycle_manager_navigation to the correct robot.
    ld.add_action(TimerAction(
        period=start_delay,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            namespace=robot_namespace,
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', PathJoinSubstitution([config_path, rviz_config])],
            remappings=[
                ('/tf', '/' + robot_namespace + '/tf'),
                ('/tf_static', '/' + robot_namespace + '/tf_static'),
            ],
        )],
        condition=_if_all_true('rviz', robot_arg),
    ))


def generate_launch_description():

    ld = LaunchDescription()

    pkg_path = FindPackageShare('41068_ignition_bringup')
    config_path = PathJoinSubstitution([pkg_path, 'config'])

    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time',
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    ld.add_action(use_sim_time_launch_arg)

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Flag to launch RViz',
    )
    ld.add_action(rviz_launch_arg)

    slam_launch_arg = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Flag to launch SLAM Toolbox for each enabled robot',
    )
    ld.add_action(slam_launch_arg)

    nav2_launch_arg = DeclareLaunchArgument(
        'nav2',
        default_value='False',
        description='Flag to launch Nav2 for each enabled robot. Nav2 also starts SLAM.',
    )
    ld.add_action(nav2_launch_arg)

    husky_launch_arg = DeclareLaunchArgument(
        'husky',
        default_value='True',
        description='Launch the Husky UGV instance in namespace /husky1',
    )
    ld.add_action(husky_launch_arg)

    parrot_launch_arg = DeclareLaunchArgument(
        'parrot',
        default_value='False',
        description='Launch the Parrot drone instance in namespace /parrot1',
    )
    ld.add_action(parrot_launch_arg)

    world_launch_arg = DeclareLaunchArgument(
        'world',
        default_value='simple_trees',
        description='Which world to load',
        choices=['simple_trees', 'large_demo'],
    )
    ld.add_action(world_launch_arg)

    # Spawn pose of each robot, exposed as launch arguments so the starting
    # position can be changed without editing this file. Values are in the
    # Gazebo world frame, which coincides with husky1_map at startup.
    for name, default, description in (
        ('husky_x', '0.0', 'Husky spawn X position in metres'),
        ('husky_y', '0.0', 'Husky spawn Y position in metres'),
        ('husky_z', '0.4', 'Husky spawn Z position in metres'),
        ('husky_yaw', '0.0', 'Husky spawn yaw in radians'),
        ('parrot_x', '2.0', 'Parrot spawn X position in metres'),
        ('parrot_y', '0.0', 'Parrot spawn Y position in metres'),
        ('parrot_z', '0.8', 'Parrot spawn Z position in metres'),
        ('parrot_yaw', '0.0', 'Parrot spawn yaw in radians'),
    ):
        ld.add_action(DeclareLaunchArgument(name, default_value=default, description=description))

    # Husky sensor tuning.
    #
    # enable_camera defaults to false. On this project's reference machine
    # (WSL2 + software GL) spawning the Husky with its rgbd_camera makes the
    # Ignition server stop stepping entirely: no /clock, no scan, no odometry,
    # so SLAM and Nav2 never start. With the camera off the lidar runs at a
    # solid 10 Hz and the whole navigation stack works. Neither SLAM nor Nav2
    # consumes the camera, so nothing in the navigation pipeline is lost.
    # Set enable_camera:=true only on a machine with working GPU rendering.
    for name, default, description in (
        ('enable_camera', 'false',
         'Enable the Husky RGB-D camera. Known to stall Ignition on WSL/software GL; '
         'not used by SLAM or Nav2.'),
        ('lidar_update_rate', '10', 'Husky lidar update rate in Hz'),
        ('lidar_samples', '360', 'Husky lidar horizontal sample count'),
        ('camera_update_rate', '2', 'Husky RGB-D camera update rate in Hz'),
        ('camera_width', '320', 'Husky RGB-D camera image width in pixels'),
        ('camera_height', '240', 'Husky RGB-D camera image height in pixels'),
    ):
        ld.add_action(DeclareLaunchArgument(name, default_value=default, description=description))

    # Which Gazebo system converts cmd_vel into motion. diff_drive turns the
    # wheel joints and can rotate the robot in place, which Nav2 requires;
    # velocity_control hard-sets base_link's velocity and measurably cannot
    # (0.045 of 0.5 rad/s). Kept selectable so the comparison is reproducible.
    ld.add_action(DeclareLaunchArgument(
        'drive_plugin',
        default_value='diff_drive',
        description='Gazebo system used to drive the Husky base',
        choices=['diff_drive', 'velocity_control'],
    ))
    ld.add_action(DeclareLaunchArgument(
        'effective_wheel_separation',
        default_value='0.94',
        description='Wheel separation used by DiffDrive, in metres. Larger than the '
                    'geometric 0.5708 m track to compensate for skid-steer slip; '
                    'calibrated so achieved yaw rate matches the commanded one.',
    ))

    ld.add_action(DeclareLaunchArgument(
        'nav_start_delay',
        default_value='15.0',
        description='Seconds after launch before starting namespaced SLAM/Nav2. '
                    'Must be after spawn (3 s) so Gazebo has settled; 15 s is '
                    'enough for large_demo Fuel models on software GL.',
    ))

    gui_launch_arg = DeclareLaunchArgument(
        'gui',
        default_value='True',
        description='Launch the Gazebo GUI. Set false for headless (server-only), useful on WSL.',
    )
    ld.add_action(gui_launch_arg)

    # Ensure package models (grass_plane, forest_*, etc.) resolve via model://
    # even when gazebo_ros export hooks are not applied (e.g. bare ign gazebo).
    pkg_share = get_package_share_directory('41068_ignition_bringup')
    models_path = os.path.join(pkg_share, 'models')
    resource_path_parts = [models_path]
    for env_name in ('IGN_GAZEBO_RESOURCE_PATH', 'GZ_SIM_RESOURCE_PATH'):
        existing = os.environ.get(env_name, '')
        if existing:
            resource_path_parts.extend(p for p in existing.split(os.pathsep) if p)
    # Preserve order, drop duplicates.
    seen = set()
    resource_path = os.pathsep.join(
        p for p in resource_path_parts if not (p in seen or seen.add(p))
    )
    ld.add_action(SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=resource_path,
    ))
    ld.add_action(SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_path,
    ))

    # Load common Gazebo server systems from a shared config file. This keeps
    # required systems such as Sensors out of individual robot and world files,
    # so custom student worlds do not need to copy plugin blocks.
    server_config_file = PathJoinSubstitution([
        config_path,
        'ignition_server.config',
    ])
    ld.add_action(SetEnvironmentVariable(
        name='IGN_GAZEBO_SERVER_CONFIG_PATH',
        value=server_config_file,
    ))
    ld.add_action(SetEnvironmentVariable(
        name='GZ_SIM_SERVER_CONFIG_PATH',
        value=server_config_file,
    ))

    # WSL / software-GL mitigations for Ignition rendering (sensors need a
    # render engine even when the Gazebo GUI is off).
    ld.add_action(SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='1'))
    ld.add_action(SetEnvironmentVariable(name='MESA_GL_VERSION_OVERRIDE', value='3.3'))
    ld.add_action(SetEnvironmentVariable(name='MESA_GLSL_VERSION_OVERRIDE', value='330'))
    ld.add_action(SetEnvironmentVariable(name='QT_QPA_PLATFORM', value='xcb'))

    # -r runs immediately. When gui:=false: -s (no GUI) + --headless-rendering
    # so camera/lidar sensors still render without a window (needed on WSL).
    ign_gui_flag = PythonExpression([
        "'' if '", LaunchConfiguration('gui'), "'.lower() in ", _TRUE_STRINGS,
        " else ' -s --headless-rendering'",
    ])

    # Start Gazebo once.
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_ign_gazebo'),
                'launch',
                'ign_gazebo.launch.py',
            ])
        ),
        launch_arguments={
            'ign_args': [
                PathJoinSubstitution([
                    pkg_path,
                    'worlds',
                    [LaunchConfiguration('world'), '.sdf'],
                ]),
                ' -r --render-engine-server ogre',
                ign_gui_flag,
            ],
            'on_exit_shutdown': 'true',
        }.items(),
    ))

    # Bridge the global simulation clock once.
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

    add_robot(
        ld,
        pkg_path=pkg_path,
        config_path=config_path,
        use_sim_time=use_sim_time,
        enabled_arg='husky',
        namespace='husky1',
        frame_prefix='husky1_',
        gz_model_name='husky1',
        xacro_parts=['urdf_husky', 'husky.urdf.xacro'],
        bridge_config='gazebo_bridge_husky1.yaml',
        localization_config='robot_localization_husky1.yaml',
        x=LaunchConfiguration('husky_x'),
        y=LaunchConfiguration('husky_y'),
        z=LaunchConfiguration('husky_z'),
        yaw=LaunchConfiguration('husky_yaw'),
        spawn_delay=3.0,
        xacro_extra_args=(
            'enable_camera',
            'lidar_update_rate',
            'lidar_samples',
            'camera_update_rate',
            'camera_width',
            'camera_height',
            'drive_plugin',
            'effective_wheel_separation',
        ),
    )

    add_robot(
        ld,
        pkg_path=pkg_path,
        config_path=config_path,
        use_sim_time=use_sim_time,
        enabled_arg='parrot',
        namespace='parrot1',
        frame_prefix='parrot1_',
        gz_model_name='parrot1',
        xacro_parts=['urdf_parrot', 'parrot.urdf.xacro'],
        bridge_config='gazebo_bridge_parrot1.yaml',
        localization_config='robot_localization_parrot1.yaml',
        x=LaunchConfiguration('parrot_x'),
        y=LaunchConfiguration('parrot_y'),
        z=LaunchConfiguration('parrot_z'),
        yaw=LaunchConfiguration('parrot_yaw'),
        spawn_delay=6.0,
    )

    add_navigation_instance(
        ld,
        pkg_path=pkg_path,
        use_sim_time=use_sim_time,
        robot_arg='husky',
        robot_namespace='husky1',
        start_delay=LaunchConfiguration('nav_start_delay'),
    )

    add_navigation_instance(
        ld,
        pkg_path=pkg_path,
        use_sim_time=use_sim_time,
        robot_arg='parrot',
        robot_namespace='parrot1',
        start_delay=10.0,
    )

    add_rviz_instance(
        ld,
        config_path=config_path,
        use_sim_time=use_sim_time,
        robot_arg='husky',
        robot_namespace='husky1',
        rviz_config='41068_husky1.rviz',
        start_delay=11.0,
    )

    add_rviz_instance(
        ld,
        config_path=config_path,
        use_sim_time=use_sim_time,
        robot_arg='parrot',
        robot_namespace='parrot1',
        rviz_config='41068_parrot1.rviz',
        start_delay=13.0,
    )

    return ld
