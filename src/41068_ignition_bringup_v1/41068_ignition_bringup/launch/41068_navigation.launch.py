from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.substitutions import FindPackageShare


def _params_filename(prefix, suffix):
    return [
        TextSubstitution(text=prefix),
        suffix,
        TextSubstitution(text='.yaml')
    ]


_TRUE_STRINGS = "['true', '1', 'yes', 'on']"


def _is_true_expression(name):
    return ["'", LaunchConfiguration(name), "'.lower() in ", _TRUE_STRINGS]


def _if_true(name):
    return IfCondition(PythonExpression(_is_true_expression(name)))


def _if_slam_needed():
    return IfCondition(PythonExpression(
        _is_true_expression('slam') + [' or '] + _is_true_expression('nav2')
    ))


def _if_nav2_and(name):
    return IfCondition(PythonExpression(
        _is_true_expression('nav2') + [' and '] + _is_true_expression(name)
    ))


def generate_launch_description():

    ld = LaunchDescription()

    pkg_share = FindPackageShare('41068_ignition_bringup')
    config_path = PathJoinSubstitution([pkg_share, 'config'])

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time'
    )

    namespace = LaunchConfiguration('namespace')
    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace, e.g. husky1 or parrot1'
    )

    config_filename_suffix = LaunchConfiguration('config_filename_suffix')
    config_filename_suffix_launch_arg = DeclareLaunchArgument(
        'config_filename_suffix',
        default_value='',
        description='Robot filename suffix, e.g. _husky1 or _parrot1'
    )

    slam_launch_arg = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Flag to launch SLAM Toolbox'
    )

    nav2_launch_arg = DeclareLaunchArgument(
        'nav2',
        default_value='False',
        description='Flag to launch Nav2. Nav2 also starts SLAM.'
    )

    use_prior_map_launch_arg = DeclareLaunchArgument(
        'use_prior_map',
        default_value='False',
        description='Publish saved occupancy map on prior_map for StaticLayer'
    )

    fire_avoidance_launch_arg = DeclareLaunchArgument(
        'fire_avoidance',
        default_value='False',
        description='Launch fire detector + fire_hazard_nav_bridge'
    )

    slam_params_file = PathJoinSubstitution([
        config_path,
        _params_filename('slam_params', config_filename_suffix)
    ])

    nav2_params_file = PathJoinSubstitution([
        config_path,
        _params_filename('nav2_params', config_filename_suffix)
    ])

    prior_map_yaml = PathJoinSubstitution([
        pkg_share, 'maps', 'my_map.yaml'
    ])

    # Start Simultaneous Localisation and Mapping (SLAM).
    # Use the official slam_toolbox launch file, but wrap it in the robot
    # namespace and remap TF to relative names so it uses /<robot>/tf.
    slam = GroupAction(scoped=True, condition=_if_slam_needed(), actions=[
        PushRosNamespace(namespace),
        SetRemap(src='/tf', dst='tf'),
        SetRemap(src='/tf_static', dst='tf_static'),
        # Some packages publish/subscribe absolute /map topics even when
        # started under a namespace. Force them back to relative names so
        # they become /<robot>/map and /<robot>/map_updates.
        SetRemap(src='/map', dst='map'),
        SetRemap(src='/map_updates', dst='map_updates'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('slam_toolbox'),
                    'launch',
                    'online_async_launch.py'
                ])
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': slam_params_file,
                # Keep slam_toolbox lifecycle handling inside its own launch
                # file. This avoids mismatches between executable/node type
                # across ROS distributions.
                'use_lifecycle_manager': 'True',
                'autostart': 'True',
            }.items()
        )
    ])

    # Start Navigation Stack.
    # We push the namespace ourselves rather than relying on nav2_bringup's
    # use_namespace handling, which differs across distro/package versions.
    navigation_group = GroupAction(scoped=True, condition=_if_true('nav2'), actions=[
        PushRosNamespace(namespace),
        SetRemap(src='/tf', dst='tf'),
        SetRemap(src='/tf_static', dst='tf_static'),
        # Some packages publish/subscribe absolute /map topics even when
        # started under a namespace. Force them back to relative names so
        # they become /<robot>/map and /<robot>/map_updates.
        SetRemap(src='/map', dst='map'),
        SetRemap(src='/map_updates', dst='map_updates'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'navigation_launch.py'
                ])
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_params_file,
            }.items()
        )
    ])

    # Prior map for global StaticLayer (does not replace SLAM /map).
    prior_map = GroupAction(scoped=True, condition=_if_nav2_and('use_prior_map'), actions=[
        PushRosNamespace(namespace),
        SetRemap(src='/tf', dst='tf'),
        SetRemap(src='/tf_static', dst='tf_static'),
        Node(
            package='41068_ignition_bringup',
            executable='prior_map_publisher.py',
            name='prior_map_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_filename': prior_map_yaml,
                'topic_name': 'prior_map',
                'robot_name': namespace,
            }],
        ),
    ])

    # Thermal fire detection → PointCloud2 keep-outs on costmaps.
    # Known fire poses match active simple_fire includes in custom_world_1.
    fire_nav = GroupAction(scoped=True, condition=_if_nav2_and('fire_avoidance'), actions=[
        PushRosNamespace(namespace),
        SetRemap(src='/tf', dst='tf'),
        SetRemap(src='/tf_static', dst='tf_static'),
        Node(
            package='beer_fire_detection',
            executable='fire_detector',
            name='beer_fire_detector',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'thermal_topic': 'thermal/image',
                'fire_threshold_kelvin': 350.0,
                'minimum_hot_pixels': 10,
                'thermal_resolution': 0.01,
            }],
            remappings=[
                ('/fire_detected', 'fire_detected'),
                ('/fire_temperature', 'fire_temperature'),
            ],
        ),
        Node(
            package='41068_ignition_bringup',
            executable='fire_hazard_nav_bridge.py',
            name='fire_hazard_nav_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_name': namespace,
                'scan_topic': 'scan',
                'fire_detected_topic': 'fire_detected',
                'hazard_topic': 'fire_hazards',
                # No default world fires (SDF fires remain commented). Thermal
                # detections still mark a forward keep-out disc.
                'hazard_radius': 0.9,
                'hazard_step': 0.25,
                'publish_rate_hz': 5.0,
            }],
        ),
    ])

    ld.add_action(use_sim_time_launch_arg)
    ld.add_action(namespace_launch_arg)
    ld.add_action(config_filename_suffix_launch_arg)
    ld.add_action(slam_launch_arg)
    ld.add_action(nav2_launch_arg)
    ld.add_action(use_prior_map_launch_arg)
    ld.add_action(fire_avoidance_launch_arg)
    ld.add_action(slam)
    ld.add_action(navigation_group)
    ld.add_action(prior_map)
    ld.add_action(fire_nav)

    return ld
