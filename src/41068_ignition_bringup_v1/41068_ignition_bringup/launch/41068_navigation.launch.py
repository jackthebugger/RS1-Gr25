import os
import tempfile
from typing import Text

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, Substitution
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch.conditions import IfCondition
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
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


def _set_yaml_leaf(data, key, value):
    """Set every non-container leaf named ``key`` (RewrittenYaml leaf style)."""
    if isinstance(data, dict):
        for k, v in data.items():
            if k == key and not isinstance(v, (dict, list)):
                data[k] = value
            else:
                _set_yaml_leaf(v, key, value)
    elif isinstance(data, list):
        for item in data:
            _set_yaml_leaf(item, key, value)


def _set_yaml_path(data, dotted_path, value):
    """Set a dotted path; list indices are numeric path segments.

    Humble RewrittenYaml.updateYamlPathVals fails on list *leaf* indices
    (assigns with a string key). This helper is used for test_speed velocity
    list[0] rewrites without a second permanent YAML.
    """
    keys = dotted_path.split('.')
    cur = data
    for key in keys[:-1]:
        if isinstance(cur, list):
            cur = cur[int(key)]
        else:
            cur = cur[key]
    last = keys[-1]
    if isinstance(cur, list):
        cur[int(last)] = value
    else:
        cur[last] = value


class ConfiguredNav2Params(Substitution):
    """Rewrite Nav2 YAML: BT path always; test_speed velocities when enabled.

    Production defaults in nav2_params_husky1.yaml stay 0.5 / ±0.6. When
    test_speed is true: desired_linear_vel=0.9 and smoother max/min[0]=±1.0.
    """

    def __init__(self, source_file, bt_xml, test_speed):
        super().__init__()
        self.__source_file = normalize_to_list_of_substitutions(source_file)
        self.__bt_xml = bt_xml
        self.__test_speed = normalize_to_list_of_substitutions(test_speed)

    def describe(self) -> Text:
        return 'ConfiguredNav2Params(bt + optional test_speed)'

    def perform(self, context) -> Text:
        yaml_filename = perform_substitutions(context, self.__source_file)
        test_speed = perform_substitutions(
            context, self.__test_speed).strip().lower() in (
            'true', '1', 'yes', 'on')
        with open(yaml_filename, 'r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle)
        _set_yaml_leaf(data, 'default_nav_to_pose_bt_xml', self.__bt_xml)
        if test_speed:
            _set_yaml_leaf(data, 'desired_linear_vel', 0.9)
            _set_yaml_path(
                data, 'velocity_smoother.ros__parameters.max_velocity.0', 1.0)
            _set_yaml_path(
                data, 'velocity_smoother.ros__parameters.min_velocity.0', -1.0)
        out = tempfile.NamedTemporaryFile(
            mode='w', delete=False, suffix='.yaml', encoding='utf-8')
        yaml.dump(data, out)
        out.close()
        return out.name


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

    prior_map_file_launch_arg = DeclareLaunchArgument(
        'prior_map_file',
        default_value='bush_trail_world.yaml',
        description=(
            'Map YAML under share/41068_ignition_bringup/maps/ used by '
            'prior_map_publisher (StaticLayer baseline + path bank)'
        ),
    )

    path_bank_launch_arg = DeclareLaunchArgument(
        'path_bank',
        default_value='True',
        description='Launch path_bank_manager for alternative-route selection'
    )

    path_bank_switch_resend_launch_arg = DeclareLaunchArgument(
        'path_bank_switch_resend',
        default_value='True',
        description=(
            'When true, path_bank_manager preempts NavigateToPose onto the next '
            'valid banked route after live lethal invalidation (Entry 019+)'
        ),
    )

    fire_avoidance_launch_arg = DeclareLaunchArgument(
        'fire_avoidance',
        default_value='False',
        description='Launch fire detector + fire_hazard_nav_bridge'
    )

    test_speed_launch_arg = DeclareLaunchArgument(
        'test_speed',
        default_value='false',
        description=(
            'TEST ONLY: raise RPP desired_linear_vel to 0.9 and velocity_smoother '
            'max/min_velocity[0] to ±1.0 via params rewrite. Default false leaves '
            'production 0.5 / ±0.6 unchanged in nav2_params_*.yaml.'
        ),
    )

    slam_params_file = PathJoinSubstitution([
        config_path,
        _params_filename('slam_params', config_filename_suffix)
    ])

    nav2_params_file = PathJoinSubstitution([
        config_path,
        _params_filename('nav2_params', config_filename_suffix)
    ])

    # Prefer the package-local BT so route-stable replanning ships with the
    # bringup package. ConfiguredNav2Params substitutes into the params file
    # before nav2_bringup applies its own namespace rewrite.
    # Note: Humble RewrittenYaml cannot rewrite list leaf indices (e.g.
    # max_velocity.0); ConfiguredNav2Params handles that for test_speed.
    _pkg_share = get_package_share_directory('41068_ignition_bringup')
    _route_stable_bt = os.path.join(
        _pkg_share,
        'behavior_trees',
        'navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml',
    )
    if not os.path.isfile(_route_stable_bt):
        _route_stable_bt = (
            '/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/'
            'navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml'
        )
    configured_nav2_params = ConfiguredNav2Params(
        source_file=nav2_params_file,
        bt_xml=_route_stable_bt,
        test_speed=LaunchConfiguration('test_speed'),
    )

    prior_map_yaml = PathJoinSubstitution([
        pkg_share, 'maps', LaunchConfiguration('prior_map_file')
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
                'params_file': configured_nav2_params,
            }.items()
        )
    ])

    # Prior map for global StaticLayer (does not replace SLAM /map).
    # Must start with Nav2 so TransientLocal /prior_map is available before
    # StaticLayer initialises (Entry 003 hang was "no map received").
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

    # Sensor-only obstacle memory → KeepoutFilter mask on global costmap.
    # Must start with Nav2 so TransientLocal filter_info+mask exist before
    # KeepoutFilter initialises (Entry 021).
    obstacle_memory = GroupAction(scoped=True, condition=_if_true('nav2'), actions=[
        PushRosNamespace(namespace),
        SetRemap(src='/tf', dst='tf'),
        SetRemap(src='/tf_static', dst='tf_static'),
        Node(
            package='41068_ignition_bringup',
            executable='obstacle_memory_manager.py',
            name='obstacle_memory_manager',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_name': namespace,
                'enabled': True,
                'publish_hz': 2.0,
                # Publish-on-change only; rare filter_info refresh rehydrates
                # KeepoutFilter after resetLayers (see Entry 023 research).
                'mask_force_republish_s': 0.0,
                'filter_info_refresh_s': 20.0,
                'mask_min_publish_period_s': 1.5,
                # Neighbour=1 helps thin barriers register. Mark range must
                # reach as far as ObstacleLayer inflation can stop the robot
                # (else memory never sees the hit — T2 regression at 8 m).
                'mark_neighbour_radius_cells': 1,
                'prior_edge_margin_cells': 4,
                'obstacle_max_range_m': 12.0,
                'clear_max_range_m': 10.0,
                'raytrace_max_m': 12.0,
                # Entry 023: never seal Keepout under the Husky footprint.
                'mark_exclude_radius_m': 0.65,
                'fire_hold_s': 8.0,
            }],
        ),
    ])

    # Path bank: ranked alternative routes; switches only when live costmap
    # invalidates the active candidate (does not replan at 1 Hz).
    path_bank = GroupAction(scoped=True, condition=_if_nav2_and('path_bank'), actions=[
        PushRosNamespace(namespace),
        SetRemap(src='/tf', dst='tf'),
        SetRemap(src='/tf_static', dst='tf_static'),
        Node(
            package='41068_ignition_bringup',
            executable='path_bank_manager.py',
            name='path_bank_manager',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_name': namespace,
                'enabled': True,
                'max_candidates': 4,
                'exclusion_radius_m': 2.5,
                'min_path_separation_m': 2.0,
                # Monitor + rank; resend mission goal on switch when enabled.
                # Entry 024: empty bank → NavigateToPose home (not indefinite halt).
                'auto_switch': True,
                'switch_resend_goal': ParameterValue(
                    LaunchConfiguration('path_bank_switch_resend'), value_type=bool
                ),
                'cancel_on_no_path': False,
                # Entry 024: bank exhaustion → NavigateToPose back to mission start.
                'return_home_on_all_blocked': True,
                'home_yaw': 0.0,
                'validity_grace_s': 15.0,
                # Entry 023: settle after A→B before re-validating active path.
                'post_switch_grace_s': 8.0,
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
    ld.add_action(prior_map_file_launch_arg)
    ld.add_action(path_bank_launch_arg)
    ld.add_action(path_bank_switch_resend_launch_arg)
    ld.add_action(fire_avoidance_launch_arg)
    ld.add_action(test_speed_launch_arg)
    # prior_map before navigation_group so latched OccupancyGrid exists when
    # StaticLayer starts (avoids Entry 003 "no map received" hang).
    # obstacle_memory before/with nav so KeepoutFilter can latch filter_info.
    ld.add_action(prior_map)
    ld.add_action(obstacle_memory)
    ld.add_action(slam)
    ld.add_action(navigation_group)
    ld.add_action(path_bank)
    ld.add_action(fire_nav)

    return ld
