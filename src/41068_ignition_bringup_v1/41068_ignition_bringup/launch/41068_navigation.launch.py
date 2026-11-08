import os
import tempfile
from typing import Text

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, Substitution
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
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


def _deep_merge_params(base, overlay):
    """Deep-merge overlay into base; plugin swaps replace the whole subtree.

    When both sides are dicts and the overlay declares ``plugin``, the overlay
    dict replaces ``base`` entirely so bake-offs (Smac2D / MPPI / constrained
    smoother) do not leave stale RPP/NavFn/SimpleSmoother leaf keys behind.
    Lists from the overlay replace lists from the base.
    """
    if not isinstance(base, dict) or not isinstance(overlay, dict):
        return overlay
    out = dict(base)
    for key, oval in overlay.items():
        if key not in out:
            out[key] = oval
            continue
        bval = out[key]
        if (
            isinstance(oval, dict)
            and isinstance(bval, dict)
            and 'plugin' in oval
        ):
            out[key] = oval
        elif isinstance(oval, dict) and isinstance(bval, dict):
            out[key] = _deep_merge_params(bval, oval)
        else:
            out[key] = oval
    return out


def _resolve_bake_overlay_paths(
    pkg_config_dir,
    planner_bake,
    controller_bake,
    smoother_bake,
    extra_overlays,
):
    """Map bake launch args → absolute overlay YAML paths (may be empty)."""
    presets = {
        'planner': {
            'smac2d': 'nav2_params_husky1_bake_smac2d.yaml',
        },
        'controller': {
            'mppi': 'nav2_params_husky1_bake_mppi.yaml',
        },
        'smoother': {
            'constrained': 'nav2_params_husky1_bake_constrained_smoother.yaml',
        },
    }
    paths = []

    def _add_preset(kind, token):
        token = (token or 'none').strip().lower()
        if token in ('', 'none', 'off', 'false', '0'):
            return
        name = presets.get(kind, {}).get(token)
        if name is None:
            raise ValueError(
                f'Unknown {kind}_bake={token!r}; '
                f'expected none|{"|".join(sorted(presets[kind]))}'
            )
        paths.append(os.path.join(pkg_config_dir, name))

    _add_preset('planner', planner_bake)
    _add_preset('controller', controller_bake)
    _add_preset('smoother', smoother_bake)

    for part in (extra_overlays or '').split(','):
        part = part.strip()
        if not part or part.lower() in ('none', 'off', 'false', '0'):
            continue
        if not os.path.isabs(part):
            part = os.path.join(pkg_config_dir, part)
        paths.append(part)
    return paths


class ConfiguredNav2Params(Substitution):
    """Rewrite Nav2 YAML: BT path, optional bake overlays, optional test_speed.

    Production defaults in nav2_params_husky1.yaml are NavFn+MPPI (Entry 054) and
    0.5 / ±0.6. Bake overlays (Smac2D / MPPI / ConstrainedSmoother) merge
    only when planner_bake / controller_bake / smoother_bake / nav2_params_overlay
    select them. When test_speed is true (bake/non-GUI only): raise MPPI
    FollowPath.vx_max / wz_max and smoother max/min[0] lockstep — NOT a GUI
    max-speed proof (Entry 063; historical RPP desired_linear_vel=1.0 obsolete).
    """

    def __init__(
        self,
        source_file,
        bt_xml,
        test_speed,
        *,
        pkg_config_dir,
        planner_bake,
        controller_bake,
        smoother_bake,
        nav2_params_overlay,
    ):
        super().__init__()
        self.__source_file = normalize_to_list_of_substitutions(source_file)
        self.__bt_xml = bt_xml
        self.__test_speed = normalize_to_list_of_substitutions(test_speed)
        self.__pkg_config_dir = pkg_config_dir
        self.__planner_bake = normalize_to_list_of_substitutions(planner_bake)
        self.__controller_bake = normalize_to_list_of_substitutions(
            controller_bake)
        self.__smoother_bake = normalize_to_list_of_substitutions(smoother_bake)
        self.__nav2_params_overlay = normalize_to_list_of_substitutions(
            nav2_params_overlay)

    def describe(self) -> Text:
        return 'ConfiguredNav2Params(bt + optional bake overlays + test_speed)'

    def perform(self, context) -> Text:
        yaml_filename = perform_substitutions(context, self.__source_file)
        test_speed = perform_substitutions(
            context, self.__test_speed).strip().lower() in (
            'true', '1', 'yes', 'on')
        planner_bake = perform_substitutions(context, self.__planner_bake)
        controller_bake = perform_substitutions(
            context, self.__controller_bake)
        smoother_bake = perform_substitutions(context, self.__smoother_bake)
        extra = perform_substitutions(context, self.__nav2_params_overlay)
        with open(yaml_filename, 'r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle)
        for overlay_path in _resolve_bake_overlay_paths(
            self.__pkg_config_dir,
            planner_bake,
            controller_bake,
            smoother_bake,
            extra,
        ):
            with open(overlay_path, 'r', encoding='utf-8') as handle:
                overlay = yaml.safe_load(handle) or {}
            data = _deep_merge_params(data, overlay)
        _set_yaml_leaf(data, 'default_nav_to_pose_bt_xml', self.__bt_xml)
        if test_speed:
            # Non-GUI bake only: lockstep MPPI + smoother (do not claim 1.0 under
            # MPPI as production max; Wave-1 validated envelope remains 0.50).
            _set_yaml_path(
                data,
                'controller_server.ros__parameters.FollowPath.vx_max',
                0.5,
            )
            _set_yaml_path(
                data,
                'controller_server.ros__parameters.FollowPath.wz_max',
                1.2,
            )
            _set_yaml_path(
                data,
                'controller_server.ros__parameters.FollowPath.vx_min',
                0.0,
            )
            _set_yaml_path(
                data, 'velocity_smoother.ros__parameters.max_velocity.0', 0.5)
            _set_yaml_path(
                data, 'velocity_smoother.ros__parameters.min_velocity.0', 0.0)
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

    obstacle_memory_launch_arg = DeclareLaunchArgument(
        'obstacle_memory',
        default_value='True',
        description=(
            'Launch obstacle_memory_manager + KeepoutFilter ingress. '
            'Set false for custom_world_1 forest_gap (prior Keepout seal).'
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
            'TEST ONLY (non-GUI bake): lockstep MPPI FollowPath.vx_max/wz_max + '
            'velocity_smoother to validated 0.50/1.2. NOT a GUI max-speed proof '
            '(Entry 063). Default false leaves production YAML unchanged.'
        ),
    )

    # Wave 2 bake-offs: optional overlays; default none keeps NavFn+RPP production.
    planner_bake_launch_arg = DeclareLaunchArgument(
        'planner_bake',
        default_value='none',
        description=(
            'Bake-off planner overlay: none (production NavFn) or smac2d '
            '(nav2_params_husky1_bake_smac2d.yaml). Does not mutate production yaml.'
        ),
    )
    controller_bake_launch_arg = DeclareLaunchArgument(
        'controller_bake',
        default_value='none',
        description=(
            'Bake-off controller overlay: none (production RPP) or mppi '
            '(nav2_params_husky1_bake_mppi.yaml DiffDrive). Default unchanged.'
        ),
    )
    smoother_bake_launch_arg = DeclareLaunchArgument(
        'smoother_bake',
        default_value='none',
        description=(
            'Bake-off smoother overlay: none (production SimpleSmoother) or '
            'constrained (ConstrainedSmoother). Default unchanged.'
        ),
    )
    nav2_params_overlay_launch_arg = DeclareLaunchArgument(
        'nav2_params_overlay',
        default_value='none',
        description=(
            'Optional comma-separated extra Nav2 YAML overlays (absolute paths '
            'or filenames under share/.../config/). Applied after planner/controller/'
            'smoother bake presets. Default none.'
        ),
    )

    mission_nav_authority_launch_arg = DeclareLaunchArgument(
        'mission_nav_authority',
        default_value='bank_follow_path',
        description=(
            'Phase 3 default bank_follow_path: split lifecycle in '
            'rs1_navigation + bank-owned FollowPath. Rollback '
            'legacy_navigate_to_pose: single lifecycle manager + NTP ownership. '
            'Cmd path is always smoother→cmd_vel_nav_smooth→rs1_cmd_vel_gate→cmd_vel.'
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
        pkg_config_dir=os.path.join(_pkg_share, 'config'),
        planner_bake=LaunchConfiguration('planner_bake'),
        controller_bake=LaunchConfiguration('controller_bake'),
        smoother_bake=LaunchConfiguration('smoother_bake'),
        nav2_params_overlay=LaunchConfiguration('nav2_params_overlay'),
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

    # Package-local rs1_navigation: Node-level smoother→cmd_vel_nav_smooth +
    # gate (SetRemap cannot override stock Node remaps). RewrittenYaml
    # root_key MUST be the robot namespace (same as PushRosNamespace) so
    # /husky1/controller_server loads FollowPath RPP — namespace:='' caused
    # DWB default + "No critics defined for FollowPath".
    navigation_group = GroupAction(scoped=True, condition=_if_true('nav2'), actions=[
        PushRosNamespace(namespace),
        SetRemap(src='/tf', dst='tf'),
        SetRemap(src='/tf_static', dst='tf_static'),
        SetRemap(src='/map', dst='map'),
        SetRemap(src='/map_updates', dst='map_updates'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    pkg_share,
                    'launch',
                    'rs1_navigation.launch.py',
                ])
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': configured_nav2_params,
                'namespace': namespace,
                'robot_namespace': namespace,
                'mission_nav_authority': LaunchConfiguration('mission_nav_authority'),
                'log_level': 'info',
            }.items()
        ),
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
    obstacle_memory = GroupAction(
        scoped=True,
        condition=_if_nav2_and('obstacle_memory'),
        actions=[
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
                # Entry 027: FIRE clear only inside thermal camera ROI frustum.
                'thermal_hfov_rad': 1.047,
                'thermal_vfov_rad': 0.810,
                'thermal_roi_width_frac': 0.6,
                'thermal_roi_height_frac': 0.7,
                'thermal_max_range': 8.0,
                'thermal_near_m': 0.1,
                # Entry 028: optical origin vs camera_link + TF age gate.
                'thermal_optical_forward_m': 0.45,
                'thermal_optical_up_m': 0.45,
                'thermal_tf_max_age_s': 0.5,
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
                # Entry 024: empty bank → home FollowPath/NTP (not indefinite halt).
                'auto_switch': True,
                'switch_resend_goal': ParameterValue(
                    LaunchConfiguration('path_bank_switch_resend'), value_type=bool
                ),
                'cancel_on_no_path': False,
                # Entry 024: bank exhaustion → return to mission start.
                'return_home_on_all_blocked': True,
                'home_yaw': 0.0,
                'validity_grace_s': 15.0,
                # Entry 023: settle after A→B before re-validating active path.
                'post_switch_grace_s': 8.0,
                # Phase 3: bank_follow_path (default) or legacy_navigate_to_pose.
                'mission_nav_authority': LaunchConfiguration('mission_nav_authority'),
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
                'camera_hfov_rad': 1.047,
            }],
            remappings=[
                ('/fire_detected', 'fire_detected'),
                ('/fire_temperature', 'fire_temperature'),
                ('/fire_bearing', 'fire_bearing'),
                ('/fire_observation', 'fire_observation'),
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
                'fire_bearing_topic': 'fire_bearing',
                'fire_observation_topic': 'fire_observation',
                'hazard_topic': 'fire_hazards',
                # No default world fires (SDF fires remain commented).
                # Entry 028: thermal marks need detector positive + hot-blob
                # bearing + LiDAR return along it (no default range, no hold).
                'hazard_radius': 0.9,
                'hazard_step': 0.25,
                'publish_rate_hz': 5.0,
                'thermal_max_range': 8.0,
                'thermal_min_range': 1.5,
                'camera_forward_m': 0.90,
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
    ld.add_action(obstacle_memory_launch_arg)
    ld.add_action(path_bank_launch_arg)
    ld.add_action(path_bank_switch_resend_launch_arg)
    ld.add_action(fire_avoidance_launch_arg)
    ld.add_action(test_speed_launch_arg)
    ld.add_action(planner_bake_launch_arg)
    ld.add_action(controller_bake_launch_arg)
    ld.add_action(smoother_bake_launch_arg)
    ld.add_action(nav2_params_overlay_launch_arg)
    ld.add_action(mission_nav_authority_launch_arg)
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
