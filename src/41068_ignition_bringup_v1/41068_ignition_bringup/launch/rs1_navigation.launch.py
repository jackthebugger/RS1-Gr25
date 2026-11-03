# Copyright (c) 2018 Intel Corporation
# Adapted for RS1: velocity_smoother → cmd_vel_nav_smooth → rs1_cmd_vel_gate → cmd_vel;
# optional split lifecycle (nav vs bt) per mission_nav_authority.
#
# Licensed under the Apache License, Version 2.0.

"""Package-local Nav2 bringup with cmd_vel gate wiring.

Mirrors stock Humble ``nav2_bringup/navigation_launch.py`` param loading
(RewrittenYaml + ParameterFile + convert_types) so FollowPath/RPP plugin
params are not emptied. Differences vs stock:

* smoother ``cmd_vel_smoothed`` → ``cmd_vel_nav_smooth`` (not ``cmd_vel``)
* ``rs1_cmd_vel_gate`` is the sole publisher of ``cmd_vel``
* ``mission_nav_authority=bank_follow_path``: split lifecycle managers;
  BT starts *after* nav servers (avoids compute_path_* wait race), then
  ManageLifecycleNodes PAUSE → INACTIVE (Humble STARTUP→PAUSE)
* ``mission_nav_authority=legacy_navigate_to_pose``: single manager + BT active

Parent must PushRosNamespace and pass ``namespace:=''`` so RewrittenYaml
root_key stays empty (double-namespace was a prior "No critics" failure mode).
Pass ``robot_namespace`` (e.g. husky1) for absolute ManageLifecycleNodes calls.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')

    namespace = LaunchConfiguration('namespace')
    robot_namespace = LaunchConfiguration('robot_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    # Hardcode log level in Node argv — LaunchConfiguration('log_level') inside
    # a scoped IncludeLaunchDescription previously raised "does not exist" and
    # correlated with empty FollowPath plugin params (No critics).
    log_level = 'info'
    mission_nav_authority = LaunchConfiguration('mission_nav_authority')

    is_legacy = PythonExpression([
        "'", mission_nav_authority, "' == 'legacy_navigate_to_pose'"
    ])
    is_split = PythonExpression([
        "'", mission_nav_authority, "' != 'legacy_navigate_to_pose'"
    ])

    lifecycle_nodes_all = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]
    lifecycle_nodes_nav = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'waypoint_follower',
        'velocity_smoother',
    ]
    lifecycle_nodes_bt = ['bt_navigator']

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Stock remaps smoother cmd_vel_smoothed→cmd_vel; we send it to the gate.
    smoother_remappings = remappings + [
        ('cmd_vel', 'cmd_vel_nav'),
        ('cmd_vel_smoothed', 'cmd_vel_nav_smooth'),
    ]

    # Identical to stock nav2_bringup: RewrittenYaml + convert_types.
    # Parent PushRosNamespace + namespace:='' keeps root_key empty.
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
    }
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='RewrittenYaml root_key; keep empty when parent PushRosNamespace')

    declare_robot_namespace_cmd = DeclareLaunchArgument(
        'robot_namespace',
        default_value='',
        description='Absolute ROS namespace for ManageLifecycleNodes (e.g. husky1)')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description=(
            'Full path to the ROS2 parameters file for all nav nodes. '
            'Parent 41068_navigation.launch.py may already merge Wave 2 bake '
            'overlays (planner_bake/controller_bake/smoother_bake) into a '
            'temp file before passing it here; default production remains '
            'NavFn+RPP when those args are none.'
        ))

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Use composed bringup if True')

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='nav2_container',
        description='Container name when use_composition is True')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes (non-composition)')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    declare_mission_nav_authority_cmd = DeclareLaunchArgument(
        'mission_nav_authority',
        default_value='bank_follow_path',
        description=(
            'bank_follow_path: split lifecycle, BT STARTUP then PAUSE. '
            'legacy_navigate_to_pose: single manager, BT stays active.'
        ),
    )

    # Server nodes (stock order). Gate is always present.
    def _server_nodes():
        return [
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                # Spin/backup must feed the gate via cmd_vel_nav, not raw cmd_vel.
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=smoother_remappings),
            Node(
                package='41068_ignition_bringup',
                executable='rs1_cmd_vel_gate.py',
                name='rs1_cmd_vel_gate',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'nav_topic': 'cmd_vel_nav_smooth',
                    'teleop_topic': 'cmd_vel_teleop',
                    'authority_topic': 'mission_owner/command_authority',
                    'out_topic': 'cmd_vel',
                }],
            ),
        ]

    legacy_lifecycle = Node(
        condition=IfCondition(is_legacy),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes_all}],
    )

    split_lifecycle_nav = Node(
        condition=IfCondition(is_split),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes_nav}],
    )

    # Delay BT manager until nav servers are ACTIVE. TimerAction drops
    # PushRosNamespace for delayed Nodes — pass namespace= explicitly.
    def _split_bt_lifecycle(context, *args, **kwargs):
        auth = LaunchConfiguration('mission_nav_authority').perform(context).strip()
        if auth == 'legacy_navigate_to_pose':
            return []
        ns = LaunchConfiguration('robot_namespace').perform(context).strip()
        use_sim = LaunchConfiguration('use_sim_time').perform(context)
        bt_mgr = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_bt',
            namespace=ns,
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{'use_sim_time': use_sim in ('true', 'True', '1')},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes_bt},
                        {'attempt_respawn_reconnection': False}],
        )
        if ns:
            svc = f'/{ns}/lifecycle_manager_bt/manage_nodes'
        else:
            svc = '/lifecycle_manager_bt/manage_nodes'
        pause_script = (
            f'for i in $(seq 1 40); do '
            f'  ros2 service list 2>/dev/null | grep -qx "{svc}" && break; '
            f'  sleep 0.5; done; '
            f'ros2 service call "{svc}" nav2_msgs/srv/ManageLifecycleNodes "{{command: 1}}"; '
            f'echo "rs1: BT PAUSE via {svc} exit=$?"'
        )
        return [
            TimerAction(period=8.0, actions=[bt_mgr]),
            TimerAction(
                period=16.0,
                actions=[ExecuteProcess(
                    cmd=['bash', '-c', pause_script], output='screen')],
            ),
        ]

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=_server_nodes() + [
            legacy_lifecycle,
            split_lifecycle_nav,
        ],
    )

    load_composable_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_controller',
                        plugin='nav2_controller::ControllerServer',
                        name='controller_server',
                        parameters=[configured_params],
                        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
                    ComposableNode(
                        package='nav2_smoother',
                        plugin='nav2_smoother::SmootherServer',
                        name='smoother_server',
                        parameters=[configured_params],
                        remappings=remappings),
                    ComposableNode(
                        package='nav2_planner',
                        plugin='nav2_planner::PlannerServer',
                        name='planner_server',
                        parameters=[configured_params],
                        remappings=remappings),
                    ComposableNode(
                        package='nav2_behaviors',
                        plugin='behavior_server::BehaviorServer',
                        name='behavior_server',
                        parameters=[configured_params],
                        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
                    ComposableNode(
                        package='nav2_bt_navigator',
                        plugin='nav2_bt_navigator::BtNavigator',
                        name='bt_navigator',
                        parameters=[configured_params],
                        remappings=remappings),
                    ComposableNode(
                        package='nav2_waypoint_follower',
                        plugin='nav2_waypoint_follower::WaypointFollower',
                        name='waypoint_follower',
                        parameters=[configured_params],
                        remappings=remappings),
                    ComposableNode(
                        package='nav2_velocity_smoother',
                        plugin='nav2_velocity_smoother::VelocitySmoother',
                        name='velocity_smoother',
                        parameters=[configured_params],
                        remappings=smoother_remappings),
                ],
            ),
            Node(
                package='41068_ignition_bringup',
                executable='rs1_cmd_vel_gate.py',
                name='rs1_cmd_vel_gate',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'nav_topic': 'cmd_vel_nav_smooth',
                    'teleop_topic': 'cmd_vel_teleop',
                    'authority_topic': 'mission_owner/command_authority',
                    'out_topic': 'cmd_vel',
                }],
            ),
            Node(
                condition=IfCondition(is_legacy),
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time,
                             'autostart': autostart,
                             'node_names': lifecycle_nodes_all}],
            ),
            Node(
                condition=IfCondition(is_split),
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time,
                             'autostart': autostart,
                             'node_names': lifecycle_nodes_nav}],
            ),
        ],
    )

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_robot_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_mission_nav_authority_cmd)
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)
    ld.add_action(OpaqueFunction(function=_split_bt_lifecycle))
    return ld
