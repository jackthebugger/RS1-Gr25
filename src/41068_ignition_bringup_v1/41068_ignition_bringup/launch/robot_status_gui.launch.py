from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix, get_package_share_directory
import glob
import os
from typing import List, Optional


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


def _find_rs1_interfaces_prefix() -> Optional[str]:
    """Locate install/rs1_interfaces even when the parent shell forgot to source it."""
    try:
        return get_package_prefix('rs1_interfaces')
    except Exception:  # noqa: BLE001
        pass
    for root in os.environ.get('COLCON_PREFIX_PATH', '').split(os.pathsep):
        root = root.strip()
        if not root:
            continue
        cand = os.path.join(root, 'rs1_interfaces')
        if os.path.isdir(os.path.join(cand, 'share', 'rs1_interfaces')):
            return cand
    try:
        bringup = get_package_prefix('41068_ignition_bringup')
        cand = os.path.join(os.path.dirname(bringup), 'rs1_interfaces')
        if os.path.isdir(os.path.join(cand, 'share', 'rs1_interfaces')):
            return cand
    except Exception:  # noqa: BLE001
        pass
    return None


def _python_dist_packages(prefix: str) -> List[str]:
    patterns = [
        os.path.join(prefix, 'local', 'lib', 'python*', 'dist-packages'),
        os.path.join(prefix, 'lib', 'python*', 'site-packages'),
        os.path.join(prefix, 'lib', 'python*', 'dist-packages'),
    ]
    found: List[str] = []
    for pattern in patterns:
        found.extend(p for p in glob.glob(pattern) if os.path.isdir(p))
    return found


def _as_bool(raw: str) -> bool:
    return raw.strip().lower() in ('true', '1', 'yes', 'on')


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
    allow_ntp = _as_bool(
        LaunchConfiguration('allow_ntp_fallback').perform(context))
    start_wait = float(
        LaunchConfiguration('start_mission_wait_s').perform(context) or '30.0')
    # Default false: husky bringup already launches fire_detector when
    # fire_avoidance:=true. A second detector caused duplicate thermal pressure
    # during GUI BLOCK FIRE demos (gui_stall_fix G2).
    launch_fire = _as_bool(
        LaunchConfiguration('launch_fire_detector').perform(context))

    actions = []
    rs1_prefix = _find_rs1_interfaces_prefix()
    if rs1_prefix:
        ament = os.environ.get('AMENT_PREFIX_PATH', '')
        if rs1_prefix not in ament.split(os.pathsep):
            actions.append(SetEnvironmentVariable(
                name='AMENT_PREFIX_PATH',
                value=rs1_prefix + (os.pathsep + ament if ament else ''),
            ))
        py_extra = _python_dist_packages(rs1_prefix)
        if py_extra:
            pythonpath = os.environ.get('PYTHONPATH', '')
            prepend = os.pathsep.join(py_extra)
            # Avoid duplicating if already present.
            if not any(p in pythonpath.split(os.pathsep) for p in py_extra):
                actions.append(SetEnvironmentVariable(
                    name='PYTHONPATH',
                    value=prepend + (os.pathsep + pythonpath if pythonpath else ''),
                ))

    actions.append(
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
                'allow_ntp_fallback': allow_ntp,
                'start_mission_wait_s': start_wait,
            }],
        ),
    )
    if launch_fire:
        actions.append(
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
        )
    return actions


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
        DeclareLaunchArgument(
            'allow_ntp_fallback',
            default_value='false',
            description=(
                'If true, Start Mission may fall back to NavigateToPose when '
                'StartMission is missing (legacy path_bank:=false only).'
            ),
        ),
        DeclareLaunchArgument(
            'start_mission_wait_s',
            default_value='30.0',
            description='Wall-time seconds to wait for mission_owner/start_mission',
        ),
        DeclareLaunchArgument(
            'launch_fire_detector',
            default_value='false',
            description=(
                'If true, also launch beer_fire_detector. Default false because '
                '41068_ignition_husky.launch.py already starts one when '
                'fire_avoidance:=true.'
            ),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
