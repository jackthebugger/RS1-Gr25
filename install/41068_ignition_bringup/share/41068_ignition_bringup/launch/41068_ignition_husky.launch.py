from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    slam = LaunchConfiguration('slam')
    nav2 = LaunchConfiguration('nav2')
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time',
    ))
    ld.add_action(DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Flag to launch RViz',
    ))
    ld.add_action(DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Flag to launch SLAM Toolbox',
    ))
    ld.add_action(DeclareLaunchArgument(
        'nav2',
        default_value='False',
        description='Flag to launch Nav2. Nav2 also starts SLAM.',
    ))
    ld.add_action(DeclareLaunchArgument(
        'world',
        default_value='simple_trees',
        description='Which world to load',
        choices=['simple_trees', 'large_demo', 'custom_world_1'],
    ))
    ld.add_action(DeclareLaunchArgument(
        'gui',
        default_value='True',
        description='Launch the Gazebo GUI. Set false for headless (useful on WSL).',
    ))

    # Starting position of the Husky, plus sensor tuning. Change these to move
    # the start pose without editing any launch or Python implementation file.
    passthrough_args = (
        ('husky_x', '0.0', 'Husky spawn X position in metres'),
        ('husky_y', '0.0', 'Husky spawn Y position in metres'),
        ('husky_z', '0.4', 'Husky spawn Z position in metres'),
        ('husky_yaw', '0.0', 'Husky spawn yaw in radians'),
        ('enable_camera', 'false',
         'Enable the Husky RGB-D camera. Known to stall Ignition on WSL/software GL; '
         'not used by SLAM or Nav2.'),
        ('lidar_update_rate', '10', 'Husky lidar update rate in Hz'),
        ('lidar_samples', '360', 'Husky lidar horizontal sample count'),
        ('camera_update_rate', '2', 'Husky RGB-D camera update rate in Hz'),
        ('camera_width', '320', 'Husky RGB-D camera image width in pixels'),
        ('camera_height', '240', 'Husky RGB-D camera image height in pixels'),
        ('drive_plugin', 'diff_drive',
         'Gazebo system used to drive the base: diff_drive or velocity_control'),
        ('effective_wheel_separation', '0.94',
         'Wheel separation used by DiffDrive, in metres (calibrated for skid-steer slip)'),
        ('nav_start_delay', '15.0',
         'Seconds after launch before starting SLAM/Nav2'),
    )
    for name, default, description in passthrough_args:
        ld.add_action(DeclareLaunchArgument(name, default_value=default, description=description))

    forwarded = {
        'use_sim_time': use_sim_time,
        'rviz': rviz,
        'slam': slam,
        'nav2': nav2,
        'world': world,
        'gui': gui,
        'husky': 'True',
        'parrot': 'False',
    }
    forwarded.update({name: LaunchConfiguration(name) for name, _d, _desc in passthrough_args})

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('41068_ignition_bringup'),
            'launch',
            '41068_ignition.launch.py',
        ])),
        launch_arguments=forwarded.items(),
    ))

    return ld
