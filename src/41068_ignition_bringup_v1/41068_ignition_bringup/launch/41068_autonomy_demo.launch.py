from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    robot = LaunchConfiguration('robot')
    use_sim_time = LaunchConfiguration('use_sim_time')
    show_gui = LaunchConfiguration('show_gui')
    mission_mode = LaunchConfiguration('mission_mode')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_yaw = LaunchConfiguration('goal_yaw')
    world = LaunchConfiguration('world')

    ld.add_action(DeclareLaunchArgument(
        'robot',
        default_value='husky1',
        choices=['husky1', 'parrot1'],
        description='Robot namespace to control. Use husky1 or parrot1.',
    ))
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time',
    ))
    ld.add_action(DeclareLaunchArgument(
        'show_gui',
        default_value='True',
        description='Whether to launch the status GUI panel for the chosen robot.',
    ))
    ld.add_action(DeclareLaunchArgument(
        'mission_mode',
        default_value='single_goal',
        choices=['single_goal', 'replan', 'random_walk'],
        description='single_goal, replan (inject obstacle), or original random_walk',
    ))
    ld.add_action(DeclareLaunchArgument(
        'goal_x', default_value='0.0',
        description='Goal X in the robot map frame, metres',
    ))
    ld.add_action(DeclareLaunchArgument(
        'goal_y', default_value='-5.0',
        description='Goal Y in the robot map frame, metres',
    ))
    ld.add_action(DeclareLaunchArgument(
        'goal_yaw', default_value='0.0',
        description='Goal yaw in the robot map frame, radians',
    ))
    ld.add_action(DeclareLaunchArgument(
        'world', default_value='simple_trees',
        description='Gazebo world name (used by replan mode for obstacle insertion)',
    ))

    # This launch file intentionally does not start Gazebo, robots, SLAM,
    # Nav2, or RViz. Start the normal simulation first, then run this launch
    # file from a separate terminal. --attach tells the demo not to spawn a
    # second simulation.
    ld.add_action(Node(
        package='41068_ignition_bringup',
        executable='basic_autonomy_demo.py',
        namespace=robot,
        name='basic_autonomy_demo',
        output='screen',
        arguments=[
            '--attach',
            '--mode', mission_mode,
            '--world', world,
            '--robot', robot,
            '--goal', goal_x, goal_y, goal_yaw,
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_name': robot,
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    ))

    ld.add_action(Node(
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
            'obstacle_threshold': 1.0,
        }],
        condition=IfCondition(show_gui),
    ))

    return ld
