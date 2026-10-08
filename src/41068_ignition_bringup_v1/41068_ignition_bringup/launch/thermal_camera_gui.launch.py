from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot = LaunchConfiguration('robot')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='husky1'),
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        Node(
            package='41068_ignition_bringup',
            executable='thermal_camera_gui.py',
            namespace=robot,
            name='thermal_camera_gui',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'image_topic': '/husky1/thermal/image',
                'fire_topic': '/husky1/fire_detected',
                'temperature_topic': '/husky1/fire_temperature',
            }],
        ),
    ])
