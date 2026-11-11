from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    next_wp_button = LaunchConfiguration('next_wp_button')
    next_wp_service = LaunchConfiguration('next_wp_service')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'),
        DeclareLaunchArgument(
            'next_wp_button',
            default_value='1',
            description='Joy button index that calls next_wp'),
        DeclareLaunchArgument(
            'next_wp_service',
            default_value='/waypoint_sender_node/next_wp',
            description='next_wp service to call'),
        Node(
            package='joy_command',
            executable='joy_command_node',
            name='joy_command_node',
            parameters=[{
                'use_sim_time': use_sim_time,
                'next_wp_button': next_wp_button,
                'next_wp_service': next_wp_service,
            }],
            output='screen',
        )
    ])
