import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # パッケージパスの取得
    slam_toolbox_dir = FindPackageShare('slam_toolbox')
    orne_box_bringup_dir = FindPackageShare('orne_box_bringup')

    # ローカライゼーション用のmapをロード（オプション）
    use_lifecycle_manager = LaunchConfiguration('use_lifecycle_manager', default='True')
    slam_params_file = LaunchConfiguration(
        'slam_params_file',
        default=PathJoinSubstitution(
            [orne_box_bringup_dir, 'config', 'slam_toolbox', 'mapper_params_online_async.yaml']
        ),
    )

    declare_use_lifecycle_manager_cmd = DeclareLaunchArgument(
        'use_lifecycle_manager',
        default_value='True',
        description='Use lifecycle manager',
    )

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=slam_params_file,
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node',
    )

    # SLAM Toolbox async mapper
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file],
        remappings=[],
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_lifecycle_manager_cmd)
    ld.add_action(declare_slam_params_file_cmd)

    # Add nodes
    ld.add_action(slam_toolbox_node)

    return ld
