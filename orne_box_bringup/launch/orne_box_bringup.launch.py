import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    bringup_dir = get_package_share_directory('orne_box_bringup')
    ypspur_param = os.path.join(bringup_dir, 'config/ypspur', 'box_v3.param')
    ypspur_coordinator_path = os.path.join(bringup_dir, 'scripts', 'ypspur_coordinator_bridge')
    include_dir = os.path.join(bringup_dir, 'launch/include')
    localization = LaunchConfiguration('localization')
    # emcl2 のときだけ rfans の /scan と robot_localization EKF (odom->base_link) を起動する
    use_emcl2 = IfCondition(PythonExpression(["'", localization, "' == 'emcl2'"]))

    # Nodes
    icart_driver_node = Node(
        package='orne_box_bringup',
        executable='icart_mini_driver',
        parameters=[{
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_link',
            'Hz': 40,
            'left_wheel_joint': 'left_wheel_joint',
            'right_wheel_joint': 'right_wheel_joint',
            'liner_vel_lim': 1.5,
            'liner_accel_lim': 1.5,
            'angular_vel_lim': 3.14,
            'angular_accel_lim': 3.14,
            'calculate_odom_from_ypspur': True,
            'publish_odom_tf': False
        }]
    )

    # Launch Description
    return LaunchDescription([
        DeclareLaunchArgument(
            'localization', default_value='ndt',
            description='Localization method: ndt or emcl2'),
        LogInfo(msg="Launch ypspur coordinator."),
        ExecuteProcess(cmd=[ypspur_coordinator_path, ypspur_param], shell=True),
        LogInfo(msg="Launch icart_mini_driver node."),
        icart_driver_node,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([include_dir, '/description.launch.py'])
        ),
        LogInfo(msg="Launch robot description nodes."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([include_dir, '/teleop.launch.py'])
        ),
        LogInfo(msg="Launch teleop node."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([include_dir, '/adis16465.launch.py'])
        ),
        LogInfo(msg="Launch IMU node."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([include_dir, '/rfans16_filters.launch.py'])
        ),
        LogInfo(msg="Launch rfans16_filters node."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([include_dir, '/pointcloud_to_laserscan.launch.py'])
        ),
        LogInfo(msg="Launch pointcloud_to_laserscan node."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([include_dir, '/rfans_scan.launch.py']),
            condition=use_emcl2
        ),
        LogInfo(msg="Launch rfans_scan node.", condition=use_emcl2),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([include_dir, '/robot_localization_ekf.launch.py']),
            condition=use_emcl2
        ),
        LogInfo(msg="Launch robot_localization_ekf node.", condition=use_emcl2),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(include_dir, 'MID360.launch.py')
            )
        ),
        LogInfo(msg="Launch MID360 driver.")
    ])
