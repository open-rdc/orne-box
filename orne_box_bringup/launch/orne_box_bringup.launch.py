import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess, LogInfo
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    bringup_dir = get_package_share_directory('orne_box_bringup')
    ypspur_param = os.path.join(bringup_dir, 'config/ypspur', 'box_v3.param')
    ypspur_coordinator_path = os.path.join(bringup_dir, 'scripts', 'ypspur_coordinator_bridge')
    include_dir = os.path.join(bringup_dir, 'launch/include')

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
        LogInfo(msg="Launch ypspur coordinator."),
        ExecuteProcess(cmd=[ypspur_coordinator_path, ypspur_param], shell=True),
        LogInfo(msg="Launch icart_mini_driver node."),
        icart_driver_node,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([include_dir, '/description.launch.py'])
        ),
        LogInfo(msg="Launch robot description nodes."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([include_dir, '/robot_localization_ekf.launch.py'])
        ),
        LogInfo(msg="Launch robot_localization_ekf node."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([include_dir, '/teleop.launch.py'])
        ),
        LogInfo(msg="Launch teleop node."),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([include_dir, '/urg_node2.launch.py'])
        # ),
        # LogInfo(msg="Launch URG node."),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([include_dir, '/mirror_lidar.launch.py'])
        # ), 
        # LogInfo(msg="Launch mirror_lidar node."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([include_dir, '/adis16465.launch.py'])
        ),
        LogInfo(msg="Launch IMU node."),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([include_dir, '/rfans16.launch.py'])
        # ),
        # LogInfo(msg="Launch rfans16 node."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([include_dir, '/rfans16_filters.launch.py'])
        ),
        LogInfo(msg="Launch rfans16_filters node."),
    ])
