"""Start MID360 with sensor_msgs/msg/PointCloud2 output and convert it to /low_scan."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    livox_dir = get_package_share_directory('livox_ros_driver2')

    return LaunchDescription([
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            output='screen',
            parameters=[{
                'xfer_format': 0,
                'multi_topic': 0,
                'data_src': 0,
                'publish_freq': 10.0,
                'output_data_type': 0,
                'frame_id': 'livox_frame',
                'lvx_file_path': '/home/livox/livox_test.lvx',
                'user_config_path': os.path.join(
                    livox_dir, 'config', 'MID360_config.json'
                ),
                'cmdline_input_bd_code': 'livox0000000001',
            }],
        ),
        # /livox/lidar -> /low_scan (costmap, collision_monitor 用)
        # 出力は best_effort。/low_scan の利用者 (costmap, collision_monitor, RViz) は
        # best_effort で購読するので、laser_filters による QoS 変換は挟まない
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='livox_pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', 'livox/lidar'),
                ('scan', 'low_scan'),
            ],
            parameters=[{
                'target_frame': '',
                'transform_tolerance': 0.1,
                'min_height': -0.4,
                'max_height': 0.0,
                'angle_min': -3.1415,  # -M_PI/2
                'angle_max': 0.0,  # M_PI/2
                'angle_increment': 0.00087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.0,
                'range_max': 200.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
            }],
        ),
    ])
