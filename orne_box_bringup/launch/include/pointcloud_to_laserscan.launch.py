import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription  # ←追加
from launch.launch_description_sources import PythonLaunchDescriptionSource  # ←追加
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time',
    )

    pointcloud_to_laserscan_node = launch_ros.actions.Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', 'surestar_points'),
            # ('cloud_in', 'rfans_points'),
            ('scan', 'surestar_scan'), # scan
            #('scan', 'scan'), 
        ],
        parameters=[{
                'target_frame': '',
                'transform_tolerance': 0.1,
                'min_height': 0.1,
                'max_height': 5.0,
                'angle_min': -3.1415,  # -M_PI/2
                'angle_max': 3.1415,  # M_PI/2
                'angle_increment': 0.0261, #0.0087,  # M_PI/360.0
                'scan_time': 0.1, #0.3333,
                'range_min': 0.0,
                'range_max': 100.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'use_sim_time': use_sim_time,
            }]
    )

    laser_filters_node = launch_ros.actions.Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[
            {'use_sim_time': use_sim_time},
            PathJoinSubstitution([
                get_package_share_directory("orne_box_bringup"),
                "config", "box_filter_box3.yaml",
            ])],
        remappings=[
            ('scan_filtered', 'scan'),
            ('scan', 'surestar_scan'),
        ],    
    )

    ground_segmentation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('ground_segmentation_ros2'),
            '/launch/ground_segmentation.launch.py'
        ]),
        launch_arguments={
            'pointcloud_topic': '/surestar_points',
            'use_sim_time': "true",
            'params_file': os.path.join(get_package_share_directory('ground_segmentation_ros2'),'config', 'parameters.yaml'),
        }.items()
    )

    low_pointcloud_to_laserscan_node = launch_ros.actions.Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/ground_segmentation/obstacle_points'), #ground_segmentation_output
            ('scan', 'low_surestar_scan'), # scan
        ],
        parameters=[{
                'target_frame': '',
                'transform_tolerance': 0.1,
                'min_height': -1.0, #-0.4 
                'max_height': 0.5,
                'angle_min': -3.1415,  # -M_PI/2
                'angle_max': 3.1415,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.0,
                'range_max': 100.0,
                'use_inf': True,
                'use_sim_time': False,
                'inf_epsilon': 1.0
            }]
    )

    low_laser_filters_node = launch_ros.actions.Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[
            {'use_sim_time': False},
            PathJoinSubstitution([
                get_package_share_directory("orne_box_bringup"),
                "config", "box_filter_box3.yaml",
            ])],
        remappings=[
            ('scan_filtered', 'low_scan'),
            ('scan', 'low_surestar_scan'),
        ],
    )

    return launch.LaunchDescription([
        declare_use_sim_time_cmd,
        pointcloud_to_laserscan_node,
        laser_filters_node,
        ground_segmentation_launch,  # ←追加
        low_pointcloud_to_laserscan_node,
        low_laser_filters_node
    ])

if __name__ == '__main__':
    generate_launch_description()
