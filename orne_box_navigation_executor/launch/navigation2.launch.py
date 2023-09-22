import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # <path>
    pkg_dir = get_package_share_directory('orne_box_navigation_executor')
    nav2_launch_file_dir = os.path.join(pkg_dir, 'launch')
    config_dir = os.path.join(pkg_dir, 'config')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            config_dir,
            'maps',
            'cit_3f_map.yaml'))

    param_file_name = 'nav2_params.yaml'
    bt_file_name ='navigate_w_replanning_and_wait.xml'
    # bt_file_name ='navigate_w_replanning_and_recovery.xml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            config_dir,
            'params',
            param_file_name))
    bt_dir = LaunchConfiguration(
        'default_bt_xml_filename',
        default=os.path.join(
            config_dir,
            'behavior_trees',
            bt_file_name))
    rviz_config_dir = os.path.join(
        config_dir,
        'rviz',
        'nav2_default_view2.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),
        
        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'use_composition':'True',
                'params_file': param_dir,
                'emcl2_params_file': param_dir,
                'default_bt_xml_filename':bt_dir}.items(),
                
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
