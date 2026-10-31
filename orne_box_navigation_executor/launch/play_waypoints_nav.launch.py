import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    nav_dir = get_package_share_directory('orne_box_navigation_executor')
    launch_file_dir = os.path.join(get_package_share_directory('orne_box_navigation_executor'), 'launch')
    config_dir = os.path.join(nav_dir, 'config')

    # config/maps/<map_pass>.pcd (ndt_localization)
    # config/maps/<map_pass>.yaml (emcl2, map_server)
    # config/maps/<map_pass>_keepout.yaml (costmap, 経路計画用) 
    map_pass = 'tsukuba/tsukuba2026'
    WAYPOINT_PATH = 'tsukuba/tsukuba2026'

    bt_file_name ='navigate_to_pose_w_replanning_and_recovery.xml' # 旧: navigate_w_replanning_and_recovery.xml

    map_yaml = os.path.join(config_dir, 'maps', map_pass + '.yaml')
    keepout_yaml = os.path.join(config_dir, 'maps', map_pass + '_keepout.yaml')
    map_data = LaunchConfiguration('map', default=map_yaml)
    costmap_data = LaunchConfiguration('costmap', default=LaunchConfiguration(
        'mask', default=keepout_yaml))
    pcd_map_data = LaunchConfiguration('pcd_map', default=os.path.join(config_dir, 'maps', map_pass + '.pcd'))
    bt_dir = LaunchConfiguration('default_bt_xml_filename', default=os.path.join(config_dir, 'behavior_trees', bt_file_name))
    rviz_config_dir = os.path.join(config_dir, 'rviz', 'nav2_waypoint_tools.rviz')
    waypoint_tools_launch_dir = os.path.join(get_package_share_directory('waypoint_tools'), 'launch')
    
    lifecycle_nodes = ['filter_mask_server', 'costmap_filter_info_server']

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
 
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(config_dir, 'params', 'nav2_params.yaml'),
            description='Full path to the ROS 2 parameters file to use'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=map_data,
            description='Full path to map file to load'
        ),    
        DeclareLaunchArgument(
            'pcd_map',
            default_value=pcd_map_data,
            description='Full path to PCD map file for ndt_localization'
        ),
        DeclareLaunchArgument(
            'localization', default_value='ndt',
            description='Localization method: ndt or emcl2'
        ),
        DeclareLaunchArgument(
            'costmap',
            default_value=costmap_data,
            description='Full path to map file to load'
        ),     
        DeclareLaunchArgument(
            'use_composition', default_value='False',
            description='Use composed Nav2 nodes if true'
        ),     
        DeclareLaunchArgument(
            'waypoint_path',
            default_value=os.path.join(config_dir, 'waypoints', f'{WAYPOINT_PATH}.yaml'),
            description='Waypoint YAML file sent by waypoint_tools'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_data,
                'pcd_map': pcd_map_data,
                'localization': LaunchConfiguration('localization'),
                'costmap': costmap_data,
                'use_sim_time': use_sim_time,
                'use_composition': LaunchConfiguration('use_composition'),
                'params_file': params_file,
                'default_bt_xml_filename':bt_dir}.items(),            
        ),
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([waypoint_tools_launch_dir, '/send.launch.py']),
                launch_arguments={
                    'params_file': params_file,
                    'send_waypoint_path': LaunchConfiguration('waypoint_path'),
                    'use_sim_time': use_sim_time}.items(),
            ),
        ]),
        # ジョイスティックコマンド(Joyで/next_wpを送る)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/joy_command.launch.py'])
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])
