import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('orne_box_navigation_executor')
    config_dir = os.path.join(bringup_dir, 'config')
    launch_dir = os.path.join(bringup_dir, 'launch')

    wll_dir = get_package_share_directory('wheel_lidar_localization')
    wll_params_file = LaunchConfiguration('wll_params_file')
    pcd_map = LaunchConfiguration('pcd_map')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    costmap_yaml_file = LaunchConfiguration('costmap')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    # log_level = LaunchConfiguration('log_level')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}
    
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=map_yaml_file,
        description='Full path to map yaml file to load')
    
    declare_costmap_yaml_cmd = DeclareLaunchArgument(
        'costmap',
        default_value=costmap_yaml_file,
        description='Full path to map yaml file to load')


    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        #default_value='false',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(config_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_wll_params_file_cmd = DeclareLaunchArgument(
        'wll_params_file',
        default_value=os.path.join(wll_dir, 'config', 'rfans_wheel.yaml'),
        description='Full path to the wheel_lidar_localization parameters file')

    declare_pcd_map_cmd = DeclareLaunchArgument(
        'pcd_map', default_value='',
        description='PCD map for wheel_lidar_localization (empty: use wll_params_file value)')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Whether to use composed bringup')
    
    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=os.path.join(
            config_dir,
            'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml'),
        description='Full path to the behavior tree xml file to use')


    # Specify the actions
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),
        Node(
            condition=IfCondition(use_composition),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': autostart}],
            # arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'slam_launch.py')),
            condition=IfCondition(slam),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file}.items()),
        # # use amcl
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(launch_dir,
        #                                                'localization_launch.py')),
        #     condition=UnlessCondition(slam),
        #     launch_arguments={'namespace': namespace,
        #                       'map': map_yaml_file,
        #                       'use_sim_time': use_sim_time,
        #                       'autostart': autostart,
        #                       'params_file': params_file,
        #                       'use_composition': use_composition,
        #                       'use_respawn': use_respawn,
        #                       'container_name': 'nav2_container'}.items()
        # ),
        # use wheel_lidar_localization (map_server still serves the 2D map for costmaps)
        GroupAction(
            condition=UnlessCondition(slam),
            actions=[
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    parameters=[{'yaml_filename': map_yaml_file,
                                 'use_sim_time': use_sim_time}],
                    output='screen'),
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_localization',
                    parameters=[{'use_sim_time': use_sim_time,
                                 'autostart': True,
                                 'node_names': ['map_server']}],
                    output='screen'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(wll_dir, 'launch',
                                                               'localization.launch.py')),
                    launch_arguments={
                        'params_file': wll_params_file,
                        'map_path': pcd_map,
                        'use_sim_time': PythonExpression(
                            ["'", use_sim_time, "'.lower()"]),
                        'rviz': 'false'}.items()),
            ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'default_bt_xml_filename': default_bt_xml_filename,
                              'use_composition': use_composition,
                              'use_respawn': use_respawn,
                            #   'use_lifecycle_mgr': 'false',
                              'costmap': costmap_yaml_file,
                              'map_subscribe_transient_local': 'true',
                              'container_name': 'nav2_container'}.items()
        ),
    ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_costmap_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_wll_params_file_cmd)
    ld.add_action(declare_pcd_map_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_bt_xml_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld
