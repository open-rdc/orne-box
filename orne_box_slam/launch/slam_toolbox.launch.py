
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    autostart = LaunchConfiguration('autostart')
    slam_toolbox_params_file = LaunchConfiguration('slam_toolbox_params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    launch_include_file_dir = os.path.join(get_package_share_directory('orne_box_bringup'), 'launch/include') 
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='True',
        description='Automatically startup the nav2 stack')
    declare_arg_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Set "true" to launch rviz.')
    declare_slam_toolbox_params_file = DeclareLaunchArgument(
        'slam_toolbox_params_file',
        default_value=os.path.join(get_package_share_directory("orne_box_slam"),
                                   'config', 'slam_toolbox', 'slam_toolbox.param.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')
    declare_use_respawn = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')
    declare_log_level = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    param_substitutions = {
        'use_sim_time': use_sim_time}

    configured_params = RewrittenYaml(
        source_file=slam_toolbox_params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    rviz_config_file = os.path.join(
        get_package_share_directory('orne_box_slam'), 
            'config', 'rviz', 'slam_toolbox.rviz')

    lifecycle_nodes = ['map_saver']

    map_saver_server = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[configured_params])

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}])
    
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[configured_params],)    
    
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_rviz))
    
    description_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [launch_include_file_dir, '/description.launch.py'])
        )
    ld = LaunchDescription()

    ld.add_action(declare_namespace)
    ld.add_action(declare_slam_toolbox_params_file)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_autostart)
    ld.add_action(declare_use_respawn)
    ld.add_action(declare_log_level)
    ld.add_action(declare_arg_use_rviz)

    ld.add_action(map_saver_server)
    ld.add_action(lifecycle_manager)

    ld.add_action(slam_toolbox)
    ld.add_action(description_launch)
    ld.add_action(rviz2)

    return ld