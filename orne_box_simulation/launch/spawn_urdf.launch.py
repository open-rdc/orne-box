import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_urdf_package = get_package_share_directory('orne_box_description')
    urdf_file = os.path.join(pkg_urdf_package, 'urdf', 'orne_box.urdf.xacro')
    
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-entity', 'orne_box',
                    '-topic', '/robot_description',
                    '-x','0.0',
                    '-y','0.0',
                    '-z','0.0',
                    '-R','0.0',
                    '-P','0.0',
                    '-Y','0.0'
                    ],
    )

    ld = LaunchDescription()

    # ld.add_action(DeclareLaunchArgument(''))

    ld.add_action(spawn_entity_node)

    return ld
