import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )

    # 引数の宣言を追加
    ignition_gazebo = LaunchConfiguration("ignition_gazebo")
    ignition_gazebo_arg = DeclareLaunchArgument(
        "ignition_gazebo",
        default_value="false",
        description="Use Ignition Gazebo if true",
    )

    packages_name = "orne_box_description"
    xacro_file_name = "orne_box_3d_lidar_rfans.urdf.xacro"
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(packages_name), "urdf", xacro_file_name]
            ),
            " ignition_gazebo:=",
            ignition_gazebo,   # ← 引数を xacro に渡す
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )
    
    # joint_state_pub_gui_node = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui",
    #     output="screen",
    # )
    joint_state_pub_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
        
    nodes = [
        use_sim_time_arg,
        ignition_gazebo_arg,  # ← 追加
        robot_state_pub_node,
        # joint_state_pub_gui_node
        joint_state_pub_node
    ]

    return LaunchDescription(nodes)
