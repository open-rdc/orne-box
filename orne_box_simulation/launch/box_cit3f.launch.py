#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Darby Lim

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    # パッケージディレクトリの取得
    this_directory = get_package_share_directory('orne_box_simulation')
    
    # orne_box側のファイル
    xacro_path = os.path.join(
        get_package_share_directory('orne_box_description'),
        'urdf',
        'orne_box_3d_lidar_rfans.urdf.xacro'
    )
    world_file_name = 'Tsudanuma_2-3.world'
    world = os.path.join(this_directory, 'world', world_file_name)
    
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    bringup_dir = os.path.join(
        get_package_share_directory('orne_box_bringup'), 'launch/include'
    )
    
    # Launch引数の宣言 (velodyne方式)
    declare_gui_cmd = DeclareLaunchArgument(
        'gui',
        default_value='True',
        description='Whether to launch the Gazebo GUI or not (headless)')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time')
    
    # LaunchConfiguration
    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # robot_description (velodyne方式: launchファイル内でxacroを展開)
    robot_description = Command(['xacro', ' ', xacro_path])
    
    # robot_state_publisher (velodyne方式: launchファイル内で定義)
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )
    
    # Gazebo起動 (velodyne方式: gazebo.launch.pyを使用)
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world, 'gui': gui}.items()
    )
    
    # ロボットのスポーン (velodyne方式: launchファイル内で定義)
    spawn_example_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'orne_box',
            '-topic', 'robot_description',
        ],
        output='screen',
    )
    
    # テレオペレーション (orne_box側の機能を維持)
    start_teleop_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'teleop.launch.py')
        )
    )

    # LaunchDescriptionの構築
    ld = LaunchDescription()
    
    # 引数の追加
    ld.add_action(declare_gui_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    
    # Gazebo起動
    ld.add_action(start_gazebo)
    
    # ロボット関連
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(spawn_example_cmd)
    
    # テレオペレーション
    ld.add_action(start_teleop_cmd)
    
    return ld
