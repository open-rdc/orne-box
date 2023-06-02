# SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    adi_driver2_dir = get_package_share_directory('orne_box_bringup')
    params_file = os.path.join(adi_driver2_dir, 'config/imu/', 'adis16465.param.yaml')

    push_ns = PushRosNamespace([LaunchConfiguration('namespace')])
    
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='imu',
        description='Set namespace for node.')

    adis16465 = Node(
        name='adis16465_node',
        package='adi_driver2',
        executable='adis16465',
        parameters=[params_file],
        output='screen')

    imu = GroupAction(
        actions=[
        push_ns,
        adis16465,
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace)
    
    ld.add_action(imu)

    return ld
