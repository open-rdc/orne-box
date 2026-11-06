#!/usr/bin/env python3
#
# Gazebo Ignition (Fortress) launch file for orne_box (tsukuba world)
# 事前に scripts/download_tsukuba_meshes.sh でメッシュを取得すること
#

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # パッケージディレクトリの取得
    this_directory = get_package_share_directory('orne_box_simulation')

    world_file_name = 'tsukuba.sdf'
    world = os.path.join(this_directory, 'world', world_file_name)

    # [変更] gazebo_ros → ros_gz_sim に変更
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    bringup_dir = os.path.join(
        get_package_share_directory('orne_box_bringup'), 'launch/include'
    )

    # [追加] velodyne_description の share ディレクトリと tsukuba モデルを IGN_GAZEBO_RESOURCE_PATH に追加
    # Ignition は model:// URIを IGN_GAZEBO_RESOURCE_PATH から解決するため、
    # Gazebo Classic の GAZEBO_MODEL_PATH に相当する設定が必要
    velodyne_description_share_parent = os.path.dirname(
        get_package_share_directory('velodyne_description')
    )
    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            velodyne_description_share_parent,
            ':',
            os.path.join(this_directory, 'models'),
            ':',
            # 既存の IGN_GAZEBO_RESOURCE_PATH があれば引き継ぐ
            os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
        ]
    )

    # Launch引数の宣言
    declare_gui_cmd = DeclareLaunchArgument(
        'gui',
        default_value='True',
        description='Whether to launch the Gazebo GUI or not (headless)')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time')

    # スポーン位置の既定値は waypoint の始点、z は地形高さ 24.999 + 0.05
    declare_spawn_x_cmd = DeclareLaunchArgument(
        'spawn_x', default_value='24.808', description='Spawn X (m)')
    declare_spawn_y_cmd = DeclareLaunchArgument(
        'spawn_y', default_value='3.303', description='Spawn Y (m)')
    declare_spawn_z_cmd = DeclareLaunchArgument(
        'spawn_z', default_value='25.049', description='Spawn Z (m)')
    declare_spawn_yaw_cmd = DeclareLaunchArgument(
        'spawn_yaw', default_value='3.065', description='Spawn yaw (rad)')

    # LaunchConfiguration
    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ロボットdescriptionはbringup側launchを読み込んで統一管理
    # [注意] description.launch.py 内で ignition_gazebo:=true を渡すこと
    start_description_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'description.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'ignition_gazebo': 'true',   # [追加] Ignition用URDFを選択
        }.items()
    )

    # [変更] Gazebo Ignition 起動
    # gazebo_ros の gazebo.launch.py → ros_gz_sim の gz_sim.launch.py
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [world, ' -r'],  # -r: シミュレーション自動開始
            'on_exit_shutdown': 'True',
        }.items()
    )

    # [変更] ロボットのスポーン
    # gazebo_ros の spawn_entity.py → ros_gz_sim の create
    spawn_robot_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'orne_box',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('spawn_x'),    # X座標 (m)
            '-y', LaunchConfiguration('spawn_y'),    # Y座標 (m)
            '-z', LaunchConfiguration('spawn_z'),    # Z座標 (m)
            '-R', '0.0',    # ロール (rad)
            '-P', '0.0',    # ピッチ (rad)
            '-Y', LaunchConfiguration('spawn_yaw'),    # ヨー (rad)
        ],
        output='screen',
    )

    # [追加] ros_gz_bridge: ROS 2 ↔ Ignition トピックブリッジ
    # Classic では gazebo_ros プラグインが自動的にROS側へパブリッシュしていたが、
    # Ignitionでは明示的にブリッジが必要
    start_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            # 制御コマンド: ROS 2 → Ignition
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            # オドメトリ: Ignition → ROS 2
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            # IMU: Ignition → ROS 2
            '/world/default/model/orne_box/link/base_footprint/sensor/imu/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            # 3D LiDAR (PointCloud2): Ignition → ROS 2
            '/model/orne_box/rfans/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            # 2D LiDAR (LaserScan): Ignition → ROS 2
            '/model/orne_box/low_scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            # Joint states: Ignition → ROS 2
            '/world/default/model/orne_box/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
            # odom -> base_link は EKF / odometry_integrator が配信するので
            # DiffDrive の /model/orne_box/tf はブリッジしない (二重配信でTFが跳ぶ)
            # シミュレーション時刻
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        remappings=[
            # LiDARトピック名を既存の名前に合わせてリマップ
            ('/world/default/model/orne_box/joint_state', '/joint_state'),
            ('/world/default/model/orne_box/link/base_footprint/sensor/imu/imu', '/imu/data'),
            ('/model/orne_box/rfans/points', '/surestar_points'),
            ('/model/orne_box/low_scan', '/low_scan'),
        ],
        output='screen',
    )

    # [追加] 3D LiDAR frame_id の静的TF
    start_lidar_tf_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0',
                   'surestar',
                   'orne_box/base_footprint/surestar'],
        output='screen',
    )

    # GUI とロボットの起動を待ってからカメラ追従を開始
    start_camera_follow_cmd = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'until ign service -l | grep -q "^/gui/follow$" && '
            'ign topic -l | grep -q "^/model/orne_box/low_scan$"; do sleep 1; done; '
            'sleep 2; '
            'ign service -s /gui/follow --reqtype ignition.msgs.StringMsg '
            '--reptype ignition.msgs.Boolean --timeout 5000 --req \'data: "orne_box"\'',
        ],
        output='log',
    )

    # テレオペレーション (変更なし)
    start_teleop_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'teleop.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # PointCloud -> LaserScan 変換 (rfans -> /scan)
    start_rfans_scan_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'rfans_scan.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # LaunchDescriptionの構築
    ld = LaunchDescription()

    # 引数の追加
    ld.add_action(declare_gui_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_spawn_x_cmd)
    ld.add_action(declare_spawn_y_cmd)
    ld.add_action(declare_spawn_z_cmd)
    ld.add_action(declare_spawn_yaw_cmd)

    # [追加] IGN_GAZEBO_RESOURCE_PATH の設定（Gazebo起動より前に実行）
    ld.add_action(set_ign_resource_path)

    # Gazebo Ignition 起動
    ld.add_action(start_gazebo)

    # ロボット関連
    ld.add_action(start_description_cmd)
    ld.add_action(spawn_robot_cmd)

    # [追加] ブリッジ起動
    ld.add_action(start_bridge_cmd)

    # [追加] LiDAR frame_id TF
    ld.add_action(start_lidar_tf_cmd)

    # カメラ追従
    ld.add_action(start_camera_follow_cmd)

    # センサー処理
    ld.add_action(start_rfans_scan_cmd)

    # テレオペレーション
    ld.add_action(start_teleop_cmd)

    return ld
