import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
def generate_launch_description():
    icart_mini_driver_dir = get_package_share_directory('orne_box_bringup')

    #for ypspur param file ex.RADIUS,TREAD
    # ypspur_param = os.path.join(icart_mini_driver_dir,'config/ypspur','icart_mini.param')
    ypspur_param = os.path.join(icart_mini_driver_dir,'config/ypspur','box_v3.param')
    
    launch_file_dir = os.path.join(get_package_share_directory('orne_box_bringup'), 'launch') 
    launch_include_file_dir = os.path.join(get_package_share_directory('orne_box_bringup'), 'launch/include') 
    
    #for icart_driver_node param file ex. odom_frame_id,Hz
    # driver_param = os.path.join(icart_mini_driver_dir,'config','driver_node.param.yaml')
    
    ypspur_coordinator_path = os.path.join(icart_mini_driver_dir,'scripts','ypspur_coordinator_bridge')
    return LaunchDescription([
        launch.actions.LogInfo(
            msg="Launch ypspur coordinator."
        ),
        
        launch.actions.ExecuteProcess(
            cmd=[ypspur_coordinator_path,ypspur_param],
            shell=True,

        ),

        launch.actions.LogInfo(
            msg="Launch icart_mini_mini_driver node."
        ),
        Node(
            package='orne_box_bringup',
            executable='icart_mini_driver',
            parameters=[{'odom_frame_id':'odom',
                        'base_frame_id':'base_footprint',
                        'Hz':40,
                        'left_wheel_joint':'left_wheel_joint',
                        'right_wheel_joint':'right_wheel_joint',
                        'liner_vel_lim':1.5,
                        'liner_accel_lim':1.5,
                        'angular_vel_lim':3.14,
                        'angular_accel_lim':3.14,
                        'calculate_odom_from_ypspur':True,
                        'publish_odom_tf':True
            }]
        ),
        #robot_state_publisher and joint_state_publisher
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [launch_include_file_dir, '/description.launch.py'])
        # ),
        # launch.actions.LogInfo(
        #     msg="Launch robot_description  node."
        # ),
          IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [launch_include_file_dir, '/teleop.launch.py'])
        ),
        launch.actions.LogInfo(
            msg="Launch joy node."
        ),
        #mixed wheel_odom and other (IMU etc..)
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [launch_include_file_dir, '/robot_localization_ekf.launch.py'])
        # ),
        # launch.actions.LogInfo(
        #     msg="Launch robot_localization_ekf node."
        # ),

        # # sensors
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [launch_include_file_dir, '/urg_node2.launch.py'])
        # ),
        # launch.actions.LogInfo(
        #     msg="Launch URG  node."
        # ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [launch_include_file_dir, '/adis16465.launch.py'])
        # ),
        # launch.actions.LogInfo(
        #     msg="Launch IMU node."
        # )
    ])