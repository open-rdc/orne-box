import launch
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pointcloud_to_laserscan_node = launch_ros.actions.Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            #('cloud_in', 'surestar_points'),
            ('cloud_in', 'rfans_points'),
            ('scan', 'surestar_scan'), # scan
            #('scan', 'scan'), 
        ],
        parameters=[{
                'target_frame': '',
                'transform_tolerance': 0.1,
                'min_height': 0.1,
                'max_height': 5.0,
                'angle_min': -3.1415,  # -M_PI/2
                'angle_max': 3.1415,  # M_PI/2
                'angle_increment': 0.0261, #0.0087,  # M_PI/360.0
                'scan_time': 0.1, #0.3333,
                'range_min': 0.0,
                'range_max': 100.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'use_sim_time': True,
            }]
    )

    laser_filters_node = launch_ros.actions.Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[
            {'use_sim_time': True},
            PathJoinSubstitution([
                get_package_share_directory("orne_box_bringup"),
                "config", "box_filter_box3.yaml",
            ])],
        remappings=[
            ('scan_filtered', 'scan'),
            ('scan', 'surestar_scan'),
        ],    
    )

    return launch.LaunchDescription([
        pointcloud_to_laserscan_node,
        laser_filters_node
    ])

if __name__ == '__main__':
    generate_launch_description()

