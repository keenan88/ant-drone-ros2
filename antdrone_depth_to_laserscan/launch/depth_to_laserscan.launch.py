import launch
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    ld = launch.LaunchDescription()

    config_filepath = '/home/humble_ws/src/antdrone_depth_to_laserscan/config/pcl_to_laserscan.yaml'

    for realsense_placement in ['front_rs', 'left_rs', 'right_rs']:

        depth_to_scan = Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name = realsense_placement + '_pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', realsense_placement + '/pointcloud_downsampled'),
                ('scan', realsense_placement + '/scan')
            ],
            parameters = [
                config_filepath,
                {'target_frame': 'base_link'},
                {'use_sim_time': LaunchConfiguration("USE_SIM_TIME")}
            ]
        )

        ld.add_action(depth_to_scan)

    realsense_placement = 'rear_rs'

    depth_to_scan_rear_rs_left_side = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name = realsense_placement + '_left_pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', realsense_placement + '/pointcloud_downsampled'),
            ('scan', realsense_placement + '/scan_left')
        ],
        parameters = [
            config_filepath,
            {'target_frame': 'base_link'},
            {'use_sim_time': LaunchConfiguration("USE_SIM_TIME")}
        ]
    )
    
    depth_to_scan_rear_rs_right_side = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name = realsense_placement + '_right_pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', realsense_placement + '/pointcloud_downsampled'),
            ('scan', realsense_placement + '/scan_right')
        ],
        parameters = [
            config_filepath,
            {'target_frame': 'base_link'},
            {'use_sim_time': LaunchConfiguration("USE_SIM_TIME")}
        ]
    )

    scan_merger = Node(
        package='antdrone_depth_to_laserscan',
        executable='laser_scan_merger',
        parameters=[
            {'use_sim_time': LaunchConfiguration("USE_SIM_TIME")}
        ]
    )

    ld.add_action(depth_to_scan_rear_rs_left_side)
    ld.add_action(depth_to_scan_rear_rs_right_side)
    ld.add_action(scan_merger)

    return ld