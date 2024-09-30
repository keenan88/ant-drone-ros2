import launch
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    
    ld = launch.LaunchDescription()


    for realsense_placement in ['front_rs', 'rear_rs', 'left_rs', 'right_rs']:

        depth_to_scan = Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name = realsense_placement + '_pointcloud_to_laserscan',
            remappings=[
                ('/cloud_in', '/' + realsense_placement + '/pointcloud_downsampled'),
                ('/scan', '/' + realsense_placement + '/scan')
            ],
            parameters = [
                '/home/humble_ws/src/antdrone_depth_to_laserscan/config/' + realsense_placement + '_pcl_to_laserscan.yaml'
            ]
        )

        ld.add_action(depth_to_scan)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '/home/humble_ws/src/antdrone_depth_to_laserscan/config/depth_to_laserscan.rviz'],
    )

    return ld