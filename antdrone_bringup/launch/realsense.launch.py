import launch
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Realsenses freeze up if FPS or resolution too high. Depends on USB ports used.

    rs_common_settings = {
        'pointcloud.enable':'True',
        'rgb_camera.color_profile' : '424x240x15',
        'depth_module.depth_profile' : '480x270x15',
        'depth_module.infra_profile': '480x270x15',
        'decimation_filter.enable': 'True', # Decimation filter in realsense hardware allows pointcloud FPS to keep up with RGB & depth FPS
        'decimation_filter.filter_magnitude': '3', # Filter magnitude appears stuck at 2.. that is fine for now
        'clip_distance': '3.0'
    }

    rs_launch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
    )


    front_rs_launch = IncludeLaunchDescription(
        rs_launch,
        launch_arguments=(rs_common_settings | {
            'serial_no': "'207522073816'",
            'camera_name':'front_rs',
            'camera_namespace':'front_rs',
        }).items()
    )

    rear_rs_launch = IncludeLaunchDescription(
        rs_launch,
        launch_arguments=(rs_common_settings | {
            'serial_no': "'207122078912'",
            'camera_name':'rear_rs',
            'camera_namespace':'rear_rs'
        }).items()
    )

    left_rs_launch = IncludeLaunchDescription(
        rs_launch,
        launch_arguments=(rs_common_settings | {
            'serial_no': "'109122070837'",
            'camera_name':'left_rs',
            'camera_namespace':'left_rs'
        }).items()
    )

    right_rs_launch = IncludeLaunchDescription(
        rs_launch,
        launch_arguments=(rs_common_settings | {
            'serial_no': "'207522073046'",
            'camera_name':'right_rs',
            'camera_namespace':'right_rs'
        }).items()
    )

    front_rs_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_front_rs",
        output="log",
        arguments = [
            "0.407", "0.0", "0.0",
            "0.0", "0.0", "0.0",
            "base_link",
            "front_rs_link",
        ],
    )

    left_rs_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_left_rs",
        output="log",
        arguments = [
            "0.0", "0.247", "0.0",
            "1.57", "0.0", "0.0",
            "base_link",
            "left_rs_link",
        ],
    )

    rear_rs_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_rear_rs",
        output="log",
        arguments = [
            "-0.407", "0.0", "0.0",
            "3.14", "0.0", "0.0",
            "base_link",
            "rear_rs_link",
        ]
    )

    right_rs_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_right_rs",
        output="log",
        arguments = [
            "0.0", "-0.247", "0.0",
            "-1.57", "0.0", "0.0",
            "base_link",
            "right_rs_link",
        ],
    )

    
    ld = launch.LaunchDescription()

    ld.add_action(front_rs_launch)
    ld.add_action(left_rs_launch)
    ld.add_action(rear_rs_launch)
    ld.add_action(right_rs_launch)


    
    ld.add_action(front_rs_tf)
    ld.add_action(left_rs_tf)
    ld.add_action(rear_rs_tf)
    ld.add_action(right_rs_tf)
    
    
    return ld



    
