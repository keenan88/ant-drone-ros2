import launch
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # front_and_rear_rs_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource('/home/humble_ws/src/antdrone_bringup/launch/realsenses.launch.py'),
    #     launch_arguments={
    #         'serial_no1': "'207522073046'", # 
    #         'serial_no2': "'207522073816'"
    #     }.items()
    # )

    front_rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'serial_no': "'207522073046'",
            'camera_name':'front_rs',
            'camera_namespace':'front_rs',
        }.items()
    )

    rear_rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'serial_no': "'207522073816'",
            'camera_name':'rear_rs',
            'camera_namespace':'rear_rs',
        }.items()
    )

    left_rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'serial_no': "'108222250264'",
            'camera_name':'left_rs',
            'camera_namespace':'left_rs',
        }.items()
    )



    bridge_out_tf = Node(
        package="domain_bridge",
        executable="domain_bridge",
        name = "bridge_out_tf",
        arguments = ['/home/humble_ws/src/antworker_realsense/config/bridge_out_tf.yaml']
    )

    front_rs_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_front_rs",
        output="log",
        arguments = [
            "0.5", "0.0", "-0.18",
            "0.0", "0.0", "0.0",
            "base_link",
            "front_rs_link",
        ],
    )

    rear_rs_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_rear_rs",
        output="log",
        arguments = [
            "-0.5", "0.0", "-0.18",
            "3.14", "0.0", "0.0",
            "base_link",
            "rear_rs_link",
        ],
    )

    left_rs_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_front_rs",
        output="log",
        arguments = [
            "0.5", "0.0", "-0.18",
            "0.0", "0.0", "0.0",
            "base_link",
            "left_rs_link",
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', '/home/humble_ws/src/antdrone_realsense/config/rs.rviz'],
        parameters=[
            {'use_sim_time': False}, 
        ]
    )
    
    ld = launch.LaunchDescription()

    # ld.add_action(front_and_rear_rs_launch)
    ld.add_action(front_rs_launch)
    ld.add_action(rear_rs_launch)
    ld.add_action(left_rs_launch)
    ld.add_action(rviz)
    # ld.add_action(bridge_out_tf)
    ld.add_action(front_rs_tf)
    ld.add_action(rear_rs_tf)
    ld.add_action(left_rs_tf)
    
    return ld



    
