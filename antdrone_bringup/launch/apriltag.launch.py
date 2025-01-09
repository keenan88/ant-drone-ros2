
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os

def generate_launch_description():

    ld = LaunchDescription()

    for realsense_placement in ["front_rs", "rear_rs", "left_rs", "right_rs"]:

        gz_img_frame_fixer = Node(
            package='antdrone_apriltag',
            executable='gz_img_frame_fixer',
            name = realsense_placement + "_gz_img_frame_fixer",
            output='screen',
            parameters=[
                {'camera_pos': realsense_placement},
                {'use_sim_time' : LaunchConfiguration("USE_SIM_TIME")},
            ],
            condition = IfCondition(LaunchConfiguration("USE_SIM_TIME"))
        )

        ld.add_action(gz_img_frame_fixer)


    antdrone_apriltag = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        parameters=[
            '/home/humble_ws/src/antdrone_apriltag/config/apriltag.yaml'
        ],
        remappings = [
            ('/camera_info', '/front_rs/front_rs/color/camera_info'),
            ('/image_rect',  '/front_rs/front_rs/color/image_raw')
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', '/home/humble_ws/src/antdrone_apriltag/config/apriltag.rviz']
    )

    ld.add_action(antdrone_apriltag)
    ld.add_action(rviz)


    return ld


