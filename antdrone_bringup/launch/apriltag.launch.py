
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os

def generate_launch_description():

    ld = LaunchDescription()

    gz_img_frame_fixer = Node(
        package='antdrone_apriltag',
        executable='gz_img_frame_fixer',
        name =  "apriltag_cam_gz_img_frame_fixer",
        output='screen',
        parameters=[
            {'use_sim_time' : LaunchConfiguration("USE_SIM_TIME")},
        ],
        condition = IfCondition(LaunchConfiguration("USE_SIM_TIME"))
    )

    antdrone_apriltag = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        parameters=[
            '/home/humble_ws/src/antdrone_apriltag/config/apriltag.yaml'
        ],
        remappings = [
            ('/camera_info', '/apriltag_cam/apriltag_cam/color/camera_info'),
            ('/image_rect',  '/apriltag_cam/apriltag_cam/color/image_raw')
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', '/home/humble_ws/src/antdrone_apriltag/config/apriltag.rviz']
    )

    go_under = Node(
        package='antdrone_apriltag',
        executable='go_under'
    )

    ld.add_action(gz_img_frame_fixer)
    ld.add_action(antdrone_apriltag)
    ld.add_action(rviz)
    ld.add_action(go_under)


    return ld


