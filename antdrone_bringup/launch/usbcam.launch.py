
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os

def generate_launch_description():

    ld = LaunchDescription()

    apriltag_cam = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name =  "front_usb_cam",
        parameters=[
            {
                'use_sim_time' : LaunchConfiguration("USE_SIM_TIME"),
                'camera_name' : 'apriltag_cam',
                'video_device' : '/dev/integrated_camera',
                'brightness': 100
            },

        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2'
    )

    ld.add_action(apriltag_cam)
    ld.add_action(rviz)

    return ld


