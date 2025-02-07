
from launch import LaunchDescription
from launch_ros.actions import Node
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
                'use_sim_time' : 'False',
                'camera_name' : 'apriltag_cam',
                'video_device' : '/dev/apriltag_cam',
                'brightness': 100
            },
            '/home/humble_ws/src/antdrone_apriltag/config/usb_cam.yaml',
        ],
        remappings = [
            ('/camera_info', '/apriltag_cam/apriltag_cam/color/camera_info'),
            ('/image_raw',  '/apriltag_cam/apriltag_cam/color/image_raw')
        ]
    )

    apriltag_cam_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_front_rs",
        output="log",
        arguments = [
            "0.0", "0.215", "0.195",
            "-1.57", "0", "-0.95",
            "base_link",
            "apriltag_cam",
        ],
    )

    ld.add_action(apriltag_cam)
    ld.add_action(apriltag_cam_tf)

    return ld


