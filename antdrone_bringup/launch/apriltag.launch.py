
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():

    ld = LaunchDescription()


    antdrone_apriltag = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        parameters=[
            '/home/humble_ws/src/antdrone_apriltag/config/apriltag.yaml'
        ],
        remappings = [
            ('/camera_info', ''),
            ('/image_rect', '')
        ]
    )

    ld.add_action(antdrone_apriltag)


    return ld


