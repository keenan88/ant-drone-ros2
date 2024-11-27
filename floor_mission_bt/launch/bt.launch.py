
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    drone_name = os.getenv('DRONE_NAME')

    ld = LaunchDescription()


    keepout_filter_mask_server = Node(
        package='floor_mission_bt',
        executable='floor_mission_bt',
        namespace = drone_name,
        parameters=[
            {
                'use_sim_time': True,
            }
        ]
    )

    
    ld.add_action(keepout_filter_mask_server)

    return ld


