
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    drone_name = os.getenv('DRONE_NAME')

    ld = LaunchDescription()


    floor_mission_bt = Node(
        package='floor_mission_bt',
        executable='floor_mission_bt',
        namespace = drone_name,
        parameters=[
            {
                'use_sim_time': True,
            }
        ]
    )

    heartbeat = Node(
        package='floor_mission_helper',
        executable='heartbeat',
        namespace = drone_name,
        parameters=[
            {
                'use_sim_time': True,
            }
        ]
    )



    
    ld.add_action(floor_mission_bt)
    ld.add_action(heartbeat)

    return ld


