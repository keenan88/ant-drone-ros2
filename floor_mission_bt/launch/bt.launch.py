
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():

    ld = LaunchDescription()


    floor_mission_bt = Node(
        package='floor_mission_bt',
        executable='floor_mission_bt',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration("USE_SIM_TIME"),
                'DRONE_NAME': LaunchConfiguration("DRONE_NAME")
            }
        ]
    )

    bridge_to_queen_and_rmf = Node(
        package='floor_mission_bt',
        executable='domain_bridge',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration("USE_SIM_TIME"),
            }
        ],
        arguments = [
            '--from', os.getenv('ROS_DOMAIN_ID'), '--to', os.getenv('MAIN_ROS_DOMAIN_ID'),
            '/home/humble_ws/src/floor_mission_bt/domain_bridge/domain_bridge.yaml'
        ]
    )

    ld.add_action(floor_mission_bt)
    ld.add_action(bridge_to_queen_and_rmf)

    return ld


