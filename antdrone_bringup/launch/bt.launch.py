
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():

    ld = LaunchDescription()


    antdrone_bt = Node(
        package='antdrone_bt',
        executable='antdrone_bt',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration("USE_SIM_TIME"),
                'DRONE_NAME': LaunchConfiguration("DRONE_NAME")
            }
        ]
    )

    bridge_to_queen_and_rmf = Node(
        package='antdrone_bt',
        executable='queen_domain_bridge',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration("USE_SIM_TIME"),
            }
        ],
        arguments = [
            '--from', os.getenv('ROS_DOMAIN_ID'), '--to', os.getenv('MAIN_ROS_DOMAIN_ID'),
            '/home/humble_ws/src/antdrone_bt/queen_domain_bridge/queen_domain_bridge.yaml'
        ]
    )

    ld.add_action(antdrone_bt)
    ld.add_action(bridge_to_queen_and_rmf)

    return ld


