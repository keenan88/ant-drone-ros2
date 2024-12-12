
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

    bridge_to_queen_and_rmf = Node(
        package='floor_mission_bt',
        executable='domain_bridge',
        namespace = drone_name,
        parameters=[
            {
                'use_sim_time': True,
            }
        ],
        arguments = [
            '--from', os.getenv('ROS_DOMAIN_ID'), '--to', os.getenv('MAIN_ROS_DOMAIN_ID'),
            '/home/humble_ws/src/floor_mission_bt/domain_bridge/domain_bridge.yaml'
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

    moveout = Node(
        package='floor_mission_helper',
        executable='moveout',
        namespace = drone_name,
        parameters=[
            {
                # self.get_clock() does not increment when use_sim_time=True, not sure why.
                'use_sim_time': False,
            }
        ]
    )



    
    ld.add_action(floor_mission_bt)
    ld.add_action(heartbeat)
    ld.add_action(moveout)
    ld.add_action(bridge_to_queen_and_rmf)

    return ld


