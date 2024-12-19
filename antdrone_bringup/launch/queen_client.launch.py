from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    rmf_client = Node(
        package='antdrone_queen_client',
        executable='rmf_client',
        parameters = [
            {'use_sim_time' : LaunchConfiguration("USE_SIM_TIME")},
            {'DRONE_NAME' : LaunchConfiguration('DRONE_NAME')},
            {'FLEET_NAME': LaunchConfiguration('FLEET_NAME')}
        ]
    )

    mission_heartbeat = Node(
        package='antdrone_queen_client',
        executable='heartbeat',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration("USE_SIM_TIME"),
                'DRONE_NAME': LaunchConfiguration("DRONE_NAME")
            }
        ]
    )

    ld = LaunchDescription()
    ld.add_action(rmf_client)
    ld.add_action(mission_heartbeat)
    
    return ld