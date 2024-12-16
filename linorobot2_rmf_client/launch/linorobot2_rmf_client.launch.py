import launch
from launch_ros.actions import Node
import os
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription




def generate_launch_description():

    linorobot2_rmf_client = Node(
        package='linorobot2_rmf_client',
        executable='linorobot2_rmf_client'
        parameters = [
            {'use_sim_time' : True}
        ]
    )

    ld = LaunchDescription()
    ld.add_action(linorobot2_rmf_client)
    
    
    return ld