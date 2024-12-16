from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    linorobot2_rmf_client = Node(
        package='linorobot2_rmf_client',
        executable='linorobot2_rmf_client',
        parameters = [
            {'use_sim_time' : True},
            {'DRONE_NAME' : LaunchConfiguration('DRONE_NAME')}
        ]
    )

    ld = LaunchDescription()
    ld.add_action(linorobot2_rmf_client)
    
    
    return ld