from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    wheel_odometry = Node(
        package='antdrone_localization',
        executable='wheel_odometry', 
        parameters=[
            {
                'use_sim_time' : LaunchConfiguration("USE_SIM_TIME")
            }
        ],
        condition=UnlessCondition(LaunchConfiguration('USE_SIM_TIME'))
    )

    ld = LaunchDescription()
    ld.add_action(wheel_odometry)

    return ld


