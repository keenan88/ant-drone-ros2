from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    
    gz_wheel_odometry = Node(
        package='antdrone_localization',
        executable='wheel_odometry', 
        parameters=[
            {
                'use_sim_time' : LaunchConfiguration("USE_SIM_TIME")
            }
        ],
        condition=IfCondition(LaunchConfiguration('USE_SIM_TIME'))
    )

    

    

    ld = LaunchDescription()

    
    ld.add_action(gz_wheel_odometry)


    return ld


