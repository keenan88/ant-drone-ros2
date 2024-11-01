from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    wheels_or_body_odometry = DeclareLaunchArgument(
        'WHEEL_ODOMETRY',
        default_value='BODY',
        description='Whether to determine odometry from mecanum wheels or body (body given by IsaacSim)'
    )
    
    wheel_odometry = Node(
        package='linorobot2_localization',
        executable='wheel_odometry', 
        parameters=[
            {
                'use_sim_time' : True
            }
        ],
        condition=IfCondition(LaunchConfiguration('WHEEL_ODOMETRY'))
    )

    wheel_unraveller = Node(
        package='linorobot2_localization',
        executable='wheel_unraveller',
        parameters=[
            {'use_sim_time': True}
        ],
        condition=IfCondition(LaunchConfiguration('WHEEL_ODOMETRY'))
    )


    ld = LaunchDescription()

    ld.add_action(wheels_or_body_odometry)
    ld.add_action(wheel_odometry)
    ld.add_action(wheel_unraveller)

    return ld


