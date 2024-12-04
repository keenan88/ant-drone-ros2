from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():

    drone_name = os.getenv('DRONE_NAME')
    

    print(drone_name)

    wheels_or_body_odometry = DeclareLaunchArgument(
        'WHEEL_ODOMETRY',
        default_value='BODY',
        description='Whether to determine odometry from mecanum wheels or body (body given by IsaacSim)'
    )
    
    wheel_odometry = Node(
        package='linorobot2_localization',
        executable='wheel_odometry', 
        namespace = drone_name,
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
        namespace = drone_name,
        parameters=[
            {'use_sim_time': True}
        ],
        condition=IfCondition(LaunchConfiguration('WHEEL_ODOMETRY'))
    )

    odom_vel_scale_gz = Node(
        package='linorobot2_localization',
        executable='odom_vel_scale_gz',
        namespace = drone_name,
        parameters=[
            {'use_sim_time': True},
            {'drone_name': drone_name}
        ]
    )

    

    ld = LaunchDescription()

    ld.add_action(wheels_or_body_odometry)
    ld.add_action(wheel_odometry)
    ld.add_action(wheel_unraveller)
    ld.add_action(odom_vel_scale_gz)


    return ld


