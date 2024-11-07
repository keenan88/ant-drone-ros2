from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
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

    odom_vel_scale_gz = Node(
        package='linorobot2_localization',
        executable='odom_vel_scale_gz',
        # condition=UnlessCondition(LaunchConfiguration('WHEEL_ODOMETRY'))
    )


    ld = LaunchDescription()

    ld.add_action(wheels_or_body_odometry)
    ld.add_action(wheel_odometry)
    ld.add_action(wheel_unraveller)
    ld.add_action(odom_vel_scale_gz)
    return ld


