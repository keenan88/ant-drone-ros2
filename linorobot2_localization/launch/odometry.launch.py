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
                'use_sim_time' : LaunchConfiguration("USE_SIM_TIME")
            }
        ],
        condition=IfCondition(LaunchConfiguration('WHEEL_ODOMETRY'))
    )

    wheel_unraveller = Node(
        package='linorobot2_localization',
        executable='wheel_unraveller',
        parameters=[
            {'use_sim_time': LaunchConfiguration("USE_SIM_TIME")}
        ],
        condition=IfCondition(LaunchConfiguration('WHEEL_ODOMETRY'))
    )

    gz_localization = Node(
        package='linorobot2_localization',
        executable='gz_localization',
        parameters=[
            {'use_sim_time': LaunchConfiguration("USE_SIM_TIME")},
            {'drone_name': LaunchConfiguration("DRONE_NAME")}
        ]
    )

    gz_frame_name_fixer = Node(
        package='linorobot2_localization',
        executable='gz_frame_name_fixer',
        parameters=[
            {'use_sim_time': LaunchConfiguration("USE_SIM_TIME")},
            {'drone_name': LaunchConfiguration("DRONE_NAME")}
        ]
    )

    

    ld = LaunchDescription()

    ld.add_action(wheels_or_body_odometry)
    ld.add_action(wheel_odometry)
    ld.add_action(wheel_unraveller)
    ld.add_action(gz_localization)
    ld.add_action(gz_frame_name_fixer)


    return ld


