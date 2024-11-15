from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, LogInfo
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration, LocalSubstitution
from launch.conditions import IfCondition, UnlessCondition
import os


def generate_launch_description():

    drone_name = os.getenv('DRONE_NAME')

    wheeled_motion = DeclareLaunchArgument(
        'WHEELED_MOTION',
        default_value='False', 
        description='Whether robot moves with wheels or as a teleported block'
    )

    joystick_driver = Node(
        package = 'joy', 
        executable = 'joy_node', 
        name = 'joy_node',
        namespace = drone_name,
        parameters = [
            '/home/humble_ws/src/linorobot2_joystick/config/joy_node_config.yaml',
            {'dev': '/dev/input/js0'}
        ]
    )
    
    joystick_twist = Node(
        package = 'teleop_twist_joy', 
        executable = 'teleop_node',
        name = 'teleop_twist_joy_node',
        namespace = drone_name,
        parameters = [
            '/home/humble_ws/src/linorobot2_joystick/config/teleop_twist_joy_config.yaml'
        ]
    )

    cmd_vel_to_motor_vel = Node(
        package = 'linorobot2_joystick', 
        executable = 'cmd_vel_to_motor_vel',
        namespace=drone_name,
        parameters = [
            {
                'use_sim_time': True
            }
        ],
        condition=IfCondition(LaunchConfiguration('WHEELED_MOTION'))
    )  

    

    # robot_teleporter = Node(
    #     package = 'linorobot2_joystick', 
    #     executable = 'robot_teleporter',
    #     parameters = [
    #         {
    #             'use_sim_time': True
    #         }
    #     ],
    #     condition=UnlessCondition(LaunchConfiguration('WHEELED_MOTION'))
    # )    

    # reset_robot_pose = RegisterEventHandler(
    #     OnShutdown(
    #         on_shutdown=[LogInfo(
    #             msg=['Launch was asked to shutdown: ',
    #                 LocalSubstitution('event.reason')]
    #         )]
    #     )
    # )

    ld = LaunchDescription()

    ld.add_action(wheeled_motion)

    ld.add_action(joystick_driver)
    ld.add_action(joystick_twist)
    ld.add_action(cmd_vel_scaler)
    ld.add_action(cmd_vel_to_motor_vel)
    # ld.add_action(robot_teleporter)

    # ld.add_action(reset_robot_pose)

    return ld
