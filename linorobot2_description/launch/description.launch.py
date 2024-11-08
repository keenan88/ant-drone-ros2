from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    urdf_path = '/home/humble_ws/src/linorobot2_description/urdf/robots/mecanum_libplanar.urdf.xacro'
    # urdf_path =  '/home/humble_ws/src/antworker_description/description/worker/urdf/worker.urdf.xacro'


    return LaunchDescription([
        DeclareLaunchArgument(
            name='urdf', 
            default_value=urdf_path,
            description='URDF path'
        ),

        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='true',
            description='Use simulation time'
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'robot_description': Command([
                        'xacro ', LaunchConfiguration('urdf'),
                        ' namespace:=', LaunchConfiguration('namespace')
                    ])
                }
            ]
        ),
    ])