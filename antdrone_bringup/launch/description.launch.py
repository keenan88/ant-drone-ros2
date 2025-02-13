from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description(): 

    urdf_path = os.path.join(get_package_share_directory('antdrone_description'), 'urdf', 'mecanum_libplanar.urdf.xacro')

    return LaunchDescription([

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'robot_description': Command([
                        'xacro ', urdf_path,
                        ' drone_name:=', LaunchConfiguration('DRONE_NAME')
                    ])
                }
            ]
        ),
    ])