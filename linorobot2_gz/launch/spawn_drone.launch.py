import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml

def generate_launch_description():
    use_sim_time = True
    
    drone_description_launch_path = os.path.join(get_package_share_directory('linorobot2_description'), 'launch', 'description.launch.py')

    return LaunchDescription([        

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            namespace=LaunchConfiguration('namespace'),
            arguments=[
                '-topic', 'robot_description', 
                '-entity', LaunchConfiguration('namespace'), 
                '-x', LaunchConfiguration('spawn_x'),
                '-y', LaunchConfiguration('spawn_y'),
                '-z', LaunchConfiguration('spawn_z'),
                '-Y', LaunchConfiguration('spawn_yaw'),

            ]
        ),       

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(drone_description_launch_path),
            launch_arguments={
                'use_sim_time': str(use_sim_time),
                'namespace': LaunchConfiguration('namespace')
            }.items()
        )
    ])