import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    drone_description_launch_path = os.path.join(get_package_share_directory('antdrone_description'), 'launch', 'description.launch.py')

    return LaunchDescription([        

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            namespace=LaunchConfiguration('DRONE_NAME'),
            arguments=[
                '-topic', 'robot_description', 
                '-entity', LaunchConfiguration('DRONE_NAME'), 
                '-x', LaunchConfiguration('x0'),
                '-y', LaunchConfiguration('y0'),
                '-z', LaunchConfiguration('z0'),
                '-Y', LaunchConfiguration('yaw0'),

            ]
        ),       

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(drone_description_launch_path),
            launch_arguments={
                'use_sim_time': 'True',
                'DRONE_NAME': LaunchConfiguration('DRONE_NAME')
            }.items()
        )
    ])