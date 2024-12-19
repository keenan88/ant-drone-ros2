import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    drone_description_launch_path = os.path.join(get_package_share_directory('antdrone_bringup'), 'launch', 'description.launch.py')

    gz_spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        output='screen',
        arguments=[
            '-topic', 'robot_description', 
            '-entity', LaunchConfiguration('DRONE_NAME'), 
            '-x', LaunchConfiguration('x0'),
            '-y', LaunchConfiguration('y0'),
            '-z', LaunchConfiguration('z0'),
            '-Y', LaunchConfiguration('yaw0'),

        ]
    )     

    drone_gz_ros_domain_bridge = Node(
        package="antdrone_gz",
        executable="gz_ros_domain_bridge",
        arguments = [
            '--from', os.getenv('MAIN_ROS_DOMAIN_ID'), '--to', os.getenv('ROS_DOMAIN_ID'),
            '/home/humble_ws/src/antdrone_gz/config/drone_gz_ros_domain_bridge.yaml'
        ],
        parameters = [
            {
                'DRONE_NAME': LaunchConfiguration('DRONE_NAME'),
                'from': int(os.getenv('MAIN_ROS_DOMAIN_ID')),
                'to': int(os.getenv('ROS_DOMAIN_ID'))
            }
        ]
    )

    gz_frame_name_fixer = Node(
        package="antdrone_gz",
        executable="gz_frame_name_fixer",
        parameters = [
            {
                'use_sim_time': True,
                'DRONE_NAME': LaunchConfiguration('DRONE_NAME')
            }
        ]
    )

    drone_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(drone_description_launch_path),
        launch_arguments={
            'use_sim_time': 'True',
            'DRONE_NAME': LaunchConfiguration('DRONE_NAME')
        }.items()
    )
    
    return LaunchDescription([        
        gz_spawn,
        drone_gz_ros_domain_bridge,
        gz_frame_name_fixer,
        drone_description
    ])