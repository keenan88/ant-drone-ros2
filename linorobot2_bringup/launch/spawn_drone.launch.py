import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml


def load_yaml(package_name, yaml_file):
    package_path = get_package_share_directory(package_name)
    yaml_path = os.path.join(package_path, 'config', yaml_file)
    with open(yaml_path, 'r') as file:
        return yaml.safe_load(file)

def generate_launch_description():
    use_sim_time = True
    
    drone_description_launch_path = os.path.join(get_package_share_directory('linorobot2_description'), 'launch', 'description.launch.py')

    spawn_params = load_yaml(
        'linorobot2_bringup',
        'drone0_spawn.yaml'
    )['spawn_positions']


    return LaunchDescription([        

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=[
                '-topic', 'robot_description', 
                '-entity', 'linorobot2', 
                '-x', str(spawn_params['spawn_x']),
                '-y', str(spawn_params['spawn_y']),
                '-z', str(spawn_params['spawn_z']),
                '-Y', str(spawn_params['spawn_yaw']),

            ]
        ),       

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(drone_description_launch_path),
            launch_arguments={
                'use_sim_time': str(use_sim_time),
            }.items()
        )
    ])