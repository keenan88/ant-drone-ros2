import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory




def generate_launch_description():
    use_sim_time = True
    
    description_launch_path = os.path.join(get_package_share_directory('linorobot2_description'), 'launch', 'description.launch.py')

    print(description_launch_path)

    return LaunchDescription([
        DeclareLaunchArgument(
            name='world', 
            default_value='/home/humble_ws/src/simulation/gazebo/playground.world',
            description='Gazebo world'
        ),

        DeclareLaunchArgument(
            name='spawn_x', 
            default_value='0.0',
            description='Robot spawn position in X axis'
        ),

        DeclareLaunchArgument(
            name='spawn_y', 
            default_value='0.0',
            description='Robot spawn position in Y axis'
        ),

        DeclareLaunchArgument(
            name='spawn_z', 
            default_value='0.0',
            description='Robot spawn position in Z axis'
        ),
            
        DeclareLaunchArgument(
            name='spawn_yaw', 
            default_value='0.0',
            description='Robot spawn heading'
        ),

        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so',  '-s', 'libgazebo_ros_init.so', LaunchConfiguration('world')],
            output='screen'
        ),

        

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=[
                '-topic', 'robot_description', 
                '-entity', 'linorobot2', 
                '-x', LaunchConfiguration('spawn_x'),
                '-y', LaunchConfiguration('spawn_y'),
                '-z', LaunchConfiguration('spawn_z'),
                '-Y', LaunchConfiguration('spawn_yaw'),
            ]
        ),       

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            launch_arguments={
                'use_sim_time': str(use_sim_time),
            }.items()
        )
    ])