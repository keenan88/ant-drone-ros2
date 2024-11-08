import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory




def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            name='world', 
            default_value='/home/simulation/gazebo/playground.world',
            description='Gazebo world'
        ),

        ExecuteProcess(
            cmd=['gazebo', '-s', 'libgazebo_ros_factory.so',  '-s', 'libgazebo_ros_init.so', LaunchConfiguration('world')],
            output='screen'
        )
    ])