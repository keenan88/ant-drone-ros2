import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable




def generate_launch_description():

    

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('linorobot2_bringup'),
                    'launch',
                    'start_greenhouse.launch.py'
                )
            )
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(
        #             get_package_share_directory('linorobot2_bringup'), 
        #             'launch',
        #             'spawn_drone.launch.py'
        #         )
        #     )
        # )

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('antworker_description'), 
                    'launch',
                    'spawn_worker.launch.py'
                )
            ),
            launch_arguments={
                'namespace': 'worker0',
                'spawn_x': '0',
                'spawn_y': '0',
                'spawn_z': '0',
                'spawn_yaw': '0'
            }.items()
        )
    ])