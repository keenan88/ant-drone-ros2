from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():

    slam_config_path = '/home/humble_ws/src/antdrone_slam/config/slam.yaml'
    odometry_launch_path = '/home/humble_ws/src/antdrone_bringup/launch/localization.launch.py'
    rviz_config_path = '/home/humble_ws/src/antdrone_slam/rviz/' + os.getenv("DRONE_NAME") + '_slam.rviz'

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_config_path,
            {'use_sim_time': LaunchConfiguration("USE_SIM_TIME")}
        ]
    )

    odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(odometry_launch_path),
        launch_arguments={
            'DRONE_NAME': LaunchConfiguration('DRONE_NAME'),
            'USE_SIM_TIME': LaunchConfiguration("USE_SIM_TIME")
        }.items()
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[
            {'use_sim_time': LaunchConfiguration("USE_SIM_TIME")}
        ]
    )

    ld = LaunchDescription()

    ld.add_action(slam_toolbox)
    ld.add_action(odometry)
    
    # ld.add_action(rviz)

    return ld
