from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():

    drone_name = os.getenv('DRONE_NAME')

    slam_config_path = '/home/humble_ws/src/linorobot2_navigation/config/slam.yaml'
    odometry_launch_path = '/home/humble_ws/src/linorobot2_navigation/launch/odometry.launch.py'
    rviz_config_path = '/home/humble_ws/src/linorobot2_navigation/rviz/linorobot2_slam.rviz'

    wheels_or_body_odometry = DeclareLaunchArgument(
        'WHEEL_ODOMETRY',
        default_value='body',
        description='Whether to determine odometry from mecanum wheels or body (body given by IsaacSim)'
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace = drone_name,
        parameters=[
            slam_config_path,
            {'use_sim_time': True},
            {'base_frame': drone_name + '_base_link'},
            {'odom_frame': drone_name + '_odom'},
            {'map_frame': drone_name + '_map'},
        ],
        remappings = [
            ('/map', '/' + drone_name + "/map"),
            ('/map_metadata', '/' + drone_name + "/map_metadata")
        ]

    )

    image_recorder = Node(
        package='linorobot2_localization',
        executable='slam_image_recorder',
        namespace = drone_name,
        parameters=[
            {'use_sim_time': True}
        ]
    )

    odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(odometry_launch_path),
        launch_arguments={
            'WHEEL_ODOMETRY': LaunchConfiguration('WHEEL_ODOMETRY')  # Pass the argument to child
        }.items()
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        namespace = drone_name,
        parameters=[
            {'use_sim_time': True}
        ]
    )

    ld = LaunchDescription()

    ld.add_action(slam_toolbox)
    # ld.add_action(image_recorder)
    ld.add_action(odometry)
    ld.add_action(wheels_or_body_odometry)
    
    ld.add_action(rviz)

    return ld
