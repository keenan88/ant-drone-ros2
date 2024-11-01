from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration



def generate_launch_description():

    slam_config_path = '/home/humble_ws/src/linorobot2_navigation/config/slam.yaml'
    navigation_launch_path = '/home/humble_ws/src/linorobot2_navigation/launch/slam_navigation.launch.py'
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
        namespace = 'nav2',
        parameters=[
            slam_config_path,
            {'use_sim_time': True}
        ],
        remappings=[("/map", "/nav2/map")],
    )

    image_recorder = Node(
        package='linorobot2_localization',
        executable='slam_image_recorder',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_path),
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
        parameters=[
            {'use_sim_time': True}
        ]
    )

    ld = LaunchDescription()

    ld.add_action(slam_toolbox)
    # ld.add_action(navigation)
    ld.add_action(image_recorder)
    ld.add_action(odometry)
    ld.add_action(wheels_or_body_odometry)
    
    ld.add_action(rviz)

    return ld
