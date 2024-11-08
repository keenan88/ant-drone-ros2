from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():

    drone_name = os.getenv('DRONE_NAME')

    odometry_launch_path = '/home/humble_ws/src/linorobot2_navigation/launch/odometry.launch.py'

    wheels_or_body_odometry = DeclareLaunchArgument(
        'WHEEL_ODOMETRY',
        default_value='body',
        description='Whether to determine odometry from mecanum wheels or body (body given by IsaacSim)'
    )

    rviz_config_path = '/home/humble_ws/src/linorobot2_navigation/rviz/linorobot2_slam.rviz'
    
    controller = Node(
        package = 'nav2_controller',
        executable = 'controller_server',
        namespace = drone_name,
        parameters = [
            {'use_sim_time': True}, 
            {'robot_base_frame': drone_name + '_base_link'},
            {'local_costmap.local_costmap.keepout_filter_info': '/' + drone_name + '/keepout_filter_info'},
            '/home/humble_ws/src/linorobot2_navigation/config/controller.yaml'
        ],
        remappings=[
                ('plan', 'plan_with_orientations'),
                ('keepout_filter_info', '/' + drone_name + '/keepout_filter_info'),
                ('scan', '/' + drone_name + '/scan'),
        ]
    )

    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        namespace = drone_name,
        parameters=[
            {'use_sim_time': True},
            {'robot_base_frame': drone_name + '_base_link'},
            '/home/humble_ws/src/linorobot2_navigation/config/planner.yaml'
        ],
        remappings=[
            ('keepout_filter_info', '/' + drone_name + '/keepout_filter_info'),
            ('scan', '/' + drone_name + '/scan'),
        ]
    )

    behaviors = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        namespace = drone_name,
        parameters=[
            {'use_sim_time': True}, 
            '/home/humble_ws/src/linorobot2_navigation/config/behaviors.yaml'
        ],
    )

    bt = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        namespace = drone_name,
        respawn_delay=2.0,
        parameters=[
            {'use_sim_time': True},
            {'robot_base_frame': drone_name + '_base_link'},
            '/home/humble_ws/src/linorobot2_navigation/config/bt.yaml'
        ],
    )

    map_server = Node(
        package = 'nav2_map_server',
        executable = 'map_server',
        namespace = drone_name,
        parameters = [
            '/home/humble_ws/src/linorobot2_navigation/config/map_server.yaml',
            {'use_sim_time' : True}
        ]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        namespace = drone_name,
        parameters=[
            {'use_sim_time': True},
            '/home/humble_ws/src/linorobot2_navigation/config/lifecycle_manager.yaml',
        ]
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

    odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(odometry_launch_path),
        launch_arguments={
            'WHEEL_ODOMETRY': LaunchConfiguration('WHEEL_ODOMETRY'),
            'DRONE_NAME': drone_name
        }.items()
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        namespace = drone_name,
        parameters=[
            '/home/humble_ws/src/linorobot2_navigation/config/amcl.yaml',
            {
            'use_sim_time': True,
            'base_frame_id': drone_name + '_base_link',
            'odom_frame_id': drone_name + '_odom'
            }
        ]
    )

    amcl_pointcloud = Node(
        package='linorobot2_localization',
        executable='amcl_visualizer',
        namespace = drone_name,
        parameters=[
            {'use_sim_time': True}
        ]
    )

    keepout_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            '/home/humble_ws/src/linorobot2_navigation/launch/keepout.launch.py'
        ),
        launch_arguments={
            'DRONE_NAME': drone_name
        }.items()
    )

    path_orientation_updater = Node(
        package='linorobot2_localization',
        executable='path_orientation_updater',
        namespace = drone_name,
        parameters=[
            {'use_sim_time': True}
        ]
    )

    ld = LaunchDescription()

    ld.add_action(controller)
    ld.add_action(planner)
    ld.add_action(behaviors)
    ld.add_action(bt)
    ld.add_action(map_server)
    ld.add_action(lifecycle_manager)
    ld.add_action(rviz)
    ld.add_action(odometry)
    ld.add_action(amcl)
    ld.add_action(amcl_pointcloud)
    ld.add_action(keepout_launch)
    ld.add_action(path_orientation_updater)
    ld.add_action(wheels_or_body_odometry)



    return ld


