from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    odometry_launch_path = '/home/humble_ws/src/linorobot2_localization/launch/odometry.launch.py'

    rviz_config_path = '/home/humble_ws/src/linorobot2_navigation/rviz/linorobot2_slam_' + "drone_boris" + '.rviz'
    
    controller = Node(
        package = 'nav2_controller',
        executable = 'controller_server',
        parameters = [
            {'use_sim_time': LaunchConfiguration("USE_SIM_TIME")}, 
            '/home/humble_ws/src/linorobot2_navigation/config/controller.yaml'
        ]
    )

    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        parameters=[
            '/home/humble_ws/src/linorobot2_navigation/config/planner.yaml',
            {'use_sim_time': LaunchConfiguration("USE_SIM_TIME")}, 
        ]
    )

    behaviors = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        parameters=[
            {'use_sim_time': LaunchConfiguration("USE_SIM_TIME")}, 
            '/home/humble_ws/src/linorobot2_navigation/config/behaviors.yaml',
            
        ],
    )

    bt = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        respawn_delay=2.0,
        parameters=[
            {'use_sim_time': LaunchConfiguration("USE_SIM_TIME")}, 
            '/home/humble_ws/src/linorobot2_navigation/config/bt.yaml'
        ],
    )

    map_server = Node(
        package = 'nav2_map_server',
        executable = 'map_server',
        parameters = [
            '/home/humble_ws/src/linorobot2_navigation/config/map_server.yaml',
            {'use_sim_time': LaunchConfiguration("USE_SIM_TIME")}, 
        ]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        parameters=[
            {'use_sim_time': LaunchConfiguration("USE_SIM_TIME")}, 
            '/home/humble_ws/src/linorobot2_navigation/config/lifecycle_manager.yaml',
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[
            {'use_sim_time': LaunchConfiguration("USE_SIM_TIME")}, 
        ]
    )

    odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(odometry_launch_path),
        launch_arguments={
            'WHEEL_ODOMETRY': LaunchConfiguration('WHEEL_ODOMETRY'),
            'DRONE_NAME': LaunchConfiguration("DRONE_NAME"),
            'USE_SIM_TIME': LaunchConfiguration("USE_SIM_TIME")
        }.items()
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        parameters=[
            '/home/humble_ws/src/linorobot2_navigation/config/amcl.yaml',
            {
                'use_sim_time': LaunchConfiguration("USE_SIM_TIME")
            }
        ]
    )

    amcl_pointcloud = Node(
        package='linorobot2_localization',
        executable='amcl_visualizer',
        parameters=[
            {'use_sim_time': LaunchConfiguration("USE_SIM_TIME")}, 
        ]
    )

    keepout_filter_mask_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        parameters=[
            '/home/humble_ws/src/linorobot2_navigation/config/filter_mask_server.yaml',
            {
                'use_sim_time': LaunchConfiguration("USE_SIM_TIME")
            }
        ]
    )

    keepout_filter_map_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        parameters=[
            '/home/humble_ws/src/linorobot2_navigation/config/filter_mask_server.yaml',
            {
                'use_sim_time': LaunchConfiguration("USE_SIM_TIME")
            }
        ]
    )

    cmd_vel_scaler = Node(
        package = 'linorobot2_localization',
        executable = 'cmd_vel_scale_gz',
        parameters = [
            {
                'use_sim_time': LaunchConfiguration("USE_SIM_TIME")
            }
            
        ]
    )

    ld = LaunchDescription()

    ld.add_action(keepout_filter_map_server)
    ld.add_action(keepout_filter_mask_server)

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
    ld.add_action(cmd_vel_scaler)

    return ld


