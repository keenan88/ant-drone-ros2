from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os, yaml
from yaml.loader import SafeLoader

def generate_launch_description():

    world_name = os.environ.get('world_name')
    drone_name = os.getenv('DRONE_NAME')

    odometry_launch_path = '/home/humble_ws/src/antdrone_bringup/launch/localization.launch.py'

    rviz_config_path = '/home/humble_ws/src/antdrone_navigation/rviz/' + drone_name + '.rviz'
    
    controller = Node(
        package = 'nav2_controller',
        executable = 'controller_server',
        parameters = [
            {'use_sim_time': LaunchConfiguration("USE_SIM_TIME")}, 
            '/home/humble_ws/src/antdrone_navigation/config/controller.yaml'
        ]
    )


    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        parameters=[
            '/home/humble_ws/src/antdrone_navigation/config/planner.yaml',
            {'use_sim_time': LaunchConfiguration("USE_SIM_TIME")}, 
        ]
    )

    behaviors = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        parameters=[
            {'use_sim_time': LaunchConfiguration("USE_SIM_TIME")}, 
            '/home/humble_ws/src/antdrone_navigation/config/behaviors.yaml',
            
        ],
    )

    bt = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        respawn_delay=2.0,
        parameters=[
            {'use_sim_time': LaunchConfiguration("USE_SIM_TIME")}, 
            '/home/humble_ws/src/antdrone_navigation/config/bt.yaml'
        ],
    )

    map_server = Node(
        package = 'nav2_map_server',
        executable = 'map_server',
        parameters = [
            '/home/humble_ws/src/antdrone_navigation/config/map_server.yaml',
            {
                'use_sim_time': LaunchConfiguration("USE_SIM_TIME"),
                'yaml_filename': os.path.join(get_package_share_directory('antdrone_navigation'), 'maps', world_name+'.yaml')
            }, 
        ]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        parameters=[
            {'use_sim_time': LaunchConfiguration("USE_SIM_TIME")}, 
            '/home/humble_ws/src/antdrone_navigation/config/lifecycle_manager.yaml',
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
            'DRONE_NAME': LaunchConfiguration("DRONE_NAME"),
            'USE_SIM_TIME': LaunchConfiguration("USE_SIM_TIME")
        }.items()
    )

    start_poses_path = os.path.join(get_package_share_directory('antdrone_navigation'), 'config', 'start_poses.yaml')
    with open(start_poses_path, 'r') as f:
        start_poses = yaml.load(f, Loader=yaml.SafeLoader)
    
    if world_name not in start_poses.keys():
        raise Exception("Drones' Nav2 start poses in world: [" + world_name + "] must be specified in " + start_poses_path)
    if drone_name not in start_poses[world_name].keys():
        raise Exception("Drone [" + drone_name + "] Nav2 start pos in world: [" + world_name + "] must be specified in " + start_poses_path)
        
    start_pos = start_poses[world_name][drone_name]

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        parameters=[
            '/home/humble_ws/src/antdrone_navigation/config/amcl.yaml',
            {
                'use_sim_time': LaunchConfiguration("USE_SIM_TIME"),
                "set_initial_pose": LaunchConfiguration("USE_SIM_TIME"),
                "initial_pose": {
                    'x': start_pos["x0"], 
                    'y': start_pos["y0"], 
                    'z': start_pos["z0"], 
                    'yaw': start_pos["yaw0"]
                }
            }
        ]
    )

    keepout_filter_mask_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        parameters=[
            '/home/humble_ws/src/antdrone_navigation/config/keepout_filter.yaml',
            {
                'use_sim_time': LaunchConfiguration("USE_SIM_TIME"),
                'yaml_filename': os.path.join(get_package_share_directory('antdrone_navigation'), 'maps', world_name+'_keepout.yaml')
            }
        ]
    )

    keepout_filter_map_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        parameters=[
            '/home/humble_ws/src/antdrone_navigation/config/keepout_filter.yaml',
            {
                'use_sim_time': LaunchConfiguration("USE_SIM_TIME")
            }
        ]
    )

    amcl_pose_debugger = Node(
        package='antdrone_navigation',
        executable='amcl_pose_debugger'
    )

    ld = LaunchDescription()
    ld.add_action(amcl_pose_debugger)

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

    return ld


