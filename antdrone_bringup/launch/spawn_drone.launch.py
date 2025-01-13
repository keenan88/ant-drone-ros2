import os, yaml
from yaml.loader import SafeLoader
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition


def generate_launch_description():
    
    drone_description_launch_path = os.path.join(get_package_share_directory('antdrone_bringup'), 'launch', 'description.launch.py')

    world_name = os.environ.get('world_name')
    drone_name = os.environ.get('DRONE_NAME')

    spawn_poses_path = os.path.join(get_package_share_directory('antdrone_gz'), 'config', 'spawn_poses.yaml')
    with open(spawn_poses_path, 'r') as f:
        spawn_poses = yaml.load(f, Loader=yaml.SafeLoader)
    
    if world_name not in spawn_poses.keys():
        raise Exception("Drones' spawn poses for Gazebo world: [" + world_name + "] must be specified in " + spawn_poses_path)
    if drone_name not in spawn_poses[world_name].keys():
        raise Exception("Drone [" + drone_name + "] spawn pos for Gazebo world: [" + world_name + "] must be specified in " + spawn_poses_path)

    spawn_pos = spawn_poses[world_name][drone_name]

    gz_spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        output='screen',
        arguments=[
            '-topic', 'robot_description', 
            '-entity', LaunchConfiguration('DRONE_NAME'), 
            '-x', str(spawn_pos['x0']),
            '-y', str(spawn_pos['y0']),
            '-z', str(spawn_pos['z0']),
            '-Y', str(spawn_pos['yaw0']),

        ]
    )     

    drone_gz_ros_domain_bridge = Node(
        package="antdrone_gz",
        executable="gz_ros_domain_bridge",
        arguments = [
            '--from', os.getenv('MAIN_ROS_DOMAIN_ID'), '--to', os.getenv('ROS_DOMAIN_ID'),
            '/home/humble_ws/src/antdrone_gz/config/drone_gz_ros_domain_bridge.yaml'
        ],
        parameters = [
            {
                'DRONE_NAME': LaunchConfiguration('DRONE_NAME'),
                'from': int(os.getenv('MAIN_ROS_DOMAIN_ID')),
                'to': int(os.getenv('ROS_DOMAIN_ID'))
            }
        ]
    )

    gz_frame_name_fixer = Node(
        package="antdrone_gz",
        executable="gz_frame_name_fixer",
        parameters = [
            {
                'use_sim_time':True,
                'DRONE_NAME': LaunchConfiguration('DRONE_NAME')
            }
        ]
    )

    gz_odom_frame_fixer = Node(
        package='antdrone_localization',
        executable='gz_odom_frame_fixer', 
        parameters=[
            {
                'use_sim_time' : True,
            }
        ]
    )

    drone_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(drone_description_launch_path),
        launch_arguments={
            'use_sim_time': 'True',
            'DRONE_NAME': LaunchConfiguration('DRONE_NAME')
        }.items()
    )
    
    return LaunchDescription([        
        gz_spawn,
        drone_gz_ros_domain_bridge,
        gz_frame_name_fixer,
        drone_description,
        gz_odom_frame_fixer
    ])