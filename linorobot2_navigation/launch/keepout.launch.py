from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    drone_name = os.getenv('DRONE_NAME')

    ld = LaunchDescription()


    keepout_filter_mask_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        namespace = drone_name,
        parameters=[
            '/home/humble_ws/src/linorobot2_navigation/config/filter_mask_server.yaml',
            {
                'use_sim_time': True,
                'topic_name': '/' + drone_name + '/keepout_filter_mask',
            }
        ]
    )

    keepout_filter_map_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        namespace = drone_name,
        parameters=[
            '/home/humble_ws/src/linorobot2_navigation/config/filter_mask_server.yaml',
            {
                'use_sim_time': True,
                'filter_info_topic': '/' + drone_name +'/keepout_filter_info',
                'mask_topic': '/' + drone_name + '/keepout_filter_mask'

            }
        ]
    )

    keepout_filter_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap_filters',
        namespace = drone_name,
        parameters=[
            {
                'use_sim_time': True,
            },
            '/home/humble_ws/src/linorobot2_navigation/config/lifecycle_manager_costmap_filters.yaml'
        ]
    )


    ld.add_action(keepout_filter_lifecycle_manager)
    ld.add_action(keepout_filter_map_server)
    ld.add_action(keepout_filter_mask_server)

    return ld


