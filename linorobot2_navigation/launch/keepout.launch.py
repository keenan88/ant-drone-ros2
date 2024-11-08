from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    rviz_config_path = '/home/humble_ws/src/linorobot2_navigation/rviz/linorobot2_slam.rviz'

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        namespace = LaunchConfiguration('DRONE_NAME'),
        parameters=[
            {'use_sim_time': True}
        ]
    )

    ld = LaunchDescription()


    keepout_filter_mask_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        namespace = LaunchConfiguration('DRONE_NAME'),
        parameters=[
            '/home/humble_ws/src/linorobot2_navigation/config/filter_mask_server.yaml',
            {
                'use_sim_time': True,
            }
        ]
    )

    keepout_filter_map_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        namespace = LaunchConfiguration('DRONE_NAME'),
        parameters=[
            '/home/humble_ws/src/linorobot2_navigation/config/filter_mask_server.yaml',
            {
                'use_sim_time': True,
            }
        ]
    )

    keepout_filter_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap_filters',
        namespace = LaunchConfiguration('DRONE_NAME'),
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


