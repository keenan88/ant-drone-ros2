from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        namespace='nav2',
        parameters=[
            {'use_sim_time': LaunchConfiguration("USE_SIM_TIME")}, 
            '/home/humble_ws/src/antdrone_navigation/config/controller.yaml'
        ],
    )

    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        namespace='nav2',
        parameters=[
            {'use_sim_time': LaunchConfiguration("USE_SIM_TIME")},
            '/home/humble_ws/src/antdrone_navigation/config/planner.yaml'
        ],
    )

    behaviors = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        namespace='nav2',
        parameters=[
            {'use_sim_time': LaunchConfiguration("USE_SIM_TIME")}, 
            '/home/humble_ws/src/antdrone_navigation/config/behaviors.yaml'
        ],
    )

    bt = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        namespace='nav2',
        respawn_delay=2.0,
        parameters=[
            {'use_sim_time': LaunchConfiguration("USE_SIM_TIME")},
            '/home/humble_ws/src/antdrone_navigation/config/bt.yaml'
        ],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        namespace='nav2',
        parameters=[
            {'use_sim_time': LaunchConfiguration("USE_SIM_TIME")},
            '/home/humble_ws/src/antdrone_navigation/config/slam_lifecycle_manager.yaml',
        ]
    )


    ld = LaunchDescription()

    ld.add_action(controller)
    ld.add_action(planner)
    ld.add_action(behaviors)
    ld.add_action(bt)
    ld.add_action(lifecycle_manager)



    return ld


