from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    control_config = os.path.join(
        get_package_share_directory('linorobot2_jack'),
        'config',
        'sliding_joint_control.yaml'
    )

    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[control_config]
        ),
        
        # Spawn the joint_state_broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        ),
        
        # Spawn the sliding_joint_controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['sliding_joint_controller', '--controller-manager', '/controller_manager'],
        )
    ])
