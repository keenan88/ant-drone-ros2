import launch
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Realsenses freeze up if FPS or resolution too high. Depends on USB ports used.
    color_profile = '424x240x15' #'640x480x15'
    depth_profile = '480x270x15' #'640x480x15'
    infra_profile = '480x270x15' #'640x480x15'

    
    

    front_rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'serial_no': "'207522073816'",
            'camera_name':'front_rs',
            'camera_namespace':'front_rs',
            'pointcloud.enable':'True',
            'rgb_camera.color_profile' : color_profile,
            'depth_module.depth_profile' : depth_profile,
            'depth_module.infra_profile': infra_profile,
            # 'enable_accel': 'True',
            # 'enable_gyro': 'True'
        }.items()
    )

    rear_rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'serial_no': "'207122078912'",
            'camera_name':'rear_rs',
            'camera_namespace':'rear_rs',
            'pointcloud.enable':'True',
            'rgb_camera.color_profile' : color_profile,
            'depth_module.depth_profile' : depth_profile,
            'depth_module.infra_profile': infra_profile,
        }.items()
    )

    left_rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'serial_no': "'109122070837'",
            'camera_name':'left_rs',
            'camera_namespace':'left_rs',
            'pointcloud.enable':'True',
            'rgb_camera.color_profile' : color_profile,
            'depth_module.depth_profile' : depth_profile,
            'depth_module.infra_profile': infra_profile,
            # 'enable_accel': 'True',
            # 'enable_gyro': 'True',
            # 'unite_imu_method': '2'
        }.items()
    )

    right_rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'serial_no': "'207522073046'",
            'camera_name':'right_rs',
            'camera_namespace':'right_rs',
            'pointcloud.enable':'True',
            'rgb_camera.color_profile' : color_profile,
            'depth_module.depth_profile' : depth_profile,
            'depth_module.infra_profile': infra_profile,
        }.items()
    )



    bridge_out_tf = Node(
        package="domain_bridge",
        executable="domain_bridge",
        name = "bridge_out_tf",
        arguments = ['/home/humble_ws/src/antworker_realsense/config/bridge_out_tf.yaml']
    )

    front_rs_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_front_rs",
        output="log",
        arguments = [
            "0.407", "0.0", "0.0",
            "0.0", "0.0", "0.0",
            "base_link",
            "front_rs_link",
        ],
    )

    left_rs_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_left_rs",
        output="log",
        arguments = [
            "0.0", "0.247", "0.0",
            "1.57", "0.0", "0.0",
            "base_link",
            "left_rs_link",
        ],
    )

    rear_rs_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_rear_rs",
        output="log",
        arguments = [
            "-0.407", "0.0", "0.0",
            "3.14", "0.0", "0.0",
            "base_link",
            "rear_rs_link",
        ]
    )

    right_rs_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_right_rs",
        output="log",
        arguments = [
            "0.0", "-0.247", "0.0",
            "-1.57", "0.0", "0.0",
            "base_link",
            "right_rs_link",
        ],
    )

    

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', '/home/humble_ws/src/antdrone_realsense/config/rs.rviz'],
        parameters=[
            {'use_sim_time': False}, 
        ]
    )
    
    ld = launch.LaunchDescription()

    ld.add_action(front_rs_launch)
    ld.add_action(left_rs_launch)
    ld.add_action(rear_rs_launch)
    ld.add_action(right_rs_launch)

    # ld.add_action(rviz)
    ld.add_action(bridge_out_tf)
    ld.add_action(front_rs_tf)
    ld.add_action(left_rs_tf)
    ld.add_action(rear_rs_tf)
    ld.add_action(right_rs_tf)
    
    
    return ld



    
