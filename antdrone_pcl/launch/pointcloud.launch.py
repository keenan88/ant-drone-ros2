import launch
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    ld = launch.LaunchDescription()

    for realsense_placement in ["front_rs", "rear_rs", "left_rs", "right_rs"]:

        pcl_frame_fixer = Node(
            package='antdrone_pcl',
            executable='pcl_frame_fixer',
            name = realsense_placement + "_pcl_frame_fixer",
            output='screen',
            parameters=[
                {'camera_pos': realsense_placement},
                {'use_sim_time' : LaunchConfiguration("USE_SIM_TIME")},
            ]
        )

        input_topic = realsense_placement + '/frame_fixed/points'
        crop_topic = realsense_placement + '/pointcloud_cropped'
        downsample_topic = realsense_placement + '/pointcloud_downsampled'

        
        pointcloud_cropper = Node(
            package='pcl_ros',
            executable='filter_crop_box_node',
            name = realsense_placement + "_pointcloud_cropper",
            output='screen',
            parameters=[
                '/home/humble_ws/src/antdrone_pcl/config/pcl_filter_crop_box.yaml',
                {
                    'use_sim_time' : LaunchConfiguration("USE_SIM_TIME"),
                    "input_frame": realsense_placement + '_depth_optical_frame',
                    "output_frame": realsense_placement + '_depth_optical_frame'
                },
            ],
            remappings=[
                ('input', input_topic), 
                ('output', crop_topic)
            ]
        )

        pointcloud_downsampler = Node(
            package='pcl_ros',
            executable='filter_voxel_grid_node',
            name = realsense_placement + "_pointcloud_downsampler",
            output='screen',
            parameters=[
                '/home/humble_ws/src/antdrone_pcl/config/pcl_downsample_filter.yaml',
                {'use_sim_time' : LaunchConfiguration("USE_SIM_TIME")}
            ],
            remappings=[
                ('input', crop_topic),
                ('output', downsample_topic)
            ]
        )

        ld.add_action(pointcloud_cropper)
        ld.add_action(pointcloud_downsampler)
        ld.add_action(pcl_frame_fixer)


    bridge_out_pcl = Node(
        package="domain_bridge",
        executable="domain_bridge",
        name = "bridge_out_pcl",
        arguments = ['/home/humble_ws/src/antdrone_pcl/config/bridge_out_pcl.yaml'],
        condition = UnlessCondition(LaunchConfiguration("USE_SIM_TIME"))
    )
 
    ld.add_action(bridge_out_pcl)
    
    
    return ld