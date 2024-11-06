import launch
from launch_ros.actions import Node
import os

# Robot has a realsense on front and back, each with own namespacing and frame names
def generate_launch_description():

    use_sim = os.environ.get("USE_SIM")
    use_sim = use_sim == "True"

    ld = launch.LaunchDescription()

    for realsense_placement in ["front", "rear", "left", "right"]:

        pcl_frame_fixer = Node(
            package='linorobot2_pcl',
            executable='pcl_frame_fixer',
            name = realsense_placement + "_pcl_frame_fixer",
            output='screen',
            parameters=[
                {'camera_pos': realsense_placement}
            ]
        )

        pointcloud_cropper = Node(
            package='pcl_ros',
            executable='filter_crop_box_node',
            name = realsense_placement + "_rs_pointcloud_cropper",
            output='screen',
            parameters=[
                '/home/humble_ws/src/linorobot2_pcl/config/' + realsense_placement + '_rs_filter_crop_box.yaml',
                {
                    'use_sim_time' : use_sim,
                    "input_frame": realsense_placement + '_rs_depth_optical_frame',
                    "output_frame": 'base_link'
                },
            ],
            remappings=[
                ('input', realsense_placement + '_camera/frame_fixed/points'), 
                ('output', realsense_placement + '_rs/pointcloud_cropped')
            ]
        )

        pointcloud_downsampler = Node(
            package='pcl_ros',
            executable='filter_voxel_grid_node',
            name = realsense_placement + "_rs_pointcloud_downsampler",
            output='screen',
            parameters=[
                '/home/humble_ws/src/linorobot2_pcl/config/' + realsense_placement + '_rs_pcl_downsample_filter.yaml',
                {'use_sim_time' : use_sim}
            ],
            remappings=[
                ('input', realsense_placement + '_rs/pointcloud_cropped'),
                ('output', realsense_placement + '_rs/pointcloud_downsampled')
            ]
        )

        ld.add_action(pointcloud_cropper)
        ld.add_action(pointcloud_downsampler)
        ld.add_action(pcl_frame_fixer)


    # bridge_out_pcl = Node(
    #     package="domain_bridge",
    #     executable="domain_bridge",
    #     name = "bridge_out_pcl",
    #     arguments = ['/home/humble_ws/src/linorobot2_pcl/config/bridge_out_pcl.yaml']
    # )

    # # ld.add_action(rviz_node)
    # if not use_sim:
    #     ld.add_action(bridge_out_pcl)
    
    
    return ld