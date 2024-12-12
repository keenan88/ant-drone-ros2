import launch
from launch_ros.actions import Node
import os


def generate_launch_description():

    drone_name = os.environ.get("DRONE_NAME")

    print(drone_name)

    use_sim = os.environ.get("USE_SIM_TIME")
    use_sim = use_sim == "True"

    ld = launch.LaunchDescription()

    for realsense_placement in ["front_rs", "rear_rs", "left_rs", "right_rs"]:

        pcl_frame_fixer = Node(
            package='linorobot2_pcl',
            executable='pcl_frame_fixer',
            name = realsense_placement + "_pcl_frame_fixer",
            namespace = drone_name,
            output='screen',
            parameters=[
                {'camera_pos': realsense_placement},
                {'use_sim_time' : use_sim},
                {'drone_name': drone_name}
            ]
        )

        input_topic = realsense_placement + '/frame_fixed/points'
        crop_topic = realsense_placement + '/pointcloud_cropped'
        downsample_topic = realsense_placement + '/pointcloud_downsampled'

        
        pointcloud_cropper = Node(
            package='pcl_ros',
            executable='filter_crop_box_node',
            name = realsense_placement + "_pointcloud_cropper",
            namespace = drone_name,
            output='screen',
            parameters=[
                '/home/humble_ws/src/linorobot2_pcl/config/pcl_filter_crop_box.yaml',
                {
                    'use_sim_time' : use_sim,
                    "input_frame": drone_name + '_' + realsense_placement + '_depth_optical_frame',
                    "output_frame": drone_name + '_' + realsense_placement + '_depth_optical_frame'
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
            namespace = drone_name,
            output='screen',
            parameters=[
                '/home/humble_ws/src/linorobot2_pcl/config/pcl_downsample_filter.yaml',
                {'use_sim_time' : use_sim}
            ],
            remappings=[
                ('input', crop_topic),
                ('output', downsample_topic)
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
 
    # if not use_sim:
    #     ld.add_action(bridge_out_pcl)
    
    
    return ld