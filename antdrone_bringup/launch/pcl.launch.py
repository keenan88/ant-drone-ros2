import launch
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    ld = launch.LaunchDescription()

    for realsense_placement in ["front_rs", "rear_rs", "left_rs", "right_rs"]:

        gz_pcl_frame_fixer = Node(
            package='antdrone_pcl',
            executable='gz_pcl_frame_fixer',
            name = realsense_placement + "_gz_pcl_frame_fixer",
            output='screen',
            parameters=[
                {'camera_pos': realsense_placement},
                {'use_sim_time' : LaunchConfiguration("USE_SIM_TIME")},
            ],
            condition = IfCondition(LaunchConfiguration("USE_SIM_TIME"))
        )

        input_topic = realsense_placement + '/' + realsense_placement + '/depth/color/points', 
        crop_topic = realsense_placement + '/pointcloud_cropped'
        
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


        ld.add_action(pointcloud_cropper)
        ld.add_action(gz_pcl_frame_fixer)

    
    
    return ld