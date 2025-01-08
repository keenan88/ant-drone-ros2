# Taken from example at: https://github.com/IntelRealSense/realsense-ros/tree/ros2-master/realsense2_camera/examples/dual_camera 
# command line example: ros2 launch antworker_bringup realsense.launch.py serial_no1:="'112322077317'" serial_no2:="'207122078912'"

import copy, sys, pathlib, os
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration

sys.path.append(str(pathlib.Path(__file__).parent.absolute()))
from ament_index_python.packages import get_package_share_directory

sys.path.append(os.path.join(get_package_share_directory('realsense2_camera'), 'launch'))
import rs_launch

local_parameters = [
    {'name': 'camera_name1',            'default': 'front_rs', 'description': 'camera unique name'},
    {'name': 'camera_name2',            'default': 'rear_rs', 'description': 'camera unique name'},
    {'name': 'camera_namespace1',       'default': 'front_rs', 'description': 'camera1 namespace'},
    {'name': 'camera_namespace2',       'default': 'rear_rs', 'description': 'camera2 namespace'},
    {'name': 'enable_color1',           'default': 'true', 'description': 'enable color stream'},
    {'name': 'enable_color2',           'default': 'true', 'description': 'enable color stream'},
    {'name': 'enable_depth1',           'default': 'true', 'description': 'enable depth stream'},
    {'name': 'enable_depth2',           'default': 'true', 'description': 'enable depth stream'},
    {'name': 'pointcloud.enable1',      'default': 'true', 'description': 'enable pointcloud'},
    {'name': 'pointcloud.enable2',      'default': 'true', 'description': 'enable pointcloud'},
    {'name': 'spatial_filter.enable1',  'default': 'true', 'description': 'enable_spatial_filter'},
    {'name': 'spatial_filter.enable2',  'default': 'true', 'description': 'enable_spatial_filter'},
    {'name': 'temporal_filter.enable1', 'default': 'true', 'description': 'enable_temporal_filter'},
    {'name': 'temporal_filter.enable2', 'default': 'true', 'description': 'enable_temporal_filter'},

    {'name': 'depth_module.depth_profile1', 'default': '480x270x15', 'description': 'depth camera resolution and framerate'},
    {'name': 'depth_module.depth_profile2', 'default': '480x270x15', 'description': 'depth camera resolution and framerate'},

    {'name': 'rgb_camera.color_profile1', 'default': '424x240x15', 'description': 'color camera resolution and framerate'},
    {'name': 'rgb_camera.color_profile2', 'default': '424x240x15', 'description': 'color camera resolution and framerate'}
]

def set_configurable_parameters(local_params):
    return dict([(param['original_name'], LaunchConfiguration(param['name'])) for param in local_params])

def duplicate_params(general_params, posix):
    local_params = copy.deepcopy(general_params)
    for param in local_params:
        param['original_name'] = param['name']
        param['name'] += posix
    return local_params
    
def generate_launch_description():
    params1 = duplicate_params(rs_launch.configurable_parameters, '1')
    params2 = duplicate_params(rs_launch.configurable_parameters, '2')
    
    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters) +
        rs_launch.declare_configurable_parameters(params1) + 
        rs_launch.declare_configurable_parameters(params2) + 
        [
            OpaqueFunction(
                function=rs_launch.launch_setup,
                kwargs = {
                    'params': set_configurable_parameters(params1),
                    'param_name_suffix': '1'
                }
            ),

            OpaqueFunction(
                function=rs_launch.launch_setup,
                kwargs = {
                    'params': set_configurable_parameters(params2),
                    'param_name_suffix': '2'
                }
            ),
        ]
    )