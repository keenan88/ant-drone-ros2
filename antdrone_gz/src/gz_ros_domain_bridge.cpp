// Copyright 2021, Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

#include "domain_bridge/component_manager.hpp"
#include "domain_bridge/domain_bridge.hpp"
#include "domain_bridge/parse_domain_bridge_yaml_config.hpp"
#include "domain_bridge/process_cmd_line_arguments.hpp"

#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "linkattacher_msgs/srv/attach_link.hpp"
#include "linkattacher_msgs/srv/detach_link.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

int main(int argc, char **argv) {
  auto arguments = rclcpp::init_and_remove_ros_arguments(argc, argv);

  auto config_rc_pair = domain_bridge::process_cmd_line_arguments(arguments);
  if (!config_rc_pair.first || 0 != config_rc_pair.second) {
    return config_rc_pair.second;
  }
  domain_bridge::DomainBridge domain_bridge(*config_rc_pair.first);

  // Add component manager node and domain bridge to single-threaded executor
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto node = std::make_shared<domain_bridge::ComponentManager>(executor);

  node->declare_parameter("DRONE_NAME", "");
  node->declare_parameter("from", 0);
  node->declare_parameter("to", 0);

  uint8_t main_ros_domain_id = node->get_parameter("from").as_int();
  uint8_t drone_ros_domain_id = node->get_parameter("to").as_int();
  std::string drone_name = node->get_parameter("DRONE_NAME").as_string();

  RCLCPP_INFO(node->get_logger(), "Bridging topics & services from gazebo -> %s, ros domain %d -> %d", drone_name.c_str(), main_ros_domain_id, drone_ros_domain_id);

  domain_bridge.bridge_service<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK", main_ros_domain_id, drone_ros_domain_id);
  domain_bridge.bridge_service<linkattacher_msgs::srv::DetachLink>("/DETACHLINK", main_ros_domain_id, drone_ros_domain_id);
  domain_bridge.bridge_service<gazebo_msgs::srv::SpawnEntity>("/spawn_entity", main_ros_domain_id, drone_ros_domain_id);

  // Even though drone is spawned in a container with a ROS DOMAIN ID unique to the drone,
  // the drone exists in gazebo, which is in the MAIN ROS DOMAIN ID, so its topics need to be bridged
  // into the DRONE ROS DOMAIN ID.
  domain_bridge::TopicBridgeOptions odom_bridge_options;
  domain_bridge::TopicBridgeOptions tf_bridge_options;

  domain_bridge::TopicBridgeOptions front_pcl_bridge_options;
  domain_bridge::TopicBridgeOptions left_pcl_bridge_options;
  domain_bridge::TopicBridgeOptions rear_pcl_bridge_options;
  domain_bridge::TopicBridgeOptions right_pcl_bridge_options;

  domain_bridge::TopicBridgeOptions apriltag_cam_colour_img_bridge_options;
  // domain_bridge::TopicBridgeOptions left_colour_img_bridge_options;
  // domain_bridge::TopicBridgeOptions rear_colour_img_bridge_options;
  // domain_bridge::TopicBridgeOptions right_colour_img_bridge_options;

  domain_bridge::TopicBridgeOptions apriltag_cam_info_bridge_options;
  // domain_bridge::TopicBridgeOptions left_camera_info_bridge_options;
  // domain_bridge::TopicBridgeOptions rear_camera_info_bridge_options;
  // domain_bridge::TopicBridgeOptions right_camera_info_bridge_options;

  domain_bridge::TopicBridgeOptions cmd_vel_bridge_options;

  // Easier to identify namespaced topics in cpp rather than yaml config file
  odom_bridge_options.remap_name("/odom_wrong_frame");
  tf_bridge_options.remap_name("tf_gz");

  // Pointclouds remap options to match realsense topic names
  front_pcl_bridge_options.remap_name("/front_rs/front_rs/depth/color/points_wrong_frame");
  left_pcl_bridge_options.remap_name("/left_rs/left_rs/depth/color/points_wrong_frame");
  rear_pcl_bridge_options.remap_name("/rear_rs/rear_rs/depth/color/points_wrong_frame");
  right_pcl_bridge_options.remap_name("/right_rs/right_rs/depth/color/points_wrong_frame");

  // Camera info remap options to match realsense topic names
  apriltag_cam_info_bridge_options.remap_name("/apriltag_cam/apriltag_cam/color/camera_info_wrong_frame");
  // left_camera_info_bridge_options.remap_name("/left_rs/left_rs/color/camera_info_wrong_frame");
  // rear_camera_info_bridge_options.remap_name("/rear_rs/rear_rs/color/camera_info_wrong_frame");
  // right_camera_info_bridge_options.remap_name("/right_rs/right_rs/color/camera_info_wrong_frame");

  // Raw colour images remap options to match realsense topic names
  apriltag_cam_colour_img_bridge_options.remap_name("/apriltag_cam/apriltag_cam/color/image_raw_wrong_frame");
  // left_colour_img_bridge_options.remap_name("/left_rs/left_rs/color/image_raw_wrong_frame");
  // rear_colour_img_bridge_options.remap_name("/rear_rs/rear_rs/color/image_raw_wrong_frame");
  // right_colour_img_bridge_options.remap_name("/right_rs/right_rs/color/image_raw_wrong_frame");

  cmd_vel_bridge_options.remap_name("/" + drone_name + "/cmd_vel");

  domain_bridge.bridge_topic("/" + drone_name + "/odom", "nav_msgs/msg/Odometry", main_ros_domain_id, drone_ros_domain_id, odom_bridge_options);
  domain_bridge.bridge_topic("/tf", "tf2_msgs/msg/TFMessage", main_ros_domain_id, drone_ros_domain_id, tf_bridge_options);
  
  // Pointclouds
  domain_bridge.bridge_topic("/" + drone_name + "/front_rs/points", "sensor_msgs/msg/PointCloud2", main_ros_domain_id, drone_ros_domain_id, front_pcl_bridge_options);
  domain_bridge.bridge_topic("/" + drone_name + "/left_rs/points", "sensor_msgs/msg/PointCloud2", main_ros_domain_id, drone_ros_domain_id, left_pcl_bridge_options);
  domain_bridge.bridge_topic("/" + drone_name + "/rear_rs/points", "sensor_msgs/msg/PointCloud2", main_ros_domain_id, drone_ros_domain_id, rear_pcl_bridge_options);
  domain_bridge.bridge_topic("/" + drone_name + "/right_rs/points", "sensor_msgs/msg/PointCloud2", main_ros_domain_id, drone_ros_domain_id, right_pcl_bridge_options);
  
  // Camera infos
  domain_bridge.bridge_topic("/" + drone_name + "/apriltag_cam/camera_info", "sensor_msgs/msg/CameraInfo", main_ros_domain_id, drone_ros_domain_id, apriltag_cam_info_bridge_options);
  // domain_bridge.bridge_topic("/" + drone_name + "/left_rs/camera_info", "sensor_msgs/msg/CameraInfo", main_ros_domain_id, drone_ros_domain_id, left_camera_info_bridge_options);
  // domain_bridge.bridge_topic("/" + drone_name + "/rear_rs/camera_info", "sensor_msgs/msg/CameraInfo", main_ros_domain_id, drone_ros_domain_id, rear_camera_info_bridge_options);
  // domain_bridge.bridge_topic("/" + drone_name + "/right_rs/camera_info", "sensor_msgs/msg/CameraInfo", main_ros_domain_id, drone_ros_domain_id, right_camera_info_bridge_options);

  // Raw colour images
  domain_bridge.bridge_topic("/" + drone_name + "/apriltag_cam/image_raw", "sensor_msgs/msg/Image", main_ros_domain_id, drone_ros_domain_id, apriltag_cam_colour_img_bridge_options);
  // domain_bridge.bridge_topic("/" + drone_name + "/left_rs/image_raw", "sensor_msgs/msg/Image", main_ros_domain_id, drone_ros_domain_id, left_colour_img_bridge_options);
  // domain_bridge.bridge_topic("/" + drone_name + "/rear_rs/image_raw", "sensor_msgs/msg/Image", main_ros_domain_id, drone_ros_domain_id, rear_colour_img_bridge_options);
  // domain_bridge.bridge_topic("/" + drone_name + "/right_rs/image_raw", "sensor_msgs/msg/Image", main_ros_domain_id, drone_ros_domain_id, right_colour_img_bridge_options);
  
  
  domain_bridge.bridge_topic("/cmd_vel", "geometry_msgs/msg/Twist", drone_ros_domain_id, main_ros_domain_id, cmd_vel_bridge_options);

  domain_bridge.add_to_executor(*executor);
  executor->add_node(node);
  executor->spin();

  rclcpp::shutdown();
  return 0;
}